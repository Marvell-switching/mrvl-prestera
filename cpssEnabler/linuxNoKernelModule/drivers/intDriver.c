/*******************************************************************************
Copyright (C) Marvell International Ltd. and its affiliates

This software file (the "File") is owned and distributed by Marvell
International Ltd. and/or its affiliates ("Marvell") under the following
alternative licensing terms.  Once you have made an election to distribute the
File under one of the following license alternatives, please (i) delete this
introductory statement regarding license alternatives, (ii) delete the two
license alternatives that you have not elected to use and (iii) preserve the
Marvell copyright notice above.

********************************************************************************
Marvell GPL License Option

If you received this File from Marvell, you may opt to use, redistribute and/or
modify this File in accordance with the terms and conditions of the General
Public License Version 2, June 1991 (the "GPL License"), a copy of which is
available along with the File in the license.txt file or by writing to the Free
Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 or
on the worldwide web at http://www.gnu.org/licenses/gpl.txt.

THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE IMPLIED
WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY
DISCLAIMED.  The GPL License provides additional details about this warranty
disclaimer.
********************************************************************************
* intDriver.c
*
* DESCRIPTION:
*       mvIntDrv - A simple driver which passes interrupts to user-space
*                  Usage:
*                     fd=open("/dev/mvIntDrv",O_RDWR);
*                     write(fd, "eI", 2)    will enable irq I (0..255)
*                     write(fd, "dI", 2)    will disable irq I (0..255)
*                     write(fd, "EIIII", 5) will enable irq I (0..0xffffffff)
*                     write(fd, "DIIII", 5) will disable irq I (0..0xffffffff)
*                     write(fd, "mBDF",4)   will enable MSI interrupts
*                                           Here B=PCI bus (binary)
*                                           Here D=PCI device (binary)
*                                           Here F=PCI device functin (binary)
*                     write(fd, "MddBDF",6) will enable MSI interrupts
*                                           Here dd=PCI domain, LE
*                                           Here B=PCI bus (binary)
*                                           Here D=PCI device (binary)
*                                           Here F=PCI device functin (binary)
*                     X=write(fd, "cI", 2) will connect irq I (0..255)
*                     X=write(fd, "CIIII", 5) will connect irq I (0..0xffffffff)
*
*                     read(fd,NULL,X) will wait for irq
*
* DEPENDENCIES:
*
*       $Revision: 1 $
*******************************************************************************/
#define MV_DRV_NAME     "mvIntDrv"
#define INT_DRV_VER		"1.02"
#define MV_DRV_MAJOR    244
#define MV_DRV_MINOR    4
#define MV_DRV_FOPS     mvIntDrv_fops
#define MV_DRV_PREINIT  mvIntDrv_PreInitDrv
#define MV_DRV_POSTINIT mvIntDrv_postInitDrv
#define MV_DRV_RELEASE  mvIntDrv_releaseDrv
#include "mvDriverTemplate.h"

#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>

static int mvIntDrvNumOpened = 0;
static struct semaphore	*mvIntDrvInterrupsSema; /* Alert other modules on interrupt */
static struct semaphore mvint_pci_devs_sem;

struct interrupt_slot {
	int			used;
	unsigned int		irq;
	struct semaphore	sem; /* The semaphore on which the user waits */
	struct tasklet_struct	tasklet;
};

#define MAX_INTERRUPTS 32
static struct interrupt_slot mvIntDrv_slots[MAX_INTERRUPTS];

#define MAX_PCI_DEVS 8

struct pci_dev *pci_devs_list[MAX_PCI_DEVS];

int mvintdrv_add_pci_dev_to_ar(struct pci_dev *dev)
{
	int i;

	down(&mvint_pci_devs_sem);
	for (i=0; i<MAX_PCI_DEVS; i++) {
				if (!pci_devs_list[i]) {
						pci_devs_list[i] = dev;
						up(&mvint_pci_devs_sem);
						return 0;
					}
		}

	up(&mvint_pci_devs_sem);
	return -1;
}

/*
	Get and remove the head (first non-NULL item) of the array
*/
struct pci_dev *mvintdrv_get_pci_dev_from_ar(void)
{
	int i;
	struct pci_dev *dev;
	
	down(&mvint_pci_devs_sem);
	for (i=0; i<MAX_PCI_DEVS; i++) {
				if (pci_devs_list[i]) {
						dev = pci_devs_list[i];
						pci_devs_list[i] = NULL;
						up(&mvint_pci_devs_sem);
						return dev;
					}
		}

	up(&mvint_pci_devs_sem);
	return NULL;
}


int mvintdrv_register_isr_sema(struct semaphore *sema)
{
	mvIntDrvInterrupsSema = sema;

	return 0;
}
EXPORT_SYMBOL(mvintdrv_register_isr_sema);

void mvintdrv_unregister_isr_sema(struct semaphore *sema)
{
	mvIntDrvInterrupsSema = NULL;
}
EXPORT_SYMBOL(mvintdrv_unregister_isr_sema);

static irqreturn_t prestera_tl_ISR(int irq, void *tl)
{
	/* Disable the interrupt vector */
	disable_irq_nosync(irq);
	/* Enqueue the PP task BH in the tasklet */
	tasklet_hi_schedule((struct tasklet_struct *)tl);

	return IRQ_HANDLED;
}

void mvPresteraBh(unsigned long data)
{
	/* Awake any reading process */
	if (mvIntDrvInterrupsSema)
		up(mvIntDrvInterrupsSema);
	up(&((struct interrupt_slot *)data)->sem);
}

/**
 *	alloc_interrupt_slot - Allocate interupt slot
 *	@irq: Interrupt number
 *
 *	Allocates and initialize interupt slot
 *
 *	Returns the index of the entry in interrupts array
 */
static unsigned int alloc_interrupt_slot(unsigned int irq)
{
	struct interrupt_slot *sl;
	int slot;

	for (slot = 0; slot < MAX_INTERRUPTS; slot++)
		if (!mvIntDrv_slots[slot].used) {
			sl = &(mvIntDrv_slots[slot]);
			sl->used = 1;
			sl->irq = irq;
			sema_init(&sl->sem, 0);
			tasklet_init(&sl->tasklet, mvPresteraBh,
				     (unsigned long)sl);
			if (request_irq(irq, prestera_tl_ISR, IRQF_SHARED,
					"mvIntDrv", (void *)&sl->tasklet))
				panic("Can not assign IRQ %u to mvIntDrv\n",
				      irq);
			disable_irq(irq);
			return slot;
		}

	return MAX_INTERRUPTS;
}

/**
 *	free_interrupt_slot - Free interupt slot
 *	@slot: Index of an entry in interrupt array to free
 *
 *	Undoes all the steps of slot allocation
 *
 *	Note: This function assumes sl->irq is in "disable" state
 */
static void free_interrupt_slot(int slot)
{
	struct interrupt_slot *sl = &(mvIntDrv_slots[slot]);

	disable_irq(sl->irq);
	enable_irq(sl->irq);
	free_irq(sl->irq, (void*)&(sl->tasklet));
	tasklet_kill(&(sl->tasklet));
	sl->used = 0;
}

static int intConnect(unsigned int irq)
{
	int slot;

	slot = alloc_interrupt_slot(irq);
	if (unlikely(slot == MAX_INTERRUPTS)) {
		printk(KERN_ERR "%s: no free slots\n", __func__);
		return -EFAULT;
	}

	printk(KERN_DEBUG "%s: connected IRQ - %u slot %d\n", __func__, irq,
	       slot);

	return slot + 1;
}

static int intDisConnect(unsigned int irq)
{
	struct interrupt_slot *sl;
	int slot;

	for (slot = 0; slot < MAX_INTERRUPTS; slot++) {
		sl = &(mvIntDrv_slots[slot]);
		if (sl->irq == irq) {
			/* free_interrupt_slot assumes that IRQ state is disabled so userspace must
			 * ensure this before triggering disconnect */
			free_interrupt_slot(slot);
			printk(KERN_DEBUG "%s: disconnected IRQ - %u slot %d\n",
			       __func__, irq, slot);
			return slot;
		}
	}

	return 0;
}

static ssize_t mvIntDrv_write(struct file *f, const char *buf, size_t siz, loff_t *off)
{
	unsigned int irq = -1;
	char cmdBuf[6];

	/* Write 2 bytes:
	 * 'c' intNo       - connect interrupt, returns slot+1
	 * 'd' intNo       - disable interrupt
	 * 'e' intNo       - enable interrupt
	 * 'q' intNo       - query interrupt, whether other drivers are still
	 *                   attached to it
	 * 'C' i i i i     - connect interrupt, returns slot+1
	 * 'R' i i i i     - remove interrupt, returns slot+1
	 * 'D' i i i i     - disable interrupt
	 * 'E' i i i i     - enable interrupt
	 * 'Q' i i i i     - query interrupt, whether other drivers are still
	 *                   attached to it
	 * 'm' bus dev sel - enable MSI interrupts for pci device
	 * 
	 * return <1 - error, slot for connect, 0 for enable/disable, 0/1 for Q
	 * (query) */

	if (copy_from_user(cmdBuf, buf, ((siz < 6) ? siz : 6))) {
		printk(KERN_ERR "%s: EFAULT\n", __func__);
		return -EFAULT;
	}

	switch (cmdBuf[0]) {
	case 'c':
		/* Fall through */
	case 'd':
		/* Fall through */
	case 'e':
		/* Fall through */
	case 'r':
		/* Fall through */
	case 'q':
		irq = (unsigned int)cmdBuf[1];
		break;
	case 'C':
		/* Fall through */
	case 'D':
		/* Fall through */
	case 'E':
		/* Fall through */
	case 'R':
		/* Fall through */
	case 'Q':
		memcpy(&irq, cmdBuf + 1, 4);
		break;
	}

	if (cmdBuf[0] == 'c' || cmdBuf[0] == 'C')
		return intConnect(irq);

	if (cmdBuf[0] == 'r' || cmdBuf[0] == 'R')
		return intDisConnect(irq);

	if (cmdBuf[0] == 'd' || cmdBuf[0] == 'D') {
		disable_irq(irq);
		return 0;
	}

	if (cmdBuf[0] == 'e' || cmdBuf[0] == 'E') {
		enable_irq(irq);
		return 0;
	}

	if (cmdBuf[0] == 'm') {
#ifdef CONFIG_PCI_MSI
	struct pci_dev *pdev;
	pdev = pci_get_domain_bus_and_slot(0, (unsigned)cmdBuf[1],
					   PCI_DEVFN((unsigned)cmdBuf[2],
					   (unsigned)cmdBuf[3]));
	if (pdev) {
		int rc;

		pr_info("%s: Got request to enable MSI for %x:%x:%x\n",
				__func__, (((cmdBuf[2]<<8)&0xff00)|(cmdBuf[1]&0xff)),
					   (unsigned)cmdBuf[3],
					   PCI_DEVFN((unsigned)cmdBuf[4],
						     (unsigned)cmdBuf[5]));

		if (mvintdrv_add_pci_dev_to_ar(pdev)) {
			printk("%s: Cannot reg pdev %p!\n", __func__, pdev);
			} else pr_info("%s: Queueing pci device for driver cleanup MSI disablement...\n", __func__);

		if (pci_dev_msi_enabled(pdev)) {
			pr_info("%s: PCIe MSI already enabled.\n", __func__);
			return 0;
		}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,9,0)
		rc = pci_alloc_irq_vectors(pdev, 1, 1, PCI_IRQ_ALL_TYPES);
#else
		rc = pci_enable_msi(pdev);
#endif
		printk("MSI interrupts for device %s %senabled\n",
		       pdev->dev.kobj.name, (rc < 0) ? "not " : "");

		return rc;
	}
#else
	return -1;
#endif
	}

	if (cmdBuf[0] == 'M') {
#ifdef CONFIG_PCI_MSI
	struct pci_dev *pdev;
	pdev = pci_get_domain_bus_and_slot((((cmdBuf[2]<<8)&0xff00)|(cmdBuf[1]&0xff)),
					   (unsigned)cmdBuf[3],
					   PCI_DEVFN((unsigned)cmdBuf[4],
						     (unsigned)cmdBuf[5]));
	if (pdev) {
		int rc;

		pr_info("%s: Got request to enable MSI for %x:%x:%x\n",
				__func__, (((cmdBuf[2]<<8)&0xff00)|(cmdBuf[1]&0xff)),
					   (unsigned)cmdBuf[3],
					   PCI_DEVFN((unsigned)cmdBuf[4],
						     (unsigned)cmdBuf[5]));

		if (mvintdrv_add_pci_dev_to_ar(pdev)) {
			printk("%s: Cannot reg pdev %p!\n", __func__, pdev);
			} else pr_info("%s: Queueing pci device for driver cleanup MSI disablement...\n", __func__);

		if (pci_dev_msi_enabled(pdev)) {
			pr_info("%s: PCIe MSI already enabled.\n", __func__);
			return 0;
		}
		rc = pci_enable_msi(pdev);
		printk("MSI interrupts for device %s %senabled\n", pdev->dev.kobj.name,
		       (rc < 0) ? "not " : "");

		return rc;
	}
#else
	return -1;
#endif
	}

	if (cmdBuf[0] == 'q' || cmdBuf[0] == 'Q') {
		struct irq_desc *desc = irq_to_desc(irq);

		/* irq desc->action points to the first 'client' the irq needs
		 * to serve, or NULL if none */
		if (desc && desc->action)
			return 1;

		return 0;
	}

	return -EINVAL;
}

static ssize_t mvIntDrv_read(struct file *f, char *buf, size_t siz, loff_t *off)
{
	struct interrupt_slot *sl;
	int slot = (int)siz - 1;

	if (slot < 0 || slot >= MAX_INTERRUPTS)
		return -EINVAL;

	sl = &(mvIntDrv_slots[slot]);
	if (!sl->used)
		return -EINVAL;

	/* Enable the interrupt vector */
	enable_irq(sl->irq);

	if (down_interruptible(&sl->sem)) {
		disable_irq(sl->irq);
		return -EINTR;
	}

	return 0;
}

static int mvIntDrv_open(struct inode *inode, struct file *file)
{
	mvIntDrvNumOpened++;
	file->private_data = NULL;

	return 0;
}

static int mvIntDrv_release(struct inode *inode, struct file *file)
{
	mvIntDrvNumOpened--;
	if (!mvIntDrvNumOpened) {
		/* Cleanup */
		int i, slot;
		struct interrupt_slot *sl;
		struct pci_dev *pdev;
		u16 control;
		
		for (i=0; i<MAX_PCI_DEVS; i++) {
				pdev = mvintdrv_get_pci_dev_from_ar();
				if (pdev) {
						printk("%s: Disabling MSI for devfn %x vendor %x devid %x msi_cap %x msi_enabled %d\n",
							__func__, pdev->devfn, pdev->vendor, pdev->device, pdev->msi_cap, pdev->msi_enabled);
/*
 * 						Need to disable MSI before releasing the interrupts, as currently
 * 												the Kernel will only write the MSI address with zero WITHOUT
 * 																		disabling MSI, causing memory overrun of physical address zero in ARM 32-bit:
 * 																		*/
						pci_read_config_word(pdev, pdev->msi_cap + PCI_MSI_FLAGS, &control);
						control &= ~PCI_MSI_FLAGS_ENABLE;
						pci_write_config_word(pdev, pdev->msi_cap + PCI_MSI_FLAGS, control);

						printk("%s: New MSI control reg value is: %x\n", __func__, control);
					}
			}

		for (slot = 0; slot < MAX_INTERRUPTS; slot++) {
			sl = &(mvIntDrv_slots[slot]);
			if (!sl->used)
				continue;
			/* free_interrupt_slot assumes that IRQ state is disabled and we are cool
			 * with that since in both cases, waiting for interrupt or servicing an
			 * interrupt - IRQ is disabled.
			 * In abnormal case where the process dies while waiting for interrupt,
			 * the down_interruptible will exits and disable_irq will be called. On
			 * the other hand, the ISR function make sure the IRQ is disabled while
			 * servicing an interrupt */
			free_interrupt_slot(slot);
		}
	}

	return 0;
}

static struct file_operations mvIntDrv_fops = {
	.read = mvIntDrv_read,
	.write = mvIntDrv_write,
	.open = mvIntDrv_open,
	.release = mvIntDrv_release /* A.K.A close */
};


static void mvIntDrv_releaseDrv(void)
{
	/* Will be called whan all descriptors are closed */
}

static int mvIntDrv_PreInitDrv(void)
{
	sema_init(&mvint_pci_devs_sem, 1);
	pr_info("%s: Version: %s\n", __func__, INT_DRV_VER);
	return 0;
}


static void mvIntDrv_postInitDrv(void)
{
	memset(mvIntDrv_slots, 0, sizeof(mvIntDrv_slots));
	printk(KERN_DEBUG "mvIntDrv major=%d minor=%d\n", major, minor);
}

