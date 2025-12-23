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
*******************************************************************************/
#define MV_DRV_NAME     "mvIntDrv"
#define INT_DRV_VER     "1.19"
#include "mvDriverTemplate.h"

#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/sched/signal.h>

void mvPresteraBh(unsigned long data);
void mvintdrv_exit(void);
int mvintdrv_init(void);

/* Character device context */
static struct mvchrdev_ctx *chrdrv_ctx;
static struct semaphore mvint_pci_devs_sem;

static int mvIntDrvNumOpened = 0;

enum irq_slot_state {
	IRQ_SLOT_STATE_UNALLOCATED,
	IRQ_SLOT_STATE_ALLOCATED,
	IRQ_SLOT_STATE_READY_FOR_REALLOCATION /* after user-space termination, ready for quick re-allocation */
};

struct interrupt_slot {
	atomic_t		depth; /* keep track of enable/disable */
	atomic_t		cnt_missed; /* count of interrupts detected by timeout */
	unsigned int		irq;
	struct semaphore	sem; /* The semaphore on which the user waits */
	struct semaphore	close_sem; /* Sync disconnect with read */
	struct tasklet_struct	tasklet;
	enum irq_slot_state	state;
};

/* To hold list of devices we enabled MSI on */
struct mvintdrv_pci_dev {
	struct list_head list;
	struct pci_dev *pdev;
};

struct mvintdrv_file_priv {
	struct list_head msi_enabled_pdevs;
};

#define MAX_INTERRUPTS 32
static struct interrupt_slot mvIntDrv_slots[MAX_INTERRUPTS];
static bool msi_used;

#define MAX_PCI_DEVS 8

struct pci_dev *pci_devs_list[MAX_PCI_DEVS];

/* module parameter which controls the no interrupt timeout
   for generating a fake interrupt: */
static int intdrv_timeout = 10000;
module_param(intdrv_timeout,int,0660);

/* Prototypes required to avoid warnings */
void enable_irq_wrapper(unsigned int irq);
void prt_msi_state(char *msg, unsigned bus, unsigned device, unsigned func);
int mvintdrv_add_pci_dev_to_ar(struct pci_dev *dev);
struct pci_dev *mvintdrv_get_pci_dev_from_ar(void);


void enable_irq_wrapper(unsigned int irq)
{
	struct irq_desc *desc = irq_data_to_desc(irq_get_irq_data(irq));

	if (!desc)
		return;

	if (WARN(!desc->irq_data.chip,
				KERN_ERR "enable_irq before setup/request_irq: irq %u\n", irq))
		goto out;

	if (!desc->depth) {
		pr_err("%s: irq desc depth is zero\n", __FUNCTION__);
#ifndef __aarch64__
		__sync_bool_compare_and_swap(&desc->depth, 0, 1);
#endif
	}

out:

	enable_irq(irq);
}

void prt_msi_state(char *msg, unsigned bus, unsigned device, unsigned func)
{
	struct pci_dev *pdev;
	u16 control, org_control;
	u32 msiaddr;

	pdev = pci_get_domain_bus_and_slot(0, bus,
			PCI_DEVFN(device,
				func));

	if (pdev) {
		pci_read_config_word(pdev, pdev->msi_cap + PCI_MSI_FLAGS, &control);
		pci_read_config_dword(pdev, pdev->msi_cap + PCI_MSI_ADDRESS_LO,
				&msiaddr);

		if ((control & PCI_MSI_FLAGS_ENABLE) && (!msiaddr)) {
			org_control = control;
			control &= ~PCI_MSI_FLAGS_ENABLE;
			pci_write_config_word(pdev, pdev->msi_cap + PCI_MSI_FLAGS, control);

			pr_err("%s: bdf %x:%x:%x msi ctrl %x msi addr low %x msi_enabled %d MISCONFIGURED will disable MSI\n",
					msg, bus, device, func, org_control, msiaddr,
					pdev->msi_enabled
				  );
		}
		pci_dev_put(pdev);
	}
}

bool get_pci_int_state(unsigned slot)
{
	struct pci_dev *pdev;
	u32 cmd_status_dword;

	if (slot >= MAX_PCI_DEVS)
		return false;

	pdev = pci_devs_list[slot];

	if (pdev) {
		pci_dev_get(pdev);
		pci_read_config_dword(pdev, PCI_COMMAND, &cmd_status_dword);
		pci_dev_put(pdev);
		if ((cmd_status_dword >> 16) & PCI_STATUS_INTERRUPT) {
			pr_err("%s: Slot %u pdev %lx val %x\n", __func__,
			       slot, (unsigned long)pdev, cmd_status_dword);
		}
		return !!((cmd_status_dword >> 16) & PCI_STATUS_INTERRUPT);
	} else {
		return false;
	}
}

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
 *   Get and remove the head (first non-NULL item) of the array
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

static int find_interrupt_slot(unsigned int irq, bool warn)
{
	struct interrupt_slot *sl;
	int slot;

	for (slot = 0; slot < MAX_INTERRUPTS; slot++) {
		sl = &(mvIntDrv_slots[slot]);
		if ( (sl->irq == irq) && (sl->state == IRQ_SLOT_STATE_ALLOCATED) )
			return slot;
	}

	if (warn)
		printk(KERN_WARNING "%s: No slot allocated for IRQ %d\n",
		       MV_DRV_NAME, irq);

	return -ENOENT;
}

static irqreturn_t prestera_tl_ISR(int irq, void *tl)
{
	int slot = find_interrupt_slot(irq, true);
	struct interrupt_slot *sl = &(mvIntDrv_slots[slot]);

	BUG_ON(!sl);

	/* handle last interrupt after process termination
	 * disregarding to (sl->state==IRQ_SLOT_STATE_ALLOCATED)
	 */
	atomic_dec(&sl->depth);
	/* Disable the interrupt vector */
	disable_irq_nosync(irq);
	/* Enqueue the PP task BH in the tasklet */
	tasklet_hi_schedule((struct tasklet_struct *)tl);

	return IRQ_HANDLED;
}

void mvPresteraBh(unsigned long data)
{
	/* Awake any reading process */
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
		if ( (mvIntDrv_slots[slot].state == IRQ_SLOT_STATE_READY_FOR_REALLOCATION) &&
		     (mvIntDrv_slots[slot].irq == irq) ) {
			/* Interrupt was already allocated to a killed process. Just reuse it without requesting
			   IRQ again. IRQ is already disabled so no need to disable it now: */
			sl = &(mvIntDrv_slots[slot]);
			sl->irq = irq;
			atomic_set(&sl->cnt_missed, 0);
			pr_info("Performing irq %d reallocation...\n", sl->irq);
			sema_init(&sl->sem, 0);
			sema_init(&sl->close_sem, 0);
			up(&sl->close_sem);
			tasklet_init(&sl->tasklet, mvPresteraBh,
				     (unsigned long)sl);
			atomic_set(&sl->depth, -1);
			sl->state = IRQ_SLOT_STATE_ALLOCATED;
			return slot;

		}

	for (slot = 0; slot < MAX_INTERRUPTS; slot++)
		if (mvIntDrv_slots[slot].state == IRQ_SLOT_STATE_UNALLOCATED) {
			sl = &(mvIntDrv_slots[slot]);
			sl->irq = irq;
			atomic_set(&sl->cnt_missed, 0);
			sema_init(&sl->sem, 0);
			sema_init(&sl->close_sem, 0);
			up(&sl->close_sem);
			tasklet_init(&sl->tasklet, mvPresteraBh,
				     (unsigned long)sl);
			prt_msi_state("alloc before req", 1, 0 ,0);
			prt_msi_state("alloc before req", 2, 0 ,0);
			if (request_irq(irq, prestera_tl_ISR, IRQF_SHARED,
					MV_DRV_NAME, (void *)&sl->tasklet))
				panic("Can not assign IRQ %u to mvIntDrv\n",
				      irq);
			atomic_set(&sl->depth, -1);
			prt_msi_state("alloc after req", 1, 0 ,0);
			prt_msi_state("alloc after req", 2, 0 ,0);
			disable_irq(irq);
			prt_msi_state("alloc after dis", 1, 0 ,0);
			prt_msi_state("alloc after dis", 2, 0 ,0);

			sl->state = IRQ_SLOT_STATE_ALLOCATED;
			return slot;
		}

	return MAX_INTERRUPTS;
}

static void synch_irq_state(struct interrupt_slot *sl, int target_value)
{
	while (atomic_read(&sl->depth) < (target_value < 0 ? target_value : 0)) {
		atomic_inc(&sl->depth);
		enable_irq(sl->irq);
	}

	while (atomic_read(&sl->depth) > (target_value > 0 ? target_value : 0)) {
		atomic_dec(&sl->depth);
		disable_irq(sl->irq);
	}
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

	down(&sl->close_sem);
	/* In inconsistent state (ex race between disable_irq in ISR and
	   disable_irq in release event) synch to stable state before freeing
	   the IRQ. IRQ must be disabled at the end. */
	prt_msi_state("free before synch", 1, 0 ,0);
	prt_msi_state("free before synch", 2, 0 ,0);
	synch_irq_state(sl, -1);
	up(&sl->close_sem);
	prt_msi_state("free after synch", 1, 0 ,0);
	prt_msi_state("free after synch", 2, 0 ,0);
	prt_msi_state("free before freeirq", 1, 0 ,0);
	prt_msi_state("free before freeirq", 2, 0 ,0);
	if (msi_used) { /* MSI wil BUG() the kernel unless free_irq() is called */
		free_irq(sl->irq, (void*)&(sl->tasklet));
		prt_msi_state("free after freeirq", 1, 0 ,0);
		prt_msi_state("free after freeirq", 2, 0 ,0);
		tasklet_kill(&(sl->tasklet));
		sl->state = IRQ_SLOT_STATE_UNALLOCATED;
		sl->irq = 0;
	} else {
		pr_info("Leaving irq %d ready for reallocation...\n", sl->irq);
		tasklet_kill(&(sl->tasklet));
		sl->state = IRQ_SLOT_STATE_READY_FOR_REALLOCATION;
	}
}

static int intConnect(unsigned int irq)
{
	int slot;

	slot = find_interrupt_slot(irq, false);
	if (slot != -ENOENT) {
		printk(KERN_ERR "%s: irq number %d already connected\n",
		       MV_DRV_NAME, irq);
		return -EINVAL;
	}

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
	int slot;

	slot = find_interrupt_slot(irq, true);
	if (slot == -ENOENT)
		return -EINVAL;

	/*
	 * free_interrupt_slot assumes that IRQ state is disabled so userspace
	 * must ensure this before triggering disconnect
	 */
	free_interrupt_slot(slot);
	printk(KERN_DEBUG "%s: disconnected IRQ - %u slot %d\n", __func__, irq,
	       slot);

	return slot;
}

static int mvintdrv_pdev_enable_msi(struct pci_dev *pdev)
{
	int rc;

	msi_used = true;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,8,0)
	dev_info(&pdev->dev, "Enabling MSI, legacy IRQ number is %d\n",
		 pci_irq_vector(pdev, 0));
#else
	dev_info(&pdev->dev, "Enabling MSI, legacy IRQ number is %d\n",
		 pdev->irq);
#endif
	prt_msi_state("write before msi enable", 1, 0 ,0);
	prt_msi_state("write before msi enable", 2, 0 ,0);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,9,0)
	rc = pci_alloc_irq_vectors(pdev, 1, 1, PCI_IRQ_ALL_TYPES);
	rc = rc >= 1 ? 0 : -1;
#else
	rc = pci_enable_msi(pdev);
#endif
	prt_msi_state("write after msi enable", 1, 0 ,0);
	prt_msi_state("write after msi enable", 2, 0 ,0);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,8,0)
	dev_info(&pdev->dev, "MSI IRQ number is %d\n", (rc < 0) ? -1 :
		 pci_irq_vector(pdev, 0));
#else
	dev_info(&pdev->dev, "MSI IRQ number is %d\n", (rc < 0) ? -1 :
		 pdev->irq);
#endif

	return rc;
}

static int mvintdrv_enable_msi(struct file *f, int domain, unsigned int bus,
			       unsigned int dev, unsigned int func)
{
#ifdef CONFIG_PCI_MSI
	struct mvintdrv_pci_dev *mv_pdev;
	struct pci_dev *pdev;
	u32 msiaddr = 0;

	msi_used = true;
	pdev = pci_get_domain_bus_and_slot(domain, bus, PCI_DEVFN(dev, func));
	if (pdev) {
		struct mvintdrv_file_priv *priv;
		int rc;

		priv = (struct mvintdrv_file_priv *)f->private_data;
		if (mvintdrv_add_pci_dev_to_ar(pdev)) {
			pr_err("%s: Cannot reg pdev %p\n", __func__, pdev);
		} else pr_debug("%s: Queueing pci device for driver cleanup MSI disablement...\n", __func__);

		prt_msi_state("write before msi chk enable", 1, 0 ,0);
		prt_msi_state("write before msi chk enable", 2, 0 ,0);
		if (pci_dev_msi_enabled(pdev)) {
			printk(KERN_DEBUG "%s: MSI already enabled\n",
			       MV_DRV_NAME);
			prt_msi_state("write after msi enable already", 1, 0 ,0);
			prt_msi_state("write after msi enable already", 2, 0 ,0);
			pci_read_config_dword(pdev, pdev->msi_cap + PCI_MSI_ADDRESS_LO,
					&msiaddr);
			pci_dev_put(pdev);
			return msiaddr ? 0 : -EINVAL;
		}

		rc = mvintdrv_pdev_enable_msi(pdev);
		if (!rc) {
			/*
			 * Add to global list so we can disable MSI on driver
			 * driver exit
			 */
			mv_pdev = kmalloc(sizeof(*mv_pdev), GFP_KERNEL);
			mv_pdev->pdev = pdev;
			list_add(&mv_pdev->list, &priv->msi_enabled_pdevs);
		} else {
			pci_dev_put(pdev);
		}

		return rc;
	} else {
		printk(KERN_ERR "%s: Fail to find PCI device\n", MV_DRV_NAME);
		return -EINVAL;
	}
#else
	printk(KERN_ERR "%s: MSI requested however CONFIG_PCI_MSI is not enabled in the kernel build\n",
	       MV_DRV_NAME);
	return -EIO;
#endif
}

/**
 * Call the proper function to disable MSI according to kernel version in use
 */
static void mvintdrv_pdev_disable_msi(struct pci_dev *pdev)
{
	dev_info(&pdev->dev, "Disabling MSI\n");
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,9,0)
	pci_free_irq_vectors(pdev);
#else
	pci_disable_msi(pdev);
#endif
}

/**
 * Disable MSI for all devices of a given file descriptor, called when file
 * descriptor is closed
 */
static void mvintdrv_disable_msi_devs(struct file *f)
{
#ifdef CONFIG_PCI_MSI
	struct mvintdrv_file_priv *priv =
		(struct mvintdrv_file_priv *)f->private_data;
	struct list_head *p, *q;

	list_for_each_safe(p, q, &(priv->msi_enabled_pdevs)) {
		struct mvintdrv_pci_dev *mv_pdev;
		mv_pdev = list_entry(p, struct mvintdrv_pci_dev, list);
		mvintdrv_pdev_disable_msi(mv_pdev->pdev);
		pci_dev_put(mv_pdev->pdev);
		list_del(p);
		kfree(mv_pdev);
	}
#endif
}

/**
 * Disable MSI for device identified by DBDF
 */
static int mvintdrv_disable_msi(int domain, unsigned int bus, unsigned int dev,
				unsigned int func)
{
#ifdef CONFIG_PCI_MSI
	struct pci_dev *pdev;

	pdev = pci_get_domain_bus_and_slot(domain, bus, PCI_DEVFN(dev, func));
	if (pdev) {
		mvintdrv_pdev_disable_msi(pdev);
		pci_dev_put(pdev);
		return 0;
	} else {
		printk(KERN_ERR "%s: Fail to find PCI device %d:%d:%d.%d\n",
		       MV_DRV_NAME, domain, bus, dev, func);
		return -EINVAL;
	}
#else
	printk(KERN_ERR "%s: MSI requested however CONFIG_PCI_MSI is not enabled in the kernel build\n",
	       MV_DRV_NAME);
	return -EIO;
#endif
}

static ssize_t mvIntDrv_write(struct file *f, const char *buf, size_t siz, loff_t *off)
{
	struct interrupt_slot *sl;
	unsigned int irq = -1;
	char cmdBuf[6] = {0};
	int slot;

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
	 * 'm' bus, dev func - enable MSI interrupts for pci device
	 * 'M' domain, bus, dev func - enable MSI interrupts for pci device
	 * 's' bus, dev func - disable MSI interrupts for pci device
	 * 'S' domain, bus, dev func - disable MSI interrupts for pci device
	 *
	 * return <!0 - error, slot for connect, 0 for enable/disable, 0/1 for Q
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

	printk(KERN_DEBUG "%s: %c(%d, %d, %d)\n", MV_DRV_NAME, cmdBuf[0],
	       cmdBuf[1], cmdBuf[2], cmdBuf[3]);

	if (cmdBuf[0] == 'c' || cmdBuf[0] == 'C')
		return intConnect(irq);

	if (cmdBuf[0] == 'r' || cmdBuf[0] == 'R')
		return intDisConnect(irq);

	if (cmdBuf[0] == 'd' || cmdBuf[0] == 'D') {
		slot = find_interrupt_slot(irq, true);
		if (slot == -ENOENT)
			return -EINVAL;
		sl = &(mvIntDrv_slots[slot]);
		atomic_dec(&sl->depth);
		prt_msi_state("write before disable", 1, 0 ,0);
		prt_msi_state("write before disable", 2, 0 ,0);
		disable_irq(irq);
		prt_msi_state("write after disable", 1, 0 ,0);
		prt_msi_state("write after disable", 2, 0 ,0);
		return 0;
	}

	if (cmdBuf[0] == 'e' || cmdBuf[0] == 'E') {
		slot = find_interrupt_slot(irq, true);
		if (slot == -ENOENT)
			return -EINVAL;
		sl = &(mvIntDrv_slots[slot]);
		atomic_inc(&sl->depth);
		prt_msi_state("write before enable", 1, 0 ,0);
		prt_msi_state("write before enable", 2, 0 ,0);
		enable_irq_wrapper(irq);
		//enable_irq(irq);
		prt_msi_state("write after enable", 1, 0 ,0);
		prt_msi_state("write after enable", 2, 0 ,0);
		return 0;
	}

	if (cmdBuf[0] == 'm')
		return mvintdrv_enable_msi(f, 0, (unsigned)cmdBuf[1],
					   (unsigned)cmdBuf[2],
					   (unsigned)cmdBuf[3]);

	if (cmdBuf[0] == 'M')
		return mvintdrv_enable_msi(f, (((cmdBuf[2]<<8)&0xff00) |
					   (cmdBuf[1]&0xff)),
					   (unsigned)cmdBuf[3],
					   (unsigned)cmdBuf[4],
					   (unsigned)cmdBuf[5]);

	if (cmdBuf[0] == 's')
		return mvintdrv_disable_msi(0, (unsigned)cmdBuf[1],
					    (unsigned)cmdBuf[2],
					    (unsigned)cmdBuf[3]);

	if (cmdBuf[0] == 'S')
		return mvintdrv_disable_msi((((cmdBuf[2]<<8)&0xff00) |
					   (cmdBuf[1]&0xff)),
					   (unsigned)cmdBuf[3],
					   (unsigned)cmdBuf[4],
					   (unsigned)cmdBuf[5]);

	if (cmdBuf[0] == 'q' || cmdBuf[0] == 'Q')
		return find_interrupt_slot(irq, false) != -ENOENT;

	printk(KERN_ERR "%s: Invalid command '%c'\n", MV_DRV_NAME, cmdBuf[0]);

	return -EINVAL;
}

static ssize_t mvIntDrv_read(struct file *f, char *buf, size_t siz, loff_t *off)
{
	struct interrupt_slot *sl;
	int ret, slot = (int)siz - 1;

	if (slot < 0 || slot >= MAX_INTERRUPTS)
		return -EINVAL;

	sl = &(mvIntDrv_slots[slot]);
	if (sl->state != IRQ_SLOT_STATE_ALLOCATED)
		return -EINVAL;

	prt_msi_state("read before irq enable", 1, 0 ,0);
	prt_msi_state("read before irq enable", 2, 0 ,0);

	/* Enable the interrupt vector */
	atomic_inc(&sl->depth);
	enable_irq_wrapper(sl->irq);
	//enable_irq(sl->irq);

	prt_msi_state("read after irq enable", 1, 0 ,0);
	prt_msi_state("read after irq enable", 2, 0 ,0);

	/* wait for bottom half to wake us, with timeout: */
	while (atomic_read(&sl->cnt_missed) <= 1) {
		ret = down_timeout(&sl->sem, msecs_to_jiffies(intdrv_timeout));

		if (ret) { /* wait for semaphore - timeout expired / signal */
			if (get_pci_int_state(slot)) {
				atomic_inc(&sl->cnt_missed);
			}

			if ( (ret == -ETIME) &&
			     (atomic_read(&sl->cnt_missed) <= 1) &&
			     (!signal_pending(current)) ) {
				continue;
			}

			/* wait for semaphore - timeout expired twice or signal */
			atomic_set(&sl->cnt_missed, 0);
			down(&sl->close_sem);
			atomic_dec(&sl->depth);
			prt_msi_state("read before irq disable", 1, 0 ,0);
			prt_msi_state("read before irq disable", 2, 0 ,0);
			disable_irq(sl->irq);
			prt_msi_state("read after irq disable", 1, 0 ,0);
			prt_msi_state("read after irq disable", 2, 0 ,0);
			up(&sl->close_sem);
			if ( (ret == -ETIME) && (!signal_pending(current)) ) {
				ret = 0;
				pr_err("Int %u active without IRQ, doing WA\n", sl->irq);
				return ret;
			}
			return -EINTR;
		} else {
			break;
		}
	}
	/* if we got here, then a real interrupt occured */
	atomic_set(&sl->cnt_missed, 0);

	return 0;
}

static int mvIntDrv_open(struct inode *inode, struct file *file)
{
	struct mvintdrv_file_priv *priv;

	mvIntDrvNumOpened++;

	priv = kmalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	INIT_LIST_HEAD(&priv->msi_enabled_pdevs);

	file->private_data = priv;

	return 0;
}

static int mvIntDrv_release(struct inode *inode, struct file *file)
{
	mvIntDrvNumOpened--;
	if (!mvIntDrvNumOpened) {
		/* Cleanup */
		int i, slot;
		struct interrupt_slot *sl;
		struct pci_dev *pdev, *pdevs[4] = { NULL, NULL, NULL, NULL };
		u16 control;

		udelay(1750);

		prt_msi_state("release before msi disable", 1, 0 ,0);
		prt_msi_state("release before msi disable", 2, 0 ,0);
		for (i=0; i<2; i++) {
			pdev = mvintdrv_get_pci_dev_from_ar();
			if (pdev) {

				pci_dev_get(pdev);
				pr_debug("%s: Disabling MSI for %p devfn %x vendor %x devid %x msi_cap %x msi_enabled %d\n",
						__func__, pdev, pdev->devfn, pdev->vendor, pdev->device, pdev->msi_cap, pdev->msi_enabled);
				/*
				 *                     Need to disable MSI before releasing the interrupts, as currently
				 *                     the Kernel will only write the MSI address with zero WITHOUT
				 *                     disabling MSI, causing memory overrun of physical address zero in ARM 32-bit:
				 */
				pci_read_config_word(pdev, pdev->msi_cap + PCI_MSI_FLAGS, &control);
				control &= ~PCI_MSI_FLAGS_ENABLE;
				pci_write_config_word(pdev, pdev->msi_cap + PCI_MSI_FLAGS, control);

				pr_debug("%s: New MSI control reg value is: %x\n", __func__, control);

			}
		}

		udelay(20);

		prt_msi_state("release after msi disable", 1, 0 ,0);
		prt_msi_state("release after msi disable", 2, 0 ,0);

		for (slot = 0; slot < MAX_INTERRUPTS; slot++) {
			sl = &(mvIntDrv_slots[slot]);
			if (sl->state != IRQ_SLOT_STATE_ALLOCATED)
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

		prt_msi_state("release after free slot", 1, 0 ,0);
		prt_msi_state("release after free slot", 2, 0 ,0);

		udelay(20);
		for (i=0; i<2; i++) {
			pdev = pci_get_domain_bus_and_slot(0, 1+i,
					PCI_DEVFN(0,
						0));
			pdevs[i] = pdev;

			if (pdev) {
				pr_debug("%s: disabling msi via kernel for %p\n", __func__, pdev);
				pci_disable_msi(pdev);
			}
		}

		udelay(20);

		for (i=0; i<2; i++) {
			pdev = pdevs[i];
			if (pdev)
				pci_dev_put(pdev);
		}
	}

	mvintdrv_disable_msi_devs(file);

	kfree(file->private_data);

	return 0;
}

static struct file_operations mvIntDrv_fops = {
	.read = mvIntDrv_read,
	.write = mvIntDrv_write,
	.open = mvIntDrv_open,
	.release = mvIntDrv_release /* A.K.A close */
};

static int mvIntDrv_PreInitDrv(void)
{
	sema_init(&mvint_pci_devs_sem, 1);
	pr_info("Using %u timeout on interrupts\n", intdrv_timeout);
	pr_info("%s: Version: %s\n", __func__, INT_DRV_VER);
	return 0;
}

void mvintdrv_exit(void)
{
	mvchrdev_cleanup(chrdrv_ctx);
}

int mvintdrv_init(void)
{
	mvIntDrv_PreInitDrv();

	chrdrv_ctx = mvchrdev_init(MV_DRV_NAME, &mvIntDrv_fops);
	if (!chrdrv_ctx)
		return -EIO;

	memset(mvIntDrv_slots, 0, sizeof(mvIntDrv_slots));

	return 0;
}
