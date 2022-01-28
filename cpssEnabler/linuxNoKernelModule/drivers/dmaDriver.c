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
* dmaDriver.c
*
* DESCRIPTION:
*	mvDmaDrv - driver to map DMA memory to userspace
*	Usage:
*		fd = open("/dev/mvDmaDrv",O_RDWR);
*		mmap(,size, ..,fd,0) will allocate DMA block and map it
*		...
*		read(fd,uint64_t*,8) will read DMA address
*		close(fd) will unmap and free DMA block
*
*	Additional features used by linuxNokernelModule driver:
*		1. When virtual address == LINUX_VMA_DMABASE (defined below)
*		   then allocate a single DMA block which will be mapped to all
*		   applications to the same virtual address
*		2. Select PCI device for allocation:
*			lseek(fd,
*			      ((domain<<16)&0xffff0000) |
*			      ((bus<<8)   &0x0000ff00)  |
*			      ((dev<<3)   &0x000000f8)  |
*			      ((func)     &0x00000007))
*		   This should allow IOMMU transactions from PCI device to
*		   system memory
*
*	Please note:
*		On Intel CPU it may require 'intel_iommu=off' kernel option
*
*
*	Following is an example of pre-allocation in device-tree.
*
*	reserved-memory {
*		prestera_rsvd: buffer@0M {
*			compatible = "shared-dma-pool";
*			no-map;
*			reg = <0x0 0x10000 0x0 0x1000000>;
*		};
*	};
*
*	mvdma {
*		compatible = "marvell,mvdma";
*		memory-region = <&prestera_rsvd>;
*		status = "okay";
*	};
*
* DEPENDENCIES:
*	$Revision: 31 $
*
*******************************************************************************/
#define MV_DRV_NAME     "mvDmaDrv"
#define MV_DRV_MAJOR	244
#define MV_DRV_MINOR	3
#define MV_DRV_FOPS	mvDmaDrv_fops
#define MV_DRV_POSTINIT	mvDmaDrv_postInitDrv
#define MV_DRV_RELEASE	mvDmaDrv_releaseDrv

#define MV_DRV_NAME_VERSION "1.25"
#define MV_DRV_MAJOR	244
#define MV_DRV_MINOR	3
#define MV_DRV_FOPS	mvDmaDrv_fops
#define MV_DRV_PREINIT  mvDmaDrv_PreInitDrv
#define MV_DRV_POSTINIT	mvDmaDrv_postInitDrv
#define MV_DRV_RELEASE	mvDmaDrv_releaseDrv

/* registers */
#define FDB_GLOBAL_CONF_REG 0x04000000

/* MG registers */
#define MG_DEVID_REG 0x4C
#define MG_VENDID_REG 0x50
#define MG_EXT_GLOBAL_CNTRL_REG 0x5C
#define MG_AU_Q_HOST_CONF_REG 0xD8

#define MG_UEA_REG 0x208

#define MG_RX_SDMA_Q_CMD_REG 0x2680
#define MG_TX_SDMA_Q_CMD_REG 0x2868

#define MG_RX_SDMA_RES_ERR_MOD01 0x2860
#define MG_RX_SDMA_RES_ERR_MOD27 0x2878

#define MG_UDA 0x200

#define MG_INT_IRQ_CAUSE 0x38
#define MG1_INT_IRQ_CAUSE1 0x144
#define MGCAM_BAD_ADDR 0x198
#define MG_UEA 0x208
#define MG_INT_IRQ_CAUSE3 0x2e8
#define MG_CM3_SRAM_OOR_ADDR 0x358
#define MG_INT_IRQ_CAUSE1 0x618
#define MG_INT_IRQ_CAUSE2 0x678
#define MG_RX_SDMA_INT_CAUSE0 0x280c
#define MG_TX_SDMA_INT_CAUSE0 0x2810
#define MG_RX_SDMA_INT_CAUSE1 0x2890
#define MG_RX_SDMA_INT_CAUSE2 0x2894
#define MG_TX_SDMA_INT_CAUSE1 0x2898
#define MG_TX_SDMA_INT_CAUSE2 0x289C

/* CnM / external interfaces registers: */
#define CNM_PCIE_CONF_HDR_REG 0x40000
#define CNM_PCIE_CMD_STAT_REG 0x40004

/* DFX registers */
#define DFX_SKIP_PCIE_INIT_MATRIX_REG 0x000F806C
#define DFX_RST_CTRL_REG 0x000F800C

#define DFX_SKIP_REG_INIT_MATRIX_REG 0x000f8020
#define DFX_SKIP_RAM_INIT_MATRIX_REG 0x000f8030
#define DFX_SKIP_TABLES_INIT_MATRIX_REG 0x000f8060
#define DFX_SKIP_EEPROM_INIT_MATRIX_REG 0x000f8068
#define DFX_SKIP_MBUS_CTRL_INIT_MATRIX_REG 0x000f8044
#define DFX_SKIP_SERDES_INIT_MATRIX_REG 0x000f8064
#define DFX_SKIP_DFX_REGS_CONF_INIT_MATRIX_REG 0x000f8098
#define DFX_SKIP_IHB_INIT_MATRIX_REG 0x000f8048

/* bits */
#define AUMessageToCPUStop_BIT 0x40000000
#define SDMA_SW_RST_BIT 0x40
#define STOP_AU_Q_BIT 0x1
#define RESET_AU_Q_CNTRS_BITS 0x6

#define STOP_ALL_Q_BITS 0xff00

/* Retry counter = 0, No Retry, Drop in case of resource error */
#define RX_SDMA_ERR_MOD 0x100

/* CnM / external interfaces register bits: */
#define PCIE_MASTER_BIT 0x4

/* DFX register bits: */

/* all of these bits are active low (require setting to zero) */
#define SKIP_INIT_SOFT_RST_BIT 0x100
#define SKIP_INIT_SOFT_RST_ALL_SUBS_BITS 0x1FF
#define SKIP_INIT_SOFT_RST_ONLY_SUBS_BITS 0xFF

#define MG_SOFT_RST_TRIGGER_BIT 0x2
#define TABLE_START_INIT_BIT 0x4

#define DFX_SOFT_RST_BITS (MG_SOFT_RST_TRIGGER_BIT | TABLE_START_INIT_BIT)

#define SECS_DELAY_WQ 10

#include "mvDriverTemplate.h"

#include <linux/delay.h>
#include <linux/pci.h>
#include <linux/dma-mapping.h>
#include <linux/kallsyms.h>
#include <linux/platform_device.h>
#include <linux/of_reserved_mem.h>
#include <asm/div64.h>
#include <linux/platform_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>

#ifdef MTS_BUILD
#define LINUX_VMA_DMABASE 0x19000000UL
#endif

#ifndef LINUX_VMA_DMABASE
#if defined(CONFIG_X86_64)
#define LINUX_VMA_DMABASE 0x1fc00000UL
#elif defined(CONFIG_X86) || defined(CONFIG_ARCH_MULTI_V7) || defined(CONFIG_ARM64)
#define LINUX_VMA_DMABASE 0x1c800000UL
#endif /* CONFIG_X86 || CONFIG_ARCH_MULTI_V7 || CONFIG_ARM64 */
#ifdef CONFIG_MIPS
#define LINUX_VMA_DMABASE 0x2c800000UL
#endif /* CONFIG_MIPS */
#endif /* CONFIG_X86_64 */
#ifndef LINUX_VMA_DMABASE
#define LINUX_VMA_DMABASE 0x19000000UL
#endif /* LINUX_VMA_DMABASE */

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,6,0)) || defined(CONFIG_ARCH_MVEBU)
#define MMAP_USE_REMAP_PFN_RANGE
#endif

#define MV_DMA_ALLOC_FLAGS GFP_DMA32 | GFP_NOFS

/* Did we successfully registered as platform driver? zero means yes */
static u8 platdrv_registered;
static struct device *platdrv_dev;

#define MV_DMA_ALLOC_FLAGS GFP_DMA32 | GFP_NOFS

/* Did we successfully registered as platform driver? zero means yes */
static u8 platdrv_registered;
static struct device *platdrv_dev;

struct dma_mapping {
	void *virt;
    dma_addr_t dma;
	void *virt_dummy[16];
	dma_addr_t dma_dummy[16];
#ifdef MMAP_USE_REMAP_PFN_RANGE
	phys_addr_t phys;
	phys_addr_t phys_dummy[16];
#endif
	size_t size;
	loff_t pci_offset;
	struct pci_dev *pdev;
	struct device *dev;
	/* First index to array is the Packet Processor unit number, Second is the PCIe bar (0/2/4) */
	void __iomem *base[4][8];
	struct pci_dev *pdevs_list[4];
	struct delayed_work free_mem_delayed;
};

static struct dma_mapping *shared_dmaBlock;
static struct semaphore mvdma_sem;

/*
 * 64bit modulo division is undefined in 32bit armhf
 * mvDmaDrv_modulo() uses kernel do_div() to calculate modulo .
 * So we introduce
 * Wrapper for do_div(). It does in - place division .
 */

static noinline uint32_t mvDmaDrv_modulo(uint64_t num, uint64_t div) {
	return do_div(num, div);
}

static void free_dma_block(struct dma_mapping *m)
{
	if (!m->dma)
		return;

	printk(KERN_INFO "%s: dma_free_coherent(%p, 0x%lx, %p, 0x%llx)\n",
	       MV_DRV_NAME, m->dev ? m->dev : mvDrv_device,
	       (unsigned long)m->size, m->virt, (unsigned long long)m->dma);

	dma_free_coherent(m->dev ? m->dev : mvDrv_device, m->size, m->virt,
			  m->dma);
}


static int mvDmaDrv_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct dma_mapping *m = (struct dma_mapping *)file->private_data;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,11,0)
	int (*dma_configure)(struct device *dev);
	int ret;

	/* Until LK 4.11, dma_alloc_coherent(NULL, ...) is allowed, and used in
	   case of AC3 */
	if (!m->dev && !platdrv_dev) {
		printk(KERN_ERR "%s: Nither PCI, nor Platform device is registered, cannot mmap\n",
		       MV_DRV_NAME);
		return -EIO;
	}

	if (!m->dev && platdrv_dev)
		m->dev = platdrv_dev;
#endif

	dev_info(m->dev, "%s(file=%p) data=%p LINUX_VMA_DMABASE=0x%lx\n",
		 __func__, file, m, LINUX_VMA_DMABASE);

	if (m->dma && vma->vm_start != LINUX_VMA_DMABASE)
		return -ENXIO;

	if (vma->vm_start == LINUX_VMA_DMABASE && shared_dmaBlock) {
		if (m != shared_dmaBlock) {
			kfree(m);
			file->private_data = shared_dmaBlock;
		}
		m = shared_dmaBlock;
	} else {
		if (vma->vm_start == LINUX_VMA_DMABASE) {
			shared_dmaBlock = m;
		}
        if (m->dev && !platdrv_dev) { /* don't config dma_ops in case of no-dev, or for platdrv_dev */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,11,0)
		/* The new DMA framework, that was added in 4.11 (compared to
 		 * 4.4), does not initiate each PCI dev as DMA-enabled by
 		 * default (dev->dma_ops is set to dummy_dma_ops), so need to
 		 * set the PP to be DMA enabled. This can be done through DTS,
 		 * but it is not a solution for Intel CPUs, hance need to use
 		 * HACK to call dma_configure, which is not exported by the
 		 * kernel
		 */

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,4,0)
			dma_configure = m->dev->bus->dma_configure;
#else
			dma_configure = (void*)(unsigned long)
					kallsyms_lookup_name("dma_configure");
#endif

			if (!dma_configure) {
				dev_err(m->dev,
					"Fail to resolve dma_configure\n");
				return -ENXIO;
			}

			ret = dma_configure(m->dev);
			if (ret) {
				dev_err(m->dev,
					"dma_configure failed %d\n", ret);
				return -EFAULT;
			}
#endif

			dev_info(m->dev, "allocating for device %p %s\n",
				 m->dev, m->dev->kobj.name);
		}

		m->size = (size_t)(vma->vm_end - vma->vm_start);

		m->virt = dma_alloc_coherent(m->dev, m->size, &(m->dma),
					     MV_DMA_ALLOC_FLAGS);
		if (!m->virt) {
			dev_err(m->dev,
				"dma_alloc_coherent failed to allocate 0%x bytes\n",
				(unsigned)m->size);
			return -ENXIO;
		}

		/* If allocated physical address (m->dma) is not aligned with
		   size, which is a Prestera req, for example 0xb0500000 not
		   aligned with 0x200000 do:
		   1. Free DMA
		   2. Alloc (PHY mod size) up to alignment - 0x100000 in our
		      case
		   3. Alloc original size (0x200000)
		   4. free (2)
		   5. Check if aligned */
		if (mvDmaDrv_modulo(m->dma, m->size)) {
			struct dma_mapping m_1 = *m;
			m_1.size = m->size - mvDmaDrv_modulo(m->dma, m->size);

			dev_info(m->dev,
				"dma_alloc_coherent is not aligned. Reallocating\n");
			free_dma_block(m);

			m_1.virt = dma_alloc_coherent(m_1.dev, m_1.size,
						      &(m_1.dma),
						      MV_DMA_ALLOC_FLAGS);
			if (!m_1.virt) {
				dev_err(m->dev,
					"dma_alloc_coherent failed to allocate 0%x bytes\n",
					(unsigned) m_1.size);
				return -ENXIO;
			}

			m->virt = dma_alloc_coherent(m->dev, m->size, &(m->dma),
						     MV_DMA_ALLOC_FLAGS);
			free_dma_block(&m_1);
			if (!m->virt) {
				dev_err(m->dev,
					"dma_alloc_coherent failed to allocate 0%x bytes\n",
					(unsigned)m->size);
				return -ENXIO;
			}

			if (mvDmaDrv_modulo(m->dma, m->size)) {
				dev_err(m->dev,
					"dma_alloc_coherent failed to allocate aligned size of 0x%x for phys0x%lx\n",
					(unsigned)m->size,
					(unsigned long)m->dma);
				free_dma_block(m);
				return -ENXIO;
			}
		}

		dev_info(m->dev, "dma_alloc_coherent virt=%p dma=0x%llx\n",
			 m->virt, (unsigned long long)m->dma);

#ifdef MMAP_USE_REMAP_PFN_RANGE
#if defined(CONFIG_X86) || defined(CONFIG_MIPS)
		m->phys = dma_to_phys(m->dev, m->dma);
#else
		m->phys = (phys_addr_t)m->dma;
#endif
		dev_info(m->dev, "m->phys=0x%llx\n",
			 (unsigned long long)m->phys);
#endif
	}

#ifdef MMAP_USE_REMAP_PFN_RANGE
	/* VM_IO for I/O memory */
	vma->vm_flags |= VM_IO;
	vma->vm_pgoff = m->phys >> PAGE_SHIFT;

	/* If need to disable caching on mapped memory */
	/* vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot); */

	/* TODO: check if write combine support is needed ? */
	/* vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot); */

	dev_info(m->dev,
		 "remap_pfn_range(phys=0x%llx vm_start=0x%llx, vm_pgoff=0x%llx, vm_size=0x%lx)\n",
		 (unsigned long long)m->phys, (unsigned long long)vma->vm_start,
		 (unsigned long long)vma->vm_pgoff, (unsigned long)m->size);

	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff, m->size,
			    vma->vm_page_prot)) {
		dev_err(m->dev, "remap_pfn_range failed\n");
		return -ENXIO;
	}
#else /* MMAP_USE_REMAP_PFN_RANGE */
	dev_info(m->dev, "dma_mmap_coherent(vma=0x%llx, c=%p, dma=0x%llx, size=0x%llx)\n",
		(unsigned long long)vma->vm_start, m->virt,
		(unsigned long long)m->dma, (unsigned long long)m->size);
	if (dma_mmap_coherent(m->dev, vma, m->virt, m->dma, m->size)) {
		dev_info(m->dev, "dma_mmap_coherent() failed\n");
		return -ENXIO;
	}
#endif /* MMAP_USE_REMAP_PFN_RANGE */

	return 0;
}

static ssize_t mvDmaDrv_read(struct file *f, char *buf, size_t siz, loff_t *off)
{
	struct dma_mapping *m = (struct dma_mapping *)f->private_data;
	unsigned long long dma;

	if (!m)
		return -EFAULT;

	if (siz < sizeof(dma))
		return -EINVAL;

	dma = (unsigned long long)m->dma;

	if (copy_to_user(buf, &dma, sizeof(dma)))
		return -EFAULT;

	return sizeof(dma);
}

static void mvdma_free_memory_func(struct work_struct *work)
{
	struct dma_mapping *m;
	struct pci_dev *pdev;
	u32 val;
	int i, ii;

	pr_info("%s: start\n", __func__);

	m = container_of(to_delayed_work(work), struct dma_mapping, free_mem_delayed);
	if (!m->dev) {
	   pr_info("%s: No PCI device assigned!\n", __func__);
	   }
		else {
			for (i=0; i<2; i++) {

				pdev = m->pdevs_list[i];

				if (!pdev) {
					continue;
				}

				/* PCIe bars in AC3X / Aldrin are 0,2,4 */
				for (ii=0; ii<6; ii+=2) {
					if (!m->base[i][ii]) {
						continue;
						}
					pr_info("%s: Unmapping PCI bar %d...\n", __func__, i);
					pcim_iounmap(pdev, m->base[i][ii]);

				}

			   }
		     }

	if (m != shared_dmaBlock) {
		pr_info("%s: Freeing DMA memory...\n", __func__);
		mvDmaDrv_free_dma_block(m);
		kfree(m);
	}

	pr_info("%s: %s Driver freed to serve new request...\n", MV_DRV_NAME, __func__);
	up(&mvdma_sem);
}

static int mvDmaDrv_open(struct inode *inode, struct file *file)
{
	struct dma_mapping *m;

	down(&mvdma_sem);
	printk("%s: %s Driver allocating to serve new request...\n", MV_DRV_NAME, __func__);

	m = kzalloc(sizeof(struct dma_mapping), GFP_KERNEL);
	if (!m)
		return -ENOMEM;

	INIT_DELAYED_WORK(&m->free_mem_delayed, mvdma_free_memory_func);
	file->private_data = m;

	printk("%s: %s(file=%p) data=%p\n", MV_DRV_NAME, __func__, file, m);

	return 0;
}

static int mvDmaDrv_PollIRQStats(void __iomem *base)
{
 u32 val;

	/* Clear on Read registers: */
 val = readl(base + MG_RX_SDMA_INT_CAUSE0);
 pr_info("%s: reg %x = %x\n", __func__, MG_RX_SDMA_INT_CAUSE0, val);

 val = readl(base + MG_TX_SDMA_INT_CAUSE0);
 pr_info("%s: reg %x = %x\n", __func__,  MG_TX_SDMA_INT_CAUSE0, val);

 val = readl(base + MG_RX_SDMA_INT_CAUSE1);
 pr_info("%s: reg %x = %x\n", __func__,  MG_RX_SDMA_INT_CAUSE1, val);

 val = readl(base + MG_RX_SDMA_INT_CAUSE2);
 pr_info("%s: reg %x = %x\n", __func__,  MG_RX_SDMA_INT_CAUSE2, val);

 val = readl(base + MG_TX_SDMA_INT_CAUSE1);
 pr_info("%s: reg %x = %x\n",  __func__, MG_TX_SDMA_INT_CAUSE1, val);

 val = readl(base + MG_TX_SDMA_INT_CAUSE2);
 pr_info("%s: reg %x = %x\n",  __func__, MG_TX_SDMA_INT_CAUSE2, val);

 /* Clear on Write cause registers: */
 val = readl(base + MG_INT_IRQ_CAUSE);
 pr_info("%s: reg %x = %x\n",  __func__, MG_INT_IRQ_CAUSE, val);
 writel(0, base + MG_INT_IRQ_CAUSE);

 val = readl(base + MG1_INT_IRQ_CAUSE1);
 pr_info("%s: reg %x = %x\n",  __func__, MG1_INT_IRQ_CAUSE1, val);
 writel(0, base + MG1_INT_IRQ_CAUSE1);

 val = readl(base + MG_INT_IRQ_CAUSE3);
 pr_info("%s: reg %x = %x\n",  __func__, MG_INT_IRQ_CAUSE3, val);
 writel(0, base + MG_INT_IRQ_CAUSE3);

 val = readl(base + MG_INT_IRQ_CAUSE1);
 pr_info("%s: reg %x = %x\n",  __func__, MG_INT_IRQ_CAUSE1, val);
 writel(0, base + MG_INT_IRQ_CAUSE1);

 val = readl(base + MG_INT_IRQ_CAUSE2);
 pr_info("%s: reg %x = %x\n",  __func__, MG_INT_IRQ_CAUSE2, val);
 writel(0, base + MG_INT_IRQ_CAUSE2);

 val = readl(base + MGCAM_BAD_ADDR);
 pr_info("%s: reg %x = %x\n",  __func__, MGCAM_BAD_ADDR, val);

 val = readl(base + MG_UEA);
 pr_info("%s: reg %x = %x\n",  __func__, MG_UEA, val);

 val = readl(base + MG_CM3_SRAM_OOR_ADDR );
 pr_info("%s: reg %x = %x\n",  __func__, MG_CM3_SRAM_OOR_ADDR , val);

 return 0;
}

static loff_t mvDmaDrv_lseek(struct file *file, loff_t off, int unused)
{
	struct dma_mapping *m = (struct dma_mapping *)file->private_data;
	struct pci_dev *pdev;
	int domain = (off >> 16) & 0xffff;
	unsigned int bus = (off >> 8) & 0xff;
	unsigned int devfn = PCI_DEVFN(((off >> 3) & 0x1f), (off & 0x07));

	if (!m)
		return -EFAULT;

	if (platdrv_dev) /* device-tree reservation */
		return 0;

	pdev = pci_get_domain_bus_and_slot(domain, bus, devfn);
	if (pdev) {
		m->dev = &(pdev->dev);
		printk("%s: Using PCI device %s\n", MV_DRV_NAME,
		       m->dev->kobj.name);
	}

	return 0;
}

static int mvDmaDrv_do_CPSS_skip_sequence_pcie(void __iomem *base)
	u32 val;

	val = readl(base + DFX_SKIP_REG_INIT_MATRIX_REG);
	val |= SKIP_INIT_SOFT_RST_BIT;
	writel(val, base + DFX_SKIP_REG_INIT_MATRIX_REG);

	val = readl(base + DFX_SKIP_RAM_INIT_MATRIX_REG);
	val |= SKIP_INIT_SOFT_RST_BIT;
	writel(val, base + DFX_SKIP_RAM_INIT_MATRIX_REG);

	val = readl(base + DFX_SKIP_TABLES_INIT_MATRIX_REG);
	val |= SKIP_INIT_SOFT_RST_BIT;
	writel(val, base + DFX_SKIP_TABLES_INIT_MATRIX_REG);

	val = readl(base + DFX_SKIP_EEPROM_INIT_MATRIX_REG);
	val |= SKIP_INIT_SOFT_RST_BIT;
	writel(val, base + DFX_SKIP_EEPROM_INIT_MATRIX_REG);

	val = readl(base + DFX_SKIP_MBUS_CTRL_INIT_MATRIX_REG);
	val &= ~SKIP_INIT_SOFT_RST_BIT;
	writel(val, base + DFX_SKIP_MBUS_CTRL_INIT_MATRIX_REG);

	val = readl(base + DFX_SKIP_SERDES_INIT_MATRIX_REG);
	val |= SKIP_INIT_SOFT_RST_BIT;
	writel(val, base + DFX_SKIP_SERDES_INIT_MATRIX_REG);

	writel(SKIP_INIT_SOFT_RST_ALL_SUBS_BITS, base + DFX_SKIP_REG_INIT_MATRIX_REG);

	writel(SKIP_INIT_SOFT_RST_ALL_SUBS_BITS, base + DFX_SKIP_TABLES_INIT_MATRIX_REG);

	writel(SKIP_INIT_SOFT_RST_ONLY_SUBS_BITS, base + DFX_SKIP_MBUS_CTRL_INIT_MATRIX_REG);

	val = readl(base + DFX_SKIP_DFX_REGS_CONF_INIT_MATRIX_REG);
	val &= ~SKIP_INIT_SOFT_RST_BIT;
	writel(val, base + DFX_SKIP_DFX_REGS_CONF_INIT_MATRIX_REG);

	val = readl(base + DFX_SKIP_RAM_INIT_MATRIX_REG);
	val &= ~SKIP_INIT_SOFT_RST_BIT;
	writel(val, base + DFX_SKIP_RAM_INIT_MATRIX_REG);

	val = readl(base + DFX_SKIP_IHB_INIT_MATRIX_REG);
	val &= ~SKIP_INIT_SOFT_RST_BIT;
	writel(val, base + DFX_SKIP_IHB_INIT_MATRIX_REG);

	return 0;
}

static int mvDmaDrv_stopAndReset_AC3X_Aldrin_PP(struct dma_mapping *m)
{
 struct pci_dev *pdev;
 int err, i, ii;
 u32 val, did, vid, sdma_rx_q_cmd_val_before[2], pcie_conf_hdr[2], dfx_rst_reg[2], skip_pcie_reg[2];
 loff_t off;
 int domain;
 unsigned int bus;
 unsigned int devfn;
 int reset_done = 0;

 if (!m->dev) {
	pr_info("%s: No PCI device assigned!\n", __func__);
	return -ENOENT;
 	}

 for (i=0; i<2; i++) {
 	 off = m->pci_offset;
	 domain = (off >> 16) & 0xffff;
	 bus = (off >> 8) & 0xff;
	 devfn = PCI_DEVFN(((off >> 3) & 0x1f), (off & 0x07));

	 pdev = pci_get_domain_bus_and_slot(domain, bus + i, devfn);

	 if (!pdev) {
		pr_info("%s: Failed to get PCI device %x:%x:%x.%x\n", __func__, domain, bus + i, (unsigned)((off >> 3) & 0x1f), (unsigned)(off & 0x07));
		continue;
	 	}

	 m->pdevs_list[i] = pdev;
	 err = pcim_enable_device(pdev);

	 if (err) {
		pr_info("%s: pcim enable device failed err %d!\n",__func__,err);
		return err;
	 }

        if (!err){

                m->base[i][2] = pcim_iomap(pdev, 2, 64*1024*1024); /* switching registers/MG 64M AC3X/BC2/Aldrin only*/

               if (m->base[i][2]) {
                       /*
                        * Write Receive SDMA Queue Command Register in MG with stop all RX queues Values
                        * This will disable all of the SDMA RX reception.
                        * This covers situation in
                        * which SIGKILL is sent to CPSS application which will cause it to release
                        * the mvDMA driver file descriptor and call this callback. Writing these
                        * bits with ones will ensure the Packet Processor SDMA RX is reset to a determined
                        * state without having the RX DMA running unsupervised.
                        */

						/* disable retries on resource error */
						writel(RX_SDMA_ERR_MOD, m->base[i][2] + MG_RX_SDMA_RES_ERR_MOD01);

						writel(RX_SDMA_ERR_MOD, m->base[i][2] + MG_RX_SDMA_RES_ERR_MOD01 + 0x4);

						for (ii=0; ii<6; ii++)
							writel(RX_SDMA_ERR_MOD, m->base[i][2] + MG_RX_SDMA_RES_ERR_MOD27 + (0x4 * ii));

						 /* Stop SDMA RX - prevent writing on memory which will be released to kernel */
						writel(STOP_ALL_Q_BITS, m->base[i][2] + MG_RX_SDMA_Q_CMD_REG);

			   			/* Stop SDMA TX - descriptor writeback might corrupt memory */
			   		   writel(STOP_ALL_Q_BITS, m->base[i][2] + MG_TX_SDMA_Q_CMD_REG);

						/* Stop FDB AU messages to CPU (FDB unit) */

						/* Stop AU messages on MG (DMA) */
					   writel(STOP_AU_Q_BIT, m->base[i][2] + MG_AU_Q_HOST_CONF_REG);

						/* Reset AU+FU queue counters (reset queue) on MG (DMA) */
					   writel(RESET_AU_Q_CNTRS_BITS | STOP_AU_Q_BIT, m->base[i][2] + MG_AU_Q_HOST_CONF_REG);

						/* Finally, completely reset SDMA: */
						val = readl(m->base[i][2] + MG_EXT_GLOBAL_CNTRL_REG);

						val |= SDMA_SW_RST_BIT;

						writel(val, m->base[i][2] + MG_EXT_GLOBAL_CNTRL_REG);

					    val &= ~SDMA_SW_RST_BIT;

					    writel(val, m->base[i][2] + MG_EXT_GLOBAL_CNTRL_REG);

               }
	       else pr_info("%s: base %d BAR2 not mappable!\n", __func__, i);


				m->base[i][0] = pcim_iomap(pdev, 0, 1*1024*1024); /* CnM / external interfaces registers 1M AC3X/BC2/Aldrin only*/

				if (m->base[i][0]) {

					/* Disable PCIe bus mastering from AC3X/Aldrin to CPU/SOC: */

					val = readl(m->base[i][0] + CNM_PCIE_CMD_STAT_REG);

					val &= ~PCIE_MASTER_BIT;

					writel(val, m->base[i][0] + CNM_PCIE_CMD_STAT_REG);

					}
					else pr_info("%s: base %d BAR0 not mappable!\n", __func__, i);


					m->base[i][4] = pcim_iomap(pdev, 4, 8*1024*1024); /* DFX 8M AC3X/BC2/Aldrin only*/


					if (m->base[i][4]) {
						/* Set Skip PCIe reset when doing MG soft reset sequence: */
						/* Finally, soft reset Packet Processor: */
 			          /*
			                Behavior is un-predictable (probably the device will hang)
			                if CPU try to read/write registers/tables
			                of the device during the time of soft reset
			                (Soft reset is active for 2000 core clock cycles (6uS). Waiting 20uS should be enough)
			                Event when '<PEX Skip Init if MG Soft Reset> = SKIP INIT ON'
			                (no pex reset).
			                *******************************
			                meaning that even when skip pex reset there is still interval of
			                time that the CPU must not approach the device.
			            */

			            mvDmaDrv_do_CPSS_skip_sequence_pcie(m->base[i][4]);

						val = readl(m->base[i][4] + DFX_RST_CTRL_REG);

						val &= ~DFX_SOFT_RST_BITS;

						dfx_rst_reg[i] = val;

						mb(); /* Synchronize CPU to finish all writes to PP address space in order to ensure no writes to PP will happen */

						writel(val, m->base[i][4] + DFX_RST_CTRL_REG);

						mb();

						reset_done++;

						}
						else pr_info("%s: base %d BAR4 not mappable!\n", __func__, i);

        }
	else pr_info("%s: unable to iomap %d. err %d!", __func__, i, err);
 	}

	 udelay(20);

 	for (i=0; i<2; i++) {
		pr_info("%s: Stopped DMA on PCI device %x:%x:%x.%x\n", __func__, domain, bus + i, (unsigned)((off >> 3) & 0x1f), (unsigned)(off & 0x07));
		if (reset_done)
		   printk("%s: device %d: Wrote %x to DFX rst reg %x reset_cnt %d\n",
				   __func__, i, dfx_rst_reg[i], DFX_RST_CTRL_REG, reset_done);

 		}

	return err;
}

static int mvDmaDrv_release(struct inode *inode, struct file *file)
{
	struct dma_mapping *m = (struct dma_mapping *)file->private_data;
	int ret;

	printk("%s: %s(file=%p) data=%p\n", MV_DRV_NAME, __func__, file, m);
	ret = mvDmaDrv_stopAndReset_AC3X_Aldrin_PP(m);

	printk("%s: %s: return value of stop and reset PP SDMA RX is: %d\n",
			MV_DRV_NAME, __func__, ret);
	printk("%s: delaying DMA memory free by %d second...\n", __func__, SECS_DELAY_WQ);
	schedule_delayed_work(&m->free_mem_delayed, msecs_to_jiffies(1000*SECS_DELAY_WQ));

	return ret;
}

static int mvdmadrv_pdriver_probe(struct platform_device *pdev)
{
	int err;

	err = of_reserved_mem_device_init(&pdev->dev);
	if (err) {
		dev_err(&pdev->dev, "Could not get reserved memory\n");
		return -ENOMEM;
	}

	platdrv_dev = &pdev->dev;

	printk("%s: Using platform device %s\n", MV_DRV_NAME, pdev->name);

	return 0;
};

static int mvdmadrv_pdriver_remove(struct platform_device *pdev)
{
	BUG_ON(!platdrv_registered);

	of_reserved_mem_device_release(&pdev->dev);

	return 0;
 }

static const struct of_device_id mvdmadrv_of_match_ids[] = {
	 { .compatible = "marvell,mv_dma", },
};

static struct platform_driver mvdmadrv_platform_driver = {
	.probe		= mvdmadrv_pdriver_probe,
	.remove		= mvdmadrv_pdriver_remove,
	.driver		= {
		.name	= MV_DRV_NAME,
		.of_match_table = mvdmadrv_of_match_ids,
	},
};

static const struct of_device_id mvdmadrv_of_match_ids[] = {
	 { .compatible = "marvell,mv_dma", },
};

static struct platform_driver mvdmadrv_platform_driver = {
	.probe		= mvdmadrv_pdriver_probe,
	.remove		= mvdmadrv_pdriver_remove,
	.driver		= {
		.name	= MV_DRV_NAME,
		.of_match_table = mvdmadrv_of_match_ids,
	},
};

static struct file_operations mvDmaDrv_fops = {
	.mmap	= mvDmaDrv_mmap,
	.read	= mvDmaDrv_read,
	.open	= mvDmaDrv_open,
	.llseek	= mvDmaDrv_lseek,
	.release= mvDmaDrv_release /* A.K.A close */
};

static void mvDmaDrv_releaseDrv(void)
{
	if (platdrv_registered)
		platform_driver_unregister(&mvdmadrv_platform_driver);

	if (shared_dmaBlock) {
		mvDmaDrv_free_dma_block(shared_dmaBlock);
		kfree(shared_dmaBlock);
	}
}
static int mvDmaDrv_PreInitDrv(void)
{
	sema_init(&mvdma_sem, 1);
	printk("%s: %s version %s\n", __func__, MV_DRV_NAME, MV_DRV_NAME_VERSION);

	return 0;
}

static void mvDmaDrv_postInitDrv(void)
{
	int err;

	err = platform_driver_register(&mvdmadrv_platform_driver);
	if (err)
		printk(KERN_ERR "%s: Fail to register platform driver\n",
		       MV_DRV_NAME);
	else
		platdrv_registered = 1;

	printk(KERN_DEBUG "%s: major=%d minor=%d\n", MV_DRV_NAME, major, minor);
}
