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
*       mvDmaDrv - driver to map DMA memory to userspace
*                  Usage:
*                     fd=open("/dev/mvDmaDrv",O_RDWR);
*                     mmap(,size, ..,fd,0) will allocate DMA block and map it
*                     ...
*                     read(fd,uint64_t*,8) will read DMA address
*                                          (prvExtDrvDmaPhys64)
*                     close(fd) will unmap and free DMA block
*
*       Additional features used by linuxNokernelModule driver:
*           1. When virtual address == LINUX_VMA_DMABASE (defined below)
*              then allocate a single DMA block which will be mapped to
*              all applications to the same virtual address
*           2. Select PCI device for allocation:
*                   lseek(fd,
*                       ((domain<<16)&0xffff0000)
*                       |((bus<<8)   &0x0000ff00)
*                       |((dev<<3)   &0x000000f8)
*                       |((func)     &0x00000007))
*               This should allow IOMMU transactions from PCI device to
*               system memory
*
*       Please note:
*           on Intel CPU it may require 'intel_iommu=off' kernel option
*
* DEPENDENCIES:
*
*       $Revision: 31 $
*******************************************************************************/
#define MV_DRV_NAME     "mvDmaDrv"
#define MV_DRV_MAJOR    244
#define MV_DRV_MINOR    3
#define MV_DRV_FOPS     mvDmaDrv_fops
#define MV_DRV_POSTINIT mvDmaDrv_postInitDrv
#define MV_DRV_RELEASE  mvDmaDrv_releaseDrv
#include "mvDriverTemplate.h"

#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/kallsyms.h>

/* TODO: include prvExtDrvLinuxMapping.h */
#ifdef MTS_BUILD
#  define LINUX_VMA_DMABASE       0x19000000UL
#endif

#ifndef LINUX_VMA_DMABASE
#  if defined(CONFIG_X86_64)
#    define LINUX_VMA_DMABASE       0x1fc00000UL
#  elif defined(CONFIG_X86) || defined(CONFIG_ARCH_MULTI_V7) || defined(CONFIG_ARM64)
#    define LINUX_VMA_DMABASE       0x1c800000UL
#  endif
#  ifdef CONFIG_MIPS
#    define LINUX_VMA_DMABASE       0x2c800000UL
#  endif
#endif
#ifndef LINUX_VMA_DMABASE
#  define LINUX_VMA_DMABASE       0x19000000UL
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,6,0)) || defined(CONFIG_ARCH_MVEBU)
#  define MMAP_USE_REMAP_PFN_RANGE
#endif

struct dma_mapping {
    void       *virt;
    dma_addr_t  dma;
#ifdef MMAP_USE_REMAP_PFN_RANGE
    phys_addr_t phys;
#endif
    size_t      size;
    struct device *dev;
};

static struct dma_mapping *shared_dmaBlock = NULL;

static unsigned mvDmaDrv_modulo(unsigned long num, unsigned long div) {
    while (num >= div)
        num -= div;
    return num;
}

static void free_dma_block(struct dma_mapping *m)
{
    if (m->dma == 0)
        return;
    printk("dma_free_coherent(%p, 0x%lx, %p, 0x%llx)\n",
            m->dev ? m->dev : mvDrv_device,
            (unsigned long)m->size, m->virt, (unsigned long long)m->dma);

    dma_free_coherent(m->dev ? m->dev : mvDrv_device, m->size, m->virt, m->dma);
}


static int mvDmaDrv_mmap(struct file * file, struct vm_area_struct *vma)
{
    struct dma_mapping *m = (struct dma_mapping *)file->private_data;

    printk("mvDmaDrv_mmap(file=%p) data=%p LINUX_VMA_DMABASE=0x%lx\n", file, m, LINUX_VMA_DMABASE);
    if (m->dma != 0 && vma->vm_start != LINUX_VMA_DMABASE)
        return -ENXIO;

    if (vma->vm_start == LINUX_VMA_DMABASE && shared_dmaBlock != NULL) {
        if (m != shared_dmaBlock) {
            kfree(m);
            file->private_data = shared_dmaBlock;
        }
        m = shared_dmaBlock;
    } else {
        if (vma->vm_start == LINUX_VMA_DMABASE) {
            shared_dmaBlock = m;
         }
        if (m->dev != NULL) {

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,11,0)
        /* The new DMA framework, that was added in 4.11 (compared to 4.4), does not initiate each PCI dev as DMA-enabled
           by default (dev->dma_ops is set to dummy_dma_ops), so need to set the PP to be DMA enabled. This can be done
           through DTS, but it is not a solution for Intel CPUs, hance need to use HACK to call dma_configure, which is not
           exported by the kernel
         */
        {
            int ret = 0;
            int (*dma_configure)(struct device *dev) = (void*)(unsigned long)kallsyms_lookup_name("dma_configure");
            if (!dma_configure) {
                dev_err(m->dev, "could not retrive address of function dma_configure\n");
                return ENXIO;
            }
            ret = dma_configure(m->dev);
            if (ret) {
                dev_err(m->dev, "dma_configure failed %d\n", ret);
                return EFAULT;
            }
        }
#endif
            printk("allocating for device %p %s\n", m->dev, m->dev->kobj.name);
            if (dma_set_mask(m->dev, DMA_BIT_MASK(32))) {
                dev_err(m->dev, "No suitable DMA available\n");
                return EFAULT;
            }
        }
        m->size = (size_t)(vma->vm_end - vma->vm_start);
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,32)
#  define MV_DMA_ALLOC_FLAGS  GFP_DMA | GFP_KERNEL
#else
#  define MV_DMA_ALLOC_FLAGS  GFP_DMA32 | GFP_NOFS
#endif
        m->virt = dma_alloc_coherent(m->dev, m->size, &(m->dma), MV_DMA_ALLOC_FLAGS);
        if (!m->virt) {
            printk("dma_alloc_coherent() failed to allocate 0%x bytes\n",(unsigned)m->size);
            return -ENXIO;
        }

        /* If allocated phsical address (m->dma) is not aligned with size, which is a Prestera req,
           for example 0xb0500000 not aligned with 0x200000 do:
        1. Free DMA
        2. Alloc (PHY mod size) up to alignment - 0x100000 in our case
        3. Alloc original size (0x200000)
        4. free (2)
        5. Check if aligned
        */
        /* 64bit modulo division is undefined in 32bit armhf
         * undefined symbol __aeabi_uldivmod error
         * Use a local function to replace modulo operator
         */
        if (mvDmaDrv_modulo(m->dma, m->size)) {
            struct dma_mapping m_1 = *m;
            m_1.size = m->size - mvDmaDrv_modulo(m->dma, m->size);

            printk("dma_alloc_coherent() is not aligned. Reallocating\n");
            free_dma_block(m);
            m_1.virt = dma_alloc_coherent(m_1.dev, m_1.size, &(m_1.dma), MV_DMA_ALLOC_FLAGS);
            if (!m_1.virt) {
                printk("dma_alloc_coherent() failed to allocate 0%x bytes\n",(unsigned)m_1.size);
                return -ENXIO;
            }
            m->virt = dma_alloc_coherent(m->dev, m->size, &(m->dma), MV_DMA_ALLOC_FLAGS);
            free_dma_block(&m_1);
            if (!m->virt) {
                printk("dma_alloc_coherent() failed to allocate 0%x bytes\n",(unsigned)m->size);
                return -ENXIO;
            }
            if (mvDmaDrv_modulo(m->dma, m->size)) {
                printk("dma_alloc_coherent() failed to allocate aligned size of 0x%x for physical 0x%lx\n",(unsigned)m->size, (unsigned long)m->dma);
                free_dma_block(m);
                return -ENXIO;
            }
        }

        printk("dma_alloc_coherent() virt=%p dma=0x%llx\n", m->virt, (unsigned long long)m->dma);

#ifdef MMAP_USE_REMAP_PFN_RANGE
#if defined(CONFIG_X86) || defined(CONFIG_MIPS)
        m->phys = dma_to_phys(m->dev, m->dma);
#else
        m->phys = (phys_addr_t)m->dma;
#endif
        printk("m->phys=0x%llx\n",(unsigned long long)m->phys);
#endif
    }

#ifdef MMAP_USE_REMAP_PFN_RANGE
    /* VM_IO for I/O memory */
    vma->vm_flags |= VM_IO;
    vma->vm_pgoff = m->phys >> PAGE_SHIFT;

#if 0
    /* disable caching on mapped memory */
    vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
#endif
    // TODO: check if write combine support is  needed ?
    //vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

    printk("remap_pfn_range(phys=0x%llx vm_start=0x%llx, vm_pgoff=0x%llx, vm_size=0x%lx, )\n",
                (unsigned long long)m->phys,
                (unsigned long long)vma->vm_start,
                (unsigned long long)vma->vm_pgoff,
                (unsigned long)m->size);

    if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff, m->size, vma->vm_page_prot)) {
        printk("remap_pfn_range failed\n");
        return -ENXIO;
    }
#else
    printk("dma_mmap_coherent(dev=%s, vma=0x%llx, c=%p, dma=0x%llx, size=0x%llx)\n",
                ((m->dev)?m->dev->kobj.name:"NULL"),
                (unsigned long long)vma->vm_start,
                m->virt,
                (unsigned long long)m->dma,
                (unsigned long long)m->size);
    if (dma_mmap_coherent(m->dev, vma, m->virt, m->dma, m->size)) {
        printk("dma_mmap_coherent() failed\n");
        return -ENXIO;
    }
#endif

    return 0;
}


static loff_t mvDmaDrv_lseek(struct file * file, loff_t off, int unused)
{
    struct dma_mapping *m = (struct dma_mapping *)file->private_data;
    struct pci_dev *pdev;
    int domain = (off >> 16) & 0xffff;
    unsigned int bus = (off >> 8) & 0xff;
    unsigned int devfn = PCI_DEVFN(((off >> 3) & 0x1f), (off & 0x07));

    if (!m)
        return -EFAULT;
    pdev = pci_get_domain_bus_and_slot(domain, bus, devfn);
    if (pdev == NULL)
        return -ENOENT;

    m->dev = &(pdev->dev);
    printk("selected PCI device %s d=%p\n", m->dev->kobj.name,m->dev);
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

static int mvDmaDrv_open(struct inode *inode, struct file *file)
{
    struct dma_mapping *m;
    m = kzalloc(sizeof(struct dma_mapping), GFP_KERNEL);
    if (m == NULL) {
        return -ENOMEM;
    }
    file->private_data = m;
    printk("mvDmaDrv_open(file=%p) data=%p\n", file, m);

    return 0;
}

static int mvDmaDrv_release(struct inode *inode, struct file *file)
{
    struct dma_mapping *m = (struct dma_mapping *)file->private_data;
    printk("mvDmaDrv_release(file=%p) data=%p\n", file, m);
    if (m != shared_dmaBlock) {
        free_dma_block(m);
        kfree(m);
    }
    return 0;
}

static struct file_operations mvDmaDrv_fops = {
    .mmap   = mvDmaDrv_mmap,
    .read   = mvDmaDrv_read,
    .open   = mvDmaDrv_open,
    .llseek = mvDmaDrv_lseek,
    .release= mvDmaDrv_release /* A.K.A close */
};

static void mvDmaDrv_releaseDrv(void)
{
    if (shared_dmaBlock != NULL) {
        free_dma_block(shared_dmaBlock);
        kfree(shared_dmaBlock);
    }
}
static void mvDmaDrv_postInitDrv(void)
{
#if 0
    if (dma_set_mask(mvDrv_device, DMA_BIT_MASK(32))) {
        dev_warn(mvDrv_device, "mydev: No suitable DMA available\n");
    }
#endif
    printk(KERN_DEBUG "mvDmaDrv major=%d minor=%d\n", major, minor);
}

