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
*******************************************************************************/
#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/netfilter.h>
#include <net/psample.h>
#include <linux/uaccess.h>
#include <linux/proc_fs.h>
#include <linux/if_vlan.h>

#define SAI_NAME	"mvsai"
#define SAI_MAJOR	0

#define DSA_SIZE 16
#define IFNAME_SIZE 16

/* IOCTL commands */
/* sflow */
#define SFLOW_INGRESS_ENABLE 3
#define SFLOW_EGRESS_ENABLE 4
#define SFLOW_INGRESS_DISABLE 5
#define SFLOW_EGRESS_DISABLE 6
#define SFLOW_FLUSH 7
/* vlan tag */
#define VLAN_TAG_STRIP 8
#define VLAN_TAG_KEEP 9
#define VLAN_TAG_ORIGINAL 10
#define VLAN_TAG_FLUSH 11

static int major;
static struct class *sai_class;
struct proc_dir_entry *sai_proc;

struct sai_user_data {
    unsigned int len;
    char ifname[IFNAME_SIZE];
    union {
        unsigned int rate;
        unsigned int pvid;
    };
};

struct sflow_config {
    unsigned int len;
    char ifname[IFNAME_SIZE];
    unsigned int ing_rate;
    unsigned int egr_rate;
    u8 nf;
    struct net_device *dev;
    struct list_head list;
};

struct vlan_tag_config {
    unsigned int len;
    char ifname[IFNAME_SIZE];
    unsigned int cmd;
    unsigned int pvid;
    u8 nf;
    struct net_device *dev;
    struct list_head list;
};

struct list_head sflow_plist, vlantag_plist;
struct psample_group __rcu *psample_group;
u32 psample_group_num = 1;
atomic_t sflow_pkt_ing_cnt = ATOMIC_INIT(0);
atomic_t sflow_pkt_egr_cnt = ATOMIC_INIT(0);

struct dsa_128bit_var {
    u64 high, low;
};

int sflow_register_nf(struct sflow_config *add);
void sflow_unregister_nf(struct sflow_config *del);
int vlantag_register_nf(struct vlan_tag_config *add);
void vlantag_unregister_nf(struct vlan_tag_config *del);

/*
 * Sflow DSA tags:
 * CPSS CPUCode Ingress - CPSS_HAL_CTRLPKT_CPU_CODE_SAMPLE_USER_DEFINE_0 to
 *                          CPSS_HAL_CTRLPKT_CPU_CODE_SAMPLE_USER_DEFINE_6
 * CPSS CPUCode Egress - CPSS_HAL_CTRLPKT_CPU_CODE_SAMPLE_USER_DEFINE_7 to
 *                          CPSS_HAL_CTRLPKT_CPU_CODE_SAMPLE_USER_DEFINE_13
 * dsaCpuCode = PRV_CPSS_DXCH_NET_DSA_TAG_FIRST_USER_DEFINED_E +
 *                       ((cpuCode - CPSS_NET_FIRST_USER_DEFINED_E) % 64);
 */
u8 sflow_dsa_ing[DSA_SIZE] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
u8 sflow_dsa_egr[DSA_SIZE] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
u8 sflow_dsa_mask[DSA_SIZE] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static DEFINE_MUTEX(sai_config_lock);

#if 0
static void print_buf(const char *title, const u8 *data, size_t len)
{
    char b[1024];
    int i;

    memset(b, 0, sizeof(b));

    for (i = 0; i < len; i++)
        sprintf(b, "%s %.2x ", b, data[i]);

    printk("%s: %s", title, b);
}
#endif

static unsigned int sflow_callback(void *priv, struct sk_buff *skb,
        const struct nf_hook_state *state)
{
    u8 *dsa = NULL;
    struct sflow_config *sflow = NULL;
    struct dsa_128bit_var *v, *m, *d;
    struct psample_group *group;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,10,0)
    struct psample_metadata md = {};
#endif

    sflow = (struct sflow_config *)priv;
    if (!sflow) {
        return NF_ACCEPT;
    }

    dsa = ((u8 *)skb->data - ETH_HLEN - DSA_SIZE);
    d = (struct dsa_128bit_var *)dsa;
    //print_buf("dsa", dsa, 16);

    m = (struct dsa_128bit_var *)&sflow_dsa_mask;
    if (sflow->ing_rate) {
        v = (struct dsa_128bit_var *)&sflow_dsa_ing;
        if ((v->high == (d->high & m->high)) &&
                (v->low == (d->low & m->low))) {
            /* The psample module expects skb->data to point to the start of the
             * Ethernet header.
             */
            skb_push(skb, ETH_HLEN);
            // rcu lock is held during call of nf_hook
            group = rcu_dereference(psample_group);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,10,0)
            md.trunc_size = skb->len;
            md.in_ifindex = skb->dev->ifindex;
            psample_sample_packet(group, skb, sflow->ing_rate, &md);
#else
            psample_sample_packet(group, skb, skb->len, skb->dev->ifindex, 0, sflow->ing_rate);
#endif
            atomic_inc(&sflow_pkt_ing_cnt);
            skb_pull(skb, ETH_HLEN);
            consume_skb(skb);
            return NF_STOLEN;
        }
    }

    if (sflow->egr_rate) {
        v = (struct dsa_128bit_var *)&sflow_dsa_egr;
        if ((v->high == (d->high & m->high)) &&
                (v->low == (d->low & m->low))) {
            skb_push(skb, ETH_HLEN);
            // rcu lock is held during call of nf_hook
            group = rcu_dereference(psample_group);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,10,0)
            md.trunc_size = skb->len;
            md.out_ifindex = skb->dev->ifindex;
            psample_sample_packet(group, skb, sflow->egr_rate, &md);
#else
            psample_sample_packet(group, skb, skb->len, 0, skb->dev->ifindex, sflow->egr_rate);
#endif
            atomic_inc(&sflow_pkt_egr_cnt);
            skb_pull(skb, ETH_HLEN);
            consume_skb(skb);
            return NF_STOLEN;
        }
    }

    return NF_ACCEPT;
}

static struct nf_hook_ops sops = {
    .hook = sflow_callback,
    .pf = NFPROTO_NETDEV,
    .hooknum = NF_NETDEV_INGRESS,
    .priority = -2
};

int sflow_register_nf(struct sflow_config *add)
{
    int rc = 0;

    sops.dev = dev_get_by_name(&init_net, add->ifname);
    add->dev = sops.dev;
    sops.priv = add;

    rc = nf_register_net_hook(&init_net, &sops);
    if (rc) {
        pr_err("sflow: Interface %s fails to register for nf, rc=%d\n", add->ifname, rc);
        return rc;
    }

    add->nf = 1;
    return rc;
}

void sflow_unregister_nf(struct sflow_config *del)
{
    if(del->nf) {
        sops.dev = del->dev;
        dev_put(del->dev);
        nf_unregister_net_hook(&init_net, &sops);
    }
    del->nf = 0;
}

/* Callback to update vlan tag as per config.
 * Vlan tag header is already untagged when frame reaches nf.
 * Update the vlan fileds in skb.
 */
static unsigned int vlan_tag_callback(void *priv, struct sk_buff *skb,
        const struct nf_hook_state *state)
{
    struct vlan_tag_config *vlan_info = NULL;

    vlan_info = (struct vlan_tag_config *)priv;
    //KEEP
    if (vlan_info->cmd == VLAN_TAG_KEEP) {
        if (skb_vlan_tag_present(skb)) {
            return NF_ACCEPT;
        }
        __vlan_hwaccel_put_tag(skb, htons(ETH_P_8021Q), vlan_info->pvid);
        return NF_ACCEPT;
    }

    //STRIP
    if (vlan_info->cmd == VLAN_TAG_STRIP) {
        skb->vlan_tci = 0;
    }
    return NF_ACCEPT;
}

static struct nf_hook_ops vops = {
    .hook = vlan_tag_callback,
    .pf = NFPROTO_NETDEV,
    .hooknum = NF_NETDEV_INGRESS,
    .priority = -1
};

int vlantag_register_nf(struct vlan_tag_config *add)
{
    int rc = 0;

    vops.dev = dev_get_by_name(&init_net, add->ifname);
    add->dev = vops.dev;
    vops.priv = add;

    rc = nf_register_net_hook(&init_net, &vops);
    if (rc) {
        pr_err("vlan_tag : Interface %s fails to register for nf, rc=%d\n", add->ifname, rc);
        return rc;
    }

    add->nf = 1;
    return rc;
}

void vlantag_unregister_nf(struct vlan_tag_config *del)
{
    if(del->nf) {
        vops.dev = del->dev;
        dev_put(del->dev);
        nf_unregister_net_hook(&init_net, &vops);
    }
    del->nf = 0;
}


static void sflow_config_free(void)
{
    struct sflow_config *del;

    while (!list_empty(&sflow_plist)) {
        del = list_first_entry(&sflow_plist,  struct sflow_config, list);
        if (!del)
            break;

        sflow_unregister_nf(del);
        list_del(&del->list);
        kfree(del);
    }
}

static void vlantag_config_free(void)
{
    struct vlan_tag_config *del;

    while (!list_empty(&vlantag_plist)) {
        del = list_first_entry(&vlantag_plist,  struct vlan_tag_config, list);
        if (!del)
            break;

        vlantag_unregister_nf(del);
        list_del(&del->list);
        kfree(del);
    }
}

static int sai_netdevice_event(struct notifier_block *unused,
                 unsigned long event, void *ptr)
{
    struct net_device *dev = netdev_notifier_info_to_dev(ptr);
    struct vlan_tag_config *vdel = NULL;
    struct sflow_config *sdel = NULL;
    struct list_head *tmp = NULL;

    switch (event) {
         case NETDEV_UNREGISTER:
            mutex_lock(&sai_config_lock);
            list_for_each(tmp, &vlantag_plist) {
                vdel = list_entry(tmp, struct vlan_tag_config, list);

                if (vdel->dev == dev) {
                    vlantag_unregister_nf(vdel);
                    list_del(&vdel->list);
                    kfree(vdel);
                    break;
                }
            }
            list_for_each(tmp, &sflow_plist) {
                sdel = list_entry(tmp, struct sflow_config, list);

                if (sdel->dev == dev) {
                    sflow_unregister_nf(sdel);
                    list_del(&sdel->list);
                    kfree(sdel);
                    break;
                }
            }
            mutex_unlock(&sai_config_lock);
            break;
    }

    return NOTIFY_DONE;
}

static struct notifier_block sai_notifier = {
    .notifier_call = sai_netdevice_event,
};

static int sai_open(struct inode *i, struct file *f)
{
    return 0;
}

static int sai_close(struct inode *i, struct file *f)
{
    return 0;
}

static long
sai_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct sai_user_data __user *argp = (void __user *)arg;
    struct sflow_config *sadd, *sdel, *scur;
    struct vlan_tag_config *vadd, *vdel, *vcur;
    struct list_head *tmp = NULL;
    char ifname[IFNAME_SIZE];
    unsigned int len;
    unsigned int rate;
    int err = 0;

    sadd = sdel = scur = NULL;

    switch (cmd) {
        case SFLOW_INGRESS_ENABLE:
        case SFLOW_EGRESS_ENABLE:
            sadd = kmalloc(sizeof(*sadd), GFP_KERNEL);
            if (!sadd) {
                return -ENOMEM;
            }
            memset(sadd, 0, sizeof(*sadd));

            if (get_user(sadd->len, &argp->len) < 0) {
                kfree(sadd);
                return -EFAULT;
            }

            sadd->len = min_t(size_t, sadd->len, IFNAME_SIZE - 1);
            if (copy_from_user(&sadd->ifname, &argp->ifname, sadd->len)) {
                kfree(sadd);
                return -EFAULT;
            }
            sadd->ifname[sadd->len] = '\0';

            if(get_user(rate, &argp->rate) < 0) {
                kfree(sadd);
                return -EFAULT;
            }

            (cmd == SFLOW_INGRESS_ENABLE) ? (sadd->ing_rate = rate):(sadd->egr_rate = rate);

            mutex_lock(&sai_config_lock);
            list_for_each(tmp, &sflow_plist) {
                scur = list_entry(tmp, struct sflow_config, list);
                if (!strcmp(scur->ifname, sadd->ifname)) {
                    //Entry already exist. Update rate
                    (cmd == SFLOW_INGRESS_ENABLE) ? (scur->ing_rate = sadd->ing_rate):
                        (scur->egr_rate = sadd->egr_rate);
                    mutex_unlock(&sai_config_lock);
                    kfree(sadd);
                    return 0;
                }
            }

            list_add(&sadd->list, &sflow_plist);

            err = sflow_register_nf(sadd);
            if (err != 0) {
                return err;
            }
            mutex_unlock(&sai_config_lock);

            break;
        case SFLOW_INGRESS_DISABLE:
        case SFLOW_EGRESS_DISABLE:
            if (get_user(len, &argp->len) < 0) {
                return -EFAULT;
            }

            len = min_t(size_t, len, IFNAME_SIZE - 1);
            if (copy_from_user(&ifname, &argp->ifname, len)) {
                return -EFAULT;
            }
            ifname[len] = '\0';

            mutex_lock(&sai_config_lock);
            list_for_each(tmp, &sflow_plist) {
                sdel = list_entry(tmp, struct sflow_config, list);

                if (!strcmp(sdel->ifname, ifname)) {
                    (cmd == SFLOW_INGRESS_DISABLE) ? (sdel->ing_rate = 0):(sdel->egr_rate = 0);
                    if ((sdel->ing_rate == 0) &&
                            (sdel->egr_rate == 0)) {
                        // remove if both are disabled
                        sflow_unregister_nf(sdel);
                        list_del(&sdel->list);
                        kfree(sdel);
                    }
                    mutex_unlock(&sai_config_lock);
                    return 0;
                }
            }
            mutex_unlock(&sai_config_lock);

            break;
        case SFLOW_FLUSH:
            mutex_lock(&sai_config_lock);
            sflow_config_free();
            mutex_unlock(&sai_config_lock);

            break;
        case VLAN_TAG_STRIP:
        case VLAN_TAG_KEEP:
            vadd = kmalloc(sizeof(*vadd), GFP_KERNEL);
            if (!vadd) {
                return -ENOMEM;
            }
            memset(vadd, 0, sizeof(*vadd));

            if (get_user(vadd->len, &argp->len) < 0) {
                kfree(vadd);
                return -EFAULT;
            }

            vadd->len = min_t(size_t, vadd->len, IFNAME_SIZE - 1);
            if (copy_from_user(&vadd->ifname, &argp->ifname, vadd->len)) {
                kfree(vadd);
                return -EFAULT;
            }
            vadd->ifname[vadd->len] = '\0';

            vadd->cmd = cmd;
            vadd->pvid = 0;
            if (cmd == VLAN_TAG_KEEP) {
                if (get_user(vadd->pvid, &argp->pvid) < 0) {
                    kfree(vadd);
                    return -EFAULT;
                }
            }

            mutex_lock(&sai_config_lock);
            list_for_each(tmp, &vlantag_plist) {
                vcur = list_entry(tmp, struct vlan_tag_config, list);
                if (!strcmp(vcur->ifname, vadd->ifname)) {
                    vcur->pvid = vadd->pvid;
                    vcur->cmd = vadd->cmd;
                    mutex_unlock(&sai_config_lock);
                    kfree(vadd);
                    return 0;
                }
            }

            list_add(&vadd->list, &vlantag_plist);

            err = vlantag_register_nf(vadd);
            if (err != 0) {
                return err;
            }
            mutex_unlock(&sai_config_lock);

            break;
        case VLAN_TAG_ORIGINAL:
            if (get_user(len, &argp->len) < 0) {
                return -EFAULT;
            }

            len = min_t(size_t, len, IFNAME_SIZE - 1);
            if (copy_from_user(&ifname, &argp->ifname, len)) {
                return -EFAULT;
            }
            ifname[len] = '\0';

            mutex_lock(&sai_config_lock);
            list_for_each(tmp, &vlantag_plist) {
                vdel = list_entry(tmp, struct vlan_tag_config, list);

                if (!strcmp(vdel->ifname, ifname)) {
                    vlantag_unregister_nf(vdel);
                    list_del(&vdel->list);
                    kfree(vdel);
                    mutex_unlock(&sai_config_lock);
                    return 0;
                }
            }
            mutex_unlock(&sai_config_lock);

            break;
        case VLAN_TAG_FLUSH:
            mutex_lock(&sai_config_lock);
            vlantag_config_free();
            mutex_unlock(&sai_config_lock);

            break;

        default:
            pr_err("sai_ioctl: invalid value\n");
            return -EINVAL;
    }

    return 0;
}

static struct file_operations sai_fops =
{
    .owner = THIS_MODULE,
    .open = sai_open,
    .release = sai_close,
    .unlocked_ioctl = sai_ioctl
};

static int sai_proc_show(struct seq_file *m, void *v)
{
    struct vlan_tag_config *vcur;
    struct sflow_config *scur;
    struct list_head *tmp = NULL;
    u32 cnt;

    seq_puts(m, "--------VLAN TAG INFO-------\n");
    seq_puts(m, "IFNAME           CMD      PVID\n");
    mutex_lock(&sai_config_lock);
    list_for_each(tmp, &vlantag_plist) {
        vcur = list_entry(tmp, struct vlan_tag_config, list);
        if (vcur->cmd == VLAN_TAG_STRIP) {
            seq_printf(m, "%-16s STRIP    --\n",
                    vcur->ifname);
        }
        else {
            seq_printf(m, "%-16s KEEP     %u\n",
                    vcur->ifname, vcur->pvid);
        }
    }
    mutex_unlock(&sai_config_lock);
    seq_printf(m, "\n\n");

    tmp = NULL;
    seq_puts(m, "---------SFLOW INFO---------\n");
    seq_puts(m, "IFNAME           INGRESS_RATE EGRESS_RATE\n");
    mutex_lock(&sai_config_lock);
    list_for_each(tmp, &sflow_plist) {
        scur = list_entry(tmp, struct sflow_config, list);
        seq_printf(m, "%-16s %-12u %-11u\n",
                scur->ifname, scur->ing_rate, scur->egr_rate);
    }

    seq_printf(m, "\n\n");
    cnt = atomic_read(&sflow_pkt_ing_cnt);
    seq_printf(m, "SFLOW INGRESS packet count: %u\n", cnt);
    cnt = atomic_read(&sflow_pkt_egr_cnt);
    seq_printf(m, "SFLOW EGRESS packet count: %u\n", cnt);
    mutex_unlock(&sai_config_lock);

    return 0;
}

static int sai_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, sai_proc_show, NULL);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,10,0)
static struct  proc_ops proc_fops =
{
    .proc_open		= sai_proc_open,
    .proc_read		= seq_read,
    .proc_lseek 	= seq_lseek,
    .proc_release	= single_release,
};
#else
static struct file_operations proc_fops =
{
    .open		= sai_proc_open,
    .read		= seq_read,
    .llseek		= seq_lseek,
    .release   	= single_release,
};
#endif

static int __init sai_init(void)
{
    int err = 0;
    struct psample_group *group;

    INIT_LIST_HEAD(&sflow_plist);
    INIT_LIST_HEAD(&vlantag_plist);

    err = register_chrdev(SAI_MAJOR, SAI_NAME, &sai_fops);
    if (err < 0) {
        pr_err("sai_init: unable to get major number\n");
        goto out;
    }

    major = err;

#if LINUX_VERSION_CODE < KERNEL_VERSION(6,4,0)
    sai_class = class_create(THIS_MODULE, SAI_NAME);
#else
    sai_class = class_create(SAI_NAME);
#endif
    if (IS_ERR(sai_class)) {
        err = PTR_ERR(sai_class);
        goto out_chrdev;
    }
    device_create(sai_class, NULL, MKDEV(major, 0), NULL,
            SAI_NAME);

    sai_proc = proc_create("mvsai_info", 0220, NULL, &proc_fops);

    if (!sai_proc) {
        pr_err("sai_init: proc entry creation failed\n");
        goto out_class;
    }

    group = psample_group_get(&init_net, psample_group_num);
    if (!group) {
        pr_err("sai_init: out of memory\n");
        err = -ENOMEM;
        goto out_proc;
    }
    rcu_assign_pointer(psample_group, group);

    register_netdevice_notifier(&sai_notifier);
    return 0;

out_proc:
    if (sai_proc) {
        remove_proc_entry("mvsai_info", NULL);
    }
out_class:
    device_destroy(sai_class, MKDEV(major, 0));
    class_destroy(sai_class);
out_chrdev:
    unregister_chrdev(major, SAI_NAME);
out:
    return err;
}

static void __exit sai_exit(void)
{
    struct psample_group *group;

    unregister_netdevice_notifier(&sai_notifier);
    mutex_lock(&sai_config_lock);
    vlantag_config_free();
    sflow_config_free();
    mutex_unlock(&sai_config_lock);

    group = rcu_dereference(psample_group);
    RCU_INIT_POINTER(psample_group, NULL);
    if (group) {
        psample_group_put(group);
    }

    if (sai_proc) {
        remove_proc_entry("mvsai_info", NULL);
    }

    device_destroy(sai_class, MKDEV(major, 0));
    class_destroy(sai_class);
    unregister_chrdev(major, SAI_NAME);
}

module_init(sai_init);
module_exit(sai_exit);

MODULE_AUTHOR("pnaregundi@marvell.com>");
MODULE_DESCRIPTION("Marvell Sai Module");
MODULE_LICENSE("GPL");
