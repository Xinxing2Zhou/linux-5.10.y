/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/vmalloc.h>
#include <asm/pgtable.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <asm/page.h>
#include <linux/spinlock.h>
#include <linux/kernel.h>
#include <linux/ioctl.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/mm_types.h>
#include <linux/mmu_context.h>
#include <asm/tlbflush.h>
#include <linux/uaccess.h>

#include "vdma.h"

#define hidmac_error(s...) do { \
	pr_err("hidmac:%s:%d: ", __func__, __LINE__); \
	printk(s); \
	printk("\n"); \
} while (0)

static long vdma_ioctl(struct file *filep, unsigned int cmd,
			  unsigned long arg)
{
	long ret;
	struct dmac_user_para para;

	if (copy_from_user((void *)&para, (void *)(uintptr_t)arg, sizeof(para)))
		return -EINVAL;

	switch (cmd) {
	case VDMA_DATA_CMD:
		ret = vdma_m2m_copy((void *)para.dst, (void *)para.src, para.size);
		break;
	default:
		hidmac_error("unknown command :%x\n", cmd);
		ret = -1;
		break;
	}

	return ret;
}

static int vdma_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int vdma_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations vdma_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = vdma_ioctl,
	.open = vdma_open,
	.release = vdma_release,
};

static struct miscdevice vdma_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "vdma",
	.fops = &vdma_fops,
};

static int __init vdma_init(void)
{
	int ret;

	ret = misc_register(&vdma_misc_device);

	return ret;
}

static void __exit vdma_exit(void)
{
	misc_deregister(&vdma_misc_device);
}

module_init(vdma_init);
module_exit(vdma_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Vendor VDMA MISC Driver");
