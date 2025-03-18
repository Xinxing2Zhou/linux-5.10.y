 /*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#include <core.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/usb.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/proc_fs.h>

#include "proc.h"

#define MODE_BUF_LEN 32

static ssize_t wing_usb_mode_write(struct file *file,
	const char __user *ubuf, size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct wing_usb *wusb = s->private;
	struct wing_usb_event usb_event = {0};

	char buf[MODE_BUF_LEN] = {0};

	if (ubuf == NULL)
		return -EFAULT;

	if (copy_from_user(buf, ubuf, min_t(size_t, sizeof(buf) - 1, count)))
		return -EFAULT;

	if (strncmp(buf, "device", strlen("device")) == 0) {
		usb_event.type = SWITCH_TO_DEVICE;
	} else if (strncmp(buf, "host", strlen("host")) == 0) {
		usb_event.type = SWITCH_TO_HOST;
	} else {
		usb_event.type = NONE_EVENT;
		wing_usb_err("input event type error\n");
		return -EINVAL;
	}

	usb_event.ctrl_id = wusb->id;
	wing_usb_queue_event(&usb_event, wusb);

	wing_usb_dbg("write %s\n", buf);

	return count;
}

static int wing_usb_mode_show(struct seq_file *s, void *v)
{
	struct wing_usb *wusb = s->private;
	unsigned long flags;
	u32 reg;

	spin_lock_irqsave(&wusb->event_lock, flags);
	reg = readl(wusb->ctrl_base + DWC3_GCTL);
	spin_unlock_irqrestore(&wusb->event_lock, flags);
	switch (DWC3_GCTL_PRTCAP(reg)) {
	case DWC3_GCTL_PRTCAP_HOST:
		seq_printf(s, "host\n");
		wusb->state = WING_USB_STATE_HOST;
		break;
	case DWC3_GCTL_PRTCAP_DEVICE:
		seq_printf(s, "device\n");
		wusb->state = WING_USB_STATE_DEVICE;
		break;
	case DWC3_GCTL_PRTCAP_OTG:
		seq_printf(s, "otg\n");
		break;
	default:
		seq_printf(s, "UNKNOWN %08x\n", DWC3_GCTL_PRTCAP(reg));
	}

	return 0;
}

static int wing_usb_mode_open(struct inode *inode, struct file *file)
{
	return single_open(file, wing_usb_mode_show, PDE_DATA(inode));
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
static const struct proc_ops g_wing_usb_proc_mode_ops = {
	.proc_open		= wing_usb_mode_open,
	.proc_write		= wing_usb_mode_write,
	.proc_read		= seq_read,
	.proc_release		= single_release,
};

#else
static const struct file_operations g_wing_usb_proc_mode_ops = {
	.open		= wing_usb_mode_open,
	.write		= wing_usb_mode_write,
	.read		= seq_read,
	.release	= single_release,
};

#endif

int wing_usb_create_proc_entry(struct device *dev, struct wing_usb *wusb)
{
	struct proc_dir_entry *proc_entry = NULL;

	wing_usb_dbg("+\n");

	if (wusb == NULL)
		return -EINVAL;

	proc_entry = proc_mkdir(dev_name(dev), NULL);
	if (proc_entry == NULL) {
		wing_usb_err("failed to create proc file\n");
		return -ENOMEM;
	}

	wusb->proc_entry = proc_entry;

	if (proc_create_data("mode", S_IRUGO | S_IWUSR, proc_entry,
		&g_wing_usb_proc_mode_ops, wusb) == NULL) {
		wing_usb_err("Failed to create proc file mode \n");
		goto remove_entry;
	}

	wing_usb_dbg("-\n");
	return 0;

remove_entry:
	remove_proc_entry(dev_name(dev), NULL);
	wusb->proc_entry = NULL;

	return -ENOMEM;
}

void wing_usb_remove_proc_entry(struct device *dev, struct wing_usb *wusb)
{
	if (wusb->proc_entry == NULL)
		return;

	remove_proc_entry("mode", wusb->proc_entry);
	remove_proc_entry(dev_name(dev), NULL);

	wusb->proc_entry = NULL;
}

