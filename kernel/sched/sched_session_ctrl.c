// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Copyright (C) Huawei Technologies Co., Ltd. 2014-2021. All rights reserved.
 * Author: Huawei OS Kernel Lab
 *
 * This file is part of Session scheduler.
 *
 * This program is a free software, you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#define pr_fmt(fmt) "session_ctrl: " fmt

#include <linux/module.h>
#include <linux/compat.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/sched/task.h>
#include <linux/session_ctrl.h>
#include <linux/uaccess.h>

extern int sched_setsession(pid_t pid, unsigned int prio);

int session_ctrl_add(void __user *uarg)
{
	struct session_stat stat;
	int ret;

	if (uarg == NULL)
		return -EINVAL;

	if (copy_from_user(&stat, uarg, sizeof(struct session_stat))) {
		pr_err("%s, copy from user failed\n", __func__);
		return -EFAULT;
	}

	ret = sched_setsession(stat.pid, stat.prio);
	if (ret != 0)
		pr_err("set session error\n");

	return ret;
}

static long session_ctrl_ioctl(struct file *file, unsigned int cmd,
				unsigned long arg)
{
	void __user *uarg = (void __user *)(uintptr_t)arg;
	unsigned int func_id = _IOC_NR(cmd);
	long ret;

	if (uarg == NULL) {
		pr_err("%s: invalid user uarg\n", __func__);
		return -EINVAL;
	}

	if (_IOC_TYPE(cmd) != SESSION_CTRL_MAGIC) {
		pr_err("%s: SESSION_CTRL_MAGIC fail, TYPE=%d\n",
			__func__, _IOC_TYPE(cmd));
		return -EINVAL;
	}

	if (func_id >= SESSION_CTRL_MAX_NR) {
		pr_err("%s: SESSION_CTRL_MAX_NR fail, _IOC_NR(cmd)=%d, MAX_NR=%d\n",
			__func__, _IOC_NR(cmd), SESSION_CTRL_MAX_NR);
		return -EINVAL;
	}

	switch (cmd) {
	case SESSION_CTRL_ADD:
		return session_ctrl_add(uarg);

	default:
		pr_err("cmd error, here is deault, cmd = %u\n", cmd);
		return -EINVAL;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long session_ctrl_compact_ioctl(struct file *file,
					unsigned int cmd, unsigned long arg)
{
	return session_ctrl_ioctl(file, cmd, (unsigned long)(compat_ptr(arg)));
}
#endif

static int session_ctrl_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int session_ctrl_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations session_ctrl_fops = {
	.owner = THIS_MODULE,
#ifdef CONFIG_COMPAT
	.compat_ioctl = session_ctrl_compact_ioctl,
#endif
	.unlocked_ioctl = session_ctrl_ioctl,
	.open = session_ctrl_open,
	.release = session_ctrl_release,
};

static struct miscdevice session_ctrl_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "session_ctrl",
	.fops = &session_ctrl_fops,
};

static int __init session_ctrl_dev_init(void)
{
	int err;

	err = misc_register(&session_ctrl_device);
	if (err != 0)
		return err;

	return 0;
}

static void __exit session_ctrl_dev_exit(void)
{
	misc_deregister(&session_ctrl_device);
}

module_init(session_ctrl_dev_init);
module_exit(session_ctrl_dev_exit);
