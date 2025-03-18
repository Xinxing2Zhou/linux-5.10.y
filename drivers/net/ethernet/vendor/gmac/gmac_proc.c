/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#include <linux/proc_fs.h>

#include "gmac_pm.h"
#include "gmac_proc.h"

/* debug code */
int set_suspend(int eth_n)
{
	return 0;
}

/* debug code */
int set_resume(int eth_n)
{
	return 0;
}

static int hw_states_read(struct seq_file *m, void *v)
{
	return 0;
}

static struct proc_dir_entry *gmac_proc_root;

static int proc_open_hw_states_read(struct inode *inode, struct file *file)
{
	return single_open(file, hw_states_read, PDE_DATA(inode));
}

static struct proc_file {
	char *name;
	const struct proc_ops ops;

} proc_file[] = {
	{
		.name = "hw_stats",
		.ops = {
			.proc_open           = proc_open_hw_states_read,
			.proc_read           = seq_read,
			.proc_lseek         = seq_lseek,
			.proc_release        = single_release,
		},
	}
};

/*
 * /proc/gmac/
 *	|---hw_stats
 *	|---skb_pools
 */
void gmac_proc_create(void)
{
	struct proc_dir_entry *entry = NULL;
	int i;

	gmac_proc_root = proc_mkdir("gmac", NULL);
	if (gmac_proc_root == NULL)
		return;

	for (i = 0; i < ARRAY_SIZE(proc_file); i++) {
		entry = proc_create(proc_file[i].name, 0, gmac_proc_root,
				    &proc_file[i].ops);
		if (entry == NULL)
			pr_err("failed to create %s\n", proc_file[i].name);
	}
}

void gmac_proc_destroy(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(proc_file); i++)
		remove_proc_entry(proc_file[i].name, gmac_proc_root);

	remove_proc_entry("gmac", NULL);
}
