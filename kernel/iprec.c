/*
 * iprec.c:  Record any ip io transfer process to
 * system memory. Reduce printk log latency
 * or ftrace complexity.
 *
 *
 */
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/securec.h>

#include <linux/iprec.h>


#define SEC_FOUR_DIGIT 10000
#define USEC_TO_MSEC 1000
#define TIME_STAMP_LEN 16

static struct proc_dir_entry *iprec_proc;
static int iprec_init = 0;
static int iprec_proc_cnt = 0;
static char tim[TIME_STAMP_LEN] = {0};
extern void iprec_inc(void);

static char **rec_pool = NULL;
static int rec_lines = IPREC_DEFAULT_LINE;
static int rec_delay = IPREC_DEFAULT_DLAY;
static int rem_delay = -1;
static int rec_lock = 0;
static int rec_line = 0;
static int rec_ovwr = 0;


static spinlock_t rec_spinlock;

spinlock_t *iprec_spinlock(void)
{
	return &rec_spinlock;
}
EXPORT_SYMBOL_GPL(iprec_spinlock);

void iprec_slock(void)
{
	iprec("_SLOCK_");
	if (rec_delay == 0)
		rec_lock = 1;
	else
		rem_delay = rec_delay;
}
EXPORT_SYMBOL_GPL(iprec_slock);

int iprec_glock(void)
{
	return rec_lock;
}
EXPORT_SYMBOL_GPL(iprec_glock);

char *iprec_pool(void)
{
	return (rec_pool[rec_line % rec_lines]);
}
EXPORT_SYMBOL_GPL(iprec_pool);

char *iprec_tm(void)
{
	int ret;
	struct timespec64 tv;
	long s, ms, us;
	
	ktime_get_boottime_ts64(&tv);

	s = tv.tv_sec % SEC_FOUR_DIGIT;
	ms = tv.tv_nsec / USEC_TO_MSEC / USEC_TO_MSEC;
	us = tv.tv_nsec % (USEC_TO_MSEC * USEC_TO_MSEC) / USEC_TO_MSEC;

	ret = sprintf_s(tim, TIME_STAMP_LEN, "[%04ld.%03ld%03ld]", s, ms, us);

	if (ret < 0) {
		pr_err("[%s line:%d] sprintf_s failed\n", __func__, __LINE__);
	}

	return tim;
}
EXPORT_SYMBOL_GPL(iprec_tm);

void iprec_inc(void)
{
	rec_line++;
	rec_line %= rec_lines;
	if (rec_line == 0)
		rec_ovwr++;

	if (rec_delay) {
		// check if lock is actived!
		if (rem_delay > 0) {
			rem_delay--;
		} else if (rem_delay == 0) { /* set lock */
			rec_lock = 1;
			rem_delay = -1; // lock is unactived!
		}
	}
}
EXPORT_SYMBOL_GPL(iprec_inc);

static void iprec_show(struct seq_file *s)
{
	int i;
	rec_lock = 1;

	seq_printf(s, "|__ ov: %d __|__ delay: %d __|\n\n", rec_ovwr, rec_delay);

	if (rec_ovwr == 0) {
		for (i = 0; i < rec_line; i++) {
			seq_printf(s, " %04d %s\n", (i + 1), rec_pool[i]);
		}
	} else {
		for (i = rec_line; i < rec_lines; i++) {
			seq_printf(s, " %04d %s\n", (i - rec_line + 1), rec_pool[i]);
		}

		for (i = 0; i < rec_line; i++) {
			seq_printf(s, " %04d %s\n", (i + rec_lines - rec_line + 1),
				rec_pool[i]);
		}
	}

	seq_printf(s, "\n|__ ov: %d __|__ delay: %d __|\n", rec_ovwr, rec_delay);
}

static void iprec_pool_free(char **pool)
{
	int i;

	if (!pool)
		return;

	for (i = 0; i < rec_lines; i++) {
		if (pool[i])
			kfree(pool[i]);
	}
	kfree(pool);
}

static char **iprec_pool_alloc(int lines)
{
	int i;
	char *mem = NULL;
	char **pool = NULL;

	if (lines <= 0)
		return NULL;

	pool = kmalloc(lines * sizeof(char *), GFP_KERNEL);
	if (pool == NULL) {
		pr_err("%s: no enough mem iprec %d\n", __func__, lines);
		return NULL;
	}

	for (i = 0; i < lines; i++) {
		mem = kmalloc(IPREC_STR_LEN, GFP_KERNEL);
		if (mem == NULL)
			goto free_pool;
		mem[0] = 0;
		pool[i] = mem;
	}

	return pool;

free_pool:
	iprec_pool_free(pool);

	return NULL;
}

static int iprec_dump_seq_show(struct seq_file *s, void *v)
{
	iprec_show(s);
	return 0;
}

static int iprec_proc_dump_open(struct inode *inode, struct file *file)
{
	return single_open(file, iprec_dump_seq_show, PDE_DATA(inode));
}

static const struct proc_ops iprec_dump_ops = {
	.proc_open = iprec_proc_dump_open,
	.proc_read = seq_read,
	.proc_release = single_release,
};

static int iprec_line_seq_show(struct seq_file *s, void *v)
{
	seq_printf(s, "%d (N x %dB)\n", rec_lines, IPREC_STR_LEN);
	return 0;
}

static int iprec_proc_line_open(struct inode *inode, struct file *file)
{
	return single_open(file, iprec_line_seq_show, PDE_DATA(inode));
}

static ssize_t iprec_proc_line_write(struct file *file,
	const char __user *buffer, size_t count, loff_t *pos)
{
	int err;
	unsigned long flags;
	unsigned int val = 0;
	char **pool = NULL;

	err = kstrtouint_from_user(buffer, count, 0, &val);
	if (err)
		return err;

	spin_lock_irqsave(iprec_spinlock(), flags);

	pool = iprec_pool_alloc(val);
	if (pool) {
		iprec_pool_free(rec_pool);
		rec_pool = pool;
		rec_lines = val;
		rec_line = 0; // reset
	}
	spin_unlock_irqrestore(iprec_spinlock(), flags);
	return count;
}

static const struct proc_ops iprec_line_ops = {
	.proc_open = iprec_proc_line_open,
	.proc_read = seq_read,
	.proc_write = iprec_proc_line_write,
	.proc_release = single_release,
};

static int iprec_lock_seq_show(struct seq_file *s, void *v)
{
	seq_printf(s, "%d\n", rec_lock);
	return 0;
}

static int iprec_proc_lock_open(struct inode *inode, struct file *file)
{
	return single_open(file, iprec_lock_seq_show, PDE_DATA(inode));
}

static ssize_t iprec_proc_lock_write(struct file *file,
	const char __user *buffer, size_t count, loff_t *pos)
{
	int err;
	unsigned int val = 0;

	err = kstrtouint_from_user(buffer, count, 0, &val);
	if (err)
		return err;

	if (val) {
		iprec_slock();
	} else {
		rec_lock = 0;
		rem_delay = -1;
	}

	return count;
}

static const struct proc_ops iprec_lock_ops = {
	.proc_open = iprec_proc_lock_open,
	.proc_read = seq_read,
	.proc_write = iprec_proc_lock_write,
	.proc_release = single_release,
};

static int iprec_delay_seq_show(struct seq_file *s, void *v)
{
	seq_printf(s, "%d\n", rec_delay);
	return 0;
}

static int iprec_proc_delay_open(struct inode *inode, struct file *file)
{
	return single_open(file, iprec_delay_seq_show, PDE_DATA(inode));
}

static ssize_t iprec_proc_delay_write(struct file *file,
	const char __user *buffer, size_t count, loff_t *pos)
{
	int err;
	unsigned int val = 0;

	err = kstrtouint_from_user(buffer, count, 0, &val);
	if (err)
		return err;
	if (val)
		rec_delay = (val < rec_lines) ? val : (rec_lines - 1);

	return count;
}

static const struct proc_ops iprec_delay_ops = {
	.proc_open = iprec_proc_delay_open,
	.proc_read = seq_read,
	.proc_write = iprec_proc_delay_write,
	.proc_release = single_release,
};

static int iprec_lines__setup(char *line)
{
	return kstrtouint(line, 10, &rec_lines); /* 10 decimal */
}
__setup("iprec.line=", iprec_lines__setup);

static struct proc_dir_entry *iprec_create_proc_node(const char *name,
	struct proc_dir_entry *proc_entry, const struct proc_ops *fileops
	/* const struct file_operations *fileops */)
{
	struct proc_dir_entry *ret = NULL;
	ret = proc_create_data(name, 0, proc_entry, fileops, NULL);
	if (ret == NULL)
		pr_err("[iprec] failed to create proc file %s\n", name);

	return ret;
}

static int __init iprec_proc_init(void)
{
	char **pool = NULL;
	struct proc_dir_entry *proc_entry = NULL;

	if (iprec_init == 0) {
		proc_entry = proc_mkdir(IPREC, NULL);
		if (proc_entry == NULL) {
			pr_err("%s: failed to create proc file %s\n", __func__, IPREC);
			return -ENOMEM;
		}
		iprec_proc = proc_entry;

		if (iprec_create_proc_node(IPREC_DUMP, iprec_proc, &iprec_dump_ops) ==
			NULL) {
			pr_err("%s: failed to create proc file %s\n", __func__, IPREC_DUMP);
			goto remove_entry;
		}

		if (iprec_create_proc_node(IPREC_LINE, iprec_proc, &iprec_line_ops) ==
			NULL) {
			pr_err("%s: failed to create proc file %s\n", __func__, IPREC_LINE);
			goto remove_dump;
		}

		if (iprec_create_proc_node(IPREC_LOCK, iprec_proc, &iprec_lock_ops) ==
			NULL) {
			pr_err("%s: failed to create proc file %s\n", __func__, IPREC_LOCK);
			goto remove_line;
		}

		if (iprec_create_proc_node(IPREC_DLAY, iprec_proc, &iprec_delay_ops) ==
			NULL) {
			pr_err("%s: failed to create proc file %s\n", __func__, IPREC_DLAY);
			goto remove_lock;
		}

		pool = iprec_pool_alloc(rec_lines);
		if (pool == NULL)
			goto remove_delay;
		rec_pool = pool;
		spin_lock_init(&rec_spinlock);
		iprec_init = 1;
	}
	iprec_proc_cnt++;
	return 0;

remove_delay:
	remove_proc_entry(IPREC_DLAY, iprec_proc);
remove_lock:
	remove_proc_entry(IPREC_LOCK, iprec_proc);
remove_line:
	remove_proc_entry(IPREC_LINE, iprec_proc);
remove_dump:
	remove_proc_entry(IPREC_DUMP, iprec_proc);
remove_entry:
	remove_proc_entry(IPREC, NULL);
	return -1;
}

static void __exit iprec_proc_shutdown(void)
{
	if (iprec_init) {
		if (iprec_proc_cnt == 0) {
			remove_proc_entry(IPREC_DUMP, iprec_proc);
			remove_proc_entry(IPREC_DLAY, iprec_proc);
			remove_proc_entry(IPREC_LINE, iprec_proc);
			remove_proc_entry(IPREC_LOCK, iprec_proc);
			remove_proc_entry(IPREC, NULL);
			if (rec_pool) {
				iprec_pool_free(rec_pool);
				rec_pool = NULL;
			}
			iprec_proc = NULL;
			iprec_init = 0;
		}
		iprec_proc_cnt--;
	}

	return;
}
subsys_initcall(iprec_proc_init);
module_exit(iprec_proc_shutdown);

MODULE_LICENSE("GPL");
