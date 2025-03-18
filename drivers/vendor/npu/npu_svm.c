/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#include <asm/esr.h>
#include <asm/mmu_context.h>

#include <linux/version.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/miscdevice.h>
#include <linux/mman.h>
#include <linux/mmu_notifier.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/ptrace.h>
#include <linux/security.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/sched.h>
#include <linux/hugetlb.h>
#include <linux/sched/mm.h>
#include <linux/fs.h>

#include "linux/vendor/sva_ext.h"

#define SVM_DEVICE_NAME         "svm"
#define ASID_SHIFT          48
#define CORE_SID            0 /* for core sid register */

#define SVM_IOCTL_PROCESS_BIND      0xffff
#define SVM_IOCTL_PAGE_TABLE_SYNC   0xfffd

struct core_device {
	struct device dev;
	struct iommu_group *group;
	struct iommu_domain *domain;
	u8 smmu_bypass;
	struct list_head entry;
};

struct svm_device {
	unsigned long long id;
	struct miscdevice miscdev;
	struct device *dev;
	phys_addr_t l2buff;
	unsigned long l2size;
};

struct svm_bind_process {
	pid_t vpid;
	u64 ttbr;
	u64 tcr;
	int pasid;
#define SVM_BIND_PID		(1 << 0)
	u32 flags;
};

struct svm_pg_sync_para {
	u64 vaddr;
	u32 len;
};

/*
 * svm_process is released in svm_notifier_release() when mm refcnt
 * goes down to 0. We should access svm_process only in the context
 * where mm_struct is valid, which means wes should always get mm
 * refcnt first (unless we are operating on current task).
 */
struct svm_process {
	struct pid *pid;
	struct mm_struct *mm;
	unsigned long asid;
	struct rb_node rb_node;
	struct mmu_notifier notifier;
	/* For postponed release */
	struct rcu_head rcu;
	int pasid;
	struct mutex mutex;
	struct svm_device *sdev;
};

typedef void (*smmu_clk_live_func)(void);

static smmu_clk_live_func g_smmu_clk_live_enter = NULL;
static smmu_clk_live_func g_smmu_clk_live_exit = NULL;

#define SVM_DEV_MAX 2
static struct rb_root svm_process_root[SVM_DEV_MAX] = {RB_ROOT, RB_ROOT};

static struct mutex svm_process_mutex;

static DECLARE_RWSEM(svm_sem);

static int probe_index = 0;

static void *npu_dts_sys_peri = NULL;
#define NPU_SVM_DEV_NAME	  "svm_npu"
#define NPU_SMMU_DEV_NAME     "smmu_npu"
#define NPU_CRG_NAME          "npu_crg_6560"

#if defined(CONFIG_ARCH_SD3491V100)
#define SVP_NPU_SVM_DEV_NAME      "svm_svp_npu"
#define SVP_NPU_SMMU_DEV_NAME     "smmu_svp_npu"
#define SVP_NPU_CRG_NAME          "svp_npu_crg_6592"

#elif defined(CONFIG_ARCH_HI3519DV500_FAMILY)
#define SVP_NPU_SVM_DEV_NAME	  "svm_pqp"
#define SVP_NPU_SMMU_DEV_NAME     "smmu_pqp"
#define SVP_NPU_CRG_NAME          "pqp_crg_6592"
#endif

#define SVM_DEV_NAME_LEN 64
#define CRG_NAME_LEN 16
#define CLK_EN_BIT 4

struct svm_dev_wl_mng {
	char   svm_dev_name[SVM_DEV_NAME_LEN];
	char   smmu_dev_name[SVM_DEV_NAME_LEN];
	char   crg_name[CRG_NAME_LEN];
	int    crg_offset;
	bool   is_inited;
	bool   is_suspend;
	void  *dev;
};
static struct mutex svm_dev_pm_mutex;

static struct svm_dev_wl_mng svm_dev_white_list[SVM_DEV_MAX] = {
		{ NPU_SVM_DEV_NAME, NPU_SMMU_DEV_NAME, NPU_CRG_NAME, false, false, NULL },
		{ SVP_NPU_SVM_DEV_NAME, SVP_NPU_SMMU_DEV_NAME, SVP_NPU_CRG_NAME, false, false, NULL }
};

static char *svm_cmd_to_string(unsigned int cmd)
{
	switch (cmd) {
	case SVM_IOCTL_PROCESS_BIND:
		return "bind";
	case SVM_IOCTL_PAGE_TABLE_SYNC:
		return "sync page table";
	default:
			return "unsupported";
	}
}

static int svm_device_get_smmu_devno(struct svm_device *sdev)
{
	int i;
	const char *device_name = dev_name(sdev->dev);

	if (device_name == NULL)
		return -1;

	for (i = 0; i < SVM_DEV_MAX; i++) {
		if (strnstr(device_name, svm_dev_white_list[i].svm_dev_name, SVM_DEV_NAME_LEN) != NULL)
			return i;
	}
	return -1;
}

static struct svm_process *find_svm_process(unsigned long asid, int smmu_devid)
{
	struct rb_node *node = svm_process_root[smmu_devid].rb_node;

	while (node != NULL) {
		struct svm_process *process = NULL;

		process = rb_entry(node, struct svm_process, rb_node);
		if (asid < process->asid) {
			node = node->rb_left;
		} else if (asid > process->asid) {
			node = node->rb_right;
		} else {
			return process;
		}
	}

	return NULL;
}

static void insert_svm_process(struct svm_process *process, int smmu_devid)
{
	struct rb_node **p = &svm_process_root[smmu_devid].rb_node;
	struct rb_node *parent = NULL;

	while (*p) {
		struct svm_process *tmp_process = NULL;

		parent = *p;
		tmp_process = rb_entry(parent, struct svm_process, rb_node);
		if (process->asid < tmp_process->asid) {
			p = &(*p)->rb_left;
		} else if (process->asid > tmp_process->asid) {
			p = &(*p)->rb_right;
		} else {
			WARN_ON_ONCE("asid already in the tree");
			return;
		}
	}

	rb_link_node(&process->rb_node, parent, p);
	rb_insert_color(&process->rb_node, &svm_process_root[smmu_devid]);
}

static void delete_svm_process(struct svm_process *process)
{
	int smmu_devid;

	smmu_devid = svm_device_get_smmu_devno(process->sdev);
	if (smmu_devid < 0)
		pr_err("fail to get smmu dev number\n");

	rb_erase(&process->rb_node, &svm_process_root[smmu_devid]);
	RB_CLEAR_NODE(&process->rb_node);
}

struct bus_type svm_bus_type = {
	.name		= "svm-bus",
};

static inline struct core_device *to_core_device(struct device *d)
{
	return container_of(d, struct core_device, dev);
}

static int svm_unbind_core(struct device *dev, void *data)
{
	struct svm_process *process = data;
	struct core_device *cdev = to_core_device(dev);

	if (cdev->smmu_bypass)
		return 0;

	iommu_sva_unbind_device(&cdev->dev, process->pasid);
	return 0;
}

static int svm_bind_core(struct device *dev, void *data)
{
	int err;
	struct task_struct *task = NULL;
	struct svm_process *process = data;
	struct core_device *cdev = to_core_device(dev);

	if (cdev->smmu_bypass)
		return 0;

	task = get_pid_task(process->pid, PIDTYPE_PID);
	if (task == NULL) {
		pr_err("failed to get task_struct\n");
		return -ESRCH;
	}

	err = iommu_sva_bind_device(&cdev->dev, task->mm, &process->pasid, IOMMU_SVA_FEAT_IOPF, NULL);
	if (err)
		pr_err("failed to get the pasid\n");

	put_task_struct(task);

	return err;
}

static void svm_bind_cores(struct svm_process *process)
{
	mutex_lock(&svm_dev_pm_mutex);
	device_for_each_child(process->sdev->dev, process, svm_bind_core);
	mutex_unlock(&svm_dev_pm_mutex);
}

static void svm_unbind_cores(struct svm_process *process)
{
	mutex_lock(&svm_dev_pm_mutex);
	device_for_each_child(process->sdev->dev, process, svm_unbind_core);
	mutex_unlock(&svm_dev_pm_mutex);
}

static void cdev_device_release(struct device *dev)
{
	struct core_device *cdev = to_core_device(dev);

	kfree(cdev);
}

static int svm_remove_core(struct device *dev, void *data)
{
	int err;
	struct core_device *cdev = to_core_device(dev);

	if (!cdev->smmu_bypass) {
		iommu_detach_group(cdev->domain, cdev->group);
		iommu_group_put(cdev->group);
		iommu_domain_free(cdev->domain);
	}

	err = iommu_sva_device_shutdown(&cdev->dev);
	if (err)
		dev_err(&cdev->dev, "failed to shutdown sva device\n");

	device_unregister(&cdev->dev);

	return 0;
}

static int svm_register_device(struct svm_device *sdev, struct device_node *np, struct core_device **pcdev)
{
	int err;
	char *name = NULL;
	struct core_device *cdev = NULL;

	name = devm_kasprintf(sdev->dev, GFP_KERNEL, "svm%llu_%s", sdev->id, np->name);
	if (name == NULL)
		return -ENOMEM;

	cdev = kzalloc(sizeof(*cdev), GFP_KERNEL);
	if (cdev == NULL)
		return -ENOMEM;

	cdev->dev.of_node = np;
	cdev->dev.parent = sdev->dev;
	cdev->dev.bus = &svm_bus_type;
	cdev->dev.release = cdev_device_release;
	cdev->smmu_bypass = of_property_read_bool(np, "vendor,smmu_bypass");
	dev_set_name(&cdev->dev, "%s", name);

	err = device_register(&cdev->dev);
	if (err) {
		dev_info(&cdev->dev, "core_device register failed\n");
		kfree(cdev);
		return err;
	}
	*pcdev = cdev;
	return 0;
}

static int svm_iommu_attach_group(struct svm_device *sdev, struct core_device *cdev)
{
	int err;

	cdev->group = iommu_group_get(&cdev->dev);
	if (IS_ERR_OR_NULL(cdev->group)) {
		dev_err(&cdev->dev, "smmu is not right configured\n");
		return -ENXIO;
	}

	cdev->domain = iommu_domain_alloc(sdev->dev->bus);
	if (cdev->domain == NULL) {
		dev_err(&cdev->dev, "failed to alloc domain\n");
		iommu_group_put(cdev->group);
		return -ENOMEM;
	}

	err = iommu_attach_group(cdev->domain, cdev->group);
	if (err) {
		dev_err(&cdev->dev, "failed group to domain\n");
		iommu_group_put(cdev->group);
		iommu_domain_free(cdev->domain);
		return err;
	}

	return 0;
}

static int svm_of_add_core(struct svm_device *sdev, struct device_node *np)
{
	int err;
	struct resource res;
	struct core_device *cdev = NULL;

	err = svm_register_device(sdev, np, &cdev);
	if (err) {
		dev_info(&cdev->dev, "fail to register svm device\n");
		return err;
	}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 18, 0))
	err = of_dma_configure(&cdev->dev, np, true);
#else
	err = of_dma_configure(&cdev->dev, np);
#endif
	if (err) {
		dev_dbg(&cdev->dev, "of_dma_configure failed\n");
		goto err_unregister_dev;
	}

	err = of_address_to_resource(np, 0, &res);
	if (err)
		dev_info(&cdev->dev, "no reg, FW should install the sid\n");

	/* If core device is smmu bypass, request direct map. */
	if (cdev->smmu_bypass) {
		err = iommu_request_dm_for_dev(&cdev->dev);
		if (err)
			goto err_unregister_dev;
		return 0;
	}

	err = svm_iommu_attach_group(sdev, cdev);
	if (err) {
		dev_err(&cdev->dev, "failed to init sva device\n");
		goto err_unregister_dev;
	}

	err = iommu_sva_device_init(&cdev->dev, IOMMU_SVA_FEAT_IOPF, UINT_MAX, 0);
	if (err) {
		dev_err(&cdev->dev, "failed to init sva device\n");
		goto err_detach_group;
	}

	return 0;
err_detach_group:
	iommu_detach_group(cdev->domain, cdev->group);
	iommu_domain_free(cdev->domain);
	iommu_group_put(cdev->group);
err_unregister_dev:
	device_unregister(&cdev->dev);
	return err;
}

static void svm_process_free(struct rcu_head *rcu)
{
	struct svm_process *process = NULL;

	process = container_of(rcu, struct svm_process, rcu);
	mm_context_put(process->mm);
	kfree(process);
}

static void svm_process_release(struct svm_process *process)
{
	delete_svm_process(process);
	put_pid(process->pid);

	/*
	 * If we're being released from process exit, the notifier callback
	 * ->release has already been called. Otherwise we don't need to go
	 * through there, the process isn't attached to anything anymore. Hence
	 * no_release.
	 */
	mmu_notifier_unregister_no_release(&process->notifier, process->mm);

	/*
	 * We can't free the structure here, because ->release might be
	 * attempting to grab it concurrently. And in the other case, if the
	 * structure is being released from within ->release, then
	 * __mmu_notifier_release expects to still have a valid mn when
	 * returning. So free the structure when it's safe, after the RCU grace
	 * period elapsed.
	 */
	mmu_notifier_call_srcu(&process->rcu, svm_process_free);
}

static void svm_notifier_release(struct mmu_notifier *mn, struct mm_struct *mm)
{
	struct svm_process *process = NULL;
	process = container_of(mn, struct svm_process, notifier);

	svm_smmu_clk_live_enter();
	svm_unbind_cores(process);
	svm_smmu_clk_live_exit();

	mutex_lock(&svm_process_mutex);
	svm_process_release(process);
	mutex_unlock(&svm_process_mutex);
}

/*
 * Device CPU have the ability of DVM, which means when control CPU flush
 * TLB, it will notify the device CPU by hardware instead of mmu_notifier.
 */
static struct mmu_notifier_ops svm_process_mmu_notifier = {
	.release	= svm_notifier_release,
};

static struct svm_process *svm_process_alloc(struct svm_device *sdev, struct pid *pid,
											 struct mm_struct *mm, unsigned long asid)
{
	struct svm_process *process = kzalloc(sizeof(*process), GFP_ATOMIC);
	if (process == NULL)
		return ERR_PTR(-ENOMEM);

	process->sdev = sdev;
	process->pid = pid;
	process->mm = mm;
	process->asid = asid;
	mutex_init(&process->mutex);
	process->notifier.ops = &svm_process_mmu_notifier;

	return process;
}

static int get_task_info(struct task_struct *task, struct pid **ppid, struct mm_struct **pmm, unsigned long *pasid)
{
	unsigned long asid;
	struct pid *pid = NULL;
	struct mm_struct *mm = NULL;

	pid = get_task_pid(task, PIDTYPE_PID);
	if (pid == NULL)
		return -EINVAL;

	mm = get_task_mm(task);
	if (mm == NULL) {
		put_pid(pid);
		return -EINVAL;
	}

	asid = mm_context_get(mm);
	if (!asid) {
		mmput(mm);
		put_pid(pid);
		return -ENOSPC;
	}

	*ppid = pid;
	*pmm = mm;
	*pasid = asid;
	return 0;
}

static int svm_process_bind(struct task_struct *task, struct svm_device *sdev,
							u64 *ttbr, u64 *tcr, int *pasid)
{
	int err;
	unsigned long asid;
	struct pid *pid = NULL;
	struct svm_process *process = NULL;
	struct mm_struct *mm = NULL;
	int smmu_devid = svm_device_get_smmu_devno(sdev);
	if ((ttbr == NULL) || (tcr == NULL) || (pasid == NULL) || smmu_devid < 0)
		return -EINVAL;

	err = get_task_info(task, &pid, &mm, &asid);
	if (err != 0)
		return err;

	/* If a svm_process already exists, use it */
	mutex_lock(&svm_process_mutex);
	process = find_svm_process(asid, smmu_devid);
	if (process == NULL) {
		process = svm_process_alloc(sdev, pid, mm, asid);
		if (IS_ERR(process)) {
			err = PTR_ERR(process);
			mutex_unlock(&svm_process_mutex);
			goto err_put_mm_context;
		}

		err = mmu_notifier_register(&process->notifier, mm);
		if (err) {
			mutex_unlock(&svm_process_mutex);
			goto err_free_svm_process;
		}

		insert_svm_process(process, smmu_devid);
		svm_bind_cores(process);
		mutex_unlock(&svm_process_mutex);
	} else {
		mutex_unlock(&svm_process_mutex);
		mm_context_put(mm);
		put_pid(pid);
	}

	*ttbr = virt_to_phys(mm->pgd) | (asid << ASID_SHIFT);
	*tcr  = read_sysreg(tcr_el1);
	*pasid = process->pasid;

	mmput(mm);
	return 0;

err_free_svm_process:
	kfree(process);
	process = NULL;
err_put_mm_context:
	mm_context_put(mm);
	mmput(mm);
	put_pid(pid);

	return err;
}

static struct svm_device *file_to_sdev(struct file *file)
{
	return container_of(file->private_data, struct svm_device, miscdev);
}

static struct svm_dev_wl_mng *svm_device_get_mng(const char *device_name)
{
	int i;
	int svm_name_len;
	if (device_name == NULL)
		return NULL;

	for (i = 0; i < SVM_DEV_MAX; i++) {
		svm_name_len = strlen(svm_dev_white_list[i].svm_dev_name);
		if (strncmp(device_name, svm_dev_white_list[i].svm_dev_name, svm_name_len) == 0) {
			pr_debug("strncmp will return i = %d, svm_dev_name = %s, smmu_dev_name = %s\n",
					i, svm_dev_white_list[i].svm_dev_name, svm_dev_white_list[i].smmu_dev_name);
			return &svm_dev_white_list[i];
		}
	}
	return NULL;
}
static bool svm_device_is_power_on(const char *device_name)
{
	int i;
	int svm_name_len;
	int smmu_name_len;
	int device_name_len;
	int smmu_name_offset;
	unsigned int read_val;
	if (device_name == NULL || npu_dts_sys_peri == NULL)
		return false;
	device_name_len = strlen(device_name);
	for (i = 0; i < SVM_DEV_MAX; i++) {
		svm_name_len = strlen(svm_dev_white_list[i].svm_dev_name);
		smmu_name_len = strlen(svm_dev_white_list[i].smmu_dev_name);
		smmu_name_offset = device_name_len > smmu_name_len ? device_name_len - smmu_name_len : 0;
		if (strncmp(device_name, svm_dev_white_list[i].svm_dev_name, svm_name_len) == 0 ||
			strncmp(device_name + smmu_name_offset, svm_dev_white_list[i].smmu_dev_name, smmu_name_len) == 0) {
			read_val = readl_relaxed(npu_dts_sys_peri + svm_dev_white_list[i].crg_offset);
			pr_debug("npu_dts_sys_peri = 0x%llx  offset = 0x%x val = 0x%x, smmu_name_offset = %d\n",
					(uint64_t)(uintptr_t)npu_dts_sys_peri, svm_dev_white_list[i].crg_offset, read_val, smmu_name_offset);
			if ((read_val & BIT(CLK_EN_BIT)) != 0)
				return true;
		}
	}
	pr_err("error : device name = %s ,smmu_name_offset = %d , svm is not powner on , please powner on svm first.\n",
		device_name, smmu_name_offset);
	return false;
}

static int svm_device_post_probe(const char *device_name);
static int svm_open(struct inode *inode, struct file *file)
{
	int ret;
	struct svm_dev_wl_mng *dev_mng = NULL;
	struct svm_device *sdev = file_to_sdev(file);
	const char *device_name = dev_name(sdev->dev);

	if (!svm_device_is_power_on(device_name)) {
		dev_err(sdev->dev, "svm_open:  svm is not power on\n");
		return -EFAULT;
	}

	dev_mng = svm_device_get_mng(device_name);
	if (dev_mng == NULL) {
		dev_err(sdev->dev, "fail to get svm device mng\n");
		return -EFAULT;
	}

	mutex_lock(&svm_dev_pm_mutex);
	if (dev_mng->is_inited == false) {
		ret = arm_smmu_device_post_probe(dev_mng->smmu_dev_name);
		if (ret != 0) {
			dev_err(sdev->dev, "fail to do smmu post probe\n");
			goto err_exit;
		}

		ret = svm_device_post_probe(dev_mng->svm_dev_name);
		if (ret != 0) {
			dev_err(sdev->dev, "fail to do svm post probe\n");
			goto err_exit;
		}
		dev_mng->is_inited = true;
	} else {
		if (dev_mng->is_suspend == true) {
			ret = arm_smmu_device_resume(dev_mng->smmu_dev_name);
			if (ret != 0) {
				dev_err(sdev->dev, "fail to resume smmu\n");
				goto err_exit;
			}
			dev_mng->is_suspend = false;
		}
	}
	mutex_unlock(&svm_dev_pm_mutex);
	return 0;

err_exit:
	mutex_unlock(&svm_dev_pm_mutex);
	return -EFAULT;
}

static struct task_struct *svm_get_task(struct svm_bind_process params)
{
	struct task_struct *task = NULL;

	if (params.flags & ~SVM_BIND_PID)
		return ERR_PTR(-EINVAL);

	if (params.flags & SVM_BIND_PID) {
		struct mm_struct *mm = NULL;

		rcu_read_lock();
		task = find_task_by_vpid(params.vpid);
		if (task != NULL)
			get_task_struct(task);
		rcu_read_unlock();
		if (task == NULL)
			return ERR_PTR(-ESRCH);

		/* check the permission */
		mm = mm_access(task, PTRACE_MODE_ATTACH_REALCREDS);
		if (IS_ERR_OR_NULL(mm)) {
			pr_err("cannot access mm\n");
			put_task_struct(task);
			return ERR_PTR(-ESRCH);
		}

		mmput(mm);
	} else {
		get_task_struct(current);
		task = current;
	}

	return task;
}

int svm_get_pasid(pid_t vpid, int dev_id __maybe_unused)
{
	int pasid;
	unsigned long asid;
	struct task_struct *task = NULL;
	struct mm_struct *mm = NULL;
	struct svm_process *process = NULL;
	struct svm_bind_process params;

	params.flags = SVM_BIND_PID;
	params.vpid = vpid;
	params.pasid = -1;
	params.ttbr = 0;
	params.tcr = 0;
	task = svm_get_task(params);
	if (IS_ERR(task))
		return PTR_ERR(task);

	mm = get_task_mm(task);
	if (mm == NULL) {
		pasid = -EINVAL;
		goto put_task;
	}

	asid = mm_context_get(mm);
	if (!asid) {
		pasid = -ENOSPC;
		goto put_mm;
	}

	mutex_lock(&svm_process_mutex);
	process = find_svm_process(asid, dev_id);
	mutex_unlock(&svm_process_mutex);
	if (process != NULL)
		pasid = process->pasid;
	else
		pasid = -ESRCH;

	mm_context_put(mm);
put_mm:
	mmput(mm);
put_task:
	put_task_struct(task);

	return pasid;
}
EXPORT_SYMBOL_GPL(svm_get_pasid);

static void pte_flush_range(pmd_t *pmd, unsigned long addr, unsigned long end)
{
	pte_t *pte = NULL;
	pte_t *pte4k = NULL;

	pte = pte_offset_map(pmd, addr);

	pte4k = (pte_t *)round_down((u64)pte, PAGE_SIZE);
	__flush_dcache_area(pte4k, PAGE_SIZE);

	pte_unmap(pte);
}

static void pmd_flush_range(pud_t *pud, unsigned long addr, unsigned long end)
{
	pmd_t *pmd = NULL;
	pmd_t *pmd4k = NULL;
	unsigned long next;

	pmd = pmd_offset(pud, addr);
	pmd4k = (pmd_t *)round_down((u64)pmd, PAGE_SIZE);

	do {
		next = pmd_addr_end(addr, end);
		pte_flush_range(pmd, addr, next);
		pmd++;
		addr = next;
	} while (addr != end);

	__flush_dcache_area(pmd4k, PAGE_SIZE);
}

static void pud_flush_range(pgd_t *pgd, unsigned long addr, unsigned long end)
{
	pud_t *pud = NULL;
#if CONFIG_PGTABLE_LEVELS > 3
	pud_t *pud4k = NULL;
#endif
	unsigned long next;

	pud = pud_offset(pgd, addr);
#if CONFIG_PGTABLE_LEVELS > 3
	pud4k = (pud_t *)round_down((u64)pud, PAGE_SIZE);
#endif

	do {
		next = pud_addr_end(addr, end);
		pmd_flush_range(pud, addr, next);
		pud++;
		addr = next;
	} while (addr != end);

#if CONFIG_PGTABLE_LEVELS > 3
	__flush_dcache_area(pud4k, PAGE_SIZE);
#endif
}

int svm_flush_cache(struct mm_struct *mm, unsigned long addr, size_t size)
{
	pgd_t *pgd = NULL;
	pgd_t *pgd4k = NULL;
	unsigned long next;
	unsigned long end = addr + PAGE_ALIGN(size);
	const char *device_name = NULL;

	if (mm == NULL) {
		printk("%s: mm is null !!!!!\n", __FUNCTION__);
		return -1;
	}

	pgd = pgd_offset(mm, addr);
	pgd4k = (pgd_t *)round_down((u64)pgd, PAGE_SIZE);

	do {
		next = pgd_addr_end(addr, end);
		pud_flush_range(pgd, addr, next);
		pgd++;
		addr = next;
	} while (addr != end);

	__flush_dcache_area(pgd4k, PAGE_SIZE);

	mutex_lock(&svm_dev_pm_mutex);
	device_name = iommu_sva_get_smmu_device_name(mm);
	if (svm_device_is_power_on(device_name) == true)
		iommu_sva_flush_iotlb_single(mm);
	mutex_unlock(&svm_dev_pm_mutex);
	return 0;
}
EXPORT_SYMBOL_GPL(svm_flush_cache);

int svm_vma_check(const struct vm_area_struct *pvma1, const struct vm_area_struct *pvma2,
					unsigned long vm_start, unsigned long vm_end)
{
	if (pvma1 != pvma2) {
		pr_err("ERROR: pvma1:[0x%lx,0x%lx) and pvma2:[0x%lx,0x%lx) are not equal\n",
			pvma1->vm_start, pvma1->vm_end, pvma2->vm_start, pvma2->vm_end);
		return -1;
	}

	if (!(pvma1->vm_flags & VM_WRITE)) {
		pr_err("ERROR vma flag:0x%lx\n", pvma1->vm_flags);
		return -1;
	}

	if (pvma1->vm_start > vm_start) {
		pr_err("cannot find corresponding vma, vm[%lx, %lx], user range[%lx,%lx]\n",
			pvma1->vm_start, pvma1->vm_end, vm_start, vm_end);
		return -1;
	}

	if (pvma1->vm_ops == NULL || pvma1->vm_file == NULL) {
		pr_err("pvma1->vm_flags = 0x%lx, pvma2->vm_flags = 0x%lx, vm_ops = 0x%lx, vm_file = 0x%lx\n",
			pvma1->vm_flags, pvma2->vm_flags, (uintptr_t)pvma1->vm_ops, (uintptr_t)pvma1->vm_file);
		return -1;
	}
	return 0;
}

static long svm_page_table_sync(unsigned long __user *arg)
{
	int ret = -EINVAL;
	struct svm_pg_sync_para remap_para;
	struct vm_area_struct *pvma1 = NULL;
	struct vm_area_struct *pvma2 = NULL;
	unsigned long end;

	if (arg == NULL) {
		pr_err("arg is invalid.\n");
		return ret;
	}

	ret = copy_from_user(&remap_para, (void __user *)arg, sizeof(remap_para));
	if (ret) {
		pr_err("failed to copy args from user space.\n");
		return ret;
	}
	end = remap_para.vaddr + remap_para.len;
	pvma1 = find_vma(current->mm, remap_para.vaddr);
	if (pvma1 == NULL) {
		pr_err("ERROR: pvma1 is null, vir addr = 0x%llx or len = %d is illegal.\n", remap_para.vaddr, remap_para.len);
		return -1;
	}

	pvma2 = find_vma(current->mm, end - 1);
	if (pvma2 == NULL) {
		pr_err("ERROR: pvma2 is null, vir addr = 0x%llx or len = %d is illegal.\n", remap_para.vaddr, remap_para.len);
		return -1;
	}

	ret = svm_vma_check(pvma1, pvma2, remap_para.vaddr, end);
	if (ret != 0) {
		pr_err("ERROR vma_check failed vir addr = 0x%llx or len = %d is illegal.\n", remap_para.vaddr, remap_para.len);
		return -ESRCH;
	}

	if (end > pvma1->vm_end || end < remap_para.vaddr) {
		ret = -EINVAL;
		pr_err("memory length is out of range, vaddr:%pK, len:%u.\n", (void *)remap_para.vaddr, remap_para.len);
		return ret;
	}

	svm_flush_cache(pvma1->vm_mm, remap_para.vaddr, remap_para.len);
	return 0;
}

static long svm_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = -EINVAL;
	struct svm_bind_process params;
	struct task_struct *task = NULL;
	struct svm_device *sdev = file_to_sdev(file);
	if (!arg || sdev == NULL)
		return -EINVAL;

	if (cmd == SVM_IOCTL_PROCESS_BIND) {
		if (copy_from_user(&params, (void __user *)arg, sizeof(params))) {
			dev_err(sdev->dev, "fail to copy params\n");
			return -EFAULT;
		}
	}
	if (!svm_device_is_power_on(dev_name(sdev->dev))) {
		dev_err(sdev->dev, "svm_ioctl:  svm is not power on\n");
		return -EFAULT;
	}

	switch (cmd) {
	case SVM_IOCTL_PROCESS_BIND:
		task = svm_get_task(params);
		if (IS_ERR(task)) {
			dev_err(sdev->dev, "failed to get task\n");
			return PTR_ERR(task);
		}

		err = svm_process_bind(task, sdev, &params.ttbr, &params.tcr, &params.pasid);
		if (err) {
			put_task_struct(task);
			dev_err(sdev->dev, "failed to bind task %d\n", err);
			return err;
		}

		put_task_struct(task);
		if (copy_to_user((void __user *)arg, &params, sizeof(params)))
			err = -EFAULT;
		break;
	case SVM_IOCTL_PAGE_TABLE_SYNC:
		err = svm_page_table_sync((unsigned long __user*)arg);
		break;
	default:
		err = -EINVAL;
		break;
	}

	if (err)
		dev_err(sdev->dev, "%s: %s failed err = %d\n", __func__, svm_cmd_to_string(cmd), err);

	return err;
}

int svm_release(struct inode *inode_ptr, struct file *file_ptr)
{
	return 0;
}

static const struct file_operations svm_fops = {
	.owner			= THIS_MODULE,
	.open			= svm_open,
	.unlocked_ioctl 	= svm_ioctl,
	.release = svm_release,
};

static int svm_init_core(struct svm_device *sdev, struct device_node *np)
{
	int err = 0;
	struct device_node *child = NULL;
	struct device *dev = sdev->dev;

	down_write(&svm_sem);
	if (svm_bus_type.iommu_ops == NULL) {
		err = bus_register(&svm_bus_type);
		if (err) {
			up_write(&svm_sem);
			dev_err(dev, "failed to register svm_bus_type\n");
			return err;
		}

		err = bus_set_iommu(&svm_bus_type, dev->bus->iommu_ops);
		if (err) {
			up_write(&svm_sem);
			dev_err(dev, "failed to set iommu for svm_bus_type\n");
			goto err_unregister_bus;
		}
	} else if (svm_bus_type.iommu_ops != dev->bus->iommu_ops) {
		err = -EBUSY;
		up_write(&svm_sem);
		dev_err(dev, "iommu_ops configured, but changed!\n");
		return err;
	}
	up_write(&svm_sem);

	for_each_available_child_of_node(np, child) {
		err = svm_of_add_core(sdev, child);
		if (err)
			device_for_each_child(dev, NULL, svm_remove_core);
	}

	return err;

err_unregister_bus:
	bus_unregister(&svm_bus_type);

	return err;
}

static int svm_device_wl_process(struct platform_device *pdev, struct device *dev)
{
	int i;

	for (i = 0; i < SVM_DEV_MAX; i++) {
		if (strnstr(pdev->name, svm_dev_white_list[i].svm_dev_name, SVM_DEV_NAME_LEN) != NULL) {
			svm_dev_white_list[i].dev = (void *)dev;
			return 0;
		}
	}
	return -1;
}
static int svm_get_sys_and_crg(const struct device *dev)
{
	unsigned int crg_base;
	unsigned int crg_size;
	int i;
	int svm_name_len;
	const char *device_name = dev_name(dev);
	struct device_node *np = dev->of_node;

	for (i = 0; i < SVM_DEV_MAX; i++) {
		svm_name_len = strlen(svm_dev_white_list[i].svm_dev_name);
		if (strncmp(device_name, svm_dev_white_list[i].svm_dev_name, svm_name_len) == 0) {
			pr_debug("svm_get_sys_and_crg : strncmp will return i = %d, svm_dev_name = %s, smmu_dev_name = %s\n",
					i, svm_dev_white_list[i].svm_dev_name, svm_dev_white_list[i].smmu_dev_name);
			break;
		}
	}
	if (i == SVM_DEV_MAX) {
		dev_err(dev, "defer probe svm device, device name = %s not match\n", device_name);
		return -EPROBE_DEFER;
	}
	if (of_property_read_u32(np, "crg-base", &crg_base) || of_property_read_u32(np, "crg-size", &crg_size) ||
		of_property_read_u32(np, svm_dev_white_list[i].crg_name, &svm_dev_white_list[i].crg_offset)) {
		pr_warn("Warning: missing crg-base property in dts tree, we don't support smmu powner on check!!!\n");
		npu_dts_sys_peri = NULL;
	} else {
		npu_dts_sys_peri = ioremap(crg_base, crg_size);
	}
	dev_dbg(dev, " read crg_offset i = %d, crg_name = %s, crg_offset = 0x%x \n",
			i, svm_dev_white_list[i].crg_name, svm_dev_white_list[i].crg_offset);
	return 0;
}
static int svm_device_probe(struct platform_device *pdev)
{
	int err;
	struct device *dev = &pdev->dev;
	struct svm_device *sdev = NULL;
	struct device_node *np = dev->of_node;
	if (np == NULL)
		return -ENODEV;

	if (!dev->bus || !dev->bus->iommu_ops) {
		/* If SMMU is not probed, it should defer probe of this driver */
		dev_dbg(dev, "this dev bus is NULL or defer probe svm device\n");
		return -EPROBE_DEFER;
	}
	sdev = devm_kzalloc(dev, sizeof(*sdev), GFP_KERNEL);
	if (sdev == NULL)
		return -ENOMEM;

	sdev->id = probe_index;
	sdev->dev = dev;
	sdev->miscdev.minor = MISC_DYNAMIC_MINOR;
	sdev->miscdev.fops = &svm_fops;
	sdev->miscdev.name = devm_kasprintf(dev, GFP_KERNEL, SVM_DEVICE_NAME"%llu", sdev->id);
	if (sdev->miscdev.name == NULL)
		err = -ENOMEM;

	dev_set_drvdata(dev, sdev);
	err = misc_register(&sdev->miscdev);
	if (err) {
		dev_err(dev, "Unable to register misc device\n");
		return err;
	}

	if (svm_device_wl_process(pdev, dev) != 0) {
		err = svm_init_core(sdev, np);
		if (err) {
			dev_err(dev, "failed to init cores\n");
			goto err_unregister_misc;
		}
	}
	probe_index++;
	if (svm_get_sys_and_crg(dev) != 0) {
		dev_err(dev, "failed to svm_get_sys_and_crg, device error.\n");
		goto err_unregister_misc;
	}

	mutex_init(&svm_process_mutex);
	mutex_init(&svm_dev_pm_mutex);

	return err;
err_unregister_misc:
	misc_deregister(&sdev->miscdev);
	return err;
}

static int svm_device_post_probe(const char *device_name)
{
	int err, i;
	struct device *dev = NULL;
	struct svm_device *sdev = NULL;

	for (i = 0; i < SVM_DEV_MAX; i++) {
		if (strnstr(device_name, svm_dev_white_list[i].svm_dev_name, SVM_DEV_NAME_LEN) != NULL) {
			dev = (struct device *)svm_dev_white_list[i].dev;
			break;
		}
	}

	if (dev == NULL || i >= SVM_DEV_MAX) {
		dev_err(dev, "faile to find svm device in white list \n");
		return -1;
	}

	sdev = dev_get_drvdata(dev);
	if (sdev == NULL) {
		dev_err(dev, "failed get drv data\n");
		return -1;
	}

	err = svm_init_core(sdev, dev->of_node);
	if (err) {
		dev_err(dev, "failed to init cores\n");
		return -1;
	}

	return 0;
}

int svm_smmu_device_suspend(const char *device_name)
{
	int ret = 0;
	struct svm_dev_wl_mng *dev_mng = NULL;

	dev_mng = svm_device_get_mng(device_name);
	if (dev_mng == NULL)
		return -1;

	mutex_lock(&svm_dev_pm_mutex);
	if (dev_mng->is_suspend == false) {
		dev_mng->is_suspend = true;
		ret = arm_smmu_device_suspend(dev_mng->smmu_dev_name);
	}
	mutex_unlock(&svm_dev_pm_mutex);
	return ret;
}
EXPORT_SYMBOL_GPL(svm_smmu_device_suspend);

int svm_smmu_device_resume(const char *device_name)
{
	int ret = 0;
	struct svm_dev_wl_mng *dev_mng = NULL;

	dev_mng = svm_device_get_mng(device_name);
	if (dev_mng == NULL)
		return -1;

	mutex_lock(&svm_dev_pm_mutex);
	if (dev_mng->is_suspend == true) {
		ret = arm_smmu_device_resume(dev_mng->smmu_dev_name);
		dev_mng->is_suspend = false;
	}
	mutex_unlock(&svm_dev_pm_mutex);
	return ret;
}
EXPORT_SYMBOL_GPL(svm_smmu_device_resume);

void svm_smmu_device_reset_lock(void)
{
	mutex_lock(&svm_dev_pm_mutex);
	return;
}
EXPORT_SYMBOL(svm_smmu_device_reset_lock);

int svm_smmu_device_reset(const char *device_name)
{
	int ret = -1;
	struct svm_dev_wl_mng *dev_mng = NULL;

	dev_mng = svm_device_get_mng(device_name);
	if (dev_mng == NULL)
		return -1;

	if (dev_mng->is_suspend == false)
		ret = arm_smmu_device_reset_ex(dev_mng->smmu_dev_name);
	return ret;
}
EXPORT_SYMBOL(svm_smmu_device_reset);

void svm_smmu_device_reset_unlock(void)
{
	mutex_unlock(&svm_dev_pm_mutex);
	return;
}
EXPORT_SYMBOL(svm_smmu_device_reset_unlock);


int svm_smmu_clk_live_process_register(smmu_clk_live_func enter, smmu_clk_live_func exit)
{
	if (g_smmu_clk_live_enter == NULL)
		g_smmu_clk_live_enter = enter;

	if (g_smmu_clk_live_exit == NULL)
		g_smmu_clk_live_exit = exit;

	return 0;
}
EXPORT_SYMBOL_GPL(svm_smmu_clk_live_process_register);


void svm_smmu_clk_live_enter(void)
{
	if (g_smmu_clk_live_enter != NULL)
		g_smmu_clk_live_enter();
}
EXPORT_SYMBOL_GPL(svm_smmu_clk_live_enter);

void svm_smmu_clk_live_exit(void)
{
	if (g_smmu_clk_live_exit != NULL)
		g_smmu_clk_live_exit();
}
EXPORT_SYMBOL_GPL(svm_smmu_clk_live_exit);

static bool svm_device_is_inited(const char *device_name)
{
	int i;

	if (device_name == NULL)
		return false;

	for (i = 0; i < SVM_DEV_MAX; i++) {
		if (strnstr(device_name, svm_dev_white_list[i].svm_dev_name, SVM_DEV_NAME_LEN) != NULL) {
			if (svm_dev_white_list[i].is_inited == true)
				return true;
		}
	}
	return false;
}

static int svm_device_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct svm_device *sdev = dev_get_drvdata(dev);
	const char *device_name = dev_name(dev);

	if (npu_dts_sys_peri != NULL) {
		iounmap(npu_dts_sys_peri);
		npu_dts_sys_peri = NULL;
	}

	svm_smmu_clk_live_enter();
	if (svm_device_is_inited(device_name))
		device_for_each_child(sdev->dev, NULL, svm_remove_core);
	svm_smmu_clk_live_exit();

	misc_deregister(&sdev->miscdev);
	return 0;
}

static void svm_device_shutdown(struct platform_device *pdev)
{
	svm_device_remove(pdev);
	return;
}

static const struct of_device_id svm_of_match[] = {
	{ .compatible = "vendor,svm" },
	{ }
};
MODULE_DEVICE_TABLE(of, svm_of_match);

static struct platform_driver svm_driver = {
	.probe    = svm_device_probe,
	.remove   = svm_device_remove,
	.shutdown = svm_device_shutdown,
	.driver   = {
	.name = SVM_DEVICE_NAME,
	.of_match_table = svm_of_match,
	},
};

module_platform_driver(svm_driver);

MODULE_LICENSE("GPL v2");
