/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#include <asm/esr.h>
#include <asm/mmu_context.h>

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
#include <linux/dma-buf.h>
#include "linux/securec.h"

#define NPU_MISC_DEVICE_NAME		 "nmsc"

#define NPU_MISC_IOCTL_GET_PHYS 	 0xfff4
#define NPU_MISC_IOCTL_ADD_DBF		 0xfff5
#define NPU_MISC_IOCTL_RMV_DBF		 0xfff6

#ifdef MISC_MEM_DBG
#define NPU_MISC_IOCTL_DATA_COPY_TEST	0xfff8
#define NPU_MISC_IOCTL_SHOW_MEM 		0xfff9
#endif

#define NPU_DBF_INDEX_VALID_FLAG		0x0

#define NPU_MODULE_ID 0xAA
#define NPU_MISC_SIG_INDEX_VALUE 2048

#define NPU_MISC_MAX_DMA_BUF_TABLE 10
#define NPU_MISC_MAX_INDEX_VALUE (NPU_MISC_SIG_INDEX_VALUE * NPU_MISC_MAX_DMA_BUF_TABLE)

struct nmsc_device {
	unsigned long long id;
	struct miscdevice miscdev;
	struct device *dev;
};

/* dbf(dma buffer fd) process */
struct dbf_process {
	unsigned int	   flag;
	unsigned int	   index;
	struct dma_buf	  *dma_buf;
	struct rb_node rb_node;
};

struct data_copy_info {
	unsigned long long src_addr_info;
	unsigned long long dst_addr_info;
	unsigned int	   src_size;
	unsigned int	   dst_size;
};

struct kva_map_params {
	unsigned long long	user_va;   /* addr info from user space */
	void			   *kva;	   /* kernel virtual addr */
	void			   *buf_handle;    /* buffer handle, e.g. dma buffer handle */
};

struct mem_show_params {
	unsigned long long user_addr_info;
	unsigned int	   size;
	unsigned int	   flag;  /* 0: show vitual mem, 1: show physical mem */
};

static struct mutex   dbf_process_mutex;

struct dma_buf_table_index {
	unsigned int valid_index;
	unsigned int cur_table_index;
	struct dma_buf** buff_index_table[NPU_MISC_MAX_DMA_BUF_TABLE]; // index for dma_buf table .
};

struct dfb_manager {
	struct rb_root dfb_process_root; // rb tree to store dma_buffer info
	struct dma_buf_table_index buf_table;
};

static struct dfb_manager dfb_man;

static char *nmsc_cmd_to_string(unsigned int cmd)
{
	switch (cmd) {
	case NPU_MISC_IOCTL_ADD_DBF:
		return "add dma buffer fd";
	case NPU_MISC_IOCTL_RMV_DBF:
		return "remove dma buffer fd";
#ifdef MISC_MEM_DBG
	case NPU_MISC_IOCTL_GET_PHYS:
		return "get phys";
	case NPU_MISC_IOCTL_DATA_COPY_TEST:
		return "data copy test";
#endif
	default:
		return "unsupported";
	}
}
static struct nmsc_device *file_to_sdev(struct file *file)
{
	return container_of(file->private_data, struct nmsc_device, miscdev);
}
#ifdef USE_ION

static void *kal_mem_handle_get(long fd, unsigned int module_id)
{
	struct dma_buf *dmabuf = NULL;

	dmabuf = dma_buf_get(fd);
	if (IS_ERR_OR_NULL(dmabuf)) {
		pr_err("osal get handle failed!\n");
		return NULL;
	}

	pr_debug("%s: module_id=%d get handle,ref:%pa,!\n", __func__,
			 module_id, &(dmabuf->file->f_count.counter));

	return (void *)dmabuf;
}

static void kal_mem_ref_put(void *handle, unsigned int module_id)
{
	struct dma_buf *dmabuf = NULL;

	if (IS_ERR_OR_NULL(handle)) {
		pr_err("%s, osal err args!\n", __func__);
		return;
	}

	dmabuf = (struct dma_buf *)handle;
	dma_buf_put(dmabuf);
	pr_debug("%s: module_id=%d put handle,ref:%pa,!\n", __func__,
			 module_id, &(dmabuf->file->f_count.counter));
	return;
}

/* map cpu addr  */
static void *kal_mem_kmap(void *handle, unsigned long offset, int cache)
{
	void *virt = NULL;
	struct dma_buf *dmabuf = NULL;
	int ret;

	if (IS_ERR_OR_NULL(handle)) {
		pr_err("%s, osal err args!\n", __func__);
		return NULL;
	}
	dmabuf = (struct dma_buf *)handle;
	ret = set_buffer_cached(dmabuf, cache);
	if (ret) {
		pr_err("osal set cache attr failed!\n");
		return NULL;
	}

	virt = dma_buf_kmap(dmabuf, offset >> PAGE_SHIFT);
	if (virt == NULL) {
		set_buffer_cached(dmabuf, !cache);
		pr_err("osal map failed!\n");
		return NULL;
	}

	return virt;
}

/* unmap cpu addr  */
static void kal_mem_kunmap(void *handle, void *virt, unsigned long offset)
{
	struct dma_buf *dmabuf = NULL;

	if (IS_ERR_OR_NULL(handle) || virt == NULL) {
		pr_err("%s, osal err args!\n", __func__);
		return;
	}

	dmabuf = (struct dma_buf *)handle;
	dma_buf_kunmap(dmabuf, offset >> PAGE_SHIFT, virt);
}
struct dma_buf *npu_misc_get_dma_buf(unsigned int db_idx)
{
	struct dma_buf *temp = NULL;
	unsigned int table_index;
	unsigned int dma_index;
	if (db_idx >= NPU_MISC_MAX_INDEX_VALUE) {
		pr_err("%s, db_idx err args!\n", __func__);
		return NULL;
	}

	mutex_lock(&dbf_process_mutex);
	table_index = db_idx / NPU_MISC_SIG_INDEX_VALUE;
	dma_index = db_idx % NPU_MISC_SIG_INDEX_VALUE;
	if (table_index > dfb_man.buf_table.cur_table_index) {
		pr_err("%s, db_idx = %d is out of range! current table index = %d.\n",
			__func__, db_idx, dfb_man.buf_table.cur_table_index);
		return NULL;
	}
	temp = dfb_man.buf_table.buff_index_table[table_index][dma_index];
	if (temp == NULL)
		pr_err("ERROR: db_idx = %d, has no dmabuff stored!!!\n", db_idx);

	mutex_unlock(&dbf_process_mutex);
	return temp;
}
EXPORT_SYMBOL_GPL(npu_misc_get_dma_buf);


int npu_kva_map(struct kva_map_params *kva_para)
{
	unsigned int db_idx;
	unsigned int buf_offset;
	void *dma_buf = NULL;
	void *kva = NULL;
	if (kva_para == NULL) {
		pr_err("%s[%d]: kva_para is illegal\n", __FUNCTION__, __LINE__);
		return -EFAULT;
	}

	db_idx = (unsigned int)(kva_para->user_va >> 32);   // high 32 bit is used to save dfb index
	if ((db_idx >> 24) != NPU_DBF_INDEX_VALID_FLAG) {  // flag in in offset 24 bit
		pr_err("%s[%d]: invalid user addr info \n", __FUNCTION__, __LINE__);
		return -EFAULT;
	}

	buf_offset = (unsigned int)(kva_para->user_va & 0xFFFFFFFF);

	dma_buf = (void *)npu_misc_get_dma_buf(db_idx & 0x00FFFFFF);
	if (dma_buf == NULL) {
		pr_err("%s[%d]: fail get dma buf handle\n", __FUNCTION__, __LINE__);
		return -EFAULT;
	}
	kva_para->buf_handle = dma_buf;

	kva = kal_mem_kmap(dma_buf, buf_offset, 0);
	if (kva == NULL) {
		pr_err("%s[%d]: fail to map src address \n", __FUNCTION__, __LINE__);
		return -EFAULT;
	}

	kva_para->kva = kva + buf_offset % PAGE_SIZE;

	return 0;
}
EXPORT_SYMBOL_GPL(npu_kva_map);


int npu_kva_unmap(struct kva_map_params *kva_para)
{
	void  *kva = NULL;
	unsigned int buf_offset;

	if (kva_para->kva == NULL || kva_para->buf_handle == NULL) {
		pr_err("%s[%d]: invalid parameters\n", __FUNCTION__, __LINE__);
		return -EFAULT;
	}

	buf_offset = (unsigned int)(kva_para->user_va & 0xFFFFFFFF);
	kva = kva_para->kva - buf_offset % PAGE_SIZE;
	kal_mem_kunmap((void *)kva_para->buf_handle, kva, buf_offset);
	return 0;
}
EXPORT_SYMBOL_GPL(npu_kva_unmap);

static struct dbf_process *find_dfb_process(struct dma_buf *dma_buf)
{
	struct rb_node *node = dfb_man.dfb_process_root.rb_node;

	while (node != NULL) {
		struct dbf_process *process = NULL;
		process = rb_entry(node, struct dbf_process, rb_node);
		if (dma_buf < process->dma_buf)
			node = node->rb_left;
		else if (dma_buf > process->dma_buf)
			node = node->rb_right;
		else
			return process;
	}

	return NULL;
}

static void delete_dfb_process(struct dbf_process *process)
{
	rb_erase(&process->rb_node, &dfb_man.dfb_process_root);
	RB_CLEAR_NODE(&process->rb_node);
}

static unsigned int get_valid_index(struct dma_buf_table_index *buff_table)
{
	unsigned int index;
	unsigned int update_index;
	unsigned int table_index;
	unsigned int dma_index;

	if (buff_table == NULL) {
		pr_err("%s[%d], buff_table_index is null!!\n", __func__, __LINE__);
		return -1;
	}

	index = buff_table->valid_index;
	update_index = index + 1;
	while (update_index < NPU_MISC_MAX_INDEX_VALUE) {
		table_index = update_index / NPU_MISC_SIG_INDEX_VALUE;
		dma_index = update_index % NPU_MISC_SIG_INDEX_VALUE;
		if (table_index <= buff_table->cur_table_index && buff_table->buff_index_table[table_index][dma_index] == NULL) {
			break;
		} else if (table_index > buff_table->cur_table_index) {
			// malloc
			dfb_man.buf_table.cur_table_index = table_index;
			dfb_man.buf_table.buff_index_table[table_index] =
				kzalloc(sizeof(struct dma_buf*) * NPU_MISC_SIG_INDEX_VALUE, GFP_ATOMIC);
			if (dfb_man.buf_table.buff_index_table[table_index] == NULL) {
				pr_err("devm_kzalloc failed, unable to malloc buff_index_table space.\n");
				return -1;
			}
			break;
		}

		update_index++;
	}

	if (index >= NPU_MISC_MAX_INDEX_VALUE) {
		pr_err("no valid index can be used, current index = %d, max index = %d!!!\n",
			index, NPU_MISC_MAX_INDEX_VALUE);
		return -1;
	}

	buff_table->valid_index = update_index;
	return index;
}

static int reset_index(unsigned int index)
{
	unsigned int table_index;
	unsigned int dma_index;

	table_index = index / NPU_MISC_SIG_INDEX_VALUE;
	dma_index = index % NPU_MISC_SIG_INDEX_VALUE;
	if (table_index > dfb_man.buf_table.cur_table_index) {
		pr_err("%s, db_idx = %d is out of range! current table index = %d.\n",
			__func__, index, dfb_man.buf_table.cur_table_index);
		return -1;
	}

	dfb_man.buf_table.buff_index_table[table_index][dma_index] = 0;
	if (dfb_man.buf_table.valid_index > index)
		dfb_man.buf_table.valid_index = index;

	return 0;
}
static int add_dfb_node(struct dma_buf *dma_buf, unsigned int *dfb_idx)
{
	struct rb_node **p = &dfb_man.dfb_process_root.rb_node;
	struct rb_node *parent = NULL;
	struct dbf_process *process = NULL;
	unsigned int table_index;
	unsigned int dma_index;

	mutex_lock(&dbf_process_mutex);
	while (*p) {
		struct dbf_process *tmp_process = NULL;
		parent = *p;
		tmp_process = rb_entry(parent, struct dbf_process, rb_node);
		if (dma_buf < tmp_process->dma_buf) {
			p = &(*p)->rb_left;
		} else if (dma_buf > tmp_process->dma_buf) {
			p = &(*p)->rb_right;
		} else {
			*dfb_idx = tmp_process->index; // asid already in the tree.
			mutex_unlock(&dbf_process_mutex);
			return 0;
		}
	}

	process = kzalloc(sizeof(*process), GFP_ATOMIC);
	if (process == NULL) {
		pr_err("%s, Fail to kzalloc memory for dfb node!\n", __func__);
		mutex_unlock(&dbf_process_mutex);
		return -1;
	}

	process->flag = 0xA5A5A5A5;
	process->dma_buf = dma_buf;

	process->index = get_valid_index(&dfb_man.buf_table);
	if (process->index == -1) {
		mutex_unlock(&dbf_process_mutex);
		pr_err("%s, line: %d, Fail to get valid index!\n", __func__, __LINE__);
		return -1;
	}

	table_index = process->index / NPU_MISC_SIG_INDEX_VALUE;
	dma_index = process->index % NPU_MISC_SIG_INDEX_VALUE;
	dfb_man.buf_table.buff_index_table[table_index][dma_index] = dma_buf;

	rb_link_node(&process->rb_node, parent, p);
	rb_insert_color(&process->rb_node, &dfb_man.dfb_process_root);
	*dfb_idx = process->index;
	mutex_unlock(&dbf_process_mutex);
	return 0;
}

static int rmv_dfb_node(struct dma_buf *dma_buf)
{
	int ret;
	struct dbf_process *temp_process = NULL;

	mutex_lock(&dbf_process_mutex);
	temp_process = find_dfb_process(dma_buf);
	if (temp_process == NULL) {
		pr_err("%s, Fail to find dfb process, no such dma buff!\n", __func__);
		mutex_unlock(&dbf_process_mutex);
		return -1;
	}
	ret = reset_index(temp_process->index);
	if (ret != 0) {
		pr_err("%s, Fail to reset index!\n", __func__);
		mutex_unlock(&dbf_process_mutex);
		return ret;
	}

	delete_dfb_process(temp_process);
	kfree(temp_process);
	temp_process = NULL;

	mutex_unlock(&dbf_process_mutex);
	return ret;
}

static int npu_misc_dbf_add(unsigned long __user *arg)
{
	int err, db_fd;
	unsigned int db_idx;
	void *dma_buf = NULL;
	unsigned long user_addr_info;

	if (arg == NULL)
		return -EINVAL;

	if (get_user(user_addr_info, arg))
		return -EFAULT;

	db_fd = (int)(user_addr_info & 0xFFFFFFFF);
	dma_buf = kal_mem_handle_get(db_fd, NPU_MODULE_ID);
	if (dma_buf == NULL) {
		pr_err("%s[%d]: call osal_mem_handle_get failure\n", __FUNCTION__, __LINE__);
		return -EFAULT;
	}

	err = add_dfb_node(dma_buf, &db_idx);
	if (err < 0) {
		pr_err("%s[%d]: fail to add dfb node, err = %d\n", __FUNCTION__, __LINE__, err);
		kal_mem_ref_put(dma_buf, NPU_MODULE_ID);
		return -EFAULT;
	}

	db_idx |= (NPU_DBF_INDEX_VALID_FLAG << 24);

	if (dma_buf != NULL) {
		kal_mem_ref_put(dma_buf, NPU_MODULE_ID);
	}

	return put_user(db_idx, arg);
}

static int npu_misc_dbf_rmv(unsigned long __user *arg)
{
	int err, db_fd;
	void *dma_buf = NULL;
	unsigned long dbf_value;

	if (arg == NULL)
		return -EINVAL;

	if (get_user(dbf_value, arg))
		return -EFAULT;

	db_fd = (int)(dbf_value & 0xFFFFFFFF);

	dma_buf = kal_mem_handle_get(db_fd, NPU_MODULE_ID);
	if (dma_buf == NULL) {
		pr_err("%s[%d]: call osal_mem_handle_get failure\n", __FUNCTION__, __LINE__);
		return -EFAULT;
	}

	err = rmv_dfb_node((struct dma_buf *)dma_buf);
	if (err < 0) {
		pr_err("%s[%d]: fail to rmv dfb node\n", __FUNCTION__, __LINE__);
		return -EFAULT;
	}
	if (dma_buf != NULL) {
		kal_mem_ref_put(dma_buf, NPU_MODULE_ID);
	}

	return err;
}
#endif

#ifdef MISC_MEM_DBG
static int npu_misc_data_copy_test(struct data_copy_info *copy_info)
{
	int err;
	struct kva_map_params src_kva_para = {0};
	struct kva_map_params dst_kva_para = {0};

	src_kva_para.user_va = copy_info->src_addr_info;
	err = npu_kva_map(&src_kva_para);
	if (err != 0) {
		pr_err("%s[%d]: Error: fail to kmap source address \n", __FUNCTION__, __LINE__);
		err = -EFAULT;
		goto __err_exit;
	}

	dst_kva_para.user_va = copy_info->dst_addr_info;
	err = npu_kva_map(&dst_kva_para);
	if (err != 0) {
		pr_err("%s[%d]: Error: fail to kmap source address \n", __FUNCTION__, __LINE__);
		err = -EFAULT;
		goto __err_exit;
	}

	if (src_kva_para.kva + copy_info->src_size >= dst_kva_para.kva) {
		pr_err("%s[%d]: Error: copy address override \n", __FUNCTION__, __LINE__);
		err = -EFAULT;
		goto __err_exit;
	}

	err = memcpy_s(dst_kva_para.kva, copy_info->dst_size, src_kva_para.kva, copy_info->src_size);

__err_exit:
	if (src_kva_para.kva != NULL)
		npu_kva_unmap(&src_kva_para);
	if (dst_kva_para.kva != NULL)
		npu_kva_unmap(&dst_kva_para);
	return err;
}

static int npu_misc_show_mem(struct device *dev, struct mem_show_params *mem_params)
{
	int err, db_fd;
	unsigned int buf_offset, i;
	void *dma_buf = NULL;
	void *kva = NULL;
	char *ptr = NULL;

	db_fd = (int)(mem_params->user_addr_info >> 32); // high 32 bit is used to save dma buffer fd
	buf_offset = (unsigned int)(mem_params->user_addr_info & 0xFFFFFFFF);

	dma_buf = kal_mem_handle_get(db_fd, NPU_MODULE_ID);
	if (dma_buf == NULL) {
		pr_err("%s[%d]: call osal_mem_handle_get failure\n", __FUNCTION__, __LINE__);
		return -EFAULT;
	}

	kva = kal_mem_kmap(dma_buf, buf_offset, 0);
	if (kva == NULL) {
		pr_err("%s[%d]: fail to map src address \n", __FUNCTION__, __LINE__);
		err = -EFAULT;
		goto __err_exit;
	}

	ptr = (char *)(uintptr_t)kva;
	for (i = 0; i < mem_params->size; i++) {
		if (i % 16 == 0)  /* 16 bytes align for print output */
			dev_info(dev, "\n");

		dev_info(dev, "0x%x ", ptr[i]);
	}
	dev_info(dev, "\n");

	err = 0;
__err_exit:
	if (kva != NULL)
		kal_mem_kunmap(dma_buf, kva, buf_offset);
	if (dma_buf != NULL)
		kal_mem_ref_put(dma_buf, NPU_MODULE_ID);
	return err;
}
#endif

static long npu_misc_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = -EINVAL;
	struct nmsc_device *sdev = file_to_sdev(file);
#ifdef MISC_MEM_DBG
	struct data_copy_info params;
	struct mem_show_params mem_show;
#endif

	if (!arg)
		return -EINVAL;

	switch (cmd) {
#ifdef USE_ION
	case NPU_MISC_IOCTL_ADD_DBF:
		err = npu_misc_dbf_add((unsigned long __user *)arg);
		break;
	case NPU_MISC_IOCTL_RMV_DBF:
		err = npu_misc_dbf_rmv((unsigned long __user *)arg);
		break;
#endif
#ifdef MISC_MEM_DBG
	case NPU_MISC_IOCTL_DATA_COPY_TEST:
		err = copy_from_user(&params, (void __user *)arg, sizeof(params));
		if (err) {
			dev_err(sdev->dev, "fail to copy params\n");
			return -EFAULT;
		}
		err = npu_misc_data_copy_test(&params);
		break;
	case NPU_MISC_IOCTL_SHOW_MEM:
		err = copy_from_user(&mem_show, (void __user *)arg, sizeof(mem_show));
		if (err) {
			dev_err(sdev->dev, "fail to copy params\n");
			return -EFAULT;
		}
		err = npu_misc_show_mem(sdev->dev, &mem_show);
		break;
#endif
	default:
		err = -EINVAL;
		break;
	}

	if (err)
		dev_err(sdev->dev, "%s: %s failed err = %d\n", __func__, nmsc_cmd_to_string(cmd), err);

	return err;
}

static int npu_misc_open(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations npu_misc_fops = {
	.owner			= THIS_MODULE,
	.open			= npu_misc_open,
	.unlocked_ioctl 	= npu_misc_ioctl,
};

static int npu_misc_device_probe(struct platform_device *pdev)
{
	int err;
	struct device *dev = &pdev->dev;
	struct nmsc_device *sdev = NULL;

	sdev = devm_kzalloc(dev, sizeof(*sdev), GFP_KERNEL);
	if (sdev == NULL)
		return -ENOMEM;

	sdev->dev = dev;
	sdev->miscdev.minor = MISC_DYNAMIC_MINOR;
	sdev->miscdev.fops = &npu_misc_fops;
	sdev->miscdev.name = devm_kasprintf(dev, GFP_KERNEL, NPU_MISC_DEVICE_NAME);
	if (sdev->miscdev.name == NULL)
		err = -ENOMEM;

	dev_set_drvdata(dev, sdev);
	err = misc_register(&sdev->miscdev);
	if (err) {
		dev_err(dev, "Unable to register misc device\n");
		return err;
	}

	dfb_man.dfb_process_root = RB_ROOT;
	dfb_man.buf_table.cur_table_index = 0;
	dfb_man.buf_table.buff_index_table[0] = kzalloc(sizeof(struct dma_buf*) * NPU_MISC_SIG_INDEX_VALUE, GFP_ATOMIC);
	if (dfb_man.buf_table.buff_index_table[0] == NULL) {
		dev_err(dev, "devm_kzalloc failed, unable to malloc buff_index_table space.\n");
		return err;
	}
	mutex_init(&dbf_process_mutex);

	return err;
}

static int npu_misc_device_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct nmsc_device *sdev = dev_get_drvdata(dev);
	int i;

	for (i = 0; i <= dfb_man.buf_table.cur_table_index; i++)
		kfree(dfb_man.buf_table.buff_index_table[i]);

	misc_deregister(&sdev->miscdev);

	return 0;
}

static const struct of_device_id npu_misc_of_match[] = {
	{ .compatible = "vendor,npu_misc_device_drv" },
	{ }
};
MODULE_DEVICE_TABLE(of, npu_misc_of_match);


static struct platform_driver npu_misc_driver = {
	.probe = npu_misc_device_probe,
	.remove = npu_misc_device_remove,
	.driver = {
	.name = "npu_misc_device_drv",
	.of_match_table = npu_misc_of_match,
	},
};

static void npu_misc_dev_release(struct device *dev)
{
	return;
}

static struct platform_device npu_misc_device = {
	.name = "npu_misc_device_drv",
	.id = -1,
	.dev = {
		.platform_data = NULL,
		.release = npu_misc_dev_release,
	},
};

int __init npu_drv_misc_platform_init(void)
{
	int ret;

	ret = platform_device_register(&npu_misc_device);
	if (ret < 0) {
		printk("call platform_device_register failed!\n");
		return ret;
	}

	ret = platform_driver_register(&npu_misc_driver);
	if (ret) {
		platform_device_unregister(&npu_misc_device);
		printk("insmod npu misc platform driver fail. ret=%d\n", ret);
		return ret;
	}

	return ret;
}
module_init(npu_drv_misc_platform_init);

void __exit npu_drv_misc_platform_exit(void)
{
	platform_driver_unregister(&npu_misc_driver);
	platform_device_unregister(&npu_misc_device);
}
module_exit(npu_drv_misc_platform_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("NPU MISC DRIVER");
MODULE_VERSION("V1.0");

