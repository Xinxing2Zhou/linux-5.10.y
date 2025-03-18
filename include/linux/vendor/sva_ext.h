/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#ifndef _SVA_EXTEND_H
#define _SVA_EXTEND_H

#ifdef CONFIG_VENDOR_NPU

extern void iommu_sva_flush_iotlb_single(struct mm_struct *mm);

extern const char *iommu_sva_get_smmu_device_name(struct mm_struct *mm);

extern int arm_smmu_device_post_probe(const char *device_name);

extern int arm_smmu_device_resume(const char *device_name);

extern int arm_smmu_device_suspend(const char *device_name);

extern int arm_smmu_device_reset_ex(const char *device_name);

extern const char *arm_smmu_get_device_name(struct iommu_domain *domain);

extern int svm_flush_cache(struct mm_struct *mm, unsigned long addr, size_t size);

extern void svm_smmu_clk_live_enter(void);
extern void svm_smmu_clk_live_exit(void);

#else
static inline void iommu_sva_flush_iotlb_single(struct mm_struct *mm)
{
	return;
}

static inline const char *iommu_sva_get_smmu_device_name(struct mm_struct *mm)
{
	return NULL;
}

static inline int arm_smmu_device_post_probe(const char *device_name)
{
	return -1;
}

static inline int arm_smmu_device_resume(const char *device_name)
{
	return -1;
}

static inline int arm_smmu_device_suspend(const char *device_name)
{
	return -1;
}

static inline int arm_smmu_device_reset_ex(const char *device_name)
{
	return -1;
}

static const char *arm_smmu_get_device_name(struct iommu_domain *domain)
{
	return NULL;
}

static inline int svm_flush_cache(struct mm_struct *mm, unsigned long addr, size_t size)
{
	return -1;
}

static inline void svm_smmu_clk_live_enter(void)
{
	return;
}

static inline void svm_smmu_clk_live_exit(void)
{
	return;
}

#endif

#endif /* _SVA_EXTEND_H */
