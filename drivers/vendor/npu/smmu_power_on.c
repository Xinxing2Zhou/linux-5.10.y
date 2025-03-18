/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */
#include <linux/atomic.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/iopoll.h>

#define VENDOR_TOP_CTL_BASE (0x30000)

#define SMMU_LP_REQ (VENDOR_TOP_CTL_BASE + 0)
#define TCU_QREQN_CG BIT(0)
#define TCU_QREQN_PD BIT(1)

#define SMMU_LP_ACK (VENDOR_TOP_CTL_BASE + 0x4)
#define TCU_QACCEPTN_CG BIT(0)
#define TCU_QACCEPTN_PD BIT(4)

/* TBU reg */
#define SMMU_TBU_CR (0)
#define TBU_EN_REQ BIT(0)

#define SMMU_TBU_CRACK (0x4)
#define TBU_EN_ACK BIT(0)
#define TBU_CONNECTED BIT(1)

#define ARM_SMMU_POLL_TIMEOUT_US   100

int npu_reg_bit_set_with_ack(void __iomem *base, unsigned int req_off,
									unsigned int ack_off, unsigned int req_bit, unsigned int ack_bit)
{
	u32 reg = 0;
	u32 val = 0;

	val = readl_relaxed(base + req_off);
	val |= req_bit;
	writel_relaxed(val, base + req_off);
	return readl_relaxed_poll_timeout(base + ack_off, reg,
						reg & ack_bit, 1, ARM_SMMU_POLL_TIMEOUT_US);
}


int svm_smmu_power_on(void *base, unsigned int tcu_offset, unsigned int tbu_offset)
{
	int ret;
	void __iomem *tmp_base;
	u32 reg;

	/****************tcu configure***************/
	tmp_base = (void __iomem *)base + tcu_offset;
	/* Request leave power-down mode */
	ret = npu_reg_bit_set_with_ack(tmp_base, SMMU_LP_REQ, SMMU_LP_ACK, TCU_QREQN_CG, TCU_QACCEPTN_CG);
	if (ret) {
		printk("TCU_QACCEPTN_CG failed !%s\n", __func__);
		return -EINVAL;
	}
	/* Request leave clock-gating mode */
	ret = npu_reg_bit_set_with_ack(tmp_base, SMMU_LP_REQ, SMMU_LP_ACK, TCU_QREQN_PD, TCU_QACCEPTN_PD);
	if (ret) {
		printk("npu_reg_bit_set_with_ack failed !%s\n", __func__);
		return -EINVAL;
	}

	/****************tbu configure***************/
	/* enable AICore tbu */
	tmp_base = (void __iomem *)base + tbu_offset;
	/* enable TBU request */
	npu_reg_bit_set_with_ack(tmp_base, SMMU_TBU_CR, SMMU_TBU_CRACK, TBU_EN_REQ, TBU_EN_ACK);

	/* check TBU enable acknowledge */
	reg = readl_relaxed(tmp_base + SMMU_TBU_CRACK);
	if (!(reg & TBU_CONNECTED)) {
		printk("%s:----------->Fail to CONNECTE TBU failed!\n", __func__);
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(svm_smmu_power_on);


MODULE_LICENSE("GPL v2");
