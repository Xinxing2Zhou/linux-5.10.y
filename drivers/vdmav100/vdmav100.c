/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/fcntl.h>
#include <linux/mm.h>
#include <linux/miscdevice.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/string.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/mod_devicetable.h>
#include <linux/vmalloc.h>
#include <asm/pgtable.h>
#include <asm/page.h>
#include <linux/mm_types.h>
#include <linux/mmu_context.h>
#include <asm/tlbflush.h>
#include <linux/clk.h>
#include <linux/reset.h>

#include "vdmav100.h"
#include "vdma.h"

#define user_addr(ptr) (((uintptr_t)(ptr) < TASK_SIZE) \
		&& ((uintptr_t)(ptr) > 0))

int vdma_flag = 0;
EXPORT_SYMBOL(vdma_flag);

void __iomem *reg_vdma_base_va;

spinlock_t my_lock;
spinlock_t reg_lock;

unsigned int irq_flag;

unsigned int wake_channel_flag[DMAC_MAX_CHANNELS];

wait_queue_head_t dmac_wait_queue[DMAC_MAX_CHANNELS];

int g_channel_status[DMAC_MAX_CHANNELS];

/*
 *  allocate channel.
 */
int vdma_channel_allocate(const void *pisr)
{
	unsigned int  i, channelinfo, tmp, channel_intr;
	unsigned long flags;

	spin_lock_irqsave(&my_lock, flags);

	if (reg_vdma_base_va == NULL)
		return 0;
	dmac_readw(reg_vdma_base_va + DMAC_CHANNEL_STATUS, channelinfo);

	for (i = 0; i < CHANNEL_NUM; i++) {
		if (g_channel_status[i] == DMAC_CHN_VACANCY) {
			dmac_readw(reg_vdma_base_va + dmac_cxintr_raw(i),
				   channel_intr);
			tmp = channelinfo >> i;
			/* check the vdma channel data transfer is finished ? */
			if (((tmp & 0x01) == 0x00) && (channel_intr == 0x00)) {
				g_channel_status[i] = DMAC_CHN_ALLOCAT;
				spin_unlock_irqrestore(&my_lock, flags);
				return i; /* return channel number */
			}
		}
	}

	spin_unlock_irqrestore(&my_lock, flags);
	return DMAC_CHANNEL_INVALID;
}

/*
 *  free channel
 */
int vdma_channel_free(unsigned int channel)
{
	g_channel_status[channel] = DMAC_CHN_VACANCY;

	return 0;
}

/*
 *  channel enable
 *  start a vdma transfer immediately
 */
int vdma_channelstart(unsigned int channel,
			 const unsigned int *src, const unsigned int *dest)
{
	struct mm_struct *mm = NULL;
	unsigned int reg[DMAC_MAX_CHANNELS];

	if (channel >= DMAC_MAX_CHANNELS) {
		pr_err("channel number is out of scope<%d>.\n",
		       DMAC_MAX_CHANNELS);
		return -EINVAL;
	}

	g_channel_status[channel] = DMAC_NOT_FINISHED;

	mm = current->mm;
	if (!mm)
		mm = &init_mm;

	if (mm->pgd == NULL)
		return -EINVAL;
	/* set ttbr */
	/* get TTBR from the page */
	reg[channel] = __pa(mm->pgd);

	/* only [31:10] is the ttbr */
	reg[channel] &= 0xfffffc00;

	/* set the RGN,AFE,AFFD,TRE */
	reg[channel] |= TTB_RGN | AFE | TRE;

	if (user_addr(dest))
		reg[channel] &= ~DEST_IS_KERNEL;
	else
		reg[channel] |= DEST_IS_KERNEL;

	if (user_addr(src))
		reg[channel] &= ~SRC_IS_KERNEL;
	else
		reg[channel] |= SRC_IS_KERNEL;

	if (in_atomic() || in_interrupt()) {
		/* disable the channel interrupt */
		reg[channel] &= ~DMAC_INTR_ENABLE;
	} else {
		/* enable the channel interrupt */
		reg[channel] |= DMAC_INTR_ENABLE;
	}

	reg[channel] |= DMAC_CHANNEL_ENABLE;

	if (reg_vdma_base_va == NULL)
		return 0;
	/* set the TTBR register */
	dmac_writew(reg_vdma_base_va + dmac_cxttbr(channel),
		    reg[channel]);
	return 0;
}

/*
 *  Apply VDMA interrupt resource
 *  init channel state
 */
int vdma_driver_init(const struct vdmac_host *dma)
{
	unsigned int i;
	unsigned int tmp_reg = 0;

	if (reg_vdma_base_va == NULL)
		return 0;
	dmac_readw(reg_vdma_base_va + DMAC_GLOBLE_CTRL, tmp_reg);
	tmp_reg |= AUTO_CLK_GT_EN;
	dmac_writew(reg_vdma_base_va + DMAC_GLOBLE_CTRL, tmp_reg);

	/* set rd dust address is ram 0 */
	dmac_writew(reg_vdma_base_va + DMAC_RD_DUSTB_ADDR, 0x04c11000);

	/* set wr dust address is ram 0x1000 */
	dmac_writew(reg_vdma_base_va + DMAC_WR_DUSTB_ADDR, 0x04c11000);

	/* set prrr register */
	dmac_writew(reg_vdma_base_va + DMAC_MMU_PRRR, PRRR);
	/* set nmrr register */
	dmac_writew(reg_vdma_base_va + DMAC_MMU_NMRR, NMRR);

	/* config global reg for VDMA */
	tmp_reg |= EVENT_BROADCAST_EN | WR_CMD_NUM_PER_ARB |
		   RD_CMD_NUM_PER_ARB | WR_OTD_NUM | RD_OTD_NUM | WFE_EN;
	dmac_writew(reg_vdma_base_va + DMAC_GLOBLE_CTRL, tmp_reg);

	for (i = 0; i < CHANNEL_NUM; i++) {
		g_channel_status[i] = DMAC_CHN_VACANCY;
		init_waitqueue_head(&dmac_wait_queue[i]);
	}

	spin_lock_init(&my_lock);
	spin_lock_init(&reg_lock);

	return 0;
}

/*
 *  wait for transfer end
 */
int vdma_wait(unsigned int channel)
{
	unsigned long data_jiffies_timeout = jiffies + DMA_TIMEOUT_HZ;

	while (1) {
		unsigned int channel_intr_raw;
		if (reg_vdma_base_va == NULL)
			return 0;
		/* read the status of current interrupt */
		dmac_readw(reg_vdma_base_va + dmac_cxintr_raw(channel),
			   channel_intr_raw);

		/* clear the interrupt */
		dmac_writew(reg_vdma_base_va
			    + dmac_cxintr_raw(channel),
			    channel_intr_raw);

		if (channel >= DMAC_MAX_CHANNELS)
			return 0;
		/* save the current channel transfer status to *
		 * g_channel_status[i] */
		if ((channel_intr_raw & CX_INT_STAT) == CX_INT_STAT) {
			/* transfer finish interrupt */
			if ((channel_intr_raw & CX_INT_TC_RAW) ==
			    CX_INT_TC_RAW) {
				g_channel_status[channel] = DMAC_CHN_SUCCESS;
				return DMAC_CHN_SUCCESS;
			}

			/* transfer abort interrupt */
			pr_debug("data transfer error in VDMA %x channel!",
				 channel);
			pr_debug("intr_raw=%x\n", channel_intr_raw);
			g_channel_status[channel] =
				-DMAC_CHN_CONFIG_ERROR;
			return -DMAC_CHN_CONFIG_ERROR;
		}

		if (!time_before(jiffies, data_jiffies_timeout)) { /* timeout */
			pr_err("wait interrupt timeout, channel=%d, func:%s, line:%d\n",
			       channel, __func__, __LINE__);
			return -1;
		}
	}

	return -1;
}

/*
 *  execute memory to memory vdma transfer
 */
static int vdma_m2m_transfer(const unsigned int *psource,
				const unsigned int *pdest,
				unsigned int uwtransfersize)
{
	unsigned int ulchnn;
	int ret = 0;

	ulchnn = (unsigned int)vdma_channel_allocate(NULL);
	if (ulchnn == DMAC_CHANNEL_INVALID) {
		pr_err("DMAC_CHANNEL_INVALID.\n");

		return -1;
	}

	wake_channel_flag[ulchnn] = 0;

	dmac_writew(reg_vdma_base_va + dmac_cxlength(ulchnn),
		    uwtransfersize);
	dmac_writew(reg_vdma_base_va + dmac_cxsrcaddr(ulchnn),
		    (uintptr_t)psource);
	dmac_writew(reg_vdma_base_va + dmac_cxdestaddr(ulchnn),
		    (uintptr_t)pdest);

	if (vdma_channelstart(ulchnn, psource, pdest) != 0) {
		ret = -1;
		goto exit;
	}

	if (vdma_wait(ulchnn) != DMAC_CHN_SUCCESS) {
		ret = -1;
		goto exit;
	}

exit:
	vdma_channel_free(ulchnn);

	return ret;
}
EXPORT_SYMBOL(vdma_m2m_transfer);

int vdma_m2m_copy(void *dst, const void *src, size_t count)
{
	int ret;

	if (((uintptr_t)dst & 0xff) || ((uintptr_t)src & 0xff))
		return -1;

	if (((uintptr_t)src < (uintptr_t)dst) &&
	    (((uintptr_t)src + count) > (uintptr_t)dst))
		return -1;

	if (((uintptr_t)src >= (uintptr_t)dst) &&
	    ((uintptr_t)src < ((uintptr_t)dst + count)))
		return -1;

	if (abs((uintptr_t)dst - (uintptr_t)src) < PAGE_SIZE)
		return -1;

	ret = vdma_m2m_transfer((unsigned int *)src, dst, count);
	if (ret < 0)
		return ret;
	else
		return 0;
}
EXPORT_SYMBOL(vdma_m2m_copy);

static int vdmac_probe(struct platform_device *platdev)
{
	unsigned int i;
	struct vdmac_host *dma = NULL;
	struct resource *res = NULL;
	int ret;
	dma = devm_kzalloc(&platdev->dev, sizeof(*dma), GFP_KERNEL);
	if (!dma)
		return -ENOMEM;

	res = platform_get_resource(platdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&platdev->dev, "no mmio resource\n");
		return -ENODEV;
	}

	dma->regbase = devm_ioremap_resource(&platdev->dev, res);
	if (IS_ERR(dma->regbase))
		return PTR_ERR(dma->regbase);

	dma->clk = devm_clk_get(&platdev->dev, NULL);
	if (IS_ERR(dma->clk))
		return PTR_ERR(dma->clk);

	clk_prepare_enable(dma->clk);

	dma->rstc = devm_reset_control_get(&platdev->dev, "dma-reset");
	if (IS_ERR(dma->rstc))
		return PTR_ERR(dma->rstc);

	dma->irq = platform_get_irq(platdev, 0);
	if (unlikely(dma->irq < 0))
		return -ENODEV;

	reg_vdma_base_va = dma->regbase;
	pr_debug("vdma reg base is %p\n", reg_vdma_base_va);
	dma->dev = &platdev->dev;

	ret = vdma_driver_init(dma);
	if (ret)
		return -ENODEV;

	platform_set_drvdata(platdev, dma);

	for (i = 0; i < CONFIG_VDMA_CHN_NUM; i++)
		g_channel_status[i] = DMAC_CHN_VACANCY;

	vdma_flag = 1;
	printk("vdmav100 driver inited.\n");
	return ret;
}

static int vdmac_remove(struct platform_device *platdev)
{
	int i;
	struct vdmac_host *dma = platform_get_drvdata(platdev);

	clk_disable_unprepare(dma->clk);

	for (i = 0; i < CONFIG_VDMA_CHN_NUM; i++)
		g_channel_status[i] = DMAC_CHN_VACANCY;

	vdma_flag = 0;
	printk("vdmav100 driver deinited.\n");

	return 0;
}

static int vdmac_suspend(struct platform_device *platdev,
			   pm_message_t state)
{
	int i;
	struct vdmac_host *dma = platform_get_drvdata(platdev);

	clk_prepare_enable(dma->clk);

	for (i = 0; i < CONFIG_VDMA_CHN_NUM; i++)
		g_channel_status[i] = DMAC_CHN_VACANCY;

	clk_disable_unprepare(dma->clk);

	vdma_flag = 0;

	return 0;
}

static int vdmac_resume(struct platform_device *platdev)
{
	int i;
	struct vdmac_host *dma = platform_get_drvdata(platdev);

	vdma_driver_init(dma);

	for (i = 0; i < CONFIG_VDMA_CHN_NUM; i++)
		g_channel_status[i] = DMAC_CHN_VACANCY;

	vdma_flag = 1;

	return 0;
}

static const struct of_device_id vdmac_dt_ids[] = {
	{.compatible = "vendor,vdmac"},
	{ },
};
MODULE_DEVICE_TABLE(of, vdmac_dt_ids);

static struct platform_driver vdmac_driver = {
	.driver = {
		.name   = "vdma",
		.of_match_table = vdmac_dt_ids,
	},
	.probe      = vdmac_probe,
	.remove     = vdmac_remove,
	.suspend    = vdmac_suspend,
	.resume     = vdmac_resume,
};

module_platform_driver(vdmac_driver);

MODULE_LICENSE("GPL");
