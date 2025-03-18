/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/export.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_dma.h>
#include <linux/pm_runtime.h>
#include <linux/seq_file.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/edmac.h>

#include "edmacv310.h"
#include <linux/securec.h>

#ifdef CONFIG_ARCH_HI3519DV500_FAMILY
#include "edma_hi3519dv500.h"
#endif

int g_channel_status[EDMAC_CHANNEL_NUM];
DMAC_ISR *function[EDMAC_CHANNEL_NUM];
unsigned long pllihead[2] = {0, 0};
void __iomem *dma_regbase;
int edmacv310_trace_level_n = EDMACV310_TRACE_LEVEL;

struct edmac_host {
	struct platform_device *pdev;
	void __iomem *base;
	struct regmap *misc_regmap;
	unsigned int misc_ctrl_base;
	void __iomem *crg_ctrl;
	unsigned int id;
	struct clk *clk;
	struct clk *axi_clk;
	int irq;
	struct reset_control *rstc;
	unsigned int channels;
	unsigned int slave_requests;
};

#define DRIVER_NAME "edmacv310"

int dmac_channel_allocate(void)
{
	unsigned int i;

	for (i = 0; i < EDMAC_CHANNEL_NUM; i++) {
		if (g_channel_status[i] == DMAC_CHN_VACANCY) {
			g_channel_status[i] = DMAC_CHN_ALLOCAT;
			return i;
		}
	}

	edmacv310_error("no to alloc\n");
	return -1;
}
EXPORT_SYMBOL(dmac_channel_allocate);

static void edmac_read_err_status(unsigned int *channel_err_status,
				    const unsigned int err_status_len)
{
	if (err_status_len < EDMAC_ERR_REG_NUM) {
		edmacv310_error("channel_err_status size err.\n");
		return;
	}
	channel_err_status[0] = edmacv310_readl(dma_regbase + EDMAC_INT_ERR1);
	channel_err_status[1] = edmacv310_readl(dma_regbase + EDMAC_INT_ERR2);
	channel_err_status[2] = edmacv310_readl(dma_regbase + EDMAC_INT_ERR3);
}

static void edmac_err_status_filter(unsigned int *channel_err_status,
				      const unsigned int err_status_len,
				      unsigned int curr_channel)
{
	if (err_status_len < EDMAC_ERR_REG_NUM) {
		edmacv310_error("channel_err_status size err.\n");
		return;
	}
	channel_err_status[0] = (channel_err_status[0] >> curr_channel) & 0x01;
	channel_err_status[1] = (channel_err_status[1] >> curr_channel) & 0x01;
	channel_err_status[2] = (channel_err_status[2] >> curr_channel) & 0x01;
}

static void edmac_clear_err_status(unsigned int curr_channel)
{
	edmacv310_writel(1 << curr_channel, dma_regbase + EDMAC_INT_ERR1_RAW);
	edmacv310_writel(1 << curr_channel, dma_regbase + EDMAC_INT_ERR2_RAW);
	edmacv310_writel(1 << curr_channel, dma_regbase + EDMAC_INT_ERR3_RAW);
}

/*
 *	update the state of channels
 */
static int edmac_update_status(unsigned int channel)
{
	unsigned int channel_tc_status;
	unsigned int channel_err_status[EDMAC_ERR_REG_NUM];
	unsigned int i = channel;
	unsigned long update_jiffies_timeout;

	update_jiffies_timeout = jiffies + EDMAC_UPDATE_TIMEOUT;
	while (1) {
		unsigned int channel_status;
		channel_status = edmacv310_readl(dma_regbase + EDMAC_INT_STAT);
		channel_status = (channel_status >> i) & 0x01;
		if (channel_status) {
			channel_tc_status = edmacv310_readl(dma_regbase + EDMAC_INT_TC1);
			channel_tc_status = (channel_tc_status >> i) & 0x01;
			if (channel_tc_status) {
				edmacv310_writel(1 << i, dma_regbase + EDMAC_INT_TC1_RAW);
				g_channel_status[i] = DMAC_CHN_SUCCESS;
				break;
			}

			channel_tc_status = edmacv310_readl(dma_regbase + EDMAC_INT_TC2);
			channel_tc_status = (channel_tc_status >> i) & 0x01;
			if (channel_tc_status) {
				edmacv310_writel(1 << i, dma_regbase + EDMAC_INT_TC2_RAW);
				g_channel_status[i] = DMAC_CHN_SUCCESS;
				break;
			}

			edmac_read_err_status(channel_err_status, EDMAC_ERR_REG_NUM);
			edmac_err_status_filter(channel_err_status, EDMAC_ERR_REG_NUM, i);

			if (channel_err_status[0] | channel_err_status[1] | channel_err_status[2]) {
				edmacv310_error("Error in EDMAC %d finish!\n", i);
				edmac_read_err_status(channel_err_status, EDMAC_ERR_REG_NUM);
				edmac_clear_err_status(i);
				g_channel_status[i] = -DMAC_CHN_ERROR;
				break;
			}
		}

		if (!time_before(jiffies, update_jiffies_timeout)) {
			edmacv310_error("Timeout in DMAC %d!\n", i);
			g_channel_status[i] = -DMAC_CHN_TIMEOUT;
			break;
		}
	}
	return g_channel_status[i];
}

/*
 * register user's function
 */
int dmac_register_isr(unsigned int channel, void *pisr)
{
	if (channel > EDMAC_CHANNEL_NUM - 1) {
		edmacv310_error("invalid channel,channel=%0d\n", channel);
		return -EINVAL;
	}

	function[channel] = (void *)pisr;

	return 0;
}
EXPORT_SYMBOL(dmac_register_isr);

/*
 *	free channel
 */
int dmac_channel_free(unsigned int channel)
{
	if (channel < EDMAC_CHANNEL_NUM)
		g_channel_status[channel] = DMAC_CHN_VACANCY;

	return 0;
}
EXPORT_SYMBOL(dmac_channel_free);

static unsigned int dmac_check_request(unsigned int peripheral_addr,
				       int direction)
{
	int i;

	for (i = direction; i < EDMAC_MAX_PERIPHERALS; i += 2) {
		if (g_peripheral[i].peri_addr == peripheral_addr)
			return i;
	}
	edmacv310_error("Invalid devaddr\n");
	return -1;
}

void edmac_channel_free(int channel)
{
	if ((channel >= 0) && (channel < EDMAC_CHANNEL_NUM))
		g_channel_status[channel] = DMAC_CHN_VACANCY;
}
/*
 *	wait for transfer end
 */
int dmac_wait(int channel)
{
	int ret_result;
	int ret = 0;

	if (channel < 0)
		return -1;

	while (1) {
		ret_result = edmac_update_status(channel);
		if (ret_result == -DMAC_CHN_ERROR) {
			edmacv310_error("Transfer Error.\n");
			ret = -1;
			goto end;
		} else  if (ret_result == DMAC_NOT_FINISHED) {
			udelay(DMAC_FINISHED_WAIT_TIME);
		} else if (ret_result == DMAC_CHN_SUCCESS) {
			ret = DMAC_CHN_SUCCESS;
			goto end;
		} else if (ret_result == DMAC_CHN_VACANCY) {
			ret = DMAC_CHN_SUCCESS;
			goto end;
		} else if (ret_result == -DMAC_CHN_TIMEOUT) {
			edmacv310_error("Timeout.\n");
			edmacv310_writel(EDMAC_CX_DISABLE,
					   dma_regbase + edmac_cx_config(channel));
			g_channel_status[channel] = DMAC_CHN_VACANCY;
			ret = -1;
			return ret;
		}
	}
end:
	edmacv310_writel(EDMAC_CX_DISABLE,
			   dma_regbase + edmac_cx_config(channel));
	edmac_channel_free(channel);
	return ret;
}
EXPORT_SYMBOL(dmac_wait);

/*
 *	execute memory to peripheral dma transfer without LLI
 */
int dmac_m2p_transfer(unsigned long long memaddr, unsigned int uwperipheralid,
		      unsigned int length)
{
	unsigned int ulchnn;
	unsigned int uwwidth;
	unsigned int temp;

	ulchnn = dmac_channel_allocate();
	if (-1 == ulchnn)
		return -1;

	edmacv310_trace(EDMACV310_TRACE_LEVEL_INFO, "ulchnn = %d\n", ulchnn);
	uwwidth = g_peripheral[uwperipheralid].transfer_width;
	if ((length >> uwwidth) >= EDMAC_TRANS_MAXSIZE) {
		edmacv310_error("The length is more than 64k!\n");
		return -1;
	}

	edmacv310_writel(memaddr & 0xffffffff,
			   dma_regbase + edmac_cx_src_addr_l(ulchnn));
#ifdef CONFIG_ARM64
	edmacv310_writel((memaddr >> EDMACV310_32BIT) & 0xffffffff,
			   dma_regbase + edmac_cx_src_addr_h(ulchnn));
#endif
	edmacv310_trace(EDMACV310_TRACE_LEVEL_INFO, "EDMAC_Cx_SRC_ADDR_L = 0x%x\n",
			  edmacv310_readl(dma_regbase + edmac_cx_src_addr_l(ulchnn)));

	edmacv310_writel(g_peripheral[uwperipheralid].peri_addr & 0xffffffff,
			   dma_regbase + edmac_cx_dest_addr_l(ulchnn));
#ifdef CONFIG_ARM64
	edmacv310_writel((g_peripheral[uwperipheralid].peri_addr >> EDMACV310_32BIT) & 0xffffffff,
			   dma_regbase + edmac_cx_dest_addr_h(ulchnn));
#endif
	edmacv310_trace(EDMACV310_TRACE_LEVEL_INFO, "EDMAC_Cx_DEST_ADDR_L = 0x%x\n",
			  edmacv310_readl(dma_regbase + edmac_cx_dest_addr_l(ulchnn)));

	edmacv310_writel(0, dma_regbase + edmac_cx_lli_l(ulchnn));
	edmacv310_trace(EDMACV310_TRACE_LEVEL_INFO, "EDMAC_Cx_LLI_L = 0x%x\n",
			  edmacv310_readl(dma_regbase + edmac_cx_lli_l(ulchnn)));

	edmacv310_writel(length, dma_regbase + edmac_cx_cnt0(ulchnn));
	edmacv310_trace(EDMACV310_TRACE_LEVEL_INFO, "EDMAC_Cx_CNT0 = 0x%x\n",
			  edmacv310_readl(dma_regbase + edmac_cx_cnt0(ulchnn)));

	temp = g_peripheral[uwperipheralid].transfer_cfg | (uwwidth << EDMA_SRC_WIDTH_OFFSET) |
	       (uwperipheralid << PERI_ID_OFFSET) | EDMA_CH_ENABLE;
	edmacv310_trace(EDMACV310_TRACE_LEVEL_INFO, "EDMAC_Cx_CONFIG = 0x%x\n", temp);
	edmacv310_writel(temp, dma_regbase + edmac_cx_config(ulchnn));
	return ulchnn;
}

/*
 *	execute memory to peripheral dma transfer without LLI
 */
int dmac_p2m_transfer(unsigned long memaddr, unsigned int uwperipheralid,
		      unsigned int length)
{
	unsigned int ulchnn;
	unsigned int uwwidth;
	unsigned int temp;

	ulchnn = dmac_channel_allocate();
	if (-1 == ulchnn)
		return -1;

	edmacv310_trace(EDMACV310_TRACE_LEVEL_INFO, "ulchnn = %d\n", ulchnn);
	uwwidth = g_peripheral[uwperipheralid].transfer_width;
	if ((length >> uwwidth) >= EDMAC_TRANS_MAXSIZE) {
		edmacv310_error("The length is more than 64k!\n");
		return -1;
	}

	edmacv310_writel(memaddr & 0xffffffff,
			   dma_regbase + edmac_cx_dest_addr_l(ulchnn));
#ifdef CONFIG_ARM64
	edmacv310_writel((memaddr >> EDMACV310_32BIT) & 0xffffffff,
			   dma_regbase + edmac_cx_dest_addr_h(ulchnn));
#endif
	edmacv310_trace(EDMACV310_TRACE_LEVEL_INFO, "EDMAC_Cx_DEST_ADDR_L = 0x%x\n",
			  edmacv310_readl(dma_regbase + edmac_cx_dest_addr_l(ulchnn)));

	edmacv310_writel(g_peripheral[uwperipheralid].peri_addr & 0xffffffff,
			   dma_regbase + edmac_cx_src_addr_l(ulchnn));
#ifdef CONFIG_ARM64
	edmacv310_writel(0, dma_regbase + edmac_cx_src_addr_h(ulchnn));
#endif
	edmacv310_trace(EDMACV310_TRACE_LEVEL_INFO, "EDMAC_Cx_SRC_ADDR_L = 0x%x\n",
			  edmacv310_readl(dma_regbase + edmac_cx_src_addr_l(ulchnn)));

	edmacv310_writel(0, dma_regbase + edmac_cx_lli_l(ulchnn));
	edmacv310_trace(EDMACV310_TRACE_LEVEL_INFO, "EDMAC_Cx_LLI_L = 0x%x\n",
			  edmacv310_readl(dma_regbase + edmac_cx_lli_l(ulchnn)));

	edmacv310_writel(length, dma_regbase + edmac_cx_cnt0(ulchnn));
	edmacv310_trace(EDMACV310_TRACE_LEVEL_INFO, "EDMAC_Cx_CNT0 = 0x%x\n",
			  edmacv310_readl(dma_regbase + edmac_cx_cnt0(ulchnn)));

	temp = g_peripheral[uwperipheralid].transfer_cfg | (uwwidth << EDMA_SRC_WIDTH_OFFSET) |
	       (uwperipheralid << PERI_ID_OFFSET) | EDMA_CH_ENABLE;
	edmacv310_trace(EDMACV310_TRACE_LEVEL_INFO, "EDMAC_Cx_CONFIG = 0x%x\n", temp);
	edmacv310_writel(temp, dma_regbase + edmac_cx_config(ulchnn));
	return ulchnn;
}

int do_dma_m2p(unsigned long long memaddr, unsigned int peripheral_addr,
	       unsigned int length)
{
	int ret;
	int uwperipheralid;

	uwperipheralid = dmac_check_request(peripheral_addr, EDMAC_TX);
	if (uwperipheralid < 0) {
		edmacv310_error("m2p:Invalid devaddr\n");
		return -1;
	}

	ret = dmac_m2p_transfer(memaddr, uwperipheralid, length);
	if (ret == -1) {
		edmacv310_error("m2p:trans err\n");
		return -1;
	}

	return ret;
}
EXPORT_SYMBOL(do_dma_m2p);

int do_dma_p2m(unsigned long memaddr, unsigned int peripheral_addr,
	       unsigned int length)
{
	int ret;
	int uwperipheralid;

	uwperipheralid = dmac_check_request(peripheral_addr, EDMAC_RX);
	if (uwperipheralid < 0) {
		edmacv310_error("p2m:Invalid devaddr.\n");
		return -1;
	}

	ret = dmac_p2m_transfer(memaddr, uwperipheralid, length);
	if (ret == -1) {
		edmacv310_error("p2m:trans err\n");
		return -1;
	}

	return ret;
}
EXPORT_SYMBOL(do_dma_p2m);

/*
 *	buile LLI for memory to memory DMA transfer
 */
int dmac_buildllim2m(const unsigned long *ppheadlli,
		     unsigned long psource,
		     unsigned long pdest,
		     unsigned int totaltransfersize,
		     unsigned int uwnumtransfers)
{
	int lli_num;
	unsigned long phy_address;
	int j;
	dmac_lli  *plli = NULL;

	if (uwnumtransfers == 0)
		return -EINVAL;

	lli_num = (totaltransfersize / uwnumtransfers);
	if ((totaltransfersize % uwnumtransfers) != 0)
		lli_num++;

	edmacv310_trace(EDMACV310_TRACE_LEVEL_INFO, "lli_num:%d\n", lli_num);

	phy_address = ppheadlli[0];
	plli = (dmac_lli *)ppheadlli[1];
	edmacv310_trace(EDMACV310_TRACE_LEVEL_INFO, "phy_address: 0x%lx\n", phy_address);
	edmacv310_trace(EDMACV310_TRACE_LEVEL_INFO, "address: 0x%p\n", plli);
	for (j = 0; j < lli_num; j++) {
		(void)memset_s(plli, sizeof(dmac_lli), 0x0, sizeof(dmac_lli));
		/*
		 * at the last transfer, chain_en should be set to 0x0;
		 * others tansfer,chain_en should be set to 0x2;
		 */
		plli->next_lli = (phy_address + (j + 1) * sizeof(dmac_lli)) &
				 (~(EDMAC_LLI_ALIGN - 1));
		if (j < lli_num - 1) {
			plli->next_lli |= EDMAC_LLI_ENABLE;
			plli->count = uwnumtransfers;
		} else {
			plli->next_lli |= EDMAC_LLI_DISABLE;
			plli->count = totaltransfersize % uwnumtransfers;
		}

		plli->src_addr = psource;
		plli->dest_addr = pdest;
		plli->config = EDMAC_CXCONFIG_M2M_LLI;

		psource += uwnumtransfers;
		pdest += uwnumtransfers;
		plli++;
	}

	return 0;
}
EXPORT_SYMBOL(dmac_buildllim2m);

/*
 *	load configuration from LLI for memory to memory
 */
int dmac_start_llim2m(unsigned int channel, const unsigned long *pfirst_lli)
{
	unsigned int i = channel;
	dmac_lli  *plli;

	plli = (dmac_lli *)pfirst_lli[1];
	edmacv310_trace(EDMACV310_TRACE_LEVEL_INFO, "plli.src_addr: 0x%lx\n", plli->src_addr);
	edmacv310_trace(EDMACV310_TRACE_LEVEL_INFO, "plli.dst_addr: 0x%lx\n", plli->dest_addr);
	edmacv310_trace(EDMACV310_TRACE_LEVEL_INFO, "plli.next_lli: 0x%lx\n", plli->next_lli);
	edmacv310_trace(EDMACV310_TRACE_LEVEL_INFO, "plli.count: 0x%d\n", plli->count);

	edmacv310_writel(plli->dest_addr & 0xffffffff,
			   dma_regbase + edmac_cx_lli_l(i));
#ifdef CONFIG_ARM64
	edmacv310_writel((plli->dest_addr >> EDMACV310_32BIT) & 0xffffffff,
			   dma_regbase + edmac_cx_lli_h(i));
#endif
	edmacv310_writel(plli->count, dma_regbase + edmac_cx_cnt0(i));

	edmacv310_writel(plli->src_addr & 0xffffffff,
			   dma_regbase + edmac_cx_src_addr_l(i));
#ifdef CONFIG_ARM64
	edmacv310_writel((plli->src_addr >> EDMACV310_32BIT) & 0xffffffff,
			   dma_regbase + edmac_cx_src_addr_h(i));
#endif
	edmacv310_writel(plli->dest_addr & 0xffffffff,
			   dma_regbase + edmac_cx_dest_addr_l(i));
#ifdef CONFIG_ARM64
	edmacv310_writel((plli->dest_addr >> EDMACV310_32BIT) & 0xffffffff,
			   dma_regbase + edmac_cx_dest_addr_h(i));
#endif
	edmacv310_writel(plli->config | EDMA_CH_ENABLE,
			   dma_regbase + edmac_cx_config(i));

	return 0;
}
EXPORT_SYMBOL(dmac_start_llim2m);

/*
 * config register for memory to memory DMA transfer without LLI
 */
int dmac_start_m2m(unsigned int  channel, unsigned long psource,
		   unsigned long pdest, unsigned int uwnumtransfers)
{
	unsigned int i = channel;

	if (uwnumtransfers > EDMAC_TRANS_MAXSIZE || uwnumtransfers == 0) {
		edmacv310_error("Invalidate transfer size,size=%x\n", uwnumtransfers);
		return -EINVAL;
	}
	edmacv310_trace(EDMACV310_TRACE_LEVEL_INFO, "channel[%d],source=0x%lx,dest=0x%lx,length=%d\n",
			  channel, psource, pdest, uwnumtransfers);

	edmacv310_writel(psource & 0xffffffff,
			   dma_regbase + edmac_cx_src_addr_l(i));
	edmacv310_trace(EDMACV310_TRACE_LEVEL_INFO, "EDMAC_Cx_SRC_ADDR_L = 0x%x\n",
			  edmacv310_readl(dma_regbase + edmac_cx_src_addr_l(i)));
#ifdef CONFIG_ARM64
	edmacv310_writel((psource >> EDMACV310_32BIT) & 0xffffffff,
			   dma_regbase + edmac_cx_src_addr_h(i));
	edmacv310_trace(EDMACV310_TRACE_LEVEL_INFO, "EDMAC_Cx_SRC_ADDR_H = 0x%x\n",
			  edmacv310_readl(dma_regbase + edmac_cx_src_addr_h(i)));
#endif
	edmacv310_writel(pdest & 0xffffffff, dma_regbase + edmac_cx_dest_addr_l(i));
	edmacv310_trace(EDMACV310_TRACE_LEVEL_INFO, "EDMAC_Cx_DEST_ADDR_L = 0x%x\n",
			  edmacv310_readl(dma_regbase + edmac_cx_dest_addr_l(i)));
#ifdef CONFIG_ARM64
	edmacv310_writel((pdest >> EDMACV310_32BIT) & 0xffffffff,
			   dma_regbase + edmac_cx_dest_addr_h(i));
	edmacv310_trace(EDMACV310_TRACE_LEVEL_INFO, "EDMAC_Cx_DEST_ADDR_H = 0x%x\n",
			  edmacv310_readl(dma_regbase + edmac_cx_dest_addr_h(i)));
#endif
	edmacv310_writel(0, dma_regbase + edmac_cx_lli_l(i));

	edmacv310_writel(uwnumtransfers, dma_regbase + edmac_cx_cnt0(i));

	edmacv310_writel(EDMAC_CXCONFIG_M2M | EDMA_CH_ENABLE,
			   dma_regbase + edmac_cx_config(i));

	return 0;
}
EXPORT_SYMBOL(dmac_start_m2m);

/*
 *	execute memory to memory dma transfer without LLI
 */
int dmac_m2m_transfer(unsigned long source, unsigned long dest,
		      unsigned int length)
{
	unsigned int dma_size, dma_count, left_size;
	int ulchnn;

	left_size = length;
	dma_count = 0;
	ulchnn = dmac_channel_allocate();
	if (ulchnn < 0)
		return -EINVAL;

	edmacv310_trace(EDMACV310_TRACE_LEVEL_DEBUG, "using channel[%d],source=0x%lx,dest=0x%lx,length=%d\n",
			  ulchnn, source, dest, length);

	while (left_size) {
		if (left_size >= EDMAC_TRANS_MAXSIZE)
			dma_size = EDMAC_TRANS_MAXSIZE;
		else
			dma_size = left_size;
		if (dmac_start_m2m(ulchnn, source + dma_count * dma_size,
			       dest + dma_count * dma_size, dma_size)) {
			edmacv310_error("dma transfer error...\n");
			return -1;
		}

		if (dmac_wait(ulchnn) != DMAC_CHN_SUCCESS) {
			edmacv310_error("dma transfer error...\n");
			return -1;
		}
		left_size -= dma_size;
		dma_count++;
		edmacv310_trace(EDMACV310_TRACE_LEVEL_INFO, "left_size is %d.\n", left_size);
	}

	return 0;
}
EXPORT_SYMBOL(dmac_m2m_transfer);

/*
 * memory to memory dma transfer with LLI
 *
 * @source
 * @dest
 * @length
 * */
int do_dma_llim2m(unsigned long source,
		  unsigned long dest,
		  unsigned long length)
{
	int ret = 0;
	int chnn;

	chnn = dmac_channel_allocate();
	if (chnn < 0) {
		ret = -1;
		goto end;
	}
	edmacv310_trace(EDMACV310_TRACE_LEVEL_INFO, "chnn:%d,src:%lx,dst:%lx,len:%ld.\n", chnn, source, dest,
			  length);

	if (pllihead[0] == 0) {
		edmacv310_error("ppheadlli[0] is NULL.\n");
		ret = -ENOMEM;
		goto end;
	}

	ret = dmac_buildllim2m(pllihead, source, dest, length, EDMAC_TRANS_MAXSIZE);
	if (ret) {
		edmacv310_error("build lli error...\n");
		ret = -EIO;
		goto end;
	}
	ret = dmac_start_llim2m(chnn, pllihead);
	if (ret) {
		edmacv310_error("start lli error...\n");
		ret = -EIO;
		goto end;
	}

end:
	return ret;
}
EXPORT_SYMBOL(do_dma_llim2m);

/*
 *	alloc_dma_lli_space
 *	output:
 *             ppheadlli[0]: memory physics address
 *             ppheadlli[1]: virtual address
 *
 */
int allocate_dmalli_space(struct device *dev, unsigned long *ppheadlli,
			  unsigned int page_num)
{
	dma_addr_t dma_phys;
	void *dma_virt;

	dma_virt = dma_alloc_coherent(dev, page_num * PAGE_SIZE,
				      &dma_phys, GFP_DMA);
	if (dma_virt == NULL) {
		edmacv310_error("can't get dma mem from system\n");
		return -1;
	}

	ppheadlli[0] = (unsigned long)(dma_phys);
	ppheadlli[1] = (unsigned long)(dma_virt);

	if (dma_phys & (EDMAC_LLI_ALIGN - 1))
		return -1;

	return 0;
}
EXPORT_SYMBOL(allocate_dmalli_space);

static int edmac_priv_init(struct edmac_host *edmac,
			     edmac_peripheral *peripheral_info)
{
	struct regmap *misc = edmac->misc_regmap;
	int i;
	unsigned int count = 0;
	unsigned int offset;
	unsigned ctrl = 0;

	for (i = 0; i < EDMAC_MAX_PERIPHERALS; i++) {
		if (peripheral_info[i].host_sel == edmac->id) {
			if (misc != NULL) {
				offset = edmac->misc_ctrl_base + (count & (~0x3)); /* config misc reg for signal line */
				regmap_read(misc, offset, &ctrl);
				ctrl &= ~(0x3f << ((count & 0x3) << 3));
				ctrl |= peripheral_info[i].peri_id << ((count & 0x3) << 3);
				regmap_write(misc, offset, ctrl);
			}
			peripheral_info[i].dynamic_periphery_num = count;
			count++;
		}
	}

	return 0;
}

static int of_probe_read(struct edmac_host *edmac, struct device_node *np)
{
	int ret;

	ret = of_property_read_u32(np,
				   "devid", &(edmac->id));
	if (ret) {
		edmacv310_error("get edmac id fail\n");
		return -ENODEV;
	}

	if (!of_find_property(np, "misc_regmap", NULL) ||
	    !of_find_property(np, "misc_ctrl_base", NULL)) {
		edmac->misc_regmap = 0;
	} else {
		edmac->misc_regmap = syscon_regmap_lookup_by_phandle(np, "misc_regmap");
		if (IS_ERR(edmac->misc_regmap)) {
			edmacv310_error("get edmac misc fail\n");
			return PTR_ERR(edmac->misc_regmap);
		}

		ret = of_property_read_u32(np,
					   "misc_ctrl_base", &(edmac->misc_ctrl_base));
		if (ret) {
			edmacv310_error("get dma-misc_ctrl_base fail\n");
			return -ENODEV;
		}
	}
	ret = of_property_read_u32(np,
				   "dma-channels", &(edmac->channels));
	if (ret) {
		edmacv310_error("get dma-channels fail\n");
		return -ENODEV;
	}
	ret = of_property_read_u32(np,
				   "dma-requests", &(edmac->slave_requests));
	if (ret) {
		edmacv310_error("get dma-requests fail\n");
		return -ENODEV;
	}
	edmacv310_trace(EDMACV310_TRACE_LEVEL_INFO, "dma-channels = %d, dma-requests = %d\n",
			  edmac->channels, edmac->slave_requests);
	return 0;
}

static int of_probe_get_resource(struct edmac_host *edmac,
				 struct platform_device *platdev)
{
	struct resource *res = NULL;
	edmac->clk = devm_clk_get(&(platdev->dev), "apb_pclk");
	if (IS_ERR(edmac->clk)) {
		edmacv310_error("get edmac clk fail\n");
		return PTR_ERR(edmac->clk);
	}

	edmac->axi_clk = devm_clk_get(&(platdev->dev), "axi_aclk");
	if (IS_ERR(edmac->axi_clk)) {
		edmacv310_error("get edmac axi clk fail\n");
		return PTR_ERR(edmac->axi_clk);
	}

	edmac->rstc = devm_reset_control_get(&(platdev->dev), "dma-reset");
	if (IS_ERR(edmac->rstc)) {
		edmacv310_error("get edmac rstc fail\n");
		return PTR_ERR(edmac->rstc);
	}

	res = platform_get_resource(platdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		edmacv310_error("no reg resource\n");
		return -ENODEV;
	}

	edmac->base = devm_ioremap_resource(&(platdev->dev), res);
	if (IS_ERR(edmac->base)) {
		edmacv310_error("get edmac base fail\n");
		return PTR_ERR(edmac->base);
	}

	res = platform_get_resource_byname(platdev, IORESOURCE_MEM, "dma_peri_channel_req_sel");
	if (res != NULL) {
		void *dma_peri_channel_req_sel = ioremap(res->start, res->end - res->start);
		if (IS_ERR(dma_peri_channel_req_sel))
			return PTR_ERR(dma_peri_channel_req_sel);
		writel(0xffffffff, dma_peri_channel_req_sel);
		iounmap(dma_peri_channel_req_sel);
	}
	return 0;
}

static int get_of_probe(struct edmac_host *edmac)
{
	struct platform_device *platdev = edmac->pdev;
	struct device_node *np = platdev->dev.of_node;
	int ret;

	ret = of_probe_read(edmac, np);
	if (ret)
		return ret;

	ret = of_probe_get_resource(edmac, platdev);
	if (ret)
		return ret;

	edmac->irq = platform_get_irq(platdev, 0);
	if (unlikely(edmac->irq < 0))
		return -ENODEV;

	edmac_priv_init(edmac, (edmac_peripheral *)&g_peripheral);
	return 0;
}

/* Don't need irq mode now */
#if defined(CONFIG_EDMAC_INTERRUPT)
static irqreturn_t emdacv310_irq(int irq, void *dev)
{
	struct edmac_host *edmac = (struct edmac_host *)dev;
	unsigned int channel_err_status[3];
	unsigned int channel_tc_status, channel_status;
	int i;
	unsigned int mask = 0;

	channel_status = edmacv310_readl(edmac->base + EDMAC_INT_STAT);
	if (!channel_status) {
		edmacv310_error("channel_status = 0x%x\n", channel_status);
		return IRQ_NONE;
	}

	for (i = 0; i < edmac->channels; i++) {
		channel_status = (channel_status >> i) & 0x1;
		if (channel_status) {
			channel_tc_status = edmacv310_readl(edmac->base + EDMAC_INT_TC1_RAW);
			channel_tc_status = (channel_tc_status >> i) & 0x01;
			if (channel_tc_status)
				edmacv310_writel(channel_tc_status << i, edmac->base + EDMAC_INT_TC1_RAW);

			channel_tc_status = edmacv310_readl(edmac->base + EDMAC_INT_TC2);
			channel_tc_status = (channel_tc_status >> i) & 0x01;
			if (channel_tc_status)
				edmacv310_writel(channel_tc_status << i, edmac->base + EDMAC_INT_TC2_RAW);

			channel_err_status[0] = edmacv310_readl(edmac->base + EDMAC_INT_ERR1);
			channel_err_status[0] = (channel_err_status[0] >> i) & 0x01;
			channel_err_status[1] = edmacv310_readl(edmac->base + EDMAC_INT_ERR2);
			channel_err_status[1] = (channel_err_status[1] >> i) & 0x01;
			channel_err_status[2] = edmacv310_readl(edmac->base + EDMAC_INT_ERR3);
			channel_err_status[2] = (channel_err_status[2] >> i) & 0x01;

			if (channel_err_status[0] | channel_err_status[1] | channel_err_status[2]) {
				edmacv310_error("Error in edmac %d finish!,ERR1 = 0x%x,ERR2 = 0x%x,ERR3 = 0x%x\n",
						  i, channel_err_status[0], channel_err_status[1], channel_err_status[2]);
				edmacv310_writel(1 << i, edmac->base + EDMAC_INT_ERR1_RAW);
				edmacv310_writel(1 << i, edmac->base + EDMAC_INT_ERR2_RAW);
				edmacv310_writel(1 << i, edmac->base + EDMAC_INT_ERR3_RAW);
			}
			if ((function[i]) != NULL)
				function[i](i, g_channel_status[i]);

			mask |= (1 << i);
			edmacv310_writel(EDMAC_CX_DISABLE, dma_regbase + edmac_cx_config(i));
			edmac_channel_free(i);
		}
	}

	return mask ? IRQ_HANDLED : IRQ_NONE;
}
#endif

static void edmac310_dev_init(const struct edmac_host *edmac)
{
	clk_prepare_enable(edmac->clk);
	clk_prepare_enable(edmac->axi_clk);

	reset_control_deassert(edmac->rstc);

	edmacv310_writel(EDMAC_ALL_CHAN_CLR, edmac->base + EDMAC_INT_TC1_RAW);
	edmacv310_writel(EDMAC_ALL_CHAN_CLR, edmac->base + EDMAC_INT_TC2_RAW);
	edmacv310_writel(EDMAC_ALL_CHAN_CLR, edmac->base + EDMAC_INT_ERR1_RAW);
	edmacv310_writel(EDMAC_ALL_CHAN_CLR, edmac->base + EDMAC_INT_ERR2_RAW);
	edmacv310_writel(EDMAC_ALL_CHAN_CLR, edmac->base + EDMAC_INT_ERR3_RAW);

	edmacv310_writel(EDMAC_INT_ENABLE_ALL_CHAN,
			   edmac->base + EDMAC_INT_TC1_MASK);
	edmacv310_writel(EDMAC_INT_ENABLE_ALL_CHAN,
			   edmac->base + EDMAC_INT_TC2_MASK);
	edmacv310_writel(EDMAC_INT_ENABLE_ALL_CHAN,
			   edmac->base + EDMAC_INT_ERR1_MASK);
	edmacv310_writel(EDMAC_INT_ENABLE_ALL_CHAN,
			   edmac->base + EDMAC_INT_ERR2_MASK);
	edmacv310_writel(EDMAC_INT_ENABLE_ALL_CHAN,
			   edmac->base + EDMAC_INT_ERR3_MASK);
}

static int __init edmacv310_probe(struct platform_device *pdev)
{
	int ret = 0;
	int i = 0;
	struct edmac_host *edmac = NULL;

	edmac = kzalloc(sizeof(*edmac), GFP_KERNEL);
	if (!edmac) {
		edmacv310_error("malloc for edmac fail!");
		ret = -ENOMEM;
		return ret;
	}
	edmac->pdev = pdev;

	ret = get_of_probe(edmac);
	if (ret) {
		edmacv310_error("get dts info fail!");
		goto free_edmac;
	}

	for (i = 0; i < EDMAC_CHANNEL_NUM; i++)
		g_channel_status[i] = DMAC_CHN_VACANCY;

	dma_regbase = edmac->base;

	ret = allocate_dmalli_space(&(edmac->pdev->dev), pllihead,
				    EDMAC_LLI_PAGE_NUM);
	if (ret < 0)
		goto free_edmac;

#if defined(CONFIG_EDMAC_INTERRUPT)
	/* register irq if necessary !*/
	ret = request_irq(edmac->irq, emdacv310_irq, 0, DRIVER_NAME, edmac);
	if (ret) {
		edmacv310_error("fail to request irq");
		goto free_edmac;
	}
#endif
	edmac310_dev_init(edmac);
	return 0;

free_edmac:
	kfree(edmac);

	return ret;
}

static int emda_remove(struct platform_device *pdev)
{
	int err = 0;
	return err;
}

static const struct of_device_id edmacv310_match[] = {
	{ .compatible = "vendor,edmacv310_n" },
	{},
};

static struct platform_driver edmacv310_driver = {
	.remove = emda_remove,
	.driver = {
		.name   = "edmacv310_n",
		.of_match_table = edmacv310_match,
	},
};

static int __init edmacv310_init(void)
{
	return platform_driver_probe(&edmacv310_driver, edmacv310_probe);
}
subsys_initcall(edmacv310_init);

static void __exit edmacv310_exit(void)
{
	platform_driver_unregister(&edmacv310_driver);
}
module_exit(edmacv310_exit);

MODULE_LICENSE("GPL");
