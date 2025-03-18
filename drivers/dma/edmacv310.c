/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/dmaengine.h>
#include <linux/dmapool.h>
#include <linux/dma-mapping.h>
#include <linux/export.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_dma.h>
#include <linux/pm_runtime.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>

#include "edmacv310.h"
#include "dmaengine.h"
#include "virt-dma.h"
#include <linux/securec.h>

#define DRIVER_NAME "edmacv310"

#define MAX_TSFR_LLIS           512
#define EDMACV300_LLI_WORDS     64
#define EDMACV300_POOL_ALIGN    64
#define BITS_PER_HALF_WORD 32
#define ERR_STATUS_REG_NUM 3

typedef struct edmac_lli {
	u64 next_lli;
	u32 reserved[5];
	u32 count;
	u64 src_addr;
	u64 dest_addr;
	u32 config;
	u32 pad[3];
} edmac_lli;

struct edmac_sg {
	dma_addr_t src_addr;
	dma_addr_t dst_addr;
	size_t len;
	struct list_head node;
};

struct transfer_desc {
	struct virt_dma_desc virt_desc;
	dma_addr_t llis_busaddr;
	u64 *llis_vaddr;
	u32 ccfg;
	size_t size;
	bool done;
	bool cyclic;
};

enum edmac_dma_chan_state {
	EDMAC_CHAN_IDLE,
	EDMAC_CHAN_RUNNING,
	EDMAC_CHAN_PAUSED,
	EDMAC_CHAN_WAITING,
};

struct edmacv310_dma_chan {
	bool slave;
	int signal;
	int id;
	struct virt_dma_chan virt_chan;
	struct edmacv310_phy_chan *phychan;
	struct dma_slave_config cfg;
	struct transfer_desc *at;
	struct edmacv310_driver_data *host;
	enum edmac_dma_chan_state state;
};

struct edmacv310_phy_chan {
	unsigned int id;
	void __iomem *base;
	spinlock_t lock;
	struct edmacv310_dma_chan *serving;
};

struct edmacv310_driver_data {
	struct platform_device *dev;
	struct dma_device slave;
	struct dma_device memcpy;
	void __iomem *base;
	struct regmap *misc_regmap;
	void __iomem *crg_ctrl;
	struct edmacv310_phy_chan *phy_chans;
	struct dma_pool *pool;
	unsigned int misc_ctrl_base;
	int irq;
	unsigned int id;
	struct clk *clk;
	struct clk *axi_clk;
	struct reset_control *rstc;
	unsigned int channels;
	unsigned int slave_requests;
	unsigned int max_transfer_size;
};

#ifdef DEBUG_EDMAC
void dump_lli(const u64 *llis_vaddr, unsigned int num)
{
	edmac_lli *plli = (edmac_lli *)llis_vaddr;
	unsigned int i;

	edmacv310_trace(EDMACV310_CONFIG_TRACE_LEVEL, "lli num = 0%d\n", num);
	for (i = 0; i < num; i++) {
		printk("lli%d:lli_L:      0x%llx\n", i, plli[i].next_lli & 0xffffffff);
		printk("lli%d:lli_H:      0x%llx\n", i, (plli[i].next_lli >> BITS_PER_HALF_WORD) & 0xffffffff);
		printk("lli%d:count:      0x%x\n", i, plli[i].count);
		printk("lli%d:src_addr_L: 0x%llx\n", i, plli[i].src_addr & 0xffffffff);
		printk("lli%d:src_addr_H: 0x%llx\n", i, (plli[i].src_addr >> BITS_PER_HALF_WORD) & 0xffffffff);
		printk("lli%d:dst_addr_L: 0x%llx\n", i, plli[i].dest_addr & 0xffffffff);
		printk("lli%d:dst_addr_H: 0x%llx\n", i, (plli[i].dest_addr >> BITS_PER_HALF_WORD) & 0xffffffff);
		printk("lli%d:CONFIG:	  0x%x\n", i, plli[i].config);
	}
}

#else
void dump_lli(const u64 *llis_vaddr, unsigned int num)
{
}
#endif

static inline struct edmacv310_dma_chan *to_edamc_chan(const struct dma_chan *chan)
{
	return container_of(chan, struct edmacv310_dma_chan, virt_chan.chan);
}

static inline struct transfer_desc *to_edmac_transfer_desc(
	const struct dma_async_tx_descriptor *tx)
{
	return container_of(tx, struct transfer_desc, virt_desc.tx);
}

static struct dma_chan *edmac_find_chan_id(
	const struct edmacv310_driver_data *edmac,
	int request_num)
{
	struct edmacv310_dma_chan *edmac_dma_chan = NULL;

	list_for_each_entry(edmac_dma_chan, &edmac->slave.channels,
			    virt_chan.chan.device_node) {
		if (edmac_dma_chan->id == request_num)
			return &edmac_dma_chan->virt_chan.chan;
	}
	return NULL;
}

static struct dma_chan *edma_of_xlate(struct of_phandle_args *dma_spec,
					struct of_dma *ofdma)
{
	struct edmacv310_driver_data *edmac = ofdma->of_dma_data;
	struct edmacv310_dma_chan *edmac_dma_chan = NULL;
	struct dma_chan *dma_chan = NULL;
	struct regmap *misc = NULL;
	unsigned int signal, request_num;
	unsigned int reg = 0;
	unsigned int offset = 0;

	if (!edmac)
		return NULL;

	misc = edmac->misc_regmap;

	if (dma_spec->args_count != 2) { /* check num of dts node args */
		edmacv310_error("args count not true!\n");
		return NULL;
	}

	request_num = dma_spec->args[0];
	signal = dma_spec->args[1];

	edmacv310_trace(EDMACV310_CONFIG_TRACE_LEVEL, "host->id = %d,signal = %d, request_num = %d\n",
			  edmac->id, signal, request_num);

	if (misc != NULL) {
#ifdef CONFIG_ACCESS_M7_DEV
		offset = edmac->misc_ctrl_base;
		reg = 0xc0;
		regmap_write(misc, offset, reg);
#else
		offset = edmac->misc_ctrl_base + (request_num & (~0x3));
		regmap_read(misc, offset, &reg);
		/* set misc for signal line */
		reg &= ~(0x3f << ((request_num & 0x3) << 3));
		reg |= signal << ((request_num & 0x3) << 3);
		regmap_write(misc, offset, reg);
#endif
	}

	edmacv310_trace(EDMACV310_CONFIG_TRACE_LEVEL, "offset = 0x%x, reg = 0x%x\n", offset, reg);

	dma_chan = edmac_find_chan_id(edmac, request_num);
	if (!dma_chan) {
		edmacv310_error("DMA slave channel is not found!\n");
		return NULL;
	}

	edmac_dma_chan = to_edamc_chan(dma_chan);
	edmac_dma_chan->signal = request_num;
	return dma_get_slave_channel(dma_chan);
}

static int edmacv310_devm_get(struct edmacv310_driver_data *edmac)
{
	struct platform_device *platdev = edmac->dev;
	struct resource *res = NULL;

	edmac->clk = devm_clk_get(&(platdev->dev), "apb_pclk");
	if (IS_ERR(edmac->clk))
		return PTR_ERR(edmac->clk);

	edmac->axi_clk = devm_clk_get(&(platdev->dev), "axi_aclk");
	if (IS_ERR(edmac->axi_clk))
		return PTR_ERR(edmac->axi_clk);

	edmac->irq = platform_get_irq(platdev, 0);
	if (unlikely(edmac->irq < 0))
		return -ENODEV;

	edmac->rstc = devm_reset_control_get(&(platdev->dev), "dma-reset");
	if (IS_ERR(edmac->rstc))
		return PTR_ERR(edmac->rstc);

	res = platform_get_resource(platdev, IORESOURCE_MEM, 0);
	if (!res) {
		edmacv310_error("no reg resource\n");
		return -ENODEV;
	}

	edmac->base = devm_ioremap_resource(&(platdev->dev), res);
	if (IS_ERR(edmac->base))
		return PTR_ERR(edmac->base);

	res = platform_get_resource_byname(platdev, IORESOURCE_MEM, "dma_peri_channel_req_sel");
	if (res) {
		void *dma_peri_channel_req_sel = ioremap(res->start, res->end - res->start);
		if (IS_ERR(dma_peri_channel_req_sel))
			return PTR_ERR(dma_peri_channel_req_sel);
		writel(0xffffffff, dma_peri_channel_req_sel);
		iounmap(dma_peri_channel_req_sel);
	}
	return 0;
}

static int edmacv310_of_property_read(struct edmacv310_driver_data *edmac)
{
	struct platform_device *platdev = edmac->dev;
	struct device_node *np = platdev->dev.of_node;
	int ret;

	if (!of_find_property(np, "misc_regmap", NULL) ||
	    !of_find_property(np, "misc_ctrl_base", NULL)) {
		edmac->misc_regmap = 0;
	} else {
		edmac->misc_regmap = syscon_regmap_lookup_by_phandle(np, "misc_regmap");
		if (IS_ERR(edmac->misc_regmap))
			return PTR_ERR(edmac->misc_regmap);

		ret = of_property_read_u32(np, "misc_ctrl_base", &(edmac->misc_ctrl_base));
		if (ret) {
			edmacv310_error("get dma-misc_ctrl_base fail\n");
			return -ENODEV;
		}
	}
	ret = of_property_read_u32(np, "devid", &(edmac->id));
	if (ret) {
		edmacv310_error("get edmac id fail\n");
		return -ENODEV;
	}
	ret = of_property_read_u32(np, "dma-channels", &(edmac->channels));
	if (ret) {
		edmacv310_error("get dma-channels fail\n");
		return -ENODEV;
	}
	ret = of_property_read_u32(np, "dma-requests", &(edmac->slave_requests));
	if (ret) {
		edmacv310_error("get dma-requests fail\n");
		return -ENODEV;
	}
	edmacv310_trace(EDMACV310_REG_TRACE_LEVEL, "dma-channels = %d, dma-requests = %d\n",
			  edmac->channels, edmac->slave_requests);
	return 0;
}

static int get_of_probe(struct edmacv310_driver_data *edmac)
{
	struct platform_device *platdev = edmac->dev;
	int ret;

	ret = edmacv310_devm_get(edmac);
	if (ret)
		return ret;

	ret = edmacv310_of_property_read(edmac);
	if (ret)
		return ret;

	return of_dma_controller_register(platdev->dev.of_node,
					  edma_of_xlate, edmac);
}

static void edmac_free_chan_resources(struct dma_chan *chan)
{
	vchan_free_chan_resources(to_virt_chan(chan));
}

static size_t read_residue_from_phychan(
	const struct edmacv310_dma_chan *edmac_dma_chan,
	const struct transfer_desc *tsf_desc)
{
	size_t bytes;
	u64 next_lli;
	struct edmacv310_phy_chan *phychan = edmac_dma_chan->phychan;
	unsigned int i, index;
	struct edmacv310_driver_data *edmac = edmac_dma_chan->host;
	edmac_lli *plli = NULL;

	next_lli = (edmacv310_readl(edmac->base + edmac_cx_lli_l(phychan->id)) &
		    (~(EDMAC_LLI_ALIGN - 1)));
	next_lli |= ((u64)(edmacv310_readl(edmac->base + edmac_cx_lli_h(
			phychan->id)) & 0xffffffff) << BITS_PER_HALF_WORD);
	bytes = edmacv310_readl(edmac->base + edmac_cx_curr_cnt0(
					  phychan->id));
	if (next_lli != 0) {
		/* It means lli mode */
		bytes += tsf_desc->size;
		index = (next_lli - tsf_desc->llis_busaddr) / sizeof(*plli);
		plli = (edmac_lli *)(tsf_desc->llis_vaddr);

		if (index > MAX_TSFR_LLIS)
			return 0;

		for (i = 0; i < index; i++)
			bytes -= plli[i].count;
	}
	return bytes;
}

static enum dma_status edmac_tx_status(struct dma_chan *chan,
		dma_cookie_t cookie,
		struct dma_tx_state *txstate)
{
	enum dma_status ret;
	struct edmacv310_dma_chan *edmac_dma_chan = to_edamc_chan(chan);
	struct virt_dma_desc *vd = NULL;
	struct transfer_desc *tsf_desc = NULL;
	unsigned long flags;
	size_t bytes;

	ret = dma_cookie_status(chan, cookie, txstate);
	if (ret == DMA_COMPLETE)
		return ret;

	spin_lock_irqsave(&edmac_dma_chan->virt_chan.lock, flags);
	vd = vchan_find_desc(&edmac_dma_chan->virt_chan, cookie);
	if (vd) {
		/* no been trasfer */
		tsf_desc = to_edmac_transfer_desc(&vd->tx);
		bytes = tsf_desc->size;
	} else {
		/* trasfering */
		tsf_desc = edmac_dma_chan->at;

		if (!(edmac_dma_chan->phychan) || !tsf_desc) {
			spin_unlock_irqrestore(&edmac_dma_chan->virt_chan.lock, flags);
			return ret;
		}
		bytes = read_residue_from_phychan(edmac_dma_chan, tsf_desc);
	}
	spin_unlock_irqrestore(&edmac_dma_chan->virt_chan.lock, flags);
	dma_set_residue(txstate, bytes);

	if (edmac_dma_chan->state == EDMAC_CHAN_PAUSED && ret == DMA_IN_PROGRESS)
		ret = DMA_PAUSED;

	return ret;
}

static struct edmacv310_phy_chan *edmac_get_phy_channel(
	const struct edmacv310_driver_data *edmac,
	struct edmacv310_dma_chan *edmac_dma_chan)
{
	struct edmacv310_phy_chan *ch = NULL;
	unsigned long flags;
	int i;

	for (i = 0; i < edmac->channels; i++) {
		ch = &edmac->phy_chans[i];

		spin_lock_irqsave(&ch->lock, flags);

		if (!ch->serving) {
			ch->serving = edmac_dma_chan;
			spin_unlock_irqrestore(&ch->lock, flags);
			break;
		}
		spin_unlock_irqrestore(&ch->lock, flags);
	}

	if (i == edmac->channels)
		return NULL;

	return ch;
}

static void edmac_write_lli(const struct edmacv310_driver_data *edmac,
			      const struct edmacv310_phy_chan *phychan,
			      const struct transfer_desc *tsf_desc)
{
	edmac_lli *plli = (edmac_lli *)tsf_desc->llis_vaddr;

	if (plli->next_lli != 0x0)
		edmacv310_writel((plli->next_lli & 0xffffffff) | EDMAC_LLI_ENABLE,
				   edmac->base + edmac_cx_lli_l(phychan->id));
	else
		edmacv310_writel((plli->next_lli & 0xffffffff),
				   edmac->base + edmac_cx_lli_l(phychan->id));

	edmacv310_writel(((plli->next_lli >> 32) & 0xffffffff),
			   edmac->base + edmac_cx_lli_h(phychan->id));
	edmacv310_writel(plli->count, edmac->base + edmac_cx_cnt0(phychan->id));
	edmacv310_writel(plli->src_addr & 0xffffffff,
			   edmac->base + edmac_cx_src_addr_l(phychan->id));
	edmacv310_writel((plli->src_addr >> 32) & 0xffffffff,
			   edmac->base + edmac_cx_src_addr_h(phychan->id));
	edmacv310_writel(plli->dest_addr & 0xffffffff,
			   edmac->base + edmac_cx_dest_addr_l(phychan->id));
	edmacv310_writel((plli->dest_addr >> 32) & 0xffffffff,
			   edmac->base + edmac_cx_dest_addr_h(phychan->id));
	edmacv310_writel(plli->config,
			   edmac->base + edmac_cx_config(phychan->id));
}

static void edmac_start_next_txd(struct edmacv310_dma_chan *edmac_dma_chan)
{
	struct edmacv310_driver_data *edmac = edmac_dma_chan->host;
	struct edmacv310_phy_chan *phychan = edmac_dma_chan->phychan;
	struct virt_dma_desc *vd = vchan_next_desc(&edmac_dma_chan->virt_chan);
	struct transfer_desc *tsf_desc = to_edmac_transfer_desc(&vd->tx);
	unsigned int val;
	list_del(&tsf_desc->virt_desc.node);
	edmac_dma_chan->at = tsf_desc;
	edmac_write_lli(edmac, phychan, tsf_desc);
	val = edmacv310_readl(edmac->base + edmac_cx_config(phychan->id));
	edmacv310_trace(EDMACV310_REG_TRACE_LEVEL, " EDMAC_Cx_CONFIG  = 0x%x\n", val);
	edmacv310_writel(val | EDMAC_CXCONFIG_LLI_START,
			   edmac->base + edmac_cx_config(phychan->id));
}

static void edmac_start(struct edmacv310_dma_chan *edmac_dma_chan)
{
	struct edmacv310_driver_data *edmac = edmac_dma_chan->host;
	struct edmacv310_phy_chan *ch;
	ch = edmac_get_phy_channel(edmac, edmac_dma_chan);
	if (!ch) {
		edmacv310_error("no phy channel available !\n");
		edmac_dma_chan->state = EDMAC_CHAN_WAITING;
		return;
	}
	edmac_dma_chan->phychan = ch;
	edmac_dma_chan->state = EDMAC_CHAN_RUNNING;
	edmac_start_next_txd(edmac_dma_chan);
}

static void edmac_issue_pending(struct dma_chan *chan)
{
	struct edmacv310_dma_chan *edmac_dma_chan = to_edamc_chan(chan);
	unsigned long flags;
	spin_lock_irqsave(&edmac_dma_chan->virt_chan.lock, flags);
	if (vchan_issue_pending(&edmac_dma_chan->virt_chan)) {
		if (!edmac_dma_chan->phychan && edmac_dma_chan->state != EDMAC_CHAN_WAITING)
			edmac_start(edmac_dma_chan);
	}
	spin_unlock_irqrestore(&edmac_dma_chan->virt_chan.lock, flags);
}

static void edmac_free_txd_list(struct edmacv310_dma_chan *edmac_dma_chan)
{
	LIST_HEAD(head);
	vchan_get_all_descriptors(&edmac_dma_chan->virt_chan, &head);
	vchan_dma_desc_free_list(&edmac_dma_chan->virt_chan, &head);
}

static int edmac_config(struct dma_chan *chan,
			  struct dma_slave_config *config)
{
	struct edmacv310_dma_chan *edmac_dma_chan = to_edamc_chan(chan);
	if (!edmac_dma_chan->slave) {
		edmacv310_error("slave is null!");
		return -EINVAL;
	}
	edmac_dma_chan->cfg = *config;
	return 0;
}

static void edmac_pause_phy_chan(const struct edmacv310_dma_chan *edmac_dma_chan)
{
	struct edmacv310_driver_data *edmac = edmac_dma_chan->host;
	struct edmacv310_phy_chan *phychan = edmac_dma_chan->phychan;
	unsigned int val;
	int timeout;

	val = edmacv310_readl(edmac->base + edmac_cx_config(phychan->id));
	val &= ~CCFG_EN;
	edmacv310_writel(val, edmac->base + edmac_cx_config(phychan->id));
	/* Wait for channel inactive */
	for (timeout = 2000; timeout > 0; timeout--) {
		if (!((0x1 << phychan->id) & edmacv310_readl(edmac->base + EDMAC_CH_STAT)))
			break;
		edmacv310_writel(val, edmac->base + edmac_cx_config(phychan->id));
		udelay(1);
	}
	if (timeout == 0)
		edmacv310_error(":channel%u timeout waiting for pause, timeout:%d\n",
				  phychan->id, timeout);
}

static int edmac_pause(struct dma_chan *chan)
{
	struct edmacv310_dma_chan *edmac_dma_chan = to_edamc_chan(chan);
	unsigned long flags;

	spin_lock_irqsave(&edmac_dma_chan->virt_chan.lock, flags);
	if (!edmac_dma_chan->phychan) {
		spin_unlock_irqrestore(&edmac_dma_chan->virt_chan.lock, flags);
		return 0;
	}
	edmac_pause_phy_chan(edmac_dma_chan);
	edmac_dma_chan->state = EDMAC_CHAN_PAUSED;
	spin_unlock_irqrestore(&edmac_dma_chan->virt_chan.lock, flags);
	return 0;
}

static void edmac_resume_phy_chan(const struct edmacv310_dma_chan *edmac_dma_chan)
{
	struct edmacv310_driver_data *edmac = edmac_dma_chan->host;
	struct edmacv310_phy_chan *phychan = edmac_dma_chan->phychan;
	unsigned int val;
	val = edmacv310_readl(edmac->base + edmac_cx_config(phychan->id));
	val |= CCFG_EN;
	edmacv310_writel(val, edmac->base + edmac_cx_config(phychan->id));
}

static int edmac_resume(struct dma_chan *chan)
{
	struct edmacv310_dma_chan *edmac_dma_chan = to_edamc_chan(chan);
	unsigned long flags;

	spin_lock_irqsave(&edmac_dma_chan->virt_chan.lock, flags);

	if (!edmac_dma_chan->phychan) {
		spin_unlock_irqrestore(&edmac_dma_chan->virt_chan.lock, flags);
		return 0;
	}

	edmac_resume_phy_chan(edmac_dma_chan);
	edmac_dma_chan->state = EDMAC_CHAN_RUNNING;
	spin_unlock_irqrestore(&edmac_dma_chan->virt_chan.lock, flags);

	return 0;
}

void edmac_phy_free(struct edmacv310_dma_chan *chan);
static void edmac_desc_free(struct virt_dma_desc *vd);
static int edmac_terminate_all(struct dma_chan *chan)
{
	struct edmacv310_dma_chan *edmac_dma_chan = to_edamc_chan(chan);
	unsigned long flags;

	spin_lock_irqsave(&edmac_dma_chan->virt_chan.lock, flags);
	if (!edmac_dma_chan->phychan && !edmac_dma_chan->at) {
		spin_unlock_irqrestore(&edmac_dma_chan->virt_chan.lock, flags);
		return 0;
	}

	edmac_dma_chan->state = EDMAC_CHAN_IDLE;

	if (edmac_dma_chan->phychan)
		edmac_phy_free(edmac_dma_chan);
	if (edmac_dma_chan->at) {
		edmac_desc_free(&edmac_dma_chan->at->virt_desc);
		edmac_dma_chan->at = NULL;
	}
	edmac_free_txd_list(edmac_dma_chan);
	spin_unlock_irqrestore(&edmac_dma_chan->virt_chan.lock, flags);

	return 0;
}

static u32 get_width(enum dma_slave_buswidth width)
{
	switch (width) {
	case DMA_SLAVE_BUSWIDTH_1_BYTE:
		return EDMAC_WIDTH_8BIT;
	case DMA_SLAVE_BUSWIDTH_2_BYTES:
		return EDMAC_WIDTH_16BIT;
	case DMA_SLAVE_BUSWIDTH_4_BYTES:
		return EDMAC_WIDTH_32BIT;
	case DMA_SLAVE_BUSWIDTH_8_BYTES:
		return EDMAC_WIDTH_64BIT;
	default:
		edmacv310_error("check here, width warning!\n");
		return ~0;
	}
}

static unsigned int edmac_set_config_value(enum dma_transfer_direction direction,
		unsigned int addr_width,
		unsigned int burst,
		unsigned int signal)
{
	unsigned int config, width;

	if (direction == DMA_MEM_TO_DEV)
		config = EDMAC_CONFIG_SRC_INC;
	else
		config = EDMAC_CONFIG_DST_INC;

	edmacv310_trace(EDMACV310_CONFIG_TRACE_LEVEL, "addr_width = 0x%x\n", addr_width);
	width = get_width(addr_width);
	edmacv310_trace(EDMACV310_CONFIG_TRACE_LEVEL, "width = 0x%x\n", width);
	config |= width << EDMAC_CONFIG_SRC_WIDTH_SHIFT;
	config |= width << EDMAC_CONFIG_DST_WIDTH_SHIFT;
	edmacv310_trace(EDMACV310_REG_TRACE_LEVEL, "tsf_desc->ccfg = 0x%x\n", config);
	edmacv310_trace(EDMACV310_CONFIG_TRACE_LEVEL, "burst = 0x%x\n", burst);
	config |= burst << EDMAC_CONFIG_SRC_BURST_SHIFT;
	config |= burst << EDMAC_CONFIG_DST_BURST_SHIFT;
	if (signal >= 0) {
		edmacv310_trace(EDMACV310_REG_TRACE_LEVEL, "edmac_dma_chan->signal = %d\n", signal);
		config |= (unsigned int)signal << EDMAC_CXCONFIG_SIGNAL_SHIFT;
	}
	config |= EDMAC_CXCONFIG_DEV_MEM_TYPE << EDMAC_CXCONFIG_TSF_TYPE_SHIFT;
	return config;
}

struct transfer_desc *edmac_init_tsf_desc(const struct dma_chan *chan,
		enum dma_transfer_direction direction,
		dma_addr_t *slave_addr)
{
	struct edmacv310_dma_chan *edmac_dma_chan = to_edamc_chan(chan);
	struct transfer_desc *tsf_desc;
	unsigned int burst = 0;
	unsigned int addr_width = 0;
	unsigned int maxburst = 0;
	tsf_desc = kzalloc(sizeof(*tsf_desc), GFP_NOWAIT);
	if (!tsf_desc)
		return NULL;
	if (direction == DMA_MEM_TO_DEV) {
		*slave_addr = edmac_dma_chan->cfg.dst_addr;
		addr_width = edmac_dma_chan->cfg.dst_addr_width;
		maxburst = edmac_dma_chan->cfg.dst_maxburst;
	} else if (direction == DMA_DEV_TO_MEM) {
		*slave_addr = edmac_dma_chan->cfg.src_addr;
		addr_width = edmac_dma_chan->cfg.src_addr_width;
		maxburst = edmac_dma_chan->cfg.src_maxburst;
	} else {
		kfree(tsf_desc);
		edmacv310_error("direction unsupported!\n");
		return NULL;
	}

	if (maxburst > (EDMAC_MAX_BURST_WIDTH))
		burst |= (EDMAC_MAX_BURST_WIDTH - 1);
	else if (maxburst == 0)
		burst |= EDMAC_MIN_BURST_WIDTH;
	else
		burst |= (maxburst - 1);

	tsf_desc->ccfg = edmac_set_config_value(direction, addr_width,
			 burst, edmac_dma_chan->signal);
	edmacv310_trace(EDMACV310_REG_TRACE_LEVEL, "tsf_desc->ccfg = 0x%x\n", tsf_desc->ccfg);
	return tsf_desc;
}

static int edmac_fill_desc(const struct edmac_sg *dsg,
			     struct transfer_desc *tsf_desc,
			     unsigned int length, unsigned int num)
{
	edmac_lli *plli = NULL;

	if (num >= MAX_TSFR_LLIS) {
		edmacv310_error("lli out of range. \n");
		return -ENOMEM;
	}

	plli = (edmac_lli *)(tsf_desc->llis_vaddr);
	(void)memset_s(&plli[num], sizeof(edmac_lli), 0x0, sizeof(*plli));

	plli[num].src_addr = dsg->src_addr;
	plli[num].dest_addr = dsg->dst_addr;
	plli[num].config = tsf_desc->ccfg;
	plli[num].count = length;
	tsf_desc->size += length;

	if (num > 0) {
		plli[num - 1].next_lli = (tsf_desc->llis_busaddr + (num) * sizeof(
						  *plli)) & (~(EDMAC_LLI_ALIGN - 1));
		plli[num - 1].next_lli |= EDMAC_LLI_ENABLE;
	}
	return 0;
}

static void free_dsg(struct list_head *dsg_head)
{
	struct edmac_sg *dsg = NULL;
	struct edmac_sg *_dsg = NULL;

	list_for_each_entry_safe(dsg, _dsg, dsg_head, node) {
		list_del(&dsg->node);
		kfree(dsg);
	}
}

static int edmac_add_sg(struct list_head *sg_head,
			  dma_addr_t dst, dma_addr_t src,
			  size_t len)
{
	struct edmac_sg *dsg = NULL;

	if (len == 0) {
		free_dsg(sg_head);
		edmacv310_error("Transfer length is 0. \n");
		return -ENOMEM;
	}

	dsg = (struct edmac_sg *)kzalloc(sizeof(*dsg), GFP_NOWAIT);
	if (!dsg) {
		free_dsg(sg_head);
		edmacv310_error("alloc memory for dsg fail.\n");
		return -ENOMEM;
	}

	list_add_tail(&dsg->node, sg_head);
	dsg->src_addr = src;
	dsg->dst_addr = dst;
	dsg->len = len;
	return 0;
}

static int edmac_add_sg_slave(struct list_head *sg_head,
		dma_addr_t slave_addr, dma_addr_t addr,
		size_t length,
		enum dma_transfer_direction direction)
{
	dma_addr_t src = 0;
	dma_addr_t dst = 0;
	if (direction == DMA_MEM_TO_DEV) {
		src = addr;
		dst = slave_addr;
	} else if (direction == DMA_DEV_TO_MEM) {
		src = slave_addr;
		dst = addr;
	} else {
		edmacv310_error("invali dma_transfer_direction.\n");
		return -ENOMEM;
	}
	return edmac_add_sg(sg_head, dst, src, length);
}

static int edmac_fill_sg_for_slave(struct list_head *sg_head,
		dma_addr_t slave_addr,
		struct scatterlist *sgl,
		unsigned int sg_len,
		enum dma_transfer_direction direction)
{
	struct scatterlist *sg = NULL;
	int tmp, ret;
	size_t length;
	dma_addr_t addr;
	if (sgl == NULL) {
		edmacv310_error("sgl is null!\n");
		return -ENOMEM;
	}

	for_each_sg(sgl, sg, sg_len, tmp) {
		addr = sg_dma_address(sg);
		length = sg_dma_len(sg);
		ret = edmac_add_sg_slave(sg_head, slave_addr, addr, length, direction);
		if (ret)
			break;
	}
	return ret;
}

static inline int edmac_fill_sg_for_m2m_copy(struct list_head *sg_head,
		dma_addr_t dst, dma_addr_t src,
		size_t len)
{
	return edmac_add_sg(sg_head, dst, src, len);
}

struct edmac_cyclic_args {
	dma_addr_t slave_addr;
	dma_addr_t buf_addr;
	size_t buf_len;
	size_t period_len;
	enum dma_transfer_direction direction;
};

static int edmac_fill_sg_for_cyclic(struct list_head *sg_head,
				      struct edmac_cyclic_args args)
{
	size_t count_in_sg = 0;
	size_t trans_bytes;
	int ret;
	while (count_in_sg < args.buf_len) {
		trans_bytes = min(args.period_len, args.buf_len - count_in_sg);
		count_in_sg += trans_bytes;
		ret = edmac_add_sg_slave(sg_head, args.slave_addr, args.buf_addr + count_in_sg, count_in_sg, args.direction);
		if (ret)
			return ret;
	}
	return 0;
}

static unsigned short get_max_width(dma_addr_t ccfg)
{
	unsigned short src_width = (ccfg & EDMAC_CONTROL_SRC_WIDTH_MASK) >>
				   EDMAC_CONFIG_SRC_WIDTH_SHIFT;
	unsigned short dst_width = (ccfg & EDMAC_CONTROL_DST_WIDTH_MASK) >>
				   EDMAC_CONFIG_DST_WIDTH_SHIFT;
	return 1 << max(src_width, dst_width); /* to byte */
}

static int edmac_fill_asg_lli_for_desc(struct edmac_sg *dsg,
		struct transfer_desc *tsf_desc,
		unsigned int *lli_count)
{
	int ret;
	unsigned short width = get_max_width(tsf_desc->ccfg);

	while (dsg->len != 0) {
		size_t lli_len = MAX_TRANSFER_BYTES;
		lli_len = (lli_len / width) * width; /* bus width align */
		lli_len = min(lli_len, dsg->len);
		ret = edmac_fill_desc(dsg, tsf_desc, lli_len, *lli_count);
		if (ret)
			return ret;

		if (tsf_desc->ccfg & EDMAC_CONFIG_SRC_INC)
			dsg->src_addr += lli_len;
		if (tsf_desc->ccfg & EDMAC_CONFIG_DST_INC)
			dsg->dst_addr += lli_len;
		dsg->len -= lli_len;
		(*lli_count)++;
	}
	return 0;
}

static int edmac_fill_lli_for_desc(const struct list_head *sg_head,
				     struct transfer_desc *tsf_desc)
{
	struct edmac_sg *dsg = NULL;
	struct edmac_lli *last_plli = NULL;
	unsigned int lli_count = 0;
	int ret;

	list_for_each_entry(dsg, sg_head, node) {
		ret = edmac_fill_asg_lli_for_desc(dsg, tsf_desc, &lli_count);
		if (ret)
			return ret;
	}

	if (tsf_desc->cyclic) {
		last_plli = (edmac_lli *)((uintptr_t)tsf_desc->llis_vaddr +
					    (lli_count - 1) * sizeof(*last_plli));
		last_plli->next_lli = tsf_desc->llis_busaddr | EDMAC_LLI_ENABLE;
	} else {
		last_plli = (edmac_lli *)((uintptr_t)tsf_desc->llis_vaddr +
					    (lli_count - 1) * sizeof(*last_plli));
		last_plli->next_lli = 0;
	}
	dump_lli(tsf_desc->llis_vaddr, lli_count);
	return 0;
}

static struct dma_async_tx_descriptor *edmac_prep_slave_sg(
	struct dma_chan *chan, struct scatterlist *sgl,
	unsigned int sg_len, enum dma_transfer_direction direction,
	unsigned long flags, void *context)
{
	struct edmacv310_dma_chan *edmac_dma_chan = to_edamc_chan(chan);
	struct edmacv310_driver_data *edmac = edmac_dma_chan->host;
	struct transfer_desc *tsf_desc = NULL;
	dma_addr_t slave_addr = 0;
	int ret;
	LIST_HEAD(sg_head);
	if (sgl == NULL) {
		edmacv310_error("sgl is null!\n");
		return NULL;
	}

	tsf_desc = edmac_init_tsf_desc(chan, direction, &slave_addr);
	if (!tsf_desc)
		return NULL;

	tsf_desc->llis_vaddr = dma_pool_alloc(edmac->pool, GFP_NOWAIT,
					      &tsf_desc->llis_busaddr);
	if (!tsf_desc->llis_vaddr) {
		edmacv310_error("malloc memory from pool fail !\n");
		goto err_alloc_lli;
	}

	ret = edmac_fill_sg_for_slave(&sg_head, slave_addr, sgl, sg_len, direction);
	if (ret)
		goto err_fill_sg;
	ret = edmac_fill_lli_for_desc(&sg_head, tsf_desc);
	free_dsg(&sg_head);
	if (ret)
		goto err_fill_sg;
	return vchan_tx_prep(&edmac_dma_chan->virt_chan, &tsf_desc->virt_desc, flags);

err_fill_sg:
	dma_pool_free(edmac->pool, tsf_desc->llis_vaddr, tsf_desc->llis_busaddr);
err_alloc_lli:
	kfree(tsf_desc);
	return NULL;
}

static struct dma_async_tx_descriptor *edmac_prep_dma_m2m_copy(
	struct dma_chan *chan, dma_addr_t dst, dma_addr_t src,
	size_t len, unsigned long flags)
{
	struct edmacv310_dma_chan *edmac_dma_chan = to_edamc_chan(chan);
	struct edmacv310_driver_data *edmac = edmac_dma_chan->host;
	struct transfer_desc *tsf_desc = NULL;
	LIST_HEAD(sg_head);
	u32 config = 0;
	int ret;

	if (!len)
		return NULL;

	tsf_desc = kzalloc(sizeof(*tsf_desc), GFP_NOWAIT);
	if (tsf_desc == NULL) {
		edmacv310_error("get tsf desc fail!\n");
		return NULL;
	}

	tsf_desc->llis_vaddr = dma_pool_alloc(edmac->pool, GFP_NOWAIT,
					      &tsf_desc->llis_busaddr);
	if (!tsf_desc->llis_vaddr) {
		edmacv310_error("malloc memory from pool fail !\n");
		goto err_alloc_lli;
	}

	config |= EDMAC_CONFIG_SRC_INC | EDMAC_CONFIG_DST_INC;
	config |= EDMAC_CXCONFIG_MEM_TYPE << EDMAC_CXCONFIG_TSF_TYPE_SHIFT;
	/*  max burst width is 16 ,but reg value set 0xf */
	config |= (EDMAC_MAX_BURST_WIDTH - 1) << EDMAC_CONFIG_SRC_BURST_SHIFT;
	config |= (EDMAC_MAX_BURST_WIDTH - 1) << EDMAC_CONFIG_DST_BURST_SHIFT;
	config |= EDMAC_MEM_BIT_WIDTH << EDMAC_CONFIG_SRC_WIDTH_SHIFT;
	config |= EDMAC_MEM_BIT_WIDTH << EDMAC_CONFIG_DST_WIDTH_SHIFT;
	tsf_desc->ccfg = config;
	ret = edmac_fill_sg_for_m2m_copy(&sg_head, dst, src, len);
	if (ret)
		goto err_fill_sg;
	ret = edmac_fill_lli_for_desc(&sg_head, tsf_desc);
	free_dsg(&sg_head);
	if (ret)
		goto err_fill_sg;
	return vchan_tx_prep(&edmac_dma_chan->virt_chan, &tsf_desc->virt_desc, flags);

err_fill_sg:
	dma_pool_free(edmac->pool, tsf_desc->llis_vaddr, tsf_desc->llis_busaddr);
err_alloc_lli:
	kfree(tsf_desc);
	return NULL;
}

static struct dma_async_tx_descriptor *edmac_prep_dma_cyclic(
	struct dma_chan *chan, dma_addr_t buf_addr, size_t buf_len,
	size_t period_len, enum dma_transfer_direction direction,
	unsigned long flags)
{
	struct edmacv310_dma_chan *edmac_dma_chan = to_edamc_chan(chan);
	struct edmacv310_driver_data *edmac = edmac_dma_chan->host;
	struct transfer_desc *tsf_desc = NULL;
	struct edmac_cyclic_args args = {
		.slave_addr = 0,
		.buf_addr = buf_addr,
		.buf_len = buf_len,
		.period_len = period_len,
		.direction = direction
	};
	LIST_HEAD(sg_head);
	int ret;

	tsf_desc = edmac_init_tsf_desc(chan, direction, &(args.slave_addr));
	if (!tsf_desc)
		return NULL;

	tsf_desc->llis_vaddr = dma_pool_alloc(edmac->pool, GFP_NOWAIT,
					      &tsf_desc->llis_busaddr);
	if (!tsf_desc->llis_vaddr) {
		edmacv310_error("malloc memory from pool fail !\n");
		goto err_alloc_lli;
	}

	tsf_desc->cyclic = true;
	ret = edmac_fill_sg_for_cyclic(&sg_head, args);
	if (ret)
		goto err_fill_sg;
	ret = edmac_fill_lli_for_desc(&sg_head, tsf_desc);
	free_dsg(&sg_head);
	if (ret)
		goto err_fill_sg;
	return vchan_tx_prep(&edmac_dma_chan->virt_chan, &tsf_desc->virt_desc, flags);

err_fill_sg:
	dma_pool_free(edmac->pool, tsf_desc->llis_vaddr, tsf_desc->llis_busaddr);
err_alloc_lli:
	kfree(tsf_desc);
	return NULL;
}

static void edmac_phy_reassign(struct edmacv310_phy_chan *phy_chan,
				 struct edmacv310_dma_chan *chan)
{
	phy_chan->serving = chan;
	chan->phychan = phy_chan;
	chan->state = EDMAC_CHAN_RUNNING;

	edmac_start_next_txd(chan);
}

static void edmac_terminate_phy_chan(const struct edmacv310_driver_data *edmac,
				       const struct edmacv310_dma_chan *edmac_dma_chan)
{
	unsigned int val;
	struct edmacv310_phy_chan *phychan = edmac_dma_chan->phychan;
	edmac_pause_phy_chan(edmac_dma_chan);
	val = 0x1 << phychan->id;
	edmacv310_writel(val, edmac->base + EDMAC_INT_TC1_RAW);
	edmacv310_writel(val, edmac->base + EDMAC_INT_ERR1_RAW);
	edmacv310_writel(val, edmac->base + EDMAC_INT_ERR2_RAW);
}

void edmac_phy_free(struct edmacv310_dma_chan *chan)
{
	struct edmacv310_driver_data *edmac = chan->host;
	struct edmacv310_dma_chan *p = NULL;
	struct edmacv310_dma_chan *next = NULL;

	list_for_each_entry(p, &edmac->memcpy.channels, virt_chan.chan.device_node) {
		if (p->state == EDMAC_CHAN_WAITING) {
			next = p;
			break;
		}
	}

	if (!next) {
		list_for_each_entry(p, &edmac->slave.channels, virt_chan.chan.device_node) {
			if (p->state == EDMAC_CHAN_WAITING) {
				next = p;
				break;
			}
		}
	}
	edmac_terminate_phy_chan(edmac, chan);

	if (next) {
		spin_lock(&next->virt_chan.lock);
		edmac_phy_reassign(chan->phychan, next);
		spin_unlock(&next->virt_chan.lock);
	} else {
		chan->phychan->serving = NULL;
	}

	chan->phychan = NULL;
	chan->state = EDMAC_CHAN_IDLE;
}

bool handle_irq(const struct edmacv310_driver_data *edmac, int chan_id)
{
	struct edmacv310_dma_chan *chan = NULL;
	struct edmacv310_phy_chan *phy_chan = NULL;
	struct transfer_desc *tsf_desc = NULL;
	unsigned int channel_tc_status, channel_err_status[ERR_STATUS_REG_NUM];

	phy_chan = &edmac->phy_chans[chan_id];
	chan = phy_chan->serving;
	if (!chan) {
		edmacv310_error("error interrupt on chan: %d!\n", chan_id);
		return 0;
	}
	tsf_desc = chan->at;

	channel_tc_status = edmacv310_readl(edmac->base + EDMAC_INT_TC1_RAW);
	channel_tc_status = (channel_tc_status >> chan_id) & 0x01;
	if (channel_tc_status)
		edmacv310_writel(channel_tc_status << chan_id, edmac->base + EDMAC_INT_TC1_RAW);

	channel_tc_status = edmacv310_readl(edmac->base + EDMAC_INT_TC2);
	channel_tc_status = (channel_tc_status >> chan_id) & 0x01;
	if (channel_tc_status)
		edmacv310_writel(channel_tc_status << chan_id, edmac->base + EDMAC_INT_TC2_RAW);

	channel_err_status[0] = edmacv310_readl(edmac->base + EDMAC_INT_ERR1);
	channel_err_status[1] = edmacv310_readl(edmac->base + EDMAC_INT_ERR2);
	channel_err_status[2] = edmacv310_readl(edmac->base + EDMAC_INT_ERR3);
	if ((channel_err_status[0] | channel_err_status[1] | channel_err_status[2]) & (1 << chan_id)) {
		edmacv310_error("Error in edmac %d!,ERR1 = 0x%x,ERR2 = 0x%x,ERR3 = 0x%x\n",
				  chan_id, channel_err_status[0],
				  channel_err_status[1], channel_err_status[2]);
		edmacv310_writel(1 << chan_id, edmac->base + EDMAC_INT_ERR1_RAW);
		edmacv310_writel(1 << chan_id, edmac->base + EDMAC_INT_ERR2_RAW);
		edmacv310_writel(1 << chan_id, edmac->base + EDMAC_INT_ERR3_RAW);
	}

	spin_lock(&chan->virt_chan.lock);

	if (tsf_desc->cyclic) {
		vchan_cyclic_callback(&tsf_desc->virt_desc);
		spin_unlock(&chan->virt_chan.lock);
		return 1;
	}
	chan->at = NULL;
	tsf_desc->done = true;
	vchan_cookie_complete(&tsf_desc->virt_desc);

	if (vchan_next_desc(&chan->virt_chan))
		edmac_start_next_txd(chan);
	else
		edmac_phy_free(chan);
	spin_unlock(&chan->virt_chan.lock);
	return 1;
}

static irqreturn_t emdacv310_irq(int irq, void *dev)
{
	struct edmacv310_driver_data *edmac = (struct edmacv310_driver_data *)dev;
	u32 mask = 0;
	unsigned int channel_status, temp, i;

	channel_status = edmacv310_readl(edmac->base + EDMAC_INT_STAT);
	if (!channel_status) {
		edmacv310_error("channel_status = 0x%x\n", channel_status);
		return IRQ_NONE;
	}

	for (i = 0; i < edmac->channels; i++) {
		temp = (channel_status >> i) & 0x1;
		if (temp)
			mask |= handle_irq(edmac, i) << i;
	}
	return mask ? IRQ_HANDLED : IRQ_NONE;
}

static inline void edmac_dma_slave_init(struct edmacv310_dma_chan *chan)
{
	chan->slave = true;
}

static void edmac_desc_free(struct virt_dma_desc *vd)
{
	struct transfer_desc *tsf_desc = to_edmac_transfer_desc(&vd->tx);
	struct edmacv310_dma_chan *edmac_dma_chan = to_edamc_chan(vd->tx.chan);
	dma_descriptor_unmap(&vd->tx);
	dma_pool_free(edmac_dma_chan->host->pool, tsf_desc->llis_vaddr, tsf_desc->llis_busaddr);
	kfree(tsf_desc);
}

static int edmac_init_virt_channels(struct edmacv310_driver_data *edmac,
				      struct dma_device *dmadev,
				      unsigned int channels, bool slave)
{
	struct edmacv310_dma_chan *chan = NULL;
	int i;
	INIT_LIST_HEAD(&dmadev->channels);

	for (i = 0; i < channels; i++) {
		chan = kzalloc(sizeof(struct edmacv310_dma_chan), GFP_KERNEL);
		if (!chan) {
			edmacv310_error("fail to allocate memory for virt channels!");
			return -1;
		}

		chan->host = edmac;
		chan->state = EDMAC_CHAN_IDLE;
		chan->signal = -1;

		if (slave) {
			chan->id = i;
			edmac_dma_slave_init(chan);
		}
		chan->virt_chan.desc_free = edmac_desc_free;
		vchan_init(&chan->virt_chan, dmadev);
	}
	return 0;
}

void edmac_free_virt_channels(struct dma_device *dmadev)
{
	struct edmacv310_dma_chan *chan = NULL;
	struct edmacv310_dma_chan *next = NULL;

	list_for_each_entry_safe(chan, next, &dmadev->channels, virt_chan.chan.device_node) {
		list_del(&chan->virt_chan.chan.device_node);
		kfree(chan);
	}
}

static void edmacv310_prep_dma_device(struct platform_device *pdev,
					struct edmacv310_driver_data *edmac)
{
	dma_cap_set(DMA_MEMCPY, edmac->memcpy.cap_mask);
	edmac->memcpy.dev = &pdev->dev;
	edmac->memcpy.device_free_chan_resources = edmac_free_chan_resources;
	edmac->memcpy.device_prep_dma_memcpy = edmac_prep_dma_m2m_copy;
	edmac->memcpy.device_tx_status = edmac_tx_status;
	edmac->memcpy.device_issue_pending = edmac_issue_pending;
	edmac->memcpy.device_config = edmac_config;
	edmac->memcpy.device_pause = edmac_pause;
	edmac->memcpy.device_resume = edmac_resume;
	edmac->memcpy.device_terminate_all = edmac_terminate_all;
	edmac->memcpy.directions = BIT(DMA_MEM_TO_MEM);
	edmac->memcpy.residue_granularity = DMA_RESIDUE_GRANULARITY_SEGMENT;

	dma_cap_set(DMA_SLAVE, edmac->slave.cap_mask);
	dma_cap_set(DMA_CYCLIC, edmac->slave.cap_mask);
	edmac->slave.dev = &pdev->dev;
	edmac->slave.device_free_chan_resources = edmac_free_chan_resources;
	edmac->slave.device_tx_status = edmac_tx_status;
	edmac->slave.device_issue_pending = edmac_issue_pending;
	edmac->slave.device_prep_slave_sg = edmac_prep_slave_sg;
	edmac->slave.device_prep_dma_cyclic = edmac_prep_dma_cyclic;
	edmac->slave.device_config = edmac_config;
	edmac->slave.device_resume = edmac_resume;
	edmac->slave.device_pause = edmac_pause;
	edmac->slave.device_terminate_all = edmac_terminate_all;
	edmac->slave.directions = BIT(DMA_DEV_TO_MEM) | BIT(DMA_MEM_TO_DEV);
	edmac->slave.residue_granularity = DMA_RESIDUE_GRANULARITY_SEGMENT;
}

static int edmacv310_init_chan(struct edmacv310_driver_data *edmac)
{
	int i, ret;
	edmac->phy_chans = kzalloc((edmac->channels * sizeof(
					      struct edmacv310_phy_chan)),
				     GFP_KERNEL);
	if (!edmac->phy_chans) {
		edmacv310_error("malloc for phy chans fail!");
		return -ENOMEM;
	}

	for (i = 0; i < edmac->channels; i++) {
		struct edmacv310_phy_chan *phy_ch = &edmac->phy_chans[i];
		phy_ch->id = i;
		phy_ch->base = edmac->base + edmac_cx_base(i);
		spin_lock_init(&phy_ch->lock);
		phy_ch->serving = NULL;
	}

	ret = edmac_init_virt_channels(edmac, &edmac->memcpy, edmac->channels,
					 false);
	if (ret) {
		edmacv310_error("fail to init memory virt channels!");
		goto  free_phychans;
	}

	ret = edmac_init_virt_channels(edmac, &edmac->slave, edmac->slave_requests,
					 true);
	if (ret) {
		edmacv310_error("fail to init slave virt channels!");
		goto  free_memory_virt_channels;
	}
	return 0;

free_memory_virt_channels:
	edmac_free_virt_channels(&edmac->memcpy);
free_phychans:
	kfree(edmac->phy_chans);
	return -ENOMEM;
}

static void edmacv310_free_chan(struct edmacv310_driver_data *edmac)
{
	edmac_free_virt_channels(&edmac->slave);
	edmac_free_virt_channels(&edmac->memcpy);
	kfree(edmac->phy_chans);
}

static void edmacv310_prep_phy_device(const struct edmacv310_driver_data *edmac)
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

static struct edmacv310_driver_data *edmacv310_prep_edmac_device(struct platform_device *pdev)
{
	int ret;
	struct edmacv310_driver_data *edmac = NULL;
	ssize_t trasfer_size;

	ret = dma_set_mask_and_coherent(&(pdev->dev), DMA_BIT_MASK(64));
	if (ret)
		return NULL;

	edmac = kzalloc(sizeof(*edmac), GFP_KERNEL);
	if (!edmac) {
		edmacv310_error("malloc for edmac fail!");
		return NULL;
	}

	edmac->dev = pdev;

	ret = get_of_probe(edmac);
	if (ret) {
		edmacv310_error("get dts info fail!");
		goto free_edmac;
	}

	edmacv310_prep_dma_device(pdev, edmac);
	edmac->max_transfer_size = MAX_TRANSFER_BYTES;
	trasfer_size = MAX_TSFR_LLIS * EDMACV300_LLI_WORDS * sizeof(u32);

	edmac->pool = dma_pool_create(DRIVER_NAME, &(pdev->dev),
					trasfer_size, EDMACV300_POOL_ALIGN, 0);
	if (!edmac->pool) {
		edmacv310_error("create pool fail!");
		goto free_edmac;
	}

	ret = edmacv310_init_chan(edmac);
	if (ret)
		goto free_pool;

	return edmac;

free_pool:
	dma_pool_destroy(edmac->pool);
free_edmac:
	kfree(edmac);
	return NULL;
}

static void free_edmac_device(struct edmacv310_driver_data *edmac)
{
	edmacv310_free_chan(edmac);
	dma_pool_destroy(edmac->pool);
	kfree(edmac);
}

static int __init edmacv310_probe(struct platform_device *pdev)
{
	int ret;
	struct edmacv310_driver_data *edmac = NULL;

	edmac = edmacv310_prep_edmac_device(pdev);
	if (edmac == NULL)
		return -ENOMEM;

	ret = request_irq(edmac->irq, emdacv310_irq, 0, DRIVER_NAME, edmac);
	if (ret) {
		edmacv310_error("fail to request irq");
		goto free_edmac;
	}
	edmacv310_prep_phy_device(edmac);
	ret = dma_async_device_register(&edmac->memcpy);
	if (ret) {
		edmacv310_error("%s failed to register memcpy as an async device - %d\n", __func__, ret);
		goto free_irq_res;
	}

	ret = dma_async_device_register(&edmac->slave);
	if (ret) {
		edmacv310_error("%s failed to register slave as an async device - %d\n", __func__, ret);
		goto free_memcpy_device;
	}
	return 0;

free_memcpy_device:
	dma_async_device_unregister(&edmac->memcpy);
free_irq_res:
	free_irq(edmac->irq, edmac);
free_edmac:
	free_edmac_device(edmac);
	return -ENOMEM;
}

static int emda_remove(struct platform_device *pdev)
{
	int err = 0;
	return err;
}

static const struct of_device_id edmacv310_match[] = {
	{ .compatible = "vendor,edmacv310" },
	{},
};

static struct platform_driver edmacv310_driver = {
	.remove = emda_remove,
	.driver = {
		.name = "edmacv310",
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
