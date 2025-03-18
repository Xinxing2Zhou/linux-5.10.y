/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#include <asm/setup.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/unistd.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/mm.h>
#include <linux/mii.h>
#include <linux/ethtool.h>
#include <linux/phy.h>
#include <linux/dma-mapping.h>
#include <linux/workqueue.h>
#include <linux/device.h>
#include <linux/atomic.h>
#include <linux/platform_device.h>
#include <linux/capability.h>
#include <linux/time.h>
#include <linux/module.h>

#include <linux/circ_buf.h>
#include <linux/of_net.h>
#include <linux/of_mdio.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/securec.h>
#include "autoeee/autoeee.h"
#include "gmac_ethtool_ops.h"
#include "gmac_netdev_ops.h"
#include "gmac_phy_fixup.h"
#include "gmac_pm.h"
#include "gmac_proc.h"
#include "gmac.h"

#define has_tso_cap(hw_cap)		((((hw_cap) >> 28) & 0x3) == VER_TSO)
#define has_rxhash_cap(hw_cap)		((hw_cap) & BIT(30))
#define has_rss_cap(hw_cap)		((hw_cap) & BIT(31))

#define DEFAULT_MSG_ENABLE (NETIF_MSG_DRV | NETIF_MSG_PROBE | NETIF_MSG_LINK)
static int debug = -1;
module_param(debug, int, 0000);
MODULE_PARM_DESC(debug, "Debug level (0=none,...,16=all)");

static void gmac_set_desc_depth(struct gmac_netdev_local const *priv,
		  u32 rx, u32 tx)
{
	u32 reg, val;
	int i;

	writel(BITS_RX_FQ_DEPTH_EN, priv->gmac_iobase + RX_FQ_REG_EN);
	val = readl(priv->gmac_iobase + RX_FQ_DEPTH);
	val &= ~Q_ADDR_HI8_MASK;
	val |= rx << DESC_WORD_SHIFT;
	writel(val, priv->gmac_iobase + RX_FQ_DEPTH);
	writel(0, priv->gmac_iobase + RX_FQ_REG_EN);

	writel(BITS_RX_BQ_DEPTH_EN, priv->gmac_iobase + RX_BQ_REG_EN);
	val = readl(priv->gmac_iobase + RX_BQ_DEPTH);
	val &= ~Q_ADDR_HI8_MASK;
	val |= rx << DESC_WORD_SHIFT;
	writel(val, priv->gmac_iobase + RX_BQ_DEPTH);
	for (i = 1; i < priv->num_rxqs; i++) {
		reg = rx_bq_depth_queue(i);
		val = readl(priv->gmac_iobase + reg);
		val &= ~Q_ADDR_HI8_MASK;
		val |= rx << DESC_WORD_SHIFT;
		writel(val, priv->gmac_iobase + reg);
	}
	writel(0, priv->gmac_iobase + RX_BQ_REG_EN);

	writel(BITS_TX_BQ_DEPTH_EN, priv->gmac_iobase + TX_BQ_REG_EN);
	val = readl(priv->gmac_iobase + TX_BQ_DEPTH);
	val &= ~Q_ADDR_HI8_MASK;
	val |= tx << DESC_WORD_SHIFT;
	writel(val, priv->gmac_iobase + TX_BQ_DEPTH);
	writel(0, priv->gmac_iobase + TX_BQ_REG_EN);

	writel(BITS_TX_RQ_DEPTH_EN, priv->gmac_iobase + TX_RQ_REG_EN);
	val = readl(priv->gmac_iobase + TX_RQ_DEPTH);
	val &= ~Q_ADDR_HI8_MASK;
	val |= tx << DESC_WORD_SHIFT;
	writel(val, priv->gmac_iobase + TX_RQ_DEPTH);
	writel(0, priv->gmac_iobase + TX_RQ_REG_EN);
}

static void gmac_set_rx_fq(struct gmac_netdev_local const *priv,
		  dma_addr_t phy_addr)
{
#if defined(CONFIG_GMAC_DDR_64BIT )
	u32 val;
#endif
	writel(BITS_RX_FQ_START_ADDR_EN, priv->gmac_iobase + RX_FQ_REG_EN);
#if defined(CONFIG_GMAC_DDR_64BIT )
	val = readl(priv->gmac_iobase + RX_FQ_DEPTH);
	val &= Q_ADDR_HI8_MASK;
	val |= (phy_addr >> REG_BIT_WIDTH) << Q_ADDR_HI8_OFFSET;
	writel(val, priv->gmac_iobase + RX_FQ_DEPTH);
#endif
	writel((u32)phy_addr, priv->gmac_iobase + RX_FQ_START_ADDR);
	writel(0, priv->gmac_iobase + RX_FQ_REG_EN);
}

static void gmac_set_rx_bq(struct gmac_netdev_local const *priv,
		  dma_addr_t phy_addr)
{
#if defined(CONFIG_GMAC_DDR_64BIT )
	u32 val;
#endif

	writel(BITS_RX_BQ_START_ADDR_EN, priv->gmac_iobase + RX_BQ_REG_EN);
#if defined(CONFIG_GMAC_DDR_64BIT )
	val = readl(priv->gmac_iobase + RX_BQ_DEPTH);
	val &= Q_ADDR_HI8_MASK;
	val |= (phy_addr >> REG_BIT_WIDTH) << Q_ADDR_HI8_OFFSET;
	writel(val, priv->gmac_iobase + RX_BQ_DEPTH);
#endif
	writel((u32)phy_addr, priv->gmac_iobase + RX_BQ_START_ADDR);
	writel(0, priv->gmac_iobase + RX_BQ_REG_EN);
}

static void gmac_set_tx_bq(struct gmac_netdev_local const *priv,
		  dma_addr_t phy_addr)
{
#if defined(CONFIG_GMAC_DDR_64BIT )
	u32 val;
#endif
	writel(BITS_TX_BQ_START_ADDR_EN, priv->gmac_iobase + TX_BQ_REG_EN);
#if defined(CONFIG_GMAC_DDR_64BIT )
	val = readl(priv->gmac_iobase + TX_BQ_DEPTH);
	val &= Q_ADDR_HI8_MASK;
	val |= (phy_addr >> REG_BIT_WIDTH) << Q_ADDR_HI8_OFFSET;
	writel(val, priv->gmac_iobase + TX_BQ_DEPTH);
#endif
	writel((u32)phy_addr, priv->gmac_iobase + TX_BQ_START_ADDR);
	writel(0, priv->gmac_iobase + TX_BQ_REG_EN);
}

static void gmac_set_tx_rq(struct gmac_netdev_local const *priv,
		  dma_addr_t phy_addr)
{
#if defined(CONFIG_GMAC_DDR_64BIT )
	u32 val;
#endif
	writel(BITS_TX_RQ_START_ADDR_EN, priv->gmac_iobase + TX_RQ_REG_EN);
#if defined(CONFIG_GMAC_DDR_64BIT )
	val = readl(priv->gmac_iobase + TX_RQ_DEPTH);
	val &= Q_ADDR_HI8_MASK;
	val |= (phy_addr >> REG_BIT_WIDTH) << Q_ADDR_HI8_OFFSET;
	writel(val, priv->gmac_iobase + TX_RQ_DEPTH);
#endif
	writel((u32)phy_addr, priv->gmac_iobase + TX_RQ_START_ADDR);
	writel(0, priv->gmac_iobase + TX_RQ_REG_EN);
}

static void gmac_hw_set_desc_addr(struct gmac_netdev_local const *priv)
{
	u32 reg;
	int i;
#if defined(CONFIG_GMAC_DDR_64BIT )
	u32 val;
#endif

	gmac_set_rx_fq(priv, priv->RX_FQ.phys_addr);
	gmac_set_rx_bq(priv, priv->RX_BQ.phys_addr);
	gmac_set_tx_rq(priv, priv->TX_RQ.phys_addr);
	gmac_set_tx_bq(priv, priv->TX_BQ.phys_addr);

	for (i = 1; i < priv->num_rxqs; i++) {
		reg = rx_bq_depth_queue(i);
		/*
		 * Logical limitation: We must enable BITS_RX_BQ_DEPTH_EN
		 * to write rx_bq_start_addr_39to32 successfully.
		 */
		writel(BITS_RX_BQ_START_ADDR_EN | BITS_RX_BQ_DEPTH_EN,
		       priv->gmac_iobase + RX_BQ_REG_EN);
#if defined(CONFIG_GMAC_DDR_64BIT )
		val = readl(priv->gmac_iobase + reg);
		val &= Q_ADDR_HI8_MASK;
		val |= ((priv->pool[BASE_QUEUE_NUMS + i].phys_addr) >> REG_BIT_WIDTH) <<
		       Q_ADDR_HI8_OFFSET;
		writel(val, priv->gmac_iobase + reg);
#endif
		reg = (u32)rx_bq_start_addr_queue(i);
		/* pool 3 add i */
		writel((u32)(priv->pool[BASE_QUEUE_NUMS + i].phys_addr),
		       priv->gmac_iobase + reg);
		writel(0, priv->gmac_iobase + RX_BQ_REG_EN);
	}
}

static void gmac_set_rss_cap(struct gmac_netdev_local const *priv)
{
	u32 val = 0;

	if (priv->has_rxhash_cap)
		val |= BIT_RXHASH_CAP;
	if (priv->has_rss_cap)
		val |= BIT_RSS_CAP;
	writel(val, priv->gmac_iobase + HW_CAP_EN);
}

static void gmac_hw_init(struct gmac_netdev_local *priv)
{
	u32 val;
	u32 reg;
	int i;

	/* disable and clear all interrupts */
	writel(0, priv->gmac_iobase + ENA_PMU_INT);
	writel(~0, priv->gmac_iobase + RAW_PMU_INT);

	for (i = 1; i < priv->num_rxqs; i++) {
		reg = (u32)rss_ena_int_queue(i);
		writel(0, priv->gmac_iobase + reg);
	}
	writel(~0, priv->gmac_iobase + RSS_RAW_PMU_INT);

	/* enable CRC erro packets filter */
	val = readl(priv->gmac_iobase + REC_FILT_CONTROL);
	val |= BIT_CRC_ERR_PASS;
	writel(val, priv->gmac_iobase + REC_FILT_CONTROL);

	/* set tx min packet length */
	val = readl(priv->gmac_iobase + CRF_MIN_PACKET);
	val &= ~BIT_MASK_TX_MIN_LEN;
	val |= ETH_HLEN << BIT_OFFSET_TX_MIN_LEN;
	writel(val, priv->gmac_iobase + CRF_MIN_PACKET);

	/* fix bug for udp and ip error check */
	writel(CONTROL_WORD_CONFIG, priv->gmac_iobase + CONTROL_WORD);

	writel(0, priv->gmac_iobase + COL_SLOT_TIME);

	writel(DUPLEX_HALF, priv->gmac_iobase + MAC_DUPLEX_HALF_CTRL);

	/* interrupt when rcv packets >= RX_BQ_INT_THRESHOLD */
	val = RX_BQ_INT_THRESHOLD |
		(TX_RQ_INT_THRESHOLD << BITS_OFFSET_TX_RQ_IN_TH);
	writel(val, priv->gmac_iobase + IN_QUEUE_TH);

	/* RX_BQ/TX_RQ in timeout threshold */
	writel(0x10000, priv->gmac_iobase + RX_BQ_IN_TIMEOUT_TH);

	writel(0x18000, priv->gmac_iobase + TX_RQ_IN_TIMEOUT_TH);

	gmac_set_desc_depth(priv, RX_DESC_NUM, TX_DESC_NUM);
}

/*
 * the func stop the hw desc and relaim the software skb resource
 * before reusing the gmac, you'd better reset the gmac
 */
static void gmac_reclaim_rx_tx_resource(struct gmac_netdev_local *ld)
{
	unsigned long rxflags, txflags;
	int rd_offset, wr_offset;
	int i;

	if (ld == NULL)
		return;

	gmac_irq_disable_all_queue(ld);
	gmac_hw_desc_disable(ld);
	writel(STOP_RX_TX, ld->gmac_iobase + STOP_CMD);

	spin_lock_irqsave(&ld->rxlock, rxflags);
	/* RX_BQ: logic write pointer */
	wr_offset = readl(ld->gmac_iobase + RX_BQ_WR_ADDR);
	/* prevent to reclaim skb in rx bottom half */
	writel(wr_offset, ld->gmac_iobase + RX_BQ_RD_ADDR);

	for (i = 1; i < ld->num_rxqs; i++) {
		u32 rx_bq_wr_reg, rx_bq_rd_reg;

		rx_bq_wr_reg = rx_bq_wr_addr_queue(i);
		rx_bq_rd_reg = rx_bq_rd_addr_queue(i);

		wr_offset = readl(ld->gmac_iobase + rx_bq_wr_reg);
		writel(wr_offset, ld->gmac_iobase + rx_bq_rd_reg);
	}

	/* RX_FQ: logic read pointer */
	rd_offset = readl(ld->gmac_iobase + RX_FQ_RD_ADDR);
	if (rd_offset == 0)
		rd_offset = (RX_DESC_NUM - 1) << DESC_BYTE_SHIFT;
	else
		rd_offset -= DESC_SIZE;
	/* stop to feed hw desc */
	writel(rd_offset, ld->gmac_iobase + RX_FQ_WR_ADDR);

	for (i = 0; i < ld->RX_FQ.count; i++) {
		if (!ld->RX_FQ.skb[i])
			ld->RX_FQ.skb[i] = SKB_MAGIC;
	}
	spin_unlock_irqrestore(&ld->rxlock, rxflags);

	/*
	 * no need to wait pkts in TX_RQ finish to free all skb,
	 * because gmac_xmit_reclaim is in the tx_lock,
	 */
	spin_lock_irqsave(&ld->txlock, txflags);
	/* TX_RQ: logic write */
	wr_offset = readl(ld->gmac_iobase + TX_RQ_WR_ADDR);
	/* stop to reclaim tx skb */
	writel(wr_offset, ld->gmac_iobase + TX_RQ_RD_ADDR);

	/* TX_BQ: logic read */
	rd_offset = readl(ld->gmac_iobase + TX_BQ_RD_ADDR);
	if (rd_offset == 0)
		rd_offset = (TX_DESC_NUM - 1) << DESC_BYTE_SHIFT;
	else
		rd_offset -= DESC_SIZE;
	/* stop software tx skb */
	writel(rd_offset, ld->gmac_iobase + TX_BQ_WR_ADDR);

	for (i = 0; i < ld->TX_BQ.count; i++) {
		if (!ld->TX_BQ.skb[i])
			ld->TX_BQ.skb[i] = SKB_MAGIC;
	}
	spin_unlock_irqrestore(&ld->txlock, txflags);
}

void gmac_hw_set_mac_addr(struct net_device *dev)
{
	struct gmac_netdev_local *priv = netdev_priv(dev);
	unsigned char *mac = dev->dev_addr;
	u32 val;

	val = mac[1] | (mac[0] << 8); /* mac[1]->(7, 0) mac[0]->(15, 8) */
	writel(val, priv->gmac_iobase + STATION_ADDR_HIGH);
	val = mac[5] | (mac[4] << 8) | /* mac[5]->(7, 0) mac[4]->(8, 15) */
	      (mac[3] << 16) | (mac[2] << 24); /* mac[3]->(23, 16) mac[2]->(31, 24) */
	writel(val, priv->gmac_iobase + STATION_ADDR_LOW);
}

static void gmac_free_rx_skb(struct gmac_netdev_local const *ld)
{
	struct sk_buff *skb = NULL;
	int i;

	for (i = 0; i < ld->RX_FQ.count; i++) {
		skb = ld->RX_FQ.skb[i];
		if (skb != NULL) {
			ld->rx_skb[i] = NULL;
			ld->RX_FQ.skb[i] = NULL;
			if (skb == SKB_MAGIC)
				continue;
			dev_kfree_skb_any(skb);
			/*
			 * need to unmap the skb here
			 * but there is no way to get the dma_addr here,
			 * and unmap(TO_DEVICE) ops do nothing in fact,
			 * so we ignore to call
			 * dma_unmap_single(dev, dma_addr, skb->len,
			 *      DMA_TO_DEVICE)
			 */
		}
	}
}

static void gmac_free_tx_skb(struct gmac_netdev_local const *ld)
{
	struct sk_buff *skb = NULL;
	int i;

	for (i = 0; i < ld->TX_BQ.count; i++) {
		skb = ld->TX_BQ.skb[i];
		if (skb != NULL) {
			ld->tx_skb[i] = NULL;
			ld->TX_BQ.skb[i] = NULL;
			if (skb == SKB_MAGIC)
				continue;
			netdev_completed_queue(ld->netdev, 1, skb->len);
			dev_kfree_skb_any(skb);
		}
	}
}

/* board related func */
static void gmac_mac_core_reset(struct gmac_netdev_local const *priv)
{
	/* undo reset */
	if (priv == NULL || priv->port_rst == NULL)
		return;
	reset_control_deassert(priv->port_rst);
	usleep_range(50, 60); /* wait 50~60us */

	/* soft reset mac port */
	reset_control_assert(priv->port_rst);
	usleep_range(50, 60); /* wait 50~60us */
	/* undo reset */
	reset_control_deassert(priv->port_rst);
}

/* reset and re-config gmac */
static void gmac_restart(struct gmac_netdev_local *ld)
{
	unsigned long rxflags, txflags;

	if (ld == NULL || ld->netdev == NULL)
		return;
	/* restart hw engine now */
	gmac_mac_core_reset(ld);

	spin_lock_irqsave(&ld->rxlock, rxflags);
	spin_lock_irqsave(&ld->txlock, txflags);

	gmac_free_rx_skb(ld);
	gmac_free_tx_skb(ld);

	pmt_reg_restore(ld);
	gmac_hw_init(ld);
	gmac_hw_set_mac_addr(ld->netdev);
	gmac_hw_set_desc_addr(ld);

	/* we don't set macif here, it will be set in adjust_link */
	if (netif_running(ld->netdev)) {
		/*
		 * when resume, only do the following operations
		 * when dev is up before suspend.
		 */
		gmac_rx_refill(ld);
		gmac_set_multicast_list(ld->netdev);

		gmac_hw_desc_enable(ld);
		gmac_port_enable(ld);
		gmac_irq_enable_all_queue(ld);
	}
	spin_unlock_irqrestore(&ld->txlock, txflags);
	spin_unlock_irqrestore(&ld->rxlock, rxflags);
}

#define GMAC_LINK_CHANGE_PROTECT
#define GMAC_MAC_TX_RESET_IN_LINKUP

#ifdef GMAC_LINK_CHANGE_PROTECT
#define GMAC_MS_TO_NS (1000000ULL)
#define GMAC_FLUSH_WAIT_TIME (100*GMAC_MS_TO_NS)
/* protect code */
static void gmac_linkup_flush(struct gmac_netdev_local const *ld)
{
	int tx_bq_wr_offset, tx_bq_rd_offset;
	unsigned long long time_limit, time_now;

	time_now = sched_clock();
	time_limit = time_now + GMAC_FLUSH_WAIT_TIME;

	do {
		tx_bq_wr_offset = readl(ld->gmac_iobase + TX_BQ_WR_ADDR);
		tx_bq_rd_offset = readl(ld->gmac_iobase + TX_BQ_RD_ADDR);

		time_now = sched_clock();
		if (unlikely(((long long)time_now - (long long)time_limit) >= 0))
			break;
	} while (tx_bq_rd_offset != tx_bq_wr_offset);

	mdelay(1);
}
#endif

#ifdef GMAC_MAC_TX_RESET_IN_LINKUP
static void gmac_mac_tx_state_engine_reset(struct gmac_netdev_local const *priv)
{
	u32 val;
	val = readl(priv->gmac_iobase + MAC_CLEAR);
	val |= BIT_TX_SOFT_RESET;
	writel(val, priv->gmac_iobase + MAC_CLEAR);

	mdelay(5); /* wait 5ms */

	val = readl(priv->gmac_iobase + MAC_CLEAR);
	val &= ~BIT_TX_SOFT_RESET;
	writel(val, priv->gmac_iobase + MAC_CLEAR);
}
#endif

static void gmac_config_port(struct net_device const *dev, u32 speed, u32 duplex)
{
	struct gmac_netdev_local *priv = netdev_priv(dev);
	u32 val;

	switch (priv->phy_mode) {
	case PHY_INTERFACE_MODE_RGMII:
	case PHY_INTERFACE_MODE_RGMII_ID:
	case PHY_INTERFACE_MODE_RGMII_RXID:
	case PHY_INTERFACE_MODE_RGMII_TXID:
		if (speed == SPEED_1000)
			val = RGMII_SPEED_1000;
		else if (speed == SPEED_100)
			val = RGMII_SPEED_100;
		else
			val = RGMII_SPEED_10;
		break;
	case PHY_INTERFACE_MODE_MII:
		if (speed == SPEED_100)
			val = MII_SPEED_100;
		else
			val = MII_SPEED_10;
		break;
	case PHY_INTERFACE_MODE_RMII:
		if (speed == SPEED_100)
			val = RMII_SPEED_100;
		else
			val = RMII_SPEED_10;
		break;
	default:
		netdev_warn(dev, "not supported mode\n");
		val = MII_SPEED_10;
		break;
	}

	if (duplex)
		val |= GMAC_FULL_DUPLEX;

	reset_control_assert(priv->macif_rst);
	writel_relaxed(val, priv->macif_base);
	reset_control_deassert(priv->macif_rst);

	writel_relaxed(BIT_MODE_CHANGE_EN, priv->gmac_iobase + MODE_CHANGE_EN);
	if (speed == SPEED_1000)
		val = GMAC_SPEED_1000;
	else if (speed == SPEED_100)
		val = GMAC_SPEED_100;
	else
		val = GMAC_SPEED_10;
	writel_relaxed(val, priv->gmac_iobase + PORT_MODE);
	writel_relaxed(0, priv->gmac_iobase + MODE_CHANGE_EN);
	writel_relaxed(duplex, priv->gmac_iobase + MAC_DUPLEX_HALF_CTRL);
}

static unsigned int flow_ctrl_en = FLOW_OFF;
static int tx_flow_ctrl_pause_time = CONFIG_TX_FLOW_CTRL_PAUSE_TIME;
static int tx_flow_ctrl_pause_interval = CONFIG_TX_FLOW_CTRL_PAUSE_INTERVAL;
static int tx_flow_ctrl_active_threshold = CONFIG_TX_FLOW_CTRL_ACTIVE_THRESHOLD;
static int tx_flow_ctrl_deactive_threshold =
				CONFIG_TX_FLOW_CTRL_DEACTIVE_THRESHOLD;

static void gmac_set_flow_ctrl_args(struct gmac_netdev_local *ld)
{
	if (ld == NULL)
		return;
	ld->flow_ctrl = flow_ctrl_en;
	ld->pause = tx_flow_ctrl_pause_time;
	ld->pause_interval = tx_flow_ctrl_pause_interval;
	ld->flow_ctrl_active_threshold = tx_flow_ctrl_active_threshold;
	ld->flow_ctrl_deactive_threshold = tx_flow_ctrl_deactive_threshold;
}

static void gmac_set_flow_ctrl_params(struct gmac_netdev_local const *ld)
{
	unsigned int rx_fq_empty_th;
	unsigned int rx_fq_full_th;
	unsigned int rx_bq_empty_th;
	unsigned int rx_bq_full_th;
	unsigned int rec_filter;
	if (ld == NULL)
		return;
	writel(ld->pause, ld->gmac_iobase + FC_TX_TIMER);
	writel(ld->pause_interval, ld->gmac_iobase + PAUSE_THR);

	rx_fq_empty_th = readl(ld->gmac_iobase + RX_FQ_ALEMPTY_TH);
	rx_fq_empty_th &= ~(BITS_Q_PAUSE_TH_MASK << BITS_Q_PAUSE_TH_OFFSET);
	rx_fq_empty_th |= (ld->flow_ctrl_active_threshold <<
			BITS_Q_PAUSE_TH_OFFSET);
	writel(rx_fq_empty_th, ld->gmac_iobase + RX_FQ_ALEMPTY_TH);

	rx_fq_full_th = readl(ld->gmac_iobase + RX_FQ_ALFULL_TH);
	rx_fq_full_th &= ~(BITS_Q_PAUSE_TH_MASK << BITS_Q_PAUSE_TH_OFFSET);
	rx_fq_full_th |= (ld->flow_ctrl_deactive_threshold <<
			BITS_Q_PAUSE_TH_OFFSET);
	writel(rx_fq_full_th, ld->gmac_iobase + RX_FQ_ALFULL_TH);

	rx_bq_empty_th = readl(ld->gmac_iobase + RX_BQ_ALEMPTY_TH);
	rx_bq_empty_th &= ~(BITS_Q_PAUSE_TH_MASK << BITS_Q_PAUSE_TH_OFFSET);
	rx_bq_empty_th |= (ld->flow_ctrl_active_threshold <<
			BITS_Q_PAUSE_TH_OFFSET);
	writel(rx_bq_empty_th, ld->gmac_iobase + RX_BQ_ALEMPTY_TH);

	rx_bq_full_th = readl(ld->gmac_iobase + RX_BQ_ALFULL_TH);
	rx_bq_full_th &= ~(BITS_Q_PAUSE_TH_MASK << BITS_Q_PAUSE_TH_OFFSET);
	rx_bq_full_th |= (ld->flow_ctrl_deactive_threshold <<
			BITS_Q_PAUSE_TH_OFFSET);
	writel(rx_bq_full_th, ld->gmac_iobase + RX_BQ_ALFULL_TH);

	writel(0, ld->gmac_iobase + CRF_TX_PAUSE);

	rec_filter = readl(ld->gmac_iobase + REC_FILT_CONTROL);
	rec_filter |= BIT_PAUSE_FRM_PASS;
	writel(rec_filter, ld->gmac_iobase + REC_FILT_CONTROL);
}

void gmac_set_flow_ctrl_state(struct gmac_netdev_local const *ld, int pause)
{
	unsigned int flow_rx_q_en;
	unsigned int flow;
	if (ld == NULL)
		return;
	flow_rx_q_en = readl(ld->gmac_iobase + RX_PAUSE_EN);
	flow_rx_q_en &= ~(BIT_RX_FQ_PAUSE_EN | BIT_RX_BQ_PAUSE_EN);
	if (pause && (ld->flow_ctrl & FLOW_TX))
		flow_rx_q_en |= (BIT_RX_FQ_PAUSE_EN | BIT_RX_BQ_PAUSE_EN);
	writel(flow_rx_q_en, ld->gmac_iobase + RX_PAUSE_EN);

	flow = readl(ld->gmac_iobase + PAUSE_EN);
	flow &= ~(BIT_RX_FDFC | BIT_TX_FDFC);
	if (pause) {
		if (ld->flow_ctrl & FLOW_RX)
			flow |= BIT_RX_FDFC;
		if (ld->flow_ctrl & FLOW_TX)
			flow |= BIT_TX_FDFC;
	}
	writel(flow, ld->gmac_iobase + PAUSE_EN);
}

static void gmac_adjust_link(struct net_device *dev)
{
	struct gmac_netdev_local *priv = NULL;
	struct phy_device *phy = NULL;
	bool link_status_changed = false;
	if (dev == NULL)
		return;
	priv = netdev_priv(dev);
	if (priv == NULL || priv->phy == NULL)
		return;
	phy = priv->phy;
	if (phy->link) {
		if ((priv->old_speed != phy->speed) ||
		    (priv->old_duplex != phy->duplex)) {
#ifdef GMAC_LINK_CHANGE_PROTECT
			unsigned long txflags;

			spin_lock_irqsave(&priv->txlock, txflags);

			gmac_linkup_flush(priv);
#endif
			gmac_config_port(dev, phy->speed, phy->duplex);
#ifdef GMAC_MAC_TX_RESET_IN_LINKUP
			gmac_mac_tx_state_engine_reset(priv);
#endif
#ifdef GMAC_LINK_CHANGE_PROTECT
			spin_unlock_irqrestore(&priv->txlock, txflags);
#endif
			gmac_set_flow_ctrl_state(priv, phy->pause);

			if (priv->autoeee)
				init_autoeee(priv);

			link_status_changed = true;
			priv->old_link = 1;
			priv->old_speed = phy->speed;
			priv->old_duplex = phy->duplex;
		}
	} else if (priv->old_link) {
		link_status_changed = true;
		priv->old_link = 0;
		netif_carrier_off(dev);
		priv->old_speed = SPEED_UNKNOWN;
		priv->old_duplex = DUPLEX_UNKNOWN;
	}

	if (link_status_changed && netif_msg_link(priv))
		phy_print_status(phy);
}

static int gmac_init_sg_desc_queue(struct gmac_netdev_local *ld)
{
	ld->sg_count = ld->TX_BQ.count + GMAC_SG_DESC_ADD;
	ld->dma_sg_desc = (struct sg_desc *)dma_alloc_coherent(ld->dev,
					ld->sg_count * sizeof(struct sg_desc),
					&ld->dma_sg_phy, GFP_KERNEL);

	if (ld->dma_sg_desc == NULL) {
		pr_err("alloc sg desc dma error!\n");
		return -ENOMEM;
	}

	ld->sg_head = 0;
	ld->sg_tail = 0;

	return 0;
}

static void gmac_destroy_sg_desc_queue(struct gmac_netdev_local *ld)
{
	if (ld->dma_sg_desc) {
		dma_free_coherent(ld->dev,
				  ld->sg_count * sizeof(struct sg_desc),
				  ld->dma_sg_desc, ld->dma_sg_phy);
		ld->dma_sg_desc = NULL;
	}
}

static bool gmac_rx_fq_empty(struct gmac_netdev_local const *priv)
{
	u32 start, end;

	start = readl(priv->gmac_iobase + RX_FQ_WR_ADDR);
	end = readl(priv->gmac_iobase + RX_FQ_RD_ADDR);
	if (start == end)
		return true;
	else
		return false;
}

static bool gmac_rxq_has_packets(struct gmac_netdev_local const *priv, int rxq_id)
{
	u32 rx_bq_rd_reg, rx_bq_wr_reg;
	u32 start, end;

	rx_bq_rd_reg = rx_bq_rd_addr_queue(rxq_id);
	rx_bq_wr_reg = rx_bq_wr_addr_queue(rxq_id);

	start = readl(priv->gmac_iobase + rx_bq_rd_reg);
	end = readl(priv->gmac_iobase + rx_bq_wr_reg);
	if (start == end)
		return false;
	else
		return true;
}

static void gmac_trace(int level, const char *fmt, ...)
{
	if (level >= GMAC_TRACE_LEVEL) {
		va_list args;
		va_start(args, fmt);
		printk("gmac_trace:");
		printk(fmt, args);
		printk("\n");
		va_end(args);
	}
}

static void gmac_monitor_func(struct timer_list *t)
{
	struct gmac_netdev_local *ld = from_timer(ld, t, monitor);
	struct net_device *dev = NULL;
	u32 refill_cnt;

	if (ld == NULL) {
		gmac_trace(GMAC_NORMAL_LEVEL, "ld is null");
		return;
	}

	if (ld->netdev == NULL) {
		gmac_trace(GMAC_NORMAL_LEVEL, "ld->netdev is null");
		return;
	}
	dev_hold(ld->netdev);
	dev = ld->netdev;
	if (!netif_running(dev)) {
		dev_put(dev);
		gmac_trace(GMAC_NORMAL_LEVEL, "network driver is stopped");
		return;
	}
	dev_put(dev);

	spin_lock(&ld->rxlock);
	refill_cnt = gmac_rx_refill(ld);
	if (!refill_cnt && gmac_rx_fq_empty(ld)) {
		int rxq_id;

		for (rxq_id = 0; rxq_id < ld->num_rxqs; rxq_id++) {
			if (gmac_rxq_has_packets(ld, rxq_id))
				napi_schedule(&ld->q_napi[rxq_id].napi);
		}
	}
	spin_unlock(&ld->rxlock);

	ld->monitor.expires = jiffies + GMAC_MONITOR_TIMER;
	mod_timer(&ld->monitor, ld->monitor.expires);
}

u32 gmac_rx_refill(struct gmac_netdev_local *priv)
{
	struct gmac_desc *desc = NULL;
	struct sk_buff *skb = NULL;
	struct cyclic_queue_info dma_info;
	u32 len = ETH_MAX_FRAME_SIZE;
	dma_addr_t addr;
	u32 refill_cnt = 0;
	u32 i;
	/* software write pointer */
	dma_info.start = dma_cnt(readl(priv->gmac_iobase + RX_FQ_WR_ADDR));
	/* logic read pointer */
	dma_info.end = dma_cnt(readl(priv->gmac_iobase + RX_FQ_RD_ADDR));
	dma_info.num = CIRC_SPACE(dma_info.start, dma_info.end, RX_DESC_NUM);

	for (i = 0, dma_info.pos = dma_info.start; i < dma_info.num; i++) {
		if (priv->RX_FQ.skb[dma_info.pos] || priv->rx_skb[dma_info.pos])
			break;

		skb = netdev_alloc_skb_ip_align(priv->netdev, len);
		if (unlikely(skb == NULL))
			break;

		addr = dma_map_single(priv->dev, skb->data, len,
						DMA_FROM_DEVICE);
		if (dma_mapping_error(priv->dev, addr)) {
			dev_kfree_skb_any(skb);
			break;
		}

		desc = priv->RX_FQ.desc + dma_info.pos;
		desc->data_buff_addr = (u32)addr;
#if defined(CONFIG_GMAC_DDR_64BIT )
		desc->reserve31 = addr >> REG_BIT_WIDTH;
#endif
		priv->RX_FQ.skb[dma_info.pos] = skb;
		priv->rx_skb[dma_info.pos] = skb;

		desc->buffer_len = len - 1;
		desc->data_len = 0;
		desc->fl = 0;
		desc->descvid = DESC_VLD_FREE;
		desc->skb_id = dma_info.pos;

		refill_cnt++;
		dma_info.pos = dma_ring_incr(dma_info.pos, RX_DESC_NUM);
	}

	/*
	 * This barrier is important here.  It is required to ensure
	 * the ARM CPU flushes it's DMA write buffers before proceeding
	 * to the next instruction, to ensure that GMAC will see
	 * our descriptor changes in memory
	 */
	gmac_sync_barrier();

	if (dma_info.pos != dma_info.start)
		writel(dma_byte(dma_info.pos), priv->gmac_iobase + RX_FQ_WR_ADDR);

	return refill_cnt;
}

static int gmac_rx_checksum(struct net_device *dev, struct sk_buff *skb,
		  struct gmac_desc const *desc)
{
	int hdr_csum_done, payload_csum_done;
	int hdr_csum_err, payload_csum_err;
	if (skb == NULL || desc == NULL || dev == NULL)
		return -EINVAL;
	if (dev->features & NETIF_F_RXCSUM) {
		hdr_csum_done =	desc->header_csum_done;
		payload_csum_done =	desc->payload_csum_done;
		hdr_csum_err = desc->header_csum_err;
		payload_csum_err = desc->payload_csum_err;

		if (hdr_csum_done && payload_csum_done) {
			if (unlikely(hdr_csum_err || payload_csum_err)) {
				dev->stats.rx_errors++;
				dev->stats.rx_crc_errors++;
				dev_kfree_skb_any(skb);
				return -1;
			} else {
				skb->ip_summed = CHECKSUM_UNNECESSARY;
			}
		}
	}
	return 0;
}

static void gmac_rx_skbput(struct net_device *dev, struct sk_buff *skb,
			     struct gmac_desc const *desc, int rxq_id)
{
	struct gmac_netdev_local *ld = netdev_priv(dev);
	dma_addr_t addr;
	u32 len;
	int ret;

	len = desc->data_len;

	addr = desc->data_buff_addr;
#if defined(CONFIG_GMAC_DDR_64BIT )
	addr |= (dma_addr_t)(desc->reserve31) << REG_BIT_WIDTH;
#endif
	dma_unmap_single(ld->dev, addr, ETH_MAX_FRAME_SIZE, DMA_FROM_DEVICE);

	if ((addr & NET_IP_ALIGN) == 0)
		skb_reserve(skb, 2); /* 2:NET_IP_ALIGN */

	skb_put(skb, len);
	if (skb->len > ETH_MAX_FRAME_SIZE) {
		netdev_err(dev, "rcv len err, len = %d\n", skb->len);
		dev->stats.rx_errors++;
		dev->stats.rx_length_errors++;
		dev_kfree_skb_any(skb);
		return;
	}

	skb->protocol = eth_type_trans(skb, dev);
	skb->ip_summed = CHECKSUM_NONE;

#if defined(CONFIG_GMAC_RXCSUM)
	ret = gmac_rx_checksum(dev, skb, desc);
	if (unlikely(ret))
		return;
#endif
	if ((dev->features & NETIF_F_RXHASH) && desc->has_hash)
		skb_set_hash(skb, desc->rxhash, desc->l3_hash ?
			     PKT_HASH_TYPE_L3 : PKT_HASH_TYPE_L4);

	skb_record_rx_queue(skb, rxq_id);

	napi_gro_receive(&ld->q_napi[rxq_id].napi, skb);
	dev->stats.rx_packets++;
	dev->stats.rx_bytes += len;
}

static int gmac_rx_skb(struct net_device *dev, struct gmac_desc *desc,
			 u16 skb_id, int rxq_id)
{
	struct gmac_netdev_local *ld = netdev_priv(dev);
	struct sk_buff *skb = NULL;

	spin_lock(&ld->rxlock);
	skb = ld->rx_skb[skb_id];
	if (unlikely(skb == NULL)) {
		spin_unlock(&ld->rxlock);
		netdev_err(dev, "inconsistent rx_skb\n");
		return -1;
	}

	/* data consistent check */
	if (unlikely(skb != ld->RX_FQ.skb[skb_id])) {
		netdev_err(dev, "desc->skb(0x%p),RX_FQ.skb[%d](0x%p)\n",
			   skb, skb_id, ld->RX_FQ.skb[skb_id]);
		if (ld->RX_FQ.skb[skb_id] == SKB_MAGIC) {
			spin_unlock(&ld->rxlock);
			return 0;
		}
		WARN_ON(1);
	} else {
		ld->RX_FQ.skb[skb_id] = NULL;
	}
	spin_unlock(&ld->rxlock);

	gmac_rx_skbput(dev, skb, desc, rxq_id);
	return 0;
}

static int gmac_rx(struct net_device *dev, int limit, int rxq_id)
{
	struct gmac_netdev_local *ld = netdev_priv(dev);
	struct gmac_desc *desc = NULL;
	struct cyclic_queue_info dma_info;
	u32 rx_bq_rd_reg, rx_bq_wr_reg;
	u16 skb_id;
	u32 i;

	rx_bq_rd_reg = rx_bq_rd_addr_queue(rxq_id);
	rx_bq_wr_reg = rx_bq_wr_addr_queue(rxq_id);

	/* software read pointer */
	dma_info.start = dma_cnt(readl(ld->gmac_iobase + rx_bq_rd_reg));
	/* logic write pointer */
	dma_info.end = dma_cnt(readl(ld->gmac_iobase + rx_bq_wr_reg));
	dma_info.num = CIRC_CNT(dma_info.end, dma_info.start, RX_DESC_NUM);
	if (dma_info.num > limit)
		dma_info.num = limit;

	/* ensure get updated desc */
	rmb();
	for (i = 0, dma_info.pos = dma_info.start; i < dma_info.num; i++) {
		if (rxq_id)
			desc = ld->pool[BASE_QUEUE_NUMS + rxq_id].desc + dma_info.pos;
		else
			desc = ld->RX_BQ.desc + dma_info.pos;
		skb_id = desc->skb_id;

		if (unlikely(gmac_rx_skb(dev, desc, skb_id, rxq_id)))
			break;

		spin_lock(&ld->rxlock);
		ld->rx_skb[skb_id] = NULL;
		spin_unlock(&ld->rxlock);
		dma_info.pos = dma_ring_incr(dma_info.pos, RX_DESC_NUM);
	}

	if (dma_info.pos != dma_info.start)
		writel(dma_byte(dma_info.pos), ld->gmac_iobase + rx_bq_rd_reg);

	spin_lock(&ld->rxlock);
	gmac_rx_refill(ld);
	spin_unlock(&ld->rxlock);

	return dma_info.num;
}

static int gmac_check_tx_err(struct gmac_netdev_local const *ld,
			       struct gmac_tso_desc const *tx_bq_desc, unsigned int desc_pos)
{
	unsigned int tx_err = tx_bq_desc->tx_err;

	if (unlikely(tx_err & ERR_ALL)) {
		struct sg_desc *desc_cur = NULL;
		int *sg_word = NULL;
		int i;

		WARN((tx_err & ERR_ALL),
		     "TX ERR: desc1=0x%x, desc2=0x%x, desc5=0x%x\n",
		     tx_bq_desc->data_buff_addr,
		     tx_bq_desc->desc1.val, tx_bq_desc->tx_err);

		desc_cur = ld->dma_sg_desc + ld->TX_BQ.sg_desc_offset[desc_pos];
		sg_word = (int *)desc_cur;
		for (i = 0; i < sizeof(struct sg_desc) / sizeof(int); i++)
			pr_err("%s,%d: sg_desc word[%d]=0x%x\n",
			       __func__, __LINE__, i, sg_word[i]);

		return -1;
	}

	return 0;
}

static void gmac_xmit_release_gso_sg(struct gmac_netdev_local *ld,
				       struct gmac_tso_desc const *tx_rq_desc, unsigned int desc_pos)
{
	struct sg_desc *desc_cur = NULL;
	int nfrags = tx_rq_desc->desc1.tx.nfrags_num;
	unsigned int desc_offset;
	dma_addr_t addr;
	size_t len;
	int i;

	desc_offset = ld->TX_BQ.sg_desc_offset[desc_pos];
	WARN_ON(desc_offset != ld->sg_tail);
	desc_cur = ld->dma_sg_desc + desc_offset;

	addr = desc_cur->linear_addr;
#if defined(CONFIG_GMAC_DDR_64BIT )
	addr |= (dma_addr_t)(desc_cur->reserv3 >> SG_DESC_HI8_OFFSET) << REG_BIT_WIDTH;
#endif
	len = desc_cur->linear_len;
	dma_unmap_single(ld->dev, addr, len, DMA_TO_DEVICE);
	for (i = 0; i < nfrags; i++) {
		addr = desc_cur->frags[i].addr;
#if defined(CONFIG_GMAC_DDR_64BIT )
		addr |= (dma_addr_t) (desc_cur->frags[i].reserved >> SG_DESC_HI8_OFFSET) << REG_BIT_WIDTH;
#endif
		len = desc_cur->frags[i].size;
		dma_unmap_page(ld->dev, addr, len, DMA_TO_DEVICE);
	}
}

static int gmac_xmit_release_gso(struct gmac_netdev_local *ld,
				   struct gmac_tso_desc *tx_rq_desc, unsigned int desc_pos)
{
	int pkt_type;
	unsigned int nfrags = tx_rq_desc->desc1.tx.nfrags_num;
	dma_addr_t addr;
	size_t len;

	if (unlikely(gmac_check_tx_err(ld, tx_rq_desc, desc_pos) < 0)) {
		/* dev_close */
		gmac_irq_disable_all_queue(ld);
		gmac_hw_desc_disable(ld);

		netif_carrier_off(ld->netdev);
		netif_stop_queue(ld->netdev);

		phy_stop(ld->phy);
		del_timer_sync(&ld->monitor);
		return -1;
	}

	if (tx_rq_desc->desc1.tx.tso_flag || (nfrags != 0))
		pkt_type = PKT_SG;
	else
		pkt_type = PKT_NORMAL;

	if (pkt_type == PKT_NORMAL) {
		addr = tx_rq_desc->data_buff_addr;
#if defined(CONFIG_GMAC_DDR_64BIT)
		addr |= (dma_addr_t)(tx_rq_desc->reserve_desc2 & TX_DESC_HI8_MASK) << REG_BIT_WIDTH;
#endif
		len = tx_rq_desc->desc1.tx.data_len;
		dma_unmap_single(ld->dev, addr, len, DMA_TO_DEVICE);
	} else {
		gmac_xmit_release_gso_sg(ld, tx_rq_desc, desc_pos);

		ld->sg_tail = (ld->sg_tail + 1) % ld->sg_count;
	}

	return 0;
}

static int gmac_xmit_reclaim_release(struct net_device const *dev,
				       struct sk_buff *skb, struct gmac_desc *desc, u32 pos)
{
	struct gmac_netdev_local *priv = netdev_priv(dev);
	struct gmac_tso_desc *tso_desc = NULL;
	dma_addr_t addr;

	if (priv->tso_supported) {
		tso_desc = (struct gmac_tso_desc *)desc;
		return gmac_xmit_release_gso(priv, tso_desc, pos);
	} else {
		addr = desc->data_buff_addr;
#if defined(CONFIG_GMAC_DDR_64BIT )
		addr |= (dma_addr_t)(desc->rxhash & TX_DESC_HI8_MASK) << REG_BIT_WIDTH;
#endif
		dma_unmap_single(priv->dev, addr, skb->len, DMA_TO_DEVICE);
	}
	return 0;
}

static void gmac_xmit_reclaim(struct net_device *dev)
{
	struct sk_buff *skb = NULL;
	struct gmac_desc *desc = NULL;
	struct gmac_netdev_local *priv = netdev_priv(dev);
	unsigned int bytes_compl = 0;
	unsigned int pkts_compl = 0;
	struct cyclic_queue_info dma_info;
	u32 i;

	spin_lock(&priv->txlock);

	/* software read */
	dma_info.start = dma_cnt(readl(priv->gmac_iobase + TX_RQ_RD_ADDR));
	/* logic write */
	dma_info.end = dma_cnt(readl(priv->gmac_iobase + TX_RQ_WR_ADDR));
	dma_info.num = CIRC_CNT(dma_info.end, dma_info.start, TX_DESC_NUM);

	for (i = 0, dma_info.pos = dma_info.start; i < dma_info.num; i++) {
		skb = priv->tx_skb[dma_info.pos];
		if (unlikely(skb == NULL)) {
			netdev_err(dev, "inconsistent tx_skb\n");
			break;
		}

		if (skb != priv->TX_BQ.skb[dma_info.pos]) {
			netdev_err(dev, "wired, tx skb[%d](%p) != skb(%p)\n",
				   dma_info.pos, priv->TX_BQ.skb[dma_info.pos], skb);
			if (priv->TX_BQ.skb[dma_info.pos] == SKB_MAGIC)
				goto next;
		}

		pkts_compl++;
		bytes_compl += skb->len;
		desc = priv->TX_RQ.desc + dma_info.pos;
		if (gmac_xmit_reclaim_release(dev, skb, desc, dma_info.pos) < 0)
			break;

		priv->TX_BQ.skb[dma_info.pos] = NULL;
next:
		priv->tx_skb[dma_info.pos] = NULL;
		dev_consume_skb_any(skb);
		dma_info.pos = dma_ring_incr(dma_info.pos, TX_DESC_NUM);
	}

	if (dma_info.pos != dma_info.start)
		writel(dma_byte(dma_info.pos), priv->gmac_iobase + TX_RQ_RD_ADDR);

	if ((pkts_compl != 0) || (bytes_compl != 0))
		netdev_completed_queue(dev, pkts_compl, bytes_compl);

	if (unlikely(netif_queue_stopped(priv->netdev)) && (pkts_compl != 0))
		netif_wake_queue(priv->netdev);

	spin_unlock(&priv->txlock);
}

static int gmac_poll(struct napi_struct *napi, int budget)
{
	struct gmac_napi *q_napi = container_of(napi,
						  struct gmac_napi, napi);
	struct gmac_netdev_local *priv = q_napi->ndev_priv;
	struct net_device *dev = priv->netdev;
	int work_done = 0;
	int task = budget;
	u32 ints, num;
	u32 raw_int_reg, raw_int_mask;

	dev_hold(dev);
	if (q_napi->rxq_id) {
		raw_int_reg = RSS_RAW_PMU_INT;
		raw_int_mask = def_int_mask_queue((u32)q_napi->rxq_id);
	} else {
		raw_int_reg = RAW_PMU_INT;
		raw_int_mask = DEF_INT_MASK;
	}

	do {
		if (!q_napi->rxq_id)
			gmac_xmit_reclaim(dev);
		num = gmac_rx(dev, task, q_napi->rxq_id);
		work_done += num;
		task -= num;
		if (work_done >= budget)
			break;

		ints = readl(priv->gmac_iobase + raw_int_reg);
		ints &= raw_int_mask;
		writel(ints, priv->gmac_iobase + raw_int_reg);
	} while (ints || gmac_rxq_has_packets(priv, q_napi->rxq_id));

	if (work_done < budget) {
		napi_complete(napi);
		gmac_irq_enable_queue(priv, q_napi->rxq_id);
	}

	dev_put(dev);
	return work_done;
}

static irqreturn_t gmac_interrupt(int irq, void *dev_id)
{
	struct gmac_napi *q_napi = (struct gmac_napi *)dev_id;
	struct gmac_netdev_local *ld = q_napi->ndev_priv;
	u32 ints;
	u32 raw_int_reg, raw_int_mask;

	if (gmac_queue_irq_disabled(ld, q_napi->rxq_id))
		return IRQ_NONE;

	if (q_napi->rxq_id) {
		raw_int_reg = RSS_RAW_PMU_INT;
		raw_int_mask = def_int_mask_queue((u32)q_napi->rxq_id);
	} else {
		raw_int_reg = RAW_PMU_INT;
		raw_int_mask = DEF_INT_MASK;
	}

	ints = readl(ld->gmac_iobase + raw_int_reg);
	ints &= raw_int_mask;
	writel(ints, ld->gmac_iobase + raw_int_reg);

	if (likely(ints || gmac_rxq_has_packets(ld, q_napi->rxq_id))) {
		gmac_irq_disable_queue(ld, q_napi->rxq_id);
		napi_schedule(&q_napi->napi);
	}

	return IRQ_HANDLED;
}

void gmac_enable_napi(struct gmac_netdev_local *priv)
{
	struct gmac_napi *q_napi = NULL;
	int i;

	if (priv == NULL)
		return;

	for (i = 0; i < priv->num_rxqs; i++) {
		q_napi = &priv->q_napi[i];
		napi_enable(&q_napi->napi);
	}
}

void gmac_disable_napi(struct gmac_netdev_local *priv)
{
	struct gmac_napi *q_napi = NULL;
	int i;

	if (priv == NULL)
		return;

	for (i = 0; i < priv->num_rxqs; i++) {
		q_napi = &priv->q_napi[i];
		napi_disable(&q_napi->napi);
	}
}

void gmac_enable_rxcsum_drop(struct gmac_netdev_local const *ld, bool drop)
{
	unsigned int v;

	v = readl(ld->gmac_iobase + TSO_COE_CTRL);
	if (drop)
		v |= COE_ERR_DROP;
	else
		v &= ~COE_ERR_DROP;
	writel(v, ld->gmac_iobase + TSO_COE_CTRL);
}

static const struct ethtool_ops eth_ethtools_ops = {
	.get_drvinfo = gmac_get_drvinfo,
	.get_link = gmac_get_link,
	.get_pauseparam = gmac_get_pauseparam,
	.set_pauseparam = gmac_set_pauseparam,
	.get_msglevel = gmac_ethtool_getmsglevel,
	.set_msglevel = gmac_ethtool_setmsglevel,
	.get_rxfh_key_size = gmac_get_rxfh_key_size,
	.get_rxfh_indir_size = gmac_get_rxfh_indir_size,
	.get_rxfh = gmac_get_rxfh,
	.set_rxfh = gmac_set_rxfh,
	.get_rxnfc = gmac_get_rxnfc,
	.set_rxnfc = gmac_set_rxnfc,
	.get_link_ksettings = phy_ethtool_get_link_ksettings,
	.set_link_ksettings = phy_ethtool_set_link_ksettings,
};

static const struct net_device_ops eth_netdev_ops = {
	.ndo_open = gmac_net_open,
	.ndo_stop = gmac_net_close,
	.ndo_start_xmit = gmac_net_xmit,
	.ndo_set_rx_mode = gmac_set_multicast_list,
	.ndo_set_features = gmac_set_features,
	.ndo_do_ioctl = gmac_ioctl,
	.ndo_set_mac_address = gmac_net_set_mac_address,
	.ndo_change_mtu = eth_change_mtu,
	.ndo_get_stats = gmac_net_get_stats,
};

static int gmac_of_get_param(struct gmac_netdev_local *ld,
			       struct device_node const *node)
{
	/* get auto eee */
	ld->autoeee = of_property_read_bool(node, "autoeee");
	/* get internal flag */
	ld->internal_phy =
		of_property_read_bool(node, "internal-phy");

	return 0;
}

static void gmac_destroy_hw_desc_queue(struct gmac_netdev_local *priv)
{
	int i;

	for (i = 0; i < QUEUE_NUMS + RSS_NUM_RXQS - 1; i++) {
		if (priv->pool[i].desc) {
			dma_free_coherent(priv->dev, priv->pool[i].size,
					  priv->pool[i].desc,
					  priv->pool[i].phys_addr);
			priv->pool[i].desc = NULL;
		}
	}

	kfree(priv->RX_FQ.skb);
	kfree(priv->TX_BQ.skb);
	priv->RX_FQ.skb = NULL;
	priv->TX_BQ.skb = NULL;

	if (priv->tso_supported) {
		kfree(priv->TX_BQ.sg_desc_offset);
		priv->TX_BQ.sg_desc_offset = NULL;
	}

	kfree(priv->tx_skb);
	priv->tx_skb = NULL;

	kfree(priv->rx_skb);
	priv->rx_skb = NULL;
}

static int gmac_init_desc_queue_mem(struct gmac_netdev_local *priv)
{
	priv->RX_FQ.skb = kzalloc(priv->RX_FQ.count
				  * sizeof(struct sk_buff *), GFP_KERNEL);
	if (!priv->RX_FQ.skb)
		return -ENOMEM;

	priv->rx_skb = kzalloc(priv->RX_FQ.count
			       * sizeof(struct sk_buff *), GFP_KERNEL);
	if (priv->rx_skb == NULL)
		return -ENOMEM;

	priv->TX_BQ.skb = kzalloc(priv->TX_BQ.count
				  * sizeof(struct sk_buff *), GFP_KERNEL);
	if (!priv->TX_BQ.skb)
		return -ENOMEM;

	priv->tx_skb = kzalloc(priv->TX_BQ.count
			       * sizeof(struct sk_buff *), GFP_KERNEL);
	if (priv->tx_skb == NULL)
		return -ENOMEM;

	if (priv->tso_supported) {
		priv->TX_BQ.sg_desc_offset = kzalloc(priv->TX_BQ.count
						     * sizeof(int), GFP_KERNEL);
		if (!priv->TX_BQ.sg_desc_offset)
			return -ENOMEM;
	}

	return 0;
}

static int gmac_init_hw_desc_queue(struct gmac_netdev_local *priv)
{
	struct device *dev = NULL;
	struct gmac_desc *virt_addr = NULL;
	dma_addr_t phys_addr = 0;
	int size, i;
	if (priv == NULL || priv->dev == NULL)
		return -EINVAL;
	dev = priv->dev;
	if (dev == NULL)
		return -EINVAL;
	priv->RX_FQ.count = RX_DESC_NUM;
	priv->RX_BQ.count = RX_DESC_NUM;
	priv->TX_BQ.count = TX_DESC_NUM;
	priv->TX_RQ.count = TX_DESC_NUM;

	for (i = 1; i < RSS_NUM_RXQS; i++)
		priv->pool[BASE_QUEUE_NUMS + i].count = RX_DESC_NUM;

	for (i = 0; i < (QUEUE_NUMS + RSS_NUM_RXQS - 1); i++) {
		size = priv->pool[i].count * sizeof(struct gmac_desc);
		virt_addr = dma_alloc_coherent(dev, size, &phys_addr, GFP_KERNEL);
		if (virt_addr == NULL)
			goto error_free_pool;

		if (memset_s(virt_addr, size, 0, size) != EOK) {
			pr_info("gmac init hw desc queue: memset_s failed\n");
			goto error_free_pool;
		}
		priv->pool[i].size = (unsigned int)size;
		priv->pool[i].desc = virt_addr;
		priv->pool[i].phys_addr = phys_addr;
	}

	if (gmac_init_desc_queue_mem(priv) == -ENOMEM)
		goto error_free_pool;

	gmac_hw_set_desc_addr(priv);

	return 0;

error_free_pool:
	gmac_destroy_hw_desc_queue(priv);

	return -ENOMEM;
}

static void gmac_init_napi(struct gmac_netdev_local *priv)
{
	struct gmac_napi *q_napi = NULL;
	int i;

	if (priv == NULL || priv->netdev == NULL)
		return;

	for (i = 0; i < priv->num_rxqs; i++) {
		q_napi = &priv->q_napi[i];
		q_napi->rxq_id = (unsigned int)i;
		q_napi->ndev_priv = priv;
		netif_napi_add(priv->netdev, &q_napi->napi, gmac_poll,
			       NAPI_POLL_WEIGHT);
	}
}

static void gmac_destroy_napi(struct gmac_netdev_local *priv)
{
	struct gmac_napi *q_napi = NULL;
	int i;

	if (priv == NULL)
		return;

	for (i = 0; i < priv->num_rxqs; i++) {
		q_napi = &priv->q_napi[i];
		netif_napi_del(&q_napi->napi);
	}
}

static int gmac_request_irqs(struct platform_device *pdev,
			struct gmac_netdev_local *priv)
{
	struct device *dev = NULL;
	int ret;
	int i;

	if (priv == NULL || pdev == NULL || pdev->name == NULL)
		return -1;

	dev = priv->dev;
	if (dev == NULL)
		return -1;

	for (i = 0; i < priv->num_rxqs; i++) {
		ret = platform_get_irq(pdev, i);
		if (ret < 0) {
			dev_err(dev, "No irq[%d] resource, ret=%d\n", i, ret);
			return ret;
		}
		priv->irq[i] = (unsigned int)ret;

		ret = devm_request_irq(dev, priv->irq[i], gmac_interrupt,
				  IRQF_SHARED, pdev->name, &priv->q_napi[i]);
		if (ret) {
			dev_err(dev, "devm_request_irq failed, ret=%d\n", ret);
			return ret;
		}
	}

	return 0;
}

static int gmac_dev_probe_res(struct platform_device *pdev,
				struct gmac_netdev_local *priv)
{
	struct device *dev = &pdev->dev;
	struct net_device *ndev = priv->netdev;
	struct resource *res = NULL;
	int ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, MEM_GMAC_IOBASE);
	priv->gmac_iobase = devm_ioremap_resource(dev, res);
	if (IS_ERR(priv->gmac_iobase)) {
		ret = PTR_ERR(priv->gmac_iobase);
		return ret;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, MEM_MACIF_IOBASE);
	priv->macif_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(priv->macif_base)) {
		ret = PTR_ERR(priv->macif_base);
		return ret;
	}

	/* only for some chip to fix AXI bus burst and outstanding config */
	res = platform_get_resource(pdev, IORESOURCE_MEM, MEM_AXI_BUS_CFG_IOBASE);
	priv->axi_bus_cfg_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(priv->axi_bus_cfg_base))
		priv->axi_bus_cfg_base = NULL;

	priv->port_rst = devm_reset_control_get(dev, GMAC_PORT_RST_NAME);
	if (IS_ERR(priv->port_rst)) {
		ret = PTR_ERR(priv->port_rst);
		return ret;
	}

	priv->macif_rst = devm_reset_control_get(dev, GMAC_MACIF_RST_NAME);
	if (IS_ERR(priv->macif_rst)) {
		ret = PTR_ERR(priv->macif_rst);
		return ret;
	}

	priv->phy_rst = devm_reset_control_get(dev, GMAC_PHY_RST_NAME);
	if (IS_ERR(priv->phy_rst))
		priv->phy_rst = NULL;

	priv->clk = devm_clk_get(&pdev->dev, GMAC_MAC_CLK_NAME);
	if (IS_ERR(priv->clk)) {
		netdev_err(ndev, "failed to get clk\n");
		ret = -ENODEV;
		return ret;
	}

	ret = clk_prepare_enable(priv->clk);
	if (ret < 0) {
		netdev_err(ndev, "failed to enable clk %d\n", ret);
		return ret;
	}
	return 0;
}

static int gmac_dev_macif_clk(struct platform_device *pdev,
				struct gmac_netdev_local *priv, struct net_device *ndev)
{
	int ret;

	priv->macif_clk = devm_clk_get(&pdev->dev, GMAC_MACIF_CLK_NAME);
	if (IS_ERR(priv->macif_clk))
		priv->macif_clk = NULL;

	if (priv->macif_clk != NULL) {
		ret = clk_prepare_enable(priv->macif_clk);
		if (ret < 0) {
			netdev_err(ndev, "failed enable macif_clk %d\n", ret);
			return ret;
		}
	}
	return 0;
}

static int gmac_dev_probe_init(struct platform_device *pdev,
				 struct gmac_netdev_local *priv, struct net_device *ndev)
{
	int ret;
#if defined(CONFIG_GMAC_DDR_64BIT)
	struct device *dev = &pdev->dev;
#endif

	gmac_init_napi(priv);
	spin_lock_init(&priv->rxlock);
	spin_lock_init(&priv->txlock);
	spin_lock_init(&priv->pmtlock);

	/* init netdevice */
	ndev->irq = priv->irq[0];
	ndev->watchdog_timeo = 3 * HZ; /* 3HZ */
	ndev->netdev_ops = &eth_netdev_ops;
	ndev->ethtool_ops = &eth_ethtools_ops;

	if (priv->has_rxhash_cap)
		ndev->hw_features |= NETIF_F_RXHASH;
	if (priv->has_rss_cap)
		ndev->hw_features |= NETIF_F_NTUPLE;
	if (priv->tso_supported)
		ndev->hw_features |= NETIF_F_SG |
				     NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM |
				     NETIF_F_TSO | NETIF_F_TSO6;

#if defined(CONFIG_GMAC_RXCSUM)
	ndev->hw_features |= NETIF_F_RXCSUM;
	gmac_enable_rxcsum_drop(priv, true);
#endif

	ndev->features |= ndev->hw_features;
	ndev->features |= NETIF_F_HIGHDMA | NETIF_F_GSO;
	ndev->vlan_features |= ndev->features;

	timer_setup(&priv->monitor, gmac_monitor_func, 0);

	device_set_wakeup_capable(priv->dev, 1);
	/*
	 * when we can let phy powerdown?
	 * In some mode, we don't want phy powerdown,
	 * so I set wakeup enable all the time
	 */
	device_set_wakeup_enable(priv->dev, 1);

	priv->wol_enable = false;

	priv->msg_enable = netif_msg_init(debug, DEFAULT_MSG_ENABLE);

#if defined(CONFIG_GMAC_DDR_64BIT )
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64)); /* 64bit */
	if (ret) {
		pr_err("dma set mask 64 failed! ret=%d", ret);
		return ret;
	}
#endif

	/* init hw desc queue */
	ret = gmac_init_hw_desc_queue(priv);
	if (ret)
		return ret;

	return 0;
}

static int gmac_dev_probe_phy(struct platform_device *pdev,
				struct gmac_netdev_local *priv, struct net_device *ndev,
				bool fixed_link)
{
	int ret;

	/* phy fix here?? other way ??? */
	gmac_phy_register_fixups();
	/* Unable to handle kernel paging request at virtual address 08ffffff80052fc0  */
	priv->phy = of_phy_connect(ndev, priv->phy_node,
				   &gmac_adjust_link, 0, priv->phy_mode);
	if (priv->phy == NULL || priv->phy->drv == NULL) {
		ret = -ENODEV;
		return ret;
	}

	/* If the phy_id is all zero and not fixed link, there is no device there */
	if ((priv->phy->phy_id == 0) && !fixed_link) {
		pr_info("phy %d not found\n", priv->phy->mdio.addr);
		ret = -ENODEV;
		return ret;
	}

	pr_info("attached PHY %d to driver %s, PHY_ID=0x%x\n",
		priv->phy->mdio.addr, priv->phy->drv->name, priv->phy->phy_id);

	/* Stop Advertising 1000BASE Capability if interface is not RGMII */
	if ((priv->phy_mode == PHY_INTERFACE_MODE_MII) ||
			(priv->phy_mode == PHY_INTERFACE_MODE_RMII)) {
		linkmode_clear_bit(ETHTOOL_LINK_MODE_1000baseT_Full_BIT | ETHTOOL_LINK_MODE_1000baseT_Half_BIT,
						   priv->phy->advertising);
		/*
		 * Internal FE phy's reg BMSR bit8 is wrong, make the kernel
		 * believe it has the 1000base Capability, so fix it here
		 */
		if (priv->phy->phy_id == VENDOR_PHY_ID_FESTAV200)
			linkmode_clear_bit(ETHTOOL_LINK_MODE_1000baseT_Full_BIT | ETHTOOL_LINK_MODE_1000baseT_Half_BIT,
							   priv->phy->supported);
	}

	gmac_set_flow_ctrl_args(priv);
	gmac_set_flow_ctrl_params(priv);
	linkmode_set_bit(ETHTOOL_LINK_MODE_Pause_BIT, priv->phy->supported);
	if (priv->flow_ctrl)
		linkmode_set_bit(ETHTOOL_LINK_MODE_Pause_BIT, priv->phy->advertising);
	if (priv->autoeee)
		init_autoeee(priv);

	ret = gmac_request_irqs(pdev, priv);
	if (ret)
		return ret;

	return 0;
}

static void gmac_set_hw_cap(struct platform_device *pdev,
			      struct gmac_netdev_local *priv)
{
	unsigned int hw_cap;

	hw_cap = readl(priv->gmac_iobase + CRF_MIN_PACKET);
	priv->tso_supported = has_tso_cap(hw_cap);
	priv->has_rxhash_cap = has_rxhash_cap(hw_cap);
	priv->has_rss_cap = has_rss_cap(hw_cap);

	gmac_set_rss_cap(priv);
	gmac_get_rss_key(priv);
	if (priv->has_rss_cap) {
		priv->rss_info.ind_tbl_size = RSS_INDIRECTION_TABLE_SIZE;
		gmac_get_rss(priv);
	}

	if (priv->has_rxhash_cap) {
		priv->rss_info.hash_cfg = DEF_HASH_CFG;
		gmac_config_hash_policy(priv);
	}
}

static void gmac_set_mac_addr(struct net_device *ndev,
				struct device_node *node)
{
	const char *mac_addr = NULL;

	mac_addr = of_get_mac_address(node);
	if (!IS_ERR_OR_NULL(mac_addr))
		ether_addr_copy(ndev->dev_addr, mac_addr);
	else
		eth_hw_addr_random(ndev);

	gmac_hw_set_mac_addr(ndev);
}

/* board independent func */
static void gmac_hw_internal_phy_reset(struct gmac_netdev_local const *priv)
{
}

/* board independent func */
static void gmac_hw_external_phy_reset(struct gmac_netdev_local const *priv)
{
	if (priv == NULL)
		return;
	if (priv->phy_rst != NULL) {
		/* write 0 to cancel reset */
		reset_control_deassert(priv->phy_rst);
		msleep(50); /* wait 50ms */

		/* XX use CRG register to reset phy */
		/* RST_BIT, write 0 to reset phy, write 1 to cancel reset */
		reset_control_assert(priv->phy_rst);

		/*
		 * delay some time to ensure reset ok,
		 * this depends on PHY hardware feature
		 */
		msleep(50); /* wait 50ms */

		/* write 0 to cancel reset */
		reset_control_deassert(priv->phy_rst);
		/* delay some time to ensure later MDIO access */
		msleep(50); /* wait 50ms */
	}
}

/* board independent func */
static void gmac_hw_phy_reset(struct gmac_netdev_local *priv)
{
	if (priv == NULL)
		return;
	if (priv->internal_phy)
		gmac_hw_internal_phy_reset(priv);
	else
		gmac_hw_external_phy_reset(priv);
}

static int gmac_phy_init(struct device *dev, struct net_device *ndev,
			   struct gmac_netdev_local *priv, struct device_node *node, bool *fixed_link)
{
	int ret;

	/*
	 * phy reset, should be early than "of_mdiobus_register".
	 * becausue "of_mdiobus_register" will read PHY register by MDIO.
	 */
	gmac_hw_phy_reset(priv);

	gmac_of_get_param(priv, node);

	ret = of_get_phy_mode(node, &priv->phy_mode);
	if (ret < 0) {
		netdev_err(ndev, "not find phy-mode\n");
		return ret;
	}

	priv->phy_node = of_parse_phandle(node, "phy-handle", 0);
	if (priv->phy_node == NULL) {
		/* check if a fixed-link is defined in device-tree */
		if (of_phy_is_fixed_link(node)) {
			ret = of_phy_register_fixed_link(node);
			if (ret < 0) {
				dev_err(dev, "cannot register fixed PHY %d\n", ret);
				return ret;
			}

			/*
			 * In the case of a fixed PHY, the DT node associated
			 * to the PHY is the Ethernet MAC DT node.
			 */
			priv->phy_node = of_node_get(node);
			*fixed_link = true;
		} else {
			netdev_err(ndev, "not find phy-handle\n");
			ret = -EINVAL;
			return ret;
		}
	}
	return 0;
}

static void gmac_verify_flow_ctrl_args(void)
{
#if defined(CONFIG_TX_FLOW_CTRL_SUPPORT)
	flow_ctrl_en |= FLOW_TX;
#endif
#if defined(CONFIG_RX_FLOW_CTRL_SUPPORT)
	flow_ctrl_en |= FLOW_RX;
#endif
	if (tx_flow_ctrl_active_threshold < FC_ACTIVE_MIN ||
		tx_flow_ctrl_active_threshold > FC_ACTIVE_MAX)
		tx_flow_ctrl_active_threshold = FC_ACTIVE_DEFAULT;

	if (tx_flow_ctrl_deactive_threshold < FC_DEACTIVE_MIN ||
		tx_flow_ctrl_deactive_threshold > FC_DEACTIVE_MAX)
		tx_flow_ctrl_deactive_threshold = FC_DEACTIVE_DEFAULT;

	if (tx_flow_ctrl_active_threshold >= tx_flow_ctrl_deactive_threshold) {
		tx_flow_ctrl_active_threshold = FC_ACTIVE_DEFAULT;
		tx_flow_ctrl_deactive_threshold = FC_DEACTIVE_DEFAULT;
	}

	if (tx_flow_ctrl_pause_time < 0 ||
		tx_flow_ctrl_pause_time > FC_PAUSE_TIME_MAX)
		tx_flow_ctrl_pause_time = FC_PAUSE_TIME_DEFAULT;

	if (tx_flow_ctrl_pause_interval < 0 ||
		tx_flow_ctrl_pause_interval > FC_PAUSE_TIME_MAX)
		tx_flow_ctrl_pause_interval = FC_PAUSE_INTERVAL_DEFAULT;

	/*
	 * pause interval should not bigger than pause time,
	 * but should not too smaller to avoid sending too many pause frame.
	 */
	if ((tx_flow_ctrl_pause_interval > tx_flow_ctrl_pause_time) ||
		(tx_flow_ctrl_pause_interval < ((unsigned int)tx_flow_ctrl_pause_time >> 1)))
		tx_flow_ctrl_pause_interval = tx_flow_ctrl_pause_time;
}

static int gmac_dev_probe_device(struct platform_device *pdev,
				   struct net_device **p_ndev, struct gmac_netdev_local **p_priv)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct net_device *ndev = NULL;
	struct gmac_netdev_local *priv = NULL;
	int num_rxqs;

	gmac_verify_flow_ctrl_args();

	if (of_device_is_compatible(node, "vendor,gmac-v5"))
		num_rxqs = RSS_NUM_RXQS;
	else
		num_rxqs = 1;

	ndev = alloc_etherdev_mqs(sizeof(struct gmac_netdev_local), 1,
				  num_rxqs);
	if (ndev == NULL)
		return -ENOMEM;

	platform_set_drvdata(pdev, ndev);
	SET_NETDEV_DEV(ndev, dev);

	priv = netdev_priv(ndev);
	priv->dev = dev;
	priv->netdev = ndev;
	priv->num_rxqs = num_rxqs;

	*p_ndev = ndev;
	*p_priv = priv;
	return 0;
}

static int gmac_dev_probe_queue(struct platform_device *pdev,
				  struct gmac_netdev_local *priv, struct net_device *ndev, bool fixed_link)
{
	int ret;

	ret = gmac_dev_probe_init(pdev, priv, ndev);
	if (ret)
		goto _error_hw_desc_queue;

	if (priv->tso_supported) {
		ret = gmac_init_sg_desc_queue(priv);
		if (ret)
			goto _error_sg_desc_queue;
	}

	/* register netdevice */
	ret = register_netdev(priv->netdev);
	if (ret) {
		pr_err("register_ndev failed!");
		goto _error_sg_desc_queue;
	}

	/*
	 * reset queue here to make BQL only reset once.
	 * if we put netdev_reset_queue() in gmac_net_open(),
	 * the BQL will be reset when ifconfig eth0 down and up,
	 * but the tx ring is not cleared before.
	 * As a result, the NAPI poll will call netdev_completed_queue()
	 * and BQL throw a bug.
	 */
	netdev_reset_queue(ndev);

	/* config PHY power down to save power */
	phy_suspend(priv->phy);

	clk_disable_unprepare(priv->clk);
	if (priv->macif_clk != NULL)
		clk_disable_unprepare(priv->macif_clk);

	pr_info("ETH: %s, phy_addr=%d\n",
		phy_modes(priv->phy_mode), priv->phy->mdio.addr);

	return ret;

_error_sg_desc_queue:
	if (priv->tso_supported)
		gmac_destroy_sg_desc_queue(priv);
_error_hw_desc_queue:
	gmac_destroy_hw_desc_queue(priv);
	gmac_destroy_napi(priv);

	return ret;
}

static int gmac_dev_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct net_device *ndev = NULL;
	struct gmac_netdev_local *priv = NULL;
	int ret;
	bool fixed_link = false;

	ret = gmac_dev_probe_device(pdev, &ndev, &priv);
	if (ret)
		return ret;

	ret = gmac_dev_probe_res(pdev, priv);
	if (ret)
		goto out_free_netdev;

	ret = gmac_dev_macif_clk(pdev, priv, ndev);
	if (ret)
		goto out_clk_disable;

	gmac_mac_core_reset(priv);

	ret = gmac_phy_init(dev, ndev, priv, node, &fixed_link);
	if (ret)
		goto out_macif_clk_disable;

	gmac_set_mac_addr(ndev, node);
	gmac_set_hw_cap(pdev, priv);

	/* init hw controller */
	gmac_hw_init(priv);

	ret = gmac_dev_probe_phy(pdev, priv, ndev, fixed_link);
	if (ret) {
		if (priv->phy == NULL)
			goto out_phy_node;
		else
			goto out_phy_disconnect;
	}

	ret = gmac_dev_probe_queue(pdev, priv, ndev, fixed_link);
	if (ret)
		goto out_phy_disconnect;

	return ret;

out_phy_disconnect:
	phy_disconnect(priv->phy);
out_phy_node:
	of_node_put(priv->phy_node);
out_macif_clk_disable:
	if (priv->macif_clk != NULL)
		clk_disable_unprepare(priv->macif_clk);
out_clk_disable:
	clk_disable_unprepare(priv->clk);
out_free_netdev:
	free_netdev(ndev);

	return ret;
}

static int gmac_dev_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct gmac_netdev_local *priv = netdev_priv(ndev);

	/* stop the gmac and free all resource */
	del_timer_sync(&priv->monitor);
	gmac_destroy_napi(priv);

	unregister_netdev(ndev);

	gmac_reclaim_rx_tx_resource(priv);
	gmac_free_rx_skb(priv);
	gmac_free_tx_skb(priv);

	if (priv->tso_supported)
		gmac_destroy_sg_desc_queue(priv);
	gmac_destroy_hw_desc_queue(priv);

	phy_disconnect(priv->phy);
	of_node_put(priv->phy_node);

	free_netdev(ndev);

	gmac_phy_unregister_fixups();

	return 0;
}

#ifdef CONFIG_PM
static void gmac_disable_irq(struct gmac_netdev_local *priv)
{
	int i;

	for (i = 0; i < priv->num_rxqs; i++)
		disable_irq(priv->irq[i]);
}

static void gmac_enable_irq(struct gmac_netdev_local *priv)
{
	int i;

	for (i = 0; i < priv->num_rxqs; i++)
		enable_irq(priv->irq[i]);
}

/* board related func */
static void gmac_internal_phy_clk_disable(struct gmac_netdev_local const *priv)
{
}

/* board related func */
static void gmac_hw_all_clk_disable(struct gmac_netdev_local *priv)
{
	/*
	 * If macif clock is enabled when suspend, we should
	 * disable it here.
	 * Because when resume, PHY will link up again and
	 * macif clock will be enabled too. If we don't disable
	 * macif clock in suspend, macif clock will be enabled twice.
	 */
	if (priv == NULL || priv->clk == NULL || priv->netdev == NULL || priv->macif_clk == NULL)
		return;

	if (priv->netdev->flags & IFF_UP)
		clk_disable_unprepare(priv->macif_clk);

	/*
	 * This is called in suspend, when net device is down,
	 * MAC clk is disabled.
	 * So we need to judge whether MAC clk is enabled,
	 * otherwise kernel will WARNING if clk disable twice.
	 */
	if (priv->netdev->flags & IFF_UP)
		clk_disable_unprepare(priv->clk);

	if (priv->internal_phy)
		gmac_internal_phy_clk_disable(priv);
}

int gmac_dev_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct gmac_netdev_local *priv = netdev_priv(ndev);

	gmac_disable_irq(priv);
	/*
	 * If support Wake on LAN, we should not disconnect phy
	 * because it will call phy_suspend to power down phy.
	 */
	if (!priv->wol_enable)
		phy_disconnect(priv->phy);
	del_timer_sync(&priv->monitor);
	/*
	 * If suspend when netif is not up, the napi_disable will run into
	 * dead loop and dpm_drv_timeout will give warning.
	 */
	if (netif_running(ndev))
		gmac_disable_napi(priv);
	netif_device_detach(ndev);

	netif_carrier_off(ndev);

	/*
	 * If netdev is down, MAC clock is disabled.
	 * So if we want to reclaim MAC rx and tx resource,
	 * we must first enable MAC clock and then disable it.
	 */
	if (!(ndev->flags & IFF_UP))
		clk_prepare_enable(priv->clk);

	gmac_reclaim_rx_tx_resource(priv);

	if (!(ndev->flags & IFF_UP))
		clk_disable_unprepare(priv->clk);

	pmt_enter(priv);

	if (!priv->wol_enable) {
		/*
		 * if no WOL, then poweroff
		 * no need to call genphy_resume() in resume,
		 * because we reset everything
		 */
		genphy_suspend(priv->phy); /* power down phy */
		msleep(20); /* wait 20ms */
		gmac_hw_all_clk_disable(priv);
	}

	return 0;
}
EXPORT_SYMBOL(gmac_dev_suspend);

/* board related func */
static void gmac_internal_phy_clk_enable(struct gmac_netdev_local const *priv)
{
}

/* board related func */
static void gmac_hw_all_clk_enable(struct gmac_netdev_local *priv)
{
	if (priv == NULL || priv->netdev == NULL || priv->clk == NULL)
		return;

	if (priv->internal_phy)
		gmac_internal_phy_clk_enable(priv);

	if (priv->netdev->flags & IFF_UP)
		clk_prepare_enable(priv->macif_clk);

	/* If net device is down when suspend, we should not enable MAC clk. */
	if (priv->netdev->flags & IFF_UP)
		clk_prepare_enable(priv->clk);
}

int gmac_dev_resume(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct gmac_netdev_local *priv = netdev_priv(ndev);
	int ret;

	/*
	 * If we support Wake on LAN, we doesn't call clk_disable.
	 * But when we resume, the uboot may off mac clock and reset phy
	 * by re-write the mac CRG register.
	 * So we first call clk_disable, and then clk_enable.
	 */
	if (priv->wol_enable)
		gmac_hw_all_clk_disable(priv);

	gmac_hw_all_clk_enable(priv);
	/* internal FE_PHY: enable clk and reset  */
	gmac_hw_phy_reset(priv);

	/*
	 * If netdev is down, MAC clock is disabled.
	 * So if we want to restart MAC and re-initialize it,
	 * we must first enable MAC clock and then disable it.
	 */
	if (!(ndev->flags & IFF_UP))
		clk_prepare_enable(priv->clk);

	/* power on gmac */
	gmac_restart(priv);

	/*
	 * If support WoL, we didn't disconnect phy.
	 * But when we resume, we reset PHY, so we want to
	 * call phy_connect to make phy_fixup excuted.
	 * This is important for internal PHY fix.
	 */
	if (priv->wol_enable)
		phy_disconnect(priv->phy);

	ret = phy_connect_direct(ndev, priv->phy, gmac_adjust_link,
				 priv->phy_mode);
	if (ret)
		return ret;

	/*
	 * If we suspend and resume when net device is down,
	 * some operations are unnecessary.
	 */
	if (ndev->flags & IFF_UP) {
		priv->monitor.expires = jiffies + GMAC_MONITOR_TIMER;
		mod_timer(&priv->monitor, priv->monitor.expires);
		priv->old_link = 0;
		priv->old_speed = SPEED_UNKNOWN;
		priv->old_duplex = DUPLEX_UNKNOWN;
	}
	if (netif_running(ndev))
		gmac_enable_napi(priv);
	netif_device_attach(ndev);
	if (ndev->flags & IFF_UP)
		phy_start(priv->phy);
	gmac_enable_irq(priv);

	pmt_exit(priv);

	if (!(ndev->flags & IFF_UP))
		clk_disable_unprepare(priv->clk);

	return 0;
}
EXPORT_SYMBOL(gmac_dev_resume);
#endif

static const struct of_device_id gmac_of_match[] = {
	{ .compatible = "vendor,gmac", },
	{ .compatible = "vendor,gmac-v1", },
	{ .compatible = "vendor,gmac-v2", },
	{ .compatible = "vendor,gmac-v3", },
	{ .compatible = "vendor,gmac-v4", },
	{ .compatible = "vendor,gmac-v5", },
	{ },
};

MODULE_DEVICE_TABLE(of, gmac_of_match);

static struct platform_driver gmac_dev_driver = {
	.probe = gmac_dev_probe,
	.remove = gmac_dev_remove,
#ifdef CONFIG_PM
	.suspend = gmac_dev_suspend,
	.resume = gmac_dev_resume,
#endif
	.driver = {
		.owner = THIS_MODULE,
		.name = GMAC_DRIVER_NAME,
		.of_match_table = gmac_of_match,
	},
};

static int __init gmac_init(void)
{
	int ret;

	ret = platform_driver_register(&gmac_dev_driver);
	if (ret)
		return ret;

	gmac_proc_create();

	return 0;
}

static void __exit gmac_exit(void)
{
	platform_driver_unregister(&gmac_dev_driver);

	gmac_proc_destroy();
}

module_init(gmac_init);
module_exit(gmac_exit);

MODULE_AUTHOR("Vendor");
MODULE_DESCRIPTION("Vendor double GMAC driver, base on driver gmacv200");
MODULE_LICENSE("GPL v2");
