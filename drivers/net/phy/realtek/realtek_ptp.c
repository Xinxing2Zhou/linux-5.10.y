// SPDX-License-Identifier: (GPL-2.0)
/*
 * Driver for Realtek 8211FS(I)-VS PHYs - timestamping and PHC support
 *
 * Authors: Xinxing Zhou
 * License: GPL
 * Email: xn989695@dal.ca
 */

#include <linux/gpio/consumer.h>
#include <linux/ip.h>
#include <linux/net_tstamp.h>
#include <linux/mii.h>
#include <linux/phy.h>
#include <linux/ptp_classify.h>
#include <linux/ptp_clock_kernel.h>
#include <linux/udp.h>
#include <linux/gpio.h>
#include <asm/unaligned.h>
#include <linux/proc_fs.h>

#include "realtek.h"
#include "realtek_ptp.h"

irqreturn_t rtl8211f_handle_ts_interrupt(struct phy_device *phydev)
{
    return IRQ_HANDLED;
}

static struct rtl8211f_ptphdr *rtl8211f_get_ptp_header_l4(struct sk_buff *skb,
						struct iphdr *iphdr,
						struct udphdr *udphdr)
{
	if (iphdr->version != 4 || iphdr->protocol != IPPROTO_UDP)
		return NULL;

	return (struct rtl8211f_ptphdr *)(((unsigned char *)udphdr) + UDP_HLEN);
}

static struct rtl8211f_ptphdr *rtl8211f_get_ptp_header_tx(struct sk_buff *skb)
{
	struct ethhdr *ethhdr = eth_hdr(skb);
	struct udphdr *udphdr;
	struct iphdr *iphdr;

	if (ethhdr->h_proto == htons(ETH_P_1588))
		return (struct rtl8211f_ptphdr *)(((unsigned char *)ethhdr) +
						 skb_mac_header_len(skb));

	if (ethhdr->h_proto != htons(ETH_P_IP))
		return NULL;

	iphdr = ip_hdr(skb);
	udphdr = udp_hdr(skb);

	return rtl8211f_get_ptp_header_l4(skb, iphdr, udphdr);
}

static struct rtl8211f_ptphdr *rtl8211f_get_ptp_header_rx(struct sk_buff *skb,
						enum hwtstamp_rx_filters rx_filter)
{
	struct udphdr *udphdr;
	struct iphdr *iphdr;

	if (rx_filter == HWTSTAMP_FILTER_PTP_V2_L2_EVENT)
		return (struct rtl8211f_ptphdr *)skb->data;

	iphdr = (struct iphdr *)skb->data;
	udphdr = (struct udphdr *)(skb->data + iphdr->ihl * 4);

	return rtl8211f_get_ptp_header_l4(skb, iphdr, udphdr);
}

static int rtl8211f_get_signature(struct sk_buff *skb, u8 *sig)
{
	struct rtl8211f_ptphdr *ptphdr = rtl8211f_get_ptp_header_tx(skb);
	struct ethhdr *ethhdr = eth_hdr(skb);
	unsigned int i;

	if (!ptphdr)
		return -EOPNOTSUPP;

	sig[0] = (__force u16)ptphdr->seq_id >> 8;
	sig[1] = (__force u16)ptphdr->seq_id & GENMASK(7, 0);
	sig[2] = ptphdr->domain;
	sig[3] = ptphdr->tsmt & GENMASK(3, 0);

	memcpy(&sig[4], ethhdr->h_dest, ETH_ALEN);

	/* Fill the last bytes of the signature to reach a 16B signature */
	for (i = 10; i < 16; i++)
		sig[i] = ptphdr->tsmt & GENMASK(3, 0);

	return 0;
}

static void rtl8211f_adjust_mode_set(struct ptp_clock_info *info, u8 mode)
{
    struct rtl8211f_ptp *ptp = container_of(info, struct rtl8211f_ptp, caps);
	struct phy_device *phydev = ptp->phydev;
    
    phy_write_paged(phydev, RTL8211F_E41_PAGE, RTL8211F_PTP_CLK_CFG, mode << 1 | 0x1);
}

static int rtl8211f_adjtime(struct ptp_clock_info *info, s64 delta)
{
	struct rtl8211f_ptp *ptp = container_of(info, struct rtl8211f_ptp, caps);
	struct phy_device *phydev = ptp->phydev;
    u64 sec_diff, nsec_diff;
    int isneg = 0;

    if (delta < 0) {
        delta *= -1;
        isneg = 1;
    }

    sec_diff = delta / 1000000000;
    nsec_diff = delta - sec_diff * 1000000000;

    phy_write_paged(phydev, RTL8211F_E41_PAGE, RTL8211F_PTP_CFG_NS_LO, 
        nsec_diff & 0xffff);
    phy_write_paged(phydev, RTL8211F_E41_PAGE, RTL8211F_PTP_CFG_NS_HI, 
        (nsec_diff >> 16) & 0x3fff);

    phy_write_paged(phydev, RTL8211F_E41_PAGE, RTL8211F_PTP_CFG_S_LO, 
        sec_diff & 0xffff);
    phy_write_paged(phydev, RTL8211F_E41_PAGE, RTL8211F_PTP_CFG_S_MI, 
        (sec_diff >> 16) & 0xffff);
    phy_write_paged(phydev, RTL8211F_E41_PAGE, RTL8211F_PTP_CFG_S_HI, 
        (sec_diff >> 32) & 0xffff);
    
    if (isneg == 1)
        rtl8211f_adjust_mode_set(info, RTL8211F_DECREMENT_STEP);
    else
        rtl8211f_adjust_mode_set(info, RTL8211F_INCREMENT_STEP);

    return 0;
}

/**
 * Rate adjustment is replaced by Sync Ethernet.
 */
static int rtl8211f_adjfreq(struct ptp_clock_info *info, s32 delta)
{
    struct rtl8211f_ptp *ptp = container_of(info, struct rtl8211f_ptp, caps);
    struct phy_device *phydev = ptp->phydev;
    u16 val;

    val = (delta >> 16) & 0xffff;
    phy_write_paged(phydev, RTL8211F_E41_PAGE, RTL8211F_PTP_CFG_NS_HI, val);
    val = delta & 0xffff;
    phy_write_paged(phydev, RTL8211F_E41_PAGE, RTL8211F_PTP_CFG_NS_LO, val);

    rtl8211f_adjust_mode_set(info, RTL8211F_RATE_WRITE);

    return 0;
}

static int rtl8211f_get_rxtstamp(struct ptp_clock_info *info, u8 msg_type, struct timespec64 *ts)
{
    struct rtl8211f_ptp *ptp = container_of(info, struct rtl8211f_ptp, caps);
	struct phy_device *phydev = ptp->phydev;
    int val = 0;
    
    val = RTL8211F_TRXTS_OVERWR_EN | 
        msg_type << 2 | 
        RTL8211F_TRXTS_RX << 1 | 
        RTL8211F_TRXTS_RD;
    phy_write_paged(phydev, RTL8211F_E43_PAGE, RTL8211F_PTP_TRX_TS_STA, (u16)val);

    val = phy_read_paged(phydev, RTL8211F_E44_PAGE, RTL8211F_PTP_TRX_TS_S_HI);
    ts->tv_sec = val;
    ts->tv_sec <<= 16;
    val = phy_read_paged(phydev, RTL8211F_E44_PAGE, RTL8211F_PTP_TRX_TS_S_MI);
    ts->tv_sec |= val;
    ts->tv_sec <<= 16;
    ts->tv_sec |= phy_read_paged(phydev, RTL8211F_E44_PAGE, RTL8211F_PTP_TRX_TS_S_LO);

    val = phy_read_paged(phydev, RTL8211F_E44_PAGE, RTL8211F_PTP_TRX_TS_NS_HI);
    ts->tv_nsec = val & 0x3fff;
    ts->tv_nsec <<= 16;
    ts->tv_nsec |= phy_read_paged(phydev, RTL8211F_E44_PAGE, RTL8211F_PTP_TRX_TS_NS_LO);

    return 0;
}

static int rtl8211f_get_txtstamp(struct ptp_clock_info *info, u8 msg_type, struct timespec64 *ts)
{
    struct rtl8211f_ptp *ptp = container_of(info, struct rtl8211f_ptp, caps);
	struct phy_device *phydev = ptp->phydev;
    int val = 0;
    
    val = RTL8211F_TRXTS_OVERWR_EN | 
        msg_type << 2 | 
        RTL8211F_TRXTS_TX << 1 | 
        RTL8211F_TRXTS_RD;
    phy_write_paged(phydev, RTL8211F_E43_PAGE, RTL8211F_PTP_TRX_TS_STA, (u16)val);

    val = phy_read_paged(phydev, RTL8211F_E44_PAGE, RTL8211F_PTP_TRX_TS_S_HI);
    ts->tv_sec = val;
    ts->tv_sec <<= 16;
    val = phy_read_paged(phydev, RTL8211F_E44_PAGE, RTL8211F_PTP_TRX_TS_S_MI);
    ts->tv_sec |= val;
    ts->tv_sec <<= 16;
    ts->tv_sec |= phy_read_paged(phydev, RTL8211F_E44_PAGE, RTL8211F_PTP_TRX_TS_S_LO);

    val = phy_read_paged(phydev, RTL8211F_E44_PAGE, RTL8211F_PTP_TRX_TS_NS_HI);
    ts->tv_nsec = val & 0x3fff;
    ts->tv_nsec <<= 16;
    ts->tv_nsec |= phy_read_paged(phydev, RTL8211F_E44_PAGE, RTL8211F_PTP_TRX_TS_NS_LO);

    return 0;
}

static int __rtl8211f_gettime(struct ptp_clock_info *info, struct timespec64 *ts)
{
    struct rtl8211f_ptp *ptp = container_of(info, struct rtl8211f_ptp, caps);
	struct phy_device *phydev = ptp->phydev;
    int val = 0;
    
    rtl8211f_adjust_mode_set(info, RTL8211F_DIRECT_READ); 

    val = phy_read_paged(phydev, RTL8211F_E41_PAGE, RTL8211F_PTP_CFG_S_HI);
    ts->tv_sec = val;
    ts->tv_sec <<= 16;
    val = phy_read_paged(phydev, RTL8211F_E41_PAGE, RTL8211F_PTP_CFG_S_MI);
    ts->tv_sec |= val;
    ts->tv_sec <<= 16;
    val = phy_read_paged(phydev, RTL8211F_E41_PAGE, RTL8211F_PTP_CFG_S_LO);
    ts->tv_sec |= val;

    val = phy_read_paged(phydev, RTL8211F_E41_PAGE, RTL8211F_PTP_CFG_NS_HI);
    ts->tv_nsec = val;
    ts->tv_nsec <<= 16;
    val = phy_read_paged(phydev, RTL8211F_E41_PAGE, RTL8211F_PTP_CFG_NS_LO);
    ts->tv_nsec |= val;

    return 0;
}

int rtl8211f_gettime(struct ptp_clock_info *info, struct timespec64 *ts)
{
	__rtl8211f_gettime(info, ts);

	return 0;
}

static int __rtl8211f_settime(struct ptp_clock_info *info, const struct timespec64 *ts)
{
    struct rtl8211f_ptp *ptp = container_of(info, struct rtl8211f_ptp, caps);
	struct phy_device *phydev = ptp->phydev;

    phy_write_paged(phydev, RTL8211F_E41_PAGE, RTL8211F_PTP_CFG_S_HI, 
        ts->tv_sec >> 32 & 0xffff);
    phy_write_paged(phydev, RTL8211F_E41_PAGE, RTL8211F_PTP_CFG_S_MI, 
        ts->tv_sec >> 16 & 0xffff);
    phy_write_paged(phydev, RTL8211F_E41_PAGE, RTL8211F_PTP_CFG_S_LO, 
        ts->tv_sec & 0xffff);

    phy_write_paged(phydev, RTL8211F_E41_PAGE, RTL8211F_PTP_CFG_NS_HI, 
        ts->tv_nsec >> 16 & 0xffff);
    phy_write_paged(phydev, RTL8211F_E41_PAGE, RTL8211F_PTP_CFG_NS_LO, 
        ts->tv_nsec & 0xffff);

    rtl8211f_adjust_mode_set(info, RTL8211F_DIRECT_WRITE);

    return 0;
}

int rtl8211f_settime(struct ptp_clock_info *info, const struct timespec64 *ts)
{
	__rtl8211f_settime(info, ts);

	return 0;
}

static int rtl8211f_hwtstamp(struct mii_timestamper *mii_ts, struct ifreq *ifr)
{
	struct rtl8211f_private *rtl8211f =
		container_of(mii_ts, struct rtl8211f_private, mii_ts);
	struct hwtstamp_config cfg;
	bool one_step = false;

	if (copy_from_user(&cfg, ifr->ifr_data, sizeof(cfg)))
		return -EFAULT;

	if (cfg.flags)
		return -EINVAL;

	switch (cfg.tx_type) {
	case HWTSTAMP_TX_ONESTEP_SYNC:
		one_step = true;
		break;
	case HWTSTAMP_TX_ON:
		break;
	case HWTSTAMP_TX_OFF:
		break;
	default:
		return -ERANGE;
	}

	rtl8211f->ptp->tx_type = cfg.tx_type;

	switch (cfg.rx_filter) {
	case HWTSTAMP_FILTER_NONE:
		break;
	case HWTSTAMP_FILTER_PTP_V2_L4_EVENT:
		/* ETH->IP->UDP->PTP */
		break;
	case HWTSTAMP_FILTER_PTP_V2_L2_EVENT:
		/* ETH->PTP */
		break;
	default:
		return -ERANGE;
	}

	rtl8211f->ptp->rx_filter = cfg.rx_filter;

	spin_lock_irq(&rtl8211f->ptp->tx_queue_lock);
    
	__skb_queue_purge(&rtl8211f->ptp->tx_queue);
	__skb_queue_head_init(&rtl8211f->ptp->tx_queue);
    
    rtl8211f->ptp->configured = 1;
    
	spin_unlock_irq(&rtl8211f->ptp->tx_queue_lock);

	return copy_to_user(ifr->ifr_data, &cfg, sizeof(cfg)) ? -EFAULT : 0;
}

static int rtl8211f_ts_info(struct mii_timestamper *mii_ts,
			   struct ethtool_ts_info *info)
{
	struct rtl8211f_private *rtl8211f =
		container_of(mii_ts, struct rtl8211f_private, mii_ts);

	info->phc_index = ptp_clock_index(rtl8211f->ptp->ptp_clock);
	info->so_timestamping =
        SOF_TIMESTAMPING_TX_HARDWARE |
        SOF_TIMESTAMPING_RX_HARDWARE |
        SOF_TIMESTAMPING_RAW_HARDWARE;
	info->tx_types =
		(1 << HWTSTAMP_TX_OFF) |
		(1 << HWTSTAMP_TX_ON) |
		(1 << HWTSTAMP_TX_ONESTEP_SYNC);
	info->rx_filters =
		(1 << HWTSTAMP_FILTER_NONE) |
		(1 << HWTSTAMP_FILTER_PTP_V2_L2_EVENT) |
		(1 << HWTSTAMP_FILTER_PTP_V2_L4_EVENT);

	return 0;
}

static void rtl8211f_txtstamp(struct mii_timestamper *mii_ts,
                struct sk_buff *skb, int type)
{
    struct rtl8211f_private *rtl8211f =
        container_of(mii_ts, struct rtl8211f_private, mii_ts);

    if (!rtl8211f->ptp->configured)
        return;

    if (rtl8211f->ptp->tx_type == HWTSTAMP_TX_OFF) {
        kfree_skb(skb);
        return;
    }

    skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;

    spin_lock_irq(&rtl8211f->ptp->tx_queue_lock);
    __skb_queue_tail(&rtl8211f->ptp->tx_queue, skb);
    spin_unlock_irq(&rtl8211f->ptp->tx_queue_lock);
}

static bool rtl8211f_rxtstamp(struct mii_timestamper *mii_ts,
                struct sk_buff *skb, int type)
{
    struct rtl8211f_ptphdr *ptphdr;
    struct rtl8211f_private *rtl8211f =
        container_of(mii_ts, struct rtl8211f_private, mii_ts);

    if (skb->protocol != htons(ETH_P_IP))
        return false;

    if (!rtl8211f->ptp->configured)
		return false;

	if (rtl8211f->ptp->rx_filter == HWTSTAMP_FILTER_NONE ||
	    type == PTP_CLASS_NONE)
		return false;

    ptphdr = rtl8211f_get_ptp_header_rx(skb, rtl8211f->ptp->rx_filter);
	if (!ptphdr)
		return false;

    if (ptphdr->tsmt == DELAY_REQ)
        return false;

    spin_lock_irq(&rtl8211f->ptp->rx_queue_lock);
    __skb_queue_tail(&rtl8211f->ptp->rx_queue, skb);
    spin_unlock_irq(&rtl8211f->ptp->rx_queue_lock);

    return true;
}

static const struct ptp_clock_info rtl8211f_clk_caps = {
	.owner		= THIS_MODULE,
	.name		= "RTL8211F timer",
	.max_adj	= S32_MAX,
	.n_alarm	= 0,
	.n_pins		= 0,
	.n_ext_ts	= 0,
	.n_per_out	= 0,
	.pps		= 0,
	.adjtime    = &rtl8211f_adjtime,
	.adjfine	= NULL,
	.adjfreq    = &rtl8211f_adjfreq,
	.gettime64	= &rtl8211f_gettime,
	.settime64	= &rtl8211f_settime,
};

static struct ptp_clock_info *rtl8211f_clk_info = NULL;

static void rtl8211f_ptp_phy_reset(struct phy_device *phydev)
{
    phy_write_paged(phydev, 0x0, 0x0, 0x9040);
}

static void rtl8211f_ptp_capability_enable(struct phy_device *phydev)
{
    phy_write_paged(phydev, RTL8211F_E40_PAGE, RTL8211F_PTP_CTL, 
        RTL8211F_DR_2STEP_INS | RTL8211F_FU_2STEP_INS | RTL8211F_SYNC_1STEP | 
        RTL8211F_PTPV2_UDPIPV4 | RTL8211F_PTP_ENABLE);
}

static void rtl8211f_ptp_capability_disable(struct phy_device *phydev)
{
    phy_write_paged(phydev, RTL8211F_E40_PAGE, RTL8211F_PTP_CTL, 0x0);
}

/**
 * Enable sync ethernet function
 *
 * After phy link-up, the value of (page 0xa43, 0x1a) should be 0x302e,
 * and the value of (page 0xe40, 0x13) should be 0x1.
 */
static void rtl8211f_sync_ethernet(struct phy_device *phydev)
{
    int val = 0;
    
    phy_write(phydev, 0x09, 0x1200);
    
    val = phy_read_paged(phydev, RTL8211F_E40_PAGE, RTL8211F_SYNCE_CTL);
    val |= RTL8211F_SYNC_E_ENABLE;
    phy_write_paged(phydev, RTL8211F_E40_PAGE, RTL8211F_SYNCE_CTL, val);
}

static void rtl8211f_interupt_enable(struct phy_device *phydev)
{
    phy_write_paged(phydev, RTL8211F_E40_PAGE, RTL8211F_PTP_INER, 
        RTL8211F_TX_TIMESTAMP | RTL8211F_RX_TIMESTAMP);
}

int rtl8211f_ptp_init(struct phy_device *phydev)
{
    struct rtl8211f_private *rtl8211f = phydev->priv;

    rtl8211f_sync_ethernet(phydev);
    
    rtl8211f_ptp_capability_enable(phydev);

    rtl8211f_ptp_phy_reset(phydev);
    
    rtl8211f_interupt_enable(phydev);
    
    rtl8211f->mii_ts.rxtstamp = rtl8211f_rxtstamp;
	rtl8211f->mii_ts.txtstamp = rtl8211f_txtstamp;
	rtl8211f->mii_ts.hwtstamp = rtl8211f_hwtstamp;
	rtl8211f->mii_ts.ts_info  = rtl8211f_ts_info;
	phydev->mii_ts = &rtl8211f->mii_ts;

	memcpy(&rtl8211f->ptp->caps, &rtl8211f_clk_caps, sizeof(rtl8211f_clk_caps));

    skb_queue_head_init(&rtl8211f->ptp->tx_queue);
    spin_lock_init(&rtl8211f->ptp->tx_queue_lock);
    skb_queue_head_init(&rtl8211f->ptp->rx_queue);
    spin_lock_init(&rtl8211f->ptp->rx_queue_lock);

    mutex_init(&rtl8211f->ptp->ts_mutex);

	rtl8211f->ptp->ptp_clock = ptp_clock_register(&rtl8211f->ptp->caps,
						     &phydev->mdio.dev);
    
	return PTR_ERR_OR_ZERO(rtl8211f->ptp->ptp_clock);
}

void rtl8211f_ptp_exit(struct phy_device *phydev)
{
    struct rtl8211f_private *rtl8211f = phydev->priv;

    ptp_clock_unregister(rtl8211f->ptp->ptp_clock);

    rtl8211f_ptp_capability_disable(phydev);
}

int rtl8211f_ptp_probe(struct phy_device *phydev)
{
	struct rtl8211f_private *rtl8211f;
    struct device *dev = &phydev->mdio.dev;
    int val;

    rtl8211f = devm_kzalloc(&phydev->mdio.dev, sizeof(*rtl8211f), GFP_KERNEL);
	if (!rtl8211f)
		return -ENOMEM;

	rtl8211f->ptp = devm_kzalloc(&phydev->mdio.dev, sizeof(*rtl8211f->ptp),
				    GFP_KERNEL);
	if (!rtl8211f->ptp) {
        dev_err(dev, "ptp alloc failed\n");
		return -ENOMEM;
	}

    rtl8211f->ptp->phydev = phydev;
    rtl8211f_clk_info = &rtl8211f->ptp->caps;

    rtl8211f->ptp->gpio_base = ioremap(GPIO7_BASE_PHYADDR, 0x1000);
    if (NULL == rtl8211f->ptp->gpio_base) {
        dev_err(dev, "ioremap failed\n");
        return -ENOMEM;
    }

    /* gpio7_2 input direction */
    val = readl(rtl8211f->ptp->gpio_base + 0x400);
    val &= ~(1 << 2);
    writel(val, rtl8211f->ptp->gpio_base + 0x400);

    rtl8211f->ptp->gpio_level = readl(rtl8211f->ptp->gpio_base + (4 << 2));
    if (rtl8211f->ptp->gpio_level == 1 << 2) {
    	phydev->priv = rtl8211f;

        rtl8211f_ptp_init(phydev);

        dev_info(&phydev->mdio.dev, "RTL8211FS(1)-VS PTP is supported\n");
    }

	return 0;
}

static int rtl8211f_rx_queue_handle(struct rtl8211f_ptp *ptp, u8 message_type)
{
    struct phy_device *phydev = ptp->phydev;
    struct device *dev = &phydev->mdio.dev;
    struct skb_shared_hwtstamps *shhwtstamps = NULL;
    struct sk_buff *skb, *last;
    struct timespec64 ts;
    struct rtl8211f_ptphdr *ptphdr;
    int found = 0;

    spin_lock_irq(&ptp->rx_queue_lock);

    if (skb_queue_empty(&ptp->rx_queue)) {
        spin_unlock_irq(&ptp->rx_queue_lock);
        // dev_err(dev, "rx queue is empty\n");
        return -EINVAL;
    }

    skb = skb_peek(&ptp->rx_queue);
        
    do {
        ptphdr = rtl8211f_get_ptp_header_rx(skb, ptp->rx_filter);
    	if (!ptphdr) {
            spin_unlock_irq(&ptp->rx_queue_lock);
            dev_err(dev, "rx invalid ptp header\n");
    		return -EINVAL;
    	}

        if (ptphdr->tsmt == message_type) {
            __skb_unlink(skb, &ptp->rx_queue);

            found = 1;

            break;
        }

        last = skb;
        skb = skb->next;
    } while (!skb_queue_is_last(&ptp->rx_queue, last));

    spin_unlock_irq(&ptp->rx_queue_lock);

    if (found) {
        rtl8211f_get_rxtstamp(&ptp->caps, ptphdr->tsmt, &ts);

    	shhwtstamps = skb_hwtstamps(skb);
    	memset(shhwtstamps, 0, sizeof(struct skb_shared_hwtstamps));

    	shhwtstamps->hwtstamp = ktime_set(ts.tv_sec, ts.tv_nsec);
        netif_rx_ni(skb);
    }
    
    return 0;
}

static int rtl8211f_tx_queue_handle(struct rtl8211f_ptp *ptp, u8 message_type)
{
    struct phy_device *phydev = ptp->phydev;
    struct device *dev = &phydev->mdio.dev;
    struct skb_shared_hwtstamps ssh;
    struct sk_buff *skb, *last;
    struct timespec64 ts;
    struct rtl8211f_ptphdr *ptphdr;
    u8 skb_sig[16];
    int found = 0;

    spin_lock_irq(&ptp->tx_queue_lock);

    if (skb_queue_empty(&ptp->tx_queue)) {
        spin_unlock_irq(&ptp->tx_queue_lock);
        dev_err(dev, "tx queue is empty\n");
        return -EINVAL;
    }
    
    skb = skb_peek(&ptp->tx_queue);
        
    do {
        ptphdr = rtl8211f_get_ptp_header_tx(skb);
    	if (!ptphdr) {
            spin_unlock_irq(&ptp->tx_queue_lock);
            dev_err(dev, "tx invalid ptp header\n");
    		return -EINVAL;
    	}

        if (ptphdr->tsmt == message_type) {
            __skb_unlink(skb, &ptp->tx_queue);

            found = 1;

            break;
        }

        last = skb;
        skb = skb->next;
    } while (!skb_queue_is_last(&ptp->tx_queue, last));

    spin_unlock_irq(&ptp->tx_queue_lock);

    if (found) {
        if (rtl8211f_get_signature(skb, skb_sig) < 0) {
            dev_err(dev, "invalid signature\n");
			kfree_skb(skb);
			return -EINVAL;
		}

        rtl8211f_get_txtstamp(&ptp->caps, ptphdr->tsmt, &ts);

        ssh.hwtstamp = ktime_set(ts.tv_sec, ts.tv_nsec);

        skb_complete_tx_timestamp(skb, &ssh);
    }
    
    return 0;
}

static int rtl8211f_gpio_event_thread(void *arg)
{
    struct rtl8211f_ptp *ptp = container_of(arg, struct rtl8211f_ptp, caps);
    struct phy_device *phydev = ptp->phydev;
    struct device *dev = &phydev->mdio.dev;
    struct timespec64 ts;
    int work_done;
    u32 val;
    int ret = 0;

    while (!kthread_should_stop()) {
        ret = wait_for_completion_timeout(&ptp->complete, msecs_to_jiffies(5000));
        if (ret == 0) {
            dev_err(dev, "gpio event timeout after 5s\n");
            continue;
        }

        work_done = 0;

        do {
            /* clean the interrupt status */
            phy_read_paged(phydev, RTL8211F_E40_PAGE, RTL8211F_PTP_INSR);
            
            val = phy_read_paged(phydev, RTL8211F_E43_PAGE, RTL8211F_PTP_TRX_TS_STA);

            if (ptp->configured) {
                if (val & RTL8211F_RXTS_SYNC_RDY)
                    rtl8211f_rx_queue_handle(ptp, SYNC);

                if (val & RTL8211F_RXTS_DELAY_REQ_RDY)
                    rtl8211f_rx_queue_handle(ptp, DELAY_REQ);

                if (val & RTL8211F_TXTS_SYNC_RDY)
                    rtl8211f_tx_queue_handle(ptp, SYNC);

                if (val & RTL8211F_TXTS_DELAY_REQ_RDY)
                    rtl8211f_tx_queue_handle(ptp, DELAY_REQ);
            } else {
                if (val & RTL8211F_RXTS_SYNC_RDY)
                    rtl8211f_get_rxtstamp(&ptp->caps, SYNC, &ts);

                if (val & RTL8211F_RXTS_DELAY_REQ_RDY)
                    rtl8211f_get_rxtstamp(&ptp->caps, DELAY_REQ, &ts);

                if (val & RTL8211F_TXTS_SYNC_RDY)
                    rtl8211f_get_txtstamp(&ptp->caps, SYNC, &ts);

                if (val & RTL8211F_TXTS_DELAY_REQ_RDY)
                    rtl8211f_get_txtstamp(&ptp->caps, DELAY_REQ, &ts);
            }

            work_done++;
        } while (work_done < 32);
    }

    return 0;
}

static irqreturn_t rtl8211f_ptp_irq_handler(int irq, void *dev_id)
{
    struct rtl8211f_ptp *ptp = container_of(dev_id, struct rtl8211f_ptp, caps);

    complete(&ptp->complete);

    writel(0x04, ptp->gpio_base + 0x41C);
    
    return IRQ_NONE;
}

static int rtl8211f_proc_show(struct seq_file *m, void *v)
{
    struct rtl8211f_ptp *ptp = container_of(rtl8211f_clk_info, struct rtl8211f_ptp, caps);
	struct phy_device *phydev = ptp->phydev;

    rtl8211f_ptp_exit(phydev);

    return rtl8211f_ptp_init(phydev);;
}

static int rtl8211f_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, rtl8211f_proc_show, PDE_DATA(inode));
}

static const struct proc_ops rtl8211f_ptp_ops = {
    .proc_open      = rtl8211f_proc_open,
    .proc_read      = seq_read,
    .proc_lseek     = seq_lseek,
    .proc_release   = single_release,
};

static int __init rtl8211f_gpio_interrupt_init(void)
{
    struct rtl8211f_ptp *ptp = container_of(rtl8211f_clk_info, struct rtl8211f_ptp, caps);
	struct phy_device *phydev = ptp->phydev;
    struct device *dev = &phydev->mdio.dev;
    struct proc_dir_entry *p;
    char gname[16];
    int ret = 0;
    u32 val;
    
    /* ptp support if gpio7_2 is high */
    if (ptp->gpio_level == 0)
        return 0;
    
    /* falling edge */
    /* GPIO_ISR 0x404, set triger type, level or edge */
    val = readl(ptp->gpio_base + 0x404) & 0xfb;
    writel(val, ptp->gpio_base + 0x404);

    /* GPIO_IVE 0x40C, set up level low level rising edge or falling edge */
    val = readl(ptp->gpio_base + 0x40C) & 0xfb;
    writel(val, ptp->gpio_base + 0x40C);
 
    /* GPIO_IBE 0x408 */
    val = readl(ptp->gpio_base + 0x408) & 0xfb;
    writel(val, ptp->gpio_base + 0x408);

    /* GPIO_IC 0x41C */
    writel(0xff, ptp->gpio_base + 0x41C);

    /* GPIO_IE 0x410 */
    val = readl(ptp->gpio_base + 0x410) | 0x04;
    writel(val, ptp->gpio_base + 0x410);

    /* GPIO7_2 */
    ptp->raw_pin = BANK2PIN(7, 2);
    sprintf(gname, "gpio%d", ptp->raw_pin);

    ptp->irq = gpio_to_irq(ptp->raw_pin);
    if (ptp->irq < 0) {
        dev_err(dev, "gpio to irq failed\n");
        return -EINVAL;
    }

    init_completion(&ptp->complete);

    ptp->gpio_event_thread = kthread_run(rtl8211f_gpio_event_thread, 
        (void *)&ptp->caps, "event_gpio");
    if (IS_ERR(ptp->gpio_event_thread)) {
        dev_err(dev, "gpio event thread failed\n");
        return -EINVAL;
    }
 
    ret = request_irq(ptp->irq, rtl8211f_ptp_irq_handler, 
        IRQF_SHARED | IRQF_TRIGGER_FALLING, "rtl8211f.ptp", 
        (void *)rtl8211f_clk_info);
    if (ret != 0) {
        dev_err(dev, "request ptp irq failed, %d\n", ret);
        return -EINVAL;
    }

    p = proc_create_data("hardware_ptp", S_IRUGO, NULL, &rtl8211f_ptp_ops, NULL);
    if (NULL == p) {
        dev_err(dev, "realtek ptp creat failed, %d\n", ret);
        return -EINVAL;
    }

    return 0;
}

static void __exit rtl8211f_gpio_interrupt_exit(void)
{
    struct rtl8211f_ptp *ptp = container_of(rtl8211f_clk_info, 
        struct rtl8211f_ptp, caps);

    free_irq(ptp->irq, (void *)rtl8211f_clk_info);

    if (ptp->gpio_event_thread) {
        kthread_stop(ptp->gpio_event_thread);
    }

    gpio_free(ptp->raw_pin);
}

late_initcall(rtl8211f_gpio_interrupt_init);
module_exit(rtl8211f_gpio_interrupt_exit);
