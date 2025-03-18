/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#include <net/ipv6.h>
#include <linux/clk.h>
#include <linux/etherdevice.h>
#include <linux/if_vlan.h>
#include <linux/ip.h>

#include "gmac_pm.h"
#include "gmac_proc.h"
#include "gmac_netdev_ops.h"

int gmac_net_open(struct net_device *dev)
{
	struct gmac_netdev_local *ld = netdev_priv(dev);
	unsigned long flags;

	clk_prepare_enable(ld->macif_clk);
	clk_prepare_enable(ld->clk);

	/*
	 * If we configure mac address by
	 * "ifconfig ethX hw ether XX:XX:XX:XX:XX:XX",
	 * the ethX must be down state and mac core clock is disabled
	 * which results the mac address has not been configured
	 * in mac core register.
	 * So we must set mac address again here,
	 * because mac core clock is enabled at this time
	 * and we can configure mac address to mac core register.
	 */
	gmac_hw_set_mac_addr(dev);

	/*
	 * We should use netif_carrier_off() here,
	 * because the default state should be off.
	 * And this call should before phy_start().
	 */
	netif_carrier_off(dev);
	gmac_enable_napi(ld);
	phy_start(ld->phy);

	gmac_hw_desc_enable(ld);
	gmac_port_enable(ld);
	gmac_irq_enable_all_queue(ld);

	spin_lock_irqsave(&ld->rxlock, flags);
	gmac_rx_refill(ld);
	spin_unlock_irqrestore(&ld->rxlock, flags);

	ld->monitor.expires = jiffies + GMAC_MONITOR_TIMER;
	mod_timer(&ld->monitor, ld->monitor.expires);

	netif_start_queue(dev);

	return 0;
}

int gmac_net_close(struct net_device *dev)
{
	struct gmac_netdev_local *ld = netdev_priv(dev);

	gmac_irq_disable_all_queue(ld);
	gmac_hw_desc_disable(ld);

	gmac_disable_napi(ld);

	netif_carrier_off(dev);
	netif_stop_queue(dev);

	phy_stop(ld->phy);
	del_timer_sync(&ld->monitor);

	clk_disable_unprepare(ld->clk);
	clk_disable_unprepare(ld->macif_clk);

	return 0;
}

static int gmac_check_skb_len(struct sk_buff *skb, struct net_device *dev)
{
	if (skb->len < ETH_HLEN) {
		dev_kfree_skb_any(skb);
		dev->stats.tx_errors++;
		dev->stats.tx_dropped++;
		return -1;
	}
	return 0;
}

static int gmac_net_xmit_normal(struct sk_buff *skb, struct net_device *dev,
				  struct gmac_desc *desc, u32 pos)
{
	struct gmac_netdev_local *ld = netdev_priv(dev);
	dma_addr_t addr;

	addr = dma_map_single(ld->dev, skb->data, skb->len, DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(ld->dev, addr))) {
		dev_kfree_skb_any(skb);
		dev->stats.tx_dropped++;
		ld->tx_skb[pos] = NULL;
		ld->TX_BQ.skb[pos] = NULL;
		return -1;
	}
	desc->data_buff_addr = (u32)addr;
#if defined(CONFIG_GMAC_DDR_64BIT )
	desc->rxhash = (addr >> REG_BIT_WIDTH) & TX_DESC_HI8_MASK;
#endif
	desc->buffer_len = ETH_MAX_FRAME_SIZE - 1;
	desc->data_len = skb->len;
	desc->fl = DESC_FL_FULL;
	desc->descvid = DESC_VLD_BUSY;

	return 0;
}

static int gmac_tx_avail(struct gmac_netdev_local const *ld)
{
	unsigned int tx_bq_wr_offset, tx_bq_rd_offset;

	if (ld == NULL)
		return -1;

	tx_bq_wr_offset = readl(ld->gmac_iobase + TX_BQ_WR_ADDR);
	tx_bq_rd_offset = readl(ld->gmac_iobase + TX_BQ_RD_ADDR);

	return (int)((tx_bq_rd_offset >> DESC_BYTE_SHIFT) + TX_DESC_NUM
		- (tx_bq_wr_offset >> DESC_BYTE_SHIFT) - 1);
}

static netdev_tx_t gmac_sw_gso(struct gmac_netdev_local *ld,
				 struct sk_buff *skb)
{
	struct sk_buff *segs = NULL;
	struct sk_buff *curr_skb = NULL;
	int ret;
	int gso_segs = skb_shinfo(skb)->gso_segs;
	if (gso_segs == 0 && skb_shinfo(skb)->gso_size != 0)
		gso_segs = DIV_ROUND_UP(skb->len, skb_shinfo(skb)->gso_size);

	/* Estimate the number of fragments in the worst case */
	if (unlikely(gmac_tx_avail(ld) < gso_segs)) {
		netif_stop_queue(ld->netdev);
		if (gmac_tx_avail(ld) < gso_segs) {
			ld->netdev->stats.tx_dropped++;
			ld->netdev->stats.tx_fifo_errors++;
			return NETDEV_TX_BUSY;
		}
		netif_wake_queue(ld->netdev);
	}

	segs = skb_gso_segment(skb, ld->netdev->features & ~(NETIF_F_CSUM_MASK |
			       NETIF_F_SG | NETIF_F_GSO_SOFTWARE));
	if (IS_ERR_OR_NULL(segs))
		goto drop;

	do {
		curr_skb = segs;
		segs = segs->next;
		curr_skb->next = NULL;
		ret = gmac_net_xmit(curr_skb, ld->netdev);
		if (unlikely(ret != NETDEV_TX_OK))
			pr_err_once("gmac_net_xmit error ret=%d\n", ret);
	} while (segs != NULL);

	dev_kfree_skb_any(skb);
	return NETDEV_TX_OK;

drop:
	dev_kfree_skb_any(skb);
	ld->netdev->stats.tx_dropped++;
	return NETDEV_TX_OK;
}

static int gmac_xmit_gso_sg_frag(struct gmac_netdev_local *ld,
				   struct sk_buff *skb, struct sg_desc *desc_cur,
				   struct gmac_tso_desc *tx_bq_desc, unsigned int desc_pos)
{
	int nfrags = skb_shinfo(skb)->nr_frags;
	dma_addr_t addr;
	dma_addr_t dma_addr;
	int i, ret, len;

	for (i = 0; i < nfrags; i++) {
		skb_frag_t *frag = &skb_shinfo(skb)->frags[i];
		len = frag->bv_len;

		dma_addr = skb_frag_dma_map(ld->dev, frag, 0, len, DMA_TO_DEVICE);
		ret = dma_mapping_error(ld->dev, dma_addr);
		if (unlikely(ret)) {
			pr_err("skb frag DMA Mapping fail");
			return -EFAULT;
		}
		desc_cur->frags[i].addr = (u32)dma_addr;
#if defined(CONFIG_GMAC_DDR_64BIT )
		desc_cur->frags[i].reserved = (dma_addr >> REG_BIT_WIDTH) << SG_DESC_HI8_OFFSET;
#endif
		desc_cur->frags[i].size = len;
	}

	addr = ld->dma_sg_phy + ld->sg_head * sizeof(struct sg_desc);
	tx_bq_desc->data_buff_addr = (u32)addr;
#if defined(CONFIG_GMAC_DDR_64BIT )
	tx_bq_desc->reserve_desc2 = (addr >> REG_BIT_WIDTH) &
				    TX_DESC_HI8_MASK;
#endif
	ld->TX_BQ.sg_desc_offset[desc_pos] = ld->sg_head;

	ld->sg_head = (ld->sg_head + 1) % ld->sg_count;

	return 0;
}

static int gmac_xmit_gso_sg(struct gmac_netdev_local *ld,
			      struct sk_buff *skb,
			      struct gmac_tso_desc *tx_bq_desc, unsigned int desc_pos)
{
	struct sg_desc *desc_cur = NULL;
	dma_addr_t dma_addr;
	int ret;

	if (unlikely(((ld->sg_head + 1) % ld->sg_count) == ld->sg_tail)) {
		/* SG pkt, but sg desc all used */
		pr_err("WARNING: sg desc all used.\n");
		return -EBUSY;
	}

	desc_cur = ld->dma_sg_desc + ld->sg_head;

	desc_cur->total_len = skb->len;
	desc_cur->linear_len = skb_headlen(skb);
	dma_addr = dma_map_single(ld->dev, skb->data, desc_cur->linear_len, DMA_TO_DEVICE);
	ret = dma_mapping_error(ld->dev, dma_addr);
	if (unlikely(ret)) {
		pr_err("DMA Mapping fail");
		return -EFAULT;
	}
	desc_cur->linear_addr = (u32)dma_addr;
#if defined(CONFIG_GMAC_DDR_64BIT )
	desc_cur->reserv3 = (dma_addr >> REG_BIT_WIDTH) << SG_DESC_HI8_OFFSET;
#endif
	ret = gmac_xmit_gso_sg_frag(ld, skb, desc_cur, tx_bq_desc, desc_pos);
	if (unlikely(ret))
		return ret;

	return 0;
}

static int gmac_get_pkt_info(struct gmac_netdev_local *ld,
			struct sk_buff *skb, struct gmac_tso_desc *tx_bq_desc);

static int gmac_check_hw_capability(struct sk_buff *skb);

static int gmac_xmit_gso(struct gmac_netdev_local *ld, struct sk_buff *skb,
			   struct gmac_tso_desc *tx_bq_desc, unsigned int desc_pos)
{
	int pkt_type = PKT_NORMAL;
	int nfrags = skb_shinfo(skb)->nr_frags;
	dma_addr_t addr;
	int ret;

	if (skb_is_gso(skb) || nfrags)
		pkt_type = PKT_SG; /* TSO pkt or SG pkt */

	ret = gmac_check_hw_capability(skb);
	if (unlikely(ret))
		return ret;

	ret = gmac_get_pkt_info(ld, skb, tx_bq_desc);
	if (unlikely(ret))
		return ret;

	if (pkt_type == PKT_NORMAL) {
	addr = dma_map_single(ld->dev, skb->data, skb->len, DMA_TO_DEVICE);
	ret = dma_mapping_error(ld->dev, addr);
	if (unlikely(ret)) {
		pr_err("Normal Packet DMA Mapping fail.\n");
		return -EFAULT;
	}
	tx_bq_desc->data_buff_addr = (u32)addr;
#if defined(CONFIG_GMAC_DDR_64BIT )
	tx_bq_desc->reserve_desc2 = (addr >> REG_BIT_WIDTH) & TX_DESC_HI8_MASK;
#endif
	} else {
		ret = gmac_xmit_gso_sg(ld, skb, tx_bq_desc, desc_pos);
		if (unlikely(ret))
			return ret;
	}

	return 0;
}

netdev_tx_t gmac_net_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct gmac_netdev_local *ld = netdev_priv(dev);
	struct gmac_desc *desc = NULL;
	unsigned long txflags;
	int ret;
	u32 pos;

	if (unlikely(gmac_check_skb_len(skb, dev) < 0))
		return NETDEV_TX_OK;

	/*
	 * if adding gmac_xmit_reclaim here, iperf tcp client
	 * performance will be affected, from 550M(avg) to 513M~300M
	 */

	/* software write pointer */
	pos = dma_cnt(readl(ld->gmac_iobase + TX_BQ_WR_ADDR));

	spin_lock_irqsave(&ld->txlock, txflags);

	if (unlikely(ld->tx_skb[pos] || ld->TX_BQ.skb[pos])) {
		dev->stats.tx_dropped++;
		dev->stats.tx_fifo_errors++;
		netif_stop_queue(dev);
		spin_unlock_irqrestore(&ld->txlock, txflags);

		return NETDEV_TX_BUSY;
	}

	ld->TX_BQ.skb[pos] = skb;
	ld->tx_skb[pos] = skb;

	desc = ld->TX_BQ.desc + pos;

	if (ld->tso_supported) {
		ret = gmac_xmit_gso(ld, skb, (struct gmac_tso_desc *)desc, pos);
		if (unlikely(ret < 0)) {
			ld->tx_skb[pos] = NULL;
			ld->TX_BQ.skb[pos] = NULL;
			spin_unlock_irqrestore(&ld->txlock, txflags);

			if (ret == -ENOTSUPP)
				return gmac_sw_gso(ld, skb);

			dev_kfree_skb_any(skb);
			dev->stats.tx_dropped++;
			return NETDEV_TX_OK;
		}
	} else {
		ret = gmac_net_xmit_normal(skb, dev, desc, pos);
		if (unlikely(ret < 0)) {
			spin_unlock_irqrestore(&ld->txlock, txflags);
			return NETDEV_TX_OK;
		}
	}

	/*
	 * This barrier is important here.  It is required to ensure
	 * the ARM CPU flushes it's DMA write buffers before proceeding
	 * to the next instruction, to ensure that GMAC will see
	 * our descriptor changes in memory
	 */
	gmac_sync_barrier();
	pos = dma_ring_incr(pos, TX_DESC_NUM);
	writel(dma_byte(pos), ld->gmac_iobase + TX_BQ_WR_ADDR);

	netif_trans_update(dev);
	dev->stats.tx_packets++;
	dev->stats.tx_bytes += skb->len;
	netdev_sent_queue(dev, skb->len);

	spin_unlock_irqrestore(&ld->txlock, txflags);

	return NETDEV_TX_OK;
}

/* set gmac's multicast list, here we setup gmac's mc filter */
static void gmac_gmac_multicast_list(struct net_device const *dev)
{
	struct gmac_netdev_local *ld = netdev_priv(dev);
	struct netdev_hw_addr *ha = NULL;
	unsigned int d;
	unsigned int rec_filter;

	rec_filter = readl(ld->gmac_iobase + REC_FILT_CONTROL);
	/*
	 * when set gmac in promisc mode
	 * a. dev in IFF_PROMISC mode
	 */
	if ((dev->flags & IFF_PROMISC)) {
		/* promisc mode.received all pkgs. */
		rec_filter &= ~(BIT_BC_DROP_EN | BIT_MC_MATCH_EN |
				BIT_UC_MATCH_EN);
	} else {
		/* drop uc pkgs with field 'DA' not match our's */
		rec_filter |= BIT_UC_MATCH_EN;

		if (dev->flags & IFF_BROADCAST) /* no broadcast */
			rec_filter &= ~BIT_BC_DROP_EN;
		else
			rec_filter |= BIT_BC_DROP_EN;

		if (netdev_mc_empty(dev) || !(dev->flags & IFF_MULTICAST)) {
			/* haven't join any mc group */
			writel(0, ld->gmac_iobase + PORT_MC_ADDR_LOW);
			writel(0, ld->gmac_iobase + PORT_MC_ADDR_HIGH);
			rec_filter |= BIT_MC_MATCH_EN;
		} else if ((netdev_mc_count(dev) == 1) &&
			   (dev->flags & IFF_MULTICAST)) {
			netdev_for_each_mc_addr(ha, dev) {
				d = (ha->addr[0] << 8) | (ha->addr[1]); /* mac[0]->(15, 8) mac[1]->(7, 0) */
				writel(d, ld->gmac_iobase + PORT_MC_ADDR_HIGH);

				d = (ha->addr[2] << 24) | (ha->addr[3] << 16) | /* mac[2]->(31, 24) mac[3]->(23, 16) */
				    (ha->addr[4] << 8) | (ha->addr[5]); /* mac[4]->(15, 8) mac[5]->(7, 0) */
				writel(d, ld->gmac_iobase + PORT_MC_ADDR_LOW);
			}
			rec_filter |= BIT_MC_MATCH_EN;
		} else {
			rec_filter &= ~BIT_MC_MATCH_EN;
		}
	}
	writel(rec_filter, ld->gmac_iobase + REC_FILT_CONTROL);
}

void gmac_set_multicast_list(struct net_device *dev)
{
	gmac_gmac_multicast_list(dev);
}

int gmac_set_features(struct net_device *dev, netdev_features_t features)
{
	struct gmac_netdev_local *ld = netdev_priv(dev);
	netdev_features_t changed = dev->features ^ features;

	if (changed & NETIF_F_RXCSUM) {
		if (features & NETIF_F_RXCSUM)
			gmac_enable_rxcsum_drop(ld, true);
		else
			gmac_enable_rxcsum_drop(ld, false);
	}

	return 0;
}

int gmac_ioctl(struct net_device *ndev, struct ifreq *rq, int cmd)
{
	struct gmac_netdev_local *priv = NULL;
	struct pm_config config;
	int val = 0;
	if (ndev == NULL || rq == NULL)
		return -EINVAL;
	priv = netdev_priv(ndev);
	switch (cmd) {
	case SIOCSETPM:
		if (rq->ifr_data == NULL ||
				copy_from_user(&config, rq->ifr_data, sizeof(config)))
			return -EFAULT;
		return pmt_config(ndev, &config);

	case SIOCSETSUSPEND:
		if (rq->ifr_data == NULL || copy_from_user(&val, rq->ifr_data, sizeof(val)))
			return -EFAULT;
		return set_suspend(val);

	case SIOCSETRESUME:
		if (rq->ifr_data == NULL || copy_from_user(&val, rq->ifr_data, sizeof(val)))
			return -EFAULT;
		return set_resume(val);

	default:
		if (!netif_running(ndev))
			return -EINVAL;

		if (priv->phy == NULL)
			return -EINVAL;

		return phy_mii_ioctl(priv->phy, rq, cmd);
	}
	return 0;
}

int gmac_net_set_mac_address(struct net_device *dev, void *p)
{
	int ret;

	ret = eth_mac_addr(dev, p);
	if (!ret)
		gmac_hw_set_mac_addr(dev);

	return ret;
}

int eth_change_mtu(struct net_device *dev, int new_mtu)
{
	netdev_warn(dev, "%s is deprecated\n", __func__);
	dev->mtu = new_mtu;
	return 0;
}

struct net_device_stats *gmac_net_get_stats(struct net_device *dev)
{
	return &dev->stats;
}

static void gmac_do_udp_checksum(struct sk_buff *skb)
{
	int offset;
	__wsum csum;
	__sum16 udp_csum;

	offset = skb_checksum_start_offset(skb);
	WARN_ON(offset >= skb_headlen(skb));
	csum = skb_checksum(skb, offset, skb->len - offset, 0);

	offset += skb->csum_offset;
	WARN_ON(offset + sizeof(__sum16) > skb_headlen(skb));
	udp_csum = csum_fold(csum);
	if (udp_csum == 0)
		udp_csum = CSUM_MANGLED_0;

	*(__sum16 *)(skb->data + offset) = udp_csum;

	skb->ip_summed = CHECKSUM_NONE;
}

static int gmac_get_pkt_info_l3l4(struct gmac_tso_desc *tx_bq_desc,
		  struct sk_buff *skb, unsigned int *l4_proto, unsigned int *max_mss,
		  unsigned char *coe_enable)
{
	__be16 l3_proto; /* level 3 protocol */
	int max_data_len = skb->len - ETH_HLEN;

	l3_proto = skb->protocol;
	if (skb->protocol == htons(ETH_P_8021Q)) {
		l3_proto = vlan_get_protocol(skb);
		tx_bq_desc->desc1.tx.vlan_flag = 1;
		max_data_len -= VLAN_HLEN;
	}

	if (l3_proto == htons(ETH_P_IP)) {
		struct iphdr *iph;

		iph = ip_hdr(skb);
		tx_bq_desc->desc1.tx.ip_ver = PKT_IPV4;
		tx_bq_desc->desc1.tx.ip_hdr_len = iph->ihl;

		if ((max_data_len >= GSO_MAX_SIZE) &&
			(ntohs(iph->tot_len) <= (iph->ihl << 2))) /* shift left 2 */
			iph->tot_len = htons(GSO_MAX_SIZE - 1);

		*max_mss -= iph->ihl * WORD_TO_BYTE;
		*l4_proto = iph->protocol;
	} else if (l3_proto == htons(ETH_P_IPV6)) {
		tx_bq_desc->desc1.tx.ip_ver = PKT_IPV6;
		tx_bq_desc->desc1.tx.ip_hdr_len = PKT_IPV6_HDR_LEN;
		*max_mss -= PKT_IPV6_HDR_LEN * WORD_TO_BYTE;
		*l4_proto = ipv6_hdr(skb)->nexthdr;
	} else {
		*coe_enable = 0;
	}

	if (*l4_proto == IPPROTO_TCP) {
		tx_bq_desc->desc1.tx.prot_type = PKT_TCP;
		if (tcp_hdr(skb)->doff < sizeof(struct tcphdr) / WORD_TO_BYTE)
			return -EFAULT;
		tx_bq_desc->desc1.tx.prot_hdr_len = tcp_hdr(skb)->doff;
		*max_mss -= tcp_hdr(skb)->doff * WORD_TO_BYTE;
	} else if (*l4_proto == IPPROTO_UDP) {
		tx_bq_desc->desc1.tx.prot_type = PKT_UDP;
		tx_bq_desc->desc1.tx.prot_hdr_len = PKT_UDP_HDR_LEN;
		if (l3_proto == htons(ETH_P_IPV6))
			*max_mss -= sizeof(struct frag_hdr);
	} else {
		*coe_enable = 0;
	}

	return 0;
}

static int gmac_get_pkt_info(struct gmac_netdev_local *ld,
		  struct sk_buff *skb, struct gmac_tso_desc *tx_bq_desc)
{
	int nfrags;
	unsigned int l4_proto = IPPROTO_MAX;
	unsigned int max_mss = ETH_DATA_LEN;
	unsigned char coe_enable = 0;
	int ret;
	if (skb == NULL || tx_bq_desc == NULL)
		return -EINVAL;

	nfrags = skb_shinfo(skb)->nr_frags;
	if (likely(skb->ip_summed == CHECKSUM_PARTIAL))
		coe_enable = 1;

	tx_bq_desc->desc1.val = 0;

	if (skb_is_gso(skb)) {
		tx_bq_desc->desc1.tx.tso_flag = 1;
		tx_bq_desc->desc1.tx.sg_flag = 1;
	} else if (nfrags) {
		tx_bq_desc->desc1.tx.sg_flag = 1;
	}

	ret = gmac_get_pkt_info_l3l4(tx_bq_desc, skb, &l4_proto, &max_mss,
	    &coe_enable);
	if (ret < 0)
		return ret;

	if (skb_is_gso(skb))
		tx_bq_desc->desc1.tx.data_len =
			(skb_shinfo(skb)->gso_size > max_mss) ? max_mss :
					skb_shinfo(skb)->gso_size;
	else
		tx_bq_desc->desc1.tx.data_len = skb->len;

	if (coe_enable && skb_is_gso(skb) && (l4_proto == IPPROTO_UDP))
		gmac_do_udp_checksum(skb);

	if (coe_enable)
		tx_bq_desc->desc1.tx.coe_flag = 1;

	tx_bq_desc->desc1.tx.nfrags_num = nfrags;

	tx_bq_desc->desc1.tx.hw_own = DESC_VLD_BUSY;
	return 0;
}

static int gmac_check_hw_capability_for_udp(struct sk_buff const *skb)
{
	struct ethhdr *eth;

	/* hardware can't dea with UFO broadcast packet */
	eth = (struct ethhdr *)(skb->data);
	if (skb_is_gso(skb) && is_broadcast_ether_addr(eth->h_dest))
		return -ENOTSUPP;

	return 0;
}

static int gmac_check_hw_capability_for_ipv6(struct sk_buff *skb)
{
	unsigned int l4_proto;

	l4_proto = ipv6_hdr(skb)->nexthdr;
	if ((l4_proto != IPPROTO_TCP) && (l4_proto != IPPROTO_UDP)) {
		/*
		 * when IPv6 next header is not tcp or udp,
		 * it means that IPv6 next header is extension header.
		 * Hardware can't deal with this case,
		 * so do checksumming by software or do GSO by software.
		 */
		if (skb_is_gso(skb))
			return -ENOTSUPP;

		if (skb->ip_summed == CHECKSUM_PARTIAL &&
		    skb_checksum_help(skb))
			return -EFAULT;
	}

	return 0;
}

static __be16 gmac_get_l3_proto(struct sk_buff *skb)
{
	__be16 l3_proto;

	l3_proto = skb->protocol;
	if (skb->protocol == htons(ETH_P_8021Q))
		l3_proto = vlan_get_protocol(skb);

	return l3_proto;
}

static unsigned int gmac_get_l4_proto(struct sk_buff *skb)
{
	__be16 l3_proto;
	unsigned int l4_proto = IPPROTO_MAX;

	l3_proto = gmac_get_l3_proto(skb);
	if (l3_proto == htons(ETH_P_IP))
		l4_proto = ip_hdr(skb)->protocol;
	else if (l3_proto == htons(ETH_P_IPV6))
		l4_proto = ipv6_hdr(skb)->nexthdr;

	return l4_proto;
}

static inline bool gmac_skb_is_ipv6(struct sk_buff *skb)
{
	return (gmac_get_l3_proto(skb) == htons(ETH_P_IPV6));
}

static inline bool gmac_skb_is_udp(struct sk_buff *skb)
{
	return (gmac_get_l4_proto(skb) == IPPROTO_UDP);
}

static inline bool gmac_skb_is_ipv4_with_options(struct sk_buff *skb)
{
	return ((gmac_get_l3_proto(skb) == htons(ETH_P_IP)) &&
		(ip_hdr(skb)->ihl > IPV4_HEAD_LENGTH));
}

static int gmac_check_hw_capability(struct sk_buff *skb)
{
	int ret;

	/*
	 * if tcp_mtu_probe() use (2 * tp->mss_cache) as probe_size,
	 * the linear data length will be larger than 2048,
	 * the MAC can't handle it, so let the software do it.
	 */
	 if (skb == NULL)
		return -EINVAL;
	if (skb_is_gso(skb) && (skb_headlen(skb) > 2048)) /* 2048(2k) */
		return -ENOTSUPP;

	if (gmac_skb_is_ipv6(skb)) {
		ret = gmac_check_hw_capability_for_ipv6(skb);
		if (ret)
			return ret;
	}

	if (gmac_skb_is_udp(skb)) {
		ret = gmac_check_hw_capability_for_udp(skb);
		if (ret)
			return ret;
	}

	if (((skb->ip_summed == CHECKSUM_PARTIAL) || skb_is_gso(skb)) &&
	    gmac_skb_is_ipv4_with_options(skb))
		return -ENOTSUPP;

	return 0;
}
