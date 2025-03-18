/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#include <linux/securec.h>

#include "gmac.h"
#include "gmac_ethtool_ops.h"

void gmac_get_drvinfo(struct net_device *net_dev,
		  struct ethtool_drvinfo *info)
{
	if (info == NULL)
		return;
	if (strncpy_s(info->driver, sizeof(info->driver), "gmac driver", sizeof(info->driver)))
		printk("strncpy_s  err : %s %d.\n", __func__, __LINE__);
	if (strncpy_s(info->version, sizeof(info->version), "gmac v200", sizeof(info->version)))
		printk("strncpy_s  err : %s %d.\n", __func__, __LINE__);
	if (strncpy_s(info->bus_info, sizeof(info->bus_info), "platform", sizeof(info->bus_info)))
		printk("strncpy_s  err : %s %d.\n", __func__, __LINE__);
}

unsigned int gmac_get_link(struct net_device *net_dev)
{
	struct gmac_netdev_local *ld = netdev_priv(net_dev);

	return ld->phy->link ? GMAC_LINKED : 0;
}

void gmac_get_pauseparam(struct net_device *net_dev,
		  struct ethtool_pauseparam *pause)
{
	struct gmac_netdev_local *ld = NULL;
	if (net_dev == NULL || pause == NULL)
		return;
	ld = netdev_priv(net_dev);

	pause->rx_pause = 0;
	pause->tx_pause = 0;
	pause->autoneg = ld->phy->autoneg;

	if (ld->phy->pause && (ld->flow_ctrl & FLOW_RX))
		pause->rx_pause = 1;
	if (ld->phy->pause && (ld->flow_ctrl & FLOW_TX))
		pause->tx_pause = 1;
}

int gmac_set_pauseparam(struct net_device *net_dev,
		  struct ethtool_pauseparam *pause)
{
	struct gmac_netdev_local *ld = netdev_priv(net_dev);
	struct phy_device *phy = ld->phy;
	unsigned int new_pause = FLOW_OFF;

	if (pause == NULL)
		return -ENOMEM;

	if (pause->rx_pause)
		new_pause |= FLOW_RX;
	if (pause->tx_pause)
		new_pause |= FLOW_TX;

	if (new_pause != ld->flow_ctrl)
		ld->flow_ctrl = new_pause;

	gmac_set_flow_ctrl_state(ld, phy->pause);
	linkmode_clear_bit(ETHTOOL_LINK_MODE_Pause_BIT, phy->advertising);
	if (ld->flow_ctrl)
		linkmode_set_bit(ETHTOOL_LINK_MODE_Pause_BIT, phy->advertising);

	if (phy->autoneg) {
		if (netif_running(net_dev))
			return phy_start_aneg(phy);
	}

	return 0;
}

u32 gmac_ethtool_getmsglevel(struct net_device *ndev)
{
	struct gmac_netdev_local *priv = netdev_priv(ndev);

	return priv->msg_enable;
}

void gmac_ethtool_setmsglevel(struct net_device *ndev, u32 level)
{
	struct gmac_netdev_local *priv = netdev_priv(ndev);

	priv->msg_enable = level;
}

u32 gmac_get_rxfh_key_size(struct net_device *ndev)
{
	return RSS_HASH_KEY_SIZE;
}

u32 gmac_get_rxfh_indir_size(struct net_device *ndev)
{
	struct gmac_netdev_local *priv = netdev_priv(ndev);

	return priv->rss_info.ind_tbl_size;
}

int gmac_get_rxfh(struct net_device *ndev, u32 *indir, u8 *hkey,
		  u8 *hfunc)
{
	struct gmac_netdev_local *priv = netdev_priv(ndev);
	struct gmac_rss_info *rss = &priv->rss_info;

	if (hfunc != NULL)
		*hfunc = ETH_RSS_HASH_TOP;

	if (hkey != NULL)
		if (memcpy_s(hkey, RSS_HASH_KEY_SIZE, rss->key, RSS_HASH_KEY_SIZE) < 0)
			printk("memcpy_s  err : %s %d.\n", __func__, __LINE__);

	if (indir != NULL) {
		int i;

		for (i = 0; i < rss->ind_tbl_size; i++)
			indir[i] = rss->ind_tbl[i];
	}

	return 0;
}

void gmac_get_rss_key(struct gmac_netdev_local *priv)
{
	struct gmac_rss_info *rss = NULL;
	u32 hkey;
	if (priv == NULL)
		return;
	rss = &priv->rss_info;
	hkey = readl(priv->gmac_iobase + RSS_HASH_KEY);
	*((u32 *)rss->key) = hkey;
}

static void gmac_set_rss_key(struct gmac_netdev_local *priv)
{
	struct gmac_rss_info *rss = &priv->rss_info;

	writel(*((u32 *)rss->key), priv->gmac_iobase + RSS_HASH_KEY);
}

static int gmac_wait_rss_ready(struct gmac_netdev_local const *priv)
{
	void __iomem *base = priv->gmac_iobase;
	int i;
	const int timeout = 10000;

	for (i = 0; !(readl(base + RSS_IND_TBL) & BIT_IND_TBL_READY); i++) {
		if (i == timeout) {
			netdev_err(priv->netdev, "wait rss ready timeout!\n");
			return -ETIMEDOUT;
		}
		usleep_range(10, 20); /* wait 10~20us */
	}

	return 0;
}


static void gmac_config_rss(struct gmac_netdev_local *priv)
{
	struct gmac_rss_info *rss = NULL;
	u32 rss_val;
	unsigned int i;
	if (priv == NULL)
		return;
	rss = &priv->rss_info;
	for (i = 0; i < rss->ind_tbl_size; i++) {
		if (gmac_wait_rss_ready(priv) != 0)
			break;
		rss_val = BIT_IND_TLB_WR | (rss->ind_tbl[i] << 8) | i; /* shift 8 */
		writel(rss_val, priv->gmac_iobase + RSS_IND_TBL);
	}
}

void gmac_get_rss(struct gmac_netdev_local *priv)
{
	struct gmac_rss_info *rss = NULL;
	u32 rss_val;
	int i;
	if (priv == NULL)
		return;
	rss = &priv->rss_info;
	for (i = 0; i < rss->ind_tbl_size; i++) {
		if (gmac_wait_rss_ready(priv) != 0)
			break;
		writel(i, priv->gmac_iobase + RSS_IND_TBL);
		if (gmac_wait_rss_ready(priv) != 0)
			break;
		rss_val = readl(priv->gmac_iobase + RSS_IND_TBL);
		rss->ind_tbl[i] = (rss_val >> 10) & 0x3; /* right shift 10 */
	}
}

int gmac_set_rxfh(struct net_device *ndev, const u32 *indir,
		  const u8 *hkey, const u8 hfunc)
{
	struct gmac_netdev_local *priv = netdev_priv(ndev);
	struct gmac_rss_info *rss = &priv->rss_info;

	if (hfunc != ETH_RSS_HASH_NO_CHANGE && hfunc != ETH_RSS_HASH_TOP)
		return -EOPNOTSUPP;

	if (indir != NULL) {
		int i;

		for (i = 0; i < rss->ind_tbl_size; i++)
			rss->ind_tbl[i] = indir[i];
	}

	if (hkey != NULL) {
		if (memcpy_s(rss->key, RSS_HASH_KEY_SIZE, hkey, RSS_HASH_KEY_SIZE) < 0)
			printk("memcpy_s  err : %s %d.\n", __func__, __LINE__);
		gmac_set_rss_key(priv);
	}

	gmac_config_rss(priv);

	return 0;
}

static void gmac_get_rss_hash(struct ethtool_rxnfc *info, u32 hash_cfg,
		  u32 l3_hash_en, u32 l4_hash_en, u32 vlan_hash_en)
{
	if (hash_cfg & l3_hash_en)
		info->data |= RXH_IP_SRC | RXH_IP_DST;
	if (hash_cfg & l4_hash_en)
		info->data |= RXH_L4_B_0_1 | RXH_L4_B_2_3;
	if (hash_cfg & vlan_hash_en)
		info->data |= RXH_VLAN;
}

static int gmac_get_rss_hash_opts(struct gmac_netdev_local const *priv,
		  struct ethtool_rxnfc *info)
{
	u32 hash_cfg = priv->rss_info.hash_cfg;

	info->data = 0;

	switch (info->flow_type) {
	case TCP_V4_FLOW:
		gmac_get_rss_hash(info, hash_cfg, TCPV4_L3_HASH_EN, TCPV4_L4_HASH_EN,
			TCPV4_VLAN_HASH_EN);
		break;
	case TCP_V6_FLOW:
		gmac_get_rss_hash(info, hash_cfg, TCPV6_L3_HASH_EN, TCPV6_L4_HASH_EN,
			TCPV6_VLAN_HASH_EN);
		break;
	case UDP_V4_FLOW:
		gmac_get_rss_hash(info, hash_cfg, UDPV4_L3_HASH_EN, UDPV4_L4_HASH_EN,
			UDPV4_VLAN_HASH_EN);
		break;
	case UDP_V6_FLOW:
		gmac_get_rss_hash(info, hash_cfg, UDPV6_L3_HASH_EN, UDPV6_L4_HASH_EN,
			UDPV6_VLAN_HASH_EN);
		break;
	case IPV4_FLOW:
		gmac_get_rss_hash(info, hash_cfg, IPV4_L3_HASH_EN, 0,
			IPV4_VLAN_HASH_EN);
		break;
	case IPV6_FLOW:
		gmac_get_rss_hash(info, hash_cfg, IPV6_L3_HASH_EN, 0,
			IPV6_VLAN_HASH_EN);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

int gmac_get_rxnfc(struct net_device *ndev,
		  struct ethtool_rxnfc *info, u32 *rules)
{
	struct gmac_netdev_local *priv = netdev_priv(ndev);
	int ret = -EOPNOTSUPP;
	if (info == NULL)
		return -EINVAL;
	switch (info->cmd) {
	case ETHTOOL_GRXRINGS:
		info->data = priv->num_rxqs;
		ret = 0;
		break;
	case ETHTOOL_GRXFH:
		return gmac_get_rss_hash_opts(priv, info);
	default:
		break;
	}
	return ret;
}

void gmac_config_hash_policy(struct gmac_netdev_local const *priv)
{
	if (priv == NULL)
		return;
	writel(priv->rss_info.hash_cfg, priv->gmac_iobase + RSS_HASH_CONFIG);
}

static int gmac_set_tcp_udp_hash_cfg(struct ethtool_rxnfc const *info,
		  u32 *hash_cfg, u32 l4_mask, u32 vlan_mask)
{
	switch (info->data & (RXH_L4_B_0_1 | RXH_L4_B_2_3)) {
	case 0: // all bits is 0
		*hash_cfg &= ~l4_mask;
		break;
	case (RXH_L4_B_0_1 | RXH_L4_B_2_3):
		*hash_cfg |= l4_mask;
		break;
	default:
		return -EINVAL;
	}
	if (info->data & RXH_VLAN)
		*hash_cfg |= vlan_mask;
	else
		*hash_cfg &= ~vlan_mask;
	return 0;
}

static int gmac_ip_hash_cfg(struct ethtool_rxnfc const *info,
		  u32 *hash_cfg, u32 vlan_mask)
{
	if (info->data & (RXH_L4_B_0_1 | RXH_L4_B_2_3))
		return -EINVAL;
	if (info->data & RXH_VLAN)
		*hash_cfg |= vlan_mask;
	else
		*hash_cfg &= ~vlan_mask;
	return 0;
}

static int gmac_set_rss_hash_opts(struct gmac_netdev_local *priv,
		  struct ethtool_rxnfc const *info)
{
	u32 hash_cfg;
	if (priv == NULL || priv->netdev == NULL)
		return -EINVAL;
	hash_cfg = priv->rss_info.hash_cfg;
	netdev_info(priv->netdev, "Set RSS flow type = %d, data = %lld\n",
		    info->flow_type, info->data);

	if (!(info->data & RXH_IP_SRC) || !(info->data & RXH_IP_DST))
		return -EINVAL;

	switch (info->flow_type) {
	case TCP_V4_FLOW:
		if (gmac_set_tcp_udp_hash_cfg(info, &hash_cfg,
			TCPV4_L4_HASH_EN, TCPV4_VLAN_HASH_EN) == -EINVAL)
			return -EINVAL;
		break;
	case TCP_V6_FLOW:
		if (gmac_set_tcp_udp_hash_cfg(info, &hash_cfg,
			TCPV6_L4_HASH_EN, TCPV6_VLAN_HASH_EN) == -EINVAL)
			return -EINVAL;
		break;
	case UDP_V4_FLOW:
		if (gmac_set_tcp_udp_hash_cfg(info, &hash_cfg,
			UDPV4_L4_HASH_EN, UDPV4_L4_HASH_EN) == -EINVAL)
			return -EINVAL;
		break;
	case UDP_V6_FLOW:
		if (gmac_set_tcp_udp_hash_cfg(info, &hash_cfg,
			UDPV6_L4_HASH_EN, UDPV6_L4_HASH_EN) == -EINVAL)
			return -EINVAL;
		break;
	case IPV4_FLOW:
		if (gmac_ip_hash_cfg(info, &hash_cfg,
			IPV4_VLAN_HASH_EN) == -EINVAL)
			return -EINVAL;
		break;
	case IPV6_FLOW:
		if (gmac_ip_hash_cfg(info, &hash_cfg,
			IPV6_VLAN_HASH_EN) == -EINVAL)
			return -EINVAL;
		break;
	default:
		return -EINVAL;
	}

	priv->rss_info.hash_cfg = hash_cfg;
	gmac_config_hash_policy(priv);

	return 0;
}

int gmac_set_rxnfc(struct net_device *ndev, struct ethtool_rxnfc *info)
{
	struct gmac_netdev_local *priv = netdev_priv(ndev);
	if (info == NULL)
		return -EINVAL;
	switch (info->cmd) {
	case ETHTOOL_SRXFH:
		return gmac_set_rss_hash_opts(priv, info);
	default:
		break;
	}
	return -EOPNOTSUPP;
}
