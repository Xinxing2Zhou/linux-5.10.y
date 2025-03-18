/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#ifndef GMAC_ETHTOOL_OPS_H
#define GMAC_ETHTOOL_OPS_H

#include "gmac.h"

void gmac_get_drvinfo(struct net_device *net_dev,
			struct ethtool_drvinfo *info);
unsigned int gmac_get_link(struct net_device *net_dev);
int gmac_get_settings(struct net_device *net_dev, struct ethtool_cmd *cmd);
int gmac_set_settings(struct net_device *net_dev, struct ethtool_cmd *cmd);
void gmac_get_pauseparam(struct net_device *net_dev,
			   struct ethtool_pauseparam *pause);
int gmac_set_pauseparam(struct net_device *net_dev,
			  struct ethtool_pauseparam *pause);
u32 gmac_ethtool_getmsglevel(struct net_device *ndev);
void gmac_ethtool_setmsglevel(struct net_device *ndev, u32 level);
u32 gmac_get_rxfh_key_size(struct net_device *ndev);
u32 gmac_get_rxfh_indir_size(struct net_device *ndev);
int gmac_get_rxfh(struct net_device *ndev, u32 *indir, u8 *hkey, u8 *hfunc);
int gmac_set_rxfh(struct net_device *ndev, const u32 *indir,
		    const u8 *hkey, const u8 hfunc);
int gmac_get_rxnfc(struct net_device *ndev,
		     struct ethtool_rxnfc *info, u32 *rules);
int gmac_set_rxnfc(struct net_device *ndev, struct ethtool_rxnfc *info);
int gmac_get_ts_info(struct net_device *dev,
			      struct ethtool_ts_info *info);

/* gmac.c related func */
void gmac_get_rss_key(struct gmac_netdev_local *priv);
void gmac_get_rss(struct gmac_netdev_local *priv);
void gmac_config_hash_policy(struct gmac_netdev_local const *priv);

#endif
