/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#ifndef GMAC_NETDEV_OPS_H
#define GMAC_NETDEV_OPS_H

#include <linux/netdevice.h>

#include "gmac.h"

int gmac_net_open(struct net_device *dev);
int gmac_net_close(struct net_device *dev);
netdev_tx_t gmac_net_xmit(struct sk_buff *skb, struct net_device *dev);
void gmac_set_multicast_list(struct net_device *dev);
int gmac_set_features(struct net_device *dev, netdev_features_t features);
int gmac_ioctl(struct net_device *net_dev, struct ifreq *rq, int cmd);
int gmac_net_set_mac_address(struct net_device *dev, void *p);
int eth_change_mtu(struct net_device *dev, int new_mtu);
struct net_device_stats *gmac_net_get_stats(struct net_device *dev);

#endif
