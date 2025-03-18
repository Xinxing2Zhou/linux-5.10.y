/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#ifndef GMAC_GMAC_H
#define GMAC_GMAC_H

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/list.h>
#include <linux/phy.h>
#include <linux/io.h>
#include <linux/interrupt.h>

#define STATION_ADDR_LOW		0x0000
#define STATION_ADDR_HIGH		0x0004
#define MAC_DUPLEX_HALF_CTRL		0x0008

#define PORT_MODE			0x0040

#define PORT_EN				0x0044
#define BITS_TX_EN			BIT(2)
#define BITS_RX_EN			BIT(1)

#define FC_TX_TIMER			0x001C

#define PAUSE_THR			0x0038

#define PAUSE_EN			0x0048
#define BIT_RX_FDFC			BIT(0)
#define BIT_TX_FDFC			BIT(1)

#define RX_PAUSE_EN			0x02A4
#define BIT_RX_FQ_PAUSE_EN		BIT(0)
#define BIT_RX_BQ_PAUSE_EN		BIT(1)

#define CRF_TX_PAUSE			0x0340

#define BITS_Q_PAUSE_TH_OFFSET		16
#define BITS_Q_PAUSE_TH_MASK		0xFFFF

#define REC_FILT_CONTROL		0x0064
#define BIT_CRC_ERR_PASS		BIT(5)
#define BIT_PAUSE_FRM_PASS		BIT(4)
#define BIT_VLAN_DROP_EN		BIT(3)
#define BIT_BC_DROP_EN			BIT(2)
#define BIT_MC_MATCH_EN			BIT(1)
#define BIT_UC_MATCH_EN			BIT(0)

#define	PORT_MC_ADDR_LOW		0x0068
#define	PORT_MC_ADDR_HIGH		0x006C
#define MAC_CLEAR			0x0070
#define BIT_TX_SOFT_RESET		BIT(0)

#define MODE_CHANGE_EN			0x01b4
#define BIT_MODE_CHANGE_EN		BIT(0)

#define COL_SLOT_TIME			0x01c0

#define CRF_MIN_PACKET			0x0210
#define BIT_OFFSET_TX_MIN_LEN		8
#define BIT_MASK_TX_MIN_LEN		GENMASK(13, 8)

#define CONTROL_WORD			0x0214
#define CONTROL_WORD_CONFIG		0x640

#define TSO_COE_CTRL			0x02e8
#define BIT_COE_IPHDR_DROP		BIT(4)
#define BIT_COE_PAYLOAD_DROP		BIT(5)
#define BIT_COE_IPV6_UDP_ZERO_DROP	BIT(6)
#define COE_ERR_DROP			(BIT_COE_IPHDR_DROP | \
					BIT_COE_PAYLOAD_DROP | \
					BIT_COE_IPV6_UDP_ZERO_DROP)

#define RX_FQ_START_ADDR		0x0500
#define RX_FQ_DEPTH			0x0504
#define REG_BIT_WIDTH			32
#define Q_ADDR_HI8_OFFSET		24
#define Q_ADDR_HI8_MASK			(BIT(Q_ADDR_HI8_OFFSET) - 1)
#define TX_DESC_HI8_MASK		0xff
#define SG_DESC_HI8_OFFSET		8
#define RX_FQ_WR_ADDR			0x0508
#define BITS_RX_FQ_WR_ADDR		mk_bits(0, 21)
#define RX_FQ_RD_ADDR			0x050c
#define BITS_RX_FQ_RD_ADDR		mk_bits(0, 21)
#define RX_FQ_VLDDESC_CNT		0x0510
#define BITS_RX_FQ_VLDDESC_CNT		mk_bits(0, 16)
#define RX_FQ_ALEMPTY_TH		0x0514
#define BITS_RX_FQ_ALEMPTY_TH		mk_bits(0, 16)
#define RX_FQ_REG_EN			0x0518
#define BITS_RX_FQ_START_ADDR_EN	BIT(2)
#define BITS_RX_FQ_DEPTH_EN		BIT(1)
#define BITS_RX_FQ_RD_ADDR_EN		mk_bits(0, 1)
#define RX_FQ_ALFULL_TH			0x051c
#define BITS_RX_FQ_ALFULL_TH		mk_bits(0, 16)

#define RX_BQ_START_ADDR		0x0520
#define RX_BQ_DEPTH			0x0524
#define RX_BQ_WR_ADDR			0x0528
#define RX_BQ_RD_ADDR			0x052c
#define RX_BQ_FREE_DESC_CNT		0x0530
#define BITS_RX_BQ_FREE_DESC_CNT	mk_bits(0, 16)
#define RX_BQ_ALEMPTY_TH		0x0534
#define BITS_RX_BQ_ALEMPTY_TH		mk_bits(0, 16)
#define RX_BQ_REG_EN			0x0538
#define BITS_RX_BQ_START_ADDR_EN	BIT(2)
#define BITS_RX_BQ_DEPTH_EN		BIT(1)
#define BITS_RX_BQ_WR_ADDR_EN		mk_bits(0, 1)
#define RX_BQ_ALFULL_TH			0x053c
#define BITS_RX_BQ_ALFULL_TH		mk_bits(0, 16)

#define TX_BQ_START_ADDR		0x0580
#define TX_BQ_DEPTH			0x0584
#define TX_BQ_WR_ADDR			0x0588
#define BITS_TX_BQ_WR_ADDR		mk_bits(0, 21)
#define TX_BQ_RD_ADDR			0x058c
#define BITS_TX_BQ_RD_ADDR		mk_bits(0, 21)
#define TX_BQ_VLDDESC_CNT		0x0590
#define BITS_TX_BQ_VLDDESC_CNT		mk_bits(0, 16)
#define TX_BQ_ALEMPTY_TH		0x0594
#define BITS_TX_BQ_ALEMPTY_TH		mk_bits(0, 16)
#define TX_BQ_REG_EN			0x0598
#define BITS_TX_BQ_START_ADDR_EN	BIT(2)
#define BITS_TX_BQ_DEPTH_EN		BIT(1)
#define BITS_TX_BQ_RD_ADDR_EN		mk_bits(0, 1)
#define TX_BQ_ALFULL_TH			0x059c
#define BITS_TX_BQ_ALFULL_TH		mk_bits(0, 16)

#define TX_RQ_START_ADDR		0x05a0
#define TX_RQ_DEPTH			0x05a4
#define TX_RQ_WR_ADDR			0x05a8
#define BITS_TX_RQ_WR_ADDR		mk_bits(0, 21)
#define TX_RQ_RD_ADDR			0x05ac
#define BITS_TX_RQ_RD_ADDR		mk_bits(0, 21)
#define TX_RQ_FREE_DESC_CNT		0x05b0
#define BITS_TX_RQ_FREE_DESC_CNT	mk_bits(0, 16)
#define TX_RQ_ALEMPTY_TH		0x05b4
#define BITS_TX_RQ_ALEMPTY_TH		mk_bits(0, 16)
#define TX_RQ_REG_EN			0x05b8
#define BITS_TX_RQ_START_ADDR_EN	BIT(2)
#define BITS_TX_RQ_DEPTH_EN		BIT(1)
#define BITS_TX_RQ_WR_ADDR_EN		mk_bits(0, 1)
#define TX_RQ_ALFULL_TH			0x05bc
#define BITS_TX_RQ_ALFULL_TH		mk_bits(0, 16)

#define RAW_PMU_INT			0x05c0
#define ENA_PMU_INT			0x05c4

#define DESC_WR_RD_ENA				0x05CC

#define IN_QUEUE_TH					0x05d8
#define BITS_OFFSET_TX_RQ_IN_TH			16

#define RX_BQ_IN_TIMEOUT_TH			0x05E0

#define TX_RQ_IN_TIMEOUT_TH			0x05e4

#define STOP_CMD			0x05e8
#define BITS_TX_STOP_EN			BIT(1)
#define BITS_RX_STOP_EN			BIT(0)
#define	STOP_RX_TX			(BITS_TX_STOP_EN | BITS_RX_STOP_EN)

#define RSS_IND_TBL			0x0c0c
#define BIT_IND_TBL_READY		BIT(13)
#define BIT_IND_TLB_WR			BIT(12)
#define RSS_RAW_PMU_INT			0x0c10
#define RSS_QUEUE1_START_ADDR		0x0c20
#define rx_bq_start_addr_queue(i)	(RSS_QUEUE1_START_ADDR + \
					((i) - 1) * 0x10)
#define RSS_QUEUE1_DEPTH		0x0c24
#define RX_BQ_WR_ADDR_QUEUE1		0x0c28
#define RX_BQ_RD_ADDR_QUEUE1		0x0c2c
#define RSS_QUEUE1_ENA_INT		0x0c90
#define rss_ena_int_queue(i)		(RSS_QUEUE1_ENA_INT + ((i) - 1) * 0x4)
#define rx_bq_depth_queue(i)		(RSS_QUEUE1_DEPTH + ((i) - 1) * 0x10)
#define rx_bq_wr_addr_queue(i)		((i) ? (RX_BQ_WR_ADDR_QUEUE1 + \
					((i) - 1) * 0x10) : RX_BQ_WR_ADDR)
#define rx_bq_rd_addr_queue(i)		((i) ? (RX_BQ_RD_ADDR_QUEUE1 + \
					((i) - 1) * 0x10) : RX_BQ_RD_ADDR)

#define def_int_mask_queue(i)		(0x3 << (2 * ((i) - 1)))

/* AXI burst and outstanding config */
#define BURST_OUTSTANDING_REG		0x3014
#define BURST4_OUTSTANDING1		0x81ff
#define BURST_OUTSTANDING_OFFSET	16

#define GMAC_SPEED_1000			0x05
#define GMAC_SPEED_100			0x01
#define GMAC_SPEED_10			0x00

#define IPV4_HEAD_LENGTH		0x5

enum gmac_tx_err {
	ERR_NONE = 0,
	ERR_DESC_CFG = (1 << 0),
	ERR_DATA_LEN = (1 << 1),
	ERR_DESC_NFRAG_NUM = (1 << 2), /* bit2 */
	ERR_DESC_IP_HDR_LEN = (1 << 3), /* bit3 */
	ERR_DESC_PROT_HDR_LEN = (1 << 4), /* bit4 */
	ERR_DESC_MTU = (1 << 5), /* bit5 */
	ERR_LINK_SGPKT_LEN = (1 << 8), /* bit8 */
	ERR_LINK_TSOPKT_LINEAR = (1 << 9), /* bit9 */
	ERR_LINK_NFRAG_LEN = (1 << 10), /* bit10 */
	ERR_LINK_TOTAL_LEN = (1 << 11), /* bit11 */
	ERR_HDR_TCP_BCMC = (1 << 12), /* bit12 */
	ERR_HDR_UDP_BC = (1 << 13), /* bit13 */
	ERR_HDR_VLAN_IP_TYPE = (1 << 14), /* bit14 */
	ERR_HDR_IP_TYPE = (1 << 15), /* bit15 */
	ERR_HDR_IP_VERSION = (1 << 16), /* bit16 */
	ERR_HDR_IP_HDR_LEN = (1 << 17), /* bit17 */
	ERR_HDR_IP_TOTAL_LEN = (1 << 18), /* bit18 */
	ERR_HDR_IPV6_TTL_PROT = (1 << 19), /* bit19 */
	ERR_HDR_IPV4_OFFSET = (1 << 20), /* bit20 */
	ERR_HDR_IPV4_TTL_PROT = (1 << 21), /* bit21 */
	ERR_HDR_UDP_LEN = (1 << 22), /* bit22 */
	ERR_HDR_TCP_LEN = (1 << 23), /* bit23 */
	ERR_DESC = (ERR_DESC_CFG | ERR_DATA_LEN |
		    ERR_DESC_NFRAG_NUM | ERR_DESC_IP_HDR_LEN |
		    ERR_DESC_PROT_HDR_LEN | ERR_DESC_MTU),
	ERR_LINK = (ERR_LINK_SGPKT_LEN | ERR_LINK_TSOPKT_LINEAR |
		    ERR_LINK_NFRAG_LEN | ERR_LINK_TOTAL_LEN),
	ERR_HDR = (ERR_HDR_TCP_BCMC | ERR_HDR_UDP_BC |
		   ERR_HDR_VLAN_IP_TYPE | ERR_HDR_IP_TYPE |
		   ERR_HDR_IP_VERSION | ERR_HDR_IP_HDR_LEN |
		   ERR_HDR_IP_TOTAL_LEN | ERR_HDR_IPV6_TTL_PROT |
		   ERR_HDR_IPV4_OFFSET | ERR_HDR_IPV4_TTL_PROT |
		   ERR_HDR_UDP_LEN | ERR_HDR_TCP_LEN),
	ERR_ALL = (ERR_DESC | ERR_LINK | ERR_HDR),
};

#define GMAC_DRIVER_NAME	"gmac_v200"

#define GMAC_MAC_CLK_NAME	"gmac_clk"
#define GMAC_MACIF_CLK_NAME	"macif_clk"

#define GMAC_PORT_RST_NAME	"port_reset"
#define GMAC_MACIF_RST_NAME	"macif_reset"
#define GMAC_PHY_RST_NAME	"phy_reset"

#define GMAC_IOSIZE			0x1000
#define GMAC_OFFSET			(GMAC_IOSIZE)

#define RX_BQ_IN_INT			BIT(17)
#define TX_RQ_IN_INT			BIT(19)
#define RX_BQ_IN_TIMEOUT_INT		BIT(28)
#define TX_RQ_IN_TIMEOUT_INT		BIT(29)

#define DEF_INT_MASK			(RX_BQ_IN_INT | RX_BQ_IN_TIMEOUT_INT | \
					TX_RQ_IN_INT | TX_RQ_IN_TIMEOUT_INT)

/* write or read descriptor need memory barrier */
#define gmac_sync_barrier() do { isb(); smp_mb(); } while (0)

#define VENDOR_PHY_ID_FESTAV200      0x20669823
#define PHY_ID_KSZ8051MNL               0x00221550
#define PHY_ID_KSZ8081RNB               0x00221560
#define PHY_ID_UNKNOWN                  0x00221513
#define DEFAULT_PHY_MASK                0xfffffff0
#define REALTEK_PHY_ID_8211E            0x001cc915
#define REALTEK_PHY_MASK                0x001fffff

enum {
	GMAC_PORT0,
	GMAC_PORT1,
	GMAC_MAX_PORT,
};

enum {
	MEM_GMAC_IOBASE,
	MEM_MACIF_IOBASE,
	MEM_AXI_BUS_CFG_IOBASE,
	MEM_FWD_IOBASE,
	MEM_CTRL_IOBASE,
};

#define GMAC_LINKED		BIT(0)
#define GMAC_DUP_FULL		BIT(1)
#define GMAC_SPD_10M		BIT(2)
#define GMAC_SPD_100M		BIT(3)
#define GMAC_SPD_1000M	BIT(4)
/* Flow Control defines */
#define FLOW_OFF        0
#define FLOW_RX         1
#define FLOW_TX         2
#define FLOW_AUTO       (FLOW_TX | FLOW_RX)

#define RX_BQ_INT_THRESHOLD	0x40
#define TX_RQ_INT_THRESHOLD	0x20

#define GMAC_MONITOR_TIMER	(msecs_to_jiffies(200))

#define ETH_MAX_FRAME_SIZE	(1600 + 128)
#define SKB_SIZE		(ETH_MAX_FRAME_SIZE)

#define DESC_VLD_FREE		0
#define DESC_VLD_BUSY		1

#define DESC_FL_FIRST		2
#define DESC_FL_MID		0
#define DESC_FL_LAST		1
#define DESC_FL_FULL		3

#if defined(CONFIG_GMAC_DESC_4WORD)
#define DESC_WORD_SHIFT		2
#else
#define DESC_WORD_SHIFT		3
#endif
#define DESC_BYTE_SHIFT		(DESC_WORD_SHIFT + 2)
#define DESC_WORD_CNT		(1 << DESC_WORD_SHIFT)
#define DESC_SIZE		(1 << DESC_BYTE_SHIFT)

#define RX_DESC_NUM			1024
#define TX_DESC_NUM			1024

/* DMA descriptor ring helpers */
#define dma_ring_incr(n, s)		(((n) + 1) & ((s) - 1))
#define dma_cnt(n)			((n) >> DESC_BYTE_SHIFT)
#define dma_byte(n)			((n) << DESC_BYTE_SHIFT)

#define RSS_HASH_KEY_SIZE		4
#define RSS_INDIRECTION_TABLE_SIZE	128
#define RSS_NUM_RXQS		4

#define HW_CAP_TSO			BIT(0)
#define HW_CAP_RXCSUM			BIT(1)
#define HW_CAP_CCI			BIT(2)
#define has_cap_tso(hw_cap)		((hw_cap) & HW_CAP_TSO)
#define has_cap_rxcsum(hw_cap)		((hw_cap) & HW_CAP_RXCSUM)

#define GMAC_TRACE_LEVEL 10
#define GMAC_NORMAL_LEVEL 7

#define mk_bits(shift, nbits) ((((shift) & 0x1F) << 16) | ((nbits) & 0x3F))

#define FC_ACTIVE_MIN		1
#define FC_ACTIVE_DEFAULT	16
#define FC_ACTIVE_MAX		127
#define FC_DEACTIVE_MIN		1
#define FC_DEACTIVE_DEFAULT	32
#define FC_DEACTIVE_MAX		127

#define FC_PAUSE_TIME_DEFAULT		0xFFFF
#define FC_PAUSE_INTERVAL_DEFAULT	0xFFFF
#define FC_PAUSE_TIME_MAX		0xFFFF

#define HW_CAP_EN			0x0c00
#define BIT_RSS_CAP			BIT(0)
#define BIT_RXHASH_CAP			BIT(1)
#define RSS_HASH_KEY			0x0c04
#define RSS_HASH_CONFIG			0x0c08
#define TCPV4_L3_HASH_EN		BIT(0)
#define TCPV4_L4_HASH_EN		BIT(1)
#define TCPV4_VLAN_HASH_EN		BIT(2)
#define UDPV4_L3_HASH_EN		BIT(4)
#define UDPV4_L4_HASH_EN		BIT(5)
#define UDPV4_VLAN_HASH_EN		BIT(6)
#define IPV4_L3_HASH_EN			BIT(8)
#define IPV4_VLAN_HASH_EN		BIT(9)
#define TCPV6_L3_HASH_EN		BIT(12)
#define TCPV6_L4_HASH_EN		BIT(13)
#define TCPV6_VLAN_HASH_EN		BIT(14)
#define UDPV6_L3_HASH_EN		BIT(16)
#define UDPV6_L4_HASH_EN		BIT(17)
#define UDPV6_VLAN_HASH_EN		BIT(18)
#define IPV6_L3_HASH_EN			BIT(20)
#define IPV6_VLAN_HASH_EN		BIT(21)
#define DEF_HASH_CFG			0x377377

#define RGMII_SPEED_1000		0x2c
#define RGMII_SPEED_100			0x2f
#define RGMII_SPEED_10			0x2d
#define MII_SPEED_100			0x0f
#define MII_SPEED_10			0x0d
#define RMII_SPEED_100			0x8f
#define RMII_SPEED_10			0x8d
#define GMAC_FULL_DUPLEX		BIT(4)

/* tso stuff */
#define SG_FLAG		BIT(30)
#define COE_FLAG	BIT(29)
#define TSO_FLAG	BIT(28)
#define VLAN_FLAG	BIT(10)
#define IPV6_FLAG	BIT(9)
#define UDP_FLAG	BIT(8)

#define PKT_IPV6_HDR_LEN	10
#define PKT_UDP_HDR_LEN		2
#define WORD_TO_BYTE		4
enum {
	PKT_NORMAL,
	PKT_SG
};

enum {
	PKT_IPV4,
	PKT_IPV6
};

enum {
	PKT_TCP,
	PKT_UDP
};

struct frags_info {
	/* Word(2*i+2) */
	u32 addr;
	/* Word(2*i+3) */
	u32 size : 16;
	u32 reserved : 16;
};

struct sg_desc {
	/* Word0 */
	u32 total_len : 17;
	u32 reserv : 15;
	/* Word1 */
	u32 ipv6_id;
	/* Word2 */
	u32 linear_addr;
	/* Word3 */
	u32 linear_len : 16;
	u32 reserv3 : 16;
	/* MAX_SKB_FRAGS is 18 */
	struct frags_info frags[18];
};
/* tso stuff end */

#if defined(CONFIG_GMAC_DESC_4WORD)
struct gmac_desc {
	unsigned int data_buff_addr;

	unsigned int buffer_len : 11;
#if defined(CONFIG_GMAC_RXCSUM)
	unsigned int reserve2 : 1;
	unsigned int payload_csum_err : 1;
	unsigned int header_csum_err : 1;
	unsigned int payload_csum_done : 1;
	unsigned int header_csum_done : 1;
#else
	unsigned int reserve2 : 5;
#endif
	unsigned int data_len : 11;
	unsigned int reserve1 : 2;
	unsigned int fl : 2;
	unsigned int descvid : 1;

	unsigned int rxhash;
	unsigned int reserve3 : 8;
	unsigned int l3_hash : 1;
	unsigned int has_hash : 1;
	unsigned int skb_id : 14;
	unsigned int reserve31 : 8;
};

struct gmac_tso_desc {
	unsigned int data_buff_addr;
	union {
		struct {
			unsigned int prot_hdr_len : 4;
			unsigned int ip_hdr_len : 4;
			unsigned int prot_type : 1;
			unsigned int ip_ver : 1;
			unsigned int vlan_flag : 1;
			unsigned int nfrags_num : 5;
			unsigned int data_len : 11;
			unsigned int reservel : 1;
			unsigned int tso_flag : 1;
			unsigned int coe_flag : 1;
			unsigned int sg_flag : 1;
			unsigned int hw_own : 1;
		} tx;
		unsigned int val;
	} desc1;
	unsigned int reserve_desc2;
	unsigned int tx_err;
};
#else
struct gmac_desc {
	unsigned int data_buff_addr;

	unsigned int buffer_len : 11;
#if defined(CONFIG_GMAC_RXCSUM)
	unsigned int reserve2 : 1;
	unsigned int payload_csum_err : 1;
	unsigned int header_csum_err : 1;
	unsigned int payload_csum_done : 1;
#else
	unsigned int reserve2 : 5;
#endif
	unsigned int data_len : 11;
	unsigned int reserve1 : 2;
	unsigned int fl : 2;
	unsigned int descvid : 1;

	unsigned int rxhash;
	unsigned int reserve3 : 8;
	unsigned int l3_hash : 1;
	unsigned int has_hash : 1;
	unsigned int skb_id : 14;
	unsigned int reserve31 : 8;

	unsigned int reserve4;
	unsigned int reserve5;
	unsigned int reserve6;
	unsigned int reserve7;
};

struct gmac_tso_desc {
	unsigned int data_buff_addr;
	union {
		struct {
			unsigned int prot_hdr_len : 4;
			unsigned int ip_hdr_len : 4;
			unsigned int prot_type : 1;
			unsigned int ip_ver : 1;
			unsigned int vlan_flag : 1;
			unsigned int nfrags_num : 5;
			unsigned int data_len : 11;
			unsigned int reservel : 1;
			unsigned int tso_flag : 1;
			unsigned int coe_flag : 1;
			unsigned int sg_flag : 1;
			unsigned int hw_own : 1;
		} tx;
		unsigned int val;
	} desc1;
	unsigned int reserve_desc2;
	unsigned int reserve3;

	unsigned int tx_err;
	unsigned int reserve5;
	unsigned int reserve6;
	unsigned int reserve7;
};
#endif

#define SKB_MAGIC	((struct sk_buff *)0x5a)

struct gmac_napi {
	struct napi_struct napi;
	struct gmac_netdev_local *ndev_priv;
	int rxq_id;
};

struct gmac_rss_info {
	u32 hash_cfg;
	u32 ind_tbl_size;
	u8 ind_tbl[RSS_INDIRECTION_TABLE_SIZE];
	u8 key[RSS_HASH_KEY_SIZE];
};

#define QUEUE_NUMS	4
#define BASE_QUEUE_NUMS 3

struct gmac_netdev_local {
#define GMAC_SG_DESC_ADD	64U
	struct sg_desc *dma_sg_desc ____cacheline_aligned;
	dma_addr_t dma_sg_phy;
	unsigned int sg_head;
	unsigned int sg_tail;
	unsigned int sg_count;

	void __iomem *gmac_iobase;
	void __iomem *macif_base;
	void __iomem *axi_bus_cfg_base;
	int index; /* 0 -- mac0, 1 -- mac1 */

	u32 hw_cap;
	bool tso_supported;
	bool has_rxhash_cap;
	bool has_rss_cap;
	int num_rxqs;
	struct gmac_napi q_napi[RSS_NUM_RXQS];
	int irq[RSS_NUM_RXQS];
	struct gmac_rss_info rss_info;

	struct reset_control *port_rst;
	struct reset_control *macif_rst;
	struct reset_control *phy_rst;

	struct {
		struct gmac_desc *desc;
		dma_addr_t phys_addr;
		int *sg_desc_offset;
		/* how many desc in the desc pool */
		unsigned int count;
		struct sk_buff **skb;
		unsigned int size;
	} pool[QUEUE_NUMS + RSS_NUM_RXQS - 1];
#define RX_FQ		pool[0]
#define RX_BQ		pool[1]
#define TX_BQ		pool[2]
#define TX_RQ		pool[3]

	struct sk_buff **tx_skb;
	struct sk_buff **rx_skb;

	struct device *dev;
	struct net_device *netdev;
	struct clk *clk;
	struct clk *macif_clk;

	struct gmac_adapter *adapter;

	struct timer_list monitor;

	char phy_name[MII_BUS_ID_SIZE];
	struct phy_device *phy;
	struct device_node *phy_node;
	phy_interface_t phy_mode;
	bool autoeee;
	bool internal_phy;
	int (*eee_init)(struct phy_device *phy_dev);

	unsigned int flow_ctrl;
	unsigned int pause;
	unsigned int pause_interval;
	unsigned int flow_ctrl_active_threshold;
	unsigned int flow_ctrl_deactive_threshold;

	int old_link;
	int old_speed;
	int old_duplex;

	/* receive packet lock */
	spinlock_t rxlock;
	/* transmit packet lock */
	spinlock_t txlock;
	/* power management lock */
	spinlock_t pmtlock;

	int dev_state; /* INIT/OPEN/CLOSE */
	char pm_state;
	bool wol_enable;
	u32 msg_enable;
#define INIT			0 /* init gmac */
#define OPEN			1 /* power on gmac */
#define CLOSE			2 /* power off gmac */
};

enum tso_version {
	VER_NO_TSO = 0x0,
	VER_BYTE_SPLICE = 0x1,
	VER_SG_COE = 0x2,
	VER_TSO = 0x3,
};

struct cyclic_queue_info {
	u32 start;
	u32 end;
	u32 num;
	u32 pos;
};

/* ethtool ops related func */
void gmac_set_flow_ctrl_state(struct gmac_netdev_local const *ld, int pause);

/* netdev ops related func */
void gmac_hw_set_mac_addr(struct net_device *dev);
void gmac_enable_napi(struct gmac_netdev_local *priv);
void gmac_disable_napi(struct gmac_netdev_local *priv);
u32 gmac_rx_refill(struct gmac_netdev_local *priv);
void gmac_enable_rxcsum_drop(struct gmac_netdev_local const *ld, bool drop);

static inline void gmac_irq_enable(struct gmac_netdev_local const *ld)
{
	if (ld == NULL)
		return;

	writel(RX_BQ_IN_INT | RX_BQ_IN_TIMEOUT_INT
		| TX_RQ_IN_INT | TX_RQ_IN_TIMEOUT_INT,
		ld->gmac_iobase + ENA_PMU_INT);
}

static inline void gmac_irq_enable_queue(struct gmac_netdev_local *ld, unsigned int rxq_id)
{
	if (ld == NULL)
		return;

	if (rxq_id) {
		const u32 reg = rss_ena_int_queue(rxq_id);
		writel(~0, ld->gmac_iobase + reg);
	} else {
		gmac_irq_enable(ld);
	}
}

static inline void gmac_irq_enable_all_queue(struct gmac_netdev_local *ld)
{
	unsigned int i;

	if (ld == NULL)
		return;

	for (i = 0; i < (unsigned int)ld->num_rxqs; i++)
		gmac_irq_enable_queue(ld, i);
}

static inline void gmac_irq_disable(struct gmac_netdev_local const *ld)
{
	if (ld == NULL)
		return;

	writel(0, ld->gmac_iobase + ENA_PMU_INT);
}

static inline void gmac_irq_disable_queue(struct gmac_netdev_local const *ld,
		  int rxq_id)
{
	if (ld == NULL)
		return;

	if (rxq_id) {
		u32 reg = (u32)rss_ena_int_queue(rxq_id);
		writel(0, ld->gmac_iobase + reg);
	} else {
		gmac_irq_disable(ld);
	}
}

static inline void gmac_irq_disable_all_queue(struct gmac_netdev_local const *ld)
{
	int i;

	if (ld == NULL)
		return;

	for (i = 0; i < ld->num_rxqs; i++)
		gmac_irq_disable_queue(ld, i);
}

static inline bool gmac_queue_irq_disabled(struct gmac_netdev_local *ld,
				      int rxq_id)
{
	u32 reg, val;

	if (ld == NULL)
		return false;

	if (rxq_id)
		reg = (u32)rss_ena_int_queue(rxq_id);
	else
		reg = ENA_PMU_INT;
	val = readl(ld->gmac_iobase + reg);

	return !val;
}

static inline void gmac_hw_desc_enable(struct gmac_netdev_local const *ld)
{
	if (ld == NULL)
		return;
	writel(0xF, ld->gmac_iobase + DESC_WR_RD_ENA);
}

static inline void gmac_hw_desc_disable(struct gmac_netdev_local const *ld)
{
	if (ld == NULL)
		return;
	writel(0, ld->gmac_iobase + DESC_WR_RD_ENA);
}

static inline void gmac_port_enable(struct gmac_netdev_local const *ld)
{
	if (ld == NULL)
		return;
	writel(BITS_TX_EN | BITS_RX_EN, ld->gmac_iobase + PORT_EN);
}

static inline void gmac_port_disable(struct gmac_netdev_local const *ld)
{
	if (ld != NULL)
		writel(0, ld->gmac_iobase + PORT_EN);
}

#endif
