#ifndef _REALTEK_PTP_H_
#define _REALTEK_PTP_H_

#define RTL8211F_A43_PAGE               0xa43
#define RTL8211F_PHYCR2                 0x19
#define RTL8211F_PAGSR                  0x1f

#define RTL8211F_E40_PAGE               0xe40

#define RTL8211F_PTP_CTL                0x10

#define RTL8211F_UDP_CHKSUM_UPDATE      BIT(12)
#define RTL8211F_P_DRFU_2STEP_INS       BIT(11)
#define RTL8211F_P_DR_2STEP_INS         BIT(10)
#define RTL8211F_DR_2STEP_INS           BIT(9)
#define RTL8211F_FU_2STEP_INS           BIT(8)
#define RTL8211F_P_DR_1STEP             BIT(7)
#define RTL8211F_SYNC_1STEP             BIT(6)
#define RTL8211F_AVB_802_1AS            BIT(5)
#define RTL8211F_PTPV2_LAYER2           BIT(4)
#define RTL8211F_PTPV2_UDPIPV4          BIT(3)
#define RTL8211F_PTPV2_UDPIPV6          BIT(2)
#define RTL8211F_PTPV1                  BIT(1)
#define RTL8211F_PTP_ENABLE             BIT(0)


#define RTL8211F_PTP_INER               0x11
#define RTL8211F_PTP_INSR               0x12

#define RTL8211F_TX_TIMESTAMP           BIT(3)
#define RTL8211F_RX_TIMESTAMP           BIT(2)
#define RTL8211F_TRIGGER_GEN            BIT(1)
#define RTL8211F_EVENT_CAPTURE          BIT(0)

#define RTL8211F_SYNCE_CTL              0x13

#define RTL8211F_SYNC_E_ENABLE          BIT(0)


#define RTL8211F_E41_PAGE               0xe41
#define RTL8211F_PTP_CLK_CFG            0x10
#define RTL8211F_PTP_CFG_NS_LO          0x11
#define RTL8211F_PTP_CFG_NS_HI          0x12
#define RTL8211F_PTP_CFG_S_LO           0x13
#define RTL8211F_PTP_CFG_S_MI           0x14
#define RTL8211F_PTP_CFG_S_HI           0x15

#define RTL8211F_E42_PAGE               0xe42
#define RTL8211F_PTP_TAI_CFG            0x10
#define RTL8211F_PTP_TRIG_CFG           0x11
#define RTL8211F_PTP_TAI_STA            0x12
#define RTL8211F_PTP_TAI_TS_NS_LO       0x13
#define RTL8211F_PTP_TAI_TS_NS_HI       0x14
#define RTL8211F_PTP_TAI_TS_S_LO        0x15
#define RTL8211F_PTP_TAI_TS_S_HI        0x16

#define RTL8211F_E43_PAGE               0xe43

#define RTL8211F_PTP_TRX_TS_STA         0x10

#define RTL8211F_TXTS_SYNC_RDY          BIT(15)
#define RTL8211F_TXTS_DELAY_REQ_RDY     BIT(14)
#define RTL8211F_TXTS_PDELAY_REQ_RDY    BIT(13)
#define RTL8211F_TXTS_PDELAY_RSP_RDY    BIT(12)
#define RTL8211F_RXTS_SYNC_RDY          BIT(11)
#define RTL8211F_RXTS_DELAY_REQ_RDY     BIT(10)
#define RTL8211F_RXTS_PDELAY_REQ_RDY    BIT(9)
#define RTL8211F_RXTS_PDELAY_RSP_RDY    BIT(8)
#define RTL8211F_TRXTS_OVERWR_EN        BIT(4)

#define RTL8211F_TRXTS_MSGTYPE_SEL      BIT(3) | BIT(2)

#define RTL8211F_TRXTS_SYNC             0
#define RTL8211F_TRXTS_DELAY_REQ        1
#define RTL8211F_TRXTS_PDELAY_REQ       2
#define RTL8211F_TRXTS_PDELAY_RSP       3

#define RTL8211F_TRXTS_SEL              BIT(1)

#define RTL8211F_TRXTS_TX               0
#define RTL8211F_TRXTS_RX               1

#define RTL8211F_TRXTS_RD               BIT(0)


#define RTL8211F_E44_PAGE               0xe44
#define RTL8211F_PTP_TRX_TS_INFO        0x10
#define RTL8211F_PTP_TRX_TS_SH          0x11
#define RTL8211F_PTP_TRX_TS_SID         0x12
#define RTL8211F_PTP_TRX_TS_NS_LO       0x13
#define RTL8211F_PTP_TRX_TS_NS_HI       0x14
#define RTL8211F_PTP_TRX_TS_S_LO        0x15
#define RTL8211F_PTP_TRX_TS_S_MI        0x16
#define RTL8211F_PTP_TRX_TS_S_HI        0x17

#define GPIO_NUM_BANK       8
#define BANK2PIN(bank, pin) \
    ((bank) * GPIO_NUM_BANK + (pin))

#define GPIO7_BASE_PHYADDR              0x11097000

enum rtl8211f_ptp_clock_adj_mode {
    RTL8211F_NO_FUNCTION = 0,
    RTL8211F_RESERVED,
    RTL8211F_DIRECT_READ,
    RTL8211F_DIRECT_WRITE,
    RTL8211F_INCREMENT_STEP,
    RTL8211F_DECREMENT_STEP,
    RTL8211F_RATE_READ,
    RTL8211F_RATE_WRITE,
};

struct rtl8211f_ptphdr {
	u8 tsmt; /* transportSpecific | messageType */
	u8 ver;  /* reserved0 | versionPTP */
	__be16 msglen;
	u8 domain;
	u8 rsrvd1;
	__be16 flags;
	__be64 correction;
	__be32 rsrvd2;
	__be64 clk_identity;
	__be16 src_port_id;
	__be16 seq_id;
	u8 ctrl;
	u8 log_interval;
} __attribute__((__packed__));

struct rtl8211f_ptp {
	struct phy_device *phydev;
	struct ptp_clock *ptp_clock;
	struct ptp_clock_info caps;

    int raw_pin;
    int irq;
    
	struct sk_buff_head tx_queue;
    spinlock_t tx_queue_lock;
    struct sk_buff_head rx_queue;
    spinlock_t rx_queue_lock;

    wait_queue_head_t rx_wq;
    
	enum hwtstamp_tx_types tx_type;
	enum hwtstamp_rx_filters rx_filter;

    struct mutex ts_mutex;

    void __iomem *gpio_base;
    int gpio_level;
    
	u8 configured:1;

    struct task_struct *gpio_event_thread;
    struct completion complete;
};

#define SYNC                            0x0
#define DELAY_REQ                       0x1
#define PDELAY_REQ                      0x2
#define PDELAY_RESP                     0x3


int rtl8211f_gettime(struct ptp_clock_info *info, struct timespec64 *ts);
int rtl8211f_settime(struct ptp_clock_info *info, const struct timespec64 *ts);


#endif  /* _REALTEK_PHY_H_ */
