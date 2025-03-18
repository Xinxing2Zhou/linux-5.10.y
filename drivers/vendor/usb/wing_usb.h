/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#ifndef _WING_USB_H_
#define _WING_USB_H_

#include <linux/completion.h>
#include <linux/dcache.h>
#include <linux/kfifo.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeup.h>
#include <linux/regulator/consumer.h>
#include <linux/phy/phy.h>
#include <linux/usb.h>
#include <linux/extcon.h>
#include <linux/clk.h>

#define WING_USB_DEBUG 0

#define wing_usb_dbg(format, arg...) \
	do { \
		if (WING_USB_DEBUG) \
			printk(KERN_INFO "[WING-USB][%s]"format, __func__, ##arg); \
	} while (0)

#define wing_usb_info(format, arg...) \
	printk(KERN_INFO "[WING-USB][%s]"format, __func__, ##arg)

#define wing_usb_err(format, arg...) \
	printk(KERN_ERR "[WING-USB][%s]"format, __func__, ##arg)

#define MAX_WING_USB_EVENT_COUNT 16

#define LCSR_TX_DEEMPH_ADDR      0xD060
#define LCSR_TX_DEEMPH_CP13_ADDR 0xD064
#define LCSR_TX_DEEMPH_CP14_ADDR 0xD068
#define GEN2_TX_DEEMPH_VAL       0x14FC0
#define LCSR_TX_DEEMPH_CP13_VAL  0xFC0
#define LCSR_TX_DEEMPH_CP14_VAL  0x4FC5

#define REG_GUSB3PIPECTL0  0xc2c0
#define SUSPENDENABLE      BIT(17)
#define PCS_SSP_SOFT_RESET BIT(31)

#define GCTL             0xc110
#define PRTCAPDIR_HOST   BIT(12)
#define PRTCAPDIR_DEVICE BIT(13)
#define PRTCAPDIR_MASK   (3U << 12)
#define PWRDNSCALE_MASK  (0x1fffU << 19)
#define PWRDNSCALE_VAL   (0x3fU << 19)

#define GTXTHRCFG 0xc108
#define USB_TX_PKT_CNT_SEL    BIT(29)
#define USB_TX_PKT_CNT_MASK   (0xfU << 24)
#define USB_TX_PKT_CNT        (0x3U << 24)
#define USB_MAX_TX_BURST_SIZE_MASK (0xffU << 16)
#define USB_MAX_TX_BURST_SIZE      (0x10U << 16)

#define GRXTHRCFG             0xc10c
#define USB_RX_PKT_CNT_SEL    BIT(29)
#define USB_RX_PKT_CNT_MASK   (0xfU << 24)
#define USB_RX_PKT_CNT        (0x3U << 24)
#define USB_MAX_RX_BURST_SIZE_MASK (0xffU << 16)
#define USB_MAX_RX_BURST_SIZE      (0x10U << 16)

#define GUSB2PHYCFG0  0xc200
#define ULPI_UTMI_SEL (1U << 4)
#define SUSPENDUSB20  (1U << 6)

enum wing_usb_state {
	WING_USB_STATE_UNKNOWN = 0,
	WING_USB_STATE_OFF,
	WING_USB_STATE_HOST,
	WING_USB_STATE_DEVICE,
};

enum wing_usb_event_type {
	SWITCH_TO_HOST = 0,
	SWITCH_TO_DEVICE,
	NONE_EVENT,
};

#define SIZE_WING_USB_EVENT 32

/* size of struct wing_usb_event must be a power of 2 for kfifo */
struct wing_usb_event {
	enum wing_usb_event_type type;
	int ctrl_id;
#ifdef CONFIG_64BIT
	u32 reserved; /* to keep struct size is 32 bytes in 64bit system */
#else
	u32 reserved[3]; /* to keep struct size is 32 bytes in 32bit system */
#endif
	u32 flags;
	void (*callback)(struct wing_usb_event *event);
	void *content;
};

struct wing_usb {
	struct platform_device *pdev;
	struct extcon_dev *edev;
	struct platform_device *dwc3_dev;

	int id;
	bool support_drd;
	bool is_cur_host;
	bool deempth;
	bool disable_suspend;
	bool powerdown_scale;
	bool is_usb2;
	struct list_head list;
	u32 tx_thrcfg;
	u32 rx_thrcfg;

	enum wing_usb_state state;
	DECLARE_KFIFO(event_fifo, struct wing_usb_event,
		      MAX_WING_USB_EVENT_COUNT);

	spinlock_t event_lock;
	struct work_struct event_work;
	struct phy *usb2_phy;
	struct phy *usb3_phy;
	struct clk *ctrl_clk;

	struct mutex lock;

	void __iomem *ctrl_base;
	struct proc_dir_entry *proc_entry;
};

/*
 * The event will be added to tail of a queue, and processed in a work.
 * Return 0 means the event added sucessfully, others means event was rejected.
 */
int wing_usb_queue_event(const struct wing_usb_event *usb_event,
	struct wing_usb *wusb);

int wing_usb_otg_event(enum wing_usb_event_type type, int controller_id);

#endif /* _WING_USB_H_ */
