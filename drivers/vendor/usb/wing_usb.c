/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/pm_runtime.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/io.h>
#include <linux/version.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 16, 0))
#include <linux/usb/role.h>
#endif
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0))
#include <linux/extcon-provider.h>
#endif
#include <core.h>
#include <linux/usb/ch9.h>

#include "proc.h"
#include "wing_usb.h"

static LIST_HEAD(wing_usb_drd_dev_list);

static const unsigned int wing_usb_extcon_cable[] = {
	EXTCON_USB,
	EXTCON_USB_HOST,
	EXTCON_NONE
};

static const char *wing_usb_event_type_string(
	enum wing_usb_event_type event)
{
	static const char * const wing_usb_event_strings[] = {
		[SWITCH_TO_HOST]			= "SWITCH_TO_HOST",
		[SWITCH_TO_DEVICE]			= "SWITCH_TO_DEVICE",
		[NONE_EVENT]				= "NONE_EVENT",
	};

	if (event > NONE_EVENT)
		return "illegal event";

	return wing_usb_event_strings[event];
}

static int __maybe_unused wing_usb_event_enqueue(struct wing_usb *wusb,
	const struct wing_usb_event *event)
{
	if (event->ctrl_id != wusb->id) {
		wing_usb_info("event doesn't belong to this controller, event->ctrl_id = %d\n",
			event->ctrl_id);
	}

	if (kfifo_in(&wusb->event_fifo, event, 1) == 0) {
		wing_usb_err("drop event %s\n",
			wing_usb_event_type_string(event->type));
		return -ENOSPC;
	}

	return 0;
}

/*
 * get event frome event_queue
 * return the numbers of event dequeued, currently it is 1
 */
static int wing_usb_event_dequeue(struct wing_usb *wusb,
	struct wing_usb_event *event)
{
	return kfifo_out_spinlocked(&wusb->event_fifo, event, 1,
		&wusb->event_lock);
}

static int wing_usb_remove_child(struct device *dev, void __maybe_unused *data)
{
	int ret;

	ret = of_platform_device_destroy(dev, NULL);
	if (ret != 0) {
		wing_usb_err("device destroy error (ret %d)\n", ret);
		return ret;
	}

	return 0;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
static struct dwc3 *wing_usb_get_role_sw(const struct wing_usb *wusb)
{
	struct dwc3 *dwc = NULL;
	struct dwc3 *dwc_role_sw = NULL;

	if (wusb->dwc3_dev == NULL) {
		wing_usb_err("no dwc platform device found\n");
		return NULL;
	}

	dwc = platform_get_drvdata(wusb->dwc3_dev);
	if (dwc == NULL) {
		wing_usb_err("no dwc driver data found\n");
		return NULL;
	}

	if (dwc->role_sw == NULL) {
		wing_usb_err("no dwc_role_sw device found\n");
		return NULL;
	}

	dwc_role_sw = (struct dwc3 *)usb_role_switch_get_drvdata(dwc->role_sw);
	if (dwc_role_sw == NULL) {
		wing_usb_err("no dwc_role_sw driver data found\n");
		return NULL;
	}

	return dwc_role_sw;
}
#endif

static int wing_usb_start_device(const struct wing_usb *wusb)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
	struct dwc3 *dwc = NULL;
#endif
	int ret = extcon_set_state_sync(wusb->edev, EXTCON_USB, true);
	if (ret) {
		wing_usb_err("extcon start peripheral error\n");
		return ret;
	}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
	dwc = wing_usb_get_role_sw(wusb);
	if (dwc != NULL) {
		if (dwc->role_sw != NULL) {
			ret = usb_role_switch_set_role(dwc->role_sw, USB_ROLE_DEVICE);
		}
	}
#endif

	wing_usb_dbg("wing usb status: OFF -> DEVICE\n");

	return ret;
}

static int wing_usb_stop_device(const struct wing_usb *wusb)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
	struct dwc3 *dwc = NULL;
#endif

	int ret = extcon_set_state_sync(wusb->edev, EXTCON_USB, false);
	if (ret) {
		wing_usb_err("extcon stop peripheral error\n");
		return ret;
	}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
	dwc = wing_usb_get_role_sw(wusb);
	if (dwc != NULL) {
		if (dwc->role_sw != NULL) {
			ret = usb_role_switch_set_role(dwc->role_sw, USB_ROLE_NONE);
		}
	}
#endif

	wing_usb_dbg("wing usb status: DEVICE -> OFF\n");

	return ret;
}

static int wing_usb_start_host(const struct wing_usb *wusb)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
	struct dwc3 *dwc = NULL;
#endif
	int ret = extcon_set_state_sync(wusb->edev, EXTCON_USB_HOST, true);
	if (ret) {
		wing_usb_err("extcon start host error\n");
	}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
	dwc = wing_usb_get_role_sw(wusb);
	if (dwc != NULL) {
		if (dwc->role_sw != NULL) {
			ret = usb_role_switch_set_role(dwc->role_sw, USB_ROLE_HOST);
		}
	}
#endif

	wing_usb_dbg("wing usb status: OFF -> HOST\n");

	return ret;
}

static int wing_usb_stop_host(const struct wing_usb *wusb)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
	struct dwc3 *dwc = NULL;
#endif
	int ret = extcon_set_state_sync(wusb->edev, EXTCON_USB_HOST, false);
	if (ret) {
		wing_usb_err("extcon stop host error\n");
		return ret;
	}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
	dwc = wing_usb_get_role_sw(wusb);
	if (dwc != NULL) {
		if (dwc->role_sw != NULL) {
			ret = usb_role_switch_set_role(dwc->role_sw, USB_ROLE_HOST);
		}
	}
#endif

	wing_usb_dbg("wing usb status: HOST -> OFF\n");

	return ret;
}

static int wing_usb_switch_to_host(struct wing_usb *wusb)
{
	int ret;

	if (wusb == NULL) {
		return -EINVAL;
	}

	if (wusb->state == WING_USB_STATE_HOST) {
		return 0;
	}

	ret = wing_usb_stop_device(wusb);
	if (ret != 0) {
		wing_usb_err("stop device failed\n");
		return ret;
	}

	ret = wing_usb_start_host(wusb);
	if (ret != 0) {
		wing_usb_err("start host failed\n");
		return ret;
	}

	wusb->state = WING_USB_STATE_HOST;

	return 0;
}

static int wing_usb_switch_to_device(struct wing_usb *wusb)
{
	int ret;

	if (wusb == NULL) {
		return -EINVAL;
	}

	if (wusb->state == WING_USB_STATE_DEVICE) {
		return 0;
	}

	ret = wing_usb_stop_host(wusb);
	if (ret != 0) {
		wing_usb_err("stop host failed\n");
		return ret;
	}

	ret = wing_usb_start_device(wusb);
	if (ret != 0) {
		wing_usb_err("start device failed\n");
		return ret;
	}

	wusb->state = WING_USB_STATE_DEVICE;

	return 0;
}

static void wing_usb_handle_event(struct wing_usb *wusb,
	enum wing_usb_event_type event_type)
{
	wing_usb_dbg("type: %s\n", wing_usb_event_type_string(event_type));

	switch (event_type) {
	case SWITCH_TO_HOST:
		wing_usb_switch_to_host(wusb);
		break;

	case SWITCH_TO_DEVICE:
		wing_usb_switch_to_device(wusb);
		break;

	default:
		wing_usb_dbg("illegal event type!\n");
		break;
	}
}

static void wing_usb_event_work(struct work_struct *work)
{
	struct wing_usb_event event = {0};

	struct wing_usb *wusb = container_of(work, struct wing_usb, event_work);

	wing_usb_dbg("+\n");
	mutex_lock(&wusb->lock);

	while (wing_usb_event_dequeue(wusb, &event)) {
		wing_usb_handle_event(wusb, event.type);
	}

	mutex_unlock(&wusb->lock);

	wing_usb_dbg("-\n");
}

/*
 * return 0 means event was accepted, others means event was rejected.
 */
int wing_usb_queue_event(const struct wing_usb_event *usb_event,
	struct wing_usb *wusb)
{
#if IS_ENABLED(CONFIG_USB_DWC3_DUAL_ROLE)
	unsigned long flags;
	enum wing_usb_event_type event_type;
	int controller_id;

	if (usb_event == NULL)
		return -EINVAL;

	if (wusb == NULL)
		return -ENODEV;

	spin_lock_irqsave(&(wusb->event_lock), flags);

	event_type = usb_event->type;
	controller_id = usb_event->ctrl_id;
	wing_usb_dbg("event: %s, controller id: %d\n",
		wing_usb_event_type_string(event_type), controller_id);

	if (wing_usb_event_enqueue(wusb, usb_event)) {
		wing_usb_err("can't enqueue event:%d\n", event_type);
		spin_unlock_irqrestore(&(wusb->event_lock), flags);
		return -EBUSY;
	}

	schedule_work(&wusb->event_work);

	spin_unlock_irqrestore(&(wusb->event_lock), flags);

	return 0;
#else
	return 0;
#endif
}

int wing_usb_otg_event(enum wing_usb_event_type type, int controller_id)
{
	struct wing_usb_event event = {0};
	struct wing_usb *wusb = NULL;
	struct wing_usb *cur_wusb, *next_wusb;

	event.type = type;
	event.ctrl_id = controller_id;

	list_for_each_entry_safe(cur_wusb, next_wusb,
		&wing_usb_drd_dev_list, list) {
		wing_usb_info("cur_wusb->id = %d\n", cur_wusb->id);
		if (cur_wusb->id == controller_id) {
			wusb = cur_wusb;
			break;
		}
	}

	if (wusb == NULL) {
		wing_usb_err("find wing_usb for controler_id = %d failed\n",
			controller_id);
		return -1;
	}

	return wing_usb_queue_event(&event, wusb);
}
EXPORT_SYMBOL(wing_usb_otg_event);

static int wing_usb_get_resource(struct device *dev, struct wing_usb *wusb)
{
	int ret;

	wusb->support_drd = of_property_read_bool(dev->of_node, "support-drd");

	wusb->is_cur_host = of_property_read_bool(dev->of_node, "host-mode");

	wusb->deempth = of_property_read_bool(dev->of_node, "deempth");

	/* es chip disable suspend, cs not need, but you can disable it when you want */
	wusb->disable_suspend = of_property_read_bool(dev->of_node, "disable-suspend");

	/* USB31 need configure this */
	wusb->powerdown_scale = of_property_read_bool(dev->of_node, "powerdown-scale");

	wusb->is_usb2 = of_property_read_bool(dev->of_node, "is-usb2");

	wusb->ctrl_base = of_iomap(dev->of_node, 0);
	if (IS_ERR(wusb->ctrl_base)) {
		wing_usb_err("alloc ctrl_base failed\n");
		return -1;
	}

	wusb->usb2_phy = devm_phy_get(dev, "usb2-phy");
	if (IS_ERR(wusb->usb2_phy)) {
		wing_usb_err("get u2phy failed\n");
		return -1;
	}

	wusb->usb3_phy = devm_phy_get(dev, "usb3-phy");
	/* at least one of usb2phy and usb3phy */
	if (!wusb->is_usb2 && IS_ERR(wusb->usb3_phy)) {
		wing_usb_err("get u3phy failed\n");
		return -1;
	}

	wusb->ctrl_clk = devm_clk_get(dev, "ctrl-clk");
	if (IS_ERR(wusb->ctrl_clk)) {
		wing_usb_err("get ctrl clk failed\n");
		return -1;
	}

	ret = of_property_read_u32(dev->of_node, "tx-thrcfg", &wusb->tx_thrcfg);
	if (ret)
		wusb->tx_thrcfg = 0;

	ret = of_property_read_u32(dev->of_node, "rx-thrcfg", &wusb->rx_thrcfg);
	if (ret)
		wusb->rx_thrcfg = 0;

	return 0;
}

static void wing_usb_set_tx_deemph(const struct wing_usb *wusb)
{
	/* Set  X1 Gen2 TX de-emp value for normal use, CP13 & CP14 */
	writel(GEN2_TX_DEEMPH_VAL, wusb->ctrl_base + LCSR_TX_DEEMPH_ADDR);
	writel(LCSR_TX_DEEMPH_CP13_VAL, wusb->ctrl_base + LCSR_TX_DEEMPH_CP13_ADDR);
	writel(LCSR_TX_DEEMPH_CP14_VAL, wusb->ctrl_base + LCSR_TX_DEEMPH_CP14_ADDR);
}

static void wing_usb_disable_suspend(const struct wing_usb *wusb)
{
	u32 reg;

	reg = readl(wusb->ctrl_base + REG_GUSB3PIPECTL0);
	reg &= ~(SUSPENDENABLE);
	writel(reg, wusb->ctrl_base + REG_GUSB3PIPECTL0);
}

static void wing_usb_set_powerdown_scale(const struct wing_usb *wusb)
{
	u32 reg;

	reg = readl(wusb->ctrl_base + GCTL);
	reg &= ~(PWRDNSCALE_MASK);
	reg |= PWRDNSCALE_VAL;
	writel(reg, wusb->ctrl_base + GCTL);
}

static void wing_usb_set_mode(const struct wing_usb *wusb)
{
	u32 val;

	val = readl(wusb->ctrl_base + GCTL);
	val &= ~(PRTCAPDIR_MASK);
	if (wusb->is_cur_host) {
		val |= (PRTCAPDIR_HOST);
	} else {
		val |= (PRTCAPDIR_DEVICE);
	}
	writel(val, wusb->ctrl_base + GCTL);
}

static void wing_usb_save_mode(struct wing_usb *wusb)
{
	u32 val;

	val = readl(wusb->ctrl_base + GCTL);
	val &= PRTCAPDIR_MASK;
	if (val == PRTCAPDIR_HOST) {
		wusb->is_cur_host = true;
	} else {
		wusb->is_cur_host = false;
	}
}

static void config_tx_thrcfg(const struct wing_usb *wusb)
{
	u32 val;

	if (wusb->tx_thrcfg != 0) {
		val = readl(wusb->ctrl_base + GTXTHRCFG);
		val &= ~(USB_TX_PKT_CNT_MASK);
		val |= (wusb->tx_thrcfg & USB_TX_PKT_CNT_MASK);
		val &= ~(USB_MAX_TX_BURST_SIZE_MASK);
		val |= (wusb->tx_thrcfg & USB_MAX_TX_BURST_SIZE_MASK);
		val |= USB_TX_PKT_CNT_SEL;
		writel(val, wusb->ctrl_base + GTXTHRCFG);
	}
}

static void config_rx_thrcfg(const struct wing_usb *wusb)
{
	u32 val;

	if (wusb->rx_thrcfg != 0) {
		val = readl(wusb->ctrl_base + GRXTHRCFG);
		val &= ~(USB_RX_PKT_CNT_MASK);
		val |= (wusb->rx_thrcfg & USB_RX_PKT_CNT_MASK);
		val &= ~(USB_MAX_RX_BURST_SIZE_MASK);
		val |= (wusb->rx_thrcfg & USB_MAX_RX_BURST_SIZE_MASK);
		val |= USB_RX_PKT_CNT_SEL;
		writel(val, wusb->ctrl_base + GRXTHRCFG);
	}
}

static void wing_usb_usb20_config(const struct wing_usb *wusb)
{
	u32 reg;

	reg = readl(wusb->ctrl_base + REG_GUSB3PIPECTL0);
	reg |= PCS_SSP_SOFT_RESET;
	writel(reg, wusb->ctrl_base + REG_GUSB3PIPECTL0);

	if (wusb->support_drd) {
		/* For DRD mode, it is recommanded set SUSPENDUSB20 to 0 */
		reg = readl(wusb->ctrl_base + GUSB2PHYCFG0);
		reg &= ~(SUSPENDUSB20);
		writel(reg, wusb->ctrl_base + GUSB2PHYCFG0);
	}

	reg = readl(wusb->ctrl_base + REG_GUSB3PIPECTL0);
	reg &= ~(SUSPENDENABLE | PCS_SSP_SOFT_RESET);
	writel(reg, wusb->ctrl_base + REG_GUSB3PIPECTL0);
}

static void wing_usb_feature_config(struct wing_usb *wusb)
{
	if (wusb->deempth) {
		wing_usb_set_tx_deemph(wusb);
	}

	if (wusb->disable_suspend) {
		wing_usb_disable_suspend(wusb);
	}

	if (wusb->powerdown_scale) {
		wing_usb_set_powerdown_scale(wusb);
	}

	if (wusb->is_usb2) {
		wing_usb_usb20_config(wusb);
	}

	config_tx_thrcfg(wusb);
	config_rx_thrcfg(wusb);

	wing_usb_set_mode(wusb);
}

static void wing_usb_drd_initialize(struct device *dev, struct wing_usb *wusb)
{
	int ret;

	ret = of_property_read_s32(dev->of_node, "controller_id", &wusb->id);
	if (ret) {
		wing_usb_info("cannot read controller_id: %d\n", ret);
		wusb->id = -1;
	}

	INIT_KFIFO(wusb->event_fifo);
	spin_lock_init(&wusb->event_lock);
	INIT_WORK(&wusb->event_work, wing_usb_event_work);
	mutex_init(&wusb->lock);
}

static int wing_usb_drd_init_state(struct device *dev, struct wing_usb *wusb)
{
	struct wing_usb_event usb_event = {0};
	const char *buf = NULL;
	int ret;

	wing_usb_dbg("+\n");

	wusb->state = WING_USB_STATE_UNKNOWN;

	ret = of_property_read_string(dev->of_node, "init_mode", &buf);
	if (ret) {
		wing_usb_info("cannot read init mode: %d, set device mode\n", ret);
		usb_event.type = SWITCH_TO_DEVICE;
	} else {
		wing_usb_dbg("init state: %s\n", buf);

		if (!strncmp(buf, "host", 4)) { /* host len is 4 */
			usb_event.type = SWITCH_TO_HOST;
		} else {
			usb_event.type = SWITCH_TO_DEVICE;
		}
	}

	ret = wing_usb_queue_event(&usb_event, wusb);
	if (ret)
		wing_usb_err("usb_queue_event err: %d\n", ret);

	wing_usb_dbg("-\n");

	return 0;
}

static int wing_usb_controller_probe(struct device *dev, struct wing_usb *wusb)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
	struct device_node *child_node = NULL;
	int ret;

	ret = of_platform_populate(dev->of_node, NULL, NULL, dev);
	if (ret == 0) {
		child_node = of_get_next_available_child(dev->of_node, NULL);
		if (child_node) {
			wusb->dwc3_dev = of_find_device_by_node(child_node);
		}
	}
	return ret;
#else
	return of_platform_populate(dev->of_node, NULL, NULL, dev);
#endif
}

static int wing_usb_pm_runtime_enable(const struct wing_usb *wusb)
{
	struct platform_device *pdev = wusb->pdev;
	struct device *dev = &pdev->dev;
	int ret;

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		wing_usb_err("pm_runtime_get_sync failed %d\n", ret);
		return ret;
	}

	pm_runtime_forbid(dev);

	return ret;
}

static int wing_usb_extcon_init(struct device *dev, struct wing_usb *wusb)
{
	int ret;

	wusb->edev = devm_extcon_dev_allocate(dev, wing_usb_extcon_cable);
	if (IS_ERR(wusb->edev)) {
		dev_err(dev, "failed to allocate extcon device\n");
		return PTR_ERR(wusb->edev);
	}

	ret = devm_extcon_dev_register(dev, wusb->edev);
	if (ret < 0) {
		dev_err(dev, "failed to register extcon device\n");
		return ret;
	}

	return 0;
}

static int wing_usb_drd_init(struct device *dev, struct wing_usb *wusb)
{
	int ret;

	ret = wing_usb_extcon_init(dev, wusb);
	if (ret < 0) {
		dev_err(dev, "failed to register extcon device\n");
		return -1;
	}

	ret = wing_usb_create_proc_entry(dev, wusb);
	if (ret) {
		dev_err(dev, "create proc entry failed!\n");
		goto err_extcon_free;
	}

	wing_usb_drd_initialize(dev, wusb);

	ret = wing_usb_pm_runtime_enable(wusb);
	if (ret < 0)
		goto err_remove_attr;

	ret = wing_usb_controller_probe(dev, wusb);
	if (ret) {
		wing_usb_err("register controller failed %d!\n", ret);
		goto err_pm_put;
	}

	ret = wing_usb_drd_init_state(dev, wusb);
	if (ret) {
		wing_usb_err("wing_usb_init_state failed!\n");
		goto err_remove_child;
	}

	list_add_tail(&wusb->list, &wing_usb_drd_dev_list);

	pm_runtime_allow(dev);
	wing_usb_dbg("-\n");

	return 0;

err_remove_child:
	device_for_each_child(dev, NULL, wing_usb_remove_child);

err_pm_put:
	pm_runtime_put_sync(dev);
	pm_runtime_disable(dev);

err_remove_attr:
	wing_usb_remove_proc_entry(dev, wusb);

err_extcon_free:
	devm_extcon_dev_unregister(dev, wusb->edev);
	extcon_dev_free(wusb->edev);
	wusb->edev = NULL;

	return ret;
}

static int wing_usb_host_init(struct device *dev, struct wing_usb *wusb)
{
	int ret;

	/* enable runtime pm. */
	ret = wing_usb_pm_runtime_enable(wusb);
	if (ret < 0) {
		wing_usb_err("anble pm runtime failed\n");
		return ret;
	}

	ret = wing_usb_controller_probe(dev, wusb);
	if (ret) {
		wing_usb_err("register controller failed %d!\n", ret);
		goto err_pm_put;
	}

	pm_runtime_allow(dev);

	return 0;

err_pm_put:
	pm_runtime_put_sync(dev);
	pm_runtime_disable(dev);

	return ret;
}

static int wing_usb_clk_phy_init(const struct wing_usb *wusb)
{
	int ret = 0;

	ret = clk_prepare(wusb->ctrl_clk);
	if (ret != 0) {
		wing_usb_err("ctrl clk prepare failed\n");
		return -1;
	}

	if (!IS_ERR(wusb->usb2_phy)) {
		ret = phy_power_on(wusb->usb2_phy);
		if (ret != 0) {
			wing_usb_err("usb2 phy init failed\n");
			return -1;
		}
	}

	if (!IS_ERR(wusb->usb3_phy)) {
		ret = phy_power_on(wusb->usb3_phy);
		if (ret != 0) {
			wing_usb_err("usb3 phy init failed\n");
			return -1;
		}
	}

	ret = clk_enable(wusb->ctrl_clk);
	if (ret != 0) {
		wing_usb_err("ctrl clk enable failed\n");
		return -1;
	}

	return 0;
}

static int wing_usb_clk_phy_deinit(const struct wing_usb *wusb)
{
	int ret = 0;

	clk_disable(wusb->ctrl_clk);

	if (!IS_ERR(wusb->usb3_phy)) {
		ret = phy_power_off(wusb->usb3_phy);
		if (ret != 0) {
			wing_usb_err("usb3 phy deinit failed\n");
			return -1;
		}
	}

	if (!IS_ERR(wusb->usb2_phy)) {
		ret = phy_power_off(wusb->usb2_phy);
		if (ret != 0) {
			wing_usb_err("usb2 phy deinit failed\n");
			return -1;
		}
	}

	clk_unprepare(wusb->ctrl_clk);

	return 0;
}

static int wing_usb_probe(struct platform_device *pdev)
{
	int ret;
	struct wing_usb *wusb = NULL;
	struct device *dev = &pdev->dev;

	wing_usb_dbg("+++\n");

	BUILD_BUG_ON(sizeof(struct wing_usb_event) != SIZE_WING_USB_EVENT);

	wusb = devm_kzalloc(dev, sizeof(*wusb), GFP_KERNEL);
	if (wusb == NULL) {
		wing_usb_err("alloc wusb failed\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, wusb);
	wusb->pdev = pdev;

	ret = wing_usb_get_resource(dev, wusb);
	if (ret < 0) {
		devm_kfree(dev, wusb);
		wusb = NULL;
		return -1;
	}

	ret = wing_usb_clk_phy_init(wusb);
	if (ret != 0) {
		wing_usb_err("init phy failed\n");
		goto err_unmap;
	}

	wing_usb_feature_config(wusb);

	if (wusb->support_drd) {
		ret = wing_usb_drd_init(dev, wusb);
	} else {
		ret = wing_usb_host_init(dev, wusb);
	}

	if (ret < 0) {
		wing_usb_err("controller init failed, ret = %d\n", ret);
		goto err_unmap;
	}

	return 0;

err_unmap:
	iounmap(wusb->ctrl_base);

	return ret;
}

static int wing_usb_remove(struct platform_device *pdev)
{
	struct wing_usb *wusb = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;

	wing_usb_dbg("+\n");
	if (wusb == NULL) {
		wing_usb_err("wusb NULL\n");
		return -ENODEV;
	}

	if (wusb->support_drd) {
		wing_usb_remove_proc_entry(dev, wusb);

		devm_extcon_dev_unregister(dev, wusb->edev);
		extcon_dev_free(wusb->edev);
		wusb->edev = NULL;

		cancel_work_sync(&wusb->event_work);
	}

	device_for_each_child(dev, NULL, wing_usb_remove_child);

	pm_runtime_put_sync(dev);
	pm_runtime_disable(dev);

	wing_usb_clk_phy_deinit(wusb);

	iounmap(wusb->ctrl_base);

	wing_usb_dbg("-\n");

	return 0;
}

#if IS_ENABLED(CONFIG_PM_SLEEP)
static int wing_usb_suspend(struct device *dev)
{
	int ret;
	struct wing_usb *wusb = dev_get_drvdata(dev);

	if (wusb == NULL) {
		wing_usb_err("wusb is null\n");
		return -1;
	}

	wing_usb_save_mode(wusb);

	ret = wing_usb_clk_phy_deinit(wusb);
	if (ret != 0) {
		wing_usb_err("deinit clk and phy failed when suspend\n");
		return ret;
	}

	return 0;
}

static int wing_usb_resume(struct device *dev)
{
	int ret;
	struct wing_usb *wusb = dev_get_drvdata(dev);

	if (wusb == NULL) {
		wing_usb_err("wusb is null\n");
		return -1;
	}

	ret = wing_usb_clk_phy_init(wusb);
	if (ret != 0) {
		wing_usb_err("init clk and phy failed when reusme\n");
		return ret;
	}

	wing_usb_feature_config(wusb);

	return 0;
}
#endif

const struct dev_pm_ops g_wing_usb_dev_pm_ops = {
#if IS_ENABLED(CONFIG_PM_SLEEP)
	SET_SYSTEM_SLEEP_PM_OPS(wing_usb_suspend, wing_usb_resume)
#endif
};

static const struct of_device_id g_wing_usb_match[] = {
	{ .compatible = "wing-usb,drd" },
	{ .compatible = "wing-usb,host" },
	{},
};
MODULE_DEVICE_TABLE(of, g_wing_usb_match);

static struct platform_driver g_wing_usb_driver = {
	.probe		= wing_usb_probe,
	.remove		= wing_usb_remove,
	.driver		= {
		.name	= "wing-usb",
		.of_match_table = of_match_ptr(g_wing_usb_match),
		.pm = &g_wing_usb_dev_pm_ops,
	},
};

static int __init wing_usb_module_init(void)
{
	int ret;

	ret = platform_driver_register(&g_wing_usb_driver);
	if (ret != 0) {
		wing_usb_err("register wing usb driver failed, ret = %d\n", ret);
		return ret;
	}

	wing_usb_info("register wing usb driver\n");

	return ret;
}
module_init(wing_usb_module_init);

static void __exit wing_usb_module_exit(void)
{
	platform_driver_unregister(&g_wing_usb_driver);

	wing_usb_info("unregister wing usb driver\n");
}
module_exit(wing_usb_module_exit);

MODULE_DESCRIPTION("Wing USB Controller Driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(USB_KERNEL_VERSION);
