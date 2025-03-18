 /*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2020-2023. All rights reserved.
 */

#ifndef _WING_USB_PROC_H_
#define _WING_USB_PROC_H_

#include <linux/device.h>
#include <linux/kernel.h>
#include "wing_usb.h"

int wing_usb_create_proc_entry(struct device *dev, struct wing_usb *wusb);
void wing_usb_remove_proc_entry(struct device *dev, struct wing_usb *wusb);

#endif /* _WING_USB_PROC_H_ */
