/* SPDX-License-Identifier: GPL-2.0+ WITH Linux-syscall-note */
/*
 * g_uvc.h  --  USB Video Class Gadget driver API
 *
 * Copyright (C) 2009-2010 Laurent Pinchart <laurent.pinchart@ideasonboard.com>
 */

#ifndef __LINUX_USB_G_UVC_H
#define __LINUX_USB_G_UVC_H

#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/usb/ch9.h>

#define UVC_EVENT_FIRST			(V4L2_EVENT_PRIVATE_START + 0)
#define UVC_EVENT_CONNECT		(V4L2_EVENT_PRIVATE_START + 0)
#define UVC_EVENT_DISCONNECT		(V4L2_EVENT_PRIVATE_START + 1)
#define UVC_EVENT_STREAMON		(V4L2_EVENT_PRIVATE_START + 2)
#define UVC_EVENT_STREAMOFF		(V4L2_EVENT_PRIVATE_START + 3)
#define UVC_EVENT_SETUP			(V4L2_EVENT_PRIVATE_START + 4)
#define UVC_EVENT_DATA			(V4L2_EVENT_PRIVATE_START + 5)
#define UVC_EVENT_LAST			(V4L2_EVENT_PRIVATE_START + 5)

struct uvc_request_data {
	__s32 length;
	__u8 data[60];
};

struct uvc_event {
	union {
		enum usb_device_speed speed;
		struct usb_ctrlrequest req;
		struct uvc_request_data data;
	};
};

#if IS_ENABLED(CONFIG_MPP_TO_GADGET_UVC)
/*-------------------------------------------------------------------------*/
struct uvc_pack {
	uint64_t   buf_vir_addr;
	uint32_t   buf_size;
	uint64_t   pack_vir_addr;
	uint32_t   pack_len;
	void       *private_data;

	void       (*callback_func)(void *);
	bool       is_frame_end;
};

extern int uvc_recv_pack(struct uvc_pack *pack);
#endif /* IS_ENABLED(CONFIG_MPP_TO_GADGET_UVC) */

/*-------------------------------------------------------------------------*/
#define UVCIOC_SEND_RESPONSE		_IOW('U', 1, struct uvc_request_data)

#endif /* __LINUX_USB_G_UVC_H */
