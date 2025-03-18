// SPDX-License-Identifier: GPL-2.0+
/*
 *	uvc_video.c  --  USB Video Class Gadget driver
 *
 *	Copyright (C) 2009-2010
 *	    Laurent Pinchart (laurent.pinchart@ideasonboard.com)
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/securec.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/usb/video.h>
#include <linux/usb/g_uvc.h>
#include <media/v4l2-dev.h>

#include <linux/iprec.h>

#include "uvc.h"
#include "uvc_queue.h"
#include "uvc_video.h"

#include <linux/scatterlist.h>
#include <linux/io.h>
#include <linux/delay.h>

/*************************************************************/
#if IS_ENABLED(CONFIG_MPP_TO_GADGET_UVC)
#define MAX_FRAME 1
static struct uvc_pack_trans g_uvc_pack;

/* the caller shoulud hold g_uvc_pack.lock */
static void clean_untrans_frame(bool need_clean_all)
{
	struct uvc_pack_trans *p;

	int rem = MAX_FRAME;
	if (need_clean_all == 1)
		rem = 0;

	while ((g_uvc_pack.frame_cnts > rem) && !list_empty(&g_uvc_pack.list)) {
		p = list_first_entry(&g_uvc_pack.list, struct uvc_pack_trans, list);
		list_del(&p->list);

		if (p->need_free)
			p->pack->callback_func(p->pack);
		if (p->is_frame_end)
			g_uvc_pack.frame_cnts--;

		kfree(p->pack);
		kfree(p);
	}
}

/* the caller shoulud hold g_uvc_pack.lock */
static void pack_save_to_list(struct uvc_pack_trans *ptr,
		struct uvc_pack *pack, bool is_frame_end, bool free)
{
	ptr->pack = (struct uvc_pack *)kmalloc(sizeof(struct uvc_pack), GFP_KERNEL);
	if (ptr->pack == NULL)
		return;
	if (memcpy_s(ptr->pack, sizeof(struct uvc_pack), pack, sizeof(struct uvc_pack)) != 0) {
		kfree(ptr->pack);
		ptr->pack = NULL;
		return;
	}

	ptr->is_frame_end = is_frame_end;
	ptr->need_free = free;
	ptr->buf_used = 0;

	list_add_tail(&ptr->list, &g_uvc_pack.list);

	if (is_frame_end) {
		g_uvc_pack.frame_cnts++;
		if (g_uvc_pack.frame_cnts > MAX_FRAME) {
			clean_untrans_frame(0);
		}
		if (unlikely(g_uvc_pack.video->is_streaming == false)) {
			clean_untrans_frame(1);
		}
	}
}

int uvc_recv_pack(struct uvc_pack *pack)
{
	struct uvc_pack_trans *p = NULL;
	struct uvc_pack_trans *q = NULL;
	uint64_t end_virt_addr;
	unsigned long flags;

	if (pack->buf_vir_addr == 0 || pack->pack_vir_addr == 0 || pack->pack_len == 0) {
		printk(KERN_EMERG"[Error][uvc_recv_pack] Get NULL pointer addr or illegal length!");

		clean_untrans_frame(1);

		if (pack->callback_func != NULL)
			pack->callback_func(pack);

		return 0;
	}

	end_virt_addr = pack->buf_vir_addr + pack->buf_size;

	if ((pack->pack_vir_addr < pack->buf_vir_addr) || (pack->pack_vir_addr > end_virt_addr)) {
		printk(KERN_EMERG"[Error][uvc_recv_pack] Get illegal pack_vir_addr!");

		clean_untrans_frame(1);

		if (pack->callback_func != NULL)
			pack->callback_func(pack);

		return 0;
	}
	spin_lock_irqsave(&g_uvc_pack.lock, flags);

	p = (struct uvc_pack_trans *)kmalloc(sizeof(struct uvc_pack_trans), GFP_KERNEL);
	if (p == NULL) {
		printk("[Warning][uvc_recv_pack]Can not alloc uvc_pack_trans!");
		return -1;
	}
	if (pack->pack_vir_addr + pack->pack_len > end_virt_addr) {
		p->len = end_virt_addr - pack->pack_vir_addr;
		p->addr = pack->pack_vir_addr;
		pack_save_to_list(p, pack, false, false);

		q = (struct uvc_pack_trans *)kmalloc(sizeof(struct uvc_pack_trans), GFP_KERNEL);
		if (q == NULL) {
			printk("[Warning][uvc_recv_pack]Can not alloc uvc_pack_trans!");
			return -1;
		}
		q->len = pack->pack_len - (end_virt_addr - pack->pack_vir_addr);
		q->addr = pack->buf_vir_addr;
		pack_save_to_list(q, pack, pack->is_frame_end, true);
	} else {
		p->len = pack->pack_len;
		p->addr = pack->pack_vir_addr;
		pack_save_to_list(p, pack, pack->is_frame_end, true);
	}

	spin_unlock_irqrestore(&g_uvc_pack.lock, flags);

	return 0;
}
EXPORT_SYMBOL(uvc_recv_pack);
#endif /* IS_ENABLED(CONFIG_MPP_TO_GADGET_UVC) */

/* --------------------------------------------------------------------------
 * Video codecs
 */
#if IS_ENABLED(CONFIG_MPP_TO_GADGET_UVC)
static int
uvc_video_encode_header(struct uvc_video *video, struct uvc_pack_trans *pack,
		u8 *data, int len)
{
	data[0] = 2;
	data[1] = UVC_STREAM_EOH | video->fid;

	if ((pack->len - pack->buf_used <= len - 2) && pack->is_frame_end)
		data[1] |= UVC_STREAM_EOF;

	return 2;
}

static int
uvc_video_encode_data(struct uvc_video *video, struct uvc_pack_trans *pack,
		u8 *data, int len)
{
	unsigned int nbytes;
	void *mem;

	/* Copy video data to the USB buffer. */
	mem = (void *)pack->addr + pack->buf_used;
	nbytes = min((unsigned int)len, pack->len - pack->buf_used);

	if (memcpy_s(data, len, mem, nbytes) != 0)
		return 0;
	pack->buf_used += nbytes;

	return nbytes;
}

static void
uvc_video_encode_bulk(struct usb_request *req, struct uvc_video *video,
		struct uvc_pack_trans *pack)
{
	void *mem = req->buf;
	int len = video->req_size;
	int ret;

	/* Add a header at the beginning of the payload. */
	if (video->payload_size == 0) {
		ret = uvc_video_encode_header(video, pack, mem, len);
		video->payload_size += ret;
		mem += ret;
		len -= ret;
	}

	/* Process video data. */
	len = min((int)(video->max_payload_size - video->payload_size), len);
	ret = uvc_video_encode_data(video, pack, mem, len);

	video->payload_size += ret;
	len -= ret;

	req->length = video->req_size - len;
	req->zero = video->payload_size == video->max_payload_size;

	if (pack->len == pack->buf_used) {
		pack->buf_used = 0;
		video->fid ^= UVC_STREAM_FID;

		video->payload_size = 0;
	}

	if (video->payload_size == video->max_payload_size ||
	    pack->len == pack->buf_used)
		video->payload_size = 0;
}

/* the caller shoulud hold g_uvc_pack.lock */
static void
uvc_video_encode_isoc(struct usb_request *req, struct uvc_video *video,
		struct uvc_pack_trans *g_pack)
{
	int ret;
	struct uvc_pack_trans *pack = NULL;
#ifdef UVC_SG_REQ
	int len;
	int ttllen = 0;
	unsigned int sg_idx;
	u8 *mem = NULL;

	for (sg_idx = 0; sg_idx < video->num_sgs; sg_idx++) {
		if (unlikely(list_empty(&g_pack->list) || (g_pack->frame_cnts == 0)))
			break;

		pack = list_first_entry(&g_pack->list, struct uvc_pack_trans, list);

		mem = sg_virt(&req->sg[sg_idx]);
		len = video->req_size;

		/* Add the header. */
		ret = uvc_video_encode_header(video, pack, mem, len);
		mem += ret;
		len -= ret;

		/* Process video data. */
		ret = uvc_video_encode_data(video, pack, mem, len);
		len -= ret;

		/* Sync sg buffer len , default is 1024 or 3072 */
		sg_set_buf(&req->sg[sg_idx], sg_virt(&req->sg[sg_idx]),
				video->req_size - len);
		ttllen += video->req_size - len;

		if (pack->len == pack->buf_used) {
			pack->buf_used = 0;

			list_del(&pack->list);
			if (pack->need_free)
				pack->pack->callback_func(pack->pack);

			if (pack->is_frame_end) {
				g_pack->frame_cnts--;
				video->fid ^= UVC_STREAM_FID;
				kfree(pack->pack);
				kfree(pack);
				break;
			}

			kfree(pack->pack);
			kfree(pack);
		}
	}
	req->num_sgs = sg_idx + 1;
	sg_mark_end(&req->sg[sg_idx]);
	req->length = ttllen;
#else /* UVC_SG_REQ */
	void *mem = req->buf;
	int len = video->req_size;

	/* Add the header. */
	ret = uvc_video_encode_header(video, pack, mem, len);
	mem += ret;
	len -= ret;

	/* Process video data. */
	ret = uvc_video_encode_data(video, pack, mem, len);
	len -= ret;

	req->length = video->req_size - len;

	if (pack->len == pack->buf_used) {
		pack->buf_used = 0;
		if (pack->is_frame_end)
			video->fid ^= UVC_STREAM_FID;
	}
#endif /* UVC_SG_REQ */
}

#else /* IS_ENABLED(CONFIG_MPP_TO_GADGET_UVC) */

static int
uvc_video_encode_header(struct uvc_video *video, struct uvc_buffer *buf,
		u8 *data, int len)
{
	data[0] = 2;
	data[1] = UVC_STREAM_EOH | video->fid;

	if (buf->bytesused - video->queue.buf_used <= len - 2)
		data[1] |= UVC_STREAM_EOF;

	return 2;
}

static int
uvc_video_encode_data(struct uvc_video *video, struct uvc_buffer *buf,
		u8 *data, int len)
{
	struct uvc_video_queue *queue = &video->queue;
	unsigned int nbytes;
	void *mem;

	/* Copy video data to the USB buffer. */
	mem = buf->mem + queue->buf_used;
	nbytes = min((unsigned int)len, buf->bytesused - queue->buf_used);

	memcpy(data, mem, nbytes);
	queue->buf_used += nbytes;

	return nbytes;
}

static void
uvc_video_encode_bulk(struct usb_request *req, struct uvc_video *video,
		struct uvc_buffer *buf)
{
	void *mem = req->buf;
	int len = video->req_size;
	int ret;

	/* Add a header at the beginning of the payload. */
	if (video->payload_size == 0) {
		ret = uvc_video_encode_header(video, buf, mem, len);
		video->payload_size += ret;
		mem += ret;
		len -= ret;
	}

	/* Process video data. */
	len = min((int)(video->max_payload_size - video->payload_size), len);
	ret = uvc_video_encode_data(video, buf, mem, len);

	video->payload_size += ret;
	len -= ret;

	req->length = video->req_size - len;
	req->zero = video->payload_size == video->max_payload_size;

	if (buf->bytesused == video->queue.buf_used) {
		video->queue.buf_used = 0;
		buf->state = UVC_BUF_STATE_DONE;
		uvcg_queue_next_buffer(&video->queue, buf);
		video->fid ^= UVC_STREAM_FID;

		video->payload_size = 0;
	}

	if (video->payload_size == video->max_payload_size ||
	    buf->bytesused == video->queue.buf_used)
		video->payload_size = 0;
}

static void
uvc_video_encode_isoc(struct usb_request *req, struct uvc_video *video,
		struct uvc_buffer *buf)
{
	int ret;
#ifdef UVC_SG_REQ
	int len;
	int ttllen = 0;
	unsigned int sg_idx;
	u8 *mem = NULL;

	for (sg_idx = 0; sg_idx < video->num_sgs; sg_idx++) {
		mem = sg_virt(&req->sg[sg_idx]);
		len = video->req_size;

		/* Add the header. */
		ret = uvc_video_encode_header(video, buf, mem, len);
		mem += ret;
		len -= ret;

		/* Process video data. */
		ret = uvc_video_encode_data(video, buf, mem, len);
		len -= ret;

		/* Sync sg buffer len , default is 1024 or 3072 */
		sg_set_buf(&req->sg[sg_idx], sg_virt(&req->sg[sg_idx]),
				video->req_size - len);
		ttllen += video->req_size - len;

		if (buf->bytesused == video->queue.buf_used) {
			video->queue.buf_used = 0;
			buf->state = UVC_BUF_STATE_DONE;
			uvcg_queue_next_buffer(&video->queue, buf);
			video->fid ^= UVC_STREAM_FID;
			break;
		}
	}
	req->num_sgs = sg_idx + 1;
	sg_mark_end(&req->sg[sg_idx]);
	req->length = ttllen;
#else /* UVC_SG_REQ */
	void *mem = req->buf;
	int len = video->req_size;
	int ret;

	/* Add the header. */
	ret = uvc_video_encode_header(video, buf, mem, len);
	mem += ret;
	len -= ret;

	/* Process video data. */
	ret = uvc_video_encode_data(video, buf, mem, len);
	len -= ret;

	req->length = video->req_size - len;

	if (buf->bytesused == video->queue.buf_used) {
		video->queue.buf_used = 0;
		buf->state = UVC_BUF_STATE_DONE;
		uvcg_queue_next_buffer(&video->queue, buf);
		video->fid ^= UVC_STREAM_FID;
	}
#endif /* UVC_SG_REQ */
}
#endif /* IS_ENABLED(CONFIG_MPP_TO_GADGET_UVC) */
/* --------------------------------------------------------------------------
 * Request handling
 */

static int uvcg_video_ep_queue(struct uvc_video *video, struct usb_request *req)
{
	int ret;

	/*
	 * Fixme, this is just to workaround the warning by udc core when the ep
	 * is disabled, this may happens when the uvc application is still
	 * streaming new data while the uvc gadget driver has already recieved
	 * the streamoff but the streamoff event is not yet received by the app
	 */
	if (!video->ep->enabled)
		return -EINVAL;

	ret = usb_ep_queue(video->ep, req, GFP_ATOMIC);
	if (ret < 0) {
		uvcg_err(&video->uvc->func, "Failed to queue request (%d).\n",
			 ret);

		/* If the endpoint is disabled the descriptor may be NULL. */
		if (video->ep->desc) {
			/* Isochronous endpoints can't be halted. */
			if (usb_endpoint_xfer_bulk(video->ep->desc))
				usb_ep_set_halt(video->ep);
		}
	}

	return ret;
}

static void
uvc_video_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct uvc_video *video = req->context;
#if !(IS_ENABLED(CONFIG_MPP_TO_GADGET_UVC))
	struct uvc_video_queue *queue = &video->queue;
#endif
	unsigned long flags;

	switch (req->status) {
	case 0:
		break;

	case -ESHUTDOWN:	/* disconnect from host. */
		uvcg_dbg(&video->uvc->func, "VS request cancelled.\n");
#if !(IS_ENABLED(CONFIG_MPP_TO_GADGET_UVC))
		uvcg_queue_cancel(queue, 1);
#endif
		break;

	default:
		uvcg_warn(&video->uvc->func,
			  "VS request completed with status %d.\n",
			  req->status);
#if !(IS_ENABLED(CONFIG_MPP_TO_GADGET_UVC))
		uvcg_queue_cancel(queue, 0);
#endif
	}

	spin_lock_irqsave(&video->req_lock, flags);
	list_add_tail(&req->list, &video->req_free);
	spin_unlock_irqrestore(&video->req_lock, flags);

	schedule_work(&video->pump);
}

static int
uvc_video_free_requests(struct uvc_video *video)
{
	unsigned int i;
#if IS_ENABLED(CONFIG_MPP_TO_GADGET_UVC)
	unsigned long flags;
#endif
#ifdef UVC_SG_REQ
	unsigned int sg_idx;
#endif

	for (i = 0; i < UVC_NUM_REQUESTS; ++i) {
		if (video->req[i]) {
#ifdef UVC_SG_REQ
			for (sg_idx = 0; sg_idx < video->num_sgs; sg_idx++)
				if (sg_page(&video->req[i]->sg[sg_idx])) {
					kfree(sg_virt(&video->req[i]->sg[sg_idx]));
					video->req[i]->num_mapped_sgs = 0;
				}

			if (video->req[i]->sg) {
				kfree(video->req[i]->sg);
				video->req[i]->sg = NULL;
			}
#endif
			usb_ep_free_request(video->ep, video->req[i]);
			video->req[i] = NULL;
		}

		if (video->req_buffer[i]) {
			kfree(video->req_buffer[i]);
			video->req_buffer[i] = NULL;
		}
	}

#if IS_ENABLED(CONFIG_MPP_TO_GADGET_UVC)
	spin_lock_irqsave(&g_uvc_pack.lock, flags);
	clean_untrans_frame(1);
	g_uvc_pack.video->is_streaming = false;
	spin_unlock_irqrestore(&g_uvc_pack.lock, flags);

#endif

	INIT_LIST_HEAD(&video->req_free);
	video->req_size = 0;
	return 0;
}

static int
uvc_video_alloc_requests(struct uvc_video *video)
{
	unsigned int req_size;
	unsigned int i;
	int ret = -ENOMEM;
#ifdef UVC_SG_REQ
	struct scatterlist  *sg;
	unsigned int num_sgs;
	unsigned int sg_idx;
#endif

	BUG_ON(video->req_size);

	req_size = video->ep->maxpacket
		 * max_t(unsigned int, video->ep->maxburst, 1)
		 * (video->ep->mult);

#ifdef UVC_SG_REQ
	num_sgs = ((video->imagesize / (req_size - 2)) + 1);
	video->num_sgs = num_sgs;

	for (i = 0; i < UVC_NUM_REQUESTS; ++i) {
		sg = kmalloc(num_sgs * sizeof(struct scatterlist), GFP_ATOMIC);
		if (sg == NULL)
			goto error;
		sg_init_table(sg, num_sgs);

		video->req[i] = usb_ep_alloc_request(video->ep, GFP_KERNEL);
		if (video->req[i] == NULL)
			goto error;

		for (sg_idx = 0 ; sg_idx < num_sgs ; sg_idx++) {
			video->sg_buf = kmalloc(req_size, GFP_KERNEL);
			if (video->sg_buf == NULL)
				goto error;
			sg_set_buf(&sg[sg_idx], video->sg_buf, req_size);
		}
		video->req[i]->sg = sg;
		video->req[i]->num_sgs = num_sgs;
		video->req[i]->length = 0;
		video->req[i]->complete = uvc_video_complete;
		video->req[i]->context = video;

		list_add_tail(&video->req[i]->list, &video->req_free);
	}
#else
	for (i = 0; i < UVC_NUM_REQUESTS; ++i) {
		video->req_buffer[i] = kmalloc(req_size, GFP_KERNEL);
		if (video->req_buffer[i] == NULL)
			goto error;

		video->req[i] = usb_ep_alloc_request(video->ep, GFP_KERNEL);
		if (video->req[i] == NULL)
			goto error;

		video->req[i]->buf = video->req_buffer[i];
		video->req[i]->length = 0;
		video->req[i]->complete = uvc_video_complete;
		video->req[i]->context = video;

		list_add_tail(&video->req[i]->list, &video->req_free);
	}
#endif
	video->req_size = req_size;

	return 0;

error:
	uvc_video_free_requests(video);
	return ret;
}

/* --------------------------------------------------------------------------
 * Video streaming
 */

/*
 * uvcg_video_pump - Pump video data into the USB requests
 *
 * This function fills the available USB requests (listed in req_free) with
 * video data from the queued buffers.
 */
#if IS_ENABLED(CONFIG_MPP_TO_GADGET_UVC)
static void uvcg_video_pump(struct work_struct *work)
{
	struct uvc_video *video = container_of(work, struct uvc_video, pump);
	struct usb_request *req;
	unsigned long flags;
	int ret;

	if (g_uvc_pack.video->is_streaming == false)
		return;

	while (1) {
		/* Retrieve the first available USB request, protected by the
		 * request lock.
		 */
		spin_lock_irqsave(&video->req_lock, flags);
		if (list_empty(&video->req_free)) {
			spin_unlock_irqrestore(&video->req_lock, flags);
			return;
		}
		req = list_first_entry(&video->req_free, struct usb_request,
					list);
		list_del(&req->list);
		spin_unlock_irqrestore(&video->req_lock, flags);

		/* Retrieve the first available video buffer and fill the
		 * request, protected by the video queue irqlock.
		 */
		spin_lock_irqsave(&g_uvc_pack.lock, flags);
		if (list_empty(&g_uvc_pack.list) || (g_uvc_pack.frame_cnts == 0)) {
			spin_unlock_irqrestore(&g_uvc_pack.lock, flags);
			break;
		}

#ifdef UVC_SG_REQ
		sg_unmark_end(&req->sg[req->num_sgs - 1]);
#endif

		video->encode(req, video, &g_uvc_pack);

		/* Queue the USB request */
		ret = uvcg_video_ep_queue(video, req);
		if (ret < 0) {
			spin_unlock_irqrestore(&g_uvc_pack.lock, flags);
			break;
		}

		spin_unlock_irqrestore(&g_uvc_pack.lock, flags);
	}

	spin_lock_irqsave(&video->req_lock, flags);
#ifdef UVC_SG_REQ
	sg_unmark_end(&req->sg[req->num_sgs - 1]);
#endif
	list_add_tail(&req->list, &video->req_free);
	spin_unlock_irqrestore(&video->req_lock, flags);
	return;
}
#else /* IS_ENABLED(CONFIG_MPP_TO_GADGET_UVC) */
static void uvcg_video_pump(struct work_struct *work)
{
	struct uvc_video *video = container_of(work, struct uvc_video, pump);

	struct uvc_video_queue *queue = &video->queue;
	struct usb_request *req;
	struct uvc_buffer *buf;
	unsigned long flags;
	int ret;

	while (1) {
		/* Retrieve the first available USB request, protected by the
		 * request lock.
		 */
		spin_lock_irqsave(&video->req_lock, flags);
		if (list_empty(&video->req_free)) {
			spin_unlock_irqrestore(&video->req_lock, flags);
			return;
		}
		req = list_first_entry(&video->req_free, struct usb_request,
					list);
		list_del(&req->list);
		spin_unlock_irqrestore(&video->req_lock, flags);

		/* Retrieve the first available video buffer and fill the
		 * request, protected by the video queue irqlock.
		 */
		spin_lock_irqsave(&queue->irqlock, flags);
		buf = uvcg_queue_head(queue);
		if (buf == NULL) {
			spin_unlock_irqrestore(&queue->irqlock, flags);
			break;
		}

#ifdef UVC_SG_REQ
		sg_unmark_end(&req->sg[req->num_sgs - 1]);
#endif

		video->encode(req, video, buf);

		/* Queue the USB request */
		ret = uvcg_video_ep_queue(video, req);
		spin_unlock_irqrestore(&queue->irqlock, flags);

		if (ret < 0) {
			uvcg_queue_cancel(queue, 0);
			break;
		}
	}

	spin_lock_irqsave(&video->req_lock, flags);
#ifdef UVC_SG_REQ
	sg_unmark_end(&req->sg[req->num_sgs - 1]);
#endif
	list_add_tail(&req->list, &video->req_free);
	spin_unlock_irqrestore(&video->req_lock, flags);
	return;
}
#endif /* IS_ENABLED(CONFIG_MPP_TO_GADGET_UVC) */

/*
 * Enable or disable the video stream.
 */
int uvcg_video_enable(struct uvc_video *video, int enable)
{
	unsigned int i;
	int ret;
#if IS_ENABLED(CONFIG_MPP_TO_GADGET_UVC)
	unsigned long flags;
#endif

	iprec("[%s] %d", __func__, enable);
	if (video->ep == NULL) {
		uvcg_info(&video->uvc->func,
			  "Video enable failed, device is uninitialized.\n");
		return -ENODEV;
	}

	if (!enable) {
		cancel_work_sync(&video->pump);
		uvcg_queue_cancel(&video->queue, 0);

		for (i = 0; i < UVC_NUM_REQUESTS; ++i)
			if (video->req[i])
				usb_ep_dequeue(video->ep, video->req[i]);

		uvc_video_free_requests(video);
#if !(IS_ENABLED(CONFIG_MPP_TO_GADGET_UVC))
		uvcg_queue_enable(&video->queue, 0);
#endif
		return 0;
	}

#if !(IS_ENABLED(CONFIG_MPP_TO_GADGET_UVC))
	if ((ret = uvcg_queue_enable(&video->queue, 1)) < 0)
		return ret;
#endif

	if ((ret = uvc_video_alloc_requests(video)) < 0)
		return ret;

	if (video->max_payload_size) {
		video->encode = uvc_video_encode_bulk;
		video->payload_size = 0;
	} else
		video->encode = uvc_video_encode_isoc;

#if IS_ENABLED(CONFIG_MPP_TO_GADGET_UVC)
	spin_lock_irqsave(&g_uvc_pack.lock, flags);
	g_uvc_pack.video->is_streaming = true;
	spin_unlock_irqrestore(&g_uvc_pack.lock, flags);
#endif

	schedule_work(&video->pump);

	return ret;
}

/*
 * Initialize the UVC video stream.
 */
int uvcg_video_init(struct uvc_video *video, struct uvc_device *uvc)
{
#if IS_ENABLED(CONFIG_MPP_TO_GADGET_UVC)
	INIT_LIST_HEAD(&g_uvc_pack.list);
	spin_lock_init(&g_uvc_pack.lock);
	g_uvc_pack.frame_cnts = 0;
	video->is_streaming = false;
	g_uvc_pack.video = video;
#endif
	INIT_LIST_HEAD(&video->req_free);
	spin_lock_init(&video->req_lock);
	INIT_WORK(&video->pump, uvcg_video_pump);

	video->uvc = uvc;
	video->fcc = V4L2_PIX_FMT_YUYV;
	video->bpp = 16;
	video->width = 320;
	video->height = 240;
	video->imagesize = 320 * 240 * 2;

	/* Initialize the video buffers queue. */
	uvcg_queue_init(&video->queue, V4L2_BUF_TYPE_VIDEO_OUTPUT,
			&video->mutex);
	return 0;
}
