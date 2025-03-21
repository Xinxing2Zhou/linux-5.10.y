// SPDX-License-Identifier: GPL-2.0
/*
 * Arm Statistical Profiling Extensions (SPE) support
 * Copyright (c) 2017-2018, Arm Ltd.
 */

#include <stdio.h>
#include <string.h>
#include <endian.h>
#include <byteswap.h>
#include <linux/bitops.h>
#include <stdarg.h>

#include "arm-spe-pkt-decoder.h"

#if __BYTE_ORDER == __BIG_ENDIAN
#define le16_to_cpu bswap_16
#define le32_to_cpu bswap_32
#define le64_to_cpu bswap_64
#define memcpy_le64(d, s, n) do { \
	memcpy((d), (s), (n));    \
	*(d) = le64_to_cpu(*(d)); \
} while (0)
#else
#define le16_to_cpu
#define le32_to_cpu
#define le64_to_cpu
#define memcpy_le64 memcpy
#endif

static const char * const arm_spe_packet_name[] = {
	[ARM_SPE_PAD]		= "PAD",
	[ARM_SPE_END]		= "END",
	[ARM_SPE_TIMESTAMP]	= "TS",
	[ARM_SPE_ADDRESS]	= "ADDR",
	[ARM_SPE_COUNTER]	= "LAT",
	[ARM_SPE_CONTEXT]	= "CONTEXT",
	[ARM_SPE_OP_TYPE]	= "OP-TYPE",
	[ARM_SPE_EVENTS]	= "EVENTS",
	[ARM_SPE_DATA_SOURCE]	= "DATA-SOURCE",
};

const char *arm_spe_pkt_name(enum arm_spe_pkt_type type)
{
	return arm_spe_packet_name[type];
}

/* return ARM SPE payload size from its encoding,
 * which is in bits 5:4 of the byte.
 * 00 : byte
 * 01 : halfword (2)
 * 10 : word (4)
 * 11 : doubleword (8)
 */
static int payloadlen(unsigned char byte)
{
	return 1 << ((byte & 0x30) >> 4);
}

static int arm_spe_get_payload(const unsigned char *buf, size_t len,
			       struct arm_spe_pkt *packet)
{
	size_t payload_len = payloadlen(buf[0]);

	if (len < 1 + payload_len)
		return ARM_SPE_NEED_MORE_BYTES;

	buf++;

	switch (payload_len) {
	case 1: packet->payload = *(uint8_t *)buf; break;
	case 2: packet->payload = le16_to_cpu(*(uint16_t *)buf); break;
	case 4: packet->payload = le32_to_cpu(*(uint32_t *)buf); break;
	case 8: packet->payload = le64_to_cpu(*(uint64_t *)buf); break;
	default: return ARM_SPE_BAD_PACKET;
	}

	return 1 + payload_len;
}

static int arm_spe_get_pad(struct arm_spe_pkt *packet)
{
	packet->type = ARM_SPE_PAD;
	return 1;
}

static int arm_spe_get_alignment(const unsigned char *buf, size_t len,
				 struct arm_spe_pkt *packet)
{
	unsigned int alignment = 1 << ((buf[0] & 0xf) + 1);

	if (len < alignment)
		return ARM_SPE_NEED_MORE_BYTES;

	packet->type = ARM_SPE_PAD;
	return alignment - (((uintptr_t)buf) & (alignment - 1));
}

static int arm_spe_get_end(struct arm_spe_pkt *packet)
{
	packet->type = ARM_SPE_END;
	return 1;
}

static int arm_spe_get_timestamp(const unsigned char *buf, size_t len,
				 struct arm_spe_pkt *packet)
{
	packet->type = ARM_SPE_TIMESTAMP;
	return arm_spe_get_payload(buf, len, packet);
}

static int arm_spe_get_events(const unsigned char *buf, size_t len,
			      struct arm_spe_pkt *packet)
{
	int ret = arm_spe_get_payload(buf, len, packet);

	packet->type = ARM_SPE_EVENTS;

	/* we use index to identify Events with a less number of
	 * comparisons in arm_spe_pkt_desc(): E.g., the LLC-ACCESS,
	 * LLC-REFILL, and REMOTE-ACCESS events are identified iff
	 * index > 1.
	 */
	packet->index = ret - 1;

	return ret;
}

static int arm_spe_get_data_source(const unsigned char *buf, size_t len,
				   struct arm_spe_pkt *packet)
{
	packet->type = ARM_SPE_DATA_SOURCE;
	return arm_spe_get_payload(buf, len, packet);
}

static int arm_spe_get_context(const unsigned char *buf, size_t len,
			       struct arm_spe_pkt *packet)
{
	packet->type = ARM_SPE_CONTEXT;
	packet->index = buf[0] & 0x3;

	return arm_spe_get_payload(buf, len, packet);
}

static int arm_spe_get_op_type(const unsigned char *buf, size_t len,
			       struct arm_spe_pkt *packet)
{
	packet->type = ARM_SPE_OP_TYPE;
	packet->index = buf[0] & 0x3;
	return arm_spe_get_payload(buf, len, packet);
}

static int arm_spe_get_counter(const unsigned char *buf, size_t len,
			       const unsigned char ext_hdr, struct arm_spe_pkt *packet)
{
	if (len < 2)
		return ARM_SPE_NEED_MORE_BYTES;

	packet->type = ARM_SPE_COUNTER;
	if (ext_hdr)
		packet->index = ((buf[0] & 0x3) << 3) | (buf[1] & 0x7);
	else
		packet->index = buf[0] & 0x7;

	packet->payload = le16_to_cpu(*(uint16_t *)(buf + 1));

	return 1 + ext_hdr + 2;
}

static int arm_spe_get_addr(const unsigned char *buf, size_t len,
			    const unsigned char ext_hdr, struct arm_spe_pkt *packet)
{
	if (len < 8)
		return ARM_SPE_NEED_MORE_BYTES;

	packet->type = ARM_SPE_ADDRESS;

	if (ext_hdr)
		packet->index = SPE_HDR_EXTENDED_INDEX(buf[0], buf[1]);
	else
		packet->index = SPE_HDR_SHORT_INDEX(buf[0]);

	memcpy_le64(&packet->payload, buf + 1, 8);

	return 1 + ext_hdr + 8;
}

static int arm_spe_do_get_packet(const unsigned char *buf, size_t len,
				 struct arm_spe_pkt *packet)
{
	unsigned int hdr;
	unsigned char ext_hdr = 0;

	memset(packet, 0, sizeof(struct arm_spe_pkt));

	if (!len)
		return ARM_SPE_NEED_MORE_BYTES;

	hdr = buf[0];

	if (hdr == SPE_HEADER0_PAD)
		return arm_spe_get_pad(packet);

	if (hdr == SPE_HEADER0_END) /* no timestamp at end of record */
		return arm_spe_get_end(packet);

	if (hdr == SPE_HEADER0_TIMESTAMP)
		return arm_spe_get_timestamp(buf, len, packet);

	if ((hdr & SPE_HEADER0_MASK1) == SPE_HEADER0_EVENTS)
		return arm_spe_get_events(buf, len, packet);

	if ((hdr & SPE_HEADER0_MASK1) == SPE_HEADER0_SOURCE)
		return arm_spe_get_data_source(buf, len, packet);

	if ((hdr & SPE_HEADER0_MASK2) == SPE_HEADER0_CONTEXT)
		return arm_spe_get_context(buf, len, packet);

	if ((hdr & SPE_HEADER0_MASK2) == SPE_HEADER0_OP_TYPE)
		return arm_spe_get_op_type(buf, len, packet);

	if ((hdr & SPE_HEADER0_MASK2) == SPE_HEADER0_EXTENDED) {
		/* 16-bit extended format header */
		ext_hdr = 1;

		hdr = buf[1];
		if (hdr == SPE_HEADER1_ALIGNMENT)
			return arm_spe_get_alignment(buf, len, packet);
	}

	/*
	 * The short format header's byte 0 or the extended format header's
	 * byte 1 has been assigned to 'hdr', which uses the same encoding for
	 * address packet and counter packet, so don't need to distinguish if
	 * it's short format or extended format and handle in once.
	 */
	if ((hdr & SPE_HEADER0_MASK3) == SPE_HEADER0_ADDRESS)
		return arm_spe_get_addr(buf, len, ext_hdr, packet);

	if ((hdr & SPE_HEADER0_MASK3) == SPE_HEADER0_COUNTER)
		return arm_spe_get_counter(buf, len, ext_hdr, packet);

	return ARM_SPE_BAD_PACKET;
}

int arm_spe_get_packet(const unsigned char *buf, size_t len,
		       struct arm_spe_pkt *packet)
{
	int ret;

	ret = arm_spe_do_get_packet(buf, len, packet);
	/* put multiple consecutive PADs on the same line, up to
	 * the fixed-width output format of 16 bytes per line.
	 */
	if (ret > 0 && packet->type == ARM_SPE_PAD) {
		while (ret < 16 && len > (size_t)ret && !buf[ret])
			ret += 1;
	}
	return ret;
}

static int arm_spe_pkt_out_string(int *err, char **buf_p, size_t *blen,
				  const char *fmt, ...)
{
	va_list ap;
	int ret;

	/* Bail out if any error occurred */
	if (err && *err)
		return *err;

	va_start(ap, fmt);
	ret = vsnprintf(*buf_p, *blen, fmt, ap);
	va_end(ap);

	if (ret < 0) {
		if (err && !*err)
			*err = ret;

	/*
	 * A return value of *blen or more means that the output was
	 * truncated and the buffer is overrun.
	 */
	} else if ((size_t)ret >= *blen) {
		(*buf_p)[*blen - 1] = '\0';

		/*
		 * Set *err to 'ret' to avoid overflow if tries to
		 * fill this buffer sequentially.
		 */
		if (err && !*err)
			*err = ret;
	} else {
		*buf_p += ret;
		*blen -= ret;
	}

	return ret;
}

static int arm_spe_pkt_desc_addr(const struct arm_spe_pkt *packet,
				 char *buf, size_t buf_len)
{
	int ns, el, idx = packet->index;
	u64 payload = packet->payload;
	int err = 0;

	switch (idx) {
	case SPE_ADDR_PKT_HDR_INDEX_INS:
	case SPE_ADDR_PKT_HDR_INDEX_BRANCH:
		ns = !!SPE_ADDR_PKT_GET_NS(payload);
		el = SPE_ADDR_PKT_GET_EL(payload);
		payload = SPE_ADDR_PKT_ADDR_GET_BYTES_0_6(payload);
		arm_spe_pkt_out_string(&err, &buf, &buf_len,
				"%s 0x%llx el%d ns=%d",
				(idx == 1) ? "TGT" : "PC", payload, el, ns);
		break;
	case SPE_ADDR_PKT_HDR_INDEX_DATA_VIRT:
		arm_spe_pkt_out_string(&err, &buf, &buf_len,
				       "VA 0x%llx", payload);
		break;
	case SPE_ADDR_PKT_HDR_INDEX_DATA_PHYS:
		ns = !!SPE_ADDR_PKT_GET_NS(payload);
		payload = SPE_ADDR_PKT_ADDR_GET_BYTES_0_6(payload);
		arm_spe_pkt_out_string(&err, &buf, &buf_len,
				       "PA 0x%llx ns=%d", payload, ns);
		break;
	default:
		/* Unknown index */
		err = -1;
		break;
	}

	return err;
}

int arm_spe_pkt_desc(const struct arm_spe_pkt *packet, char *buf,
		     size_t buf_len)
{
	int idx = packet->index;
	unsigned long long payload = packet->payload;
	const char *name = arm_spe_pkt_name(packet->type);
	char *buf_orig = buf;
	size_t blen = buf_len;
	int err = 0;

	switch (packet->type) {
	case ARM_SPE_BAD:
	case ARM_SPE_PAD:
	case ARM_SPE_END:
		arm_spe_pkt_out_string(&err, &buf, &blen, "%s", name);
		break;
	case ARM_SPE_EVENTS:
		arm_spe_pkt_out_string(&err, &buf, &blen, "EV");

		if (payload & 0x1)
			arm_spe_pkt_out_string(&err, &buf, &blen, " EXCEPTION-GEN");
		if (payload & 0x2)
			arm_spe_pkt_out_string(&err, &buf, &blen, " RETIRED");
		if (payload & 0x4)
			arm_spe_pkt_out_string(&err, &buf, &blen, " L1D-ACCESS");
		if (payload & 0x8)
			arm_spe_pkt_out_string(&err, &buf, &blen, " L1D-REFILL");
		if (payload & 0x10)
			arm_spe_pkt_out_string(&err, &buf, &blen, " TLB-ACCESS");
		if (payload & 0x20)
			arm_spe_pkt_out_string(&err, &buf, &blen, " TLB-REFILL");
		if (payload & 0x40)
			arm_spe_pkt_out_string(&err, &buf, &blen, " NOT-TAKEN");
		if (payload & 0x80)
			arm_spe_pkt_out_string(&err, &buf, &blen, " MISPRED");
		if (idx > 1) {
			if (payload & 0x100)
				arm_spe_pkt_out_string(&err, &buf, &blen, " LLC-ACCESS");
			if (payload & 0x200)
				arm_spe_pkt_out_string(&err, &buf, &blen, " LLC-REFILL");
			if (payload & 0x400)
				arm_spe_pkt_out_string(&err, &buf, &blen, " REMOTE-ACCESS");
		}
		break;
	case ARM_SPE_OP_TYPE:
		switch (idx) {
		case 0:
			arm_spe_pkt_out_string(&err, &buf, &blen,
					payload & 0x1 ? "COND-SELECT" : "INSN-OTHER");
			break;
		case 1:
			arm_spe_pkt_out_string(&err, &buf, &blen,
					       payload & 0x1 ? "ST" : "LD");

			if (payload & 0x2) {
				if (payload & 0x4)
					arm_spe_pkt_out_string(&err, &buf, &blen, " AT");
				if (payload & 0x8)
					arm_spe_pkt_out_string(&err, &buf, &blen, " EXCL");
				if (payload & 0x10)
					arm_spe_pkt_out_string(&err, &buf, &blen, " AR");
			} else if (payload & 0x4) {
				arm_spe_pkt_out_string(&err, &buf, &blen, " SIMD-FP");
			}
			break;
		case 2:
			arm_spe_pkt_out_string(&err, &buf, &blen, "B");

			if (payload & 0x1)
				arm_spe_pkt_out_string(&err, &buf, &blen, " COND");
			if (payload & 0x2)
				arm_spe_pkt_out_string(&err, &buf, &blen, " IND");

			break;
		default:
			/* Unknown index */
			err = -1;
			break;
		}
		break;
	case ARM_SPE_DATA_SOURCE:
	case ARM_SPE_TIMESTAMP:
		arm_spe_pkt_out_string(&err, &buf, &blen, "%s %lld", name, payload);
		break;
	case ARM_SPE_ADDRESS:
		err = arm_spe_pkt_desc_addr(packet, buf, buf_len);
		break;
	case ARM_SPE_CONTEXT:
		arm_spe_pkt_out_string(&err, &buf, &blen, "%s 0x%lx el%d",
				       name, (unsigned long)payload, idx + 1);
		break;
	case ARM_SPE_COUNTER:
		arm_spe_pkt_out_string(&err, &buf, &blen, "%s %d ", name,
				       (unsigned short)payload);
		switch (idx) {
		case 0:
			arm_spe_pkt_out_string(&err, &buf, &blen, "TOT");
			break;
		case 1:
			arm_spe_pkt_out_string(&err, &buf, &blen, "ISSUE");
			break;
		case 2:
			arm_spe_pkt_out_string(&err, &buf, &blen, "XLAT");
			break;
		default:
			break;
		}
		break;
	default:
		/* Unknown packet type */
		err = -1;
		break;
	}

	/* Output raw data if detect any error */
	if (err) {
		err = 0;
		arm_spe_pkt_out_string(&err, &buf_orig, &buf_len, "%s 0x%llx (%d)",
				       name, payload, packet->index);
	}

	return err;
}
