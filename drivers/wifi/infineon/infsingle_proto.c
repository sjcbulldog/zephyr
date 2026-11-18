/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file infsingle_proto.c
 * @brief SDPCM/CDC/BDC/event helpers. Layer: protocol.
 *
 * Reference: infdocs/hw.md sections 7, 8, 11. See infsingle_wire.h for diagrams.
 */

#include <errno.h>
#include <string.h>
#include <zephyr/sys/byteorder.h>
#include "infsingle_proto.h"

size_t infsingle_sdpcm_check_tag(const uint8_t tag[4])
{
	/*
	 * The first four bytes of every frame are a self-checking tag: a 16-bit
	 * length followed by its bitwise complement. This is how the receive
	 * path learns the size before reading the rest of the frame.
	 */
	uint16_t size = sys_get_le16(&tag[0]);
	uint16_t inv = sys_get_le16(&tag[2]);

	/* An idle device returns zeros; a valid tag is a bitwise complement. */
	if ((size | inv) == 0 || (uint16_t)(size ^ inv) != 0xFFFF) {
		return 0;
	}
	return size;
}

int infsingle_sdpcm_parse(const uint8_t *buf, size_t len, struct infsingle_rx_frame *out)
{
	size_t size;
	uint8_t hdr_len;

	if (len < INFSINGLE_SDPCM_HDR_LEN) {
		return -EBADMSG;
	}
	size = infsingle_sdpcm_check_tag(buf);
	if (size < INFSINGLE_SDPCM_HDR_LEN || size > len) {
		return -EBADMSG;
	}

	/*
	 * SDPCM header (12 bytes): [0..1] length, [2..3] ~length, [4] sequence,
	 * [5] channel, [6] next length, [7] header length (offset of payload),
	 * [8] flow control, [9] credit limit, [10..11] reserved.
	 * The payload (CDC, event or BDC+data) starts at hdr_len.
	 */
	out->seq = buf[4];
	out->chan = buf[5] & INFSINGLE_SDPCM_CHAN_MASK;
	hdr_len = buf[7];
	out->credit = buf[9];
	out->flow_ctl = buf[8];

	/* A bare 12-byte frame only carries flow-control credit. */
	if (size == INFSINGLE_SDPCM_HDR_LEN) {
		out->payload = NULL;
		out->payload_len = 0;
		return -ENODATA;
	}
	if (hdr_len < INFSINGLE_SDPCM_HDR_LEN || hdr_len > size) {
		return -EBADMSG;
	}
	out->payload = buf + hdr_len;
	out->payload_len = size - hdr_len;
	return 0;
}

void infsingle_sdpcm_fill(uint8_t *buf, size_t total_len, uint8_t chan, uint8_t seq)
{
	memset(buf, 0, INFSINGLE_SDPCM_HDR_LEN);
	sys_put_le16((uint16_t)total_len, &buf[0]);
	sys_put_le16((uint16_t)~total_len, &buf[2]);
	buf[4] = seq;
	buf[5] = chan & INFSINGLE_SDPCM_CHAN_MASK;
	/* Data frames carry 2 pad bytes so the BDC header is 4-byte aligned
	 * relative to the Ethernet payload that follows.
	 */
	buf[7] = (chan == INFSINGLE_SDPCM_CHAN_DATA) ?
		 (INFSINGLE_SDPCM_HDR_LEN + INFSINGLE_SDPCM_DATA_PAD) : INFSINGLE_SDPCM_HDR_LEN;
}

uint8_t infsingle_credit_avail(const struct infsingle_credit *c)
{
	/*
	 * Credit scheme: tx_seq counts frames sent, tx_max is the highest
	 * sequence the firmware will accept. Both are 8-bit and wrap, so the
	 * difference is taken modulo 256.
	 */
	uint8_t n = (uint8_t)(c->tx_max - c->tx_seq);

	/* Zero means exhausted; bit 7 means tx_max is "behind" tx_seq. */
	return (n == 0 || (n & 0x80)) ? 0 : n;
}

void infsingle_credit_update(struct infsingle_credit *c, uint8_t new_max)
{
	c->tx_max = new_max;
}

uint8_t infsingle_credit_take(struct infsingle_credit *c)
{
	return c->tx_seq++;
}

int infsingle_cdc_build(uint8_t *buf, size_t cap, uint32_t cmd, bool set,
		  uint16_t id, uint8_t ifidx, const void *data, size_t len)
{
	struct infsingle_cdc_hdr *h = (struct infsingle_cdc_hdr *)buf;

	if (cap < sizeof(*h) + len) {
		return -ENOMEM;
	}
	h->cmd = sys_cpu_to_le32(cmd);
	h->len = sys_cpu_to_le32((uint32_t)len);
	h->flags = sys_cpu_to_le32((set ? INFSINGLE_CDC_FLAG_SET : 0) |
				   ((uint32_t)ifidx << INFSINGLE_CDC_IF_SHIFT) |
				   ((uint32_t)id << INFSINGLE_CDC_ID_SHIFT));
	h->status = 0;
	if (len > 0 && data != NULL) {
		memmove(buf + sizeof(*h), data, len);
	}
	return (int)(sizeof(*h) + len);
}

int infsingle_iovar_build(uint8_t *buf, size_t cap, const char *name,
		    const void *val, size_t vlen)
{
	size_t nlen = strlen(name) + 1;

	if (cap < nlen + vlen) {
		return -ENOMEM;
	}
	/* Value may alias the destination tail; use memmove for safety. */
	if (vlen > 0) {
		memmove(buf + nlen, val, vlen);
	}
	memcpy(buf, name, nlen);
	return (int)(nlen + vlen);
}

void infsingle_bdc_fill(struct infsingle_bdc_hdr *h, uint8_t ifidx, uint8_t prio)
{
	h->flags = INFSINGLE_BDC_FLAGS_VER;
	h->priority = prio & 0x7;
	h->flags2 = ifidx & 0x0F;
	h->data_offset = 0;
}

uint8_t infsingle_prio_to_ac(uint8_t prio)
{
	/* 802.1D: 0,3=BE 1,2=BK 4,5=VI 6,7=VO. Note 0 is BE, 1-2 are BK. */
	static const uint8_t map[8] = {0, 1, 1, 0, 2, 2, 3, 3};

	return map[prio & 7];
}

int infsingle_event_parse(const uint8_t *buf, size_t len, struct infsingle_event *ev)
{
	/*
	 * Firmware events are disguised as Ethernet frames with a Broadcom
	 * ethertype, carried on the event channel. Layout after the BDC header:
	 * Ethernet header, Broadcom header, event message (big-endian fields),
	 * then event-specific data. Every length is checked before use because
	 * the content comes from the device.
	 */
	/* BDC(4)+off*4, Ethernet(14), bcm hdr(10), message(48) */
	size_t off;
	const struct infsingle_event_msg *m;
	uint16_t ethertype;

	if (len < sizeof(struct infsingle_bdc_hdr)) {
		return -EBADMSG;
	}
	off = sizeof(struct infsingle_bdc_hdr) + (size_t)buf[3] * 4;
	if (len < off + 14 + sizeof(struct infsingle_bcm_ether_hdr) + sizeof(*m)) {
		return -EBADMSG;
	}
	ethertype = sys_get_be16(&buf[off + 12]);
	if (ethertype != INFSINGLE_ETHERTYPE_BRCM) {
		return -EBADMSG;
	}
	off += 14 + sizeof(struct infsingle_bcm_ether_hdr);
	m = (const struct infsingle_event_msg *)(buf + off);

	ev->flags = sys_be16_to_cpu(m->flags);
	ev->type = sys_be32_to_cpu(m->event_type);
	ev->status = sys_be32_to_cpu(m->status);
	ev->reason = sys_be32_to_cpu(m->reason);
	ev->auth_type = sys_be32_to_cpu(m->auth_type);
	ev->ifidx = m->ifidx;
	memcpy(ev->addr, m->addr, 6);

	off += sizeof(*m);
	ev->data_len = sys_be32_to_cpu(m->datalen);
	if (ev->data_len > len - off) {
		return -EBADMSG;
	}
	ev->data = buf + off;
	return 0;
}

void infsingle_event_mask_set(uint8_t *mask, unsigned int event)
{
	mask[event / 8] |= BIT(event % 8);
}
