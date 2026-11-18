/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file infsingle_proto.h
 * @brief Pure protocol helpers: SDPCM, credits, CDC, BDC, event parsing.
 *
 * Layer: protocol. Reference: infdocs/hw.md sections 7, 8, 11.
 *
 * Nothing here touches hardware, allocates, or blocks, so every function is
 * unit-testable on the host with a plain buffer. The caller owns all buffers;
 * parse functions return pointers INTO the caller's buffer (no copy, R-PERF-1).
 */

#ifndef ZEPHYR_DRIVERS_WIFI_INFSINGLE_INFSINGLE_PROTO_H_
#define ZEPHYR_DRIVERS_WIFI_INFSINGLE_INFSINGLE_PROTO_H_

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "infsingle_wire.h"

/* ---- SDPCM -------------------------------------------------------------- */

/** Result of validating and decomposing a received SDPCM frame. */
struct infsingle_rx_frame {
	uint8_t chan;            /* INFSINGLE_SDPCM_CHAN_*                           */
	uint8_t seq;
	uint8_t credit;          /* tx_max announced by the firmware           */
	uint8_t flow_ctl;        /* non-zero: firmware asks the host to stop data */
	const uint8_t *payload;  /* first byte after SDPCM header (+hdr_len)   */
	size_t payload_len;
};

/**
 * Validate the 4-byte hardware tag only (size and ~size).
 * @return frame length on success, 0 when the tag is invalid or idle.
 */
size_t infsingle_sdpcm_check_tag(const uint8_t tag[4]);

/**
 * Parse a complete received frame.
 * @retval 0 ok, -EBADMSG malformed, -ENODATA credit-only frame (still
 *         updates @p out->credit).
 */
int infsingle_sdpcm_parse(const uint8_t *buf, size_t len, struct infsingle_rx_frame *out);

/**
 * Write the SDPCM header for a transmit frame of @p total_len bytes.
 * The caller has already reserved INFSINGLE_SDPCM_HDR_LEN (+pad for data) bytes.
 */
void infsingle_sdpcm_fill(uint8_t *buf, size_t total_len, uint8_t chan, uint8_t seq);

/* ---- Credits ------------------------------------------------------------ */

/**
 * Flow-control state. The firmware tells us the highest sequence number it
 * will accept (tx_max). We may send while tx_seq != tx_max, with 8-bit wrap:
 *
 *      credits = (tx_max - tx_seq) & 0xFF;   0 or bit 7 set => none
 */
struct infsingle_credit {
	uint8_t tx_seq;
	uint8_t tx_max;
};

uint8_t infsingle_credit_avail(const struct infsingle_credit *c);
void infsingle_credit_update(struct infsingle_credit *c, uint8_t new_max);
/** Consume one credit and return the sequence number to place in the frame. */
uint8_t infsingle_credit_take(struct infsingle_credit *c);

/* ---- CDC ---------------------------------------------------------------- */

/**
 * Build a CDC request in @p buf (header followed by @p data).
 * @return total bytes written (header + len), or -ENOMEM if @p cap too small.
 */
int infsingle_cdc_build(uint8_t *buf, size_t cap, uint32_t cmd, bool set,
		  uint16_t id, uint8_t ifidx, const void *data, size_t len);

/**
 * Build the data part of an iovar request: "name\0" followed by @p val.
 * @return bytes written or -ENOMEM.
 */
int infsingle_iovar_build(uint8_t *buf, size_t cap, const char *name,
		    const void *val, size_t vlen);

/* ---- BDC / events ------------------------------------------------------- */

/** Fill a BDC header for TX on interface @p ifidx with priority @p prio. */
void infsingle_bdc_fill(struct infsingle_bdc_hdr *h, uint8_t ifidx, uint8_t prio);

/** Map an 802.1D priority (0-7) to an access category 0=BE 1=BK 2=VI 3=VO. */
uint8_t infsingle_prio_to_ac(uint8_t prio);

/** Decoded event, all fields in host order. data points into the frame. */
struct infsingle_event {
	uint32_t type;
	uint32_t status;
	uint32_t reason;
	uint32_t auth_type;
	uint16_t flags;
	uint8_t addr[6];
	uint8_t ifidx;
	const uint8_t *data;
	size_t data_len;
};

/**
 * Decode an event frame (channel 1 payload, starting at the BDC header).
 * @retval 0 ok, -EBADMSG not a Broadcom event / truncated.
 */
int infsingle_event_parse(const uint8_t *buf, size_t len, struct infsingle_event *ev);

/** Set bit @p event in the event_msgs bitmask (INFSINGLE_EVT_MAX bits). */
void infsingle_event_mask_set(uint8_t *mask, unsigned int event);

#endif /* ZEPHYR_DRIVERS_WIFI_INFSINGLE_INFSINGLE_PROTO_H_ */
