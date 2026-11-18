/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file infsingle_netif.c
 * @brief Network interface, zero-copy data path and the driver thread.
 *
 * Layer: netif. Reference: infdocs/hw.md sections 4.4 (F2 transfers), 7
 * (SDPCM, credits), 8.2 (BDC); infdocs/req.md R-PERF-*.
 *
 * ---------------------------------------------------------------------------
 * TRANSMIT (zero copy)
 *
 * The ethernet L2 reserves headroom in front of every TX packet (we report it
 * through ETHERNET_CONFIG_TYPE_EXTRA_TX_PKT_HEADROOM). We write the SDPCM and
 * BDC headers into that headroom and hand the very same memory to the SDIO
 * host controller:
 *
 *   net_buf:  [ ...headroom... | pad | SDPCM(12) | pad | BDC(4) | Ethernet frame | tailroom ]
 *                                     ^ frame start, aligned to the sdhc DMA alignment
 *
 * The SDPCM "header length" field tells the firmware where the BDC header
 * begins, so variable padding after the 12-byte SDPCM header is what lets us
 * align the frame start without moving any payload byte.
 *
 * If a packet cannot be sent in place (several fragments, insufficient
 * headroom or tailroom) it is linearised into tx_scratch and the copied byte
 * count is added to stats.bytes_copied so the cost is visible (decision D-1).
 *
 * RECEIVE (zero copy)
 *
 * The 4-byte SDPCM tag is read first to learn the frame size. A net_pkt of
 * that size is then allocated and the rest of the frame is read straight into
 * it. Headers are removed with net_buf_pull(), never moved:
 *
 *   net_buf:  [ pad | SDPCM(12+) | BDC(4+) | Ethernet frame ]
 *                                          ^ net_buf_pull() leaves data here
 *
 * The pad shifts the buffer so the bulk read (starting at frame offset 4) lands
 * on a DMA-aligned address.
 *
 * END-TO-END DATA FLOW
 *
 *   TX: IP stack -> infsingle_send() -> txq[AC] (k_fifo, one per access category)
 *       -> driver thread: tx_data() -> tx_pkt() (add BDC + SDPCM, take a credit)
 *       -> tx_frame() -> bus->ops->frame_write() -> SDIO/gSPI -> firmware
 *
 *   RX: firmware -> IRQ -> infsingle_irq() posts WAKE_IRQ -> driver thread
 *       -> service_int() (read/ack status) -> rx_one() (read frame, parse SDPCM)
 *       -> by SDPCM channel:
 *            control -> infsingle_ctrl_rx()       completes a pending ioctl/iovar
 *            event   -> infsingle_ctrl_event_rx() -> mgmt callbacks (scan, join, AP)
 *            data    -> strip BDC -> net_recv_data() -> IP stack
 *
 *   Every received frame also carries the firmware's TX credit limit, which
 *   rx_one() feeds to infsingle_credit_update(); tx_pkt()/tx_ctrl() consume one
 *   credit per frame sent.
 *
 * THREADING
 *
 * One driver thread owns the bus after boot. Producers (net TX thread, control
 * requests, the SDIO ISR) only post events or queue packets.
 *
 *   ISR --WAKE_IRQ--+
 *   send() --WAKE_TX-+--> driver thread: service int -> rx frames -> ctrl tx -> data tx
 *   ioctl --WAKE_CTRL+
 */

#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/conn_mgr/connectivity_wifi_mgmt.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>
#include "infsingle.h"
#include "infsingle_wire.h"

LOG_MODULE_REGISTER(infsingle_netif, CONFIG_WIFI_AIROC_LOG_LEVEL);

#define DT_DRV_COMPAT infineon_airoc_wifi

#if DT_INST_ON_BUS(0, spi)
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/pinctrl.h>
#include "infsingle_bus.h"

#define INFSINGLE_SPI_OPERATION                                                                   \
	(SPI_WORD_SET(DT_INST_PROP_OR(0, spi_word_size, 8)) |                                     \
	 (DT_INST_PROP(0, spi_half_duplex) ? SPI_HALF_DUPLEX : SPI_FULL_DUPLEX) | SPI_TRANSFER_MSB)

#if DT_INST_PROP(0, spi_data_irq_shared)
/* The "host_wake" pinctrl state follows "default" in the devicetree */
#define PINCTRL_STATE_HOST_WAKE PINCTRL_STATE_PRIV_START
PINCTRL_DT_INST_DEFINE(0);
#define INFSINGLE_PCFG PINCTRL_DT_INST_DEV_CONFIG_GET(0)
#else
#define INFSINGLE_PCFG NULL
#endif
#endif /* DT_INST_ON_BUS(0, spi) */

#ifdef CONFIG_SDHC_BUFFER_ALIGNMENT
#define INFSINGLE_ALIGN CONFIG_SDHC_BUFFER_ALIGNMENT
#else
#define INFSINGLE_ALIGN 4
#endif

/* Worst case headroom: SDPCM + BDC plus alignment slack. */
#define INFSINGLE_TX_HEADROOM_MAX (INFSINGLE_SDPCM_HDR_LEN + sizeof(struct infsingle_bdc_hdr) + (INFSINGLE_ALIGN - 1))

/* Bus read-modify-write bound on how many frames we drain per wake-up. */
#define INFSINGLE_RX_BURST 16
#define INFSINGLE_POLL_MS 50

#define INFSINGLE_RX_ALLOC_TIMEOUT K_MSEC(20)
/* Credits data may not use, so control requests still get through (as brcmfmac). */
#define INFSINGLE_TXCTL_CREDITS 2

static struct infsingle_dev infsingle_dev_data;

K_THREAD_STACK_DEFINE(infsingle_stack, CONFIG_WIFI_AIROC_THREAD_STACK_SIZE);

/* ------------------------------------------------------------------------- */
/* Interrupt servicing                                                       */
/* ------------------------------------------------------------------------- */

static void infsingle_irq(struct infsingle_bus *bus, void *user)
{
	struct infsingle_dev *d = user;

	/*
	 * Runs in interrupt context: no bus access here (the bus transfer can
	 * sleep). Just wake the driver thread, which does all device I/O.
	 * Level-sensitive source: mask until the thread has drained the device.
	 */
	(void)bus->ops->irq_mask(bus, false);
	k_event_post(&d->wake, INFSINGLE_WAKE_IRQ);
}

/* Read and acknowledge the SDIO core interrupt status; returns frame bits. */
static int service_int(struct infsingle_dev *d, uint32_t *frames)
{
	struct infsingle_bus *bus = d->bus;
	uint32_t base = d->chip->sdio_core;
	uint32_t status = 0, mbx = 0;
	int ret;

	*frames = 0;
	ret = bus->ops->bp_read32(bus, base + INFSINGLE_SDIOCORE_INT_STATUS, &status);
	if (ret) {
		return ret;
	}
	if (status & INFSINGLE_INT_HMB_HOST_INT) {
		ret = bus->ops->bp_read32(bus, base + INFSINGLE_SDIOCORE_TO_HOST_MBX_DATA, &mbx);
		if (ret == 0 && mbx != 0) {
			(void)bus->ops->bp_write32(bus, base + INFSINGLE_SDIOCORE_TO_SB_MAILBOX,
						   INFSINGLE_SMB_INT_ACK);
		}
		if (mbx & INFSINGLE_HMB_DATA_FWHALT) {
			INFSINGLE_STATS_ADD(&d->stats, fw_halts, 1U);
			LOG_ERR("firmware halted");
		}
	}
	/* Write-1-to-clear the bits we are going to handle. */
	if (status) {
		(void)bus->ops->bp_write32(bus, base + INFSINGLE_SDIOCORE_INT_STATUS, status);
	}
	*frames = status & INFSINGLE_INT_FRAME_AVAILABLE_MASK;
	return 0;
}

/* ------------------------------------------------------------------------- */
/* Receive                                                                   */
/* ------------------------------------------------------------------------- */

/*
 * Read one frame. Returns 1 if a frame was consumed, 0 if the device had none,
 * negative on bus error.
 *
 * Steps:
 *   1. Learn the frame length (gSPI: rx_len(); SDIO: read the 4-byte tag).
 *   2. Allocate a net_pkt big enough for the whole frame, or fall back to
 *      rx_scratch. The frame start is shifted by "pad" so the bulk read is
 *      DMA aligned.
 *   3. Read the remaining bytes of the frame into that buffer.
 *   4. Parse the SDPCM header: record the credit, find the channel.
 *   5. Dispatch by channel. For data frames the SDPCM/BDC headers are removed
 *      with net_buf_pull() and the packet goes to the stack. For control and
 *      event frames the payload is consumed in place and the packet is freed.
 */
static int rx_one(struct infsingle_dev *d)
{
	struct infsingle_bus *bus = d->bus;
	uint8_t tag[INFSINGLE_ALIGN > 4 ? INFSINGLE_ALIGN : 4] __aligned(INFSINGLE_ALIGN);
	struct infsingle_rx_frame f;
	struct net_pkt *pkt;
	struct net_buf *buf;
	size_t size, rest, pad;
	bool have_tag = true;
	uint8_t *p;
	int ret;

	/*
	 * SDIO: the SDPCM tag read first tells the frame length, so the
	 * remaining bytes can go directly into a right-sized buffer.
	 * gSPI: the device reports the length up front and the whole frame is
	 * read in one transfer.
	 */
	if (bus->ops->rx_len != NULL) {
		size_t n;

		ret = bus->ops->rx_len(bus, &n);
		if (ret) {
			infsingle_stats_rx_bus_error(&d->stats);
			return ret;
		}
		if (n == 0) {
			return 0;
		}
		size = n;
		have_tag = false;
	} else {
		ret = bus->ops->frame_read(bus, tag, 4);
		if (ret) {
			(void)bus->ops->abort_read(bus);
			infsingle_stats_rx_bus_error(&d->stats);
			return ret;
		}
		/*
		 * The tag is only valid if its length and complement agree. A zero
		 * size means the device FIFO is empty (not an error).
		 */
		size = infsingle_sdpcm_check_tag(tag);
		if (size == 0) {
			return 0;
		}
	}
	if (size > CONFIG_WIFI_AIROC_RX_BUF_SIZE) {
		/* Cannot buffer it; abort so the device drops the frame. */
		(void)bus->ops->abort_read(bus);
		infsingle_stats_drop(&d->stats, false, true);
		return 1;
	}

	/*
	 * Pad so that frame offset 4 (where the bulk read starts) is aligned.
	 * A net_pkt is used directly only if it is a single contiguous net_buf;
	 * otherwise the frame is read into rx_scratch and copied (counted in
	 * bytes_copied) so correctness never depends on the net_buf size.
	 */
	rest = infsingle_bus_round(bus, have_tag ? size - 4 : size);
	if (INFSINGLE_ALIGN + 4 + rest <= CONFIG_NET_BUF_DATA_SIZE) {
		pkt = net_pkt_rx_alloc_with_buffer(d->iface, INFSINGLE_ALIGN + 4 + rest, AF_UNSPEC, 0,
						   INFSINGLE_RX_ALLOC_TIMEOUT);
	} else {
		pkt = NULL;
	}
	if (pkt != NULL && (pkt->buffer == NULL || pkt->buffer->frags != NULL)) {
		/* Not contiguous (small net_buf data size): use the scratch buffer. */
		net_pkt_unref(pkt);
		pkt = NULL;
	}
	if (pkt != NULL) {
		buf = pkt->buffer;
		pad = (INFSINGLE_ALIGN - (((uintptr_t)buf->data + 4) & (INFSINGLE_ALIGN - 1))) &
		      (INFSINGLE_ALIGN - 1);
		p = (uint8_t *)net_buf_add(buf, pad + size) + pad; /* frame start */
	} else {
		buf = NULL;
		pad = (INFSINGLE_ALIGN - (((uintptr_t)d->rx_scratch + 4) & (INFSINGLE_ALIGN - 1))) &
		      (INFSINGLE_ALIGN - 1);
		p = d->rx_scratch + pad;
		if (pad + 4 + rest > sizeof(d->rx_scratch)) {
			(void)bus->ops->abort_read(bus);
			infsingle_stats_drop(&d->stats, false, true);
			return 1;
		}
	}
	if (have_tag) {
		memcpy(p, tag, 4);                      /* the only copy: 4 header bytes */
	}

	if (rest > 0) {
		ret = bus->ops->frame_read(bus, have_tag ? p + 4 : p, rest);
		if (ret) {
			LOG_ERR("rx body read %zu failed: %d", rest, ret);
			infsingle_stats_rx_bus_error(&d->stats);
			if (!have_tag) {
				(void)bus->ops->abort_read(bus);
			}
			if (pkt != NULL) {
				net_pkt_unref(pkt);
			}
			return ret;
		}
	}

	ret = infsingle_sdpcm_parse(p, size, &f);
	/* Every frame header carries the current flow-control state (as in WHD). */
	if (ret == 0 || ret == -ENODATA) {
		d->flow_ctl = f.flow_ctl;
	}
	/* -ENODATA: a credit-only frame with no payload; nothing to deliver. */
	if (ret == -ENODATA) {
		infsingle_credit_update(&d->credit, f.credit);
		if (pkt != NULL) {
			net_pkt_unref(pkt);
		}
		return 1;
	}
	if (ret) {
		infsingle_stats_drop(&d->stats, false, false);
		if (pkt != NULL) {
			net_pkt_unref(pkt);
		}
		return 1;
	}
	/* Every received frame, whatever its channel, may raise the TX credit limit. */
	if (f.chan <= INFSINGLE_SDPCM_CHAN_DATA) {
		infsingle_credit_update(&d->credit, f.credit);
	}

	/* Dispatch on the SDPCM channel field. */
	switch (f.chan) {
	case INFSINGLE_SDPCM_CHAN_CONTROL:
		/* Reply to our ioctl/iovar request (CDC): wakes the waiting caller. */
		infsingle_ctrl_rx(d, f.payload, f.payload_len);
		break;
	case INFSINGLE_SDPCM_CHAN_EVENT:
		/* Asynchronous firmware event (join, link, scan result, ...). */
		infsingle_ctrl_event_rx(d, f.payload, f.payload_len);
		break;
	case INFSINGLE_SDPCM_CHAN_DATA: {
		/* BDC header may carry extra words before the Ethernet frame. */
		size_t bdc;

		if (f.payload_len < sizeof(struct infsingle_bdc_hdr)) {
			infsingle_stats_drop(&d->stats, false, false);
			break;
		}
		bdc = sizeof(struct infsingle_bdc_hdr) + (size_t)f.payload[3] * 4U;
		if (f.payload_len < bdc + sizeof(struct net_eth_hdr) || d->iface == NULL) {
			infsingle_stats_drop(&d->stats, false, false);
			break;
		}
		if (pkt == NULL) {
			size_t elen = f.payload_len - bdc;

			pkt = net_pkt_rx_alloc_with_buffer(d->iface, elen, AF_UNSPEC, 0,
							   INFSINGLE_RX_ALLOC_TIMEOUT);
			if (pkt == NULL) {
				infsingle_stats_drop(&d->stats, false, true);
				break;
			}
			if (net_pkt_write(pkt, f.payload + bdc, elen) < 0) {
				infsingle_stats_drop(&d->stats, false, false);
				break; /* falls to unref below */
			}
			INFSINGLE_STATS_ADD(&d->stats, bytes_copied, elen);
			infsingle_stats_packet(&d->stats, false, f.payload + bdc, elen);
			ret = net_recv_data(d->iface, pkt);
			if (ret < 0) {
				infsingle_stats_drop(&d->stats, false,
						    ret == -ENOBUFS || ret == -ENOMEM);
				break;
			}
			return 1;
		}
		/* Zero-copy: drop pad + SDPCM + BDC so buf->data is the Ethernet frame. */
		net_buf_pull(buf, pad + (size_t)(f.payload + bdc - p));
		infsingle_stats_packet(&d->stats, false, buf->data, f.payload_len - bdc);
		ret = net_recv_data(d->iface, pkt);
		if (ret < 0) {
			infsingle_stats_drop(&d->stats, false, ret == -ENOBUFS || ret == -ENOMEM);
			break; /* falls to unref below */
		}
		return 1; /* ownership passed to the stack */
	}
	default:
		break;
	}
	if (pkt != NULL) {
		net_pkt_unref(pkt);
	}
	return 1;
}

/* ------------------------------------------------------------------------- */
/* Transmit                                                                    */
/* ------------------------------------------------------------------------- */

/* Single point where a finished SDPCM frame is written to the bus. */
static int tx_frame(struct infsingle_dev *d, const void *frame, size_t total)
{
	struct infsingle_bus *bus = d->bus;
	int ret = bus->ops->frame_write(bus, frame, infsingle_bus_round(bus, total));

	if (ret) {
		INFSINGLE_STATS_ADD(&d->stats, bus_errors, 1U);
	}
	return ret;
}

/* Send the pending control request (if any and credit allows). */
static void tx_ctrl(struct infsingle_dev *d)
{
	size_t cdc = d->ctrl_tx_len;

	if (cdc == 0 || infsingle_credit_avail(&d->credit) == 0) {
		return;
	}
	infsingle_sdpcm_fill(d->ctrl_tx, INFSINGLE_SDPCM_HDR_LEN + cdc, INFSINGLE_SDPCM_CHAN_CONTROL,
		       infsingle_credit_take(&d->credit));
	if (tx_frame(d, d->ctrl_tx, INFSINGLE_SDPCM_HDR_LEN + cdc) == 0) {
		infsingle_ctrl_tx_done(d);
	} else {
		LOG_ERR("ctrl tx failed");
	}
}

/*
 * The bus rounds the transfer up by a few bytes that the DMA reads but the
 * firmware ignores. net_pkt trims a fixed-pool buffer's size to the requested
 * length, so the padding may come from the unused rest of the pool slot.
 */
static bool tx_tail_ok(struct net_buf *buf, size_t need)
{
	const struct net_buf_pool *pool;

	if (net_buf_tailroom(buf) >= need) {
		return true;
	}
	pool = net_buf_pool_get(buf->pool_id);
	return pool->alloc->cb == &net_buf_fixed_cb &&
	       net_buf_tail(buf) + need <= buf->__buf + pool->alloc->max_alloc_size;
}

/* Send one data packet. Takes ownership of the reference held in the queue. */
static void tx_pkt(struct infsingle_dev *d, struct net_pkt *pkt)
{
	struct net_buf *buf = pkt->buffer;
	size_t len = net_pkt_get_len(pkt);
	uint8_t prio = net_pkt_priority(pkt);
	uint8_t *frame, *bdc;
	size_t hdr_len, total;
	bool in_place = false;

	/*
	 * Zero-copy test: the packet must be one contiguous net_buf with enough
	 * headroom for SDPCM+BDC (plus alignment) in front of the Ethernet frame
	 * and enough tailroom to round the transfer up to the bus block size.
	 */
	if (buf != NULL && buf->frags == NULL) {
		bdc =  buf->data - sizeof(struct infsingle_bdc_hdr);
		frame = (uint8_t *)ROUND_DOWN((uintptr_t)bdc - INFSINGLE_SDPCM_HDR_LEN, INFSINGLE_ALIGN);
		hdr_len = (size_t)(bdc - frame);
		in_place = hdr_len <= 255 &&
			   net_buf_headroom(buf) >= (size_t)(buf->data - frame) &&
			   tx_tail_ok(buf, infsingle_bus_round(d->bus, len + 4 + hdr_len) -
					   (len + 4 + hdr_len));
	}
	if (!in_place) {
		if (len + INFSINGLE_SDPCM_HDR_LEN + INFSINGLE_SDPCM_DATA_PAD + 4 > sizeof(d->tx_scratch)) {
			infsingle_stats_drop(&d->stats, true, true);
			goto out;
		}
		frame = d->tx_scratch;
		hdr_len = INFSINGLE_SDPCM_HDR_LEN + INFSINGLE_SDPCM_DATA_PAD;
		bdc = frame + hdr_len;
		net_pkt_cursor_init(pkt);
		if (net_pkt_read(pkt, bdc + 4, len) < 0) {
			infsingle_stats_drop(&d->stats, true, false);
			goto out;
		}
		INFSINGLE_STATS_ADD(&d->stats, bytes_copied, len);
	}

	/*
	 * Fill the headers in front of the frame (in the headroom, or in
	 * tx_scratch on the copy path): BDC (priority), then SDPCM (length,
	 * sequence number taken from the credit counter, header length).
	 */
	total = hdr_len + 4 + len;
	infsingle_bdc_fill((struct infsingle_bdc_hdr *)bdc, 0, prio);
	infsingle_sdpcm_fill(frame, total, INFSINGLE_SDPCM_CHAN_DATA, infsingle_credit_take(&d->credit));
	frame[7] = (uint8_t)hdr_len;

	if (tx_frame(d, frame, total) == 0) {
		infsingle_stats_packet(&d->stats, true,
				      bdc + sizeof(struct infsingle_bdc_hdr), len);
	} else {
		infsingle_stats_drop(&d->stats, true, false);
	}
out:
	/* Whether sent or dropped: release the queue slot and our packet reference. */
	atomic_dec(&d->txq_len[infsingle_prio_to_ac(prio)]);
	net_pkt_unref(pkt);
}

/* Drain the queues, highest priority first, while credit lasts. */
static bool tx_data(struct infsingle_dev *d)
{
	bool more = false;

	/* Highest access category (voice) first. Returns true if credit ran out. */
	if (d->flow_ctl != 0) {
		for (int ac = 0; ac < INFSINGLE_NUM_AC; ac++) {
			if (!k_fifo_is_empty(&d->txq[ac])) {
				INFSINGLE_STATS_ADD(&d->stats, tx_flow_ctl, 1U);
				return true;
			}
		}
		return false;
	}
	for (int ac = INFSINGLE_NUM_AC - 1; ac >= 0; ac--) {
		struct net_pkt *pkt;

		while (!k_fifo_is_empty(&d->txq[ac])) {
			if (infsingle_credit_avail(&d->credit) <= INFSINGLE_TXCTL_CREDITS) {
				INFSINGLE_STATS_ADD(&d->stats, tx_no_credit, 1U);
				return true;
			}
			pkt = k_fifo_get(&d->txq[ac], K_NO_WAIT);
			if (pkt == NULL) {
				break;
			}
			tx_pkt(d, pkt);
		}
	}
	return more;
}

/*
 * Enqueue only: the stack's TX thread must not do bus I/O. The queue holds its
 * own packet reference, released in tx_pkt() after the frame is on the wire.
 */
static int infsingle_send(const struct device *dev, struct net_pkt *pkt)
{
	struct infsingle_dev *d = dev->data;
	uint8_t ac = infsingle_prio_to_ac(net_pkt_priority(pkt));

	if (!d->up || atomic_get(&d->txq_len[ac]) >= CONFIG_WIFI_AIROC_TX_QUEUE_DEPTH) {
		infsingle_stats_drop(&d->stats, true, d->up);
		return -ENOBUFS;
	}
	atomic_inc(&d->txq_len[ac]);
	/* The queue holds its own reference; the stack drops its own on return. */
	k_fifo_put(&d->txq[ac], net_pkt_ref(pkt));
	k_event_post(&d->wake, INFSINGLE_WAKE_TX);
	return 0;
}

/* ------------------------------------------------------------------------- */
/* Driver thread                                                             */
/* ------------------------------------------------------------------------- */

/* Credit starvation longer than this triggers a poke. */
#define INFSINGLE_POKE_MS 20

struct poke_state {
	int64_t since;   /* uptime when the credit limit last changed */
	uint8_t tx_max;  /* credit limit seen at that time */
};

/*
 * Under sustained load the firmware occasionally stops returning TX credit
 * with frames queued, and only resumes when the host sends it another frame
 * (a mailbox SMB_DEV_INT poke, as WHD uses, does not help). So while data
 * waits for credit, a harmless control request (WLC_GET_MAGIC) is sent with
 * one of the reserved control credits. Its reply carries id 0, which never
 * matches a pending request, and is discarded.
 */
static void tx_poke(struct infsingle_dev *d, bool starved, struct poke_state *ps)
{
	static uint8_t frame[64] __aligned(32);
	int64_t now = k_uptime_get();
	int n;

	if (!starved || d->credit.tx_max != ps->tx_max) {
		ps->tx_max = d->credit.tx_max;
		ps->since = now;
		return;
	}
	if (now - ps->since < INFSINGLE_POKE_MS || d->ctrl_tx_len != 0 ||
	    infsingle_credit_avail(&d->credit) == 0) {
		return;
	}
	ps->since = now;
	n = infsingle_cdc_build(frame + INFSINGLE_CTRL_CDC_OFF, sizeof(frame) - INFSINGLE_CTRL_CDC_OFF,
				INFSINGLE_WLC_GET_MAGIC, false, 0, 0, NULL, 4);
	if (n < 0) {
		return;
	}
	infsingle_sdpcm_fill(frame, INFSINGLE_SDPCM_HDR_LEN + n, INFSINGLE_SDPCM_CHAN_CONTROL,
			     infsingle_credit_take(&d->credit));
	INFSINGLE_STATS_ADD(&d->stats, tx_pokes, 1U);
	(void)tx_frame(d, frame, INFSINGLE_SDPCM_HDR_LEN + n);
}

/*
 * Driver thread main loop. Wake sources: SDIO interrupt, a queued TX packet, a
 * pending control request, or the poll timeout. Each pass:
 *   1. service the device interrupt and drain received frames (bounded burst),
 *   2. send the pending control request,
 *   3. send queued data while TX credit lasts.
 * RX is handled before TX so that credit updates carried by received frames
 * are seen before deciding whether TX is blocked. If TX is blocked on credit
 * (or RX hit the burst limit) the thread re-polls quickly instead of waiting
 * for an interrupt that may not come, since credits arrive inside RX frames.
 */
static void infsingle_thread(void *p1, void *p2, void *p3)
{
	struct infsingle_dev *d = p1;
	struct poke_state poke = {0};
	bool blocked = false;
	bool pending = false;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	d->bus->ops->irq_set(d->bus, infsingle_irq, d);
	d->bus->ops->irq_mask(d->bus, true);
	k_event_post(&d->wake, INFSINGLE_WAKE_IRQ); /* catch anything pending from boot */

	for (;;) {
		/*
		 * pending: a full RX burst left frames in the device. Their interrupt status was
		 * already acknowledged, so no new interrupt will announce them: carry on at once.
		 */
		uint32_t ev = k_event_wait(&d->wake,
					   INFSINGLE_WAKE_IRQ | INFSINGLE_WAKE_TX | INFSINGLE_WAKE_CTRL |
					   INFSINGLE_WAKE_STOP,
					   false,
					   pending ? K_NO_WAIT :
					   blocked ? K_MSEC(5) : K_MSEC(INFSINGLE_POLL_MS));
		bool more = false;

		k_event_clear(&d->wake, ev);
		if (ev & INFSINGLE_WAKE_STOP) {
			break;
		}
		/* ev == 0 is the poll timeout: covers an interrupt lost while masked. */
		if (ev == 0 || (ev & INFSINGLE_WAKE_IRQ) || blocked || pending) {
			uint32_t frames = 0;
			int sret = pending ? 0 : service_int(d, &frames);

			/*
			 * Read the FIFO when the device reported a frame, one is known to remain,
			 * or on a poll pass (timeout or TX blocked). The firmware does not always
			 * raise a frame indication; a frame left unread can hold the buffers it
			 * needs to return TX credit, which stalls TX for good.
			 */
			if (sret == 0 && (pending || frames != 0 || blocked || ev == 0)) {
				int i;

				for (i = 0; i < INFSINGLE_RX_BURST; i++) {
					if (rx_one(d) <= 0) {
						break;
					}
				}
				if (frames == 0 && !pending && i > 0) {
					INFSINGLE_STATS_ADD(&d->stats, rx_unsignaled, (uint32_t)i);
				}
				/* Frames may remain; their status bit is already cleared. */
				more = (i == INFSINGLE_RX_BURST);
			}
			d->bus->ops->irq_mask(d->bus, true);
		}
		pending = more;
		tx_ctrl(d);
		blocked = tx_data(d) || d->ctrl_tx_len != 0 || more;
		tx_poke(d, blocked && infsingle_credit_avail(&d->credit) <= INFSINGLE_TXCTL_CREDITS,
			&poke);
	}
}

int infsingle_netif_start(struct infsingle_dev *d)
{
	/* Initial credit before the first frame carries a limit (as WHD does). */
	d->credit.tx_seq = 0;
	d->credit.tx_max = 4;
	k_thread_create(&d->thread, infsingle_stack, K_THREAD_STACK_SIZEOF(infsingle_stack), infsingle_thread, d,
			NULL, NULL, K_PRIO_COOP(CONFIG_WIFI_AIROC_THREAD_PRIORITY), 0, K_NO_WAIT);
	k_thread_name_set(&d->thread, "infsingle");
	d->up = true;
	return 0;
}

#ifdef CONFIG_SHELL
#include <zephyr/shell/shell.h>

static int cmd_stats(const struct shell *sh, size_t argc, char **argv)
{
	struct infsingle_stats snapshot;
	const struct infsingle_stats *s = &snapshot;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	infsingle_stats_snapshot(&infsingle_dev_data.stats, &snapshot);
	shell_print(sh, "tx pkts %u bytes %llu dropped %u no_credit %u flow_ctl %u",
		    s->wifi.pkts.tx, (unsigned long long)s->wifi.bytes.sent,
		    s->tx_dropped, s->tx_no_credit, s->tx_flow_ctl);
	shell_print(sh, "rx pkts %u bytes %llu dropped %u unsignaled %u pokes %u", s->wifi.pkts.rx,
		    (unsigned long long)s->wifi.bytes.received, s->rx_dropped, s->rx_unsignaled,
		    s->tx_pokes);
	shell_print(sh, "bytes_copied %llu bus_errors %u fw_halts %u",
		    (unsigned long long)s->bytes_copied,
		    s->bus_errors, s->fw_halts);
	shell_print(sh, "credit tx_seq %u tx_max %u flow_ctl 0x%02x", infsingle_dev_data.credit.tx_seq,
		    infsingle_dev_data.credit.tx_max, infsingle_dev_data.flow_ctl);
	return 0;
}

static int cmd_stats_reset(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(sh);
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	infsingle_stats_reset(&infsingle_dev_data.stats);
	return 0;
}

static int cmd_link(const struct shell *sh, size_t argc, char **argv)
{
	struct infsingle_dev *d = &infsingle_dev_data;
	uint32_t v[2] = {0};

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (infsingle_ioctl(d, INFSINGLE_WLC_GET_RSSI, false, &v[0], 4) == 0) {
		shell_print(sh, "rssi %d dBm", (int)v[0]);
	}
	if (infsingle_ioctl(d, 12 /* WLC_GET_RATE */, false, &v[1], 4) == 0) {
		shell_print(sh, "rate %u kbps", (unsigned int)(v[1] * 500U));
	}
	if (infsingle_iovar_get(d, "chanspec", &v[0], 4) == 0) {
		shell_print(sh, "chanspec 0x%04x", (unsigned int)v[0]);
	}
	if (infsingle_iovar_get(d, "nrate", &v[0], 4) == 0) {
		shell_print(sh, "nrate 0x%08x", (unsigned int)v[0]);
	}
	if (infsingle_ioctl(d, 85 /* WLC_GET_PM */, false, &v[0], 4) == 0) {
		shell_print(sh, "pm %u", (unsigned int)v[0]);
	}
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(infsingle_cmds,
	SHELL_CMD(link, NULL, "Show RSSI and rate", cmd_link),
	SHELL_CMD(stats, NULL, "Show driver counters", cmd_stats),
	SHELL_CMD(reset, NULL, "Clear driver counters", cmd_stats_reset),
	SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(infsingle, &infsingle_cmds, "AIROC Wi-Fi driver", NULL);
#endif /* CONFIG_SHELL */

/* ------------------------------------------------------------------------- */
/* Network interface glue                                                    */
/* ------------------------------------------------------------------------- */

void infsingle_netif_set_link(struct infsingle_dev *d, bool up)
{
	if (d->iface == NULL) {
		return;
	}
	if (up) {
		net_if_dormant_off(d->iface);
	} else {
		net_if_dormant_on(d->iface);
	}
}

/*
 * The interface starts dormant and is turned on only when a join completes, so
 * the stack does not run DHCP or transmit before the link exists.
 */
static void infsingle_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct infsingle_dev *d = dev->data;

	d->iface = iface;
	net_eth_set_if_type_wifi(iface);
	net_if_set_link_addr(iface, d->mac, sizeof(d->mac), NET_LINK_ETHERNET);
	ethernet_init(iface);
	net_if_dormant_on(iface);
	infsingle_mgmt_init(d);
}

static enum ethernet_hw_caps infsingle_caps(const struct device *dev, struct net_if *iface)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(iface);
	return 0;
}

static int infsingle_get_config(const struct device *dev, struct net_if *iface,
			  enum ethernet_config_type type, struct ethernet_config *config)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(iface);

	if (type == ETHERNET_CONFIG_TYPE_EXTRA_TX_PKT_HEADROOM) {
		config->extra_tx_pkt_headroom = INFSINGLE_TX_HEADROOM_MAX;
		return 0;
	}
	return -ENOTSUP;
}

const struct net_wifi_mgmt_offload infsingle_api = {
	.wifi_iface.iface_api.init = infsingle_iface_init,
	.wifi_iface.send = infsingle_send,
	.wifi_iface.get_capabilities = infsingle_caps,
	.wifi_iface.get_config = infsingle_get_config,
	.wifi_mgmt_api = &infsingle_mgmt_ops,
};

/*
 * Drive the chip's WL_GPIO pins (the onboard LED on Pico W boards) through the
 * "gpioout" iovar, which takes a pin mask and the new levels.
 */
int airoc_wifi_gpio_out(uint32_t mask, uint32_t value)
{
	struct infsingle_dev *d = &infsingle_dev_data;
	uint32_t v[2] = {sys_cpu_to_le32(mask), sys_cpu_to_le32(value)};

	if (!d->up) {
		return -EAGAIN;
	}
	return infsingle_iovar_set(d, "gpioout", v, sizeof(v));
}

/* ------------------------------------------------------------------------- */
/* Device                                                                    */
/* ------------------------------------------------------------------------- */

/*
 * Boot-time probe: powers the chip, downloads firmware and configures it. This
 * runs during device init (POST_KERNEL), so the first net_if bring-up already
 * has a working firmware and a MAC address. The driver thread is started
 * between boot and post-boot because post-boot ioctls need it to service them.
 */
static int infsingle_init(const struct device *dev)
{
	struct infsingle_dev *d = dev->data;
	static const struct gpio_dt_spec reg_on = GPIO_DT_SPEC_INST_GET(0, wifi_reg_on_gpios);
#if DT_INST_NODE_HAS_PROP(0, wifi_host_wake_gpios)
	static const struct gpio_dt_spec host_wake =
		GPIO_DT_SPEC_INST_GET(0, wifi_host_wake_gpios);
	const struct gpio_dt_spec *hw = &host_wake;
#else
	const struct gpio_dt_spec *hw = NULL;
#endif
	int ret;

	d->dev = dev;
	d->iface = net_if_lookup_by_dev(dev); /* needed before the first RX frame */
	d->stack = infsingle_stack;
#if DT_INST_ON_BUS(0, spi)
	static const struct spi_dt_spec spi = SPI_DT_SPEC_INST_GET(0, INFSINGLE_SPI_OPERATION);
	static const struct gpio_dt_spec bus_sel = GPIO_DT_SPEC_INST_GET(0, bus_select_gpios);

	d->bus = infsingle_bus_spi_create(&spi, &reg_on, &bus_sel, hw, INFSINGLE_PCFG);
#else
	d->bus = infsingle_bus_sdio_create(DEVICE_DT_GET(DT_BUS(DT_DRV_INST(0))), &reg_on, hw);
#endif
	k_event_init(&d->wake);
	k_mutex_init(&d->ctrl_lock);
	k_sem_init(&d->ctrl_done, 0, 1);
	k_sem_init(&d->scan_done, 0, 1);
	k_sem_init(&d->join_done, 0, 1);
	k_sem_init(&d->ap_done, 0, 1);
	for (int i = 0; i < INFSINGLE_NUM_AC; i++) {
		k_fifo_init(&d->txq[i]);
	}

	ret = infsingle_boot(d);
	if (ret) {
		return ret;
	}
	infsingle_netif_start(d);
	ret = infsingle_ctrl_postboot(d, DT_INST_PROP_OR(0, country_code, "XX"));
	if (ret) {
		LOG_ERR("post-boot configuration failed: %d", ret);
	}
	return ret;
}

NET_DEVICE_DT_INST_DEFINE(0, infsingle_init, NULL, &infsingle_dev_data, NULL, CONFIG_WIFI_INIT_PRIORITY,
			  &infsingle_api, ETHERNET_L2, NET_L2_GET_CTX_TYPE(ETHERNET_L2),
			  NET_ETH_MTU);

CONNECTIVITY_WIFI_MGMT_BIND(Z_DEVICE_DT_DEV_ID(DT_DRV_INST(0)));
