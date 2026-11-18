/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file infsingle.h
 * @brief Driver context and inter-module API.
 *
 * Layering (infdocs/plan.md): bus -> chip/boot -> ctrl (ioctl/iovar/events)
 * -> netif (data path) -> mgmt (Wi-Fi management). Each layer only calls the
 * layer below it. One driver thread owns the bus; other threads reach it
 * through ctrl (blocking request/response) or the TX queues.
 *
 * Design summary:
 *
 *  - Transport: the chip is reached over SDIO (function 1 = backplane
 *    registers, function 2 = WLAN packet FIFO). infsingle_bus.h abstracts this
 *    so that other transports could be added without touching upper layers.
 *  - Framing: every packet on function 2 is wrapped in an SDPCM header
 *    (channel 0 = control, 1 = events, 2 = data) carrying sequence numbers and
 *    the flow-control credit limit (infsingle_proto.c).
 *  - Control: ioctls and iovars travel as CDC messages on channel 0, one at a
 *    time, and are answered on channel 0. Firmware events arrive on channel 1.
 *  - Data: Ethernet frames carry a 4-byte BDC header on channel 2.
 *  - Flow control: the firmware grants TX credits; one credit is consumed per
 *    frame sent. Without credit the driver thread polls until more arrive.
 *  - Ownership: struct infsingle_dev is a singleton (one chip per build). All
 *    bus access after boot happens in the driver thread, so the bus layer
 *    needs no locking of its own.
 *  - Blobs: firmware, NVRAM and CLM for the one configured device are linked in
 *    as C arrays generated at build time (gen_blob.py); see infsingle_fw.c.
 */

#ifndef ZEPHYR_DRIVERS_WIFI_INFSINGLE_INFSINGLE_H_
#define ZEPHYR_DRIVERS_WIFI_INFSINGLE_INFSINGLE_H_

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/wifi/infineon_offload.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/wifi_mgmt.h>
#include "infsingle_bus.h"
#include "infsingle_chip.h"
#include "infsingle_proto.h"
#include "infsingle_stats.h"

#define INFSINGLE_NUM_AC       4
#define INFSINGLE_MAX_EVT_CB   4
#define INFSINGLE_CTRL_BUF_LEN 1536

/** Driver-thread wake-up reasons (struct infsingle_dev::wake events). */
#define INFSINGLE_WAKE_IRQ     BIT(0) /* device has data / mailbox event       */
#define INFSINGLE_WAKE_TX      BIT(1) /* a packet was queued                   */
#define INFSINGLE_WAKE_CTRL    BIT(2) /* a control request is pending          */
#define INFSINGLE_WAKE_STOP    BIT(3)

struct infsingle_dev;

typedef void (*infsingle_event_cb_t)(struct infsingle_dev *d, const struct infsingle_event *ev,
			       void *user);

struct infsingle_dev {
	const struct device *dev;
	struct infsingle_bus *bus;
	const struct infsingle_chip *chip;
	struct net_if *iface;
	uint8_t mac[6];
	uint8_t chip_rev;
	bool up;

	/* Driver thread (single owner of the bus once running). */
	struct k_thread thread;
	k_thread_stack_t *stack;
	struct k_event wake;

	/*
	 * Data path. One TX FIFO per WMM access category, served highest first;
	 * txq_len bounds each queue so a stalled link cannot exhaust net_pkt buffers.
	 */
	struct infsingle_credit credit;
	/* Flow-control bits from the latest SDPCM header; data TX waits while set. */
	uint8_t flow_ctl;
	struct k_fifo txq[INFSINGLE_NUM_AC];
	atomic_t txq_len[INFSINGLE_NUM_AC];
	struct infsingle_stats_state stats;
	/* Linearisation buffer for packets that cannot be sent in place. */
	uint8_t tx_scratch[CONFIG_WIFI_AIROC_RX_BUF_SIZE] __aligned(32);
	uint8_t rx_scratch[CONFIG_WIFI_AIROC_RX_BUF_SIZE + 128] __aligned(32);
	struct k_sem boot_done;

	/*
	 * Control path: one outstanding request at a time. ctrl_lock serialises
	 * callers; ctrl_pending_id lets the RX side drop stale replies;
	 * ctrl_tx_len is the hand-off flag to the driver thread (non-zero means
	 * ctrl_tx holds a frame ready to send).
	 */
	struct k_mutex ctrl_lock;
	struct k_sem ctrl_done;
	uint16_t ctrl_id;
	uint16_t ctrl_pending_id;
	uint8_t *ctrl_rsp;
	size_t ctrl_rsp_cap, ctrl_rsp_len;
	int32_t ctrl_status;
	uint8_t ctrl_tx[INFSINGLE_CTRL_BUF_LEN] __aligned(32);
	size_t ctrl_tx_len;      /* bytes of CDC to send; 0 = nothing pending */

	/* Event subscribers: called from the driver thread, so they must not block. */
	struct {
		infsingle_event_cb_t cb;
		void *user;
	} evt_cb[INFSINGLE_MAX_EVT_CB];

	/* Management: scan_cb is non-NULL while a scan runs; scan_sync tags its escan events. */
	struct k_sem scan_done;
	scan_result_cb_t scan_cb;
	uint16_t scan_sync;
	/*
	 * Join state machine: the firmware reports association progress as
	 * separate events (auth, link up, key exchange); join_* track which
	 * have been seen so the join completes only when all required ones did.
	 */
	struct k_sem join_done;
	int join_result;
	bool join_active, join_secured, join_link, join_keyed;
	bool connected;
	enum wifi_security_type sec;   /* security of the current association */
	uint8_t ssid[32];
	uint8_t ssid_len;
	bool ap_mode;    /* soft AP is up on bsscfg 0 */
	bool ap_wait;    /* an AP bring-up/down is waiting for a LINK event */
	struct k_sem ap_done;
	bool fw_sae;     /* firmware advertises SAE in "cap"                 */
	bool fw_offloads;
	bool fw_icmp;
	struct k_spinlock offload_cb_lock;
	wifi_infineon_icmp_callback_t icmp_callback;
	void *icmp_callback_user;
};

/* ---- infsingle_boot.c ---------------------------------------------------------- */
/** Power up, identify, download firmware+NVRAM, start the core. */
int infsingle_boot(struct infsingle_dev *d);

/* ---- infsingle_fw.c ------------------------------------------------------------ */
/** Blob access. Data is streamed; the driver never copies a whole image. */
enum infsingle_blob { INFSINGLE_BLOB_FW, INFSINGLE_BLOB_NVRAM, INFSINGLE_BLOB_CLM };
const uint8_t *infsingle_fw_get(uint16_t chip_id, enum infsingle_blob which, size_t *len);

/* ---- infsingle_ctrl.c ---------------------------------------------------------- */
int infsingle_ioctl(struct infsingle_dev *d, uint32_t cmd, bool set, void *buf, size_t len);
int infsingle_iovar_set(struct infsingle_dev *d, const char *name, const void *val, size_t len);
int infsingle_iovar_get(struct infsingle_dev *d, const char *name, void *out, size_t len);
int infsingle_iovar_set_u32(struct infsingle_dev *d, const char *name, uint32_t v);
/* Set received for variable-length replies; NULL requires the exact output size. */
int infsingle_iovar_xfer(struct infsingle_dev *d, const char *name, bool set,
			const void *input, size_t input_len, void *output, size_t output_len,
			size_t *received);
int infsingle_event_subscribe(struct infsingle_dev *d, infsingle_event_cb_t cb, void *user);
/** Called by the driver thread for every channel-0 payload. */
void infsingle_ctrl_rx(struct infsingle_dev *d, const uint8_t *p, size_t len);
/** Called by the driver thread for every channel-1 payload. */
void infsingle_ctrl_event_rx(struct infsingle_dev *d, const uint8_t *p, size_t len);
/** Post-boot configuration: cap, apsta, country, event mask, CLM, WLC_UP. */
int infsingle_ctrl_postboot(struct infsingle_dev *d, const char *country);
/** Driver thread asks ctrl for the pending request; returns its CDC length
 * (0 if none). The CDC bytes sit at d->ctrl_tx + INFSINGLE_CTRL_CDC_OFF.
 */
#define INFSINGLE_CTRL_CDC_OFF INFSINGLE_SDPCM_HDR_LEN
/** Called by the driver thread after sending the pending control frame. */
void infsingle_ctrl_tx_done(struct infsingle_dev *d);

/* ---- infsingle_netif.c --------------------------------------------------------- */
int infsingle_netif_start(struct infsingle_dev *d);
/** Mark the link up/down for the network stack. */
void infsingle_netif_set_link(struct infsingle_dev *d, bool up);

/** Interface offload table (ethernet api + wifi mgmt), defined in infsingle_netif.c */
extern const struct net_wifi_mgmt_offload infsingle_api;

/* ---- infsingle_mgmt.c ---------------------------------------------------------- */
extern const struct wifi_mgmt_ops infsingle_mgmt_ops;
int infsingle_mgmt_init(struct infsingle_dev *d);

/* ---- infsingle_offload.c ------------------------------------------------------- */
void infsingle_offload_set_caps(struct infsingle_dev *d, const char *caps);
void infsingle_offload_event(struct infsingle_dev *d, const struct infsingle_event *event);

#endif /* ZEPHYR_DRIVERS_WIFI_INFSINGLE_INFSINGLE_H_ */
