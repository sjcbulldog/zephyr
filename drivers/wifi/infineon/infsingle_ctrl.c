/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file infsingle_ctrl.c
 * @brief Control path: ioctl/iovar engine, event dispatch, post-boot setup.
 *
 * Layer: ctrl. Reference: infdocs/hw.md sections 8 (CDC), 9 (CLM, post-boot),
 * 11 (events).
 *
 * Request/response flow (one request outstanding at a time):
 *
 *   caller thread                    driver thread              device
 *   -------------                    -------------              ------
 *   lock; build CDC in ctrl_tx
 *   ctrl_tx_len = n; signal WAKE_CTRL
 *   wait ctrl_done  ----------------> credit available?
 *                                     SDPCM-wrap, frame_write -> request
 *                                     <--- reply on channel 0 --
 *                     <-------------- infsingle_ctrl_rx(): match id, copy, give sem
 *   return status
 *
 * Replies are matched on the request id carried in CDC flags bits 31:16;
 * stale replies (after a timeout) are ignored.
 *
 * Buffer ownership: ctrl_tx belongs to the caller until infsingle_ctrl_tx_done();
 * the response is copied once into the caller's buffer (control traffic only,
 * not counted as data-path copying).
 */

#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>
#include "infsingle.h"
#include "infsingle_wire.h"

LOG_MODULE_REGISTER(infsingle_ctrl, CONFIG_WIFI_AIROC_LOG_LEVEL);

#define CDC_DATA(d) ((d)->ctrl_tx + INFSINGLE_CTRL_CDC_OFF + sizeof(struct infsingle_cdc_hdr))
#define CDC_DATA_CAP (INFSINGLE_CTRL_BUF_LEN - INFSINGLE_CTRL_CDC_OFF - sizeof(struct infsingle_cdc_hdr))

/* Caller holds ctrl_lock and has placed @p req_len payload bytes at CDC_DATA. */
static int do_ioctl(struct infsingle_dev *d, uint32_t cmd, bool set, size_t req_len,
		    void *out, size_t out_cap)
{
	int n, ret;
	uint16_t id = ++d->ctrl_id;

	n = infsingle_cdc_build(d->ctrl_tx + INFSINGLE_CTRL_CDC_OFF, INFSINGLE_CTRL_BUF_LEN - INFSINGLE_CTRL_CDC_OFF,
			  cmd, set, id, 0, CDC_DATA(d), req_len);
	if (n < 0) {
		return n;
	}
	d->ctrl_pending_id = id;
	d->ctrl_rsp = out;
	d->ctrl_rsp_cap = out_cap;
	d->ctrl_rsp_len = 0;
	k_sem_reset(&d->ctrl_done);
	/*
	 * Hand the request to the driver thread: ctrl_tx_len != 0 marks it
	 * pending, WAKE_CTRL wakes the thread, which sends it via tx_ctrl() once
	 * a TX credit is available. The reply arrives on the control channel and
	 * is matched to this request by id in infsingle_ctrl_rx().
	 */
	d->ctrl_tx_len = n;
	k_event_post(&d->wake, INFSINGLE_WAKE_CTRL);

	/*
	 * On timeout ctrl_pending_id is cleared so a late reply is discarded in
	 * infsingle_ctrl_rx() rather than completing the next request, and the
	 * queued frame is withdrawn if the thread has not sent it yet.
	 */
	ret = k_sem_take(&d->ctrl_done, K_MSEC(CONFIG_WIFI_AIROC_IOCTL_TIMEOUT_MS));
	d->ctrl_pending_id = 0;
	d->ctrl_rsp = NULL;
	if (ret) {
		LOG_ERR("ioctl %u timed out", cmd);
		d->ctrl_tx_len = 0;
		return -ETIMEDOUT;
	}
	return d->ctrl_status ? -EIO : 0;
}

int infsingle_ioctl(struct infsingle_dev *d, uint32_t cmd, bool set, void *buf, size_t len)
{
	int ret;

	if (len > CDC_DATA_CAP) {
		return -EMSGSIZE;
	}
	k_mutex_lock(&d->ctrl_lock, K_FOREVER);
	if (len > 0) {
		memcpy(CDC_DATA(d), buf, len);
	}
	ret = do_ioctl(d, cmd, set, len, buf, len);
	k_mutex_unlock(&d->ctrl_lock);
	return ret;
}

/*
 * iovars are ioctls WLC_GET_VAR/WLC_SET_VAR whose payload is the NUL-terminated
 * name followed by the value. A GET returns its value in the same buffer, so
 * the request buffer is zero padded to the expected reply size.
 */
int infsingle_iovar_set(struct infsingle_dev *d, const char *name, const void *val, size_t len)
{
	int n, ret;

	k_mutex_lock(&d->ctrl_lock, K_FOREVER);
	n = infsingle_iovar_build(CDC_DATA(d), CDC_DATA_CAP, name, val, len);
	ret = (n < 0) ? n : do_ioctl(d, INFSINGLE_WLC_SET_VAR, true, n, NULL, 0);
	k_mutex_unlock(&d->ctrl_lock);
	return ret;
}

int infsingle_iovar_get(struct infsingle_dev *d, const char *name, void *out, size_t len)
{
	int n, ret;

	k_mutex_lock(&d->ctrl_lock, K_FOREVER);
	n = infsingle_iovar_build(CDC_DATA(d), CDC_DATA_CAP, name, NULL, 0);
	if (n >= 0 && (size_t)n < len) {
		/* The firmware replies in the same length as the request buffer. */
		memset(CDC_DATA(d) + n, 0, len - n);
		n = len;
	}
	ret = (n < 0) ? n : do_ioctl(d, INFSINGLE_WLC_GET_VAR, false, n, out, len);
	k_mutex_unlock(&d->ctrl_lock);
	return ret;
}

int infsingle_iovar_set_u32(struct infsingle_dev *d, const char *name, uint32_t v)
{
	uint32_t le = sys_cpu_to_le32(v);

	return infsingle_iovar_set(d, name, &le, sizeof(le));
}

int infsingle_iovar_xfer(struct infsingle_dev *d, const char *name, bool set,
			const void *input, size_t input_len, void *output, size_t output_len,
			size_t *received)
{
	size_t name_len = strlen(name) + 1U;
	int n, ret;

	if (received != NULL) {
		*received = 0U;
	}
	if ((input_len > 0U && input == NULL) || (output_len > 0U && output == NULL)) {
		return -EINVAL;
	}
	if (name_len > CDC_DATA_CAP || input_len > CDC_DATA_CAP - name_len ||
	    output_len > CDC_DATA_CAP) {
		return -EMSGSIZE;
	}
	k_mutex_lock(&d->ctrl_lock, K_FOREVER);
	n = infsingle_iovar_build(CDC_DATA(d), CDC_DATA_CAP, name, input, input_len);
	if (n < 0) {
		ret = n;
	} else {
		if (!set && (size_t)n < output_len) {
			memset(CDC_DATA(d) + n, 0, output_len - n);
			n = output_len;
		}
		ret = do_ioctl(d, set ? INFSINGLE_WLC_SET_VAR : INFSINGLE_WLC_GET_VAR,
			       set, n, output, output_len);
		if (ret == -EIO) {
			LOG_DBG("offload iovar %s: firmware status %d", name, d->ctrl_status);
		}
		if (ret == -EIO && d->ctrl_status == -23) {
			/* BCME_UNSUPPORTED; do not confuse it with a transport failure. */
			ret = -ENOTSUP;
		} else if (ret == -EIO && d->ctrl_status == -2) {
			ret = -EINVAL;
		} else if (ret == -EIO && d->ctrl_status == -24) {
			ret = -EMSGSIZE;
		} else if (ret == -EIO && d->ctrl_status == -14) {
			ret = -ENOBUFS;
		} else if (ret == -EIO && d->ctrl_status == -26) {
			ret = -ENOTCONN;
		} else if (ret == 0 && !set) {
			if (received != NULL) {
				*received = d->ctrl_rsp_len;
			} else if (d->ctrl_rsp_len < output_len) {
				ret = -EBADMSG;
			}
		}
	}
	k_mutex_unlock(&d->ctrl_lock);
	return ret;
}

void infsingle_ctrl_tx_done(struct infsingle_dev *d)
{
	d->ctrl_tx_len = 0;
}

void infsingle_ctrl_rx(struct infsingle_dev *d, const uint8_t *p, size_t len)
{
	/* Reply path: runs on the driver thread, completes do_ioctl()'s k_sem_take(). */
	const struct infsingle_cdc_hdr *h = (const struct infsingle_cdc_hdr *)p;
	uint32_t flags;
	size_t dlen;

	if (len < sizeof(*h)) {
		return;
	}
	flags = sys_le32_to_cpu(h->flags);
	if ((flags >> INFSINGLE_CDC_ID_SHIFT) != d->ctrl_pending_id || d->ctrl_pending_id == 0) {
		return; /* stale or unsolicited */
	}
	dlen = MIN((size_t)sys_le32_to_cpu(h->len), len - sizeof(*h));
	if (d->ctrl_rsp != NULL) {
		d->ctrl_rsp_len = MIN(dlen, d->ctrl_rsp_cap);
		memcpy(d->ctrl_rsp, p + sizeof(*h), d->ctrl_rsp_len);
	}
	d->ctrl_status = (flags & INFSINGLE_CDC_FLAG_ERROR) ? (int32_t)sys_le32_to_cpu(h->status) : 0;
	k_sem_give(&d->ctrl_done);
}

int infsingle_event_subscribe(struct infsingle_dev *d, infsingle_event_cb_t cb, void *user)
{
	for (int i = 0; i < INFSINGLE_MAX_EVT_CB; i++) {
		if (d->evt_cb[i].cb == NULL) {
			d->evt_cb[i].user = user;
			d->evt_cb[i].cb = cb;
			return 0;
		}
	}
	return -ENOMEM;
}

/*
 * Event dispatch. Runs on the driver thread, so subscribers must not issue
 * ioctls (see infsingle_mgmt.c). Every subscriber sees every event and
 * filters on ev->type itself; the list is small and fixed.
 */
void infsingle_ctrl_event_rx(struct infsingle_dev *d, const uint8_t *p, size_t len)
{
	struct infsingle_event ev;

	if (infsingle_event_parse(p, len, &ev) != 0) {
		return;
	}
	infsingle_offload_event(d, &ev);
	for (int i = 0; i < INFSINGLE_MAX_EVT_CB; i++) {
		if (d->evt_cb[i].cb != NULL) {
			d->evt_cb[i].cb(d, &ev, d->evt_cb[i].user);
		}
	}
}

/* CLM (regulatory) blob download: hw.md 9.3. */
static int load_clm(struct infsingle_dev *d)
{
	size_t len, off = 0;
	const uint8_t *clm = infsingle_fw_get(d->chip->id, INFSINGLE_BLOB_CLM, &len);
	/* Header plus chunk is built in a stack buffer; control path only. */
	uint8_t buf[sizeof(struct infsingle_dload_hdr) + INFSINGLE_DLOAD_CHUNK];
	uint32_t status = 0;
	int ret;

	if (clm == NULL) {
		LOG_WRN("no CLM blob, using firmware defaults");
		return 0;
	}
	while (off < len) {
		struct infsingle_dload_hdr *h = (struct infsingle_dload_hdr *)buf;
		size_t n = MIN(len - off, (size_t)INFSINGLE_DLOAD_CHUNK);
		uint16_t flag = INFSINGLE_DLOAD_FLAG_VER;

		if (off == 0) {
			flag |= INFSINGLE_DLOAD_FLAG_BEGIN;
		}
		if (off + n == len) {
			flag |= INFSINGLE_DLOAD_FLAG_END;
		}
		h->flag = sys_cpu_to_le16(flag);
		h->dload_type = sys_cpu_to_le16(INFSINGLE_DLOAD_TYPE_CLM);
		h->len = sys_cpu_to_le32(n);
		h->crc = 0;
		memcpy(buf + sizeof(*h), clm + off, n);
		ret = infsingle_iovar_set(d, "clmload", buf, sizeof(*h) + n);
		if (ret) {
			return ret;
		}
		off += n;
	}
	ret = infsingle_iovar_get(d, "clmload_status", &status, sizeof(status));
	if (ret == 0 && status != 0) {
		LOG_ERR("clmload_status %u", status);
		ret = -EIO;
	}
	return ret;
}

int infsingle_ctrl_postboot(struct infsingle_dev *d, const char *country)
{
	/* wl_country_t: abbrev[4], rev (int32, -1 = default), ccode[4] */
	struct {
		char abbrev[4];
		int32_t rev;
		char ccode[4];
	} __packed cc = {.rev = -1};
	uint8_t mask[INFSINGLE_EVT_MAX / 8] = {0};
	char cap[769] = {0};
	static const uint16_t evts[] = {
		INFSINGLE_EVT_SET_SSID, INFSINGLE_EVT_AUTH, INFSINGLE_EVT_DEAUTH, INFSINGLE_EVT_DEAUTH_IND,
		INFSINGLE_EVT_ASSOC, INFSINGLE_EVT_ASSOC_IND, INFSINGLE_EVT_REASSOC, INFSINGLE_EVT_REASSOC_IND,
		INFSINGLE_EVT_DISASSOC, INFSINGLE_EVT_DISASSOC_IND, INFSINGLE_EVT_LINK, INFSINGLE_EVT_PSK_SUP,
		INFSINGLE_EVT_ESCAN_RESULT, INFSINGLE_EVT_EXT_AUTH_REQ, INFSINGLE_EVT_EXT_AUTH_FRAME_RX,
	};
	int ret;

	/*
	 * Post-boot configuration order matters: CLM first (it sets the
	 * regulatory limits the country code selects from), then capabilities,
	 * country, the event mask, and finally WLC_UP to bring the radio up.
	 */
	ret = load_clm(d);
	if (ret) {
		return ret;
	}
	/* Capability string tells us whether the firmware supplicant has SAE. */
	if (infsingle_iovar_get(d, "cap", cap, sizeof(cap) - 1) == 0) {
		d->fw_sae = strstr(cap, "sae") != NULL;
		infsingle_offload_set_caps(d, cap);
		LOG_DBG("firmware capabilities: %s", cap);
	}
	(void)infsingle_iovar_set_u32(d, "bus:txglom", 0);   /* no glomming: no chaining */
	ret = infsingle_iovar_set_u32(d, "apsta", 1);
	if (ret) {
		return ret;
	}
	strncpy(cc.abbrev, country, 3);
	strncpy(cc.ccode, country, 3);
	ret = infsingle_iovar_set(d, "country", &cc, sizeof(cc));
	if (ret) {
		LOG_WRN("country %s rejected (%d)", country, ret);
	}
	for (size_t i = 0; i < ARRAY_SIZE(evts); i++) {
		infsingle_event_mask_set(mask, evts[i]);
	}
	if (d->fw_icmp) {
		infsingle_event_mask_set(mask, INFSINGLE_EVT_ICMP_ECHO_REQ);
	}
	ret = infsingle_iovar_set(d, "event_msgs", mask,
				 d->fw_icmp ? sizeof(mask) : INFSINGLE_EVT_LEGACY_MAX / 8U);
	if (ret) {
		return ret;
	}
	ret = infsingle_iovar_get(d, "cur_etheraddr", d->mac, sizeof(d->mac));
	if (ret) {
		return ret;
	}
	ret = infsingle_ioctl(d, INFSINGLE_WLC_UP, true, NULL, 0);
	if (ret) {
		return ret;
	}
	/* Power save makes the AP hold frames until the next beacon; keep the radio awake. */
	{
		uint32_t pm = 0;

		(void)infsingle_ioctl(d, INFSINGLE_WLC_SET_PM, true, &pm, sizeof(pm));
	}
	return 0;
}
