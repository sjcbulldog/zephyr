/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file infsingle_mgmt.c
 * @brief Zephyr Wi-Fi management ops: scan, connect, disconnect, status.
 *
 * Everything here is expressed in terms of ioctls/iovars (infsingle_ctrl.c) and
 * firmware events. Division of labour:
 *
 *  - Zephyr management thread: calls scan/connect/disconnect/iface_status.
 *    These may block on ioctls (the control path serialises them).
 *  - Driver thread: delivers firmware events to infsingle_mgmt_event() below. That
 *    callback MUST NOT issue ioctls (it runs on the thread that would have
 *    to service the reply: instant deadlock). It only updates state and
 *    gives semaphores / calls Zephyr callbacks.
 *
 * Scan: "escan" iovar. The firmware answers with a stream of
 * INFSINGLE_EVT_ESCAN_RESULT events, one BSS each (status PARTIAL), then a final
 * event with status SUCCESS. Each BSS is converted to a wifi_scan_result and
 * handed to the user callback; a NULL result marks the end of the scan.
 *
 * Connect: the join sequence mirrors the WHD reference sequence
 * (hw.md section 12): configure wsec / supplicant / auth / MFP, then
 * WLC_SET_SSID, then wait for LINK (and PSK_SUP KEYED when a 4-way
 * handshake is expected).
 *
 * NOT IMPLEMENTED (tracked in TODO.md):
 *  - SAE through the host supplicant (EXT_AUTH_REQ). Only the firmware SAE
 *    ("sae_password" iovar) path exists, and only when "cap" lists "sae".
 *
 * Soft AP runs on the primary interface (see the AP section below). It is
 * untested on hardware.
 */

#include <errno.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/dhcpv4.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>

#include "infsingle.h"
#include "infsingle_wire.h"

LOG_MODULE_REGISTER(infsingle_mgmt, CONFIG_WIFI_AIROC_LOG_LEVEL);

/* Wi-Fi channel numbers up to this value are in the 2.4 GHz band. */
#define INFSINGLE_MAX_2G_CHANNEL 14
/* wsec supplicant status "KEYED": 4-way handshake finished. */
#define INFSINGLE_SUP_KEYED 6
/* "any EAP version" argument of the sup_wpa2_eapver iovar. */
#define INFSINGLE_JOIN_SUP_EAPVER_ANY (-1)

/* ---- IE helpers ----------------------------------------------------------- */

/* Information element ids and OUIs used for security detection. */
#define IE_ID_RSN    48
#define IE_ID_VENDOR 221
#define AKM_PSK      2
#define AKM_PSK_SHA256 6
#define AKM_SAE      8
#define AKM_OWE      18

/* Little-endian reads from a byte pointer (BSS info is not aligned). */
static inline uint16_t rd16(const uint8_t *p)
{
	return sys_get_le16(p);
}

static inline uint32_t rd32(const uint8_t *p)
{
	return sys_get_le32(p);
}

/**
 * Walk the RSN element and report which AKM suites it offers.
 * Layout: version(2) group(4) pcount(2) pairwise(4*n) acount(2) akm(4*m).
 * Returns a bitmask: BIT(0) PSK, BIT(1) PSK-SHA256, BIT(2) SAE, BIT(3) OWE.
 */
static unsigned int rsn_akms(const uint8_t *ie, size_t len)
{
	unsigned int mask = 0;
	size_t off = 2 + 4;
	uint16_t n;

	if (len < off + 2) {
		return 0;
	}
	n = rd16(ie + off);
	off += 2 + (size_t)n * 4;
	if (len < off + 2) {
		return 0;
	}
	n = rd16(ie + off);
	off += 2;
	for (uint16_t i = 0; i < n && off + 4 <= len; i++, off += 4) {
		/* Suite = OUI 00-0F-AC + type */
		if (ie[off] != 0x00 || ie[off + 1] != 0x0F || ie[off + 2] != 0xAC) {
			continue;
		}
		switch (ie[off + 3]) {
		case AKM_PSK:
			mask |= BIT(0);
			break;
		case AKM_PSK_SHA256:
			mask |= BIT(1);
			break;
		case AKM_SAE:
			mask |= BIT(2);
			break;
		case AKM_OWE:
			mask |= BIT(3);
			break;
		default:
			break;
		}
	}
	return mask;
}

/**
 * Derive the Zephyr security type of a scanned BSS from its IEs.
 *
 * Policy: a WPA2/WPA3 transition network is reported as PSK, because that is
 * the mode every client can join; the user may still request SAE explicitly.
 * A network we cannot classify is WIFI_SECURITY_TYPE_UNKNOWN, never NONE.
 */
static enum wifi_security_type classify_security(uint16_t capability,
						 const uint8_t *ies, size_t ies_len)
{
	unsigned int akm = 0;
	bool wpa1 = false;

	while (ies_len >= 2) {
		uint8_t id = ies[0];
		size_t l = ies[1];

		if (l + 2 > ies_len) {
			break;
		}
		if (id == IE_ID_RSN) {
			akm |= rsn_akms(ies + 2, l);
		} else if (id == IE_ID_VENDOR && l >= 4 && ies[2] == 0x00 &&
			   ies[3] == 0x50 && ies[4] == 0xF2 && ies[5] == 0x01) {
			wpa1 = true; /* Microsoft WPA vendor element */
		}
		ies += 2 + l;
		ies_len -= 2 + l;
	}

	if (akm & BIT(0)) {
		return WIFI_SECURITY_TYPE_PSK;
	}
	if (akm & BIT(1)) {
		return WIFI_SECURITY_TYPE_PSK_SHA256;
	}
	if (akm & BIT(2)) {
		return WIFI_SECURITY_TYPE_SAE;
	}
	if (akm & BIT(3)) {
		return WIFI_SECURITY_TYPE_OWE;
	}
	if (wpa1) {
		return WIFI_SECURITY_TYPE_WPA_PSK;
	}
	if (capability & 0x0010) { /* privacy bit without RSN/WPA = WEP */
		return WIFI_SECURITY_TYPE_WEP;
	}
	return WIFI_SECURITY_TYPE_NONE;
}

/**
 * Convert one wl_bss_info_t into a wifi_scan_result.
 * @return 0 and fill @p out, or -EBADMSG if the record is malformed.
 */
static int parse_bss(const uint8_t *b, size_t avail, struct wifi_scan_result *out)
{
	uint32_t blen, ie_off, ie_len;
	uint16_t chanspec;
	uint8_t ssid_len;

	if (avail < INFSINGLE_BSS_MIN_LEN) {
		return -EBADMSG;
	}
	blen = rd32(b + INFSINGLE_BSS_OFF_LENGTH);
	if (blen < INFSINGLE_BSS_MIN_LEN || blen > avail) {
		return -EBADMSG;
	}
	ie_off = rd16(b + INFSINGLE_BSS_OFF_IE_OFFSET);
	ie_len = rd32(b + INFSINGLE_BSS_OFF_IE_LENGTH);
	if (ie_off > blen || ie_len > blen - ie_off) {
		return -EBADMSG;
	}
	ssid_len = b[INFSINGLE_BSS_OFF_SSID_LEN];
	if (ssid_len > WIFI_SSID_MAX_LEN) {
		return -EBADMSG;
	}

	memset(out, 0, sizeof(*out));
	out->ssid_length = ssid_len;
	memcpy(out->ssid, b + INFSINGLE_BSS_OFF_SSID, ssid_len);
	memcpy(out->mac, b + INFSINGLE_BSS_OFF_BSSID, 6);
	out->mac_length = 6;
	chanspec = rd16(b + INFSINGLE_BSS_OFF_CHANSPEC);
	out->channel = chanspec & 0xFF;
	out->band = out->channel <= INFSINGLE_MAX_2G_CHANNEL ? WIFI_FREQ_BAND_2_4_GHZ
						       : WIFI_FREQ_BAND_5_GHZ;
	out->rssi = (int8_t)(int16_t)rd16(b + INFSINGLE_BSS_OFF_RSSI);
	out->security = classify_security(rd16(b + INFSINGLE_BSS_OFF_CAPABILITY),
					  b + ie_off, ie_len);
	return 0;
}

/* ---- Event handling (driver thread) --------------------------------------- */

/*
 * Complete the pending join exactly once. join_active guards against a second
 * event (e.g. LINK down after a timeout) completing a join that has already
 * been reported.
 */
static void join_finish(struct infsingle_dev *d, int result)
{
	if (d->join_active) {
		d->join_active = false;
		d->join_result = result;
		k_sem_give(&d->join_done);
	}
}

/*
 * Handle one ESCAN_RESULT event. Results of a previous (aborted) scan carry an
 * older sync_id and are dropped. The scan_cb pointer is cleared before the
 * final callback so the callback may start a new scan.
 */
static void scan_event(struct infsingle_dev *d, const struct infsingle_event *ev)
{
	scan_result_cb_t cb = d->scan_cb;
	struct wifi_scan_result res;

	if (cb == NULL) {
		return;
	}
	if (ev->status == INFSINGLE_EVT_STATUS_PARTIAL &&
	    ev->data_len >= sizeof(struct infsingle_escan_result)) {
		const struct infsingle_escan_result *er = (const void *)ev->data;
		const uint8_t *bss = ev->data + sizeof(*er);

		/* The escan result embeds one BSS; only accept our own scan. */
		if (sys_le16_to_cpu(er->sync_id) == d->scan_sync &&
		    sys_le16_to_cpu(er->bss_count) >= 1 &&
		    parse_bss(bss, ev->data_len - sizeof(*er), &res) == 0) {
			cb(d->iface, 0, &res);
		}
		return;
	}
	/* Any non-PARTIAL status ends the scan (SUCCESS, ABORT, ...). */
	d->scan_cb = NULL;
	cb(d->iface, 0, NULL);
	k_sem_give(&d->scan_done);
}

/*
 * An unexpected link loss while connected: tell the stack the link is down and
 * report the disconnect. No-op if the disconnect was user initiated.
 */
static void link_lost(struct infsingle_dev *d)
{
	if (!d->connected) {
		return;
	}
	d->connected = false;
	infsingle_netif_set_link(d, false);
	wifi_mgmt_raise_disconnect_result_event(d->iface, 0);
}

/*
 * Soft AP events. Returns true if the event was consumed. LINK reports the BSS
 * going up or down; ASSOC_IND/REASSOC_IND and DISASSOC_IND/DEAUTH_IND report a
 * station arriving or leaving (ev->addr is the station address).
 */
static bool ap_event(struct infsingle_dev *d, const struct infsingle_event *ev)
{
	struct wifi_ap_sta_info sta = {.mac_length = WIFI_MAC_ADDR_LEN};

	if (!d->ap_mode && !d->ap_wait) {
		return false;
	}
	switch (ev->type) {
	case INFSINGLE_EVT_LINK:
		if (d->ap_wait) {
			k_sem_give(&d->ap_done);
		}
		return true;
	case INFSINGLE_EVT_ASSOC_IND:
	case INFSINGLE_EVT_REASSOC_IND:
		memcpy(sta.mac, ev->addr, sizeof(sta.mac));
		wifi_mgmt_raise_ap_sta_connected_event(d->iface, &sta);
		return true;
	case INFSINGLE_EVT_DISASSOC_IND:
	case INFSINGLE_EVT_DEAUTH_IND:
		memcpy(sta.mac, ev->addr, sizeof(sta.mac));
		wifi_mgmt_raise_ap_sta_disconnected_event(d->iface, &sta);
		return true;
	default:
		return false;
	}
}

static void infsingle_mgmt_event(struct infsingle_dev *d, const struct infsingle_event *ev, void *user)
{
	ARG_UNUSED(user);

	if (ap_event(d, ev)) {
		return;
	}

	switch (ev->type) {
	case INFSINGLE_EVT_ESCAN_RESULT:
		scan_event(d, ev);
		break;

	case INFSINGLE_EVT_SET_SSID:
		/* Non-zero status: join failed before the link came up. */
		if (ev->status != INFSINGLE_EVT_STATUS_SUCCESS) {
			join_finish(d, ev->status == INFSINGLE_EVT_STATUS_NO_NETWORKS ? -ENETUNREACH
										: -ECONNREFUSED);
		}
		break;

	case INFSINGLE_EVT_LINK:
		/*
		 * Open networks are done when the link comes up; secured ones
		 * must also see PSK_SUP KEYED, whichever arrives last.
		 */
		if (ev->flags & INFSINGLE_EVT_FLAG_LINK_UP) {
			d->join_link = true;
			if (d->join_active && (!d->join_secured || d->join_keyed)) {
				join_finish(d, 0);
			}
		} else {
			d->join_link = false;
			d->join_keyed = false;
			if (d->join_active) {
				join_finish(d, -ECONNABORTED);
			} else {
				link_lost(d);
			}
		}
		break;

	case INFSINGLE_EVT_PSK_SUP:
		if (ev->status == INFSINGLE_SUP_KEYED) {
			d->join_keyed = true;
			if (d->join_active && d->join_link) {
				join_finish(d, 0);
			}
		}
		break;

	case INFSINGLE_EVT_DEAUTH:
	case INFSINGLE_EVT_DEAUTH_IND:
	case INFSINGLE_EVT_DISASSOC:
	case INFSINGLE_EVT_DISASSOC_IND:
		/* Peer or firmware dropped us: fail a join in progress, else report link loss. */
		if (d->join_active) {
			join_finish(d, -ECONNABORTED);
		} else {
			link_lost(d);
		}
		break;

	default:
		break;
	}
}

/* ---- scan ----------------------------------------------------------------- */

static int infsingle_mgmt_scan(const struct device *dev, struct net_if *iface,
			 struct wifi_scan_params *params, scan_result_cb_t cb)
{
	struct infsingle_dev *d = dev->data;
	struct infsingle_escan_params es = {0};
	int ret;

	ARG_UNUSED(iface);

	if (cb == NULL) {
		return -EINVAL;
	}
	if (d->scan_cb != NULL) {
		return -EINPROGRESS;
	}

	es.version = sys_cpu_to_le32(INFSINGLE_ESCAN_REQ_VERSION);
	es.action = sys_cpu_to_le16(INFSINGLE_ESCAN_ACTION_START);
	es.sync_id = sys_cpu_to_le16(++d->scan_sync);
	/* Wildcard SSID, broadcast BSSID, any BSS type, default dwell times. */
	es.params.ssid.len = 0;
	memset(es.params.bssid, 0xFF, sizeof(es.params.bssid));
	es.params.bss_type = 2;
	es.params.scan_type = (params != NULL && params->scan_type == WIFI_SCAN_TYPE_PASSIVE) ? 1
											      : 0;
	es.params.nprobes = -1;
	es.params.active_time = -1;
	es.params.passive_time = -1;
	es.params.home_time = -1;
	es.params.channel_num = 0; /* all channels */

	/*
	 * Arm the scan before issuing the iovar: results can start arriving on
	 * the driver thread before the ioctl reply is processed.
	 */
	k_sem_reset(&d->scan_done);
	d->scan_cb = cb;
	ret = infsingle_iovar_set(d, "escan", &es, sizeof(es));
	if (ret != 0) {
		d->scan_cb = NULL;
		LOG_ERR("escan failed (%d)", ret);
		return ret;
	}
	return 0;
}

/* ---- connect / disconnect ------------------------------------------------- */

static int set_u32_ioctl(struct infsingle_dev *d, uint32_t cmd, uint32_t v)
{
	uint32_t le = sys_cpu_to_le32(v);

	return infsingle_ioctl(d, cmd, true, &le, sizeof(le));
}

/** "bsscfg:<name>" iovars take {bsscfg index, value}. */
static int set_bsscfg(struct infsingle_dev *d, const char *name, int32_t value)
{
	uint32_t v[2] = {sys_cpu_to_le32(0), sys_cpu_to_le32((uint32_t)value)};

	return infsingle_iovar_set(d, name, v, sizeof(v));
}

/** Parameters derived from the requested Zephyr security type. */
struct sec_cfg {
	uint32_t wsec, wpa_auth, auth;
	int32_t mfp;
	bool psk, sae, keyed;
};

static int sec_config(const struct infsingle_dev *d, enum wifi_security_type t, struct sec_cfg *c)
{
	memset(c, 0, sizeof(*c));
	c->mfp = -1; /* leave the firmware default */

	switch (t) {
	case WIFI_SECURITY_TYPE_NONE:
		c->wsec = INFSINGLE_WSEC_NONE;
		c->wpa_auth = INFSINGLE_WPA_AUTH_DISABLED;
		c->auth = INFSINGLE_AUTH_OPEN_SYSTEM;
		return 0;
	case WIFI_SECURITY_TYPE_PSK:
	case WIFI_SECURITY_TYPE_PSK_SHA256:
		c->wsec = INFSINGLE_WSEC_AES;
		c->wpa_auth = INFSINGLE_WPA2_AUTH_PSK;
		c->auth = INFSINGLE_AUTH_OPEN_SYSTEM;
		c->mfp = INFSINGLE_MFP_CAPABLE;
		c->psk = c->keyed = true;
		return 0;
	case WIFI_SECURITY_TYPE_WPA_PSK:
		c->wsec = INFSINGLE_WSEC_TKIP | INFSINGLE_WSEC_AES;
		c->wpa_auth = INFSINGLE_WPA_AUTH_PSK;
		c->auth = INFSINGLE_AUTH_OPEN_SYSTEM;
		c->psk = c->keyed = true;
		return 0;
	case WIFI_SECURITY_TYPE_SAE:
	case WIFI_SECURITY_TYPE_SAE_H2E:
	case WIFI_SECURITY_TYPE_SAE_AUTO:
		if (!d->fw_sae) {
			return -ENOTSUP; /* host-supplicant SAE not implemented */
		}
		c->wsec = INFSINGLE_WSEC_AES;
		c->wpa_auth = INFSINGLE_WPA3_AUTH_SAE_PSK;
		c->auth = INFSINGLE_AUTH_SAE;
		c->mfp = INFSINGLE_MFP_REQUIRED; /* mandatory for WPA3 */
		c->sae = c->keyed = true;
		return 0;
	default:
		return -ENOTSUP;
	}
}

static int program_security(struct infsingle_dev *d, const struct sec_cfg *c,
			    struct wifi_connect_req_params *p)
{
	int ret;

	ret = set_u32_ioctl(d, INFSINGLE_WLC_SET_WSEC, c->wsec);
	if (ret) {
		return ret;
	}
	(void)infsingle_iovar_set_u32(d, "roam_off", 0);

	if (c->keyed) {
		/* Enable the on-chip WPA supplicant for bsscfg 0. */
		ret = set_bsscfg(d, "bsscfg:sup_wpa", 1);
		if (ret) {
			return ret;
		}
		ret = set_bsscfg(d, "bsscfg:sup_wpa2_eapver", INFSINGLE_JOIN_SUP_EAPVER_ANY);
		if (ret) {
			return ret;
		}
	}
	if (c->psk) {
		struct infsingle_wsec_pmk pmk = {0};

		pmk.key_len = sys_cpu_to_le16(p->psk_length);
		pmk.flags = sys_cpu_to_le16(INFSINGLE_WSEC_PASSPHRASE);
		memcpy(pmk.key, p->psk, p->psk_length);
		/* WHD waits 1 ms here: the firmware intermittently drops a PMK
		 * that arrives immediately after the supplicant is enabled.
		 */
		k_msleep(1);
		ret = infsingle_ioctl(d, INFSINGLE_WLC_SET_WSEC_PMK, true, &pmk, sizeof(pmk));
		memset(&pmk, 0, sizeof(pmk)); /* do not leave the key on the stack */
		if (ret) {
			return ret;
		}
	}
	if (c->sae) {
		struct {
			uint16_t len;
			uint8_t pw[INFSINGLE_SAE_MAX_PW_LEN];
		} __packed sae = {0};

		sae.len = sys_cpu_to_le16(p->sae_password_length);
		memcpy(sae.pw, p->sae_password, p->sae_password_length);
		k_msleep(1);
		ret = infsingle_iovar_set(d, "sae_password", &sae, sizeof(sae));
		memset(&sae, 0, sizeof(sae));
		if (ret) {
			return ret;
		}
	}
	ret = set_u32_ioctl(d, INFSINGLE_WLC_SET_INFRA, 1);
	if (ret) {
		return ret;
	}
	ret = set_u32_ioctl(d, INFSINGLE_WLC_SET_AUTH, c->auth);
	if (ret) {
		return ret;
	}
	if (c->mfp >= 0) {
		ret = infsingle_iovar_set_u32(d, "mfp", (uint32_t)c->mfp);
		if (ret) {
			return ret;
		}
	}
	return set_u32_ioctl(d, INFSINGLE_WLC_SET_WPA_AUTH, c->wpa_auth);
}

static int infsingle_mgmt_connect(const struct device *dev, struct net_if *iface,
			    struct wifi_connect_req_params *p)
{
	struct infsingle_dev *d = dev->data;
	struct {
		struct infsingle_ssid ssid;
		uint8_t bssid[WIFI_MAC_ADDR_LEN];
		uint16_t bssid_count;
		uint32_t chanspec_count;
	} __packed join = {0};
	static const uint8_t any_bssid[WIFI_MAC_ADDR_LEN];
	struct sec_cfg c;
	size_t join_len = sizeof(join.ssid);
	int ret;

	if (d->connected) {
		return -EALREADY;
	}
	if (d->ap_mode) {
		return -EBUSY;
	}
	if (p->ssid_length == 0 || p->ssid_length > WIFI_SSID_MAX_LEN) {
		return -EINVAL;
	}
	ret = sec_config(d, p->security, &c);
	if (ret) {
		LOG_ERR("security type %d not supported (%d)", p->security, ret);
		wifi_mgmt_raise_connect_result_event(iface, ret);
		return ret;
	}
	if ((c.psk && (p->psk_length < INFSINGLE_PSK_MIN_LEN || p->psk_length > INFSINGLE_PSK_MAX_LEN)) ||
	    (c.sae && (p->sae_password_length == 0 ||
		       p->sae_password_length > INFSINGLE_SAE_MAX_PW_LEN)) ||
	    (!c.psk && !c.sae && p->psk_length > 0)) {
		return -EINVAL;
	}

	/*
	 * Join state is reset before programming security so that events from
	 * the firmware, which may arrive as soon as WLC_SET_SSID is issued, see
	 * a consistent state. join_result defaults to a timeout.
	 */
	d->join_active = true;
	d->join_link = d->join_keyed = false;
	d->join_secured = c.keyed;
	d->join_result = -ETIMEDOUT;
	k_sem_reset(&d->join_done);

	ret = program_security(d, &c, p);
	if (ret) {
		goto fail;
	}

	/* The band hint restricts the firmware's scan; any channel hint is ignored. */
	ret = set_u32_ioctl(d, INFSINGLE_WLC_SET_BAND,
			    p->band == WIFI_FREQ_BAND_5_GHZ ? 1 :
			    p->band == WIFI_FREQ_BAND_2_4_GHZ ? 2 : 0);
	if (ret) {
		LOG_WRN("band restriction not applied (%d)", ret);
	}
	join.ssid.len = sys_cpu_to_le32(p->ssid_length);
	memcpy(join.ssid.ssid, p->ssid, p->ssid_length);
	if (memcmp(p->bssid, any_bssid, sizeof(any_bssid)) != 0) {
		/* WLC_SET_SSID accepts wl_join_params: SSID followed by association parameters. */
		memcpy(join.bssid, p->bssid, sizeof(join.bssid));
		join_len = sizeof(join);
	}
	ret = infsingle_ioctl(d, INFSINGLE_WLC_SET_SSID, true, &join, join_len);
	if (ret) {
		goto fail;
	}

	if (k_sem_take(&d->join_done, K_MSEC(CONFIG_WIFI_AIROC_JOIN_TIMEOUT_MS)) != 0) {
		d->join_active = false;
		(void)infsingle_ioctl(d, INFSINGLE_WLC_DISASSOC, true, NULL, 0);
		ret = -ETIMEDOUT;
		goto fail;
	}
	ret = d->join_result;
	if (ret) {
		(void)infsingle_ioctl(d, INFSINGLE_WLC_DISASSOC, true, NULL, 0);
		goto fail;
	}

	memcpy(d->ssid, p->ssid, p->ssid_length);
	d->ssid_len = p->ssid_length;
	d->sec = p->security;
	d->connected = true;
	infsingle_netif_set_link(d, true);
#if defined(CONFIG_WIFI_STA_AUTO_DHCPV4)
	net_dhcpv4_restart(iface);
#endif
	wifi_mgmt_raise_connect_result_event(iface, 0);
	return 0;

fail:
	d->join_active = false;
	LOG_ERR("connect failed (%d)", ret);
	wifi_mgmt_raise_connect_result_event(iface, ret);
	return ret;
}

static int infsingle_mgmt_disconnect(const struct device *dev, struct net_if *iface)
{
	struct infsingle_dev *d = dev->data;
	int ret;

	if (!d->connected) {
		return -ENOTCONN;
	}
	/* Clear first so the DISASSOC event does not raise a second report. */
	d->connected = false;
	infsingle_netif_set_link(d, false);
	ret = infsingle_ioctl(d, INFSINGLE_WLC_DISASSOC, true, NULL, 0);
	wifi_mgmt_raise_disconnect_result_event(iface, ret);
	return ret;
}

/* ---- AP ------------------------------------------------------------------- */

/*
 * The soft AP runs on the primary interface (bsscfg 0) with APSTA turned off,
 * as the reference driver does for a primary-interface AP. Station mode is
 * restored (APSTA on) when the AP stops. The BSS-up and BSS-down requests are
 * confirmed by a LINK event; if it does not arrive the "bss" iovar is queried.
 */

static int bss_state(struct infsingle_dev *d, uint32_t *state)
{
	uint32_t out = 0;
	int ret;

	ret = infsingle_iovar_get(d, "bss", &out, sizeof(out));
	if (ret == 0) {
		*state = sys_le32_to_cpu(out);
	}
	return ret;
}

/** Set the BSS up or down and wait until the firmware confirms it. */
static int bss_set(struct infsingle_dev *d, uint32_t want)
{
	uint32_t v[2] = {sys_cpu_to_le32(0), sys_cpu_to_le32(want)};
	uint32_t state;
	int ret;

	k_sem_reset(&d->ap_done);
	d->ap_wait = true;
	ret = infsingle_iovar_set(d, "bss", v, sizeof(v));
	if (ret == 0 && k_sem_take(&d->ap_done, K_MSEC(CONFIG_WIFI_AIROC_JOIN_TIMEOUT_MS)) != 0) {
		/* No event: trust the firmware's own report. */
		ret = bss_state(d, &state);
		if (ret == 0 && state != want) {
			ret = -ETIMEDOUT;
		}
	}
	d->ap_wait = false;
	return ret;
}

static int ap_program_security(struct infsingle_dev *d, const struct sec_cfg *c,
			       const struct wifi_connect_req_params *p)
{
	int ret;

	ret = set_bsscfg(d, "bsscfg:wsec", c->wsec);
	if (ret) {
		return ret;
	}
	ret = infsingle_iovar_set_u32(d, "mfp", c->mfp > 0 ? (uint32_t)c->mfp : 0);
	if (ret) {
		return ret;
	}
	ret = set_bsscfg(d, "bsscfg:wpa_auth", c->wpa_auth);
	if (ret) {
		return ret;
	}
	if (c->psk) {
		struct infsingle_wsec_pmk pmk = {0};

		pmk.key_len = sys_cpu_to_le16(p->psk_length);
		pmk.flags = sys_cpu_to_le16(INFSINGLE_WSEC_PASSPHRASE);
		memcpy(pmk.key, p->psk, p->psk_length);
		/* Same settling delay as in connect: the PMK is dropped otherwise. */
		k_msleep(1);
		ret = infsingle_ioctl(d, INFSINGLE_WLC_SET_WSEC_PMK, true, &pmk, sizeof(pmk));
		memset(&pmk, 0, sizeof(pmk));
		return ret;
	}
	if (c->sae) {
		struct {
			uint16_t len;
			uint8_t pw[INFSINGLE_SAE_MAX_PW_LEN];
		} __packed sae = {0};
		const uint8_t *pw = p->sae_password != NULL ? p->sae_password : p->psk;
		size_t len = p->sae_password != NULL ? p->sae_password_length : p->psk_length;

		sae.len = sys_cpu_to_le16(len);
		memcpy(sae.pw, pw, len);
		k_msleep(1);
		ret = infsingle_iovar_set(d, "sae_password", &sae, sizeof(sae));
		memset(&sae, 0, sizeof(sae));
		return ret;
	}
	return 0;
}

static int ap_configure(struct infsingle_dev *d, const struct sec_cfg *c,
			const struct wifi_connect_req_params *p, uint16_t chanspec)
{
	struct {
		uint32_t idx;
		uint32_t len;
		uint8_t ssid[WIFI_SSID_MAX_LEN];
	} __packed ssid = {.idx = sys_cpu_to_le32(0), .len = sys_cpu_to_le32(p->ssid_length)};
	int ret;

	/* Leave APSTA: the AP owns the primary interface. The radio is down while it changes. */
	ret = infsingle_ioctl(d, INFSINGLE_WLC_DOWN, true, NULL, 0);
	if (ret) {
		return ret;
	}
	(void)infsingle_iovar_set_u32(d, "apsta", 0); /* not all firmware has the iovar */
	ret = infsingle_ioctl(d, INFSINGLE_WLC_UP, true, NULL, 0);
	if (ret) {
		return ret;
	}
	ret = set_u32_ioctl(d, INFSINGLE_WLC_SET_AP, 1);
	if (ret) {
		return ret;
	}
	memcpy(ssid.ssid, p->ssid, p->ssid_length);
	ret = infsingle_iovar_set(d, "bsscfg:ssid", &ssid, sizeof(ssid));
	if (ret) {
		return ret;
	}
	ret = infsingle_iovar_set_u32(d, "chanspec", chanspec);
	if (ret) {
		return ret;
	}
	ret = ap_program_security(d, c, p);
	if (ret) {
		return ret;
	}
	if ((chanspec & 0xFF) <= INFSINGLE_MAX_2G_CHANNEL) {
		(void)infsingle_iovar_set_u32(d, "2g_mrate", INFSINGLE_MCAST_RATE_11MBPS);
	}
	return bss_set(d, INFSINGLE_BSS_UP);
}

/** Return to station operation (APSTA on) after the AP stops or fails to start. */
static void ap_restore_sta(struct infsingle_dev *d)
{
	(void)set_u32_ioctl(d, INFSINGLE_WLC_SET_AP, 0);
	(void)infsingle_ioctl(d, INFSINGLE_WLC_DOWN, true, NULL, 0);
	(void)infsingle_iovar_set_u32(d, "apsta", 1);
	(void)infsingle_ioctl(d, INFSINGLE_WLC_UP, true, NULL, 0);
}

static int infsingle_mgmt_ap_enable(const struct device *dev, struct net_if *iface,
			      struct wifi_connect_req_params *p)
{
	struct infsingle_dev *d = dev->data;
	struct sec_cfg c;
	uint16_t chanspec;
	uint8_t channel;
	int ret;

	ARG_UNUSED(iface);

	if (d->ap_mode) {
		return -EALREADY;
	}
	if (d->connected || d->join_active) {
		return -EBUSY;
	}
	if (p->ssid_length == 0 || p->ssid_length > WIFI_SSID_MAX_LEN) {
		return -EINVAL;
	}
	switch (p->security) {
	case WIFI_SECURITY_TYPE_NONE:
	case WIFI_SECURITY_TYPE_PSK:
	case WIFI_SECURITY_TYPE_SAE:
		break;
	default:
		return -ENOTSUP;
	}
	ret = sec_config(d, p->security, &c);
	if (ret) {
		return ret;
	}
	if (c.psk && (p->psk_length < INFSINGLE_PSK_MIN_LEN || p->psk_length > INFSINGLE_PSK_MAX_LEN)) {
		return -EINVAL;
	}
	if (c.sae) {
		size_t len = p->sae_password != NULL ? p->sae_password_length : p->psk_length;

		if (len == 0 || len > INFSINGLE_SAE_MAX_PW_LEN ||
		    (p->sae_password == NULL && p->psk == NULL)) {
			return -EINVAL;
		}
	}
	if (!c.psk && !c.sae && p->psk_length > 0) {
		return -EINVAL;
	}

	/* 2.4 GHz channels 1-14 and 5 GHz channels 36-165; anything else falls back to 1. */
	channel = p->channel;
	if (channel == 0 || channel == WIFI_CHANNEL_ANY ||
	    (channel > INFSINGLE_MAX_2G_CHANNEL && (channel < 36 || channel > 165))) {
		LOG_WRN("channel %u not usable for the AP, using 1", p->channel);
		channel = 1;
	}
	chanspec = channel | INFSINGLE_CHSPEC_BW_20 |
		   (channel <= INFSINGLE_MAX_2G_CHANNEL ? INFSINGLE_CHSPEC_BAND_2G
							: INFSINGLE_CHSPEC_BAND_5G);

	ret = ap_configure(d, &c, p, chanspec);
	if (ret) {
		LOG_ERR("AP start failed (%d)", ret);
		ap_restore_sta(d);
		return ret;
	}
	memcpy(d->ssid, p->ssid, p->ssid_length);
	d->ssid_len = p->ssid_length;
	d->sec = p->security;
	d->ap_mode = true;
	infsingle_netif_set_link(d, true);
	return 0;
}

static int infsingle_mgmt_ap_disable(const struct device *dev, struct net_if *iface)
{
	struct infsingle_dev *d = dev->data;
	int ret;

	ARG_UNUSED(iface);

	if (!d->ap_mode) {
		return -EALREADY;
	}
	ret = bss_set(d, INFSINGLE_BSS_DOWN);
	if (ret) {
		LOG_WRN("BSS down failed (%d)", ret);
	}
	d->ap_mode = false;
	infsingle_netif_set_link(d, false);
	ap_restore_sta(d);
	return ret;
}

/* ---- status --------------------------------------------------------------- */

static int infsingle_mgmt_iface_status(const struct device *dev, struct net_if *iface,
				 struct wifi_iface_status *st)
{
	struct infsingle_dev *d = dev->data;
	int32_t rssi_le = 0;
	uint32_t chanspec_le = 0;

	ARG_UNUSED(iface);
	memset(st, 0, sizeof(*st));
	st->iface_mode = WIFI_MODE_INFRA;

	if (!d->connected) {
		st->state = d->join_active ? WIFI_STATE_ASSOCIATING : WIFI_STATE_DISCONNECTED;
		return 0;
	}
	st->state = WIFI_STATE_COMPLETED;
	st->ssid_len = d->ssid_len;
	memcpy(st->ssid, d->ssid, d->ssid_len);
	st->security = d->sec;

	/* Best effort: a failed query leaves the field at zero. */
	(void)infsingle_ioctl(d, INFSINGLE_WLC_GET_BSSID, false, st->bssid, WIFI_MAC_ADDR_LEN);
	if (infsingle_ioctl(d, INFSINGLE_WLC_GET_RSSI, false, &rssi_le, sizeof(rssi_le)) == 0) {
		st->rssi = (int32_t)sys_le32_to_cpu((uint32_t)rssi_le);
	}
	if (infsingle_iovar_get(d, "chanspec", &chanspec_le, sizeof(chanspec_le)) == 0) {
		st->channel = sys_le32_to_cpu(chanspec_le) & 0xFF;
		st->band = st->channel <= INFSINGLE_MAX_2G_CHANNEL ? WIFI_FREQ_BAND_2_4_GHZ
							     : WIFI_FREQ_BAND_5_GHZ;
	}
	return 0;
}

#ifdef CONFIG_NET_STATISTICS_WIFI
static int infsingle_mgmt_get_stats(const struct device *dev, struct net_if *iface,
				   struct net_stats_wifi *stats)
{
	struct infsingle_dev *d = dev->data;
	struct infsingle_stats snapshot;

	if (iface != d->iface || stats == NULL) {
		return -EINVAL;
	}

	infsingle_stats_snapshot(&d->stats, &snapshot);
	*stats = snapshot.wifi;
	return 0;
}

static int infsingle_mgmt_reset_stats(const struct device *dev, struct net_if *iface)
{
	struct infsingle_dev *d = dev->data;

	if (iface != d->iface) {
		return -EINVAL;
	}

	infsingle_stats_reset(&d->stats);
	return 0;
}
#endif

const struct wifi_mgmt_ops infsingle_mgmt_ops = {
	.scan = infsingle_mgmt_scan,
	.connect = infsingle_mgmt_connect,
	.disconnect = infsingle_mgmt_disconnect,
	.ap_enable = infsingle_mgmt_ap_enable,
	.ap_disable = infsingle_mgmt_ap_disable,
	.iface_status = infsingle_mgmt_iface_status,
#ifdef CONFIG_NET_STATISTICS_WIFI
	.get_stats = infsingle_mgmt_get_stats,
	.reset_stats = infsingle_mgmt_reset_stats,
#endif
};

int infsingle_mgmt_init(struct infsingle_dev *d)
{
	/* Called from iface init, which can run more than once per boot. */
	static bool subscribed;

	k_sem_init(&d->scan_done, 0, 1);
	k_sem_init(&d->join_done, 0, 1);
	if (!subscribed) {
		subscribed = true;
		return infsingle_event_subscribe(d, infsingle_mgmt_event, NULL);
	}
	return 0;
}
