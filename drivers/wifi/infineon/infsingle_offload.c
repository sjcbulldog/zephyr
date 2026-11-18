/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>
#include <zephyr/sys/byteorder.h>
#include "infsingle.h"

#define OL_CONFIG_SIZE 32U
#define TKO_HEADER_SIZE 4U
#define TKO_MAX_TCP 0U
#define TKO_PARAM 1U
#define TKO_ENABLE 3U
#define TKO_STATUS 4U
#define TKO_AUTO_ENABLE 6U
#define TKO_FILTER 8U
#define ICMP_HEADER_SIZE 8U
#define ICMP_IP_SIZE 24U
#define ICMP_PEER_SIZE 40U
#define ICMP_INFO_SIZE 52U

static int offload_device(const struct device *dev, struct infsingle_dev **out)
{
	if (dev == NULL) {
		return -EINVAL;
	}
	/* Validate the API before interpreting data belonging to another driver. */
	if (dev->api != &infsingle_api) {
		return -ENODEV;
	}
	if (!device_is_ready(dev)) {
		return -EAGAIN;
	}
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}
	*out = dev->data;
	if (k_current_get() == &(*out)->thread) {
		return -EDEADLK;
	}
	return 0;
}

static int offload_xfer(const struct device *dev, const char *name, bool set,
			const void *input, size_t input_len, void *output, size_t output_len,
			size_t *received)
{
	struct infsingle_dev *d;
	int ret = offload_device(dev, &d);

	if (ret != 0) {
		return ret;
	}
	return infsingle_iovar_xfer(d, name, set, input, input_len, output, output_len, received);
}

static int offload_set(const struct device *dev, const char *name, const void *data, size_t len)
{
	return offload_xfer(dev, name, true, data, len, NULL, 0, NULL);
}

static int offload_get(const struct device *dev, const char *name, void *data, size_t len)
{
	return offload_xfer(dev, name, false, NULL, 0, data, len, NULL);
}

static int offload_u32_set(const struct device *dev, const char *name, uint32_t value)
{
	uint32_t le = sys_cpu_to_le32(value);

	return offload_set(dev, name, &le, sizeof(le));
}

static int offload_u32_get(const struct device *dev, const char *name, uint32_t *value)
{
	uint32_t le;
	int ret = offload_get(dev, name, &le, sizeof(le));

	if (ret == 0) {
		*value = sys_le32_to_cpu(le);
	}
	return ret;
}

void infsingle_offload_set_caps(struct infsingle_dev *d, const char *caps)
{
	d->fw_offloads = false;
	d->fw_icmp = false;
	while (*caps != '\0') {
		size_t len;

		caps += strspn(caps, " \t\r\n");
		len = strcspn(caps, " \t\r\n");
		if (len == 8U && memcmp(caps, "offloads", len) == 0) {
			d->fw_offloads = true;
		} else if (len == 4U && memcmp(caps, "icmp", len) == 0) {
			d->fw_icmp = true;
		}
		caps += len;
	}
}

static int tko_xfer(const struct device *dev, uint16_t command, bool set, uint8_t *data,
		    size_t size)
{
	uint8_t request[96] = {0};
	uint8_t response[44] = {0};
	size_t length = TKO_HEADER_SIZE + size;
	size_t received;
	int ret;

	if (length > sizeof(response)) {
		return -EMSGSIZE;
	}
	sys_put_le16(command, request);
	sys_put_le16(length, request + 2);
	if (set) {
		memcpy(request + TKO_HEADER_SIZE, data, size);
	}
	/* Firmware handlers require workspace beyond the encoded subcommand. */
	ret = offload_xfer(dev, "tko", set, request, sizeof(request),
			   set ? NULL : response, set ? 0U : length, &received);
	if (ret == 0 && !set) {
		if (command == TKO_STATUS) {
			if (received < TKO_HEADER_SIZE + 1U || response[4] > size - 1U) {
				return -EBADMSG;
			}
			length = TKO_HEADER_SIZE + 1U + response[4];
		}
		if (received < length || sys_get_le16(response) != command ||
		    sys_get_le16(response + 2) < length - TKO_HEADER_SIZE) {
			return -EBADMSG;
		}
		memcpy(data, response + TKO_HEADER_SIZE, size);
	}
	return ret;
}

int wifi_infineon_offload_is_supported(const struct device *dev,
				      enum wifi_infineon_offload feature, bool *supported)
{
	struct infsingle_dev *d;
	uint32_t value;
	int ret;

	if (supported == NULL) {
		return -EINVAL;
	}
	*supported = false;
	ret = offload_device(dev, &d);
	if (ret != 0) {
		return ret;
	}
	switch (feature) {
	case WIFI_INFINEON_OFFLOAD_PROFILE:
		*supported = d->fw_offloads;
		return 0;
	case WIFI_INFINEON_OFFLOAD_ICMP:
		*supported = d->fw_icmp;
		return 0;
	case WIFI_INFINEON_OFFLOAD_ARP:
		ret = offload_u32_get(dev, "arp_version", &value);
		break;
	case WIFI_INFINEON_OFFLOAD_ND:
		ret = offload_u32_get(dev, "ndoe", &value);
		break;
	case WIFI_INFINEON_OFFLOAD_TCP: {
		uint8_t max_tcp[4];

		ret = tko_xfer(dev, TKO_MAX_TCP, false, max_tcp, sizeof(max_tcp));
		if (ret == 0) {
			*supported = max_tcp[0] > 0U;
			return 0;
		}
		break;
	}
	case WIFI_INFINEON_OFFLOAD_KEEPALIVE: {
		uint8_t header[6];

		ret = offload_get(dev, "keep_alive", header, sizeof(header));
		break;
	}
	case WIFI_INFINEON_OFFLOAD_PACKET_FILTER:
		ret = offload_u32_get(dev, "pkt_filter_mode", &value);
		break;
	case WIFI_INFINEON_OFFLOAD_BDO:
	case WIFI_INFINEON_OFFLOAD_DHCP:
	case WIFI_INFINEON_OFFLOAD_PNO:
	case WIFI_INFINEON_OFFLOAD_GTK:
		return -ENODATA;
	default:
		return -EINVAL;
	}
	if (ret == -ENOTSUP) {
		if (feature == WIFI_INFINEON_OFFLOAD_KEEPALIVE) {
			/* Some firmware implements SET but no read-only keepalive query. */
			return -ENODATA;
		}
		return 0;
	}
	*supported = ret == 0;
	return ret;
}

static int profile_set(const struct device *dev, uint32_t features, uint32_t command,
		       const void *data, size_t size)
{
	struct infsingle_dev *d;
	uint8_t request[OL_CONFIG_SIZE] = {0};
	int ret;

	if ((features & ~WIFI_INFINEON_OFFLOAD_ALL) != 0U) {
		return -EINVAL;
	}
	ret = offload_device(dev, &d);
	if (ret != 0) {
		return ret;
	}
	if (!d->fw_offloads) {
		return -ENOTSUP;
	}
	sys_put_le16(1U, request);
	sys_put_le16(sizeof(request), request + 2);
	sys_put_le32(command, request + 4);
	memcpy(request + 8, data, size);
	sys_put_le32(~features, request + 28);
	return offload_set(dev, "offload_config", request, sizeof(request));
}

int wifi_infineon_offload_profile_set(const struct device *dev, uint32_t features, bool reset)
{
	uint8_t profile[8] = {0};

	sys_put_le32(reset ? UINT32_MAX : 1U, profile);
	profile[4] = reset;
	return profile_set(dev, features, 1U, profile, sizeof(profile));
}

int wifi_infineon_offload_profile_enable(const struct device *dev, uint32_t features, bool enable)
{
	uint8_t activate = enable;

	return profile_set(dev, features, 4U, &activate, sizeof(activate));
}

int wifi_infineon_offload_ipv4_update(const struct device *dev, uint32_t features,
				     const struct in_addr *address, bool add)
{
	uint8_t data[8] = {0};

	if (address == NULL) {
		return -EINVAL;
	}
	memcpy(data, address, sizeof(*address));
	data[4] = !add;
	return profile_set(dev, features, 2U, data, sizeof(data));
}

int wifi_infineon_offload_ipv6_update(const struct device *dev, uint32_t features,
				     const struct in6_addr *address, bool anycast, bool add)
{
	uint8_t data[20] = {0};

	if (address == NULL) {
		return -EINVAL;
	}
	memcpy(data, address, sizeof(*address));
	data[16] = anycast;
	data[17] = !add;
	return profile_set(dev, features, 3U, data, sizeof(data));
}

int wifi_infineon_offload_arp_set(const struct device *dev,
				const struct wifi_infineon_arp_config *config)
{
	int ret;

	if (config == NULL || (config->features & ~GENMASK(3, 0)) != 0U) {
		return -EINVAL;
	}
	ret = offload_u32_set(dev, "arp_ol", config->features);
	if (ret != 0) {
		return ret;
	}
	ret = offload_u32_set(dev, "arp_peerage", config->peer_age);
	if (ret != 0) {
		return ret;
	}
	return offload_u32_set(dev, "arpoe", config->enable);
}

int wifi_infineon_offload_arp_get(const struct device *dev,
				struct wifi_infineon_arp_config *config)
{
	struct wifi_infineon_arp_config result;
	uint32_t enable;
	int ret;

	if (config == NULL) {
		return -EINVAL;
	}
	ret = offload_u32_get(dev, "arp_ol", &result.features);
	if (ret != 0) {
		return ret;
	}
	ret = offload_u32_get(dev, "arp_peerage", &result.peer_age);
	if (ret != 0) {
		return ret;
	}
	ret = offload_u32_get(dev, "arpoe", &enable);
	if (ret == 0) {
		result.enable = enable != 0U;
		*config = result;
	}
	return ret;
}

int wifi_infineon_offload_arp_address_add(const struct device *dev, const struct in_addr *address)
{
	if (address == NULL) {
		return -EINVAL;
	}
	return offload_set(dev, "arp_hostip", address, sizeof(*address));
}

int wifi_infineon_offload_arp_addresses_get(const struct device *dev, struct in_addr *addresses,
					  size_t *count)
{
	struct in_addr table[WIFI_INFINEON_OFFLOAD_MAX_IPV4_ADDRS];
	int ret;

	if (addresses == NULL || count == NULL) {
		return -EINVAL;
	}
	*count = 0U;
	ret = offload_get(dev, "arp_hostip", table, sizeof(table));
	if (ret == 0) {
		for (size_t i = 0U; i < ARRAY_SIZE(table); i++) {
			if (table[i].s_addr != 0U) {
				addresses[(*count)++] = table[i];
			}
		}
	}
	return ret;
}

int wifi_infineon_offload_arp_addresses_clear(const struct device *dev)
{
	return offload_set(dev, "arp_hostip_clear", NULL, 0);
}

int wifi_infineon_offload_arp_cache_clear(const struct device *dev)
{
	return offload_set(dev, "arp_table_clear", NULL, 0);
}

int wifi_infineon_offload_keepalive_set(const struct device *dev, uint32_t period_ms,
				      const uint8_t *packet, size_t length)
{
	uint8_t request[6U + WIFI_INFINEON_OFFLOAD_MAX_PACKET_SIZE] = {0};

	if (length > WIFI_INFINEON_OFFLOAD_MAX_PACKET_SIZE) {
		return -EMSGSIZE;
	}
	if (length > 0U && packet == NULL) {
		return -EINVAL;
	}
	sys_put_le32(period_ms, request);
	sys_put_le16(length, request + 4);
	if (length > 0U) {
		memcpy(request + 6, packet, length);
	}
	return offload_set(dev, "keep_alive", request, 6U + length);
}

int wifi_infineon_offload_tcp_set(const struct device *dev,
				const struct wifi_infineon_tcp_keepalive_config *config)
{
	uint8_t data[8] = {0};

	if (config == NULL) {
		return -EINVAL;
	}
	sys_put_le16(config->interval == 0U ? 20U : config->interval, data);
	sys_put_le16(config->retry_interval == 0U ? 3U : config->retry_interval, data + 2);
	sys_put_le16(config->retry_count == 0U ? 3U : config->retry_count, data + 4);
	return tko_xfer(dev, TKO_PARAM, true, data, sizeof(data));
}

int wifi_infineon_offload_tcp_get(const struct device *dev,
				struct wifi_infineon_tcp_keepalive_config *config)
{
	uint8_t data[8];
	int ret;

	if (config == NULL) {
		return -EINVAL;
	}
	ret = tko_xfer(dev, TKO_PARAM, false, data, sizeof(data));
	if (ret == 0) {
		config->interval = sys_get_le16(data);
		config->retry_interval = sys_get_le16(data + 2);
		config->retry_count = sys_get_le16(data + 4);
	}
	return ret;
}

int wifi_infineon_offload_tcp_enable(const struct device *dev, bool enable)
{
	uint8_t auto_enable[8] = {0};
	uint8_t toggle[4] = {enable};
	int ret;

	sys_put_le16(1U, auto_enable);
	sys_put_le16(4U, auto_enable + 2);
	auto_enable[4] = enable;
	ret = tko_xfer(dev, TKO_AUTO_ENABLE, true, auto_enable, sizeof(auto_enable));
	if (ret != 0) {
		return ret;
	}
	return tko_xfer(dev, TKO_ENABLE, true, toggle, sizeof(toggle));
}

int wifi_infineon_offload_tcp_filter_set(const struct device *dev,
				       const struct wifi_infineon_tcp_keepalive_filter *filter)
{
	uint8_t data[40] = {0};

	if (filter == NULL) {
		return -EINVAL;
	}
	sys_put_le16(1U, data);
	sys_put_le16(sizeof(data) - 4U, data + 2);
	sys_put_le16(filter->source_port, data + 4);
	sys_put_le16(filter->destination_port, data + 6);
	memcpy(data + 8, filter->source_ip, sizeof(filter->source_ip));
	memcpy(data + 24, filter->destination_ip, sizeof(filter->destination_ip));
	return tko_xfer(dev, TKO_FILTER, true, data, sizeof(data));
}

int wifi_infineon_offload_tcp_status_get(const struct device *dev,
				       struct wifi_infineon_tcp_keepalive_status *status)
{
	uint8_t data[1U + WIFI_INFINEON_OFFLOAD_MAX_TCP_CONNECTIONS];
	int ret;

	if (status == NULL) {
		return -EINVAL;
	}
	ret = tko_xfer(dev, TKO_STATUS, false, data, sizeof(data));
	if (ret == 0) {
		if (data[0] > WIFI_INFINEON_OFFLOAD_MAX_TCP_CONNECTIONS) {
			return -EBADMSG;
		}
		status->count = data[0];
		memset(status->status, WIFI_INFINEON_TCP_UNAVAILABLE, sizeof(status->status));
		memcpy(status->status, data + 1, status->count);
	}
	return ret;
}

static int icmp_ip_encode(const struct wifi_infineon_offload_ip *ip, uint8_t *version,
			   uint8_t address[16])
{
	if (ip == NULL) {
		return -EINVAL;
	}
	memset(address, 0, 16);
	if (ip->family == AF_INET) {
		*version = 1U;
		memcpy(address, &ip->address.ipv4, sizeof(ip->address.ipv4));
	} else if (ip->family == AF_INET6) {
		*version = 2U;
		memcpy(address, &ip->address.ipv6, sizeof(ip->address.ipv6));
	} else {
		return -EINVAL;
	}
	return 0;
}

static int icmp_ip_decode(uint8_t version, const uint8_t address[16],
			   struct wifi_infineon_offload_ip *ip)
{
	memset(ip, 0, sizeof(*ip));
	if (version == 1U) {
		ip->family = AF_INET;
		memcpy(&ip->address.ipv4, address, sizeof(ip->address.ipv4));
	} else if (version == 2U) {
		ip->family = AF_INET6;
		memcpy(&ip->address.ipv6, address, sizeof(ip->address.ipv6));
	} else {
		return -EBADMSG;
	}
	return 0;
}

static int icmp_xfer(const struct device *dev, uint8_t command, const uint8_t *data, size_t size,
		     uint8_t *response)
{
	struct infsingle_dev *d;
	uint8_t request[ICMP_INFO_SIZE] = {0};
	size_t length = response == NULL ? ICMP_HEADER_SIZE + size : ICMP_INFO_SIZE;
	int ret = offload_device(dev, &d);

	if (ret != 0) {
		return ret;
	}
	if (!d->fw_icmp) {
		return -ENOTSUP;
	}
	sys_put_le16(1U, request);
	sys_put_le16(length, request + 2);
	request[4] = command;
	memcpy(request + ICMP_HEADER_SIZE, data, size);
	return offload_xfer(dev, "icmp_echo_req", response == NULL, request, length,
			    response, response == NULL ? 0U : ICMP_INFO_SIZE, NULL);
}

int wifi_infineon_offload_icmp_enable(const struct device *dev, bool enable)
{
	uint8_t value = enable;

	return icmp_xfer(dev, 0U, &value, sizeof(value), NULL);
}

int wifi_infineon_offload_icmp_peer_add(const struct device *dev,
				      const struct wifi_infineon_icmp_peer *peer)
{
	uint8_t data[ICMP_PEER_SIZE] = {0};
	int ret;

	if (peer == NULL || peer->period == 0U || peer->duration == 0U) {
		return -EINVAL;
	}
	sys_put_le16(1U, data);
	sys_put_le16(sizeof(data), data + 2);
	ret = icmp_ip_encode(&peer->ip, data + 4, data + 16);
	if (ret != 0) {
		return ret;
	}
	sys_put_le32(peer->period, data + 8);
	sys_put_le32(peer->duration, data + 12);
	memcpy(data + 32, peer->mac, sizeof(peer->mac));
	return icmp_xfer(dev, 1U, data, sizeof(data), NULL);
}

static int icmp_peer_xfer(const struct device *dev, const struct wifi_infineon_offload_ip *ip,
			  uint8_t command, uint8_t *response)
{
	uint8_t data[ICMP_IP_SIZE] = {0};
	int ret;

	sys_put_le16(1U, data);
	sys_put_le16(sizeof(data), data + 2);
	ret = icmp_ip_encode(ip, data + 4, data + 8);
	if (ret != 0) {
		return ret;
	}
	return icmp_xfer(dev, command, data, sizeof(data), response);
}

int wifi_infineon_offload_icmp_peer_action(const struct device *dev,
					 const struct wifi_infineon_offload_ip *ip,
					 enum wifi_infineon_icmp_action action)
{
	if (action != WIFI_INFINEON_ICMP_DELETE && action != WIFI_INFINEON_ICMP_START &&
	    action != WIFI_INFINEON_ICMP_STOP) {
		return -EINVAL;
	}
	return icmp_peer_xfer(dev, ip, action, NULL);
}

int wifi_infineon_offload_icmp_info_get(const struct device *dev,
				      const struct wifi_infineon_offload_ip *ip,
				      struct wifi_infineon_icmp_info *info)
{
	struct wifi_infineon_icmp_info result = {0};
	uint8_t response[ICMP_INFO_SIZE];
	const uint8_t *peer = response + 12;
	int ret;

	if (info == NULL) {
		return -EINVAL;
	}
	ret = icmp_peer_xfer(dev, ip, 5U, response);
	if (ret != 0) {
		return ret;
	}
	if (sys_get_le16(response) != 1U || sys_get_le16(response + 2) < sizeof(response) ||
	    response[5] == 0U || sys_get_le16(peer) != 1U ||
	    sys_get_le16(peer + 2) != ICMP_PEER_SIZE) {
		return -EBADMSG;
	}
	result.enable = response[4] != 0U;
	result.count = response[5];
	result.state = sys_get_le32(response + 8);
	result.peer.period = sys_get_le32(peer + 8);
	result.peer.duration = sys_get_le32(peer + 12);
	memcpy(result.peer.mac, peer + 32, sizeof(result.peer.mac));
	ret = icmp_ip_decode(peer[4], peer + 16, &result.peer.ip);
	if (ret == 0) {
		*info = result;
	}
	return ret;
}

int wifi_infineon_offload_icmp_callback_set(const struct device *dev,
					  wifi_infineon_icmp_callback_t callback, void *user_data)
{
	struct infsingle_dev *d;
	k_spinlock_key_t key;
	int ret = offload_device(dev, &d);

	if (ret != 0) {
		return ret;
	}
	if (!d->fw_icmp) {
		return -ENOTSUP;
	}
	key = k_spin_lock(&d->offload_cb_lock);
	d->icmp_callback = callback;
	d->icmp_callback_user = user_data;
	k_spin_unlock(&d->offload_cb_lock, key);
	return 0;
}

void infsingle_offload_event(struct infsingle_dev *d, const struct infsingle_event *event)
{
	struct wifi_infineon_icmp_event result;
	wifi_infineon_icmp_callback_t callback;
	const uint8_t *data = event->data;
	void *user;
	k_spinlock_key_t key;

	if (event->type != INFSINGLE_EVT_ICMP_ECHO_REQ || data == NULL ||
	    event->data_len < 28U || sys_get_le16(data) != 1U ||
	    sys_get_le16(data + 2) < 28U || sys_get_le16(data + 2) > event->data_len) {
		return;
	}
	if (icmp_ip_decode(data[4], data + 12, &result.ip) != 0) {
		return;
	}
	result.reason = data[5];
	result.requests = sys_get_le32(data + 8);
	key = k_spin_lock(&d->offload_cb_lock);
	callback = d->icmp_callback;
	user = d->icmp_callback_user;
	k_spin_unlock(&d->offload_cb_lock, key);
	if (callback != NULL) {
		callback(d->dev, &result, user);
	}
}

int wifi_infineon_offload_packet_filter_add(const struct device *dev,
					  const struct wifi_infineon_packet_filter *filter)
{
	uint8_t request[20U + 2U * WIFI_INFINEON_OFFLOAD_MAX_FILTER_SIZE] = {0};

	if (filter == NULL || filter->mask == NULL || filter->pattern == NULL ||
	    filter->size == 0U) {
		return -EINVAL;
	}
	if (filter->size > WIFI_INFINEON_OFFLOAD_MAX_FILTER_SIZE ||
	    filter->offset > UINT32_MAX - filter->size) {
		return -EMSGSIZE;
	}
	sys_put_le32(filter->id, request);
	/* Type 0 is the firmware's mask-and-pattern filter. */
	sys_put_le32(filter->negate, request + 8);
	sys_put_le32(filter->offset, request + 12);
	sys_put_le32(filter->size, request + 16);
	memcpy(request + 20, filter->mask, filter->size);
	memcpy(request + 20 + filter->size, filter->pattern, filter->size);
	return offload_set(dev, "pkt_filter_add", request, 20U + 2U * filter->size);
}

int wifi_infineon_offload_packet_filter_enable(const struct device *dev, uint32_t id, bool enable)
{
	uint8_t request[8];

	sys_put_le32(id, request);
	sys_put_le32(enable, request + 4);
	return offload_set(dev, "pkt_filter_enable", request, sizeof(request));
}

int wifi_infineon_offload_packet_filter_remove(const struct device *dev, uint32_t id)
{
	return offload_u32_set(dev, "pkt_filter_delete", id);
}

int wifi_infineon_offload_packet_filter_stats_get(const struct device *dev, uint32_t id,
						struct wifi_infineon_packet_filter_stats *stats)
{
	uint32_t request = sys_cpu_to_le32(id);
	uint8_t response[12];
	int ret;

	if (stats == NULL) {
		return -EINVAL;
	}
	ret = offload_xfer(dev, "pkt_filter_stats", false, &request, sizeof(request),
			   response, sizeof(response), NULL);
	if (ret == 0) {
		stats->matched = sys_get_le32(response);
		stats->discarded = sys_get_le32(response + 4);
		stats->forwarded = sys_get_le32(response + 8);
	}
	return ret;
}
