/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_WIFI_INFINEON_OFFLOAD_H_
#define ZEPHYR_INCLUDE_DRIVERS_WIFI_INFINEON_OFFLOAD_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup wifi_infineon_offload Infineon firmware offloads
 * @ingroup wifi_mgmt
 * @{
 *
 * Driver-specific, synchronous, supervisor-only extensions to native networking.
 * Call from a thread, not an ISR or a driver event callback. Buffers belong to
 * the caller and need only remain valid until the call returns. Integer values
 * use host byte order; IP addresses, MAC addresses and packet bytes use network
 * byte order. No address or DHCP synchronization with the network stack is
 * automatic. See drivers/wifi/infineon/USERGUIDE.md.
 *
 * All int-returning functions return 0 on success, -EINVAL for invalid arguments,
 * -ENODEV for a non-Infineon device, -EAGAIN for an unready device,
 * -EWOULDBLOCK from an ISR, -EDEADLK from the driver thread, -ENOTSUP for an
 * unsupported firmware command, -EMSGSIZE for oversized data, -EBADMSG for a
 * malformed response, -ENOBUFS for an insufficient firmware workspace,
 * -ENOTCONN when association is required, -ETIMEDOUT for a firmware timeout,
 * or -EIO for other firmware failures. Partial updates are not rolled back.
 */

/** Firmware profile features; these bits are not a supported-feature bitmap. */
enum wifi_infineon_offload {
	WIFI_INFINEON_OFFLOAD_ARP = BIT(0),       /**< IPv4 ARP responder. */
	WIFI_INFINEON_OFFLOAD_ND = BIT(1),        /**< IPv6 Neighbor Discovery. */
	WIFI_INFINEON_OFFLOAD_BDO = BIT(2),       /**< Firmware-defined BDO. */
	WIFI_INFINEON_OFFLOAD_ICMP = BIT(3),      /**< ICMP echo requests. */
	WIFI_INFINEON_OFFLOAD_TCP = BIT(4),       /**< TCP keepalive (TKO). */
	WIFI_INFINEON_OFFLOAD_DHCP = BIT(5),      /**< DHCP lease renewal. */
	WIFI_INFINEON_OFFLOAD_PNO = BIT(6),       /**< Preferred network scanning. */
	WIFI_INFINEON_OFFLOAD_KEEPALIVE = BIT(7), /**< Periodic packet transmission. */
	WIFI_INFINEON_OFFLOAD_GTK = BIT(8),       /**< GTK offload engine. */
	WIFI_INFINEON_OFFLOAD_PACKET_FILTER = BIT(9), /**< Low-power packet filters. */
	WIFI_INFINEON_OFFLOAD_PROFILE = BIT(10),  /**< Query-only general profile support. */
};

/** All configurable profile bits, excluding the query-only PROFILE value. */
#define WIFI_INFINEON_OFFLOAD_ALL GENMASK(9, 0)
/** Maximum keepalive packet size in bytes. */
#define WIFI_INFINEON_OFFLOAD_MAX_PACKET_SIZE 1400U
/** Maximum packet filter mask or pattern size in bytes. */
#define WIFI_INFINEON_OFFLOAD_MAX_FILTER_SIZE 640U
/** Maximum number of ARP host addresses. */
#define WIFI_INFINEON_OFFLOAD_MAX_IPV4_ADDRS 8U
/** Maximum number of TCP keepalive connections supported by this interface. */
#define WIFI_INFINEON_OFFLOAD_MAX_TCP_CONNECTIONS 16U

/**
 * @brief Query support without changing firmware configuration.
 *
 * ARP, ND, TCP, keepalive and packet filter queries probe their control iovars.
 * ICMP and PROFILE query exact firmware capability tokens. BDO, DHCP, PNO and
 * GTK have no reliable individual query in this firmware ABI.
 *
 * @param dev Ready Infineon network device.
 * @param feature Exactly one enum wifi_infineon_offload value.
 * @param supported Non-NULL output, set false unless support is confirmed.
 * @retval 0 Query completed; inspect @p supported.
 * @retval -ENODATA No individual query exists, or firmware has no keepalive getter.
 * @return Other negative errno values described in the group documentation.
 */
int wifi_infineon_offload_is_supported(const struct device *dev,
				      enum wifi_infineon_offload feature, bool *supported);

/**
 * @brief Select or reset the firmware low-power profile.
 * @param dev Ready Infineon network device.
 * @param features OR of profile bits, or 0 to skip all features.
 * @param reset Remove the profile instead of selecting the low-power profile.
 * @retval 0 Profile updated. This does not enable TCP auto-configuration.
 * @return Negative errno described in the group documentation.
 */
int wifi_infineon_offload_profile_set(const struct device *dev, uint32_t features, bool reset);

/**
 * @brief Activate or deactivate selected profile features.
 * @param dev Ready Infineon network device.
 * @param features OR of profile bits, or 0.
 * @param enable True to activate, false to deactivate.
 * @retval 0 Activation updated.
 * @return Negative errno described in the group documentation.
 */
int wifi_infineon_offload_profile_enable(const struct device *dev, uint32_t features, bool enable);

/**
 * @brief Add or remove an IPv4 host address through the profile.
 * @param dev Ready Infineon network device.
 * @param features OR of profile bits affected by the update.
 * @param address Non-NULL IPv4 address in network byte order.
 * @param add True to add, false to remove.
 * @retval 0 Address updated.
 * @return Negative errno described in the group documentation.
 */
int wifi_infineon_offload_ipv4_update(const struct device *dev, uint32_t features,
				     const struct in_addr *address, bool add);

/**
 * @brief Add or remove an IPv6 host address through the profile.
 * @param dev Ready Infineon network device.
 * @param features OR of profile bits affected by the update.
 * @param address Non-NULL IPv6 address in network byte order.
 * @param anycast True for anycast, false for unicast.
 * @param add True to add, false to remove.
 * @retval 0 Address updated.
 * @return Negative errno described in the group documentation.
 */
int wifi_infineon_offload_ipv6_update(const struct device *dev, uint32_t features,
				     const struct in6_addr *address, bool anycast, bool add);

/** ARP agent feature bits. */
enum wifi_infineon_arp_feature {
	WIFI_INFINEON_ARP_AGENT = BIT(0),      /**< Enable ARP agent processing. */
	WIFI_INFINEON_ARP_SNOOP = BIT(1),      /**< Learn addresses from ARP traffic. */
	WIFI_INFINEON_ARP_HOST_REPLY = BIT(2), /**< Reply on behalf of the host. */
	WIFI_INFINEON_ARP_PEER_REPLY = BIT(3), /**< Reply on behalf of peers. */
};

/** ARP agent configuration, independent of the general offload profile. */
struct wifi_infineon_arp_config {
	bool enable;          /**< Enable the ARP offload agent. */
	uint32_t features;    /**< OR of enum wifi_infineon_arp_feature bits. */
	uint32_t peer_age;    /**< Maximum peer cache entry age, in seconds. */
};

/**
 * @brief Configure the ARP agent, including peer cache age and feature flags.
 * @param dev Ready Infineon network device.
 * @param config Non-NULL configuration.
 * @retval 0 Configuration applied.
 * @return Negative errno described in the group documentation.
 */
int wifi_infineon_offload_arp_set(const struct device *dev,
				const struct wifi_infineon_arp_config *config);

/**
 * @brief Read the ARP agent configuration.
 * @param dev Ready Infineon network device.
 * @param config Non-NULL output configuration.
 * @retval 0 Configuration read.
 * @return Negative errno described in the group documentation.
 */
int wifi_infineon_offload_arp_get(const struct device *dev,
				struct wifi_infineon_arp_config *config);

/**
 * @brief Add an ARP host address without requiring general profile support.
 * @param dev Ready Infineon network device.
 * @param address Non-NULL IPv4 address in network byte order.
 * @retval 0 Address added.
 * @return Negative errno described in the group documentation.
 */
int wifi_infineon_offload_arp_address_add(const struct device *dev, const struct in_addr *address);

/**
 * @brief Read the ARP host address table.
 * @param dev Ready Infineon network device.
 * @param addresses Array of WIFI_INFINEON_OFFLOAD_MAX_IPV4_ADDRS addresses.
 * @param count Non-NULL output number of nonzero entries.
 * @retval 0 Table read.
 * @return Negative errno described in the group documentation.
 */
int wifi_infineon_offload_arp_addresses_get(const struct device *dev, struct in_addr *addresses,
					  size_t *count);

/**
 * @brief Clear the ARP host address table.
 * @param dev Ready Infineon network device.
 * @retval 0 Table cleared.
 * @return Negative errno described in the group documentation.
 */
int wifi_infineon_offload_arp_addresses_clear(const struct device *dev);

/**
 * @brief Clear the ARP peer cache without removing host addresses.
 * @param dev Ready Infineon network device.
 * @retval 0 Cache cleared.
 * @return Negative errno described in the group documentation.
 */
int wifi_infineon_offload_arp_cache_clear(const struct device *dev);

/**
 * @brief Configure periodic NULL or supplied-packet keepalives.
 * @param dev Ready Infineon network device.
 * @param period_ms Period in milliseconds; 0 disables keepalive.
 * @param packet Packet bytes, or NULL when @p length is 0 for NULL keepalive.
 * @param length Packet length, 0 through WIFI_INFINEON_OFFLOAD_MAX_PACKET_SIZE.
 * @retval 0 Keepalive configured.
 * @return Negative errno described in the group documentation.
 */
int wifi_infineon_offload_keepalive_set(const struct device *dev, uint32_t period_ms,
				      const uint8_t *packet, size_t length);

/** TCP keepalive timing. Zero fields select defaults of 20 s, 3 s and 3 retries. */
struct wifi_infineon_tcp_keepalive_config {
	uint16_t interval;       /**< Probe interval, in seconds. */
	uint16_t retry_interval; /**< Retry interval, in seconds. */
	uint16_t retry_count;    /**< Maximum number of retries. */
};

/** TCP auto-configuration filter. Zero fields are wildcards. */
struct wifi_infineon_tcp_keepalive_filter {
	uint16_t source_port;     /**< Local port, host byte order. */
	uint16_t destination_port; /**< Remote port, host byte order. */
	uint8_t source_ip[16];    /**< Network-order IPv6, or IPv4 followed by 12 zeros. */
	uint8_t destination_ip[16]; /**< Network-order IPv6, or IPv4 followed by 12 zeros. */
};

/** Firmware TCP keepalive status codes. */
enum wifi_infineon_tcp_keepalive_state {
	WIFI_INFINEON_TCP_NORMAL = 0,          /**< No error. */
	WIFI_INFINEON_TCP_NO_RESPONSE = 1,     /**< Peer did not respond. */
	WIFI_INFINEON_TCP_NO_ACK = 2,          /**< ACK flag missing. */
	WIFI_INFINEON_TCP_UNEXPECTED_FLAGS = 3, /**< Unexpected TCP flags. */
	WIFI_INFINEON_TCP_BAD_SEQUENCE = 4,    /**< Invalid acknowledgment number. */
	WIFI_INFINEON_TCP_BAD_REMOTE_SEQUENCE = 5, /**< Invalid peer sequence number. */
	WIFI_INFINEON_TCP_DATA = 6,            /**< Peer has data for the host. */
	WIFI_INFINEON_TCP_UNAVAILABLE = 255,   /**< Connection not configured. */
};

/** TCP status; query before disabling TCP keepalive. */
struct wifi_infineon_tcp_keepalive_status {
	uint8_t count; /**< Number of valid entries, at most sixteen. */
	uint8_t status[WIFI_INFINEON_OFFLOAD_MAX_TCP_CONNECTIONS]; /**< Firmware status codes. */
};

/**
 * @brief Set TCP probe timing.
 * @param dev Ready Infineon network device.
 * @param config Non-NULL timing configuration.
 * @retval 0 Timing configured.
 * @return Negative errno described in the group documentation.
 */
int wifi_infineon_offload_tcp_set(const struct device *dev,
				const struct wifi_infineon_tcp_keepalive_config *config);

/**
 * @brief Read TCP probe timing.
 * @param dev Ready Infineon network device.
 * @param config Non-NULL output configuration.
 * @retval 0 Timing read.
 * @return Negative errno described in the group documentation.
 */
int wifi_infineon_offload_tcp_get(const struct device *dev,
				struct wifi_infineon_tcp_keepalive_config *config);

/**
 * @brief Enable or disable TCP keepalive and firmware connection auto-configuration.
 * @param dev Ready Infineon network device.
 * @param enable True to enable, false to disable.
 * @retval 0 Both controls updated.
 * @return Negative errno described in the group documentation.
 */
int wifi_infineon_offload_tcp_enable(const struct device *dev, bool enable);

/**
 * @brief Set the TCP auto-configuration filter.
 *
 * Disable TCP keepalive before changing the filter, then re-enable it.
 * @param dev Ready Infineon network device.
 * @param filter Non-NULL filter; zero fields match any value.
 * @retval 0 Filter configured.
 * @return Negative errno described in the group documentation.
 */
int wifi_infineon_offload_tcp_filter_set(const struct device *dev,
				       const struct wifi_infineon_tcp_keepalive_filter *filter);

/**
 * @brief Read TCP connection status.
 * @param dev Ready Infineon network device.
 * @param status Non-NULL output status.
 * @retval 0 Status read.
 * @return Negative errno described in the group documentation.
 */
int wifi_infineon_offload_tcp_status_get(const struct device *dev,
				       struct wifi_infineon_tcp_keepalive_status *status);

/** IP address for ICMP offload. */
struct wifi_infineon_offload_ip {
	sa_family_t family; /**< AF_INET or AF_INET6. */
	union {
		struct in_addr ipv4;   /**< Network-order IPv4 address. */
		struct in6_addr ipv6;  /**< Network-order IPv6 address. */
	} address; /**< Address selected by family. */
};

/** ICMP echo request peer configuration. */
struct wifi_infineon_icmp_peer {
	struct wifi_infineon_offload_ip ip; /**< Peer address. */
	uint8_t mac[6];                     /**< Peer Ethernet address. */
	uint32_t period;                    /**< Probe interval in seconds, nonzero. */
	uint32_t duration;                  /**< Test duration in seconds, nonzero. */
};

/** ICMP peer commands. */
enum wifi_infineon_icmp_action {
	WIFI_INFINEON_ICMP_DELETE = 2, /**< Remove the peer. */
	WIFI_INFINEON_ICMP_START = 3,  /**< Start probing the peer. */
	WIFI_INFINEON_ICMP_STOP = 4,   /**< Stop probing the peer. */
};

/** ICMP peer state as reported by firmware. */
struct wifi_infineon_icmp_info {
	bool enable;                         /**< ICMP engine enabled. */
	uint8_t count;                       /**< Number of firmware peers. */
	uint32_t state;                      /**< Firmware-specific peer state. */
	struct wifi_infineon_icmp_peer peer; /**< Current peer configuration. */
};

/** ICMP completion reasons. */
enum wifi_infineon_icmp_reason {
	WIFI_INFINEON_ICMP_SUCCESS = 0,    /**< Connectivity check succeeded. */
	WIFI_INFINEON_ICMP_TIMEOUT = 1,    /**< Connectivity check timed out. */
	WIFI_INFINEON_ICMP_DISCONNECT = 2, /**< Link disconnected. */
};

/** Decoded ICMP completion event. */
struct wifi_infineon_icmp_event {
	struct wifi_infineon_offload_ip ip; /**< Peer address. */
	uint8_t reason;                    /**< enum wifi_infineon_icmp_reason value. */
	uint32_t requests;                 /**< Number of echo requests sent. */
};

/**
 * @brief Handle an ICMP completion in the driver thread; do not block or call driver APIs.
 * @param dev Infineon network device.
 * @param event Event valid only during the callback.
 * @param user_data Context supplied during registration.
 */
typedef void (*wifi_infineon_icmp_callback_t)(const struct device *dev,
					    const struct wifi_infineon_icmp_event *event,
					    void *user_data);

/**
 * @brief Enable or disable the ICMP echo request engine.
 * @param dev Ready Infineon network device.
 * @param enable True to enable, false to disable.
 * @retval 0 Engine updated.
 * @return Negative errno described in the group documentation.
 */
int wifi_infineon_offload_icmp_enable(const struct device *dev, bool enable);

/**
 * @brief Add an ICMP peer.
 * @param dev Ready Infineon network device.
 * @param peer Non-NULL peer configuration.
 * @retval 0 Peer added.
 * @return Negative errno described in the group documentation.
 */
int wifi_infineon_offload_icmp_peer_add(const struct device *dev,
				      const struct wifi_infineon_icmp_peer *peer);

/**
 * @brief Start, stop or delete an ICMP peer.
 * @param dev Ready Infineon network device.
 * @param ip Non-NULL peer address.
 * @param action Valid enum wifi_infineon_icmp_action value.
 * @retval 0 Command completed.
 * @return Negative errno described in the group documentation.
 */
int wifi_infineon_offload_icmp_peer_action(const struct device *dev,
					 const struct wifi_infineon_offload_ip *ip,
					 enum wifi_infineon_icmp_action action);

/**
 * @brief Read an ICMP peer configuration and state.
 * @param dev Ready Infineon network device.
 * @param ip Non-NULL peer address.
 * @param info Non-NULL output information.
 * @retval 0 Information read.
 * @return Negative errno described in the group documentation.
 */
int wifi_infineon_offload_icmp_info_get(const struct device *dev,
				      const struct wifi_infineon_offload_ip *ip,
				      struct wifi_infineon_icmp_info *info);

/**
 * @brief Replace the ICMP callback or unregister it with NULL.
 *
 * A callback already executing may finish after unregistration returns. Keep
 * its context alive until it completes. Offload APIs cannot be called from it.
 * @param dev Ready Infineon network device.
 * @param callback Callback, or NULL to unregister.
 * @param user_data Application context, possibly NULL.
 * @retval 0 Callback updated.
 * @return Negative errno described in the group documentation.
 */
int wifi_infineon_offload_icmp_callback_set(const struct device *dev,
					  wifi_infineon_icmp_callback_t callback, void *user_data);

/** Pattern filter for firmware low-power receive filtering. */
struct wifi_infineon_packet_filter {
	uint32_t id;            /**< Application-selected filter ID. */
	bool negate;           /**< True to invert the match. */
	uint32_t offset;        /**< Byte offset from the start of the Ethernet frame. */
	size_t size;            /**< Mask/pattern size, 1 through MAX_FILTER_SIZE. */
	const uint8_t *mask;    /**< Non-NULL mask, @p size bytes. */
	const uint8_t *pattern; /**< Non-NULL pattern, @p size bytes. */
};

/** Firmware packet filter counters. */
struct wifi_infineon_packet_filter_stats {
	uint32_t matched;   /**< Packets matching the filter. */
	uint32_t discarded; /**< Packets discarded by firmware. */
	uint32_t forwarded; /**< Packets forwarded to the host. */
};

/**
 * @brief Add a filter; enable it separately after successful creation.
 * @param dev Ready Infineon network device.
 * @param filter Non-NULL filter configuration.
 * @retval 0 Filter added.
 * @return Negative errno described in the group documentation.
 */
int wifi_infineon_offload_packet_filter_add(const struct device *dev,
					  const struct wifi_infineon_packet_filter *filter);

/**
 * @brief Enable or disable an existing filter.
 * @param dev Ready Infineon network device.
 * @param id Filter ID.
 * @param enable True to enable, false to disable.
 * @retval 0 Filter updated.
 * @return Negative errno described in the group documentation.
 */
int wifi_infineon_offload_packet_filter_enable(const struct device *dev, uint32_t id, bool enable);

/**
 * @brief Remove an existing filter.
 * @param dev Ready Infineon network device.
 * @param id Filter ID.
 * @retval 0 Filter removed.
 * @return Negative errno described in the group documentation.
 */
int wifi_infineon_offload_packet_filter_remove(const struct device *dev, uint32_t id);

/**
 * @brief Read filter counters.
 * @param dev Ready Infineon network device.
 * @param id Filter ID.
 * @param stats Non-NULL output counters.
 * @retval 0 Counters read.
 * @return Negative errno described in the group documentation.
 */
int wifi_infineon_offload_packet_filter_stats_get(const struct device *dev, uint32_t id,
						struct wifi_infineon_packet_filter_stats *stats);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_WIFI_INFINEON_OFFLOAD_H_ */
