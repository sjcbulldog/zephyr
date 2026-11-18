/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_WIFI_INFSINGLE_STATS_H_
#define ZEPHYR_DRIVERS_WIFI_INFSINGLE_STATS_H_

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/net/net_stats.h>

struct infsingle_stats {
	struct net_stats_wifi wifi;
	uint32_t tx_dropped, rx_dropped;
	uint32_t tx_no_credit;
	uint32_t tx_flow_ctl;
	uint32_t rx_unsignaled;
	uint32_t tx_pokes;
	uint64_t bytes_copied;
	uint32_t bus_errors;
	uint32_t fw_halts;
};

struct infsingle_stats_state {
	struct k_spinlock lock;
	struct infsingle_stats data;
};

#define INFSINGLE_STATS_ADD(stats, member, value)                                                   \
	do {                                                                                      \
		struct infsingle_stats_state *_stats = (stats);                                    \
		k_spinlock_key_t _key = k_spin_lock(&_stats->lock);                                 \
		_stats->data.member += (value);                                                    \
		k_spin_unlock(&_stats->lock, _key);                                                \
	} while (false)

static inline void infsingle_stats_packet(struct infsingle_stats_state *stats, bool tx,
					 const uint8_t *frame, size_t len)
{
	static const uint8_t broadcast[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
	k_spinlock_key_t key = k_spin_lock(&stats->lock);
	struct net_stats_wifi *wifi = &stats->data.wifi;
	struct net_stats_pkts *category = NULL;

	if (tx) {
		wifi->pkts.tx++;
		wifi->bytes.sent += len;
	} else {
		wifi->pkts.rx++;
		wifi->bytes.received += len;
	}

	if (len >= sizeof(broadcast)) {
		if (memcmp(frame, broadcast, sizeof(broadcast)) == 0) {
			category = &wifi->broadcast;
		} else if ((frame[0] & 1U) != 0U) {
			category = &wifi->multicast;
		} else {
			category = &wifi->unicast;
		}
	}
	if (category != NULL) {
		if (tx) {
			category->tx++;
		} else {
			category->rx++;
		}
	}
	k_spin_unlock(&stats->lock, key);
}

static inline void infsingle_stats_drop(struct infsingle_stats_state *stats, bool tx, bool overrun)
{
	k_spinlock_key_t key = k_spin_lock(&stats->lock);

	if (tx) {
		stats->data.tx_dropped++;
		if (!overrun) {
			stats->data.wifi.errors.tx++;
		}
	} else {
		stats->data.rx_dropped++;
		if (!overrun) {
			stats->data.wifi.errors.rx++;
		}
	}
	if (overrun) {
		stats->data.wifi.overrun_count++;
	}
	k_spin_unlock(&stats->lock, key);
}

static inline void infsingle_stats_rx_bus_error(struct infsingle_stats_state *stats)
{
	k_spinlock_key_t key = k_spin_lock(&stats->lock);

	stats->data.bus_errors++;
	stats->data.wifi.errors.rx++;
	k_spin_unlock(&stats->lock, key);
}

static inline void infsingle_stats_snapshot(struct infsingle_stats_state *stats,
					   struct infsingle_stats *snapshot)
{
	k_spinlock_key_t key = k_spin_lock(&stats->lock);

	*snapshot = stats->data;
	k_spin_unlock(&stats->lock, key);
}

static inline void infsingle_stats_reset(struct infsingle_stats_state *stats)
{
	k_spinlock_key_t key = k_spin_lock(&stats->lock);

	memset(&stats->data, 0, sizeof(stats->data));
	k_spin_unlock(&stats->lock, key);
}

#endif /* ZEPHYR_DRIVERS_WIFI_INFSINGLE_STATS_H_ */
