/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file infsingle_chip.h
 * @brief Per-chip constants, selected at run time from the chip ID.
 *
 * Layer: chip. Reference: infdocs/hw.md sections 6 and 13.
 */

#ifndef ZEPHYR_DRIVERS_WIFI_INFSINGLE_INFSINGLE_CHIP_H_
#define ZEPHYR_DRIVERS_WIFI_INFSINGLE_INFSINGLE_CHIP_H_

#include <stdint.h>
#include <stdbool.h>

struct infsingle_chip {
	uint16_t id;           /* decimal chip ID, e.g. 55500                 */
	const char *name;
	bool wifi6;
	bool blhs;             /* bootloader handshake before firmware start  */
	uint32_t sdio_core;    /* SDIO device core base                       */
	uint32_t pmu;          /* PMU core base                               */
	uint32_t ram_base;     /* where the TRX image is written              */
	uint32_t ram_size;     /* usable RAM for the NVRAM-at-end calculation */
	uint32_t trx_hdr_size; /* TRX header bytes placed below ram_base      */
	/*
	 * Wi-Fi 5 parts (blhs == false) have no bootloader: the host halts and
	 * resets the ARM core itself and downloads a raw (header-less) image.
	 */
	uint32_t arm_core;     /* ARM CM3/CR4 core base, host-controlled reset */
	uint32_t socram_core;  /* SOCRAM core base, 0 when not used           */
	bool socsram_init;     /* extra SOCSRAM bank setup (43430, 43439)     */
};

/** Look up a chip by ID. Returns NULL when unsupported. */
const struct infsingle_chip *infsingle_chip_find(uint16_t id);

/**
 * Decode the 32-bit chip-ID register.
 * @param raw value read from SDIO core + 0x330 (or 16 bits at chipcommon).
 * @param[out] rev silicon revision (bits 19:16), may be NULL.
 * @return chip ID (bits 15:0).
 */
uint16_t infsingle_chip_decode_id(uint32_t raw, uint8_t *rev);

#endif /* ZEPHYR_DRIVERS_WIFI_INFSINGLE_INFSINGLE_CHIP_H_ */
