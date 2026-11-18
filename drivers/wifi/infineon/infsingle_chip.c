/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file infsingle_chip.c
 * @brief Chip table. Layer: chip. Reference: infdocs/hw.md sections 6, 13.4.
 *
 * Values come from WHD whd_chip_constants.c (get_sdiod_core_base_address,
 * get_pmu_base_address, get_chip_ram_size, get_atcm_ram_base_address).
 * Wi-Fi 6 parts use the bootloader handshake; Wi-Fi 5 parts (blhs == false)
 * use the host-controlled core reset in infsingle_boot.c. Only the Wi-Fi 6
 * rows are verified on hardware (kit_pse84_eval).
 */

#include <zephyr/sys/util.h>
#include "infsingle_chip.h"
#include "infsingle_wire.h"

static const struct infsingle_chip chips[] = {
	{
		/* CYW5557x family (Hatchet1-A1 / CP). TRXv5 header. */
		.id = 55500, .name = "CYW55500", .wifi6 = true, .blhs = true,
		.sdio_core = 0x18003000, .pmu = 0x18012000,
		.ram_base = 0x3A0000 + 0x20 + 0x1000,
		.ram_size = 0xE0000 - 0x20 - 0x1000,
		.trx_hdr_size = INFSINGLE_TRXV5_SIZE,
	},
	{
		.id = 55530, .name = "CYW55530", .wifi6 = true, .blhs = true,
		.sdio_core = 0x18003000, .pmu = 0x18012000,
		.ram_base = 0x3A0000 + 0x20 + 0x1000,
		.ram_size = 0xE0000 - 0x20 - 0x1000,
		.trx_hdr_size = INFSINGLE_TRXV5_SIZE,
	},
	{
		/* CYW55560. TRXv4 header (692 bytes). */
		.id = 55560, .name = "CYW55560", .wifi6 = true, .blhs = true,
		.sdio_core = 0x18004000, .pmu = 0x18012000,
		.ram_base = 0x370000 + 0x2B4 + 0x800,
		.ram_size = 0x150000 - 0x800 - 0x2B4,
		.trx_hdr_size = INFSINGLE_TRXV4_SIZE,
	},
	/*
	 * Wi-Fi 5 parts. Raw firmware image, host-controlled ARM reset, no
	 * bootloader handshake. These rows are untested on hardware.
	 */
	{
		/* CYW4343W and CYW43438 both report 43430. */
		.id = 43430, .name = "CYW4343x", .wifi6 = false, .blhs = false,
		.sdio_core = 0x18002000, .pmu = 0x18000000,
		.arm_core = 0x18003000, .socram_core = 0x18004000, .socsram_init = true,
		.ram_base = 0, .ram_size = 512 * 1024,
	},
	{
		.id = 43439, .name = "CYW43439", .wifi6 = false, .blhs = false,
		.sdio_core = 0x18002000, .pmu = 0x18000000,
		.arm_core = 0x18003000, .socram_core = 0x18004000, .socsram_init = true,
		.ram_base = 0, .ram_size = 512 * 1024,
	},
	{
		.id = 43012, .name = "CYW43012", .wifi6 = false, .blhs = false,
		.sdio_core = 0x18002000, .pmu = 0x18012000,
		.arm_core = 0x18003000, .socram_core = 0x18004000,
		.ram_base = 0, .ram_size = 0xA0000,
	},
	{
		/* CYW4373: ARM CR4 with RAM at 0x160000; core halted during download. */
		.id = 0x4373, .name = "CYW4373", .wifi6 = false, .blhs = false,
		.sdio_core = 0x18005000, .pmu = 0x18000000,
		.arm_core = 0x18002000,
		.ram_base = 0x160000, .ram_size = 0xE0000,
	},
};

const struct infsingle_chip *infsingle_chip_find(uint16_t id)
{
	for (size_t i = 0; i < ARRAY_SIZE(chips); i++) {
		if (chips[i].id == id) {
			return &chips[i];
		}
	}
	return NULL;
}

uint16_t infsingle_chip_decode_id(uint32_t raw, uint8_t *rev)
{
	if (rev != NULL) {
		*rev = (raw >> 16) & 0xF;
	}
	return raw & 0xFFFF;
}
