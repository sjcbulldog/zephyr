/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file infsingle_fw.c
 * @brief Firmware / NVRAM / CLM blob access.
 *
 * Layer: chip (boot support). Reference: infdocs/hw.md section 6.
 *
 * Exactly one device is linked into an image. The board selects it with the
 * AIROC part and module Kconfig choices (for example CYW55500 plus
 * CYW55513_MOD_PSE84_SOM); CMakeLists.txt maps that choice to the firmware,
 * CLM and NVRAM files in the Infineon HAL module and runs gen_blob.py, which
 * converts each file into a const byte array with these fixed names:
 *
 *     const uint8_t  infsingle_blob_<fw|nvram|clm>[];
 *     const uint32_t infsingle_blob_<fw|nvram|clm>_len;
 *
 * Because only the present device's blobs exist in the image, no per-chip
 * lookup table is needed and unused firmware costs no flash. The chip ID is
 * checked against CONFIG_WIFI_AIROC_CHIP_ID by the boot code, so a board
 * with the wrong part selected fails with a clear message instead of
 * downloading an incompatible image. The arrays are never copied: boot
 * streams them to the device in bounded chunks from their flash location.
 *
 * Only the production firmware is selected unless the user explicitly enables
 * CONFIG_WIFI_AIROC_MFG_FIRMWARE (hw.md 6.4, decision D-9).
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "infsingle.h"

LOG_MODULE_REGISTER(infsingle_fw, CONFIG_WIFI_AIROC_LOG_LEVEL);

extern const uint8_t infsingle_blob_fw[];
extern const uint32_t infsingle_blob_fw_len;
extern const uint8_t infsingle_blob_nvram[];
extern const uint32_t infsingle_blob_nvram_len;
extern const uint8_t infsingle_blob_clm[];
extern const uint32_t infsingle_blob_clm_len;

/* chip_id is unused: the image holds blobs for the configured device only. */
const uint8_t *infsingle_fw_get(uint16_t chip_id, enum infsingle_blob which, size_t *len)
{
	ARG_UNUSED(chip_id);

	switch (which) {
	case INFSINGLE_BLOB_FW:
		*len = infsingle_blob_fw_len;
		return infsingle_blob_fw;
	case INFSINGLE_BLOB_NVRAM:
		*len = infsingle_blob_nvram_len;
		return infsingle_blob_nvram;
	default:
		*len = infsingle_blob_clm_len;
		return infsingle_blob_clm;
	}
}
