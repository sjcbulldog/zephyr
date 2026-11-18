/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file infsingle_boot.c
 * @brief Power-up, chip identification, firmware download, interrupt setup.
 *
 * Layer: chip/boot. Reference: infdocs/hw.md sections 3, 5, 6, 12.
 *
 * Boot state machine (BLHS = bootloader handshake, Wi-Fi 6 parts):
 *
 *   POWER_ON --> ALP_CLOCK --> IDENTIFY --> BL_READY --> FW_DOWNLOAD
 *        --> TRX_PARSED --> VALIDATED --> NVRAM_DOWNLOAD --> F2_READY
 *        --> HT_CLOCK --> IRQ_SETUP --> RUNNING
 *
 *  H2D = host-to-device message register, D2H = device-to-host (F1 0x10030 and
 *  0x10038). On Wi-Fi 6 the host never touches the ARM core: the bootloader
 *  validates the image and starts it once NVRAM_DONE is signalled.
 *
 * Wi-Fi 5 parts (no bootloader) skip the handshake: the host halts and resets
 * the ARM core around the download (wifi5_prepare/wifi5_start below).
 *
 * Memory map used for download (per-chip, from infsingle_chip.c):
 *
 *   ram_base - trx_hdr_size  : TRX header + firmware (written here)
 *   ram_base + ram_size - 4  : NVRAM length token (written last)
 *   NVRAM text               : just below the token, size rounded up to 4
 */

#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>
#include "infsingle.h"
#include "infsingle_wire.h"

LOG_MODULE_REGISTER(infsingle_boot, CONFIG_WIFI_AIROC_LOG_LEVEL);

#define INFSINGLE_ALP_TIMEOUT_MS 100
#define INFSINGLE_F2_READY_TIMEOUT_MS 1000
#define INFSINGLE_CCCR_F2_READY 0x04

static int rd8(struct infsingle_dev *d, uint8_t func, uint32_t a, uint8_t *v)
{
	return d->bus->ops->reg_read8(d->bus, func, a, v);
}

static int wr8(struct infsingle_dev *d, uint8_t func, uint32_t a, uint8_t v)
{
	return d->bus->ops->reg_write8(d->bus, func, a, v);
}

/* Poll an 8-bit register until (value & mask) == mask or timeout. */
static int poll_bits(struct infsingle_dev *d, uint8_t func, uint32_t addr, uint8_t mask,
		     int timeout_ms, int step_ms)
{
	uint8_t v = 0;
	int waited = 0;
	int ret;

	while (waited <= timeout_ms) {
		ret = rd8(d, func, addr, &v);
		if (ret) {
			return ret;
		}
		if ((v & mask) == mask) {
			return 0;
		}
		k_msleep(step_ms);
		waited += step_ms;
	}
	return -ETIMEDOUT;
}

/* Wait for a D2H flag (BLHS). On failure ask the bootloader to reset. */
static int blhs_wait(struct infsingle_dev *d, uint8_t flag, const char *what)
{
	int ret = poll_bits(d, INFSINGLE_SDIO_FUNC_BACKPLANE, INFSINGLE_F1_BLHS_D2H, flag,
			    INFSINGLE_BLHS_TIMEOUT_MS, INFSINGLE_BLHS_POLL_MS);

	if (ret == -ETIMEDOUT) {
		uint8_t h2d = 0;

		LOG_ERR("BLHS timeout waiting for %s", what);
		(void)rd8(d, INFSINGLE_SDIO_FUNC_BACKPLANE, INFSINGLE_F1_BLHS_H2D, &h2d);
		(void)wr8(d, INFSINGLE_SDIO_FUNC_BACKPLANE, INFSINGLE_F1_BLHS_H2D,
			  h2d | INFSINGLE_BLHS_H2D_RESET_ON_ERROR);
	}
	return ret;
}

static int blhs_h2d_or(struct infsingle_dev *d, uint8_t bits)
{
	uint8_t v = 0;
	int ret = rd8(d, INFSINGLE_SDIO_FUNC_BACKPLANE, INFSINGLE_F1_BLHS_H2D, &v);

	return ret ? ret : wr8(d, INFSINGLE_SDIO_FUNC_BACKPLANE, INFSINGLE_F1_BLHS_H2D, v | bits);
}

/*
 * Identify the chip. The SDIO core base is not yet known, so ask the device to
 * reset the backplane window to its default (the SDIO core) via
 * DEVICE_CONTROL.ADDR_RST, then read the window registers back.
 */
static int identify(struct infsingle_dev *d)
{
	uint8_t cap, devctl, lo, mid, hi;
	uint32_t raw, addr;
	int ret;

	if (!d->bus->spi) {
		ret = rd8(d, INFSINGLE_SDIO_FUNC_BUS, INFSINGLE_CCCR_CARDCAP, &cap);
		if (ret) {
			return ret;
		}
	}
	if (!d->bus->spi && (cap & INFSINGLE_CARDCAP_CHIPID_PRESENT)) {
		ret = rd8(d, INFSINGLE_SDIO_FUNC_BACKPLANE, INFSINGLE_F1_DEVICE_CONTROL, &devctl);
		if (!ret) {
			ret = wr8(d, INFSINGLE_SDIO_FUNC_BACKPLANE, INFSINGLE_F1_DEVICE_CONTROL,
				  devctl | 0x40 /* ADDR_RST */);
		}
		if (!ret) {
			ret = rd8(d, INFSINGLE_SDIO_FUNC_BACKPLANE, INFSINGLE_SB_ADDR_LOW, &lo);
		}
		if (!ret) {
			ret = rd8(d, INFSINGLE_SDIO_FUNC_BACKPLANE, INFSINGLE_SB_ADDR_MID, &mid);
		}
		if (!ret) {
			ret = rd8(d, INFSINGLE_SDIO_FUNC_BACKPLANE, INFSINGLE_SB_ADDR_HIGH, &hi);
		}
		if (!ret) {
			ret = wr8(d, INFSINGLE_SDIO_FUNC_BACKPLANE, INFSINGLE_F1_DEVICE_CONTROL, devctl);
		}
		if (ret) {
			return ret;
		}
		addr = ((uint32_t)lo << 8) | ((uint32_t)mid << 16) | ((uint32_t)hi << 24);
		ret = d->bus->ops->bp_read32(d->bus, addr + INFSINGLE_SDIOCORE_CHIPID, &raw);
	} else {
		/* gSPI and older SDIO parts: chip ID at the start of chipcommon. */
		ret = d->bus->ops->bp_read32(d->bus, INFSINGLE_CHIPCOMMON_BASE, &raw);
		if (!d->bus->spi) {
			raw &= 0xFFFF;
		}
	}
	if (ret) {
		return ret;
	}
	uint16_t id = infsingle_chip_decode_id(raw, &d->chip_rev);

	d->chip = infsingle_chip_find(id);
	if (d->chip == NULL) {
		LOG_ERR("unsupported chip id %u (raw 0x%08x)", id, raw);
		return -ENOTSUP;
	}
	LOG_INF("chip %s rev %u", d->chip->name, d->chip_rev);
	if (CONFIG_WIFI_AIROC_CHIP_ID != 0 && id != CONFIG_WIFI_AIROC_CHIP_ID) {
		LOG_ERR("chip id %u does not match the configured part (%u); "
			"check the AIROC_PART/module selection", id, CONFIG_WIFI_AIROC_CHIP_ID);
		return -ENODEV;
	}
	return 0;
}

static int download_fw(struct infsingle_dev *d)
{
	size_t len;
	const uint8_t *fw = infsingle_fw_get(d->chip->id, INFSINGLE_BLOB_FW, &len);
	const struct infsingle_trx_hdr *trx = (const struct infsingle_trx_hdr *)fw;
	size_t total;

	if (fw == NULL) {
		return -ENOENT;
	}
	if (!d->chip->blhs) {
		/* Wi-Fi 5: raw image, no TRX container. */
		return d->bus->ops->bp_write(d->bus, d->chip->ram_base, fw, len);
	}
	if (len < sizeof(*trx)) {
		return -ENOENT;
	}
	if (sys_le32_to_cpu(trx->magic) != INFSINGLE_TRX_MAGIC) {
		LOG_ERR("firmware has bad TRX magic");
		return -EBADMSG;
	}
	/* trx->len covers the header and payload; never read beyond the blob. */
	total = MIN((size_t)sys_le32_to_cpu(trx->len), len);

	return d->bus->ops->bp_write(d->bus, d->chip->ram_base - d->chip->trx_hdr_size,
				     fw, total);
}

static int download_nvram(struct infsingle_dev *d)
{
	size_t len;
	const uint8_t *nv = infsingle_fw_get(d->chip->id, INFSINGLE_BLOB_NVRAM, &len);
	size_t padded = ROUND_UP(len, 4);
	uint32_t end, base, words;
	int ret;

	if (nv == NULL) {
		return -ENOENT;
	}
	end = d->chip->ram_size - 4 + d->chip->ram_base;
	base = end - padded;

	ret = d->bus->ops->bp_write(d->bus, base, nv, len);
	if (ret == 0 && padded != len) {
		static const uint8_t zeros[4];

		ret = d->bus->ops->bp_write(d->bus, base + len, zeros, padded - len);
	}
	if (ret) {
		return ret;
	}
	/* Token: low half = size in words, high half = its complement. */
	words = padded / 4;
	return d->bus->ops->bp_write32(d->bus, end, ((~words) << 16) | (words & 0xFFFF));
}

/*
 * Wi-Fi 5 host-controlled core reset (hw.md sections 11-13, WHD whd_chip.c).
 *
 * Each backplane core has an "wrapper" register block 0x100000 above its base.
 * IOCTRL gates the core clock (and halts the CPU), RESETCTRL holds it in
 * reset. A core is disabled by clearing IOCTRL then asserting reset, and
 * enabled by forcing the clock on, releasing reset, then dropping the force.
 */
#define INFSINGLE_AI_WRAPPER_OFFSET 0x100000
#define INFSINGLE_AI_IOCTRL         0x408
#define INFSINGLE_AI_RESETCTRL      0x800
#define INFSINGLE_AI_RESET          BIT(0)
#define INFSINGLE_AI_CLOCK_EN       BIT(0)
#define INFSINGLE_AI_FGC            BIT(1)
#define INFSINGLE_AI_CPUHALT        BIT(5)
#define INFSINGLE_SOCSRAM_BANKX_INDEX 0x10
#define INFSINGLE_SOCSRAM_BANKX_PDA   0x44

static int core_disable(struct infsingle_dev *d, uint32_t core, uint32_t ioctrl_bits)
{
	uint32_t wrap = core + INFSINGLE_AI_WRAPPER_OFFSET;
	uint32_t v;
	int ret = d->bus->ops->bp_read32(d->bus, wrap + INFSINGLE_AI_RESETCTRL, &v);

	if (ret || (v & INFSINGLE_AI_RESET)) {
		return ret;
	}
	ret = d->bus->ops->bp_write32(d->bus, wrap + INFSINGLE_AI_IOCTRL, ioctrl_bits);
	if (!ret) {
		ret = d->bus->ops->bp_read32(d->bus, wrap + INFSINGLE_AI_IOCTRL, &v);
	}
	k_msleep(1);
	if (!ret) {
		ret = d->bus->ops->bp_write32(d->bus, wrap + INFSINGLE_AI_RESETCTRL,
					      INFSINGLE_AI_RESET);
	}
	k_msleep(1);
	return ret;
}

static int core_reset(struct infsingle_dev *d, uint32_t core, uint32_t ioctrl_bits)
{
	uint32_t wrap = core + INFSINGLE_AI_WRAPPER_OFFSET;
	struct infsingle_bus *bus = d->bus;
	uint32_t v;
	int ret = core_disable(d, core, ioctrl_bits);

	if (!ret) {
		ret = bus->ops->bp_write32(bus, wrap + INFSINGLE_AI_IOCTRL,
					   INFSINGLE_AI_FGC | INFSINGLE_AI_CLOCK_EN | ioctrl_bits);
	}
	if (!ret) {
		ret = bus->ops->bp_read32(bus, wrap + INFSINGLE_AI_IOCTRL, &v);
	}
	if (!ret) {
		ret = bus->ops->bp_write32(bus, wrap + INFSINGLE_AI_RESETCTRL, 0);
	}
	k_msleep(1);
	if (!ret) {
		ret = bus->ops->bp_write32(bus, wrap + INFSINGLE_AI_IOCTRL,
					   INFSINGLE_AI_CLOCK_EN | ioctrl_bits);
	}
	if (!ret) {
		ret = bus->ops->bp_read32(bus, wrap + INFSINGLE_AI_IOCTRL, &v);
	}
	k_msleep(1);
	return ret;
}

static int core_is_up(struct infsingle_dev *d, uint32_t core)
{
	uint32_t wrap = core + INFSINGLE_AI_WRAPPER_OFFSET;
	uint32_t io, rst;
	int ret = d->bus->ops->bp_read32(d->bus, wrap + INFSINGLE_AI_IOCTRL, &io);

	if (!ret) {
		ret = d->bus->ops->bp_read32(d->bus, wrap + INFSINGLE_AI_RESETCTRL, &rst);
	}
	if (ret) {
		return ret;
	}
	if ((io & (INFSINGLE_AI_FGC | INFSINGLE_AI_CLOCK_EN)) != INFSINGLE_AI_CLOCK_EN ||
	    (rst & INFSINGLE_AI_RESET) != 0) {
		return -EIO;
	}
	return 0;
}

/* Quiesce the ARM and RAM cores so the host can write the image. */
static int wifi5_prepare(struct infsingle_dev *d)
{
	const struct infsingle_chip *c = d->chip;
	int ret;

	if (c->ram_base != 0) {
		/* ARM CR4 with RAM behind the core: just halt the CPU. */
		return core_reset(d, c->arm_core, INFSINGLE_AI_CPUHALT);
	}
	ret = core_disable(d, c->arm_core, 0);
	if (!ret) {
		ret = core_disable(d, c->socram_core, 0);
	}
	if (!ret) {
		ret = core_reset(d, c->socram_core, 0);
	}
	if (!ret && c->socsram_init) {
		/* Enable the extra SOCSRAM bank that holds part of the image. */
		ret = d->bus->ops->bp_write32(d->bus, c->socram_core +
					      INFSINGLE_SOCSRAM_BANKX_INDEX, 3);
		if (!ret) {
			ret = d->bus->ops->bp_write32(d->bus, c->socram_core +
						      INFSINGLE_SOCSRAM_BANKX_PDA, 0);
		}
	}
	return ret;
}

/* Release the ARM core after firmware and NVRAM are in RAM. */
static int wifi5_start(struct infsingle_dev *d)
{
	const struct infsingle_chip *c = d->chip;
	int ret;

	if (c->ram_base != 0) {
		/*
		 * The CPU resets to address 0, which is not RAM here: the first
		 * word of the image is the reset instruction and goes there.
		 */
		size_t len;
		const uint8_t *fw = infsingle_fw_get(c->id, INFSINGLE_BLOB_FW, &len);
		uint32_t rd;

		if (fw == NULL || len < 4) {
			return -ENOENT;
		}
		ret = d->bus->ops->bp_write(d->bus, 0, fw, 4);
		if (!ret) {
			ret = d->bus->ops->bp_read32(d->bus, 0, &rd);
		}
		if (ret) {
			return ret;
		}
		if (rd != sys_get_le32(fw)) {
			LOG_ERR("reset vector verify failed");
			return -EIO;
		}
	}
	ret = core_reset(d, c->arm_core, 0);
	if (!ret) {
		ret = core_is_up(d, c->arm_core);
		if (ret) {
			LOG_ERR("ARM core did not start: %d", ret);
		}
	}
	return ret;
}

int infsingle_boot(struct infsingle_dev *d)
{
	struct infsingle_bus *bus = d->bus;
	int ret;

	ret = bus->ops->init(bus);
	if (ret) {
		return ret;
	}

	/*
	 * Stage 1 - power and clocks. The firmware cannot be loaded until the
	 * crystal-derived ALP clock runs, and the chip ignores most backplane
	 * accesses while its sleep logic is armed, hence the KSO writes.
	 */
	if (bus->spi) {
		/* gSPI: the bus init already woke the chip; request ALP only. */
		ret = wr8(d, INFSINGLE_SDIO_FUNC_BACKPLANE, INFSINGLE_F1_CHIP_CLOCK_CSR,
			  INFSINGLE_CLK_ALP_AVAIL_REQ);
		if (!ret) {
			ret = poll_bits(d, INFSINGLE_SDIO_FUNC_BACKPLANE,
					INFSINGLE_F1_CHIP_CLOCK_CSR, INFSINGLE_CLK_ALP_AVAIL,
					INFSINGLE_ALP_TIMEOUT_MS, 1);
		}
		if (!ret) {
			ret = wr8(d, INFSINGLE_SDIO_FUNC_BACKPLANE,
				  INFSINGLE_F1_CHIP_CLOCK_CSR, 0);
		}
		if (ret) {
			LOG_ERR("ALP clock not available: %d", ret);
		}
	} else {
		/* Wake the device (KSO) first; the first write may be ignored while it sleeps. */
		(void)wr8(d, INFSINGLE_SDIO_FUNC_BACKPLANE, INFSINGLE_F1_SLEEP_CSR,
			  INFSINGLE_KSO_KEEP_WL);
		(void)wr8(d, INFSINGLE_SDIO_FUNC_BACKPLANE, INFSINGLE_F1_SLEEP_CSR,
			  INFSINGLE_KSO_KEEP_WL);

		/* ALP clock: crystal must run before backplane access is reliable. */
		ret = wr8(d, INFSINGLE_SDIO_FUNC_BACKPLANE, INFSINGLE_F1_CHIP_CLOCK_CSR,
			  INFSINGLE_CLK_FORCE_HW_CLKREQ_OFF | INFSINGLE_CLK_ALP_AVAIL_REQ |
			  INFSINGLE_CLK_FORCE_ALP);
		if (ret) {
			LOG_ERR("ALP request write failed: %d", ret);
		} else {
			uint8_t csr = 0;

			ret = poll_bits(d, INFSINGLE_SDIO_FUNC_BACKPLANE,
					INFSINGLE_F1_CHIP_CLOCK_CSR, INFSINGLE_CLK_ALP_AVAIL,
					INFSINGLE_ALP_TIMEOUT_MS, 1);
			if (ret) {
				int rret = rd8(d, INFSINGLE_SDIO_FUNC_BACKPLANE,
					       INFSINGLE_F1_CHIP_CLOCK_CSR, &csr);

				LOG_ERR("ALP clock not available: %d (CSR read %d, 0x%02x)", ret,
					rret, csr);
			}
		}
	}
	if (!ret && !bus->spi) {
		/* Enable F1 + F2; F2 only reports ready once the firmware runs. */
		ret = wr8(d, INFSINGLE_SDIO_FUNC_BUS, INFSINGLE_CCCR_IOEN, BIT(1) | BIT(2));
	}
	if (!ret) {
		ret = identify(d);
		if (ret) {
			LOG_ERR("chip identify failed: %d", ret);
		}
	}
	if (ret) {
		goto fail;
	}

	/*
	 * Stage 2 - firmware and NVRAM. Newer (BLHS) parts validate the image in
	 * their bootloader and report each step in D2H; older parts simply
	 * start running once the NVRAM token is written.
	 */
	if (d->chip->blhs) {
		/* Handshake step 1: tell the bootloader we are here, wait READY. */
		ret = wr8(d, INFSINGLE_SDIO_FUNC_BACKPLANE, INFSINGLE_F1_BLHS_H2D, INFSINGLE_BLHS_H2D_BL_INIT);
		if (!ret) {
			ret = blhs_wait(d, INFSINGLE_BLHS_D2H_READY, "READY");
		}
		if (!ret) {
			ret = wr8(d, INFSINGLE_SDIO_FUNC_BACKPLANE, INFSINGLE_F1_BLHS_H2D,
				  INFSINGLE_BLHS_H2D_DL_FW_START);
		}
		if (ret) {
			goto fail;
		}
	} else {
		ret = wifi5_prepare(d);
		if (ret) {
			LOG_ERR("core preparation failed: %d", ret);
			goto fail;
		}
	}

	ret = download_fw(d);
	if (ret) {
		LOG_ERR("firmware download failed: %d", ret);
		goto fail;
	}

	if (d->chip->blhs) {
		ret = blhs_h2d_or(d, INFSINGLE_BLHS_H2D_DL_FW_DONE);
		if (!ret) {
			ret = blhs_wait(d, INFSINGLE_BLHS_D2H_TRXHDR_PARSE, "TRX header parse");
		}
		if (!ret) {
			ret = blhs_wait(d, INFSINGLE_BLHS_D2H_VALDN_DONE, "validation done");
		}
		if (!ret) {
			ret = blhs_wait(d, INFSINGLE_BLHS_D2H_VALDN_RESULT, "validation result");
		}
		if (ret) {
			goto fail;
		}
	}

	ret = download_nvram(d);
	if (!ret && d->chip->blhs) {
		ret = blhs_h2d_or(d, INFSINGLE_BLHS_H2D_DL_NVRAM_DONE);
	}
	if (ret) {
		LOG_ERR("nvram download failed: %d", ret);
		goto fail;
	}

	if (!d->chip->blhs) {
		/* Start the ARM core, then wait for the HT clock it requests. */
		ret = wifi5_start(d);
		if (!ret) {
			ret = wr8(d, INFSINGLE_SDIO_FUNC_BACKPLANE, INFSINGLE_F1_CHIP_CLOCK_CSR,
				  INFSINGLE_CLK_HT_AVAIL_REQ);
		}
		if (!ret) {
			ret = poll_bits(d, INFSINGLE_SDIO_FUNC_BACKPLANE,
					INFSINGLE_F1_CHIP_CLOCK_CSR, INFSINGLE_CLK_HT_AVAIL,
					INFSINGLE_HT_AVAIL_TIMEOUT_MS, 1);
		}
		if (ret) {
			LOG_ERR("firmware start failed: %d", ret);
			goto fail;
		}
	}

	/*
	 * Stage 3 - hand over to firmware. F2 ready means the firmware booted and
	 * its packet interface is alive; only then are the HT clock and frame
	 * interrupts enabled.
	 */
	/* The firmware now boots; function 2 becomes ready when it is running. */
	if (bus->ops->wait_f2_ready != NULL) {
		ret = bus->ops->wait_f2_ready(bus, INFSINGLE_F2_READY_TIMEOUT_MS);
	} else {
		ret = poll_bits(d, INFSINGLE_SDIO_FUNC_BUS, 0x03 /* CCCR_IORDY */,
				INFSINGLE_CCCR_F2_READY, INFSINGLE_F2_READY_TIMEOUT_MS, 1);
	}
	if (ret) {
		LOG_ERR("function 2 never became ready (check NVRAM)");
		goto fail;
	}
	/* HT clock: needed for the full-speed backplane. */
	ret = wr8(d, INFSINGLE_SDIO_FUNC_BACKPLANE, INFSINGLE_F1_CHIP_CLOCK_CSR, INFSINGLE_CLK_HT_AVAIL_REQ);
	if (!ret) {
		ret = poll_bits(d, INFSINGLE_SDIO_FUNC_BACKPLANE, INFSINGLE_F1_CHIP_CLOCK_CSR,
				INFSINGLE_CLK_HT_AVAIL, INFSINGLE_HT_AVAIL_TIMEOUT_MS, 1);
	}
	/* Interrupts: frame-available in INT_HOST_MASK, F1+F2 enabled, then the
	 * CCCR master + function enables (master=BIT0, F1=BIT1, F2=BIT2). gSPI has
	 * neither the function mask nor the CCCR; its interrupts were set up at bus init.
	 */
	if (!ret) {
		ret = bus->ops->bp_write32(bus, d->chip->sdio_core + INFSINGLE_SDIOCORE_INT_HOST_MASK,
					   INFSINGLE_INT_HOST_MASK_VALUE);
	}
	if (!ret && !bus->spi) {
		ret = bus->ops->bp_write32(bus, d->chip->sdio_core + INFSINGLE_SDIOCORE_FUNC_INT_MASK,
					   BIT(1) | BIT(2));
	}
	if (!ret) {
		ret = wr8(d, INFSINGLE_SDIO_FUNC_BACKPLANE, INFSINGLE_F1_FUNC2_WATERMARK,
			  bus->spi ? INFSINGLE_FUNC2_WATERMARK_SPI : INFSINGLE_FUNC2_WATERMARK_VAL);
	}
	if (!ret && !bus->spi) {
		ret = wr8(d, INFSINGLE_SDIO_FUNC_BUS, INFSINGLE_CCCR_INTEN, 0x07);
	}
	if (ret) {
		goto fail;
	}
	LOG_INF("firmware running");
	return 0;

fail:
	LOG_ERR("boot failed: %d", ret);
	(void)bus->ops->deinit(bus);
	return ret;
}
