/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file infsingle_bus_sdio.c
 * @brief SDIO transport built on the Zephyr sd/sdio subsystem.
 *
 * Layer: bus. Reference: infdocs/hw.md sections 3 (power-up) and 4 (SDIO).
 *
 * Responsibilities
 *  1. Power sequencing with WL_REG_ON (hw.md 3.1).
 *  2. sd_init and enabling functions 1 and 2 with 64-byte blocks.
 *  3. Hiding the 32 KB backplane window (this module is the ONLY owner of the
 *     SB_ADDR_LOW/MID/HIGH registers, so the cached window cannot go stale).
 *  4. Zero-copy F2 frame transfers: the caller's buffer goes straight to the
 *     host controller. Cache maintenance is performed by the sdhc driver.
 *
 * The backplane window concept:
 *
 *   32-bit chip address  0x18003xyz
 *        |
 *        +-- bits 31:15 -> written to SB_ADDR_LOW/MID/HIGH  (window base)
 *        +-- bits 14:0  -> F1 register address; OR 0x8000 selects a 32-bit
 *                          access instead of 8-bit
 *
 * No Zephyr core patches are used (decision D-6): scatter/gather is not
 * available, so multi-fragment packets are linearised upstream.
 */

#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sdhc.h>
#include <zephyr/sd/sd.h>
#include <zephyr/sd/sdio.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>
#include "infsingle_bus.h"
#include "infsingle_wire.h"

LOG_MODULE_REGISTER(infsingle_bus, CONFIG_WIFI_AIROC_LOG_LEVEL);

#ifdef CONFIG_SDHC_BUFFER_ALIGNMENT
#define INFSINGLE_BUS_ALIGN CONFIG_SDHC_BUFFER_ALIGNMENT
#else
#define INFSINGLE_BUS_ALIGN 4
#endif

/* Largest single backplane transfer; matches WHD's 1536-byte limit. */
#define INFSINGLE_BP_CHUNK 1024

/* Power-up delays: hw.md 3.1 (taken from the WHD reference sequence). */
#define INFSINGLE_REG_ON_LOW_MS  10
#define INFSINGLE_REG_ON_HIGH_MS 250

struct infsingle_sdio {
	struct infsingle_bus bus;
	const struct device *sdhc;
	struct gpio_dt_spec reg_on;
	struct gpio_dt_spec host_wake;
	struct sd_card card;
	struct sdio_func f1;
	struct sdio_func f2;
	uint32_t window;           /* currently selected backplane window base */
	bool window_valid;
	infsingle_bus_irq_cb_t irq_cb;
	void *irq_user;
	/* Aligned staging buffer used only for control-path/firmware writes
	 * (blobs may live in flash that the SD DMA cannot read).
	 */
	uint8_t stage[INFSINGLE_BP_CHUNK] __aligned(INFSINGLE_BUS_ALIGN);
};

/* One Wi-Fi module per board is supported (R-ENV). */
static struct infsingle_sdio sdio_inst;

static inline struct infsingle_sdio *to_sdio(struct infsingle_bus *bus)
{
	return CONTAINER_OF(bus, struct infsingle_sdio, bus);
}

static struct sdio_func *func_of(struct infsingle_sdio *s, uint8_t func)
{
	switch (func) {
	case INFSINGLE_SDIO_FUNC_BUS:
		return &s->card.func0;
	case INFSINGLE_SDIO_FUNC_BACKPLANE:
		return &s->f1;
	default:
		return &s->f2;
	}
}

static int sdio_power_up(struct infsingle_sdio *s)
{
	int ret;

	if (!gpio_is_ready_dt(&s->reg_on)) {
		return -ENODEV;
	}
	/* Hold the core in reset long enough for it to fully discharge. */
	ret = gpio_pin_configure_dt(&s->reg_on, GPIO_OUTPUT_INACTIVE);
	if (ret) {
		return ret;
	}
	k_msleep(INFSINGLE_REG_ON_LOW_MS);
	/* Release reset; the bootrom needs time before the SD bus responds. */
	gpio_pin_set_dt(&s->reg_on, 1);
	k_msleep(INFSINGLE_REG_ON_HIGH_MS);
	return 0;
}

static int bus_init(struct infsingle_bus *bus)
{
	struct infsingle_sdio *s = to_sdio(bus);
	int ret;

	if (!device_is_ready(s->sdhc)) {
		return -ENODEV;
	}
	ret = sdio_power_up(s);
	if (ret) {
		return ret;
	}
	ret = sd_init(s->sdhc, &s->card);
	if (ret) {
		LOG_ERR("sd_init failed: %d", ret);
		return ret;
	}
	ret = sdio_init_func(&s->card, &s->card.func0, INFSINGLE_SDIO_FUNC_BUS);
	if (ret == 0) {
		ret = sdio_init_func(&s->card, &s->f1, INFSINGLE_SDIO_FUNC_BACKPLANE);
	}
	if (ret == 0) {
		ret = sdio_init_func(&s->card, &s->f2, INFSINGLE_SDIO_FUNC_WLAN);
	}
	if (ret == 0) {
		ret = sdio_set_block_size(&s->f1, INFSINGLE_SDIO_BLOCK_SIZE);
	}
	if (ret == 0) {
		ret = sdio_set_block_size(&s->f2, INFSINGLE_SDIO_BLOCK_SIZE);
	}
	if (ret == 0) {
		/* The chip ignores function 1 accesses until it is enabled. */
		ret = sdio_enable_func(&s->f1);
	}
	if (ret) {
		LOG_ERR("SDIO function init failed: %d", ret);
		return ret;
	}
	s->window_valid = false;
	return 0;
}

static int bus_deinit(struct infsingle_bus *bus)
{
	struct infsingle_sdio *s = to_sdio(bus);

	/* Reset the Wi-Fi core; it also stops all bus activity. */
	return gpio_pin_set_dt(&s->reg_on, 0);
}

#define INFSINGLE_CMD52_TRIES 3

/*
 * A CMD52 timeout (for example the first access to a sleeping device) leaves
 * the host controller command line stuck, so every later command fails until
 * the controller is reset. Reset it and retry.
 */
static int bus_reg_read8(struct infsingle_bus *bus, uint8_t func, uint32_t addr, uint8_t *val)
{
	struct infsingle_sdio *s = to_sdio(bus);
	int ret = 0;

	for (int i = 0; i < INFSINGLE_CMD52_TRIES; i++) {
		ret = sdio_read_byte(func_of(s, func), addr, val);
		if (ret == 0) {
			break;
		}
		(void)sdhc_hw_reset(s->sdhc);
	}
	return ret;
}

static int bus_reg_write8(struct infsingle_bus *bus, uint8_t func, uint32_t addr, uint8_t val)
{
	struct infsingle_sdio *s = to_sdio(bus);
	int ret = 0;

	for (int i = 0; i < INFSINGLE_CMD52_TRIES; i++) {
		ret = sdio_write_byte(func_of(s, func), addr, val);
		if (ret == 0) {
			break;
		}
		(void)sdhc_hw_reset(s->sdhc);
	}
	return ret;
}

/*
 * Select the 32 KB window containing @p addr. Only bytes that changed are
 * rewritten (typical firmware download stays in one window for 32 KB).
 */
static int bp_set_window(struct infsingle_sdio *s, uint32_t addr)
{
	uint32_t base = addr & ~(uint32_t)INFSINGLE_SB_OFFSET_MASK;
	/* SB_ADDR_LOW carries address bit 15 in its bit 7; MID carries bits
	 * 23:16; HIGH carries bits 31:24 (WHD whd_bus_sdio_set_backplane_window).
	 */
	const uint8_t val[3] = {(base >> 8) & 0x80, (base >> 16) & 0xFF, (base >> 24) & 0xFF};
	int ret = 0;

	if (s->window_valid && s->window == base) {
		return 0;
	}
	for (int i = 0; i < 3 && ret == 0; i++) {
		uint8_t old[3] = {(s->window >> 8) & 0x80, (s->window >> 16) & 0xFF,
				  (s->window >> 24) & 0xFF};

		if (!s->window_valid || val[i] != old[i]) {
			ret = sdio_write_byte(&s->f1, INFSINGLE_SB_ADDR_LOW + i, val[i]);
		}
	}
	if (ret == 0) {
		s->window = base;
		s->window_valid = true;
	} else {
		s->window_valid = false;
	}
	return ret;
}

static int bus_bp_read32(struct infsingle_bus *bus, uint32_t addr, uint32_t *val)
{
	struct infsingle_sdio *s = to_sdio(bus);
	int ret = bp_set_window(s, addr);

	if (ret) {
		return ret;
	}
	ret = sdio_read_addr(&s->f1, (addr & INFSINGLE_SB_OFFSET_MASK) | INFSINGLE_SB_ACCESS_32BIT,
			     s->stage, 4);
	if (ret == 0) {
		*val = sys_get_le32(s->stage);
	}
	return ret;
}

static int bus_bp_write32(struct infsingle_bus *bus, uint32_t addr, uint32_t val)
{
	struct infsingle_sdio *s = to_sdio(bus);
	int ret = bp_set_window(s, addr);

	if (ret) {
		return ret;
	}
	sys_put_le32(val, s->stage);
	return sdio_write_addr(&s->f1, (addr & INFSINGLE_SB_OFFSET_MASK) | INFSINGLE_SB_ACCESS_32BIT,
			       s->stage, 4);
}

/* Largest piece that stays inside the current window and the staging buffer. */
static size_t bp_piece(uint32_t addr, size_t left)
{
	size_t to_edge = INFSINGLE_SB_WINDOW_SIZE - (addr & INFSINGLE_SB_OFFSET_MASK);

	return MIN(MIN(left, to_edge), (size_t)INFSINGLE_BP_CHUNK);
}

static int bus_bp_write(struct infsingle_bus *bus, uint32_t addr, const void *buf, size_t len)
{
	/*
	 * Bulk writes (firmware, NVRAM) are split so that no piece crosses a
	 * window edge or exceeds the staging buffer; the window is re-selected
	 * per piece and only rewritten when it actually changes.
	 */
	struct infsingle_sdio *s = to_sdio(bus);
	const uint8_t *p = buf;

	while (len) {
		size_t n = bp_piece(addr, len);
		int ret = bp_set_window(s, addr);

		if (ret) {
			return ret;
		}
		memcpy(s->stage, p, n);
		/* Plain 8-bit-style bulk write; byte count stays 4-aligned. */
		ret = sdio_write_addr(&s->f1, (addr & INFSINGLE_SB_OFFSET_MASK), s->stage, n);
		if (ret) {
			return ret;
		}
		addr += n;
		p += n;
		len -= n;
	}
	return 0;
}

static int bus_bp_read(struct infsingle_bus *bus, uint32_t addr, void *buf, size_t len)
{
	struct infsingle_sdio *s = to_sdio(bus);
	uint8_t *p = buf;

	while (len) {
		size_t n = bp_piece(addr, len);
		int ret = bp_set_window(s, addr);

		if (ret) {
			return ret;
		}
		ret = sdio_read_addr(&s->f1, (addr & INFSINGLE_SB_OFFSET_MASK), s->stage, n);
		if (ret) {
			return ret;
		}
		memcpy(p, s->stage, n);
		addr += n;
		p += n;
		len -= n;
	}
	return 0;
}

static int bus_frame_read(struct infsingle_bus *bus, void *buf, size_t len)
{
	/* Data path: no staging, the host controller DMAs into the caller. */
	return sdio_read_addr(&to_sdio(bus)->f2, 0, buf, len);
}

static int bus_frame_write(struct infsingle_bus *bus, const void *buf, size_t len)
{
	/* The length is already rounded to the block size by the caller (infsingle_bus_round). */
	return sdio_write_addr(&to_sdio(bus)->f2, 0, (uint8_t *)buf, len);
}

static int bus_abort_read(struct infsingle_bus *bus)
{
	/*
	 * Used when a frame cannot be buffered or a read failed midway: the
	 * device would otherwise keep the rest of the frame queued and the
	 * stream would lose frame alignment.
	 */
	/* Writing the function number (2) to IOABORT aborts the transfer. */
	int ret = sdio_write_byte(&to_sdio(bus)->card.func0, INFSINGLE_CCCR_IOABORT,
				  INFSINGLE_SDIO_FUNC_WLAN);

	if (ret) {
		return ret;
	}
	/* Frame terminate (as WHD): the device discards the rest of the frame. */
	return bus_reg_write8(bus, INFSINGLE_SDIO_FUNC_BACKPLANE, INFSINGLE_F1_FRAME_CONTROL, 1);
}

static void sdhc_irq(const struct device *dev, int reason, const void *user)
{
	struct infsingle_sdio *s = (struct infsingle_sdio *)user;

	ARG_UNUSED(dev);
	/* Runs in the SDHC interrupt context: only forward to the registered callback. */
	if (reason == SDHC_INT_SDIO && s->irq_cb != NULL) {
		s->irq_cb(&s->bus, s->irq_user);
	}
}

static int bus_irq_set(struct infsingle_bus *bus, infsingle_bus_irq_cb_t cb, void *user)
{
	struct infsingle_sdio *s = to_sdio(bus);

	s->irq_cb = cb;
	s->irq_user = user;
	return 0;
}

/*
 * The ops member is called irq_mask because "irq_enable" is a Zephyr macro.
 * enable == true unmasks the SDIO card interrupt. The driver thread masks it
 * from the ISR and unmasks it only after draining the device, which keeps a
 * level-sensitive source from re-firing while frames are still pending.
 */
static int bus_irq_enable(struct infsingle_bus *bus, bool enable)
{
	struct infsingle_sdio *s = to_sdio(bus);

	if (enable) {
		return sdhc_enable_interrupt(s->card.sdhc, sdhc_irq, SDHC_INT_SDIO, s);
	}
	return sdhc_disable_interrupt(s->card.sdhc, SDHC_INT_SDIO);
}

static const struct infsingle_bus_ops sdio_ops = {
	.init = bus_init,
	.deinit = bus_deinit,
	.reg_read8 = bus_reg_read8,
	.reg_write8 = bus_reg_write8,
	.bp_read32 = bus_bp_read32,
	.bp_write32 = bus_bp_write32,
	.bp_write = bus_bp_write,
	.bp_read = bus_bp_read,
	.frame_read = bus_frame_read,
	.frame_write = bus_frame_write,
	.abort_read = bus_abort_read,
	.irq_set = bus_irq_set,
	.irq_mask = bus_irq_enable,
};

struct infsingle_bus *infsingle_bus_sdio_create(const struct device *sdhc,
				    const struct gpio_dt_spec *reg_on,
				    const struct gpio_dt_spec *host_wake)
{
	struct infsingle_sdio *s = &sdio_inst;

	memset(s, 0, sizeof(*s));
	s->sdhc = sdhc;
	s->reg_on = *reg_on;
	if (host_wake != NULL) {
		s->host_wake = *host_wake;
	}
	s->bus.ops = &sdio_ops;
	s->bus.align = INFSINGLE_BUS_ALIGN;
	s->bus.block = 4;
	s->bus.big_block = INFSINGLE_SDIO_BLOCK_SIZE;
	return &s->bus;
}
