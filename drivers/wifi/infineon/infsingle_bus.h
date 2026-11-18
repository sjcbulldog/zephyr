/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file infsingle_bus.h
 * @brief Bus abstraction between the chip/protocol layers and the transport.
 *
 * Layer: bus. Reference: infdocs/hw.md section 4.
 *
 * The upper layers never call the Zephyr sdio subsystem directly. They use
 * this ops table, which lets a mock bus (infsingle_bus_mock.c) back the unit tests
 * and leaves room for a future SPI transport.
 *
 * Ownership rules (R-PERF):
 *  - frame_read()/frame_write() move data directly between the device FIFO and
 *    the caller's buffer. The buffer must be aligned to bus->align and the
 *    length is rounded up by the bus to bus->block (padding lives in the
 *    caller's tailroom, never in a separate bounce buffer).
 *  - Backplane access (bp_*) hides the 32 KB window register dance. It is
 *    only for control-path accesses and firmware download, never data frames.
 *  - All functions are called from the driver thread, except irq callbacks.
 */

#ifndef ZEPHYR_DRIVERS_WIFI_INFSINGLE_INFSINGLE_BUS_H_
#define ZEPHYR_DRIVERS_WIFI_INFSINGLE_INFSINGLE_BUS_H_

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

struct infsingle_bus;

/** Callback invoked in ISR context when the device signals a frame/event. */
typedef void (*infsingle_bus_irq_cb_t)(struct infsingle_bus *bus, void *user);

struct infsingle_bus_ops {
	/** Power the device (WL_REG_ON sequence) and bring up the bus. */
	int (*init)(struct infsingle_bus *bus);
	/** Hold the device in reset and release bus resources. */
	int (*deinit)(struct infsingle_bus *bus);

	/** Read/write one register in a function (F0 CCCR, F1 direct regs). */
	int (*reg_read8)(struct infsingle_bus *bus, uint8_t func, uint32_t addr, uint8_t *val);
	int (*reg_write8)(struct infsingle_bus *bus, uint8_t func, uint32_t addr, uint8_t val);

	/** Backplane 32-bit register access (window handled internally). */
	int (*bp_read32)(struct infsingle_bus *bus, uint32_t addr, uint32_t *val);
	int (*bp_write32)(struct infsingle_bus *bus, uint32_t addr, uint32_t val);
	/** Backplane bulk access, used for firmware/NVRAM download. */
	int (*bp_write)(struct infsingle_bus *bus, uint32_t addr, const void *buf, size_t len);
	int (*bp_read)(struct infsingle_bus *bus, uint32_t addr, void *buf, size_t len);

	/** Read/write one frame on F2. len must already be rounded. */
	int (*frame_read)(struct infsingle_bus *bus, void *buf, size_t len);
	int (*frame_write)(struct infsingle_bus *bus, const void *buf, size_t len);

	/** Abort a partially read F2 frame (CCCR IOABORT / gSPI FRAME_CONTROL). */
	int (*abort_read)(struct infsingle_bus *bus);

	/**
	 * Optional (gSPI only). Report the size of the next pending F2 frame
	 * (0 = none) and acknowledge the bus interrupt when idle. When NULL the
	 * length is read from the SDPCM tag at the start of the frame.
	 */
	int (*rx_len)(struct infsingle_bus *bus, size_t *len);

	/** Optional (gSPI only). Wait until firmware accepts F2 traffic. */
	int (*wait_f2_ready)(struct infsingle_bus *bus, uint32_t timeout_ms);

	/** Register the interrupt callback and enable/disable the interrupt. */
	int (*irq_set)(struct infsingle_bus *bus, infsingle_bus_irq_cb_t cb, void *user);
	int (*irq_mask)(struct infsingle_bus *bus, bool enable);
};

struct infsingle_bus {
	const struct infsingle_bus_ops *ops;
	size_t align;   /* required buffer alignment in bytes                */
	size_t block;   /* length granularity for frame transfers            */
	/*
	 * Transfers longer than this are padded to a multiple of it so they go out
	 * as a single block-mode command instead of blocks plus a byte-mode tail
	 * (as WHD does). 0 disables the rule.
	 */
	size_t big_block;
	bool spi;       /* gSPI transport: no CCCR/SDIO-core interrupt setup */
	void *priv;     /* transport private state                           */
};

/** Round @p len up to the bus length granularity. */
static inline size_t infsingle_bus_round(const struct infsingle_bus *bus, size_t len)
{
	size_t g = (bus->big_block != 0 && len > bus->big_block) ? bus->big_block : bus->block;

	return ((len + g - 1) / g) * g;
}

/**
 * Create the SDIO transport for the Wi-Fi node. @p sdhc is the host
 * controller, the GPIOs drive WL_REG_ON and (optionally) WL_HOST_WAKE.
 */
struct infsingle_bus *infsingle_bus_sdio_create(const struct device *sdhc,
				    const struct gpio_dt_spec *reg_on,
				    const struct gpio_dt_spec *host_wake);

struct spi_dt_spec;
struct pinctrl_dev_config;

/**
 * Create the gSPI transport. @p bus_select is held low while WL_REG_ON rises
 * (selects SPI). @p pcfg is only needed when the data and interrupt lines
 * share one pin (it switches that pin between SPI data and GPIO interrupt);
 * pass NULL otherwise.
 */
struct infsingle_bus *infsingle_bus_spi_create(const struct spi_dt_spec *spi,
				    const struct gpio_dt_spec *reg_on,
				    const struct gpio_dt_spec *bus_select,
				    const struct gpio_dt_spec *host_wake,
				    const struct pinctrl_dev_config *pcfg);

#endif /* ZEPHYR_DRIVERS_WIFI_INFSINGLE_INFSINGLE_BUS_H_ */
