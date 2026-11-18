/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file infsingle_bus_spi.c
 * @brief gSPI transport built on the Zephyr SPI API.
 *
 * Layer: bus. Reference: infdocs/hw.md section 5 and the WHD gSPI protocol.
 *
 * Every transfer is one chip-select cycle: a 32-bit command word followed by
 * data. The command carries the direction, the function (0 bus, 1 backplane,
 * 2 WLAN), the register/FIFO address and the length:
 *
 *   bit 31      write
 *   bit 30      incrementing address
 *   bits 29:28  function
 *   bits 27:11  address
 *   bits 10:0   length (0 means 2048)
 *
 * Until the chip is switched to 32-bit word mode (SPI_BUS_CONTROL) it sends
 * and expects every 32-bit quantity with the bytes swapped inside each 16-bit
 * half. Only the start-up sequence runs in that mode.
 *
 * Differences from SDIO that this module hides from the upper layers:
 *  - Function 1 reads return 4 bytes of response delay before the data.
 *  - Backplane transfers are limited to 64 bytes.
 *  - The length of a received F2 frame comes from STATUS_REGISTER, not from
 *    the first bytes of the frame (rx_len op), and the frame is read in one
 *    transfer.
 *  - A transmit needs STATUS_REGISTER to report F2 ready first.
 *  - The interrupt is the WL_HOST_WAKE pin. On boards where the data line and
 *    the interrupt share one pin, the pin is switched to a GPIO input while
 *    the bus is idle and back to SPI data around each transfer.
 *
 * Frames are copied into a staging buffer for writes (command word and data
 * must be contiguous); reads go straight into the caller's buffer.
 */

#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>
#include "infsingle_bus.h"
#include "infsingle_wire.h"

LOG_MODULE_REGISTER(infsingle_bus_spi, CONFIG_WIFI_AIROC_LOG_LEVEL);

/* Pin state with the shared data/interrupt pin configured as a GPIO input. */
#define INFSINGLE_PINCTRL_STATE_HOST_WAKE PINCTRL_STATE_PRIV_START

/* gSPI function numbers (same values as the SDIO function numbers). */
#define GSPI_FUNC_BUS  0
#define GSPI_FUNC_BP   1
#define GSPI_FUNC_WLAN 2

#define GSPI_MAX_XFER  2048
#define GSPI_BP_CHUNK  64  /* largest backplane (F1) transfer */
#define GSPI_BP_RD_PAD 4   /* response delay bytes before F1 read data */

/* Function 0 registers. */
#define GSPI_BUS_CONTROL     0x00
#define GSPI_INT_REG         0x04
#define GSPI_INT_ENABLE      0x06
#define GSPI_STATUS_REG      0x08
#define GSPI_READ_TEST_REG   0x14
#define GSPI_TEST_VALUE      0xFEEDBEADU

#define GSPI_CTRL_WORD_32    0x01
#define GSPI_CTRL_ENDIAN_BIG 0x02
#define GSPI_CTRL_POL_HIGH   0x20
#define GSPI_CTRL_WAKE_UP    0x80
#define GSPI_RESP_DELAY_SHIFT 8  /* RESPONSE_DELAY register at byte 1 */
#define GSPI_STATUS_EN_SHIFT  16 /* STATUS_ENABLE register at byte 2 */
#define GSPI_INTR_WITH_STATUS 0x02
#define GSPI_RESP_DELAY_BYTES 4

#define GSPI_INT_DATA_UNAVAIL 0x0001
#define GSPI_INT_FIFO_RD_UF   0x0002
#define GSPI_INT_FIFO_WR_OF   0x0004
#define GSPI_INT_CMD_ERR      0x0008
#define GSPI_INT_DATA_ERR     0x0010
#define GSPI_INT_PKT_AVAIL    0x0020
#define GSPI_INT_F1_OVERFLOW  0x0080

#define GSPI_ST_F2_RX_READY   BIT(5)
#define GSPI_ST_UNDERFLOW     BIT(1)
#define GSPI_ST_PKT_AVAIL     BIT(8)
#define GSPI_ST_PKT_LEN(st)   (((st) >> 9) & 0x7FF)

#define GSPI_FEEDBEAD_TRIES   500
#define GSPI_F2_READY_LOOPS   1000
#define GSPI_STATUS_RETRIES   10

/* Power-up delays: hw.md 3.1 (taken from the WHD reference sequence). */
#define GSPI_REG_ON_LOW_MS  10
#define GSPI_REG_ON_HIGH_MS 250

struct infsingle_spi {
	struct infsingle_bus bus;
	struct spi_dt_spec spi;
	struct gpio_dt_spec reg_on;
	struct gpio_dt_spec bus_select;
	struct gpio_dt_spec host_wake;
	const struct pinctrl_dev_config *pcfg; /* non-NULL: data and IRQ share a pin */
	struct gpio_callback gpio_cb;
	infsingle_bus_irq_cb_t irq_cb;
	void *irq_user;
	bool has_irq;
	bool cb_added;
	bool armed;       /* interrupt currently unmasked */
	bool prev_active; /* IRQ level when the shared pin was last released */
	bool word32;      /* chip is in 32-bit word mode */
	uint32_t window;
	bool window_valid;
	size_t rx_pending; /* size reported by the last rx_len() */
	uint8_t cmd[4] __aligned(4);
	uint8_t stage[4 + GSPI_MAX_XFER + 4] __aligned(4);
};

/* One Wi-Fi module per board is supported (R-ENV). */
static struct infsingle_spi spi_inst;

static inline struct infsingle_spi *to_spi(struct infsingle_bus *bus)
{
	return CONTAINER_OF(bus, struct infsingle_spi, bus);
}

/* Swap the bytes inside each 16-bit half (pre-32-bit-mode wire format). */
static inline uint32_t swap_halves(uint32_t x)
{
	return ((x & 0x000000FFU) << 8) | ((x & 0x0000FF00U) >> 8) |
	       ((x & 0x00FF0000U) << 8) | ((x & 0xFF000000U) >> 8);
}

/* ------------------------------------------------------------------------- */
/* Interrupt line                                                            */
/* ------------------------------------------------------------------------- */

static void irq_notify(struct infsingle_spi *s)
{
	if (s->irq_cb != NULL) {
		s->irq_cb(&s->bus, s->irq_user);
	}
}

static void host_wake_isr(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
	struct infsingle_spi *s = CONTAINER_OF(cb, struct infsingle_spi, gpio_cb);

	ARG_UNUSED(port);
	ARG_UNUSED(pins);
	irq_notify(s);
}

/* Give the shared pin back to the SPI controller for the next transfer. */
static void irq_pause(struct infsingle_spi *s)
{
	if (s->pcfg == NULL || !s->armed) {
		return;
	}
	s->prev_active = gpio_pin_get_dt(&s->host_wake) > 0;
	(void)gpio_pin_interrupt_configure_dt(&s->host_wake, GPIO_INT_DISABLE);
	(void)pinctrl_apply_state(s->pcfg, PINCTRL_STATE_DEFAULT);
}

static void irq_resume(struct infsingle_spi *s)
{
	if (s->pcfg == NULL || !s->armed) {
		return;
	}
	(void)pinctrl_apply_state(s->pcfg, INFSINGLE_PINCTRL_STATE_HOST_WAKE);
	(void)gpio_pin_interrupt_configure_dt(&s->host_wake, GPIO_INT_EDGE_TO_ACTIVE);
	/*
	 * The line may have gone active while the pin was used for data, which
	 * produces no edge. Notify only if it was inactive before, assuming
	 * an earlier assertion has already woken the driver thread.
	 */
	if (gpio_pin_get_dt(&s->host_wake) > 0 && !s->prev_active) {
		irq_notify(s);
	}
}

/*
 * The ops member is called irq_mask because "irq_enable" is a Zephyr macro.
 * enable == true unmasks the host wake interrupt. The driver thread masks it
 * from the ISR and unmasks it after draining the device.
 */
static int bus_irq_mask(struct infsingle_bus *bus, bool enable)
{
	struct infsingle_spi *s = to_spi(bus);
	int ret = 0;

	if (!s->has_irq) {
		return 0;
	}
	if (enable) {
		if (s->pcfg != NULL) {
			ret = pinctrl_apply_state(s->pcfg, INFSINGLE_PINCTRL_STATE_HOST_WAKE);
		}
		if (ret == 0) {
			ret = gpio_pin_interrupt_configure_dt(&s->host_wake,
							      GPIO_INT_EDGE_TO_ACTIVE);
		}
		s->armed = (ret == 0);
		return ret;
	}
	s->armed = false;
	ret = gpio_pin_interrupt_configure_dt(&s->host_wake, GPIO_INT_DISABLE);
	if (s->pcfg != NULL) {
		(void)pinctrl_apply_state(s->pcfg, PINCTRL_STATE_DEFAULT);
	}
	return ret;
}

static int bus_irq_set(struct infsingle_bus *bus, infsingle_bus_irq_cb_t cb, void *user)
{
	struct infsingle_spi *s = to_spi(bus);

	s->irq_cb = cb;
	s->irq_user = user;
	return 0;
}

/* ------------------------------------------------------------------------- */
/* Raw transfers                                                             */
/* ------------------------------------------------------------------------- */

/*
 * One chip-select cycle: send @p tx_len bytes, then (if @p rx_len) receive
 * @p rx_len bytes. A half-duplex controller sends then turns the line around;
 * a full-duplex one discards what arrives while the command is sent.
 */
static int xfer(struct infsingle_spi *s, const uint8_t *tx, size_t tx_len, uint8_t *rx,
		size_t rx_len)
{
	const struct spi_buf tx_buf = {.buf = (void *)tx, .len = tx_len};
	const struct spi_buf_set tx_set = {.buffers = &tx_buf, .count = 1};
	struct spi_buf rx_buf[2];
	struct spi_buf_set rx_set = {.buffers = rx_buf, .count = 2};
	int ret;

	if (s->spi.config.operation & SPI_HALF_DUPLEX) {
		rx_buf[0].buf = rx;
		rx_buf[0].len = rx_len;
		rx_set.count = 1;
	} else {
		rx_buf[0].buf = NULL;
		rx_buf[0].len = rx_len ? tx_len : 0;
		rx_buf[1].buf = rx;
		rx_buf[1].len = rx_len;
	}

	irq_pause(s);
	ret = spi_transceive_dt(&s->spi, &tx_set, &rx_set);
	irq_resume(s);
	if (ret) {
		LOG_ERR("spi transfer failed: %d", ret);
	}
	return ret;
}

static uint32_t make_cmd(struct infsingle_spi *s, bool write, bool incr, uint8_t func,
			 uint32_t addr, size_t len)
{
	uint32_t c = ((uint32_t)write << 31) | ((uint32_t)incr << 30) |
		     ((uint32_t)(func & 3) << 28) | ((addr & 0x1FFFF) << 11) | (len & 0x7FF);

	return s->word32 ? c : swap_halves(c);
}

/* Read @p len bytes of a register/FIFO. @p cmd_len is the length in the command. */
static int gspi_read_to(struct infsingle_spi *s, uint8_t func, uint32_t addr, void *dst,
			size_t len, size_t cmd_len, size_t pad)
{
	size_t total = ROUND_UP(len + pad, 4);
	int ret;

	sys_put_le32(make_cmd(s, false, true, func, addr, cmd_len), s->cmd);
	if (pad == 0) {
		return xfer(s, s->cmd, 4, dst, total);
	}
	ret = xfer(s, s->cmd, 4, s->stage, total);
	if (ret == 0) {
		memcpy(dst, s->stage + pad, len);
	}
	return ret;
}

/* Register-sized read (1-4 bytes). F1 reads carry the response delay. */
static int gspi_reg_read(struct infsingle_spi *s, uint8_t func, uint32_t addr, size_t len,
			 uint32_t *val)
{
	uint8_t tmp[4] = {0};
	size_t pad = (func == GSPI_FUNC_BP) ? GSPI_BP_RD_PAD : 0;
	int ret;

	if (pad == 0) {
		/* Receive into a 4-byte-multiple scratch buffer. */
		ret = gspi_read_to(s, func, addr, s->stage, len, len, 0);
		if (ret == 0) {
			memcpy(tmp, s->stage, len);
		}
	} else {
		ret = gspi_read_to(s, func, addr, tmp, len, len + pad, pad);
	}
	if (ret == 0) {
		*val = sys_get_le32(tmp);
	}
	return ret;
}

static int gspi_write(struct infsingle_spi *s, uint8_t func, uint32_t addr, const void *src,
		      size_t len)
{
	size_t padded = ROUND_UP(len, 4);

	sys_put_le32(make_cmd(s, true, true, func, addr, len), s->stage);
	memcpy(s->stage + 4, src, len);
	memset(s->stage + 4 + len, 0, padded - len);
	return xfer(s, s->stage, 4 + padded, NULL, 0);
}

static int gspi_reg_write(struct infsingle_spi *s, uint8_t func, uint32_t addr, size_t len,
			  uint32_t val)
{
	uint8_t tmp[4];

	sys_put_le32(s->word32 ? val : swap_halves(val), tmp);
	return gspi_write(s, func, addr, tmp, len);
}

/* ------------------------------------------------------------------------- */
/* Start-up                                                                  */
/* ------------------------------------------------------------------------- */

static bool test_pattern_seen(const uint8_t *d, size_t n)
{
	/* The first read after reset may be shifted by a bit; match any variant. */
	const uint8_t pat[4] = {
		GSPI_TEST_VALUE & 0xFF,
		(GSPI_TEST_VALUE << 1) & 0xFF,
		((GSPI_TEST_VALUE << 1) + 1) & 0xFF,
		((GSPI_TEST_VALUE + 1) << 1) & 0xFF,
	};

	for (size_t i = 0; i < n; i++) {
		for (size_t j = 0; j < ARRAY_SIZE(pat); j++) {
			if (d[i] == pat[j]) {
				return true;
			}
		}
	}
	return false;
}

static int gspi_start(struct infsingle_spi *s)
{
	uint32_t v, ctrl;
	uint8_t rx[8];
	int ret = -ETIMEDOUT;

	s->word32 = false;
	s->window_valid = false;

	/*
	 * The first transfer after reset can be corrupted, so poll a register
	 * with a known value (fixed address mode) until it reads back.
	 */
	for (int i = 0; i < GSPI_FEEDBEAD_TRIES; i++) {
		sys_put_le32(make_cmd(s, false, false, GSPI_FUNC_BUS, GSPI_READ_TEST_REG, 4),
			     s->cmd);
		ret = xfer(s, s->cmd, 4, rx, sizeof(rx));
		if (ret == 0 && test_pattern_seen(rx, sizeof(rx))) {
			break;
		}
		ret = -ETIMEDOUT;
		k_msleep(1);
	}
	if (ret) {
		LOG_ERR("no response from the chip (test register never read back)");
		return ret;
	}

	/* 32-bit words, wake-up on, response delay 4, interrupt with status. */
	ctrl = GSPI_CTRL_WORD_32 | GSPI_CTRL_WAKE_UP |
	       (IS_ENABLED(CONFIG_BIG_ENDIAN) ? 0 : GSPI_CTRL_ENDIAN_BIG) |
	       ((s->host_wake.dt_flags & GPIO_ACTIVE_LOW) ? 0 : GSPI_CTRL_POL_HIGH) |
	       (GSPI_RESP_DELAY_BYTES << GSPI_RESP_DELAY_SHIFT) |
	       (GSPI_INTR_WITH_STATUS << GSPI_STATUS_EN_SHIFT);
	ret = gspi_reg_write(s, GSPI_FUNC_BUS, GSPI_BUS_CONTROL, 4, ctrl);
	if (ret) {
		return ret;
	}
	s->word32 = true;

	ret = gspi_reg_read(s, GSPI_FUNC_BUS, GSPI_READ_TEST_REG, 4, &v);
	if (ret) {
		return ret;
	}
	if (v != GSPI_TEST_VALUE) {
		LOG_ERR("test register read 0x%08x, expected 0x%08x", v, GSPI_TEST_VALUE);
		return -EIO;
	}

	/* Clear stale error bits, then enable the interrupts we use. */
	ret = gspi_reg_write(s, GSPI_FUNC_BUS, GSPI_INT_REG, 1,
			     GSPI_INT_DATA_UNAVAIL | GSPI_INT_CMD_ERR | GSPI_INT_DATA_ERR |
			     GSPI_INT_F1_OVERFLOW);
	if (ret == 0) {
		ret = gspi_reg_write(s, GSPI_FUNC_BUS, GSPI_INT_ENABLE, 2,
				     GSPI_INT_FIFO_RD_UF | GSPI_INT_FIFO_WR_OF |
				     GSPI_INT_CMD_ERR | GSPI_INT_DATA_ERR |
				     GSPI_INT_PKT_AVAIL | GSPI_INT_F1_OVERFLOW);
	}
	return ret;
}

static int bus_init(struct infsingle_bus *bus)
{
	struct infsingle_spi *s = to_spi(bus);
	int ret;

	if (!spi_is_ready_dt(&s->spi) || !gpio_is_ready_dt(&s->reg_on)) {
		return -ENODEV;
	}
	/* The bus-select pin must be low when WL_REG_ON rises to pick SPI. */
	ret = gpio_pin_configure_dt(&s->bus_select, GPIO_OUTPUT_INACTIVE);
	if (ret) {
		return ret;
	}
	ret = gpio_pin_configure_dt(&s->reg_on, GPIO_OUTPUT_INACTIVE);
	if (ret) {
		return ret;
	}
	k_msleep(GSPI_REG_ON_LOW_MS);
	gpio_pin_set_dt(&s->reg_on, 1);
	k_msleep(GSPI_REG_ON_HIGH_MS);

	if (s->has_irq) {
		if (!gpio_is_ready_dt(&s->host_wake)) {
			return -ENODEV;
		}
		if (s->pcfg != NULL) {
			ret = pinctrl_apply_state(s->pcfg, PINCTRL_STATE_DEFAULT);
			if (ret) {
				return ret;
			}
		}
		ret = gpio_pin_configure_dt(&s->host_wake, GPIO_INPUT);
		if (ret) {
			return ret;
		}
		if (!s->cb_added) {
			gpio_init_callback(&s->gpio_cb, host_wake_isr, BIT(s->host_wake.pin));
			ret = gpio_add_callback_dt(&s->host_wake, &s->gpio_cb);
			if (ret) {
				return ret;
			}
			s->cb_added = true;
		}
		if (s->pcfg != NULL) {
			/* The data line needs the SPI function during start-up. */
			ret = pinctrl_apply_state(s->pcfg, PINCTRL_STATE_DEFAULT);
			if (ret) {
				return ret;
			}
		}
	}
	s->armed = false;
	ret = gspi_start(s);
	if (ret) {
		LOG_ERR("gSPI start-up failed: %d", ret);
	}
	return ret;
}

static int bus_deinit(struct infsingle_bus *bus)
{
	struct infsingle_spi *s = to_spi(bus);

	(void)bus_irq_mask(bus, false);
	return gpio_pin_set_dt(&s->reg_on, 0);
}

/* ------------------------------------------------------------------------- */
/* Register and backplane access                                             */
/* ------------------------------------------------------------------------- */

static int bus_reg_read8(struct infsingle_bus *bus, uint8_t func, uint32_t addr, uint8_t *val)
{
	uint32_t v = 0;
	int ret = gspi_reg_read(to_spi(bus), func, addr, 1, &v);

	*val = v & 0xFF;
	return ret;
}

static int bus_reg_write8(struct infsingle_bus *bus, uint8_t func, uint32_t addr, uint8_t val)
{
	return gspi_reg_write(to_spi(bus), func, addr, 1, val);
}

/*
 * Select the 32 KB window containing @p addr; same scheme as the SDIO
 * transport, only bytes that changed are rewritten.
 */
static int bp_set_window(struct infsingle_spi *s, uint32_t addr)
{
	uint32_t base = addr & ~(uint32_t)INFSINGLE_SB_OFFSET_MASK;
	const uint8_t val[3] = {(base >> 8) & 0x80, (base >> 16) & 0xFF, (base >> 24) & 0xFF};
	int ret = 0;

	if (s->window_valid && s->window == base) {
		return 0;
	}
	for (int i = 0; i < 3 && ret == 0; i++) {
		uint8_t old[3] = {(s->window >> 8) & 0x80, (s->window >> 16) & 0xFF,
				  (s->window >> 24) & 0xFF};

		if (!s->window_valid || val[i] != old[i]) {
			ret = gspi_reg_write(s, GSPI_FUNC_BP, INFSINGLE_SB_ADDR_LOW + i, 1,
					     val[i]);
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
	struct infsingle_spi *s = to_spi(bus);
	int ret = bp_set_window(s, addr);

	if (ret) {
		return ret;
	}
	return gspi_reg_read(s, GSPI_FUNC_BP,
			     (addr & INFSINGLE_SB_OFFSET_MASK) | INFSINGLE_SB_ACCESS_32BIT, 4, val);
}

static int bus_bp_write32(struct infsingle_bus *bus, uint32_t addr, uint32_t val)
{
	struct infsingle_spi *s = to_spi(bus);
	int ret = bp_set_window(s, addr);

	if (ret) {
		return ret;
	}
	return gspi_reg_write(s, GSPI_FUNC_BP,
			      (addr & INFSINGLE_SB_OFFSET_MASK) | INFSINGLE_SB_ACCESS_32BIT, 4, val);
}

/* Largest piece that stays inside the current window and the F1 limit. */
static size_t bp_piece(uint32_t addr, size_t left)
{
	size_t to_edge = INFSINGLE_SB_WINDOW_SIZE - (addr & INFSINGLE_SB_OFFSET_MASK);

	return MIN(MIN(left, to_edge), (size_t)GSPI_BP_CHUNK);
}

static int bus_bp_write(struct infsingle_bus *bus, uint32_t addr, const void *buf, size_t len)
{
	struct infsingle_spi *s = to_spi(bus);
	const uint8_t *p = buf;

	while (len) {
		size_t n = bp_piece(addr, len);
		int ret = bp_set_window(s, addr);

		if (ret == 0) {
			ret = gspi_write(s, GSPI_FUNC_BP, addr & INFSINGLE_SB_OFFSET_MASK, p, n);
		}
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
	struct infsingle_spi *s = to_spi(bus);
	uint8_t *p = buf;

	while (len) {
		size_t n = bp_piece(addr, len);
		int ret = bp_set_window(s, addr);

		if (ret == 0) {
			ret = gspi_read_to(s, GSPI_FUNC_BP, addr & INFSINGLE_SB_OFFSET_MASK, p, n,
					   n + GSPI_BP_RD_PAD, GSPI_BP_RD_PAD);
		}
		if (ret) {
			return ret;
		}
		addr += n;
		p += n;
		len -= n;
	}
	return 0;
}

/* ------------------------------------------------------------------------- */
/* F2 frames                                                                 */
/* ------------------------------------------------------------------------- */

static int read_status(struct infsingle_spi *s, uint32_t *st)
{
	int ret;

	/* 0xFFFFFFFF means the chip did not answer yet; try again. */
	for (int i = 0; i < GSPI_STATUS_RETRIES; i++) {
		ret = gspi_reg_read(s, GSPI_FUNC_BUS, GSPI_STATUS_REG, 4, st);
		if (ret || *st != 0xFFFFFFFFU) {
			return ret;
		}
	}
	return -EIO;
}

static int bus_rx_len(struct infsingle_bus *bus, size_t *len)
{
	struct infsingle_spi *s = to_spi(bus);
	uint32_t st, intr;
	int ret;

	*len = 0;
	s->rx_pending = 0;
	for (int pass = 0; pass < 2; pass++) {
		ret = read_status(s, &st);
		if (ret) {
			return ret;
		}
		if (st & GSPI_ST_PKT_AVAIL) {
			size_t n = GSPI_ST_PKT_LEN(st);

			if (n == 0 || (st & GSPI_ST_UNDERFLOW)) {
				/* Inconsistent: make the device drop whatever it holds. */
				(void)gspi_reg_write(s, GSPI_FUNC_BP, INFSINGLE_F1_FRAME_CONTROL,
						     1, 1);
				return 0;
			}
			s->rx_pending = n;
			*len = n;
			return 0;
		}
		if (pass == 1) {
			break;
		}
		/*
		 * Idle: acknowledge the interrupt so the line drops, then look
		 * once more in case a frame arrived while it was being cleared.
		 */
		ret = gspi_reg_read(s, GSPI_FUNC_BUS, GSPI_INT_REG, 2, &intr);
		if (ret) {
			return ret;
		}
		if (intr == 0) {
			break;
		}
		ret = gspi_reg_write(s, GSPI_FUNC_BUS, GSPI_INT_REG, 2, intr);
		if (ret) {
			return ret;
		}
	}
	return 0;
}

static int bus_wait_f2_ready(struct infsingle_bus *bus, uint32_t timeout_ms)
{
	struct infsingle_spi *s = to_spi(bus);
	int64_t end = k_uptime_get() + timeout_ms;
	uint32_t st = 0;

	do {
		if (gspi_reg_read(s, GSPI_FUNC_BUS, GSPI_STATUS_REG, 4, &st) == 0 &&
		    st != 0xFFFFFFFFU && (st & GSPI_ST_F2_RX_READY)) {
			return 0;
		}
		k_msleep(1);
	} while (k_uptime_get() < end);
	return -ETIMEDOUT;
}

static int bus_frame_read(struct infsingle_bus *bus, void *buf, size_t len)
{
	struct infsingle_spi *s = to_spi(bus);
	size_t cmd_len = (s->rx_pending != 0 && s->rx_pending <= len) ? s->rx_pending : len;

	/* The caller's buffer receives the data directly (no staging). */
	return gspi_read_to(s, GSPI_FUNC_WLAN, 0, buf, len, cmd_len, 0);
}

static int bus_frame_write(struct infsingle_bus *bus, const void *buf, size_t len)
{
	struct infsingle_spi *s = to_spi(bus);
	size_t size = sys_get_le16(buf); /* SDPCM frame length tag */
	uint32_t st = 0;
	int ret = 0;

	if (size == 0 || size > len || len > GSPI_MAX_XFER) {
		return -EINVAL;
	}
	/* Wait for room in the F2 FIFO. */
	for (int i = 0; i < GSPI_F2_READY_LOOPS; i++) {
		ret = gspi_reg_read(s, GSPI_FUNC_BUS, GSPI_STATUS_REG, 4, &st);
		if (ret || (st != 0xFFFFFFFFU && (st & GSPI_ST_F2_RX_READY))) {
			break;
		}
		ret = -ETIMEDOUT;
	}
	if (ret) {
		return ret;
	}
	return gspi_write(s, GSPI_FUNC_WLAN, 0, buf, size);
}

static int bus_abort_read(struct infsingle_bus *bus)
{
	/* FRAME_CONTROL bit 0 makes the device discard the rest of the frame. */
	return gspi_reg_write(to_spi(bus), GSPI_FUNC_BP, INFSINGLE_F1_FRAME_CONTROL, 1, 1);
}

static const struct infsingle_bus_ops spi_ops = {
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
	.rx_len = bus_rx_len,
	.wait_f2_ready = bus_wait_f2_ready,
	.irq_set = bus_irq_set,
	.irq_mask = bus_irq_mask,
};

struct infsingle_bus *infsingle_bus_spi_create(const struct spi_dt_spec *spi,
				    const struct gpio_dt_spec *reg_on,
				    const struct gpio_dt_spec *bus_select,
				    const struct gpio_dt_spec *host_wake,
				    const struct pinctrl_dev_config *pcfg)
{
	struct infsingle_spi *s = &spi_inst;

	memset(s, 0, sizeof(*s));
	s->spi = *spi;
	s->reg_on = *reg_on;
	s->bus_select = *bus_select;
	if (host_wake != NULL) {
		s->host_wake = *host_wake;
		s->has_irq = true;
	}
	s->pcfg = pcfg;
	s->bus.ops = &spi_ops;
	s->bus.align = 4;
	s->bus.block = 4;
	s->bus.spi = true;
	return &s->bus;
}
