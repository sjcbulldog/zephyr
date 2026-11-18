/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file infsingle_wire.h
 * @brief On-the-wire structures and hardware constants (protocol layer).
 *
 * Layer: protocol. Reference: infdocs/hw.md sections 4 (SDIO), 7 (SDPCM),
 * 8 (CDC/BDC), 11 (events), 12 (BLHS). Every constant records where it was
 * established. All multi-byte fields are little-endian unless stated; the
 * Cortex-M hosts we support are little-endian so structures are used directly.
 *
 * Frame layout on function 2 (one SDIO CMD53 transfer per frame):
 *
 *   0       2       4   5     6     7      8     9      10    12
 *   +-------+-------+---+-----+-----+------+-----+------+-----+
 *   | size  |~size  |seq|chan |next |hdrlen|flow |credit| rsv |   SDPCM (12)
 *   +-------+-------+---+-----+-----+------+-----+------+-----+
 *   | channel 0: CDC header (16) + payload                     |
 *   | channel 1/2: pad(2) | BDC header (4) | Ethernet frame    |
 *   +----------------------------------------------------------+
 */

#ifndef ZEPHYR_DRIVERS_WIFI_INFSINGLE_INFSINGLE_WIRE_H_
#define ZEPHYR_DRIVERS_WIFI_INFSINGLE_INFSINGLE_WIRE_H_

#include <stdint.h>
#include <zephyr/sys/util.h>
#include <zephyr/toolchain.h>

/* ---- SDIO functions and block size (hw.md section 4) -------------------- */
#define INFSINGLE_SDIO_FUNC_BUS       0   /* F0: CCCR registers                     */
#define INFSINGLE_SDIO_FUNC_BACKPLANE 1   /* F1: backplane / chip registers         */
#define INFSINGLE_SDIO_FUNC_WLAN      2   /* F2: WLAN packet FIFO                   */
#define INFSINGLE_SDIO_BLOCK_SIZE     64  /* Block size for F1 and F2               */
#define INFSINGLE_SDIO_MAX_BLOCKS     511 /* CMD53 block count field is 9 bits      */

/* ---- CCCR registers (F0) ------------------------------------------------ */
#define INFSINGLE_CCCR_IOEN           0x02
#define INFSINGLE_CCCR_INTEN          0x04
#define INFSINGLE_CCCR_INTPEND        0x05
#define INFSINGLE_CCCR_IOABORT        0x06
#define INFSINGLE_CCCR_BICTRL         0x07
#define INFSINGLE_CCCR_CARDCAP        0xF0
#define INFSINGLE_CARDCAP_CHIPID_PRESENT 0x40 /* chip ID readable from SDIO core    */
#define INFSINGLE_CARDCAP_SECURE_MODE    0x80 /* bootloader security (BLHS) present */

/* ---- Backplane window (F1, hw.md section 4.3) --------------------------- */
#define INFSINGLE_SB_WINDOW_SIZE      0x8000     /* 32 KB addressable per window   */
#define INFSINGLE_SB_OFFSET_MASK      0x7FFF
#define INFSINGLE_SB_ACCESS_32BIT     0x8000     /* OR into offset: 4-byte access  */
#define INFSINGLE_SB_ADDR_LOW        0x1000A
#define INFSINGLE_SB_ADDR_MID        0x1000B
#define INFSINGLE_SB_ADDR_HIGH       0x1000C

/* ---- F1 direct registers (+0x10000) ------------------------------------- */
#define INFSINGLE_F1_FUNC2_WATERMARK  0x10008
#define INFSINGLE_F1_DEVICE_CONTROL   0x10009
#define INFSINGLE_F1_FRAME_CONTROL    0x1000D
#define INFSINGLE_F1_CHIP_CLOCK_CSR   0x1000E
#define INFSINGLE_F1_WAKEUP_CTRL      0x1001E
#define INFSINGLE_F1_SLEEP_CSR        0x1001F
#define INFSINGLE_F1_BLHS_H2D         0x10030
#define INFSINGLE_F1_BLHS_D2H         0x10038

#define INFSINGLE_FUNC2_WATERMARK_VAL 8
#define INFSINGLE_FUNC2_WATERMARK_SPI 32

/* CHIP_CLOCK_CSR bits */
#define INFSINGLE_CLK_FORCE_ALP       0x01
#define INFSINGLE_CLK_FORCE_HT        0x02
#define INFSINGLE_CLK_ALP_AVAIL_REQ   0x08
#define INFSINGLE_CLK_HT_AVAIL_REQ    0x10
#define INFSINGLE_CLK_FORCE_HW_CLKREQ_OFF 0x20
#define INFSINGLE_CLK_ALP_AVAIL       0x40
#define INFSINGLE_CLK_HT_AVAIL        0x80

/* SLEEP_CSR (KSO) */
#define INFSINGLE_KSO_KEEP_WL         0x01
#define INFSINGLE_KSO_ON_MASK         0x03 /* both KEEP and device-ack bits set    */

/* ---- SDIO core registers (offset from per-chip SDIO core base) ---------- */
#define INFSINGLE_SDIOCORE_INT_STATUS      0x20
#define INFSINGLE_SDIOCORE_INT_HOST_MASK   0x24
#define INFSINGLE_SDIOCORE_FUNC_INT_MASK   0x34
#define INFSINGLE_SDIOCORE_TO_SB_MAILBOX   0x40
#define INFSINGLE_SDIOCORE_TO_SB_MBX_DATA  0x48
#define INFSINGLE_SDIOCORE_TO_HOST_MBX_DATA 0x4C
#define INFSINGLE_SDIOCORE_CHIPID          0x330 /* valid if CARDCAP_CHIPID_PRESENT */

#define INFSINGLE_INT_HOST_MASK_VALUE      0xF0  /* frame-available bits           */
#define INFSINGLE_INT_FRAME_AVAILABLE_MASK 0xF0
#define INFSINGLE_INT_HMB_HOST_INT         0x80  /* mailbox event                  */
#define INFSINGLE_HMB_DATA_FWHALT          0x10  /* firmware crashed               */
#define INFSINGLE_SMB_INT_ACK              0x02

#define INFSINGLE_CHIPCOMMON_BASE          0x18000000

/* ---- Bootloader handshake (BLHS, hw.md section 12) ---------------------- */
#define INFSINGLE_BLHS_H2D_BL_INIT         0
#define INFSINGLE_BLHS_H2D_DL_FW_START     BIT(0)
#define INFSINGLE_BLHS_H2D_DL_FW_DONE      BIT(1)
#define INFSINGLE_BLHS_H2D_DL_NVRAM_DONE   BIT(2)
#define INFSINGLE_BLHS_H2D_RESET_ON_ERROR  BIT(3)
#define INFSINGLE_BLHS_D2H_START           BIT(0)
#define INFSINGLE_BLHS_D2H_READY           BIT(1)
#define INFSINGLE_BLHS_D2H_STEADY          BIT(2)
#define INFSINGLE_BLHS_D2H_TRXHDR_PARSE    BIT(3)
#define INFSINGLE_BLHS_D2H_VALDN_START     BIT(4)
#define INFSINGLE_BLHS_D2H_VALDN_RESULT    BIT(5)
#define INFSINGLE_BLHS_D2H_VALDN_DONE      BIT(6)
#define INFSINGLE_BLHS_D2H_NVRAM_MV_DONE   BIT(7)
#define INFSINGLE_BLHS_POLL_MS             10
#define INFSINGLE_BLHS_TIMEOUT_MS          1000
#define INFSINGLE_HT_AVAIL_TIMEOUT_MS      2500

/* ---- TRX firmware container (hw.md section 6) --------------------------- */
#define INFSINGLE_TRX_MAGIC                0x30524448u /* "HDR0" */
#define INFSINGLE_TRXV4_SIZE               692
#define INFSINGLE_TRXV5_SIZE               32

struct infsingle_trx_hdr {
	uint32_t magic;
	uint32_t len;          /* image length after the header, bytes          */
	uint32_t crc32;
	uint32_t flag_version;
	uint32_t offsets[4];
} __packed;

/* ---- SDPCM (hw.md section 7) -------------------------------------------- */
#define INFSINGLE_SDPCM_HDR_LEN        12
#define INFSINGLE_SDPCM_CHAN_CONTROL   0
#define INFSINGLE_SDPCM_CHAN_EVENT     1
#define INFSINGLE_SDPCM_CHAN_DATA      2
#define INFSINGLE_SDPCM_CHAN_MASK      0x0F
#define INFSINGLE_SDPCM_DATA_PAD       2  /* data frames: header_length = 12 + 2  */

struct infsingle_sdpcm_hdr {
	uint16_t size;       /* total frame length including this header        */
	uint16_t size_not;   /* ~size: hardware tag, verified on receive        */
	uint8_t  seq;
	uint8_t  chan_flags; /* low nibble = channel                            */
	uint8_t  next_len;
	uint8_t  hdr_len;    /* offset of payload from frame start              */
	uint8_t  flow_ctl;
	uint8_t  credit;     /* highest tx sequence the firmware will accept    */
	uint8_t  reserved[2];
} __packed;
BUILD_ASSERT(sizeof(struct infsingle_sdpcm_hdr) == INFSINGLE_SDPCM_HDR_LEN);

/* ---- CDC control header (channel 0) ------------------------------------- */
#define INFSINGLE_CDC_FLAG_ERROR       0x00000001u
#define INFSINGLE_CDC_FLAG_SET         0x00000002u
#define INFSINGLE_CDC_ID_SHIFT         16
#define INFSINGLE_CDC_IF_SHIFT         12

struct infsingle_cdc_hdr {
	uint32_t cmd;
	uint32_t len;        /* length of the data that follows                 */
	uint32_t flags;      /* ERROR | SET | ifidx<<12 | id<<16                */
	uint32_t status;
} __packed;
BUILD_ASSERT(sizeof(struct infsingle_cdc_hdr) == 16);

/* ---- BDC data header (channels 1 and 2) --------------------------------- */
#define INFSINGLE_BDC_VERSION          2
#define INFSINGLE_BDC_FLAGS_VER        (INFSINGLE_BDC_VERSION << 4)

struct infsingle_bdc_hdr {
	uint8_t flags;
	uint8_t priority;
	uint8_t flags2;      /* low nibble: interface index                     */
	uint8_t data_offset; /* extra header length in 32-bit words             */
} __packed;
BUILD_ASSERT(sizeof(struct infsingle_bdc_hdr) == 4);

/* Headroom the driver needs in front of an Ethernet frame for TX. */
#define INFSINGLE_TX_HEADROOM \
	(INFSINGLE_SDPCM_HDR_LEN + INFSINGLE_SDPCM_DATA_PAD + sizeof(struct infsingle_bdc_hdr))

/* ---- Event frames (hw.md section 11) ------------------------------------ */
#define INFSINGLE_ETHERTYPE_BRCM       0x886C

struct infsingle_bcm_ether_hdr {   /* 10 bytes after the Ethernet header */
	uint16_t subtype;
	uint16_t length;
	uint8_t  version;
	uint8_t  oui[3];
	uint16_t usr_subtype;
} __packed;
BUILD_ASSERT(sizeof(struct infsingle_bcm_ether_hdr) == 10);

/* All fields big-endian on the wire. */
struct infsingle_event_msg {
	uint16_t version;
	uint16_t flags;
	uint32_t event_type;
	uint32_t status;
	uint32_t reason;
	uint32_t auth_type;
	uint32_t datalen;
	uint8_t  addr[6];
	char     ifname[16];
	uint8_t  ifidx;
	uint8_t  bsscfgidx;
} __packed;
BUILD_ASSERT(sizeof(struct infsingle_event_msg) == 48);

#define INFSINGLE_EVT_SET_SSID         0
#define INFSINGLE_EVT_AUTH             3
#define INFSINGLE_EVT_DEAUTH           5
#define INFSINGLE_EVT_DEAUTH_IND       6
#define INFSINGLE_EVT_ASSOC            7
#define INFSINGLE_EVT_ASSOC_IND        8
#define INFSINGLE_EVT_REASSOC          9
#define INFSINGLE_EVT_REASSOC_IND      10
#define INFSINGLE_EVT_DISASSOC         11
#define INFSINGLE_EVT_DISASSOC_IND     12
#define INFSINGLE_EVT_LINK             16
#define INFSINGLE_EVT_PSK_SUP          46
#define INFSINGLE_EVT_ESCAN_RESULT     69
#define INFSINGLE_EVT_EXT_AUTH_REQ     187
#define INFSINGLE_EVT_EXT_AUTH_FRAME_RX 188
#define INFSINGLE_EVT_ICMP_ECHO_REQ    202
#define INFSINGLE_EVT_LEGACY_MAX       192
#define INFSINGLE_EVT_MAX              208 /* size of the event_msgs bitmask       */

#define INFSINGLE_EVT_STATUS_SUCCESS   0
#define INFSINGLE_EVT_STATUS_FAIL      1
#define INFSINGLE_EVT_STATUS_TIMEOUT   2
#define INFSINGLE_EVT_STATUS_NO_NETWORKS 3
#define INFSINGLE_EVT_STATUS_ABORT     4
#define INFSINGLE_EVT_STATUS_NO_ACK    5
#define INFSINGLE_EVT_STATUS_PARTIAL   8
#define INFSINGLE_EVT_PSK_SUP_BASE     256
#define INFSINGLE_EVT_PSK_SUP_KEYED    (INFSINGLE_EVT_PSK_SUP_BASE + 6)

#define INFSINGLE_EVT_FLAG_LINK_UP     0x0001

/* ---- AP mode ------------------------------------------------------------ */
#define INFSINGLE_BSS_DOWN             0
#define INFSINGLE_BSS_UP               1
/* 20 MHz chanspec (Wi-Fi 5 and Wi-Fi 6 parts): channel | band | bandwidth */
#define INFSINGLE_CHSPEC_BAND_2G       0x0000
#define INFSINGLE_CHSPEC_BAND_5G       0xC000
#define INFSINGLE_CHSPEC_BW_20         0x1000
/* Multicast rate in 500 kbit/s units: 11 Mbit/s instead of the 1 Mbit/s default */
#define INFSINGLE_MCAST_RATE_11MBPS    22

/* ---- Ioctl command numbers (hw.md section 8) ---------------------------- */
#define INFSINGLE_WLC_UP               2
#define INFSINGLE_WLC_DOWN             3
#define INFSINGLE_WLC_SET_INFRA        20
#define INFSINGLE_WLC_SET_AUTH         22
#define INFSINGLE_WLC_GET_BSSID        23
#define INFSINGLE_WLC_SET_SSID         26
#define INFSINGLE_WLC_SET_CHANNEL      30
#define INFSINGLE_WLC_DISASSOC         52
#define INFSINGLE_WLC_SET_COUNTRY      84
#define INFSINGLE_WLC_GET_MAGIC        0
#define INFSINGLE_WLC_SET_PM           86
#define INFSINGLE_WLC_SET_BAND         142 /* 0 auto, 1 5 GHz only, 2 2.4 GHz only */
#define INFSINGLE_WLC_SET_GMODE        110
#define INFSINGLE_WLC_SET_AP           118
#define INFSINGLE_WLC_GET_RSSI         127
#define INFSINGLE_WLC_SET_WSEC         134
#define INFSINGLE_WLC_SET_WPA_AUTH     165
#define INFSINGLE_WLC_DEAUTH_FOR_REASON 201
#define INFSINGLE_WLC_GET_VAR          262
#define INFSINGLE_WLC_SET_VAR          263
#define INFSINGLE_WLC_SET_WSEC_PMK     268

/* ---- Security constants ------------------------------------------------- */
#define INFSINGLE_WSEC_NONE            0x0000
#define INFSINGLE_WSEC_TKIP            0x0002
#define INFSINGLE_WSEC_AES             0x0004
#define INFSINGLE_WPA_AUTH_DISABLED    0x0000
#define INFSINGLE_WPA_AUTH_PSK         0x0004
#define INFSINGLE_WPA2_AUTH_PSK        0x0080
#define INFSINGLE_WPA3_AUTH_SAE_PSK    0x40000
#define INFSINGLE_WPA3_AUTH_OWE        0x100000
#define INFSINGLE_AUTH_OPEN_SYSTEM     0
#define INFSINGLE_AUTH_SAE             3
#define INFSINGLE_MFP_NONE             0
#define INFSINGLE_MFP_CAPABLE          1
#define INFSINGLE_MFP_REQUIRED         2
#define INFSINGLE_PSK_MIN_LEN          8
#define INFSINGLE_PSK_MAX_LEN          64
#define INFSINGLE_SAE_MAX_PW_LEN       128
#define INFSINGLE_WSEC_PASSPHRASE      1

struct infsingle_wsec_pmk {
	uint16_t key_len;
	uint16_t flags;      /* INFSINGLE_WSEC_PASSPHRASE                             */
	uint8_t  key[65];
} __packed;

struct infsingle_ssid {
	uint32_t len;
	uint8_t  ssid[32];
} __packed;

/* ---- Scan (escan) ------------------------------------------------------- */
#define INFSINGLE_ESCAN_REQ_VERSION    1
#define INFSINGLE_ESCAN_ACTION_START   1

struct infsingle_scan_params {
	struct infsingle_ssid ssid;
	uint8_t  bssid[6];
	int8_t   bss_type;
	int8_t   scan_type;
	int32_t  nprobes;
	int32_t  active_time;
	int32_t  passive_time;
	int32_t  home_time;
	int32_t  channel_num;
	uint16_t channel_list[];
} __packed;

struct infsingle_escan_params {
	uint32_t version;
	uint16_t action;
	uint16_t sync_id;
	struct infsingle_scan_params params;
} __packed;

struct infsingle_escan_result {
	uint32_t buflen;
	uint32_t version;
	uint16_t sync_id;
	uint16_t bss_count;
	/* struct infsingle_bss_info follows */
} __packed;

/* Offsets within wl_bss_info_t (hw.md section 10.2); read byte-wise. */
#define INFSINGLE_BSS_OFF_LENGTH       4
#define INFSINGLE_BSS_OFF_BSSID        8
#define INFSINGLE_BSS_OFF_CAPABILITY   16
#define INFSINGLE_BSS_OFF_SSID_LEN     18
#define INFSINGLE_BSS_OFF_SSID         19
#define INFSINGLE_BSS_OFF_CHANSPEC     72
#define INFSINGLE_BSS_OFF_RSSI         78
#define INFSINGLE_BSS_OFF_IE_OFFSET    116
#define INFSINGLE_BSS_OFF_IE_LENGTH    120
#define INFSINGLE_BSS_MIN_LEN          128

/* ---- CLM download (hw.md section 9.3) ----------------------------------- */
#define INFSINGLE_DLOAD_CHUNK          1024
#define INFSINGLE_DLOAD_FLAG_VER       (1 << 12)
#define INFSINGLE_DLOAD_FLAG_BEGIN     0x0002
#define INFSINGLE_DLOAD_FLAG_END       0x0004
#define INFSINGLE_DLOAD_TYPE_CLM       2

struct infsingle_dload_hdr {
	uint16_t flag;
	uint16_t dload_type;
	uint32_t len;
	uint32_t crc;
} __packed;

#endif /* ZEPHYR_DRIVERS_WIFI_INFSINGLE_INFSINGLE_WIRE_H_ */
