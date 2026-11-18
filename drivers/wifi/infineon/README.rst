Infineon AIROC Wi-Fi driver (native, SDIO and gSPI, Wi-Fi 5 and Wi-Fi 6)
========================================================================

Native Zephyr driver for Infineon AIROC Wi-Fi devices. It does not use the WHD
library. Source files keep the ``infsingle_`` prefix.

See ``USERGUIDE.md`` for devicetree integration and the separate firmware
offload extension API.

Goal
----

The driver lets Zephyr use an Infineon AIROC Wi-Fi chip (CYW43xxx, CYW55xxx) as a normal
Wi-Fi network interface. The chip is an external device. It runs its own firmware, which
implements the 802.11 MAC and PHY, scanning, association, WPA/WPA2/WPA3 handshakes and
power control. The host never touches radio hardware. The host only:

* powers the chip and loads its firmware, NVRAM (board calibration text) and CLM
  (regulatory data) over the bus,
* sends commands to the firmware and receives its answers and notifications,
* exchanges Ethernet frames with the firmware.

Everything between the host and the chip is carried in **packets**. The same packet format
is used on both buses; only the way a packet is moved differs:

* **SDIO.** The chip has three SDIO functions. Function 0 holds the standard CCCR
  registers. Function 1 gives access to the chip's internal registers (the *backplane*)
  through a 32 KB movable window. Function 2 is the WLAN packet FIFO. Each packet is one
  CMD53 transfer on function 2.
* **gSPI.** One SPI chip select carries the same three functions. Every transfer starts
  with a 32-bit command word (read or write, function, address, length). Packets are
  moved with the function 2 command. The packet length is sent to the chip in the command,
  so on receive the host first asks the chip for the length of the next packet.

The driver has no WHD dependency. ``infsingle_bus.h`` hides the bus; everything above it
deals only with packets.

Station connections honor a non-zero BSSID in ``wifi_connect_req_params``,
allowing selection of a particular AP when multiple radios share an SSID.
A zero BSSID preserves automatic selection. The band hint restricts the
firmware scan; station channel hints are not implemented.

Packet format
-------------

Every packet on function 2 begins with a 12-byte **SDPCM header** (``struct
infsingle_sdpcm_hdr``). All multi-byte fields are little-endian, except the event message
described later, which is big-endian.

.. code-block:: none

   byte  0       2       4    5     6     7      8     9      10    12
         +-------+-------+----+-----+-----+------+-----+------+-----+
         | size  |~size  |seq |chan |next |hdrlen|flow |credit| rsv |   SDPCM header
         +-------+-------+----+-----+-----+------+-----+------+-----+
         | payload (depends on channel), starts at offset hdrlen    |
         +----------------------------------------------------------+

======== ===========================================================================
Field    Meaning
======== ===========================================================================
size     Total packet length, including this header.
~size    Bitwise inverse of ``size``. The receiver checks it to reject a bad or idle
         tag (``infsingle_sdpcm_check_tag``).
seq      Sender's sequence number for this packet (8-bit, wraps).
chan     Low nibble is the channel: 0 control, 1 event, 2 data.
next     Length of a following packet when packets are chained (unused by this driver).
hdrlen   Offset of the payload from the start of the packet.
flow     Flow-control flags from the firmware.
credit   Sent by the firmware: the highest sequence number the host may transmit.
======== ===========================================================================

Padding and alignment: the transfer length is rounded up to the bus granularity (``block`` in
``struct infsingle_bus``: 4 bytes on gSPI; the SDIO transport sets its own). On SDIO a
transfer longer than one 64-byte block is rounded up to whole blocks (``big_block``), as
WHD does, so each packet is a single block-mode CMD53 instead of a block command plus a
byte-mode remainder. The padding is not counted in ``size``. It lives in the caller's tail
room, so no bounce buffer is needed.

Flow control (credits)
~~~~~~~~~~~~~~~~~~~~~~

The firmware has a small receive ring. It tells the host how many packets it may send by
putting the highest acceptable sequence number in ``credit`` of every packet it sends. The
host keeps ``tx_seq`` (next sequence to use) and ``tx_max`` (last ``credit`` seen) and may
send while ``(tx_max - tx_seq) & 0xFF`` is non-zero and below 0x80. Each packet sent
consumes one credit. Credit arrives inside any received packet, including packets whose
only purpose is to carry a new credit (credit-only packets, which have no payload).

Data packets never use the last two credits (``INFSINGLE_TXCTL_CREDITS``, as brcmfmac
does), so control requests still get through while data is starved. A non-zero ``flow``
byte means the firmware's own transmit queue is full; data is held (``flow_ctl`` counter)
until a packet arrives with ``flow`` clear.

Under sustained load the firmware occasionally keeps ``credit`` at the reserve with data
queued and does not move it again until the host sends another packet; a to-SB mailbox
poke (as WHD uses) does not release it. When data has waited for credit for 20 ms the
driver sends a ``WLC_GET_MAGIC`` control request with id 0 (``pokes`` counter); its reply
matches no pending request and is discarded.

Packet types
------------

The channel field selects one of three packet types.

**Control packets (channel 0)** carry a command to the firmware or its reply. The payload
is a 16-byte CDC header followed by the data.

.. code-block:: none

   +---------------+----------------+-----------------+---------------+---------------+
   | SDPCM, 12 B   | cmd, 4 B       | len, 4 B        | flags, 4 B    | status, 4 B   |  data...
   | chan = 0      | ioctl number   | bytes of data   | see below     | reply status  |
   +---------------+----------------+-----------------+---------------+---------------+

``flags`` holds ``SET`` (bit 1; clear for a read), ``ERROR`` (bit 0, set in a reply that
failed), the interface index at bit 12 and a request ``id`` at bit 16. The firmware copies
the ``id`` into its reply and the driver drops any reply that does not match the one
request in flight (``ctrl_pending_id``). The ``cmd`` is one of two kinds:

* An **ioctl**, such as ``WLC_UP`` or ``WLC_SET_SSID``. ``data`` is the argument, for
  example an SSID structure.
* An **iovar**. ``cmd`` is ``WLC_GET_VAR`` or ``WLC_SET_VAR`` and ``data`` is the variable
  name as a NUL-terminated string followed by the value (``infsingle_iovar_build``). Most
  firmware settings (``cap``, ``country``, ``event_msgs``, ``escan``, ``clmload``) are iovars.

Only one control request is outstanding at a time. The reply is a control packet with the
same ``cmd`` and ``id`` whose data is the result (for a read) and ``status`` the result code.

**Event packets (channel 1)** are sent by the firmware without being asked, to report that
something happened: a scan result, association progress, link up or down, key exchange.
They look like a data packet whose "Ethernet frame" has the special type 0x886C:

.. code-block:: none

   +----------+-------+-----+--------------------+--------------------+--------------+
   | SDPCM    | pad 2 | BDC | Ethernet header 14 | Broadcom header 10 | event msg 48 |  data
   | chan = 1 |       | 4 B | ethertype 0x886C   | subtype, OUI, ...  | big-endian   |
   +----------+-------+-----+--------------------+--------------------+--------------+

The event message holds the event type, ``status``, ``reason``, ``auth_type``, flags, the
peer MAC address, the interface index and ``datalen`` bytes of event-specific data
(for example one BSS record for ``ESCAN_RESULT``). ``infsingle_event_parse`` decodes it.
The firmware only sends events enabled in the ``event_msgs`` bitmask, which the driver
sets after boot. The events used are:

======================== =========================================================
``ESCAN_RESULT`` (69)    One scan result, or the end of the scan
``SET_SSID`` (0)         Join progress: the join request completed
``AUTH`` (3)             Join progress: authentication
``ASSOC`` (7)            Join progress: association
``LINK`` (16)            Link up or down
``PSK_SUP`` (46)         WPA key exchange progress (keyed = join complete)
``DEAUTH`` (5)           The link was dropped (also ``DEAUTH_IND`` 6,
                         ``DISASSOC`` 11, ``DISASSOC_IND`` 12)
``EXT_AUTH_REQ`` (187)   SAE authentication request (not handled: firmware SAE only)
======================== =========================================================

**Data packets (channel 2)** carry one Ethernet frame in each direction. The payload is a
4-byte **BDC header** followed by the frame. ``hdrlen`` is 14 because two pad bytes
follow the SDPCM header and keep the Ethernet payload aligned.

.. code-block:: none

   +----------+-------+----------------------------+-----------------------------+
   | SDPCM    | pad 2 | BDC: flags, priority,      | Ethernet frame              |
   | chan = 2 |       |      flags2 (iface), off   | dst, src, type, payload     |
   +----------+-------+----------------------------+-----------------------------+

``priority`` is the 802.1D priority (0-7). The driver maps it to a WMM access category
and sends higher categories first. On receive, the driver removes the SDPCM and BDC
headers and passes the Ethernet frame to the network stack. On transmit it writes the
headers into the headroom of the ``net_pkt`` (``INFSINGLE_TX_HEADROOM``).

Transport-level commands (not packets)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Besides packets, the host reads and writes registers on function 0 and function 1: the
CCCR, the clock and sleep controls, the backplane window address, and the chip's cores
(for firmware download and interrupt status). These are single reads and writes, not
SDPCM packets, and are used only during boot and when servicing an interrupt. Firmware
download itself writes the firmware image into chip RAM through the backplane window
(``bp_write``); it is a transfer of raw memory, not a stream of packets.

Design
------

* **Transport.** ``infsingle_bus.h`` is an ops table over SDIO and gSPI, so the upper
  layers never call the SDIO or SPI subsystems.
* **Control.** One request at a time, answered on channel 0. Events arrive on channel 1.
* **Flow control.** One credit is consumed per packet sent; two are reserved for control
  requests. Without credit the driver thread polls until credits arrive inside received
  packets, and pokes the firmware with a control request if starvation lasts 20 ms.
* **Ownership.** ``struct infsingle_dev`` is a singleton. After boot, all bus access
  happens in one driver thread, so the bus layer needs no locking. Other threads reach
  the thread through the control request or the TX queues.
* **Blobs.** Firmware, NVRAM and CLM for the one configured part are linked in as C
  arrays generated at build time by ``gen_blob.py``.

Code dependencies
-----------------

Each layer calls only the one below it. Arrows point from caller to callee.

.. code-block:: none

   Zephyr net stack / wifi_mgmt                  device tree, Kconfig
          |                                             |
          v                                             v
   +----------------+  events, ioctls   +--------------------------------+
   | infsingle_mgmt |------------------>| infsingle_netif                |
   | wifi_mgmt_ops  |                   | net_if init, send, driver      |
   +----------------+                   | thread, device definition      |
          |                             +--------------------------------+
          | iovar / ioctl / events             |              |
          v                                    v              |
   +----------------+  CDC / events    +----------------+     |
   | infsingle_ctrl |<-----------------| infsingle_proto|     |
   | ioctl engine,  |  encode/decode   | SDPCM/CDC/BDC  |     |
   | event dispatch,|----------------->| credit, events |     |
   | CLM, post-boot |                  | (pure helpers) |     |
   +----------------+                  +----------------+     |
          |                                                   |
          v                                                   |
   +----------------+       +----------------+                |
   | infsingle_boot |------>| infsingle_fw   |                |
   | power-up,      | blobs | blob lookup    |                |
   | download, run  |       +----------------+                |
   +----------------+                                         |
          |        \                                          |
          |         +--> infsingle_chip (chip table, cores)   |
          v                                                   v
   +----------------------------------------------------------------+
   | infsingle_bus.h  ops table                                     |
   +--------------------------------+-------------------------------+
   | infsingle_bus_sdio             | infsingle_bus_spi (gSPI)      |
   | sd subsystem, CCCR, F1/F2      | spi_transceive, gSPI commands |
   +--------------------------------+-------------------------------+
          |                                      |
          v                                      v
        SDHC controller                    SPI / PIO controller

===================== ======================================================
``infsingle_bus_*``   transport behind the ``infsingle_bus.h`` ops table, backplane window
``infsingle_chip``    chip table (Wi-Fi 6: 55500/55530/55560; Wi-Fi 5: 43430, 43439, 43012, 4373)
``infsingle_boot``    power-up; BLHS handshake (Wi-Fi 6) or core reset (Wi-Fi 5); download
``infsingle_fw``      firmware, CLM and NVRAM blob access
``infsingle_proto``   pure SDPCM/CDC/BDC/event helpers (unit tested)
``infsingle_ctrl``    ioctl/iovar engine, event dispatch, CLM load, post-boot
``infsingle_netif``   zero-copy TX/RX data path, driver thread, device definition
``infsingle_mgmt``    Zephyr ``wifi_mgmt_ops``: scan, connect, disconnect, status
===================== ======================================================

Bus differences handled below the ops table:

* SDIO reads the frame length from the SDPCM tag at the start of the frame. gSPI uses
  the optional ``rx_len`` op (STATUS register) and ``wait_f2_ready``.
* gSPI has no CCCR. Boot skips the KSO and CCCR interrupt setup and requests the ALP
  clock through ``CHIP_CLOCK_CSR``.
* On boards where the SPI data and interrupt lines share one pin (Pico W), the gSPI
  transport switches that pin between SPI data and GPIO interrupt with pinctrl.

Data flow
---------

Boot (runs in ``infsingle_init`` at POST_KERNEL):

.. code-block:: none

   infsingle_init
     |- create bus (SDIO or gSPI, chosen by the parent node of the devicetree node)
     |- infsingle_boot
     |    |- bus init: WL_REG_ON sequence, bus bring-up
     |    |- identify chip (chipcommon ID) and match it to infsingle_chip
     |    |- stage 1: clocks (ALP, KSO or ALP request)
     |    |- stage 2: download firmware, then NVRAM, through the backplane window
     |    |- stage 3: release the core, wait for F2 ready, enable interrupts
     |- infsingle_netif_start: start the driver thread
     '- infsingle_ctrl_postboot: cap, country, event mask, CLM, WLC_UP

Control path (for example ``wifi scan``). The caller blocks on ``ctrl_done``;
the driver thread owns the bus.

.. code-block:: none

   wifi_mgmt caller        infsingle_ctrl        driver thread          chip
         |  iovar/ioctl         |                     |                  |
         |--------------------->| take ctrl_lock      |                  |
         |                      | build CDC in ctrl_tx|                  |
         |                      | post WAKE_CTRL ---->| tx_ctrl          |
         |                      |                     |--- F2 write ---->|
         |                      |                     |<-- F2 read ------|
         |                      |<-- ctrl_rx (ch 0) --| rx_one           |
         |<---- ctrl_done ------| copy response       |                  |
         |                      |                     |                  |
         |   scan/join progress: events on channel 1 -| ctrl_event_rx    |
         |<-- scan_done, join_done, wifi_mgmt events --| subscribers      |

TX data path:

.. code-block:: none

   net stack -> infsingle_send
                  |- choose WMM access category queue (txq[ac]), bounded by txq_len
                  '- post WAKE_TX
   driver thread -> tx_data
                  |- needs a credit beyond the control reserve and flow clear;
                  |  otherwise the thread re-polls every 5 ms (and pokes after 20 ms)
                  |- add SDPCM + BDC headers in the net_pkt headroom
                  |- contiguous packet: write in place (zero copy)
                  |  multi-fragment packet: linearise into tx_scratch
                  '- bus frame_write (length rounded to bus block) -> F2 FIFO

RX data path:

.. code-block:: none

   chip signals data (SDIO / host-wake interrupt, or 50 ms poll fallback)
     -> infsingle_irq (ISR): mask the interrupt, post WAKE_IRQ
     -> driver thread: service_int, then up to INFSINGLE_RX_BURST x rx_one
          (also on every poll pass and while TX is blocked: the firmware does not
          always signal a frame, and an unread frame can hold back TX credit;
          such reads are counted as ``unsignaled``)
          |- find frame length (SDPCM tag, or rx_len on gSPI)
          |- allocate net_pkt, frame_read directly into its buffer
          |- parse SDPCM header: update credit limit and flow-control state
          |- channel 0 -> infsingle_ctrl_rx      (control reply)
          |- channel 1 -> infsingle_ctrl_event_rx (firmware event)
          '- channel 2 -> strip BDC header -> net_recv_data -> net stack
     -> unmask the interrupt

Zero copy: contiguous TX packets are written to the bus in place (headers go
into net_pkt headroom); RX frames are read directly into a net_pkt buffer.
Only multi-fragment TX is linearised.

Enabling the driver
-------------------

Add an ``infineon,airoc-wifi`` node under the SDHC node (``wifi-reg-on-gpios``,
optional ``wifi-host-wake-gpios``) and fetch the blobs with
``west blobs fetch hal_infineon``. ``CONFIG_WIFI_AIROC`` is then enabled
automatically. For gSPI, place the node under an SPI controller and use the
``infineon,airoc-wifi-spi`` binding properties (see ``rpi_pico_rp2040_w.dts``).

Device selection
----------------

The board Kconfig names the connectivity device with ``AIROC_PART`` and the
matching module choice (for example ``CYW55500`` plus ``CYW55513_MOD_PSE84_SOM``).
``CMakeLists.txt`` maps that choice to the firmware, CLM and NVRAM files and
``gen_blob.py`` converts only those into arrays named ``infsingle_blob_{fw,nvram,clm}``.
The boot code checks the detected chip ID against ``CONFIG_WIFI_AIROC_CHIP_ID``.
Production firmware is always used unless ``CONFIG_WIFI_AIROC_MFG_FIRMWARE`` is set.
``CONFIG_WIFI_AIROC_CUSTOM_BLOBS`` lets you supply your own files.

Statistics
----------

With :kconfig:option:`CONFIG_NET_STATISTICS_WIFI`, the driver implements the
standard Wi-Fi statistics get/reset callbacks. Enable
:kconfig:option:`CONFIG_NET_STATISTICS_USER_API` and the Wi-Fi shell to use
``wifi stats`` and ``wifi stats reset``.

Packet and 64-bit byte totals count Ethernet frames received from the firmware
or successfully transferred to it, excluding SDPCM/BDC headers and bus padding.
RX totals include frames subsequently rejected by the network stack. Unicast,
multicast and broadcast counts classify the Ethernet destination address.
Errors count host receive/parse and data-transmit failures; overruns count
buffer-capacity, allocation and queue-exhaustion drops.

``infsingle stats`` also shows driver-specific drop, credit-starvation, flow-control,
unsignaled-frame, poke, copying, bus-error and firmware-halt counters, plus the current
credit window. Both reset commands clear the same counters.
Reads and resets are synchronized with updates; they do not reset firmware
credits or alter queued packets.

These are host-driver statistics, not radio-delivery or TCP statistics. TX
success does not imply an acknowledgement from the access point. Beacon counts
remain zero because firmware beacon statistics are not collected; radio retries
and airtime are not reported.

Known gaps
----------

* gSPI attachment (``infineon,airoc-wifi-spi`` bindings, e.g. Raspberry Pi Pico W) is
  implemented in ``infsingle_bus_spi.c`` but not yet hardware verified.
* CYW43022 and CYW43438 are not supported.
* AP mode is implemented on the primary interface (open, WPA2-PSK, firmware SAE; 20 MHz) but
  untested on hardware; concurrent AP+STA is not supported.
* SAE via the host supplicant (``EXT_AUTH_REQ``): only firmware SAE.
* Power management (PM1/PM2, KSO) and statistics reporting are not implemented.
* Protocol/statistics helpers and offload control paths have tests under
  ``tests/drivers/wifi/infineon``. The ``offload/hardware`` suite exercises
  live firmware controls; connected-network and low-power behavior need
  separate validation.
* Hardware-verified: CYW55513 (kit_pse84_eval) and CYW4343W (cy8cproto_062_4343w) scan.
  The other Wi-Fi 5 rows are untested.
