# Infineon AIROC Wi-Fi driver: features not yet implemented

## Wi-Fi management (`infsingle_mgmt.c`)

- **AP mode: hardware verification.** `ap_enable`/`ap_disable` are implemented (primary
  interface, open/WPA2-PSK/firmware SAE, 20 MHz only) but have only been compile-tested.
  Not done: secondary AP interface alongside a connected STA (concurrent AP+STA), 40/80 MHz
  bandwidth, WEP, and an AP-side DHCP server (the application must provide one).
- **SAE through the host supplicant.** `EXT_AUTH_REQ` and `EXT_AUTH_FRAME_RX` events are not
  handled. WPA3-SAE works only through firmware SAE (the `sae_password` iovar) and only when
  the firmware `cap` string lists `sae`. Otherwise connect returns `-ENOTSUP`.
- **Security types** other than open, WPA/WPA2-PSK and firmware SAE return `-ENOTSUP`
  (for example enterprise, OWE).

## Power management

- PM1/PM2 power save and the SDIO KSO (keep SDIO on) sleep handshake. The chip is kept awake.
- Host sleep and wake-on-wireless, including the `WL_HOST_WAKE` pin as a wake source.

## Statistics and diagnostics

- Firmware beacon statistics and radio retry/airtime diagnostics. Host interface counters
  are exposed through the Zephyr Wi-Fi statistics API and the `infsingle stats` shell command.
- Firmware crash recovery. A firmware halt is detected but the chip is not rebooted.

## Chips and buses

- CYW43022 and CYW43438 are not in the chip table.
- gSPI (`infsingle_bus_spi.c`) is implemented but has not been verified on hardware.
- Chips verified: CYW55513 and CYW4343W only. Wi-Fi 5 rows for CYW43439, CYW43012 and CYW4373
  are untested.
- Manufacturing-test and other firmware variants are selected only by
  `CONFIG_WIFI_AIROC_MFG_FIRMWARE`; there is no runtime switch.

## Data path

- TX packets with several fragments are copied into a scratch buffer instead of being sent
  in place.
- Packet chaining (`next` field of the SDPCM header) is not used on either direction.

## Tests

- Only the protocol and statistics helpers are unit tested (`tests/drivers/wifi/infineon`).
  There is no test of the boot sequence, the control engine or the data path against a mock
  chip.Upda