<!--
SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
SPDX-License-Identifier: Apache-2.0
-->

# Infineon AIROC Wi-Fi user guide

## Zephyr integration

This is a native Ethernet-frame Wi-Fi driver, not a WHD wrapper or a socket
offload driver. Zephyr owns IP, DHCP, TCP/UDP and sockets. The WLAN firmware
owns scanning, association, authentication and the radio. Applications use
the normal Wi-Fi management requests and events (`wifi_mgmt.h`) to scan,
connect, disconnect and manage the optional SoftAP.

The driver supports one chip per image, attached over SDIO or gSPI. At device
initialization it powers the chip, verifies its ID, loads firmware, NVRAM and
CLM regulatory data, and starts the driver thread. The network interface
remains dormant until association succeeds. That thread owns the bus and
services control replies, firmware events and Ethernet traffic. Applications
never need WHD types or WHD libraries.

Enable networking and Wi-Fi in the application configuration:

```ini
CONFIG_NETWORKING=y
CONFIG_WIFI=y
CONFIG_NET_IPV4=y
CONFIG_NET_DHCPV4=y
```

Enable IPv6 as appropriate. `CONFIG_WIFI_AIROC` defaults to enabled when an
enabled `infineon,airoc-wifi` node has a supported bus parent. It selects the
Ethernet layer, Wi-Fi management and native networking. The parent controller
and its clocks, pinctrl and GPIO controllers must also be enabled.

Select the chip and module with the existing `AIROC_PART` and module Kconfig
choices. For example, a CYW55513 PSE84 SOM uses:

```ini
CONFIG_CYW55500=y
CONFIG_CYW55513_MOD_PSE84_SOM=y
```

Use the selection matching the physical module, not just its chip family.
Supported parts are CYW4343W, CYW43439, CYW43012, CYW4373, CYW55500 and
CYW55572; CYW43438 and CYW43022 are not supported. Fetch binary firmware and
CLM resources with `west blobs fetch hal_infineon`. NVRAM comes from the
Infineon HAL module. Only the selected images are linked into flash.

Custom images can instead be selected with `CONFIG_WIFI_AIROC_CUSTOM_BLOBS`
and `CONFIG_WIFI_AIROC_CUSTOM_FW_PATH`, `CONFIG_WIFI_AIROC_CUSTOM_CLM_PATH`
and `CONFIG_WIFI_AIROC_CUSTOM_NVRAM_PATH`. Use production firmware for normal
operation; `CONFIG_WIFI_AIROC_MFG_FIRMWARE` is for manufacturing tests.
The devicetree `firmware-variant` property does not select images in this driver.

### SDIO devicetree

Place the Wi-Fi node directly under the SD host controller. The following
overlay is illustrative: replace controller labels, pins and pinctrl settings
with the board's actual wiring.

```dts
#include <zephyr/dt-bindings/gpio/gpio.h>

&sdhc0 {
	status = "okay";
	bus-width = <4>;
	max-bus-freq = <50000000>;
	/* Add the board-specific SDIO pinctrl, clocks and voltage configuration. */

	wifi: wifi {
		compatible = "infineon,airoc-wifi";
		wifi-reg-on-gpios = <&gpio0 5 GPIO_ACTIVE_HIGH>;
		wifi-host-wake-gpios = <&gpio0 6 GPIO_ACTIVE_HIGH>;
		country-code = "US";
		status = "okay";
	};
};
```

`wifi-reg-on-gpios` is needed by the driver to reset and power the chip.
`wifi-host-wake-gpios` is optional on SDIO; its polarity must match the module.
Use the actual operating country's regulatory code; `"XX"` is the default.
See `dts/bindings/wifi/infineon,airoc-wifi.yaml` and the
`kit_pse84_eval_m33_ns.dts` board description for a complete SDIO example.

### gSPI devicetree

Use the **same compatible** under an SPI controller. The bus selects the SPI
binding; `"infineon,airoc-wifi-spi"` is not a compatible string.

```dts
#include <zephyr/dt-bindings/gpio/gpio.h>

&spi0 {
	status = "okay";
	cs-gpios = <&gpio0 10 GPIO_ACTIVE_LOW>;
	/* Add the board-specific SPI pinctrl and clocks. */

	wifi: wifi@0 {
		compatible = "infineon,airoc-wifi";
		reg = <0>;
		spi-max-frequency = <25000000>;
		wifi-reg-on-gpios = <&gpio0 5 GPIO_ACTIVE_HIGH>;
		wifi-host-wake-gpios = <&gpio0 6 GPIO_ACTIVE_HIGH>;
		bus-select-gpios = <&gpio0 7 GPIO_ACTIVE_HIGH>;
		country-code = "US";
		status = "okay";
	};
};
```

SPI requires `wifi-host-wake-gpios` and `bus-select-gpios`; the latter must
select SPI mode while WL_REG_ON rises. `spi-word-size` defaults to 8, with
16 and 32 also accepted. Use `spi-half-duplex` and `spi-data-irq-shared` only
when required by the board. See `infineon,airoc-wifi-spi.yaml` and the Pico W
board devicetree for shared data/IRQ wiring. gSPI is not hardware verified.

## Firmware offload extensions

Include the separate public header:

```c
#include <zephyr/drivers/wifi/infineon_offload.h>
```

These are driver-specific extensions, not `net_mgmt` commands. Every function
takes a Zephyr `const struct device *`, for example
`DEVICE_DT_GET(DT_NODELABEL(wifi))` or `net_if_get_device(iface)`. The driver
checks the device's API before accessing private state.

Offloads are **opt-in**: boot does not install WHD's default profile or enable
TCP auto-configuration. A profile changes which firmware features are skipped;
it is not a bitmap of features actually implemented by the firmware. BDO,
DHCP renewal, PNO and GTK have profile controls only, matching the usable
surface of the legacy driver.

### Check support first

```c
static int configure_keepalive(const struct device *dev)
{
	bool supported;
	int ret;

	ret = wifi_infineon_offload_is_supported(dev, WIFI_INFINEON_OFFLOAD_KEEPALIVE,
						&supported);
	if (ret != 0) {
		return ret;
	}
	if (!supported) {
		return -ENOTSUP;
	}
	return wifi_infineon_offload_keepalive_set(dev, 30000U, NULL, 0);
}
```

Pass exactly one `enum wifi_infineon_offload` value, not an OR of features.

| Result | Meaning |
| --- | --- |
| `0`, `supported == true` | Firmware advertises the capability or accepts a read-only control probe. |
| `0`, `supported == false` | Capability is absent, the probe is unsupported, or TCP supports zero connections. |
| `-ENODATA` | No reliable individual query exists, including keepalive on firmware without a getter. |
| Other negative errno | Device, context, transport or response error; support was not established. |

`WIFI_INFINEON_OFFLOAD_PROFILE` checks the exact `offloads` firmware capability
token and `WIFI_INFINEON_OFFLOAD_ICMP` checks `icmp`. ARP probes `arp_version`,
ND probes `ndoe`, TCP probes its maximum-connection subcommand, keepalive probes
`keep_alive`, and packet filtering probes `pkt_filter_mode`. A successful
query does not mean an offload is enabled or usable with every configuration.
Some firmware accepts keepalive configuration but has no corresponding getter;
the query returns `-ENODATA`, not a false claim of non-support. All configuration
calls must still have their return value checked.

### Profile and host addresses

| Function | Operation |
| --- | --- |
| `wifi_infineon_offload_profile_set(dev, features, reset)` | Select the low-power profile, or remove it when `reset` is true. |
| `wifi_infineon_offload_profile_enable(dev, features, enable)` | Activate/deactivate selected profile features. |
| `wifi_infineon_offload_ipv4_update(dev, features, address, add)` | Add/remove a network-order `struct in_addr`. |
| `wifi_infineon_offload_ipv6_update(dev, features, address, anycast, add)` | Add/remove a network-order `struct in6_addr`; false selects unicast. |

These calls require the `offloads` capability. `features` is an OR of the
following bits; `WIFI_INFINEON_OFFLOAD_ALL` selects all ten. Zero skips all
features. The query-only `WIFI_INFINEON_OFFLOAD_PROFILE` bit is not allowed.

| Profile bit (`WIFI_INFINEON_OFFLOAD_` prefix) | Purpose and configuration |
| --- | --- |
| `ARP` | IPv4 address resolution; register host addresses and configure the ARP agent. |
| `ND` | IPv6 Neighbor Discovery; register host IPv6 addresses. |
| `BDO` | Firmware-defined; the legacy driver does not document its meaning. |
| `ICMP` | Echo request connectivity checks, with the separate ICMP API below. |
| `TCP` | Connection-aware TCP keepalive, with timing and auto-configuration below. |
| `DHCP` | DHCP lease renewal; profile-only, no standalone lease provisioning API. |
| `PNO` | Preferred network scanning; profile-only, no usable preferred-network setup API in this legacy ABI. |
| `KEEPALIVE` | Periodic NULL or supplied packet transmission. |
| `GTK` | GTK offload engine; key handling is firmware-specific, with no standalone API. |
| `PACKET_FILTER` | Low-power receive mask/pattern filters. |

For example, after checking `PROFILE` support, configure an IPv6 responder:

```c
static int configure_nd(const struct device *dev, const struct in6_addr *address)
{
	int ret;

	ret = wifi_infineon_offload_profile_set(dev, WIFI_INFINEON_OFFLOAD_ND, false);
	if (ret != 0) {
		return ret;
	}
	ret = wifi_infineon_offload_ipv6_update(dev, WIFI_INFINEON_OFFLOAD_ND,
					       address, false, true);
	if (ret != 0) {
		return ret;
	}
	return wifi_infineon_offload_profile_enable(dev, WIFI_INFINEON_OFFLOAD_ND, true);
}
```

Remove obsolete addresses with `add == false`. Deactivate a profile with
`enable == false` before removing its configuration. Some firmware accepts
profile configuration and activation but rejects profile removal with
`-ENOTSUP`; deactivation still works.

### ARP agent

The ARP agent API can be used without general profile support:

| Function | Operation |
| --- | --- |
| `wifi_infineon_offload_arp_set/get` | Write/read `wifi_infineon_arp_config`: enable, feature flags and peer age in seconds. |
| `wifi_infineon_offload_arp_address_add` | Add one network-order IPv4 host address. |
| `wifi_infineon_offload_arp_addresses_get` | Read into an array of eight addresses and return the nonzero entry count. |
| `wifi_infineon_offload_arp_addresses_clear` | Remove all host addresses. |
| `wifi_infineon_offload_arp_cache_clear` | Clear the peer cache without removing host addresses. |

Feature flags are `WIFI_INFINEON_ARP_AGENT`, `WIFI_INFINEON_ARP_SNOOP`,
`WIFI_INFINEON_ARP_HOST_REPLY` and `WIFI_INFINEON_ARP_PEER_REPLY`. To respond
for an IPv4 host, add its address, then set `.enable = true` and
`.features = WIFI_INFINEON_ARP_AGENT | WIFI_INFINEON_ARP_HOST_REPLY`.
Set `.peer_age` to the required cache lifetime in seconds. The address table
holds up to eight entries. Use the profile IPv4 removal command where available,
or disable the agent and clear/rebuild the table to remove an individual entry.

### Generic keepalive

`wifi_infineon_offload_keepalive_set(dev, period_ms, packet, length)` sends a
NULL keepalive when `packet == NULL` and `length == 0`, or the supplied packet
bytes otherwise. This corresponds to the legacy NULL/NAT keepalive controls,
not connection-aware TCP keepalive. The maximum supplied packet size is 1400
bytes. A zero period disables transmission. Build any supplied packet in the
format expected by the firmware, including its required network headers.

### TCP keepalive (TKO)

| Function | Operation |
| --- | --- |
| `wifi_infineon_offload_tcp_set/get` | Configure/read probe interval, retry interval (seconds) and retry count. |
| `wifi_infineon_offload_tcp_filter_set` | Configure auto-selection by local/remote IP address and port; zeros are wildcards. |
| `wifi_infineon_offload_tcp_enable` | Enable/disable both auto-configuration and the TCP keepalive engine. |
| `wifi_infineon_offload_tcp_status_get` | Read up to sixteen connection status codes before disabling the engine. |

Zero timing fields select defaults: 20-second interval, 3-second retry interval
and three retries. Filter ports use host byte order. Filter addresses use
network-order bytes; IPv4 occupies the first four bytes of the 16-byte field,
with the remaining bytes zero. Disable TCP keepalive before changing the filter,
then re-enable it. Firmware learns connections automatically from traffic;
there is no manual connection-registration API here. Configure the filter before
the traffic from which firmware should learn the connection. Firmware can require
station association for filtering and auto-configuration (`-ENOTCONN`).

Status codes distinguish normal operation, no response, missing ACK, unexpected
flags, invalid sequence/remote sequence, available TCP data and unused slots;
see `enum wifi_infineon_tcp_keepalive_state`. Application and firmware must not
both manipulate the same live TCP state while the host sleeps. This API does
not transfer Zephyr socket state or suspend Zephyr's TCP timers.

### ICMP connectivity checks

ICMP requires the separate `icmp` capability, not just a profile bit.
Associate the station before configuring the engine or peers; firmware reports
`-ENOTCONN` when association is required.

| Function | Operation |
| --- | --- |
| `wifi_infineon_offload_icmp_enable` | Enable/disable the ICMP engine. |
| `wifi_infineon_offload_icmp_peer_add` | Configure peer IP/MAC, nonzero period and duration in seconds. |
| `wifi_infineon_offload_icmp_peer_action` | `WIFI_INFINEON_ICMP_START`, `STOP` or `DELETE` for a configured peer. |
| `wifi_infineon_offload_icmp_info_get` | Read engine enable, peer count, firmware state and peer configuration. |
| `wifi_infineon_offload_icmp_callback_set` | Register/replace a completion callback; NULL unregisters it. |

`wifi_infineon_offload_ip.family` is `AF_INET` or `AF_INET6`; fill the
corresponding network-order address union member. Enable the engine, register
a callback, add a peer, then start it. The caller supplies the peer's MAC
address. Stop and delete peers when no longer needed.

Callbacks report the peer address, request count and success, timeout or
disconnection reason. They run in the **driver thread** and must not block or
call driver APIs. Copy event data and submit work to another thread. Event
storage expires when the callback returns. Unregistering does not wait for a
callback already executing; its context must remain valid until it finishes.
The driver enables ICMP completion events during boot only when advertised.

### Low-power packet filters

`wifi_infineon_offload_packet_filter_add` accepts an ID, match inversion flag,
Ethernet-frame byte offset, and equal-size mask/pattern arrays. Both arrays
must be non-NULL and 1 through 640 bytes long. Creating a filter does not
enable it; use `wifi_infineon_offload_packet_filter_enable`. Remove it with
`wifi_infineon_offload_packet_filter_remove`. Read matched/discarded/forwarded
counters with `wifi_infineon_offload_packet_filter_stats_get`.

For an IPv4 EtherType match, use offset 12, mask `{0xff, 0xff}`, pattern
`{0x08, 0x00}`, and `.negate = false`. Choose rules to preserve needed ARP,
IPv6, DHCP and application traffic. Filter matching is not a complete host
wake policy: firmware low-power behavior and hardware wake routing must also
be configured and validated for the device.

## Execution, errors and limitations

All offload APIs are synchronous, supervisor-only thread calls. They return
zero on success or a negative errno. `-ENODEV` rejects another driver's device,
`-EAGAIN` means not ready, `-EINVAL` rejects invalid arguments, `-ENOTSUP`
means unsupported firmware functionality, `-EMSGSIZE` means oversized data,
`-EBADMSG` means malformed/truncated firmware output, `-ENOBUFS` means the
firmware needs more workspace, `-ENOTCONN` means station association is
required, `-ETIMEDOUT` means a control timeout, and `-EIO` means another firmware
failure. Calls from an ISR return `-EWOULDBLOCK`; calls from the driver thread
return `-EDEADLK`.

Control transactions are serialized, but multi-command configuration sequences
are not atomic and do not roll back earlier successful commands. Serialize
application configuration changes, check every return value, and explicitly
clean up partial configurations when needed. Packet/filter calls use up to
about 1.4 KiB of stack for encoding; provide sufficient caller stack, typically
at least 4 KiB including application use.

The application must keep firmware address lists synchronized with Zephyr
address changes and remove stale entries after DHCP changes, disconnects or
interface reconfiguration. Disable station offloads, including ARP and ND,
before switching to SoftAP; the extension does not coordinate these changes
with Wi-Fi management.

These controls do not implement host suspend/resume, radio power management,
DHCP lease handoff, Zephyr TCP-state handoff, or wake GPIO policy. The driver
still keeps the radio awake after boot. Enabling a profile does not by itself
make Zephyr sleep safely. Firmware-specific profile-only features require
vendor documentation and hardware validation before use.

See `README.rst` for the driver's wire protocol and data path. The offload
tests are in `tests/drivers/wifi/infineon/offload`. Its simulated-firmware suite
tests wire formats, capability checks, malformed replies and event dispatch
without requiring a WLAN chip. The `hardware` subdirectory exercises the live
driver and WLAN firmware; it skips association-dependent controls when disconnected.
Neither suite establishes low-power wake behavior or over-the-air offload behavior.

On the PSE84 eval kit's CYW55513 production firmware, the live suite validates
support queries, ARP configuration/address tables, IPv4/IPv6 profile updates and
activation, NULL keepalive configuration, packet filter controls/counters, and
TCP timing. That firmware accepts keepalive SET but not GET, rejects profile
removal, and requires association for ICMP and TCP auto-configuration. Those
association-dependent paths require separate connected-network validation.
