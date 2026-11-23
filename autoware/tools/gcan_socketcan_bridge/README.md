# GCAN USBCAN SocketCAN bridge

This adapter exposes the GCAN `USBCANI-V503` (`0c66:000c`) as a virtual
SocketCAN interface named `can0`. The existing Autoware/Hunter driver can keep
using `can0` without being rewritten for the proprietary ECanVCI API.

The bridge also supports one dual-channel USBCAN-II. It opens the USB device
once, then maps physical channel 0 to `can0` and physical channel 1 to `can1`
inside one process. This is required because ECanVCI permits a device to be
opened only once.

The checked-in binary libraries came from GCAN's official `Linux例程.zip`:

- Download page: <https://www.gcan.com.cn/3d05/222b/fb1d/f582/bf83>
- Archive SHA-256: `1a9430c0079df6721f950d652d920eb0978d8288acfc59a4516af5a8a890de40`
- Target: x86-64 Linux

## Build and probe

```bash
make
sudo ./gcan_socketcan_bridge --probe --bitrate 500000
```

Probe mode opens and initializes channel 0 but does not transmit a CAN frame.

## Install

```bash
sudo ./install.sh tang
systemctl status gcan-socketcan-bridge.service
ip -details link show can0
```

The default service creates a `vcan` interface named `can0`, bridges it to
physical channel 0 of device index 0, and restarts automatically if the USB
adapter is reconnected.

Defaults match this vehicle setup: USBCAN-I device type `3`, device index `0`,
channel `0`, and `500000` bit/s.

## Separate feedback and autonomous-control buses

### One USBCAN-II with two physical channels

Build and install the dual-channel service:

```bash
make
sudo ./install.sh tang usbcan2
```

The fixed mappings are:

- `can0` -> USBCAN-II device index `0`, physical channel `0`
- `can1` -> USBCAN-II device index `0`, physical channel `1`

Both channels default to 500 kbit/s. Verify them with:

```bash
systemctl status gcan-usbcan2-bridge.service
ip -details link show can0
ip -details link show can1
candump can0 can1
```

For a non-installed probe that initializes both channels but transmits no
frames, create `can0` and `can1` first and run:

```bash
sudo ./gcan_socketcan_bridge --dual --probe
```

### Two separate USBCAN-I devices

When the vehicle exposes feedback CAN and autonomous-control CAN as two
independent buses, do not electrically join their CAN-H/CAN-L pairs. Use two
USBCAN-I adapters and install dual mode:

```bash
sudo ./install.sh tang dual
```

Dual mode uses these fixed mappings:

- `can0` -> USBCAN-I device index `0`, channel `0`: feedback bus
- `can1` -> USBCAN-I device index `1`, channel `0`: autonomous-control bus

Connect and identify the adapters one at a time before labeling them. After
both adapters are connected, verify both bridge services are active before
starting the chassis driver:

```bash
systemctl status gcan-socketcan-bridge.service
systemctl status gcan-socketcan-control.service
ip -details link show can0
ip -details link show can1
```

## Limitations

- Classic CAN 2.0 frames only (up to 8 data bytes); CAN FD is not supported.
- A virtual interface cannot expose the physical controller's bus-off/error
  counters through SocketCAN. Bridge or USB errors are reported in the service
  journal instead: `journalctl -u gcan-socketcan-bridge.service -f`.
- Do not run a second ECanVCI application while the bridge owns the adapter.
