# OpenWrt for D-Link DCH-G020 A2 / mydlink Home Hub V2

Experimental OpenWrt port / firmware adaptation for the
**D-Link DCH-G020 hardware revision A2 (V2)**.

This project was created after the discontinuation of the original
mydlink Home ecosystem, with the goal of reusing the hardware instead
of discarding it.

## Status

Tested successfully on real DCH-G020 A2 hardware.

Working:

- OpenWrt 21.02.0 boots successfully
- Ethernet
- Wi-Fi
- USB
- Internal Z-Wave controller
- Persistent JFFS2 overlay
- LEDs
- GPIO / I2C expander
- SSH / LuCI
- Z-Wave Serial API communication

The device currently identifies in OpenWrt as:

    D-Link DCH-G020 A1

because the OpenWrt DTS is based on the existing A1 support.

The important A2 difference discovered during reverse engineering is
the firmware/kernel split used by the original D-Link upgrader.

## Hardware

Detected SoC:

    Qualcomm Atheros QCA9533 / QCA9531 family

Flash layout detected after boot:

    mtd0  0x00010000  u-boot
    mtd1  0x00010000  art
    mtd2  0x00010000  mp
    mtd3  0x00010000  config
    mtd4  0x00010000  bootarg
    mtd5  0x00e70000  firmware
    mtd6  0x00200000  kernel
    mtd7  0x00c70000  rootfs
    mtd8              rootfs_data
    mtd9  0x00140000  dlink

The critical A2 firmware split is:

    kernel_size = 0x200000

The standard OpenWrt A1 factory image uses:

    kernel_size = 0x20000

Changing the D-Link image header split to `0x200000`
allows the OEM A2 firmware updater to place the kernel and rootfs
at the correct flash boundaries.

## Z-Wave

The internal controller enumerates as:

    USB VID:PID 0658:0200
    driver: cdc_acm
    device: /dev/ttyACM0

The original D-Link firmware used a `zw_center` daemon and initialized
the serial interface approximately as:

    115200 baud
    8N1
    raw mode
    CLOCAL
    CREAD
    DTR = 0
    RTS = 0

Direct Serial API communication has been confirmed.

Example response:

    Z-Wave 3.92

The controller identity observed during testing was:

    Home ID: DDBC2056
    Node ID: 1

These values belong to the specific test device and are NOT universal.

`SERIAL_API_GET_INIT_DATA` confirmed the controller and Serial API are
operational under OpenWrt.

## Firmware

See:

    firmware/openwrt-21.02.0-dch-g020-a2-factory.bin

This image is based on the official OpenWrt 21.02.0
D-Link DCH-G020 A1 factory image with the D-Link A2 firmware header
kernel split adapted from:

    0x00020000

to:

    0x00200000

No OEM D-Link firmware is distributed in this repository.

## Important warning

This is experimental firmware.

Flashing firmware always carries a risk of bricking the device.

Before flashing:

- Verify that your device is DCH-G020 hardware revision A2 / V2.
- Use wired Ethernet.
- Do not interrupt power during flashing.
- Do not use this image on unknown hardware revisions.
- Keep a recovery method available if possible.

You use this firmware at your own risk.

## Installation

See:

    docs/flashing-guide.md

## Reverse engineering notes

See:

    docs/reverse-engineering-notes.md

## Z-Wave diagnostic tool

Source:

    tools/zwprobe.c

The tool was compiled using the official OpenWrt 21.02.0
ath79/generic SDK:

    GCC 8.4.0
    musl
    mips_24kc

## License

Documentation and original project utilities are released under GPL-2.0.

OpenWrt components retain their respective upstream licenses.
