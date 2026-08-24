# Reverse engineering notes

## OEM image header

The D-Link firmware uses a 72-byte image header.

Observed structure:

    uint32_t hdr_len
    uint32_t checksum
    uint32_t total_size
    uint32_t kernel_size
    char signature[32]
    char version[16]
    char region[8]

Header length:

    0x48

Signature:

    HONEYBEE-FIRMWARE-DCH-G020

OEM A2 image:

    kernel_size = 0x200000

Official OpenWrt A1 factory image:

    kernel_size = 0x20000

The payload checksum is a simple unsigned byte sum.

Changing only the kernel split field does not alter the payload checksum.

## A2 flash layout

OpenWrt runtime detection:

    0x000000 - 0x010000  u-boot
    0x010000 - 0x020000  art
    0x020000 - 0x030000  mp
    0x030000 - 0x040000  config
    0x040000 - 0x050000  bootarg
    0x050000 - 0xec0000  firmware
    0xec0000 - 0x1000000 dlink

Within firmware:

    kernel = 0x200000
    rootfs = remainder

## OEM firmware updater

The OEM updater was found to split the factory image using the
`kernel_size` field from the D-Link header.

This is why a normal OpenWrt A1 factory image is not suitable for
the A2 updater without modifying the split field.

## Z-Wave startup

OEM scripts showed:

    ttyACM[0-9]* -> run_zw.sh

and:

    zw_center /dev/$MDEV

The OEM USB initialization also power-cycled hardware using the I2C GPIO
expander.

OpenWrt exports:

    /sys/class/gpio/d-link:power:usb
    /sys/class/gpio/d-link:power:zwave

Toggling `power:zwave` was experimentally confirmed to disconnect and
re-enumerate the internal Z-Wave USB controller.

## Z-Wave USB

Detected:

    VID:PID 0658:0200
    CDC ACM
    /dev/ttyACM0

OEM `zw_center` serial configuration was reverse engineered.

Operational configuration:

    115200 baud
    8 data bits
    no parity
    1 stop bit
    raw
    CLOCAL
    CREAD
    DTR cleared
    RTS cleared

## Serial API tests

GET_VERSION:

    TX:
    01 03 00 15 E9

Observed response contained:

    Z-Wave 3.92

MEMORY_GET_ID:

    TX:
    01 03 00 20 DC

Test device response:

    Home ID = DDBC2056
    Node ID = 1

SERIAL_API_GET_INIT_DATA:

    TX:
    01 03 00 02 FE

The returned node mask showed only Node 1 on the tested controller.

These identifiers are specific to the test device and must not be copied
into other devices.
