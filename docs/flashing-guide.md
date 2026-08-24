# Flashing guide — D-Link DCH-G020 A2 / V2

## Tested device

This procedure was tested on a D-Link DCH-G020 A2 / mydlink Home Hub V2.

Do not assume other hardware revisions use the same layout.

## Preparation

Connect a computer directly to the Ethernet port of the DCH-G020.

A static computer address can be used while the original firmware is running,
for example:

    PC: 192.168.0.2/24

The original hub used:

    192.168.0.60

After OpenWrt boots, the default OpenWrt LAN address is:

    192.168.1.1

## Firmware

Use:

    firmware/openwrt-21.02.0-dch-g020-a2-factory.bin

Verify its SHA256 against `SHA256SUMS`.

## Flash

Upload the factory image through the original D-Link firmware upgrade page.

Do not remove power.

During the tested upgrade:

- the status LED became red while flashing
- it later changed to green
- the original web page countdown continued past zero
- the device eventually rebooted into OpenWrt

The browser page may not correctly detect completion.

## After reboot

Set the PC Ethernet interface to:

    192.168.1.2/24

Then test:

    ping 192.168.1.1

SSH:

    ssh root@192.168.1.1

Default OpenWrt has no root password on first boot.

Set one immediately:

    passwd

## Verify flash layout

Run:

    cat /proc/mtd

Expected important entries:

    "firmware"  0x00e70000
    "kernel"    0x00200000
    "rootfs"    0x00c70000

## Z-Wave

The internal Z-Wave module should appear as:

    /dev/ttyACM0

Check:

    ls -l /dev/ttyACM0

and:

    dmesg | grep -Ei 'cdc_acm|ttyACM'

Expected USB ID:

    0658:0200

## Recovery warning

This project does not currently provide a guaranteed recovery procedure
for a failed bootloader/flash operation.

Do not overwrite:

- u-boot
- art
- mp
- config
- bootarg
- dlink

unless you fully understand the consequences.
