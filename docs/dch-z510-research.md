# D-Link DCH-Z510 — Z-Wave reverse engineering notes

Experimental investigation performed with a D-Link DCH-G020 controller
using the local Serial API probe (`tools/zwprobe.c`).

## Device identification

The DCH-Z510 was included as physical Z-Wave Node 11.

Manufacturer Specific Report:

    72 05 01 08 00 04 00 0A

Decoded:

- Manufacturer ID: `0x0108`
- Product Type ID: `0x0004`
- Product ID: `0x000A`

Node protocol information:

    D3 9C 01 04 10 05

- Basic device class: `0x04`
- Generic device class: `0x10`
- Specific device class: `0x05`
- Listening: yes
- Routing: yes

Inclusion NIF:

    5E 71 20 25 85 70 72 86 30 59 73 5A 98 7A

Advertised command classes include:

- `0x5E` Z-Wave Plus Info
- `0x71` Notification
- `0x20` Basic
- `0x25` Switch Binary
- `0x85` Association
- `0x70` Configuration
- `0x72` Manufacturer Specific
- `0x86` Version
- `0x30` Sensor Binary
- `0x59` Association Group Information
- `0x73` Powerlevel
- `0x5A` Device Reset Locally
- `0x98` Security
- `0x7A` Firmware Update Meta Data

## Notification Command Class

Version query:

    TX: 86 13 71
    RX: 86 14 71 04

Therefore Notification Command Class version is 4.

Notification Supported Get:

    TX: 71 07 00
    RX: 71 08 01 80

Event Supported Get for Home Security:

    TX: 71 01 07
    RX: 71 02 07 01 08

## Configuration

Read-only Configuration Get queries produced:

### Parameter 7

    TX: 70 05 07
    RX: 70 06 07 01 00

Current value: `0x00`.

### Parameter 29

    TX: 70 05 1D
    RX: 70 06 1D 01 00

Current value: `0x00`.

Alarm output is enabled.

### Parameter 31

    TX: 70 05 1F
    RX: 70 06 1F 01 06

Current value: `0x06`.

Alarm duration is configured as 6 units of 30 seconds:

    6 * 30 = 180 seconds

## Basic state

Basic Get while the siren is idle:

    TX: 20 02
    RX: 20 03 00

Basic current value: OFF (`0x00`).

## Switch Binary state

Switch Binary Get while the siren is idle:

    TX: 25 02
    RX: 25 03 00

Switch Binary current value: OFF (`0x00`).

## Audible activation experiment

The following command was experimentally demonstrated to activate
the audible siren:

    25 01 FF

The Z-Wave transaction completed successfully and the siren sounded.

## IMPORTANT: Switch Binary OFF does not stop an active alarm

The following command was sent while the audible alarm was active:

    25 01 00

The Serial API transaction completed with:

    TRANSMIT_COMPLETE_OK

However, the siren continued sounding.

Therefore:

**`TRANSMIT_COMPLETE_OK` only proves successful Z-Wave transport. It does
not prove that the requested semantic action was performed by the device.**

`SWITCH_BINARY_SET 00` must NOT currently be considered a proven audible
alarm stop command for the DCH-Z510.

During this experiment mains power had to be removed to silence the unit.

## Current investigation status

Known:

- `25 01 FF` activates the audible siren.
- `25 01 00` does not immediately cancel an already active audible cycle.
- Idle Switch Binary state reports `00`.
- Idle Basic state reports `00`.
- Alarm duration parameter is currently 180 seconds.
- Alarm output is enabled.
- Notification CC version 4 is implemented.
- Manufacturer/Product IDs have been obtained directly from the device.

Next investigation target:

Determine a safe and reproducible method for cancelling an active audible
alarm.

A candidate is BASIC_SET OFF:

    20 01 00

This command has NOT yet been experimentally validated as an audible stop
mechanism.

Do not integrate automatic siren activation into the Bernal Home daemon
until a reliable stop mechanism has been demonstrated physically.

## Bernal Home integration note

Physical Z-Wave Node 11 is now the DCH-Z510 siren.

The existing logical Bernal Home Node 11 remains reserved for:

    Habitación matrimonio

These identifiers must not be treated as equivalent.

The daemon must eventually decouple physical Z-Wave node IDs from logical
Bernal Home sensor IDs before the siren is integrated.
