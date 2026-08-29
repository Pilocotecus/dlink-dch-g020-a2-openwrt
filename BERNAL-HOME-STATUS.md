# Bernal Home

## Hardware validated checkpoint

Date: 2026-08-29

Hardware:

- D-Link DCH-G020 running OpenWrt
- Sigma Z-Wave controller on /dev/ttyACM0
- D-Link DCH-Z110
- Controller Node1
- Validated sensor Node4

## Working pipeline

DCH-Z110
-> Z-Wave
-> DCH-G020
-> bernal-zwave-daemon
-> /tmp/bernal-home/state.json
-> uhttpd
-> Bernal Home web UI

## Contact mapping validated physically

Node4:

- magnet removed = OPEN
- magnet present = CLOSED

Observed embedded commands:

OPEN:
- 71 05 00 00 00 FF 06 16 00
- 31 05 03 01 09

CLOSED:
- 71 05 00 00 00 FF 06 17 00
- 31 05 03 01 08

## Current daemon

bernal-zwave-daemon v0.2

Features:

- owns /dev/ttyACM0
- Z-Wave Serial API reception
- checksum validation
- Serial API ACK
- DCH-Z110 Multi Command decoding
- OPEN/CLOSED state
- battery state
- atomic JSON state publication
- procd service
- automatic respawn
- automatic boot startup

Runtime state:

/tmp/bernal-home/state.json

## Web UI

Served by OpenWrt uhttpd.

Path:

/bernal-home/

Validated live:

- CLOSED -> green
- OPEN -> red
- automatic refresh
- controller online state
- battery warning
- last communication timestamp

## Important

sensor03 and sensor01_raw are intentionally not assigned physical
meanings yet. Their exact Z-Wave Sensor Multilevel encoding still
needs to be established.

zwprobe remains the diagnostic/reverse-engineering utility.
Bernal Home development continues in bernal-zwave-daemon.
