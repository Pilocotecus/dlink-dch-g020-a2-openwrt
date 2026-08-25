# Z-Wave status

Current development status of the D-Link DCH-G020 A2 OpenWrt Z-Wave interface.

## Hardware / controller

- USB Z-Wave controller: `0658:0200`
- Device: `/dev/ttyACM0`
- Z-Wave firmware: `Z-Wave 3.92`
- Serial API: `4.19`
- Home ID: `DD BC 20 56`
- Controller Node ID: `1`
- Controller role: primary / SIS / SUC
- Current network node count: `1`

## Validated Serial API operations

Read-only operations validated on real hardware:

- `0x15` - GET_VERSION
- `0x20` - MEMORY_GET_ID
- `0x02` - SERIAL_API_GET_INIT_DATA
- `0x05` - ZW_GET_CONTROLLER_CAPABILITIES
- `0x07` - SERIAL_API_GET_CAPABILITIES
- `0x41` - ZW_GET_NODE_PROTOCOL_INFO

## Inventory

`zwprobe --inventory /dev/ttyACM0`

discovers the Z-Wave node bitmask and queries protocol information for every
node currently present.

Current network:

- Node 1: controller

## ADD_NODE development

`ZW_ADD_NODE_TO_NETWORK (0x4A)` is advertised by the controller.

The following components have been implemented and validated offline:

- ADD_NODE START/STOP frame construction
- callback parser
- ADD_NODE state machine
- full callback pipeline
- real receive path through `receive_frame()`
- Serial API ACK handling
- timeout handling

The real Z-Wave network has NOT yet been put into inclusion mode by this tool.

## Current validated source

Development checkpoint: V6.6 RX path.

SHA256 of `tools/zwprobe.c`:

`67ff79cfbecb70493a99ecfe0ea5d80f419425f8449aaeef8fe81c3af4ae83ef`

## Next step

Implement the controlled asynchronous ADD_NODE loop with:

- global timeout
- callback ID validation
- DONE / FAILED termination
- guaranteed STOP
- post-operation inventory verification

Only after that will real inclusion be tested.
