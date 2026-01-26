# BLE TCU Address From Advertisement Design

## Goal

- QR code carries only MAC (minimal payload).
- After scanning the target MAC, iot_controller parses BLE advertisement ManufacturerData to extract the dynamic TCU address.
- iot_controller builds the Modbus query command based on the extracted TCU address and writes it to the target characteristic.

## Non-Goals

- Changing the ROS graph or brain_core protocol.
- Persisting the TCU address beyond the current connection attempt.

## QR Payload Format

Minimal QR payload:

```
{"t":"b","m":"D66562A0ADE5"}
```

Vision layer converts this into:

```
{"t":"ble","mac":"D6:65:62:A0:AD:E5"}
```

No `cmd` field is expected in the QR payload.

## Data Flow (High Level)

1. vision_qr_node publishes VisionResult with `t=ble` and `mac` only.
2. brain_core forwards a ConnectBluetooth request with:
   - mac = target MAC
   - service_uuid = "AUTO"
   - characteristic_uuid = "AUTO"
   - command = "NOOP"
3. iot_controller scans for the target MAC and reads advertisement ManufacturerData.
4. iot_controller validates and decrypts the advertisement to extract TCU address.
5. iot_controller builds the Modbus query command with the TCU address and writes it.

## Advertisement Parsing

Expected protocol data length: 26 bytes, header `0x88 0x11`.

### Input Sources

- If the BLE stack provides company_id + value:
  - Require company_id == 0x1188.
  - Use `value` as 26-byte protocol data.
- If the BLE stack provides only a raw value and it begins with `0x88 0x11`:
  - Use that raw value directly as protocol data.

### Checksum Validation

Let `protocol` be the 26-byte protocol data.

```
checksum = protocol[2]
sum = Σ protocol[3..25]
low = sum & 0xff
high = (sum >> 8) & 0xff
result = (low + high) & 0xff
valid = (checksum == result)
```

Reject if checksum fails.

### Key Generation

Let rand0 = protocol[3], rand1 = protocol[4].

```
key[0] = (rand0 + rand1) & 0xff
key[1] = (rand0 ^ rand1) & 0xff
key[2] = (rand0 ^ 0x69) & 0xff
key[3] = key[1]
key[4] = (rand0 ^ 0x16) & 0xff
key[5] = (rand1 ^ 0x58) & 0xff
key[6] = (rand0 ^ rand1 ^ 0x69) & 0xff
```

### Decryption and TCU Address

Encrypted bytes: protocol[5..25] (21 bytes)

```
for i in 0..20:
  original[i] = encrypted[i] ^ key[i % 7]
```

TCU address is `original[16]`.

Validation: require 1 <= TCU address <= 150.

## Command Assembly (Query)

Per protocol documentation, query command is 8 bytes:

```
[tcu, 0x03, 0x00, 0x00, 0x00, 0x25] + CRC16(MODBUS, low byte first)
```

- CRC16 uses standard MODBUS and is appended as low byte then high byte.
- If the ConnectBluetooth request includes a non-NOOP command, it overrides the computed command (forward-compatibility).

## Error Handling

- If no advertisement is available for the target MAC: return error.
- If checksum or decryption fails: return error.
- If TCU address is invalid (0 or > 150): return error.
- Errors are surfaced through the ConnectBluetooth response message.

## Logging

Add debug logs in iot_controller:

- ManufacturerData length and prefix
- Checksum validation result
- Parsed TCU address
- Final command bytes to be written

## Tests

- Unit test for checksum validation.
- Unit test for key generation and XOR decrypt using a provided sample.
- Unit test for command assembly (CRC16, byte order).

## Open Questions

- BLE stack exposure of manufacturer data (company_id/value vs raw value) needs to be confirmed on the Pi.
