# BLE TCU Response Parsing & TTS Design

## Goal

- Subscribe to the BLE notify characteristic (FFF1) and parse the parameter query response.
- Validate CRC16 (MODBUS, low byte first) and parse fields using big-endian for U16/I16.
- Convert parsed values into a concise Chinese TTS sentence and publish directly to `/audio/tts_play`.

## Non-Goals

- Changing brain_core or adding new ROS topics for parsed values.
- Full parsing of every field (only the required subset).

## Data Flow (High Level)

1. iot_controller connects to the peripheral.
2. Subscribe to notifications on `0000FFF1-0000-1000-8000-00805F9B34FB`.
3. Write the 8-byte query command to `0000FFF2-0000-1000-8000-00805F9B34FB`.
4. Receive notify payload (62 or 79 bytes).
5. Validate CRC16 (MODBUS, low byte first).
6. Parse target fields (big-endian for U16/I16).
7. Construct a TTS string and publish to `/audio/tts_play`.

## Response Format (Relevant Fields)

From the documentation “参数查询”:

- TCU address: U8
- Work mode: U16 (big-endian)
- Fault code: U16 (big-endian, bitmask)
- Target angle: I16 / 10
- Actual angle: I16 / 10
- Longitude: I16 / 100
- Latitude: I16 / 100
- Timezone: I8

## CRC Validation

- Compute CRC16 (MODBUS) over all bytes **except** the last two CRC bytes.
- Expect CRC low byte then high byte at the end.
- If CRC fails, log and speak “参数查询失败”.

## Fault Code Parsing

Parse bitmask to Chinese phrases (examples, full list per doc):

- BIT0: 主从倾角差异
- BIT2: 电机损坏
- BIT3: 倾角故障
- BIT4: 电机过流
- BIT5: 东限角警报
- BIT6: 西限角警报
- BIT7: RTC故障
- BIT8: 电量有限警报
- BIT9: 低电量警报
- BIT10: 开关电源损坏
- BIT14: 无线模块故障
- BIT15: 通信故障

If no faults, speak “无故障”.

## TTS Output (Example)

“目标角度 12.3 度，实际角度 12.1 度，经度 116.38，纬度 39.90，时区 8，工作模式 0x0020，故障：无故障。”

## Logging

- Raw notify hex payload
- CRC validation result
- Parsed numeric values
- Final TTS sentence

## Open Questions

- Exact notify timing and whether multiple frames are emitted; initial implementation expects first valid frame.

