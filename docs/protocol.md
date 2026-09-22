# UART protocol v2

The Raspberry Pi and STM32 communicate over USART6 at 1,000,000 baud, 8-N-1.

## Frame

```text
AA 55 | VER | TYPE | SEQ (u16 LE) | LEN | PAYLOAD | CRC16 (u16 LE)
```

CRC is CRC-16/CCITT-FALSE:

- polynomial: `0x1021`
- initial value: `0xFFFF`
- refin/refout: false
- xorout: `0x0000`
- CRC covers `VER` through the final payload byte
- SOF and CRC bytes are excluded

Protocol version is currently `2`.

## Motor IDs

| ID | Motor |
|---:|---|
| 0 | WR |
| 1 | WL |
| 2 | BR |
| 3 | BL |
| 4 | CV |

## Pi -> STM32

| Type | Name | Payload |
|---:|---|---|
| 0x10 | SETPOINT | 5 x float RPM: WR, WL, BR, BL, CV |
| 0x11 | ARM | none |
| 0x12 | DISARM | none |
| 0x30 | SYSID_COMMAND | motor_id u8 + duty float |
| 0x32 | SYSID_STOP | none |
| 0x40 | PID_SET | motor_id u8 + Kp, Ki, Kd, Tf |
| 0x41 | PID_GET | motor_id u8 |
| 0x50 | INFO_REQUEST | none |

## STM32 -> Pi

| Type | Name | Behavior |
|---:|---|---|
| 0x20 | FEEDBACK | 100 Hz |
| 0x21 | DIAGNOSTICS | returned with info request |
| 0x31 | SYSID_SAMPLE | 100 Hz while SYSID is active |
| 0x42 | PID_RESPONSE | response to PID_SET/PID_GET |
| 0x51 | INFO_RESPONSE | firmware/protocol version |
| 0x7F | ERROR | request type + error code |

## ARM/DISARM behavior

The STM32 always boots DISARMED.

E-stop press immediately disables outputs and clears the armed state. Releasing E-stop does not automatically re-enable the drivers. A fresh ARM command is required.

If the normal setpoint heartbeat expires, the STM32 disarms. If the SYSID command heartbeat expires, SYSID stops and the STM32 disarms.

## SETPOINT

Payload, little-endian:

```text
float WR_rpm
float WL_rpm
float BR_rpm
float BL_rpm
float CV_rpm
```

Limits:

- WR/WL: ±100 RPM
- BR/BL: ±400 RPM
- CV: ±80 RPM

Nominal working references:

- WR/WL: 95 RPM
- BR/BL: 390 RPM
- CV: 80 RPM

## FEEDBACK

Payload:

```text
u32 control_tick
u32 status
u16 last_setpoint_seq
i32 WR_count
i32 WL_count
i32 BR_count
i32 BL_count
i32 CV_count
f32 WR_rpm
f32 WL_rpm
f32 BR_rpm
f32 BL_rpm
f32 CV_rpm
```

Total payload: 50 bytes.

## Status bits

| Bit | Meaning |
|---:|---|
| 0 | ARMED |
| 1 | ESTOP |
| 2 | COMM_TIMEOUT |
| 3 | SYSID |
| 4 | UART_ERROR seen |
| 5 | INVALID_OUTPUT seen |
| 6 | TX queue drop seen |
| 7 | INVALID_COMMAND seen |

## SYSID_COMMAND

Payload:

```text
u8 motor_id
f32 duty
```

Duty range is -1.0 to +1.0. Only one motor is driven in SYSID mode.

The host must refresh the command faster than the 500 ms tuning timeout.

## SYSID_SAMPLE

Payload:

```text
u16 command_seq
u32 control_tick
u8 motor_id
f32 commanded_duty
f32 measured_rpm
i32 encoder_count
u32 status
```

The packet is emitted at the 100 Hz control rate while SYSID is active.

## DIAGNOSTICS

Payload:

```text
u32 uptime_ms
u32 reset_flags_raw
u32 rx_frames_ok
u32 crc_errors
u32 invalid_frames
u32 uart_errors
u32 tx_queue_drops
u8 fw_major
u8 fw_minor
u8 fw_patch
u8 protocol_version
```

## Error codes

| Code | Meaning |
|---:|---|
| 1 | bad protocol version |
| 2 | bad payload length |
| 3 | bad value / unsupported command |
| 4 | command requires ARMED state |
| 5 | E-stop active |
| 6 | invalid motor ID |
