# UART protocol

The low-level controller communicates with the high-level computer over USART6.

## Link configuration

- Baud rate: 1,000,000 bit/s
- Data: 8 bits
- Parity: none
- Stop bits: 1
- Hardware flow control: none
- Byte order for multibyte values: little-endian
- Float format: IEEE-754 binary32

## Frame format

```text
SOF1 | SOF2 | TYPE | PAYLOAD | CRC8
 AA     55
```

CRC-8 parameters:

- polynomial: `0x07`
- init: `0x00`
- refin: false
- refout: false
- xorout: `0x00`
- CRC coverage: `TYPE + PAYLOAD`
- `SOF1` and `SOF2` are not included in the CRC

## Normal operation

### Pi -> STM32

| Type | Payload | Total frame |
| --- | --- | ---: |
| `A0` | WR_ref, WL_ref | 12 bytes |
| `A1` | BR_ref, BL_ref, CV_ref | 16 bytes |

### STM32 -> Pi

Sent on the 100 Hz TIM10 control tick in normal mode.

| Type | Meaning | Payload | Total frame |
| --- | --- | --- | ---: |
| `01` | Normal | WR, WL, BR, BL, CV measured RPM | 24 bytes |
| `00` | E-stop active | WR, WL, BR, BL, CV measured RPM | 24 bytes |

Normal telemetry is suppressed while temporary F1/F3 tuning mode is active.

## PID parameter update

### Pi -> STM32

| Type | Motor | Payload |
| --- | --- | --- |
| `BA` | WR | Kp, Ki, Kd, Tf |
| `BB` | WL | Kp, Ki, Kd, Tf |
| `B0` | BR | Kp, Ki, Kd, Tf |
| `B1` | BL | Kp, Ki, Kd, Tf |
| `B2` | CV | Kp, Ki, Kd, Tf |

Each frame is 20 bytes total.

### STM32 -> Pi echo

| Type | Motor |
| --- | --- |
| `CA` | WR |
| `CB` | WL |
| `C0` | BR |
| `C1` | BL |
| `C2` | CV |

The echo payload contains Kp, Ki, Kd, Tf.

## Temporary tuning protocol

### F1 - open-loop system identification

Payload:

```text
SEQ(uint16) + WR_duty + WL_duty + BR_duty + BL_duty + CV_duty
```

Each duty is a float normalized to:

```text
-1.0 ... +1.0
```

Receiving F1 enters open-loop SYSID mode.

Total frame length: 26 bytes.

### F2 - synchronized measurement

Payload:

```text
SEQ(uint16) + WR_rpm + WL_rpm + BR_rpm + BL_rpm + CV_rpm
```

The STM32 returns F2 one control interval after the corresponding F1/F3 command is applied at a TIM10 boundary.

This sequence association is intended to provide deterministic command/measurement pairing for system identification and controller testing.

Total frame length: 26 bytes.

### F3 - closed-loop PID test

Payload:

```text
SEQ(uint16) + WR_ref + WL_ref + BR_ref + BL_ref + CV_ref
```

Values are RPM references.

Receiving F3 enters closed-loop PID-test mode.

Total frame length: 26 bytes.

### F0 - stop tuning

F0 has no payload.

Behavior:

- stop all motors
- reset PID state
- return to normal mode

Total frame length: 4 bytes.

## Timeouts

Current application configuration:

- normal drive command heartbeat timeout: 200 ms
- tuning watchdog timeout: 20 s

These values are defined in `Core/Inc/app_config.h`.
