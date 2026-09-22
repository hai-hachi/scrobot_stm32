# SCROBOT STM32 Firmware

Low-level firmware for the SCROBOT badminton shuttlecock collection robot.

This repository contains the firmware for the STM32F411CEU6 (Black Pill) that handles motor actuation, encoder feedback, low-level velocity control, safety, and communication with the high-level ROS 2 computer.

## Current platform

- MCU: STM32F411CEU6
- MCU clock: 100 MHz
- Framework: STM32 HAL
- Development environment: STM32CubeIDE / STM32CubeMX
- STM32CubeF4 package: FW_F4 V1.28.3
- UART: USART6, 1,000,000 baud, 8-N-1
- Control loop: TIM10, 100 Hz (10 ms)
- PWM period: ARR = 4999 for the motor PWM timers used by the application

## Controlled motors

| ID | Function | Calibrated CPR | Max reference | Nominal working speed |
| --- | --- | ---: | ---: | ---: |
| WR | Right drive wheel | 3264 | 100 RPM | 95 RPM |
| WL | Left drive wheel | 3264 | 100 RPM | 95 RPM |
| BR | Right brush / collection motor | 400 | 400 RPM | 390 RPM |
| BL | Left brush / collection motor | 400 | 400 RPM | 390 RPM |
| CV | Conveyor motor | 3960 | 80 RPM | 80 RPM |

## Main firmware functions

- PWM motor control
- Quadrature encoder acquisition
- M/T-based motor-speed estimation
- PIDF speed control
- DMA-based UART RX/TX
- CRC-16/CCITT-FALSE framed communication
- Communication watchdog
- Emergency-stop handling
- Open-loop system-identification mode
- Closed-loop PID test mode

## Repository layout

```text
scrobot_stm32/
├── Core/
│   ├── Inc/
│   │   ├── app.h
│   │   ├── app_config.h
│   │   └── ...
│   ├── Src/
│   │   ├── app.c
│   │   ├── main.c
│   │   └── ...
│   └── Startup/
├── Drivers/
├── docs/
│   ├── hardware.md
│   ├── pinout.md
│   ├── protocol.md
│   └── system_identification.md
├── cube2.ioc
├── STM32F411CEUX_FLASH.ld
└── STM32F411CEUX_RAM.ld
```

## Configuration

Application-level constants are in:

```text
Core/Inc/app_config.h
```

Important values include:

- control-loop period
- communication watchdog timeout
- tuning watchdog timeout
- encoder counts per revolution
- encoder direction
- motor direction
- initial PID gains
- UART baud rate

### Calibration note

The current encoder CPR values are calibrated output-shaft counts per revolution and are shared by the firmware documentation and host-side encoder test. If the installed motor/gearbox/encoder hardware changes, recalibrate CPR before repeating system identification or PID tuning.

## Communication

The STM32 communicates with the Raspberry Pi / ROS 2 computer using USART6 at 1 Mbaud.

Frames use protocol v2:

```text
AA 55 | VER | TYPE | SEQ (u16 LE) | LEN | PAYLOAD | CRC16 (u16 LE)
```

The CRC is CRC-16/CCITT-FALSE.

See [docs/protocol.md](docs/protocol.md) for the current protocol.

## System identification and PID tuning

The host tools support the complete calibration workflow:

1. encoder/filter sanity check
2. bidirectional duty sweep for deadband/hysteresis
3. multistep open-loop data capture
4. continuous/discrete transfer-function identification in MATLAB
5. PIDF tuning using the STM32-matched 100 Hz trapezoidal controller form
6. runtime PIDF upload over UART
7. closed-loop validation at the nominal working speeds

See:

- [System identification notes](docs/system_identification.md)
- [Complete tuning command workflow](docs/tuning_workflow.md)
- [Host tools](tools/README.md)

## Building

Open the project in STM32CubeIDE or import it as an existing STM32CubeIDE project.

The CubeMX configuration is stored in:

```text
cube2.ioc
```

Generated build directories such as `Debug/` and `Release/` are intentionally excluded from version control.

## Related repository

High-level ROS 2 control, navigation, localization, perception, and mission logic are maintained separately in:

- `hai-hachi/scrobot`

## Safety

Before running motors on the robot:

1. Verify E-stop polarity and operation.
2. Verify motor direction with the robot lifted or mechanically secured.
3. Verify encoder direction and CPR.
4. Confirm the communication watchdog stops the drive motors.
5. Start system-identification tests with low duty cycles and one motor at a time.
