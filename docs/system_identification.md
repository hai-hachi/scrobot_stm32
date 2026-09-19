# System identification

System identification uses UART protocol v2 and the dedicated one-motor SYSID mode.

## Architecture

```text
Laptop (SSH / MATLAB)
        |
        v
Raspberry Pi
  Python SYSID tool
        |
   UART 1 Mbaud
        |
        v
STM32F411
  100 Hz control/sample timing
```

Run the Python acquisition on the Pi. Wi-Fi timing then does not affect the motor command/sample loop.

## Encoder constants

The firmware assumes x4 quadrature decoding:

- WR/WL: 17 PPR x 51 x 4 = 3468 counts/output revolution
- BR/BL: 11 PPR x 9.6 x 4 = 422.4 counts/output revolution
- CV: 11 PPR x 10 x 4 = 440 counts/output revolution

Verify these values by physically rotating each output shaft before final identification.

## Safety sequence

1. Jack up / mechanically secure the robot.
2. Verify E-stop.
3. Run `encoder_test.py`.
4. Verify positive duty direction and encoder sign at low duty.
5. Run deadband sweep.
6. Run open-loop step tests.
7. Fit the motor model in MATLAB.
8. Tune PID.
9. Validate closed-loop speed response.

The STM32 boots DISARMED. SYSID requires ARM. A lost SYSID heartbeat for 500 ms stops the experiment and disarms the controller.

## Python tools

From `tools/`:

```bash
python motor_test.py --port /dev/ttyAMA0 --motor WR --duty 0.10
```

Step test:

```bash
python sysid.py --port /dev/ttyAMA0 step --motor WR --duty 0.25 --pre 2 --duration 5 --post 2
```

Sweep:

```bash
python sysid.py --port /dev/ttyAMA0 sweep --motor WR --start 0 --stop 0.5 --step 0.025 --hold 1.5
```

CSV columns:

```text
host_time_s
control_tick
frame_seq
command_seq
motor
command_duty
rpm
encoder_count
status
```

Use duty cycle as the identification input. The initial project assumption is a nominal 12 V motor supply.

## MATLAB

A CSV can be loaded directly:

```matlab
T = readtable("WR_step_p0.250_YYYYMMDD_HHMMSS.csv");

u = T.command_duty;
y = T.rpm;
t = T.control_tick * 0.01;
```

The 100 Hz STM32 control tick should be preferred as the identification timebase.
