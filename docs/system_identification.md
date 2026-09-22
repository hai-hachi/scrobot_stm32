# System identification and PIDF tuning

System identification uses UART protocol v2 and the dedicated one-motor SYSID
mode. Acquisition runs on the Raspberry Pi so Wi-Fi/SSH timing does not enter
the 100 Hz control/sample loop.

## Architecture

```text
Laptop / PC
  MATLAB + file storage
        |
       SCP / SSH
        |
        v
Raspberry Pi 4B
  Python host tools
        |
  UART 1 Mbaud
        |
        v
STM32F411CEU6
  100 Hz control loop
```

## Current calibrated motor data

| Motor | Encoder CPR | Max closed-loop reference | Nominal working speed |
|---|---:|---:|---:|
| WR | 3264 | 100 RPM | 95 RPM |
| WL | 3264 | 100 RPM | 95 RPM |
| BR | 400 | 400 RPM | 390 RPM |
| BL | 400 | 400 RPM | 390 RPM |
| CV | 3960 | 80 RPM | 80 RPM |

These values must stay aligned with `Core/Inc/app_config.h`.

## Speed-estimation filtering

- WR/WL use the M/T speed estimator.
- BR/BL use timer encoder input filtering plus the hybrid count-window/M/T
  estimator and a 10 Hz RPM low-pass filter.
- CV uses software quadrature deglitching, the hybrid count-window/M/T
  estimator, and a 7 Hz RPM low-pass filter.
- SYSID and normal feedback report the same final RPM signal used by PIDF.

If the estimator/filter configuration changes, repeat the multistep
identification and PIDF tuning because the measured plant has changed.

## Recommended sequence

1. Verify E-stop, motor direction, encoder sign, and calibrated CPR.
2. Run a low-duty fixed motor test.
3. Run a bidirectional sweep to characterize deadband, hysteresis, and
   saturation.
4. Run the bidirectional multistep experiment for transfer-function
   identification.
5. Copy the CSV files from the Pi to the PC.
6. Identify continuous/discrete 1P0Z and 2P1Z models in MATLAB.
7. Tune PIDF using the selected/overridden model.
8. Copy `tools/raw_data/pidf_autotune_results.csv` to the Pi and load it with
   `pid_update.py`.
9. Validate each motor at its nominal working speed with `pid_validate.py`.
10. Copy validation CSV files back to the PC and inspect them with
    `pidf_validation_all.m`.
11. After hardware validation, copy the accepted PIDF values into
    `app_config.h` so they become the reset defaults.

The STM32 boots DISARMED. SYSID requires ARM. A lost SYSID heartbeat for
500 ms stops the experiment and disarms the controller.

## Identification input/output

The identified SISO plant is:

```text
input  = PWM duty command [%]
output = measured motor speed [RPM]
Ts     = 0.01 s
```

CSV columns are:

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

The STM32 `control_tick` is the preferred identification timebase.

## PIDF implementation match

The STM32 controller uses a 10 ms sample period.

The integral state is trapezoidal:

```text
I[k] = I[k-1] + Ts/2 * (e[k] + e[k-1])
```

The filtered derivative uses the corresponding bilinear/Tustin form:

```text
D[k] = ad*D[k-1] + bd*(e[k] - e[k-1])
ad = (2*Tf - Ts)/(2*Tf + Ts)
bd = 2*Kd/(2*Tf + Ts)
```

The MATLAB autotune script therefore uses a discrete PIDF template with
`IFormula='Trapezoidal'` and `DFormula='Trapezoidal'`. If a continuous
identified plant is selected, the current tuning workflow discretizes that
plant with the bilinear/Tustin transform at 0.01 s before tuning.

For the exact commands used at every stage, including PC/Pi SCP commands, see
[tuning_workflow.md](tuning_workflow.md).
