# Host tools

These utilities talk directly to the STM32 UART v2 protocol. They are intended to run on the Raspberry Pi during bring-up and motor system identification, outside ROS 2.

## Install

```bash
cd tools
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

On Windows:

```powershell
cd tools
py -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
```

## Monitor

```bash
python serial_monitor.py --port /dev/ttyAMA0
```

## Encoder check

```bash
python encoder_test.py --port /dev/ttyAMA0
```

Expected calibrated counts per output-shaft revolution:

| Motor | Counts/rev |
|---|---:|
| WR | 3264 |
| WL | 3264 |
| BR | 400 |
| BL | 400 |
| CV | 3960 |

## Fixed-duty motor test

```bash
python motor_test.py --port /dev/ttyAMA0 --motor WR --duty 0.10 --duration 5
```

The script arms the STM32, continuously refreshes the SYSID command, then sends SYSID_STOP and DISARM on exit.

## Step test

```bash
python sysid.py --port /dev/ttyAMA0 step --motor WR --duty 0.25 --pre 2 --duration 5 --post 2
```

## Duty sweep

```bash
python sysid.py --port /dev/ttyAMA0 sweep --motor WR --start 0 --stop 0.5 --step 0.025 --hold 1.5
```

CSV files are written under `tools/data/` unless `--output` is supplied.

The Python tools should ultimately run on the Pi so Wi-Fi/SSH timing does not enter the motor identification loop. MATLAB can analyze the resulting CSV afterward.

## Closed-loop PID test

```bash
python pid_test.py --port /dev/ttyAMA0 --motor WR --rpm 100 --kp 1.0 --ki 2.0 --kd 0 --tf 0.01
```

The script writes PID gains, arms the controller, sends a 50 Hz unified setpoint heartbeat, logs 100 Hz feedback, then commands zero and disarms.


## Bidirectional deadband / hysteresis sweep

This performs:

```text
0 -> +max -> 0 -> -max -> 0
```

Example:

```bash
python sysid.py --port /dev/ttyAMA0 bidir-sweep --motor WR --max 0.30 --step 0.01 --hold 1.0
```

Use this mode to capture forward/reverse deadband and hysteresis in a single CSV.


## PIDF auto-tune and runtime update

MATLAB source:

```text
tools/matlab/pidf_autotune_all.m
```

Open it in MATLAB Live Editor (or copy it into a new Live Script and save as
`.mlx`). It uses the latest `*_multistep_*.csv` for each motor, estimates continuous
and discrete 1P0Z / 2P1Z models, supports per-motor model override, tunes a
100 Hz discrete PIDF that matches the STM32 trapezoidal implementation, and
saves:

```text
raw_data/pidf_autotune_results.csv
```

The CSV contains both MATLAB gains in percent-duty units and gains scaled for
the STM32 PWM range (ARR=4999).

Runtime PIDF updater:

```bash
python pid_update.py --port /dev/ttyAMA0 get-all
python pid_update.py --port /dev/ttyAMA0 set --motor WR --kp 10 --ki 20 --kd 0.1 --tf 0.01
python pid_update.py --port /dev/ttyAMA0 load-csv --file raw_data/pidf_autotune_results.csv
```

UART PIDF updates are volatile and return to the values in `app_config.h`
after an STM32 reset or power cycle.


## Current speed limits and validation references

| Motor | Max reference | Nominal validation step |
|---|---:|---:|
| WR | 100 RPM | 95 RPM |
| WL | 100 RPM | 95 RPM |
| BR | 400 RPM | 390 RPM |
| BL | 400 RPM | 390 RPM |
| CV | 80 RPM | 80 RPM |

`pid_validate.py` uses the nominal value automatically when `--rpm` is omitted:

```bash
python pid_validate.py --port /dev/ttyAMA0 --motor WR
python pid_validate.py --port /dev/ttyAMA0 --motor WL
python pid_validate.py --port /dev/ttyAMA0 --motor BR
python pid_validate.py --port /dev/ttyAMA0 --motor BL
python pid_validate.py --port /dev/ttyAMA0 --motor CV
```

For the complete acquisition, SCP, MATLAB, tuning, upload, and validation
sequence, see [../docs/tuning_workflow.md](../docs/tuning_workflow.md).
