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

Expected counts per output-shaft revolution:

| Motor | Counts/rev |
|---|---:|
| WR | 3468 |
| WL | 3468 |
| BR | 422.4 |
| BL | 422.4 |
| CV | 440 |

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
