# Motor system-identification and PIDF tuning command workflow

This page is the command checklist for repeating motor characterization,
system identification, PIDF tuning, and closed-loop validation.

Assumptions:

- Raspberry Pi user/host: `scrobot@192.168.1.14`
- Pi repository: `~/scrobot_stm32`
- Pi UART: `/dev/ttyAMA0`
- Python commands are run from `~/scrobot_stm32/tools`
- PC commands below use Windows PowerShell from the local
  `scrobot_stm32` repository root.
- Local experiment files are kept under `tools/raw_data/`.

Only one host program should own `/dev/ttyAMA0` at a time.

## 0. Update both machines

### Raspberry Pi

```bash
cd ~/scrobot_stm32
git switch firmware-safety-prep
git pull

cd tools
source .venv/bin/activate
mkdir -p data raw_data
```

### PC

```powershell
git switch firmware-safety-prep
git pull

New-Item -ItemType Directory -Force .\tools\raw_data\bidir | Out-Null
New-Item -ItemType Directory -Force .\tools\raw_data\multistep | Out-Null
New-Item -ItemType Directory -Force .\tools\raw_data\validation | Out-Null
```

## 1. Encoder and filter sanity check

Run on the Pi before any open-loop test:

```bash
cd ~/scrobot_stm32/tools
source .venv/bin/activate

python encoder_test.py --port /dev/ttyAMA0
```

Expected output-shaft CPR:

```text
WR = 3264
WL = 3264
BR = 400
BL = 400
CV = 3960
```

Then verify motor/encoder sign with a low fixed duty, one motor at a time:

```bash
python motor_test.py --port /dev/ttyAMA0 --motor WR --duty 0.10 --duration 5
python motor_test.py --port /dev/ttyAMA0 --motor WL --duty 0.10 --duration 5
python motor_test.py --port /dev/ttyAMA0 --motor BR --duty 0.10 --duration 5
python motor_test.py --port /dev/ttyAMA0 --motor BL --duty 0.10 --duration 5
python motor_test.py --port /dev/ttyAMA0 --motor CV --duty 0.10 --duration 5
```

For BR/BL/CV filter debugging with STM32CubeIDE, compare:

```text
g_app_debug.rpm_raw_br  vs g_app_debug.rpm_br
g_app_debug.rpm_raw_bl  vs g_app_debug.rpm_bl
g_app_debug.rpm_raw_cv  vs g_app_debug.rpm_cv
g_app_debug.cv_deglitch_rejects
```

This stage does not create a CSV by default.

## 2. Bidirectional sweep: deadband / hysteresis / saturation

Run on the Pi:

```bash
cd ~/scrobot_stm32/tools
source .venv/bin/activate

python sysid.py --port /dev/ttyAMA0 bidir-sweep --motor WR --max 1.00 --step 0.01 --hold 1.0
python sysid.py --port /dev/ttyAMA0 bidir-sweep --motor WL --max 1.00 --step 0.01 --hold 1.0
python sysid.py --port /dev/ttyAMA0 bidir-sweep --motor BR --max 1.00 --step 0.01 --hold 1.0
python sysid.py --port /dev/ttyAMA0 bidir-sweep --motor BL --max 1.00 --step 0.01 --hold 1.0
python sysid.py --port /dev/ttyAMA0 bidir-sweep --motor CV --max 1.00 --step 0.01 --hold 1.0
```

Files are written to `tools/data/*_bidir_sweep_*.csv`.

### Pi -> PC after bidirectional sweep

Run from the PC repository root:

```powershell
scp scrobot@192.168.1.14:~/scrobot_stm32/tools/data/*_bidir_sweep_*.csv .\tools\raw_data\bidir\
```

### PC -> Pi if restoring/copying bidirectional data

```powershell
scp .\tools\raw_data\bidir\*_bidir_sweep_*.csv scrobot@192.168.1.14:~/scrobot_stm32/tools/data/
```

Use the bidirectional characterization MATLAB script to calculate the measured
deadband, saturation point, and RPM corresponding to chosen duty levels.

## 3. Multistep acquisition for transfer-function identification

Current tested multistep settings:

- start = 20% duty
- increment = 5% duty
- hold = 2 s
- WR/WL max = 80%
- BR max = 70%
- BL max = 90%
- CV max = 100%

Run on the Pi:

```bash
cd ~/scrobot_stm32/tools
source .venv/bin/activate

START=0.20
SIZE=0.05
HOLD=2.0

python sysid.py --port /dev/ttyAMA0 multistep --motor WR --start $START --size $SIZE --max 0.80 --hold $HOLD
python sysid.py --port /dev/ttyAMA0 multistep --motor WL --start $START --size $SIZE --max 0.80 --hold $HOLD
python sysid.py --port /dev/ttyAMA0 multistep --motor BR --start $START --size $SIZE --max 0.70 --hold $HOLD
python sysid.py --port /dev/ttyAMA0 multistep --motor BL --start $START --size $SIZE --max 0.90 --hold $HOLD
python sysid.py --port /dev/ttyAMA0 multistep --motor CV --start $START --size $SIZE --max 1.00 --hold $HOLD
```

### Pi -> PC after multistep acquisition

```powershell
scp scrobot@192.168.1.14:~/scrobot_stm32/tools/data/*_multistep_*.csv .\tools\raw_data\multistep\
```

### PC -> Pi if restoring/copying multistep data

```powershell
scp .\tools\raw_data\multistep\*_multistep_*.csv scrobot@192.168.1.14:~/scrobot_stm32/tools/data/
```

## 4. MATLAB system identification

Use `Ts = 0.01 s`.

Compare these candidates for each motor:

```text
Continuous 1P0Z
Continuous 2P1Z
Discrete   1P0Z
Discrete   2P1Z
```

Use the validation fit to identify the automatic best model. The PIDF autotune
script also supports an explicit per-motor model override:

```matlab
modelOverride_all = [
    "AUTO";      % WR
    "AUTO";      % WL
    "AUTO";      % BR
    "AUTO";      % BL
    "AUTO"       % CV
];
```

Allowed override values are `AUTO`, `C_1P0Z`, `C_2P1Z`, `D_1P0Z`, and
`D_2P1Z`.

## 5. MATLAB PIDF autotune

Run:

```text
tools/matlab/pidf_autotune_all.m
```

The script:

1. finds the latest multistep CSV for each motor below `tools/`
2. identifies continuous/discrete 1P0Z and 2P1Z candidates
3. applies the per-motor model override if configured
4. converts a selected continuous plant to 100 Hz with bilinear/Tustin
5. tunes a 100 Hz discrete PIDF with trapezoidal I/D formulas
6. searches crossover frequency/phase margin against the configured per-motor
   settling-time and overshoot targets
7. scales duty-percent controller gains into STM32 PWM-count gains
8. writes the result CSV to:

```text
tools/raw_data/pidf_autotune_results.csv
```

## 6. Copy tuned PIDF CSV from PC to Pi

First ensure the destination exists:

```bash
mkdir -p ~/scrobot_stm32/tools/raw_data
```

Then from the PC repository root:

```powershell
scp .\tools\raw_data\pidf_autotune_results.csv scrobot@192.168.1.14:~/scrobot_stm32/tools/raw_data/
```

Reverse direction, if a copy on the Pi needs to be recovered to the PC:

```powershell
scp scrobot@192.168.1.14:~/scrobot_stm32/tools/raw_data/pidf_autotune_results.csv .\tools\raw_data\
```

## 7. Load and verify PIDF gains on the STM32

Run on the Pi:

```bash
cd ~/scrobot_stm32/tools
source .venv/bin/activate

python pid_update.py --port /dev/ttyAMA0 load-csv --file raw_data/pidf_autotune_results.csv
python pid_update.py --port /dev/ttyAMA0 get-all
```

UART PIDF updates are runtime-only. An STM32 reset restores the defaults from
`Core/Inc/app_config.h`.

## 8. Closed-loop validation at nominal working RPM

Current nominal validation steps:

```text
WR = 95 RPM
WL = 95 RPM
BR = 390 RPM
BL = 390 RPM
CV = 80 RPM
```

`pid_validate.py` uses these automatically when `--rpm` is omitted.

Run on the Pi:

```bash
cd ~/scrobot_stm32/tools
source .venv/bin/activate

python pid_validate.py --port /dev/ttyAMA0 --motor WR --pre 2 --duration 5 --post 2
python pid_validate.py --port /dev/ttyAMA0 --motor WL --pre 2 --duration 5 --post 2
python pid_validate.py --port /dev/ttyAMA0 --motor BR --pre 2 --duration 5 --post 2
python pid_validate.py --port /dev/ttyAMA0 --motor BL --pre 2 --duration 5 --post 2
python pid_validate.py --port /dev/ttyAMA0 --motor CV --pre 2 --duration 5 --post 2
```

Use `--rpm VALUE` only for an intentional non-nominal validation test.

### Pi -> PC after PIDF validation

```powershell
scp scrobot@192.168.1.14:~/scrobot_stm32/tools/data/*_pid_validation_*.csv .\tools\raw_data\validation\
```

### PC -> Pi if restoring/copying validation data

```powershell
scp .\tools\raw_data\validation\*_pid_validation_*.csv scrobot@192.168.1.14:~/scrobot_stm32/tools/data/
```

Analyze the latest validation files with:

```text
tools/matlab/pidf_validation_all.m
```

## 9. Persist accepted gains in firmware

After the real motor validation is accepted:

1. copy the final `Kp_STM32`, `Ki_STM32`, `Kd_STM32`, and `Tf_STM32_s`
   values into the corresponding `APP_PID_*_KP/KI/KD/TF` definitions in
   `Core/Inc/app_config.h`
2. rebuild and flash the STM32
3. run `pid_update.py get-all` after reset to verify the compiled defaults
4. rerun the nominal validation if the compiled defaults changed

## 10. Current closed-loop reference limits

```text
WR = +/-100 RPM
WL = +/-100 RPM
BR = +/-400 RPM
BL = +/-400 RPM
CV = +/-80 RPM
```

The host tools and firmware must use the same limits.
