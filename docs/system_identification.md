# System identification workflow

The firmware already contains a temporary open-loop mode intended for identifying the motor dynamics before final PID tuning.

## Preconditions

Before collecting identification data:

1. Verify the E-stop input and polarity.
2. Secure the robot or lift the driven wheel off the ground for initial tests.
3. Verify motor direction.
4. Verify encoder direction.
5. Measure and correct the actual encoder counts per output-shaft revolution.
6. Verify that reported RPM is physically correct.
7. Confirm F0 and communication-loss behavior stop the motor.

Do not use the current CPR constants as final values until they have been measured. The source currently marks them as placeholders.

## Relevant firmware timing

- control period: 10 ms
- control frequency: 100 Hz
- F1/F3 commands are applied at TIM10 boundaries
- F2 is associated with the command using a uint16 sequence number

## F1 open-loop command

F1 applies normalized duty directly:

```text
-1.0 <= duty <= 1.0
```

For initial tests, command only one motor while setting all other motor duties to zero.

Example conceptual command:

```text
SEQ = 25
WR = 0.15
WL = 0
BR = 0
BL = 0
CV = 0
```

The following F2 response with the same sequence number contains the synchronized measured RPM.

## Recommended identification sequence

### 1. Encoder calibration

Rotate one output shaft through a known number of revolutions and determine the actual timer-count change.

Calculate:

```text
CPR = absolute encoder count change / mechanical revolutions
```

Repeat in both directions.

### 2. Direction check

Apply a small positive duty and verify:

- mechanical positive direction
- encoder RPM sign
- configured `APP_MOTOR_SIGN_*`
- configured `APP_ENCODER_SIGN_*`

### 3. Deadband sweep

Increase duty slowly from zero in both directions.

Record:

- command duty
- measured RPM
- first duty where repeatable motion begins

This gives the positive and negative deadband.

### 4. Static duty-speed map

After the deadband is known, test several steady duty values.

For each value:

1. hold duty long enough to approach steady state
2. record measured RPM
3. repeat in both directions

This checks approximate linearity and asymmetry.

### 5. Dynamic step tests

Apply several safe duty steps and log the synchronized F2 data.

Typical first-order model:

```text
G(s) = K / (tau*s + 1)
```

Possible identified quantities:

- deadband
- static gain K
- time constant tau
- transport delay, if significant
- positive/negative asymmetry

### 6. Closed-loop validation

After choosing initial PID gains, use F3 to send RPM references and use F2 to evaluate:

- rise time
- settling time
- overshoot
- steady-state error
- saturation behavior

## Data to log on the Pi

At minimum:

```text
timestamp
sequence
motor
command_mode
command
measured_rpm
```

Useful additional fields:

```text
battery_voltage
estop_state
communication_state
```

## Safety recommendation

Start with low command levels and one wheel at a time. Do not begin with full-duty steps on the assembled mobile robot.
