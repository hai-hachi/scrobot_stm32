# Hardware / timer configuration

This page summarizes the current STM32CubeMX configuration relevant to low-level control.

## MCU

- STM32F411CEU6
- Cortex-M4
- SYSCLK: 100 MHz
- HCLK: 100 MHz
- APB1: 50 MHz
- APB1 timer clock: 100 MHz
- APB2: 100 MHz
- APB2 timer clock: 100 MHz

## Control timer

TIM10 is used as the application control-loop timer.

Current configuration:

- Prescaler: 99
- Period: 9999
- Timer input clock: 100 MHz

This produces a 100 Hz update rate:

```text
100 MHz / (99 + 1) / (9999 + 1) = 100 Hz
```

Therefore:

```text
Ts = 0.01 s
```

## PWM timers

The current motor PWM timers use an ARR value of 4999.

Relevant timers:

- TIM3: WR/WL drive PWM
- TIM9: BR/BL PWM
- TIM11: CV PWM

With a 100 MHz timer clock and prescaler 0:

```text
f_PWM = 100 MHz / (4999 + 1) = 20 kHz
```

## Encoder timers

- TIM2: WR quadrature encoder
- TIM5: WL quadrature encoder
- TIM1: BR quadrature encoder
- TIM4: BL quadrature encoder
- CV encoder: software quadrature using EXTI on PB8/PB10

## UART

USART6:

- 1 Mbaud
- RX DMA: DMA2 Stream1
- TX DMA: DMA2 Stream6

## Configuration source of truth

The CubeMX hardware configuration is stored in:

```text
cube2.ioc
```

When changing timers, pin mappings, DMA, clock configuration, or UART settings, update the CubeMX configuration and verify that the application-level constants still match.


## Motor calibration and operating speeds

Current calibrated output-shaft encoder constants and closed-loop references:

| Motor | Encoder CPR | Max accepted RPM | Nominal working RPM |
|---|---:|---:|---:|
| WR | 3264 | 100 | 95 |
| WL | 3264 | 100 | 95 |
| BR | 400 | 400 | 390 |
| BL | 400 | 400 | 390 |
| CV | 3960 | 80 | 80 |

The firmware values in `Core/Inc/app_config.h` are the source of truth for
runtime limits. Re-run encoder calibration, system identification, and PIDF
tuning if the motor, gearbox, encoder, or speed-estimator configuration changes.
