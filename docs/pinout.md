# STM32 pinout

Pin assignments below are taken from the current `cube2.ioc` configuration.

## Drive wheels

| Signal | Pin | Peripheral |
| --- | --- | --- |
| WL_encA | PA0 | TIM5 CH1 |
| WL_encB | PA1 | TIM5 CH2 |
| WR_encA | PA5 | TIM2 CH1 |
| WR_encB | PB3 | TIM2 CH2 |
| WR_pwmR | PA6 | TIM3 CH1 |
| WR_pwmL | PA7 | TIM3 CH2 |
| WL_pwmR | PB0 | TIM3 CH3 |
| WL_pwmL | PB1 | TIM3 CH4 |
| WR_en | PB4 | GPIO output |
| WL_en | PB5 | GPIO output |

## Auxiliary motors

| Signal | Pin | Peripheral |
| --- | --- | --- |
| BR_pwm | PA2 | TIM9 CH1 |
| BL_pwm | PA3 | TIM9 CH2 |
| BR_encA | PA8 | TIM1 CH1 |
| BR_encB | PA9 | TIM1 CH2 |
| BL_encA | PB6 | TIM4 CH1 |
| BL_encB | PB7 | TIM4 CH2 |
| BR_in1 | PB12 | GPIO output |
| BR_in2 | PB13 | GPIO output |
| BL_in1 | PB14 | GPIO output |
| BL_in2 | PB15 | GPIO output |
| STBY | PB2 | GPIO output |

## Conveyor

| Signal | Pin | Peripheral |
| --- | --- | --- |
| CV_pwm | PB9 | TIM11 CH1 |
| CV_encA | PB8 | EXTI |
| CV_encB | PB10 | EXTI |
| CV_in1 | PA10 | GPIO output |
| CV_in2 | PA15 | GPIO output |

## Communication

| Signal | Pin | Peripheral |
| --- | --- | --- |
| TX | PA11 | USART6 TX |
| RX | PA12 | USART6 RX |

Current UART configuration:

- 1,000,000 baud
- 8 data bits
- no parity
- 1 stop bit
- DMA RX: DMA2 Stream1
- DMA TX: DMA2 Stream6

## Safety

| Signal | Pin | Configuration |
| --- | --- | --- |
| ESTOP | PA4 | GPIO input with pull-up |

The application-level E-stop polarity must match the physical wiring before motor testing.

## Debug

| Signal | Pin |
| --- | --- |
| SWDIO | PA13 |
| SWCLK | PA14 |

The current project uses Serial Wire Debug.
