#ifndef APP_H
#define APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

void App_Init(void);
void App_Task(void);

/* Called from USER CODE blocks at the top of hardware-encoder IRQ handlers. */
void App_EncoderEdgeIRQ(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif

#endif /* APP_H */
