#ifndef APP_H
#define APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

enum
{
    APP_STATUS_ARMED              = (1UL << 0),
    APP_STATUS_ESTOP              = (1UL << 1),
    APP_STATUS_COMM_TIMEOUT       = (1UL << 2),
    APP_STATUS_SYSID_MODE         = (1UL << 3),
    APP_STATUS_UART_ERROR_SEEN    = (1UL << 4),
    APP_STATUS_INVALID_OUTPUT     = (1UL << 5),
    APP_STATUS_TX_QUEUE_DROP_SEEN = (1UL << 6),
    APP_STATUS_INVALID_COMMAND    = (1UL << 7)
};

/*
 * Live Expression / debugger snapshot.
 *
 * This is deliberately not a control-command interface. It exposes low-level
 * state for fast bring-up without requiring the UART host.
 */
typedef struct
{
    volatile uint32_t control_tick;
    volatile uint32_t main_loop_count;
    volatile uint32_t status_flags;
    volatile uint32_t reset_flags_raw;
    volatile uint32_t watchdog_started;
    volatile uint32_t watchdog_update_timeout;
    volatile uint16_t last_setpoint_seq;
    volatile uint16_t sysid_command_seq;
    volatile uint8_t sysid_motor_id;
    volatile float sysid_duty;

    volatile int32_t encoder_count_wr;
    volatile int32_t encoder_count_wl;
    volatile int32_t encoder_count_br;
    volatile int32_t encoder_count_bl;
    volatile int32_t encoder_count_cv;

    /* CV EXTI transitions rejected as invalid quadrature or too-close glitches. */
    volatile uint32_t cv_deglitch_rejects;

    volatile float ref_rpm_wr;
    volatile float ref_rpm_wl;
    volatile float ref_rpm_br;
    volatile float ref_rpm_bl;
    volatile float ref_rpm_cv;

    /* RPM used by feedback/PIDF after the configured estimator/filter. */
    volatile float rpm_wr;
    volatile float rpm_wl;
    volatile float rpm_br;
    volatile float rpm_bl;
    volatile float rpm_cv;

    /* Selected raw RPM estimate before the auxiliary BR/BL/CV low-pass filter. */
    volatile float rpm_raw_wr;
    volatile float rpm_raw_wl;
    volatile float rpm_raw_br;
    volatile float rpm_raw_bl;
    volatile float rpm_raw_cv;

    /* Controller/open-loop command after sign and output limiting, PWM counts. */
    volatile float output_wr;
    volatile float output_wl;
    volatile float output_br;
    volatile float output_bl;
    volatile float output_cv;

    volatile uint32_t uart_rx_frames_ok;
    volatile uint32_t uart_crc_errors;
    volatile uint32_t uart_invalid_frames;
    volatile uint32_t uart_errors;
    volatile uint32_t uart_tx_frames_ok;
    volatile uint32_t uart_tx_errors;
    volatile uint32_t uart_tx_queue_drops;
} AppDebug_t;

extern volatile AppDebug_t g_app_debug;

void App_Init(void);
void App_Task(void);

/* Immediate low-level output shutdown for Error_Handler / CPU fault handlers. */
void App_EmergencyShutdown(void);

/* Called from USER CODE blocks at the top of hardware-encoder IRQ handlers. */
void App_EncoderEdgeIRQ(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif

#endif /* APP_H */
