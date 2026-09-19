#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* ---------------- Control loop ---------------- */
#define APP_CONTROL_TS_S                 0.010f      /* TIM10 = 100 Hz */
#define APP_COMM_TIMEOUT_MS              200U        /* WR/WL command heartbeat */
#define APP_TUNING_TIMEOUT_MS            20000U        /* F1/F3 watchdog; timeout returns NORMAL */

/*
 * M/T zero-speed timeout.
 * If no new encoder boundary is observed for this long, measured RPM is set to 0.
 * Tune this later if you need very-low-speed measurement below this threshold.
 */
#define APP_MT_ZERO_TIMEOUT_MS           250U

/* USART protocol target baud rate. App_Init enforces this even if CubeMX still says 115200. */
#define APP_UART_BAUD                    1000000U

/*
 * Encoder counts per mechanical output-shaft revolution.
 * IMPORTANT: use the ACTUAL TIMx->CNT counts/rev in TI12 encoder mode.
 * The values below are placeholders copied from the previous project only.
 */
#define APP_CPR_WR                       3468.0f
#define APP_CPR_WL                       3468.0f
#define APP_CPR_BR                       422.4f
#define APP_CPR_BL                       422.4f
#define APP_CPR_CV                       440.0f

/* Flip to -1.0f if measured RPM sign is opposite to your chosen positive direction. */
#define APP_ENCODER_SIGN_WR              1.0f
#define APP_ENCODER_SIGN_WL              1.0f
#define APP_ENCODER_SIGN_BR              -1.0f
#define APP_ENCODER_SIGN_BL              1.0f
#define APP_ENCODER_SIGN_CV              1.0f

/* Flip to -1.0f if positive PID output rotates the motor in the wrong direction. */
#define APP_MOTOR_SIGN_WR                1.0f
#define APP_MOTOR_SIGN_WL                1.0f
#define APP_MOTOR_SIGN_BR                1.0f
#define APP_MOTOR_SIGN_BL                1.0f
#define APP_MOTOR_SIGN_CV                1.0f

/* ESTOP is currently PA4 input with pull-up, so active-low is the default assumption. */
#define APP_ESTOP_ACTIVE_LOW             0U

/*
 * Initial PIDF values.
 * Kf is fixed at 0.0f in this project. Kp/Ki/Kd/Tf can be replaced at runtime by UART.
 * Initial gains are intentionally zero for a safe first power-up.
 */
#define APP_PID_WR_KP                    0.0f
#define APP_PID_WR_KI                    0.0f
#define APP_PID_WR_KD                    0.0f
#define APP_PID_WR_TF                    0.010f

#define APP_PID_WL_KP                    0.0f
#define APP_PID_WL_KI                    0.0f
#define APP_PID_WL_KD                    0.0f
#define APP_PID_WL_TF                    0.010f

#define APP_PID_BR_KP                    0.0f
#define APP_PID_BR_KI                    0.0f
#define APP_PID_BR_KD                    0.0f
#define APP_PID_BR_TF                    0.010f

#define APP_PID_BL_KP                    0.0f
#define APP_PID_BL_KI                    0.0f
#define APP_PID_BL_KD                    0.0f
#define APP_PID_BL_TF                    0.010f

#define APP_PID_CV_KP                    0.0f
#define APP_PID_CV_KI                    0.0f
#define APP_PID_CV_KD                    0.0f
#define APP_PID_CV_TF                    0.010f

#endif /* APP_CONFIG_H */
