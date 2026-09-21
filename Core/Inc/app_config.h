#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* ---------------- Firmware / protocol ---------------- */
#define APP_FW_VERSION_MAJOR              0U
#define APP_FW_VERSION_MINOR              2U
#define APP_FW_VERSION_PATCH              0U
#define APP_PROTOCOL_VERSION              2U

/* ---------------- Control loop ---------------- */
#define APP_CONTROL_TS_S                  0.010f      /* TIM10 = 100 Hz */
#define APP_COMM_TIMEOUT_MS               200U        /* A0 drive-command heartbeat */

/*
 * F1/F3 commands must be refreshed while tuning is active.
 * A short timeout prevents a lost host connection from leaving an open-loop
 * duty or PID-test reference active for a long time.
 */
#define APP_TUNING_TIMEOUT_MS             500U

/* Independent watchdog. Refreshed only when the TIM10 control tick advances. */
#define APP_WATCHDOG_TIMEOUT_MS           500U

/*
 * M/T zero-speed timeout.
 * At present one common value is used for all motors. This can be separated
 * per motor later if low-speed testing shows a need.
 */
#define APP_MT_ZERO_TIMEOUT_MS            250U

/*
 * Auxiliary encoder input filtering.
 *
 * BR/BL use STM32 timer encoder mode. Filter 15 is the strongest timer
 * digital input filter and rejects short glitches while remaining far faster
 * than the expected encoder edge spacing.
 *
 * CV is decoded in software from EXTI edges. Ignore valid-looking quadrature
 * transitions that occur less than 25 us after the previous accepted edge.
 * This does not depend on the exact CV PPR/gear ratio and can be tuned later.
 */
#define APP_AUX_ENCODER_TIM_FILTER         15U
#define APP_CV_MIN_EDGE_US                 25U

/* USART6 protocol target baud rate. */
#define APP_UART_BAUD                     1000000U

/*
 * Encoder counts per mechanical OUTPUT-shaft revolution.
 *
 * The project uses x4 quadrature counting:
 *   counts/rev = encoder_PPR * gearbox_ratio * 4
 *
 * WR/WL: 17 PPR * 51:1 * 4 = 3468 counts/rev
 * BR/BL: 11 PPR * 9.6:1 * 4 = 422.4 counts/rev
 * CV:    11 PPR * 10:1 * 4 = 440 counts/rev
 *
 * Verify the real count change over one output-shaft revolution before final
 * system identification, since encoder vendor "PPR" terminology can vary.
 */
#define APP_CPR_WR                        3264.0f
#define APP_CPR_WL                        3264.0f
#define APP_CPR_BR                        400.0f
#define APP_CPR_BL                        400.0f
#define APP_CPR_CV                        3960.0f

/* Maximum accepted closed-loop speed references. */
#define APP_MAX_RPM_WR                    200.0f
#define APP_MAX_RPM_WL                    200.0f
#define APP_MAX_RPM_BR                    400.0f
#define APP_MAX_RPM_BL                    400.0f
#define APP_MAX_RPM_CV                    150.0f

/* Flip to -1.0f if measured RPM sign is opposite to the chosen positive direction. */
#define APP_ENCODER_SIGN_WR              -1.0f
#define APP_ENCODER_SIGN_WL               1.0f
#define APP_ENCODER_SIGN_BR               1.0f
#define APP_ENCODER_SIGN_BL               1.0f
#define APP_ENCODER_SIGN_CV               1.0f

/* Flip to -1.0f if positive controller output rotates the motor in the wrong direction. */
#define APP_MOTOR_SIGN_WR                -1.0f
#define APP_MOTOR_SIGN_WL                -1.0f
#define APP_MOTOR_SIGN_BR                 1.0f
#define APP_MOTOR_SIGN_BL                 1.0f
#define APP_MOTOR_SIGN_CV                -1.0f

/*
 * ESTOP wiring confirmed on PA4:
 *   released -> LOW (~0 V)
 *   pressed  -> HIGH (~3.3 V)
 * Therefore ESTOP is active-HIGH.
 */
#define APP_ESTOP_ACTIVE_LOW              0U

/*
 * PID update sanity bounds.
 * These are intentionally broad guards against corrupt/pathological values;
 * they are not intended to constrain normal controller tuning.
 */
#define APP_PID_GAIN_MAX                  100000.0f
#define APP_PID_TF_MAX_S                  10.0f

/*
 * Initial PIDF values.
 * Kf is fixed at 0.0f. Kp/Ki/Kd/Tf can be replaced at runtime by UART.
 * Initial gains are intentionally zero for a safe first power-up.
 */
#define APP_PID_WR_KP                     0.0f
#define APP_PID_WR_KI                     0.0f
#define APP_PID_WR_KD                     0.0f
#define APP_PID_WR_TF                     0.010f

#define APP_PID_WL_KP                     0.0f
#define APP_PID_WL_KI                     0.0f
#define APP_PID_WL_KD                     0.0f
#define APP_PID_WL_TF                     0.010f

#define APP_PID_BR_KP                     0.0f
#define APP_PID_BR_KI                     0.0f
#define APP_PID_BR_KD                     0.0f
#define APP_PID_BR_TF                     0.010f

#define APP_PID_BL_KP                     0.0f
#define APP_PID_BL_KI                     0.0f
#define APP_PID_BL_KD                     0.0f
#define APP_PID_BL_TF                     0.010f

#define APP_PID_CV_KP                     0.0f
#define APP_PID_CV_KI                     0.0f
#define APP_PID_CV_KD                     0.0f
#define APP_PID_CV_TF                     0.010f

#endif /* APP_CONFIG_H */
