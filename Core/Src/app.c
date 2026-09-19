#include "app.h"
#include "app_config.h"
#include "main.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* =========================== Peripheral handles =========================== */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim11;

extern UART_HandleTypeDef huart6;

/* =============================== Protocol =================================
 *
 * Frame:
 *   SOF1 | SOF2 | TYPE | PAYLOAD | CRC8
 *   AA     55
 *
 * Float format:
 *   IEEE-754 binary32, little-endian
 *
 * Integer format:
 *   SEQ is uint16_t, little-endian
 *
 * CRC-8:
 *   polynomial 0x07, init 0x00, refin=false, refout=false, xorout=0x00
 *   CRC covers TYPE + PAYLOAD only. SOF1/SOF2 are excluded.
 *
 * NORMAL MODE
 *   Pi -> STM32
 *     A0 : WR_ref, WL_ref                         12 bytes total
 *     A1 : BR_ref, BL_ref, CV_ref                 16 bytes total
 *
 *   STM32 -> Pi, every TIM10 control tick (100 Hz)
 *     01 : WR/WL/BR/BL/CV measured RPM, normal    24 bytes total
 *     00 : WR/WL/BR/BL/CV measured RPM, ESTOP     24 bytes total
 *
 *   Normal 00/01 telemetry is suppressed while F1/F3 tuning mode is active;
 *   tuning uses only the synchronized F2 response.
 *
 * PIDF UPDATE - Pi -> STM32
 *   BA : WR Kp, Ki, Kd, Tf                      20 bytes total
 *   BB : WL Kp, Ki, Kd, Tf                      20 bytes total
 *   B0 : BR Kp, Ki, Kd, Tf                      20 bytes total
 *   B1 : BL Kp, Ki, Kd, Tf                      20 bytes total
 *   B2 : CV Kp, Ki, Kd, Tf                      20 bytes total
 *
 * PIDF ECHO - STM32 -> Pi
 *   CA : WR Kp, Ki, Kd, Tf                      20 bytes total
 *   CB : WL Kp, Ki, Kd, Tf                      20 bytes total
 *   C0 : BR Kp, Ki, Kd, Tf                      20 bytes total
 *   C1 : BL Kp, Ki, Kd, Tf                      20 bytes total
 *   C2 : CV Kp, Ki, Kd, Tf                      20 bytes total
 *
 * TEMPORARY TUNING
 *   F1 : SEQ + WR/WL/BR/BL/CV duty              26 bytes total
 *        Duty is normalized -1.0 ... +1.0.
 *        Receiving F1 automatically enters open-loop SYSID mode.
 *
 *   F3 : SEQ + WR/WL/BR/BL/CV RPM reference     26 bytes total
 *        Receiving F3 automatically enters closed-loop PID-test mode.
 *
 *   F2 : SEQ + WR/WL/BR/BL/CV measured RPM      26 bytes total
 *        Sent by STM32 one control interval after the corresponding F1/F3
 *        command is actually applied at a TIM10 boundary.
 *
 *   F0 : no payload                               4 bytes total
 *        Stop all motors, reset PID state, and return to normal mode.
 *
 * Tuning synchronization:
 *   F1/F3 is received asynchronously and stored as a pending command.
 *   At a TIM10 boundary it becomes active.
 *   At the NEXT TIM10 boundary, measured RPM is returned in F2 with the same
 *   SEQ. This associates each F2 sample with the command that was active over
 *   the preceding 10 ms control interval.
 * ========================================================================== */
#define SOF1                       0xAAU
#define SOF2                       0x55U

#define TYPE_RPM_ESTOP             0x00U
#define TYPE_RPM_NORMAL            0x01U

#define TYPE_DRIVE_REF             0xA0U
#define TYPE_AUX_REF               0xA1U

#define TYPE_PID_WR_SET            0xBAU
#define TYPE_PID_WL_SET            0xBBU
#define TYPE_PID_BR_SET            0xB0U
#define TYPE_PID_BL_SET            0xB1U
#define TYPE_PID_CV_SET            0xB2U

#define TYPE_PID_WR_ECHO           0xCAU
#define TYPE_PID_WL_ECHO           0xCBU
#define TYPE_PID_BR_ECHO           0xC0U
#define TYPE_PID_BL_ECHO           0xC1U
#define TYPE_PID_CV_ECHO           0xC2U

#define TYPE_TUNING_STOP           0xF0U
#define TYPE_SYSID_DUTY            0xF1U
#define TYPE_TUNING_RPM            0xF2U
#define TYPE_PID_TEST_REF          0xF3U

#define TUNING_MOTOR_COUNT         5U

#define TUNING_STOP_FRAME_LEN       4U
#define DRIVE_FRAME_LEN            12U
#define AUX_FRAME_LEN              16U
#define PID_FRAME_LEN              20U
#define NORMAL_RPM_FRAME_LEN       24U
#define TUNING_FRAME_LEN           26U

#define RX_DMA_BUF_LEN             64U
#define RX_FRAME_MAX_LEN           TUNING_FRAME_LEN

#define TX_QUEUE_DEPTH              8U
#define TX_FRAME_MAX_LEN           TUNING_FRAME_LEN

/* =============================== PIDF ===================================== */
typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float Kf;
    float Ts;

    float integ;
    float prev_err;
    float d_state;

    float Tf;
    float ad;
    float bd;

    float out_min;
    float out_max;
} PIDF_t;

typedef enum
{
    ENC_TIMER_16,
    ENC_TIMER_32,
    ENC_SOFTWARE
} EncoderType_t;

typedef enum
{
    DRIVER_BTS7960,
    DRIVER_TB6612
} DriverType_t;

typedef enum
{
    APP_MODE_NORMAL = 0,
    APP_MODE_SYSID,
    APP_MODE_PID_TEST
} AppMode_t;

/* Latest encoder-boundary sample used by the M/T estimator. */
typedef struct
{
    volatile uint32_t edge_count;
    volatile uint32_t edge_time_cycles;
    volatile uint32_t edge_sequence;

    uint32_t prev_edge_count;
    uint32_t prev_edge_time_cycles;
    uint32_t prev_edge_sequence;
    bool initialized;
} MT_State_t;

typedef struct
{
    volatile float ref_rpm;
    volatile float rpm;
    volatile float last_output;

    float counts_per_rev;
    float encoder_sign;
    float motor_sign;

    EncoderType_t encoder_type;
    TIM_HandleTypeDef *encoder_timer;
    MT_State_t mt;

    DriverType_t driver_type;
    TIM_HandleTypeDef *pwm_timer;
    uint32_t pwm_ch_forward;
    uint32_t pwm_ch_reverse;

    GPIO_TypeDef *in1_port;
    uint16_t in1_pin;
    GPIO_TypeDef *in2_port;
    uint16_t in2_pin;

    PIDF_t pid;
} Motor_t;

/* =========================== Motor instances ============================== */
static Motor_t motorWR;
static Motor_t motorWL;
static Motor_t motorBR;
static Motor_t motorBL;
static Motor_t motorCV;

volatile AppDebug_t g_app_debug = {0};

static volatile uint32_t app_fault_flags = 0U;

/* CV software quadrature counter. */
static volatile int32_t cv_encoder_count = 0;
static uint8_t cv_prev_ab = 0U;

/* DWT/Cortex-M4 timestamp information. */
static uint32_t cpu_clock_hz = 100000000U;
static uint32_t mt_zero_timeout_cycles = 25000000U;
static volatile bool app_initialized = false;

/* Safety/communication state. */
static volatile uint32_t last_drive_cmd_ms = 0U;
static volatile bool comm_timeout_active = false;
static volatile bool estop_active = false;
static bool previous_estop_active = false;

/* --------------------- Temporary tuning state ----------------------------- */
typedef struct
{
    uint8_t type;               /* F1 or F3 */
    uint16_t seq;
    float value[TUNING_MOTOR_COUNT];
} TuningCommand_t;

static volatile AppMode_t app_mode = APP_MODE_NORMAL;
static volatile bool tuning_pending_valid = false;
static TuningCommand_t tuning_pending;

/* F0 is handled at the next deterministic TIM10 boundary. */
static volatile bool tuning_exit_requested = false;

/* Command currently being held by the tuning mode. */
static float sysid_duty[TUNING_MOTOR_COUNT] = {0.0f};
static uint32_t last_tuning_cmd_ms = 0U;

/* If true, the command identified by tuning_active_seq was applied at the
 * previous TIM10 tick and needs one synchronized F2 sample now. */
static bool tuning_sample_due = false;
static uint16_t tuning_active_seq = 0U;

/* UART RX parser. */
static uint8_t uart_rx_dma_buf[RX_DMA_BUF_LEN];
static uint8_t rx_frame[RX_FRAME_MAX_LEN];
static uint8_t rx_index = 0U;
static uint8_t rx_expected_len = 0U;

typedef enum
{
    RX_WAIT_SOF1 = 0,
    RX_WAIT_SOF2,
    RX_WAIT_TYPE,
    RX_COLLECT_FRAME
} RxParserState_t;

static RxParserState_t rx_state = RX_WAIT_SOF1;

/* One generic TX queue is enough for PID echoes and F2 tuning samples. */
typedef struct
{
    uint8_t len;
    uint8_t data[TX_FRAME_MAX_LEN];
} TxPacket_t;

static TxPacket_t tx_queue[TX_QUEUE_DEPTH];
static volatile uint8_t tx_head = 0U;
static volatile uint8_t tx_tail = 0U;
static uint8_t tx_dma_frame[TX_FRAME_MAX_LEN];
static volatile bool tx_busy = false;

/* ============================ Prototypes ================================== */
static uint32_t App_EnterCritical(void);
static void App_ExitCritical(uint32_t primask);

static float ClampFloat(float x, float mn, float mx);

static void PIDF_RecalcD(PIDF_t *pid);
static void PIDF_Reset(PIDF_t *pid);
static void PIDF_Init(PIDF_t *pid,
                      float Kp, float Ki, float Kd, float Kf,
                      float Ts, float out_min, float out_max, float Tf);
static void PIDF_SetTuning(PIDF_t *pid,
                           float Kp, float Ki, float Kd, float Tf);
static float PIDF_Step(PIDF_t *pid, float ref, float meas);

static void Motor_Init(Motor_t *motor,
                       float counts_per_rev,
                       float encoder_sign,
                       float motor_sign,
                       EncoderType_t encoder_type,
                       TIM_HandleTypeDef *encoder_timer,
                       DriverType_t driver_type,
                       TIM_HandleTypeDef *pwm_timer,
                       uint32_t pwm_ch_forward,
                       uint32_t pwm_ch_reverse,
                       GPIO_TypeDef *in1_port,
                       uint16_t in1_pin,
                       GPIO_TypeDef *in2_port,
                       uint16_t in2_pin,
                       float Kp, float Ki, float Kd, float Tf);
static void Motor_RecordBoundary(Motor_t *motor, uint32_t count, uint32_t timestamp);
static int32_t Motor_CountDifference(const Motor_t *motor,
                                     uint32_t current,
                                     uint32_t previous);
static void Motor_UpdateRPM_MT(Motor_t *motor, uint32_t now_cycles);
static void Motor_ApplyOutput(Motor_t *motor, float command);
static void Motor_ApplyDutyNormalized(Motor_t *motor, float duty);
static void Motor_StopOutput(Motor_t *motor);
static void Motor_ResetAllPID(void);

static void DWT_TimebaseInit(void);
static bool App_EstopIsActive(void);
static void App_SetDriverEnable(bool enable);
static void App_SafetyService(void);
static void App_ControlUpdate(void);
static void App_UpdateDebugSnapshot(void);
static void App_WatchdogInit(void);
static void App_WatchdogRefresh(void);
static bool RefWithinLimit(float value, float limit);
static void UART_RecordInvalidFrame(void);
static void CV_EncoderUpdate(void);

static uint8_t CRC8(const uint8_t *data, uint16_t len);
static uint16_t ReadU16LE(const uint8_t *p);
static void WriteU16LE(uint8_t *p, uint16_t value);
static float ReadFloatLE(const uint8_t *p);
static void WriteFloatLE(uint8_t *p, float value);
static uint8_t ExpectedRxLengthForType(uint8_t type);
static void UART_ResetParser(void);
static void UART_ProcessByte(uint8_t b);
static void UART_ProcessFrame(const uint8_t *frame, uint8_t len);
static void UART_StartReceiveToIdleDMA(void);
static bool UART_QueueFrame(const uint8_t *frame, uint8_t len);
static void UART_QueuePIDecho(uint8_t echo_type, const PIDF_t *pid);
static void UART_QueueNormalRPM(uint8_t type);
static void UART_QueueTuningRPM(uint16_t seq);
static void UART_ServiceTx(void);
static Motor_t *MotorFromPIDSetType(uint8_t type, uint8_t *echo_type);

static void Tuning_ZeroActuation(void);
static void Tuning_ReturnToNormal(void);
static void Tuning_ApplyPendingAtTick(void);
static void Tuning_RunCurrentMode(void);

/* =========================== Utility ====================================== */
static uint32_t App_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void App_ExitCritical(uint32_t primask)
{
    if (primask == 0U) {
        __enable_irq();
    }
}

static float ClampFloat(float x, float mn, float mx)
{
    if (x > mx) {
        return mx;
    }
    if (x < mn) {
        return mn;
    }
    return x;
}

static bool RefWithinLimit(float value, float limit)
{
    return isfinite(value) && isfinite(limit) && limit > 0.0f &&
           fabsf(value) <= limit;
}

static void UART_RecordInvalidFrame(void)
{
    g_app_debug.uart_invalid_frames++;
}

/* =============================== PIDF ===================================== */
static void PIDF_RecalcD(PIDF_t *pid)
{
    const float den = (2.0f * pid->Tf) + pid->Ts;

    if (den <= 0.0f) {
        pid->ad = 0.0f;
        pid->bd = 0.0f;
        return;
    }

    /* Tustin discretization of D(s) = Kd*s / (Tf*s + 1). */
    pid->ad = ((2.0f * pid->Tf) - pid->Ts) / den;
    pid->bd = (2.0f * pid->Kd) / den;
}

static void PIDF_Reset(PIDF_t *pid)
{
    pid->integ = 0.0f;
    pid->prev_err = 0.0f;
    pid->d_state = 0.0f;
}

static void PIDF_Init(PIDF_t *pid,
                      float Kp, float Ki, float Kd, float Kf,
                      float Ts, float out_min, float out_max, float Tf)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->Kf = Kf;
    pid->Ts = Ts;
    pid->Tf = Tf;
    pid->out_min = out_min;
    pid->out_max = out_max;

    PIDF_Reset(pid);
    PIDF_RecalcD(pid);
}

static void PIDF_SetTuning(PIDF_t *pid,
                           float Kp, float Ki, float Kd, float Tf)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->Tf = Tf;

    PIDF_RecalcD(pid);
    PIDF_Reset(pid);
}

static float PIDF_Step(PIDF_t *pid, float ref, float meas)
{
    const float e = ref - meas;
    const float de = e - pid->prev_err;

    const float d_new =
        (pid->ad * pid->d_state) +
        (pid->bd * de);

    const float integ_candidate =
        pid->integ +
        (0.5f * pid->Ts * (e + pid->prev_err));

    const float u_unsat =
        (pid->Kp * e) +
        (pid->Ki * integ_candidate) +
        d_new +
        (pid->Kf * ref);

    const float u_sat =
        ClampFloat(u_unsat, pid->out_min, pid->out_max);

    bool allow_integrate = false;

    if (u_sat == u_unsat) {
        allow_integrate = true;
    } else if ((u_sat >= pid->out_max && e < 0.0f) ||
               (u_sat <= pid->out_min && e > 0.0f)) {
        allow_integrate = true;
    }

    if (allow_integrate) {
        pid->integ = integ_candidate;
    }

    pid->d_state = d_new;
    pid->prev_err = e;

    return u_sat;
}

/* ============================== Motors ==================================== */
static void Motor_Init(Motor_t *motor,
                       float counts_per_rev,
                       float encoder_sign,
                       float motor_sign,
                       EncoderType_t encoder_type,
                       TIM_HandleTypeDef *encoder_timer,
                       DriverType_t driver_type,
                       TIM_HandleTypeDef *pwm_timer,
                       uint32_t pwm_ch_forward,
                       uint32_t pwm_ch_reverse,
                       GPIO_TypeDef *in1_port,
                       uint16_t in1_pin,
                       GPIO_TypeDef *in2_port,
                       uint16_t in2_pin,
                       float Kp, float Ki, float Kd, float Tf)
{
    memset(motor, 0, sizeof(*motor));

    motor->counts_per_rev = counts_per_rev;
    motor->encoder_sign = encoder_sign;
    motor->motor_sign = motor_sign;
    motor->encoder_type = encoder_type;
    motor->encoder_timer = encoder_timer;

    motor->driver_type = driver_type;
    motor->pwm_timer = pwm_timer;
    motor->pwm_ch_forward = pwm_ch_forward;
    motor->pwm_ch_reverse = pwm_ch_reverse;

    motor->in1_port = in1_port;
    motor->in1_pin = in1_pin;
    motor->in2_port = in2_port;
    motor->in2_pin = in2_pin;

    /* PID output units are raw PWM timer counts, matching the old project idea. */
    const float pwm_max = (float)__HAL_TIM_GET_AUTORELOAD(pwm_timer);

    PIDF_Init(&motor->pid,
              Kp, Ki, Kd, 0.0f,
              APP_CONTROL_TS_S,
              -pwm_max,
              pwm_max,
              Tf);
}

static void Motor_RecordBoundary(Motor_t *motor, uint32_t count, uint32_t timestamp)
{
    motor->mt.edge_count = count;
    motor->mt.edge_time_cycles = timestamp;
    motor->mt.edge_sequence++;
}

static int32_t Motor_CountDifference(const Motor_t *motor,
                                     uint32_t current,
                                     uint32_t previous)
{
    if (motor->encoder_type == ENC_TIMER_16) {
        return (int32_t)(int16_t)((uint16_t)current - (uint16_t)previous);
    }

    /* 32-bit hardware counter and software counter both use modulo subtraction. */
    return (int32_t)(current - previous);
}

static void Motor_UpdateRPM_MT(Motor_t *motor, uint32_t now_cycles)
{
    uint32_t edge_count;
    uint32_t edge_time;
    uint32_t edge_sequence;

    /* Keep the count/time/sequence snapshot coherent if an encoder IRQ preempts TIM10. */
    uint32_t primask = App_EnterCritical();
    edge_count = motor->mt.edge_count;
    edge_time = motor->mt.edge_time_cycles;
    edge_sequence = motor->mt.edge_sequence;
    App_ExitCritical(primask);

    if (edge_sequence != motor->mt.prev_edge_sequence) {
        if (!motor->mt.initialized) {
            /* First real encoder boundary: establish the M/T reference point. */
            motor->mt.prev_edge_count = edge_count;
            motor->mt.prev_edge_time_cycles = edge_time;
            motor->mt.prev_edge_sequence = edge_sequence;
            motor->mt.initialized = true;
            motor->rpm = 0.0f;
            return;
        }

        const int32_t delta_count =
            Motor_CountDifference(motor,
                                  edge_count,
                                  motor->mt.prev_edge_count);

        const uint32_t delta_cycles =
            edge_time - motor->mt.prev_edge_time_cycles;

        if (delta_cycles > 0U && delta_count != 0 && motor->counts_per_rev > 0.0f) {
            /*
             * M/T estimator:
             * RPM = 60 * delta_count / (CPR * delta_time_seconds)
             *     = 60 * HCLK * delta_count / (CPR * delta_DWT_cycles)
             */
            motor->rpm =
                motor->encoder_sign *
                ((60.0f * (float)cpu_clock_hz * (float)delta_count) /
                 (motor->counts_per_rev * (float)delta_cycles));
        }

        motor->mt.prev_edge_count = edge_count;
        motor->mt.prev_edge_time_cycles = edge_time;
        motor->mt.prev_edge_sequence = edge_sequence;
        return;
    }

    /* No new boundary: retain the last M/T speed briefly, then declare zero speed. */
    if (motor->mt.initialized) {
        const uint32_t age_cycles = now_cycles - edge_time;
        if (age_cycles >= mt_zero_timeout_cycles) {
            motor->rpm = 0.0f;
        }
    }
}

static void Motor_ApplyOutput(Motor_t *motor, float command)
{
    const uint32_t arr = __HAL_TIM_GET_AUTORELOAD(motor->pwm_timer);

    if (!isfinite(command) || !isfinite(motor->motor_sign)) {
        motor->last_output = 0.0f;
        app_fault_flags |= APP_STATUS_INVALID_OUTPUT;
        command = 0.0f;
    }

    float u = command * motor->motor_sign;

    if (!isfinite(u)) {
        motor->last_output = 0.0f;
        app_fault_flags |= APP_STATUS_INVALID_OUTPUT;
        u = 0.0f;
    }

    u = ClampFloat(u, -(float)arr, (float)arr);
    motor->last_output = u;

    const uint32_t duty =
        (uint32_t)(fabsf(u) + 0.5f);

    if (motor->driver_type == DRIVER_BTS7960) {
        if (u > 0.0f) {
            __HAL_TIM_SET_COMPARE(motor->pwm_timer,
                                  motor->pwm_ch_forward,
                                  duty);
            __HAL_TIM_SET_COMPARE(motor->pwm_timer,
                                  motor->pwm_ch_reverse,
                                  0U);
        } else if (u < 0.0f) {
            __HAL_TIM_SET_COMPARE(motor->pwm_timer,
                                  motor->pwm_ch_forward,
                                  0U);
            __HAL_TIM_SET_COMPARE(motor->pwm_timer,
                                  motor->pwm_ch_reverse,
                                  duty);
        } else {
            __HAL_TIM_SET_COMPARE(motor->pwm_timer,
                                  motor->pwm_ch_forward,
                                  0U);
            __HAL_TIM_SET_COMPARE(motor->pwm_timer,
                                  motor->pwm_ch_reverse,
                                  0U);
        }

        return;
    }

    /* TB6612: sign -> IN1/IN2, magnitude -> PWM. */
    if (u > 0.0f) {
        HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, GPIO_PIN_RESET);
    } else if (u < 0.0f) {
        HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, GPIO_PIN_RESET);
    }

    __HAL_TIM_SET_COMPARE(motor->pwm_timer,
                          motor->pwm_ch_forward,
                          duty);
}

static void Motor_ApplyDutyNormalized(Motor_t *motor, float duty)
{
    const float arr = (float)__HAL_TIM_GET_AUTORELOAD(motor->pwm_timer);
    const float normalized = ClampFloat(duty, -1.0f, 1.0f);
    Motor_ApplyOutput(motor, normalized * arr);
}

static void Motor_StopOutput(Motor_t *motor)
{
    Motor_ApplyOutput(motor, 0.0f);
}

static void Motor_ResetAllPID(void)
{
    PIDF_Reset(&motorWR.pid);
    PIDF_Reset(&motorWL.pid);
    PIDF_Reset(&motorBR.pid);
    PIDF_Reset(&motorBL.pid);
    PIDF_Reset(&motorCV.pid);
}

/* ============================ Time / safety ================================ */
static void DWT_TimebaseInit(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    cpu_clock_hz = HAL_RCC_GetHCLKFreq();
    mt_zero_timeout_cycles =
        (cpu_clock_hz / 1000U) * APP_MT_ZERO_TIMEOUT_MS;
}

static void App_WatchdogInit(void)
{
    /*
     * LSI is nominally ~32 kHz. Prescaler /32 gives approximately a 1 ms tick.
     * The watchdog is started only after TIM10 is running.
     */
    uint32_t reload = APP_WATCHDOG_TIMEOUT_MS;
    if (reload == 0U) {
        reload = 1U;
    }
    if (reload > 4095U) {
        reload = 4095U;
    }

    IWDG->KR = 0x5555U; /* Enable PR/RLR writes. */
    IWDG->PR = 0x03U;   /* Prescaler /32. */
    IWDG->RLR = reload - 1U;
    while (IWDG->SR != 0U) {
        /* Wait for register update before starting the watchdog. */
    }
    IWDG->KR = 0xAAAAU; /* Initial reload. */
    IWDG->KR = 0xCCCCU; /* Start. */
}

static void App_WatchdogRefresh(void)
{
    IWDG->KR = 0xAAAAU;
}

void App_EmergencyShutdown(void)
{
    /*
     * Use direct peripheral registers so the shutdown path does not depend on
     * Motor_t initialization or the scheduler.
     */
    TIM3->CCR1 = 0U;
    TIM3->CCR2 = 0U;
    TIM3->CCR3 = 0U;
    TIM3->CCR4 = 0U;
    TIM9->CCR1 = 0U;
    TIM9->CCR2 = 0U;
    TIM11->CCR1 = 0U;

    GPIOB->BSRR =
        ((uint32_t)(WR_en_Pin | WL_en_Pin | STBY_Pin |
                    BR_in1_Pin | BR_in2_Pin | BL_in1_Pin | BL_in2_Pin) << 16U);
    GPIOA->BSRR =
        ((uint32_t)(CV_in1_Pin | CV_in2_Pin) << 16U);
}

static bool App_EstopIsActive(void)
{
    const GPIO_PinState state = HAL_GPIO_ReadPin(ESTOP_GPIO_Port, ESTOP_Pin);

#if APP_ESTOP_ACTIVE_LOW
    return (state == GPIO_PIN_RESET);
#else
    return (state == GPIO_PIN_SET);
#endif
}

static void App_SetDriverEnable(bool enable)
{
    const GPIO_PinState state = enable ? GPIO_PIN_SET : GPIO_PIN_RESET;

    HAL_GPIO_WritePin(WR_en_GPIO_Port, WR_en_Pin, state);
    HAL_GPIO_WritePin(WL_en_GPIO_Port, WL_en_Pin, state);
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, state);
}

static void App_SafetyService(void)
{
    const bool active = App_EstopIsActive();
    estop_active = active;

    if (active) {
        motorWR.ref_rpm = 0.0f;
        motorWL.ref_rpm = 0.0f;
        motorBR.ref_rpm = 0.0f;
        motorBL.ref_rpm = 0.0f;
        motorCV.ref_rpm = 0.0f;

        Motor_StopOutput(&motorWR);
        Motor_StopOutput(&motorWL);
        Motor_StopOutput(&motorBR);
        Motor_StopOutput(&motorBL);
        Motor_StopOutput(&motorCV);
        App_SetDriverEnable(false);

        if (!previous_estop_active) {
            Motor_ResetAllPID();
        }
    } else if (previous_estop_active) {
        /* ESTOP release does not restore old references; new commands are required. */
        Motor_ResetAllPID();
        App_SetDriverEnable(true);
    }

    previous_estop_active = active;
}

/* ============================== UART ====================================== */
static uint8_t CRC8(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0x00U;

    for (uint16_t i = 0U; i < len; ++i) {
        crc ^= data[i];

        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            if ((crc & 0x80U) != 0U) {
                crc = (uint8_t)((crc << 1U) ^ 0x07U);
            } else {
                crc <<= 1U;
            }
        }
    }

    return crc;
}

static uint16_t ReadU16LE(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0]) |
                      ((uint16_t)p[1] << 8U));
}

static void WriteU16LE(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)(value & 0xFFU);
    p[1] = (uint8_t)((value >> 8U) & 0xFFU);
}

static float ReadFloatLE(const uint8_t *p)
{
    const uint32_t bits =
        ((uint32_t)p[0]) |
        ((uint32_t)p[1] << 8U) |
        ((uint32_t)p[2] << 16U) |
        ((uint32_t)p[3] << 24U);

    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

static void WriteFloatLE(uint8_t *p, float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));

    p[0] = (uint8_t)(bits & 0xFFU);
    p[1] = (uint8_t)((bits >> 8U) & 0xFFU);
    p[2] = (uint8_t)((bits >> 16U) & 0xFFU);
    p[3] = (uint8_t)((bits >> 24U) & 0xFFU);
}

static uint8_t ExpectedRxLengthForType(uint8_t type)
{
    switch (type) {
        case TYPE_TUNING_STOP:
            return TUNING_STOP_FRAME_LEN;

        case TYPE_DRIVE_REF:
            return DRIVE_FRAME_LEN;

        case TYPE_AUX_REF:
            return AUX_FRAME_LEN;

        case TYPE_PID_WR_SET:
        case TYPE_PID_WL_SET:
        case TYPE_PID_BR_SET:
        case TYPE_PID_BL_SET:
        case TYPE_PID_CV_SET:
            return PID_FRAME_LEN;

        case TYPE_SYSID_DUTY:
        case TYPE_PID_TEST_REF:
            return TUNING_FRAME_LEN;

        default:
            return 0U;
    }
}

static void UART_ResetParser(void)
{
    rx_state = RX_WAIT_SOF1;
    rx_index = 0U;
    rx_expected_len = 0U;
}

static void UART_ProcessByte(uint8_t b)
{
    switch (rx_state) {
        case RX_WAIT_SOF1:
            if (b == SOF1) {
                rx_frame[0] = b;
                rx_index = 1U;
                rx_state = RX_WAIT_SOF2;
            }
            break;

        case RX_WAIT_SOF2:
            if (b == SOF2) {
                rx_frame[1] = b;
                rx_index = 2U;
                rx_state = RX_WAIT_TYPE;
            } else if (b == SOF1) {
                /* Allow AA AA 55 to recover without dropping the second AA. */
                rx_frame[0] = SOF1;
                rx_index = 1U;
            } else {
                UART_ResetParser();
            }
            break;

        case RX_WAIT_TYPE:
            rx_expected_len = ExpectedRxLengthForType(b);

            if (rx_expected_len == 0U || rx_expected_len > sizeof(rx_frame)) {
                if (b == SOF1) {
                    rx_frame[0] = SOF1;
                    rx_index = 1U;
                    rx_state = RX_WAIT_SOF2;
                } else {
                    UART_ResetParser();
                }
                break;
            }

            rx_frame[2] = b;
            rx_index = 3U;
            rx_state = RX_COLLECT_FRAME;
            break;

        case RX_COLLECT_FRAME:
            if (rx_index >= sizeof(rx_frame)) {
                UART_ResetParser();
                break;
            }

            rx_frame[rx_index++] = b;

            if (rx_index >= rx_expected_len) {
                UART_ProcessFrame(rx_frame, rx_expected_len);
                UART_ResetParser();
            }
            break;

        default:
            UART_ResetParser();
            break;
    }
}

static Motor_t *MotorFromPIDSetType(uint8_t type, uint8_t *echo_type)
{
    switch (type) {
        case TYPE_PID_WR_SET:
            *echo_type = TYPE_PID_WR_ECHO;
            return &motorWR;

        case TYPE_PID_WL_SET:
            *echo_type = TYPE_PID_WL_ECHO;
            return &motorWL;

        case TYPE_PID_BR_SET:
            *echo_type = TYPE_PID_BR_ECHO;
            return &motorBR;

        case TYPE_PID_BL_SET:
            *echo_type = TYPE_PID_BL_ECHO;
            return &motorBL;

        case TYPE_PID_CV_SET:
            *echo_type = TYPE_PID_CV_ECHO;
            return &motorCV;

        default:
            *echo_type = 0U;
            return NULL;
    }
}

static void UART_ProcessFrame(const uint8_t *frame, uint8_t len)
{
    if (len < TUNING_STOP_FRAME_LEN ||
        frame[0] != SOF1 ||
        frame[1] != SOF2) {
        return;
    }

    /* CRC is TYPE + PAYLOAD; SOF1/SOF2 and the CRC byte itself are excluded. */
    const uint8_t received_crc = frame[len - 1U];
    const uint8_t calculated_crc = CRC8(&frame[2], (uint16_t)(len - 3U));

    if (received_crc != calculated_crc) {
        g_app_debug.uart_crc_errors++;
        return;
    }

    g_app_debug.uart_rx_frames_ok++;

    const uint8_t type = frame[2];

    /* F0: deterministic stop/return is performed at the next TIM10 boundary. */
    if (type == TYPE_TUNING_STOP && len == TUNING_STOP_FRAME_LEN) {
        const uint32_t primask = App_EnterCritical();
        tuning_exit_requested = true;
        tuning_pending_valid = false;
        App_ExitCritical(primask);
        return;
    }

    /* A0/A1 are accepted only when no temporary tuning command is active/pending. */
    if (type == TYPE_DRIVE_REF && len == DRIVE_FRAME_LEN) {
        if (app_mode != APP_MODE_NORMAL || tuning_pending_valid) {
            return;
        }

        const float wr_ref = ReadFloatLE(&frame[3]);
        const float wl_ref = ReadFloatLE(&frame[7]);

        if (!RefWithinLimit(wr_ref, APP_MAX_RPM_WR) ||
            !RefWithinLimit(wl_ref, APP_MAX_RPM_WL)) {
            UART_RecordInvalidFrame();
            return;
        }

        const uint32_t primask = App_EnterCritical();
        motorWR.ref_rpm = wr_ref;
        motorWL.ref_rpm = wl_ref;
        last_drive_cmd_ms = HAL_GetTick();
        comm_timeout_active = false;
        App_ExitCritical(primask);
        return;
    }

    if (type == TYPE_AUX_REF && len == AUX_FRAME_LEN) {
        if (app_mode != APP_MODE_NORMAL || tuning_pending_valid) {
            return;
        }

        const float br_ref = ReadFloatLE(&frame[3]);
        const float bl_ref = ReadFloatLE(&frame[7]);
        const float cv_ref = ReadFloatLE(&frame[11]);

        if (!RefWithinLimit(br_ref, APP_MAX_RPM_BR) ||
            !RefWithinLimit(bl_ref, APP_MAX_RPM_BL) ||
            !RefWithinLimit(cv_ref, APP_MAX_RPM_CV)) {
            UART_RecordInvalidFrame();
            return;
        }

        const uint32_t primask = App_EnterCritical();
        motorBR.ref_rpm = br_ref;
        motorBL.ref_rpm = bl_ref;
        motorCV.ref_rpm = cv_ref;
        App_ExitCritical(primask);
        return;
    }

    /* F1/F3: store only the newest complete valid command for the next TIM10 tick. */
    if ((type == TYPE_SYSID_DUTY || type == TYPE_PID_TEST_REF) &&
        len == TUNING_FRAME_LEN) {

        TuningCommand_t cmd;
        cmd.type = type;
        cmd.seq = ReadU16LE(&frame[3]);

        for (uint8_t i = 0U; i < TUNING_MOTOR_COUNT; ++i) {
            cmd.value[i] = ReadFloatLE(&frame[5U + (4U * i)]);
            if (!isfinite(cmd.value[i])) {
                return;
            }
        }

        if (type == TYPE_SYSID_DUTY) {
            /* Reject invalid duty rather than silently changing the experiment input. */
            for (uint8_t i = 0U; i < TUNING_MOTOR_COUNT; ++i) {
                if (cmd.value[i] < -1.0f || cmd.value[i] > 1.0f) {
                    UART_RecordInvalidFrame();
                    return;
                }
            }
        } else {
            if (!RefWithinLimit(cmd.value[0], APP_MAX_RPM_WR) ||
                !RefWithinLimit(cmd.value[1], APP_MAX_RPM_WL) ||
                !RefWithinLimit(cmd.value[2], APP_MAX_RPM_BR) ||
                !RefWithinLimit(cmd.value[3], APP_MAX_RPM_BL) ||
                !RefWithinLimit(cmd.value[4], APP_MAX_RPM_CV)) {
                UART_RecordInvalidFrame();
                return;
            }
        }

        const uint32_t primask = App_EnterCritical();
        tuning_pending = cmd;
        tuning_pending_valid = true;
        tuning_exit_requested = false;
        last_tuning_cmd_ms = HAL_GetTick();
        App_ExitCritical(primask);
        return;
    }

    /* PIDF updates are valid in normal, SYSID, and PID-test modes. */
    if (len == PID_FRAME_LEN) {
        uint8_t echo_type;
        Motor_t *motor = MotorFromPIDSetType(type, &echo_type);

        if (motor == NULL) {
            return;
        }

        const float Kp = ReadFloatLE(&frame[3]);
        const float Ki = ReadFloatLE(&frame[7]);
        const float Kd = ReadFloatLE(&frame[11]);
        const float Tf = ReadFloatLE(&frame[15]);

        if (!isfinite(Kp) || !isfinite(Ki) ||
            !isfinite(Kd) || !isfinite(Tf) ||
            Kp < 0.0f || Ki < 0.0f || Kd < 0.0f || Tf < 0.0f ||
            Kp > APP_PID_GAIN_MAX ||
            Ki > APP_PID_GAIN_MAX ||
            Kd > APP_PID_GAIN_MAX ||
            Tf > APP_PID_TF_MAX_S) {
            UART_RecordInvalidFrame();
            return;
        }

        const uint32_t primask = App_EnterCritical();
        PIDF_SetTuning(&motor->pid, Kp, Ki, Kd, Tf);
        App_ExitCritical(primask);

        UART_QueuePIDecho(echo_type, &motor->pid);
    }
}

static void UART_StartReceiveToIdleDMA(void)
{
    if (huart6.hdmarx == NULL) {
        return;
    }

    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart6,
                                     uart_rx_dma_buf,
                                     RX_DMA_BUF_LEN) == HAL_OK) {
        /* IDLE/full-buffer events are sufficient; no half-transfer callback needed. */
        __HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_HT);
    }
}

static bool UART_QueueFrame(const uint8_t *frame, uint8_t len)
{
    if (frame == NULL || len == 0U || len > TX_FRAME_MAX_LEN) {
        return false;
    }

    uint32_t primask = App_EnterCritical();
    const uint8_t next = (uint8_t)((tx_head + 1U) % TX_QUEUE_DEPTH);

    if (next == tx_tail) {
        g_app_debug.uart_tx_queue_drops++;
        app_fault_flags |= APP_STATUS_TX_QUEUE_DROP_SEEN;
        App_ExitCritical(primask);
        return false;
    }

    tx_queue[tx_head].len = len;
    memcpy(tx_queue[tx_head].data, frame, len);
    tx_head = next;

    App_ExitCritical(primask);
    return true;
}

static void UART_QueuePIDecho(uint8_t echo_type, const PIDF_t *pid)
{
    uint8_t frame[PID_FRAME_LEN];

    frame[0] = SOF1;
    frame[1] = SOF2;
    frame[2] = echo_type;

    WriteFloatLE(&frame[3],  pid->Kp);
    WriteFloatLE(&frame[7],  pid->Ki);
    WriteFloatLE(&frame[11], pid->Kd);
    WriteFloatLE(&frame[15], pid->Tf);

    frame[PID_FRAME_LEN - 1U] =
        CRC8(&frame[2], PID_FRAME_LEN - 3U);

    (void)UART_QueueFrame(frame, PID_FRAME_LEN);
}

static void UART_QueueNormalRPM(uint8_t type)
{
    if (type != TYPE_RPM_NORMAL && type != TYPE_RPM_ESTOP) {
        return;
    }

    uint8_t frame[NORMAL_RPM_FRAME_LEN];

    frame[0] = SOF1;
    frame[1] = SOF2;
    frame[2] = type;

    WriteFloatLE(&frame[3],  motorWR.rpm);
    WriteFloatLE(&frame[7],  motorWL.rpm);
    WriteFloatLE(&frame[11], motorBR.rpm);
    WriteFloatLE(&frame[15], motorBL.rpm);
    WriteFloatLE(&frame[19], motorCV.rpm);

    frame[NORMAL_RPM_FRAME_LEN - 1U] =
        CRC8(&frame[2], NORMAL_RPM_FRAME_LEN - 3U);

    (void)UART_QueueFrame(frame, NORMAL_RPM_FRAME_LEN);
}

static void UART_QueueTuningRPM(uint16_t seq)
{
    uint8_t frame[TUNING_FRAME_LEN];

    frame[0] = SOF1;
    frame[1] = SOF2;
    frame[2] = TYPE_TUNING_RPM;
    WriteU16LE(&frame[3], seq);

    WriteFloatLE(&frame[5],  motorWR.rpm);
    WriteFloatLE(&frame[9],  motorWL.rpm);
    WriteFloatLE(&frame[13], motorBR.rpm);
    WriteFloatLE(&frame[17], motorBL.rpm);
    WriteFloatLE(&frame[21], motorCV.rpm);

    frame[TUNING_FRAME_LEN - 1U] =
        CRC8(&frame[2], TUNING_FRAME_LEN - 3U);

    (void)UART_QueueFrame(frame, TUNING_FRAME_LEN);
}

static void UART_ServiceTx(void)
{
    if (tx_busy || huart6.hdmatx == NULL) {
        return;
    }

    uint8_t tail;
    uint8_t len;

    uint32_t primask = App_EnterCritical();

    if (tx_head == tx_tail) {
        App_ExitCritical(primask);
        return;
    }

    tail = tx_tail;
    len = tx_queue[tail].len;
    memcpy(tx_dma_frame, tx_queue[tail].data, len);

    App_ExitCritical(primask);

    if (HAL_UART_Transmit_DMA(&huart6, tx_dma_frame, len) == HAL_OK) {
        primask = App_EnterCritical();
        tx_tail = (uint8_t)((tail + 1U) % TX_QUEUE_DEPTH);
        tx_busy = true;
        App_ExitCritical(primask);
    }
}

/* ============================= Encoders =================================== */
void App_EncoderEdgeIRQ(TIM_HandleTypeDef *htim)
{
    if (!app_initialized || htim == NULL) {
        return;
    }

    /* We intentionally enable only CC1 interrupt: one A-channel rising boundary. */
    if ((__HAL_TIM_GET_FLAG(htim, TIM_FLAG_CC1) == RESET) ||
        (__HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_CC1) == RESET)) {
        return;
    }

    const uint32_t timestamp = DWT->CYCCNT;
    const uint32_t count = __HAL_TIM_GET_COUNTER(htim);

    if (htim->Instance == TIM1) {
        Motor_RecordBoundary(&motorBR, count, timestamp);
    } else if (htim->Instance == TIM2) {
        Motor_RecordBoundary(&motorWR, count, timestamp);
    } else if (htim->Instance == TIM4) {
        Motor_RecordBoundary(&motorBL, count, timestamp);
    } else if (htim->Instance == TIM5) {
        Motor_RecordBoundary(&motorWL, count, timestamp);
    }
}

static void CV_EncoderUpdate(void)
{
    static const int8_t quad_table[16] = {
         0, -1,  1,  0,
         1,  0,  0, -1,
        -1,  0,  0,  1,
         0,  1, -1,  0
    };

    const uint8_t old_ab = cv_prev_ab;
    const uint8_t old_a = (uint8_t)((old_ab >> 1U) & 0x01U);

    const uint8_t a =
        (HAL_GPIO_ReadPin(CV_encA_GPIO_Port, CV_encA_Pin) == GPIO_PIN_SET) ? 1U : 0U;
    const uint8_t b =
        (HAL_GPIO_ReadPin(CV_encB_GPIO_Port, CV_encB_Pin) == GPIO_PIN_SET) ? 1U : 0U;

    const uint8_t current_ab = (uint8_t)((a << 1U) | b);
    const uint8_t index = (uint8_t)((old_ab << 2U) | current_ab);
    const int8_t step = quad_table[index];

    if (step != 0) {
        cv_encoder_count += step;

        /* Match hardware M/T boundary style: timestamp only A-channel rising edges. */
        if (old_a == 0U && a == 1U) {
            Motor_RecordBoundary(&motorCV,
                                 (uint32_t)cv_encoder_count,
                                 DWT->CYCCNT);
        }
    }

    cv_prev_ab = current_ab;
}

/* =========================== Control loop ================================= */
static void Tuning_ZeroActuation(void)
{
    motorWR.ref_rpm = 0.0f;
    motorWL.ref_rpm = 0.0f;
    motorBR.ref_rpm = 0.0f;
    motorBL.ref_rpm = 0.0f;
    motorCV.ref_rpm = 0.0f;

    for (uint8_t i = 0U; i < TUNING_MOTOR_COUNT; ++i) {
        sysid_duty[i] = 0.0f;
    }

    Motor_StopOutput(&motorWR);
    Motor_StopOutput(&motorWL);
    Motor_StopOutput(&motorBR);
    Motor_StopOutput(&motorBL);
    Motor_StopOutput(&motorCV);

    Motor_ResetAllPID();
}

static void Tuning_ReturnToNormal(void)
{
    Tuning_ZeroActuation();

    app_mode = APP_MODE_NORMAL;
    tuning_pending_valid = false;
    tuning_exit_requested = false;
    tuning_sample_due = false;
    tuning_active_seq = 0U;

    /* Normal mode starts from zero and requires a fresh A0 heartbeat to move. */
    last_drive_cmd_ms = HAL_GetTick();
    comm_timeout_active = true;
}

static void Tuning_ApplyPendingAtTick(void)
{
    TuningCommand_t cmd;
    bool have_pending = false;

    uint32_t primask = App_EnterCritical();
    if (tuning_pending_valid) {
        cmd = tuning_pending;
        tuning_pending_valid = false;
        have_pending = true;
    }
    App_ExitCritical(primask);

    if (!have_pending) {
        return;
    }

    const AppMode_t requested_mode =
        (cmd.type == TYPE_SYSID_DUTY) ? APP_MODE_SYSID : APP_MODE_PID_TEST;

    if (app_mode != requested_mode) {
        /* Mode changes never inherit duty, reference, or PID internal state. */
        Tuning_ZeroActuation();
        app_mode = requested_mode;
    }

    if (requested_mode == APP_MODE_SYSID) {
        for (uint8_t i = 0U; i < TUNING_MOTOR_COUNT; ++i) {
            sysid_duty[i] = cmd.value[i];
        }
    } else {
        motorWR.ref_rpm = cmd.value[0];
        motorWL.ref_rpm = cmd.value[1];
        motorBR.ref_rpm = cmd.value[2];
        motorBL.ref_rpm = cmd.value[3];
        motorCV.ref_rpm = cmd.value[4];
    }

    /* This SEQ will be sampled on the NEXT TIM10 boundary. */
    tuning_active_seq = cmd.seq;
    tuning_sample_due = true;
}

static void Tuning_RunCurrentMode(void)
{
    if (app_mode == APP_MODE_SYSID) {
        Motor_ApplyDutyNormalized(&motorWR, sysid_duty[0]);
        Motor_ApplyDutyNormalized(&motorWL, sysid_duty[1]);
        Motor_ApplyDutyNormalized(&motorBR, sysid_duty[2]);
        Motor_ApplyDutyNormalized(&motorBL, sysid_duty[3]);
        Motor_ApplyDutyNormalized(&motorCV, sysid_duty[4]);
        return;
    }

    if (app_mode == APP_MODE_PID_TEST) {
        const float u_wr = PIDF_Step(&motorWR.pid, motorWR.ref_rpm, motorWR.rpm);
        const float u_wl = PIDF_Step(&motorWL.pid, motorWL.ref_rpm, motorWL.rpm);
        const float u_br = PIDF_Step(&motorBR.pid, motorBR.ref_rpm, motorBR.rpm);
        const float u_bl = PIDF_Step(&motorBL.pid, motorBL.ref_rpm, motorBL.rpm);
        const float u_cv = PIDF_Step(&motorCV.pid, motorCV.ref_rpm, motorCV.rpm);

        Motor_ApplyOutput(&motorWR, u_wr);
        Motor_ApplyOutput(&motorWL, u_wl);
        Motor_ApplyOutput(&motorBR, u_br);
        Motor_ApplyOutput(&motorBL, u_bl);
        Motor_ApplyOutput(&motorCV, u_cv);
    }
}

static void App_ControlUpdate(void)
{
    g_app_debug.control_tick++;

    App_SafetyService();

    const uint32_t now_cycles = DWT->CYCCNT;

    /* M/T RPM measurement always runs, independent of control mode. */
    Motor_UpdateRPM_MT(&motorWR, now_cycles);
    Motor_UpdateRPM_MT(&motorWL, now_cycles);
    Motor_UpdateRPM_MT(&motorBR, now_cycles);
    Motor_UpdateRPM_MT(&motorBL, now_cycles);
    Motor_UpdateRPM_MT(&motorCV, now_cycles);

    if (estop_active) {
        /* ESTOP aborts a tuning session. UART is never part of the safety chain. */
        app_mode = APP_MODE_NORMAL;
        tuning_pending_valid = false;
        tuning_exit_requested = false;
        tuning_sample_due = false;

        /* Preserve normal telemetry semantics: TYPE 0x00 reports ESTOP active. */
        UART_QueueNormalRPM(TYPE_RPM_ESTOP);
        return;
    }

    if (tuning_exit_requested) {
        Tuning_ReturnToNormal();
        return;
    }

    if (app_mode != APP_MODE_NORMAL) {
        if ((HAL_GetTick() - last_tuning_cmd_ms) > APP_TUNING_TIMEOUT_MS) {
            Tuning_ReturnToNormal();
            return;
        }

        /*
         * The RPM measured above belongs to the command that was active over
         * the preceding control interval. Return it before promoting a new command.
         */
        if (tuning_sample_due) {
            UART_QueueTuningRPM(tuning_active_seq);
            tuning_sample_due = false;
        }
    }

    /* A pending F1/F3 can enter tuning directly from normal mode or update it. */
    Tuning_ApplyPendingAtTick();

    if (app_mode != APP_MODE_NORMAL) {
        Tuning_RunCurrentMode();
        return;
    }

    /* ---------------------------- Normal mode ----------------------------- */
    if ((HAL_GetTick() - last_drive_cmd_ms) > APP_COMM_TIMEOUT_MS) {
        /* A0 is the normal-mode heartbeat. Never allow A1-only traffic to keep
         * an old command alive after the heartbeat has expired. */
        motorWR.ref_rpm = 0.0f;
        motorWL.ref_rpm = 0.0f;
        motorBR.ref_rpm = 0.0f;
        motorBL.ref_rpm = 0.0f;
        motorCV.ref_rpm = 0.0f;

        if (!comm_timeout_active) {
            Motor_ResetAllPID();
            comm_timeout_active = true;
        }
    }

    const float u_wr = PIDF_Step(&motorWR.pid, motorWR.ref_rpm, motorWR.rpm);
    const float u_wl = PIDF_Step(&motorWL.pid, motorWL.ref_rpm, motorWL.rpm);
    const float u_br = PIDF_Step(&motorBR.pid, motorBR.ref_rpm, motorBR.rpm);
    const float u_bl = PIDF_Step(&motorBL.pid, motorBL.ref_rpm, motorBL.rpm);
    const float u_cv = PIDF_Step(&motorCV.pid, motorCV.ref_rpm, motorCV.rpm);

    Motor_ApplyOutput(&motorWR, u_wr);
    Motor_ApplyOutput(&motorWL, u_wl);
    Motor_ApplyOutput(&motorBR, u_br);
    Motor_ApplyOutput(&motorBL, u_bl);
    Motor_ApplyOutput(&motorCV, u_cv);

    /* Normal mode continuously reports all five measured RPM values at the
     * 100 Hz TIM10 control rate. F1/F3 tuning mode suppresses this stream and
     * uses synchronized F2 packets instead. */
    UART_QueueNormalRPM(TYPE_RPM_NORMAL);

    App_UpdateDebugSnapshot();
}

static void App_UpdateDebugSnapshot(void)
{
    uint32_t status = app_fault_flags;

    if (estop_active) {
        status |= APP_STATUS_ESTOP;
    }
    if (comm_timeout_active) {
        status |= APP_STATUS_COMM_TIMEOUT;
    }
    if (app_mode == APP_MODE_SYSID) {
        status |= APP_STATUS_SYSID_MODE;
    } else if (app_mode == APP_MODE_PID_TEST) {
        status |= APP_STATUS_PID_TEST_MODE;
    }

    g_app_debug.status_flags = status;

    g_app_debug.encoder_count_wr = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    g_app_debug.encoder_count_wl = (int32_t)__HAL_TIM_GET_COUNTER(&htim5);
    g_app_debug.encoder_count_br = (int32_t)(int16_t)__HAL_TIM_GET_COUNTER(&htim1);
    g_app_debug.encoder_count_bl = (int32_t)(int16_t)__HAL_TIM_GET_COUNTER(&htim4);
    g_app_debug.encoder_count_cv = cv_encoder_count;

    g_app_debug.ref_rpm_wr = motorWR.ref_rpm;
    g_app_debug.ref_rpm_wl = motorWL.ref_rpm;
    g_app_debug.ref_rpm_br = motorBR.ref_rpm;
    g_app_debug.ref_rpm_bl = motorBL.ref_rpm;
    g_app_debug.ref_rpm_cv = motorCV.ref_rpm;

    g_app_debug.rpm_wr = motorWR.rpm;
    g_app_debug.rpm_wl = motorWL.rpm;
    g_app_debug.rpm_br = motorBR.rpm;
    g_app_debug.rpm_bl = motorBL.rpm;
    g_app_debug.rpm_cv = motorCV.rpm;

    g_app_debug.output_wr = motorWR.last_output;
    g_app_debug.output_wl = motorWL.last_output;
    g_app_debug.output_br = motorBR.last_output;
    g_app_debug.output_bl = motorBL.last_output;
    g_app_debug.output_cv = motorCV.last_output;
}

/* ============================== Public ==================================== */
void App_Init(void)
{
    DWT_TimebaseInit();

    /* Preserve the reset cause for Live Expression inspection, then clear it. */
    g_app_debug.reset_flags_raw = RCC->CSR;
    __HAL_RCC_CLEAR_RESET_FLAGS();

    /*
     * The uploaded CubeMX file still initializes USART6 at 115200.
     * Enforce the agreed 1 Mbaud here; this custom file is not overwritten by CubeMX.
     */
    if (huart6.Init.BaudRate != APP_UART_BAUD) {
        huart6.Init.BaudRate = APP_UART_BAUD;
        if (HAL_UART_Init(&huart6) != HAL_OK) {
            Error_Handler();
        }
    }

    Motor_Init(&motorWR,
               APP_CPR_WR,
               APP_ENCODER_SIGN_WR,
               APP_MOTOR_SIGN_WR,
               ENC_TIMER_32,
               &htim2,
               DRIVER_BTS7960,
               &htim3,
               TIM_CHANNEL_1, /* WR_pwmR */
               TIM_CHANNEL_2, /* WR_pwmL */
               NULL, 0U, NULL, 0U,
               APP_PID_WR_KP,
               APP_PID_WR_KI,
               APP_PID_WR_KD,
               APP_PID_WR_TF);

    Motor_Init(&motorWL,
               APP_CPR_WL,
               APP_ENCODER_SIGN_WL,
               APP_MOTOR_SIGN_WL,
               ENC_TIMER_32,
               &htim5,
               DRIVER_BTS7960,
               &htim3,
               TIM_CHANNEL_3, /* WL_pwmR */
               TIM_CHANNEL_4, /* WL_pwmL */
               NULL, 0U, NULL, 0U,
               APP_PID_WL_KP,
               APP_PID_WL_KI,
               APP_PID_WL_KD,
               APP_PID_WL_TF);

    Motor_Init(&motorBR,
               APP_CPR_BR,
               APP_ENCODER_SIGN_BR,
               APP_MOTOR_SIGN_BR,
               ENC_TIMER_16,
               &htim1,
               DRIVER_TB6612,
               &htim9,
               TIM_CHANNEL_1, /* BR_pwm */
               0U,
               BR_in1_GPIO_Port, BR_in1_Pin,
               BR_in2_GPIO_Port, BR_in2_Pin,
               APP_PID_BR_KP,
               APP_PID_BR_KI,
               APP_PID_BR_KD,
               APP_PID_BR_TF);

    Motor_Init(&motorBL,
               APP_CPR_BL,
               APP_ENCODER_SIGN_BL,
               APP_MOTOR_SIGN_BL,
               ENC_TIMER_16,
               &htim4,
               DRIVER_TB6612,
               &htim9,
               TIM_CHANNEL_2, /* BL_pwm */
               0U,
               BL_in1_GPIO_Port, BL_in1_Pin,
               BL_in2_GPIO_Port, BL_in2_Pin,
               APP_PID_BL_KP,
               APP_PID_BL_KI,
               APP_PID_BL_KD,
               APP_PID_BL_TF);

    Motor_Init(&motorCV,
               APP_CPR_CV,
               APP_ENCODER_SIGN_CV,
               APP_MOTOR_SIGN_CV,
               ENC_SOFTWARE,
               NULL,
               DRIVER_TB6612,
               &htim11,
               TIM_CHANNEL_1, /* CV_pwm */
               0U,
               CV_in1_GPIO_Port, CV_in1_Pin,
               CV_in2_GPIO_Port, CV_in2_Pin,
               APP_PID_CV_KP,
               APP_PID_CV_KI,
               APP_PID_CV_KD,
               APP_PID_CV_TF);

    /* Start hardware encoder counters without enabling both CC1/CC2 interrupts. */
    if (HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL) != HAL_OK) {
        Error_Handler();
    }

    /*
     * M/T boundary interrupt: only CH1 (encoder A rising) for each hardware encoder.
     * The timer itself still decodes A+B in TI12 mode and counts quadrature normally.
     */
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_CC1);
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC1);
    __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_CC1);
    __HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_CC1);

    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);
    __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_CC1);
    __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_CC1);
    __HAL_TIM_ENABLE_IT(&htim5, TIM_IT_CC1);

    /* Initialize CV quadrature state before EXTI edges start being interpreted. */
    {
        const uint8_t a =
            (HAL_GPIO_ReadPin(CV_encA_GPIO_Port, CV_encA_Pin) == GPIO_PIN_SET) ? 1U : 0U;
        const uint8_t b =
            (HAL_GPIO_ReadPin(CV_encB_GPIO_Port, CV_encB_Pin) == GPIO_PIN_SET) ? 1U : 0U;
        cv_prev_ab = (uint8_t)((a << 1U) | b);
    }

    /* Start all seven PWM outputs at zero duty. */
    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }

    Motor_StopOutput(&motorWR);
    Motor_StopOutput(&motorWL);
    Motor_StopOutput(&motorBR);
    Motor_StopOutput(&motorBL);
    Motor_StopOutput(&motorCV);

    app_initialized = true;

    estop_active = App_EstopIsActive();
    previous_estop_active = estop_active;
    App_SetDriverEnable(!estop_active);

    last_drive_cmd_ms = HAL_GetTick();
    /* No A0 heartbeat has been received yet after boot. */
    comm_timeout_active = true;

    app_mode = APP_MODE_NORMAL;
    tuning_pending_valid = false;
    tuning_exit_requested = false;
    tuning_sample_due = false;
    last_tuning_cmd_ms = HAL_GetTick();

    UART_StartReceiveToIdleDMA();

    /* TIM10 is the 100 Hz controller scheduler. */
    if (HAL_TIM_Base_Start_IT(&htim10) != HAL_OK) {
        Error_Handler();
    }

    App_UpdateDebugSnapshot();
    App_WatchdogInit();
}

void App_Task(void)
{
    static uint32_t last_watchdog_control_tick = 0U;

    /* Fast local ESTOP polling in addition to the 100 Hz control callback. */
    App_SafetyService();

    /* Non-time-critical UART TX is kept outside the TIM10 ISR. */
    UART_ServiceTx();

    /*
     * Refresh the independent watchdog only if the control ISR is alive.
     * This makes the watchdog cover both a stalled main loop and a stalled
     * control scheduler.
     */
    const uint32_t tick = g_app_debug.control_tick;
    if (tick != last_watchdog_control_tick) {
        last_watchdog_control_tick = tick;
        App_WatchdogRefresh();
    }
}

/* ============================ HAL callbacks =============================== */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM10) {
        App_ControlUpdate();
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (!app_initialized) {
        return;
    }

    if (GPIO_Pin == CV_encA_Pin || GPIO_Pin == CV_encB_Pin) {
        CV_EncoderUpdate();
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance != USART6) {
        return;
    }

    for (uint16_t i = 0U; i < Size; ++i) {
        UART_ProcessByte(uart_rx_dma_buf[i]);
    }

    UART_StartReceiveToIdleDMA();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6) {
        tx_busy = false;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART6) {
        return;
    }

    g_app_debug.uart_errors++;
    app_fault_flags |= APP_STATUS_UART_ERROR_SEEN;

    HAL_UART_DMAStop(&huart6);
    tx_busy = false;
    UART_ResetParser();
    UART_StartReceiveToIdleDMA();
}
