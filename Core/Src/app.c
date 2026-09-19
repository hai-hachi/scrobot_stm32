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

/* =========================== UART protocol v2 =============================
 *
 * Frame:
 *   AA 55 | VER | TYPE | SEQ(u16 LE) | LEN | PAYLOAD | CRC16(u16 LE)
 *
 * CRC-16/CCITT-FALSE:
 *   poly 0x1021, init 0xFFFF, refin=false, refout=false, xorout=0x0000
 *   CRC covers VER through the final payload byte. SOF and CRC are excluded.
 *
 * Pi -> STM32
 *   10 SETPOINT       : 5 x float RPM [WR, WL, BR, BL, CV]
 *   11 ARM            : no payload
 *   12 DISARM         : no payload
 *   30 SYSID_COMMAND  : motor_id(u8) + duty(float)
 *   32 SYSID_STOP     : no payload
 *   40 PID_SET        : motor_id(u8) + Kp, Ki, Kd, Tf
 *   41 PID_GET        : motor_id(u8)
 *   50 INFO_REQUEST   : no payload
 *
 * STM32 -> Pi
 *   20 FEEDBACK       : 100 Hz control/status/count/RPM packet
 *   21 DIAGNOSTICS    : on request / low-rate host polling
 *   31 SYSID_SAMPLE   : 100 Hz while SYSID is active
 *   42 PID_RESPONSE
 *   51 INFO_RESPONSE
 *   7F ERROR/NACK
 * ========================================================================== */
#define SOF1                       0xAAU
#define SOF2                       0x55U

#define TYPE_SETPOINT              0x10U
#define TYPE_ARM                   0x11U
#define TYPE_DISARM                0x12U

#define TYPE_FEEDBACK              0x20U
#define TYPE_DIAGNOSTICS           0x21U

#define TYPE_SYSID_COMMAND         0x30U
#define TYPE_SYSID_SAMPLE          0x31U
#define TYPE_SYSID_STOP            0x32U

#define TYPE_PID_SET               0x40U
#define TYPE_PID_GET               0x41U
#define TYPE_PID_RESPONSE          0x42U

#define TYPE_INFO_REQUEST          0x50U
#define TYPE_INFO_RESPONSE         0x51U

#define TYPE_ERROR                 0x7FU

#define MOTOR_ID_WR                 0U
#define MOTOR_ID_WL                 1U
#define MOTOR_ID_BR                 2U
#define MOTOR_ID_BL                 3U
#define MOTOR_ID_CV                 4U
#define MOTOR_COUNT                 5U

#define UART_HEADER_LEN             7U
#define UART_CRC_LEN                2U
#define UART_MAX_PAYLOAD_LEN       64U
#define UART_FRAME_MAX_LEN         (UART_HEADER_LEN + UART_MAX_PAYLOAD_LEN + UART_CRC_LEN)

#define RX_DMA_BUF_LEN            128U
#define TX_QUEUE_DEPTH              8U

#define ERROR_BAD_VERSION           1U
#define ERROR_BAD_LENGTH            2U
#define ERROR_BAD_VALUE             3U
#define ERROR_NOT_ARMED             4U
#define ERROR_ESTOP_ACTIVE          5U
#define ERROR_BAD_MOTOR             6U

/* =============================== PIDF ===================================== *//* =============================== PIDF ===================================== */
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
static volatile uint32_t last_setpoint_ms = 0U;
static volatile bool comm_timeout_active = true;
static volatile bool estop_active = false;
static bool previous_estop_active = false;
static volatile bool armed = false;
static uint16_t last_setpoint_seq = 0U;

/* System-identification state. */
static volatile bool sysid_active = false;
static uint8_t sysid_motor_id = MOTOR_ID_WR;
static float sysid_duty = 0.0f;
static uint16_t sysid_command_seq = 0U;
static uint32_t last_sysid_cmd_ms = 0U;

/* UART RX parser. */
static uint8_t uart_rx_dma_buf[RX_DMA_BUF_LEN];
static uint8_t rx_frame[UART_FRAME_MAX_LEN];
static uint8_t rx_index = 0U;
static uint8_t rx_payload_len = 0U;
static uint8_t rx_expected_len = 0U;

typedef enum
{
    RX_WAIT_SOF1 = 0,
    RX_WAIT_SOF2,
    RX_COLLECT_HEADER,
    RX_COLLECT_FRAME
} RxParserState_t;

static RxParserState_t rx_state = RX_WAIT_SOF1;

/* Generic TX queue for feedback, diagnostics and command responses. */
typedef struct
{
    uint8_t len;
    uint8_t data[UART_FRAME_MAX_LEN];
} TxPacket_t;

static TxPacket_t tx_queue[TX_QUEUE_DEPTH];
static volatile uint8_t tx_head = 0U;
static volatile uint8_t tx_tail = 0U;
static uint8_t tx_dma_frame[UART_FRAME_MAX_LEN];
static uint16_t tx_sequence = 0U;

/* ============================ Prototypes ================================== */
static uint32_t App_EnterCritical(void);
static void App_ExitCritical(uint32_t primask);
static float ClampFloat(float x, float mn, float mx);
static bool RefWithinLimit(float value, float limit);
static void UART_RecordInvalidFrame(void);

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
static void App_WatchdogInit(void);
static void App_WatchdogRefresh(void);
static bool App_EstopIsActive(void);
static void App_SetDriverEnable(bool enable);
static void App_SafetyService(void);
static void App_ZeroReferences(void);
static void App_Disarm(void);
static bool App_TryArm(void);
static void Sysid_Stop(bool disarm_after);

static uint16_t CRC16_CCITT_FALSE(const uint8_t *data, uint16_t len);
static uint16_t ReadU16LE(const uint8_t *p);
static void WriteU16LE(uint8_t *p, uint16_t value);
static uint32_t ReadU32LE(const uint8_t *p);
static void WriteU32LE(uint8_t *p, uint32_t value);
static void WriteI32LE(uint8_t *p, int32_t value);
static float ReadFloatLE(const uint8_t *p);
static void WriteFloatLE(uint8_t *p, float value);
static Motor_t *MotorFromId(uint8_t motor_id);
static int32_t EncoderCountFromId(uint8_t motor_id);

static void UART_ResetParser(void);
static void UART_ProcessByte(uint8_t b);
static void UART_ProcessFrame(const uint8_t *frame, uint8_t len);
static void UART_StartReceiveToIdleDMA(void);
static bool UART_QueueFrameRaw(const uint8_t *frame, uint8_t len);
static bool UART_QueuePacket(uint8_t type, uint16_t seq,
                             const uint8_t *payload, uint8_t payload_len);
static void UART_QueueError(uint16_t request_seq, uint8_t request_type, uint8_t code);
static void UART_QueueFeedback(void);
static void UART_QueueDiagnostics(uint16_t request_seq);
static void UART_QueueSysidSample(void);
static void UART_QueuePIDResponse(uint16_t request_seq, uint8_t motor_id, const PIDF_t *pid);
static void UART_QueueInfoResponse(uint16_t request_seq);
static void UART_ServiceTx(void);

static void CV_EncoderUpdate(void);
static void App_ControlUpdate(void);
static void App_UpdateDebugSnapshot(void);

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

    return (int32_t)(current - previous);
}

static void Motor_UpdateRPM_MT(Motor_t *motor, uint32_t now_cycles)
{
    uint32_t edge_count;
    uint32_t edge_time;
    uint32_t edge_sequence;

    uint32_t primask = App_EnterCritical();
    edge_count = motor->mt.edge_count;
    edge_time = motor->mt.edge_time_cycles;
    edge_sequence = motor->mt.edge_sequence;
    App_ExitCritical(primask);

    if (edge_sequence != motor->mt.prev_edge_sequence) {
        if (!motor->mt.initialized) {
            motor->mt.prev_edge_count = edge_count;
            motor->mt.prev_edge_time_cycles = edge_time;
            motor->mt.prev_edge_sequence = edge_sequence;
            motor->mt.initialized = true;
            motor->rpm = 0.0f;
            return;
        }

        const int32_t delta_count =
            Motor_CountDifference(motor, edge_count, motor->mt.prev_edge_count);

        const uint32_t delta_cycles =
            edge_time - motor->mt.prev_edge_time_cycles;

        if (delta_cycles > 0U && delta_count != 0 && motor->counts_per_rev > 0.0f) {
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
            __HAL_TIM_SET_COMPARE(motor->pwm_timer, motor->pwm_ch_forward, duty);
            __HAL_TIM_SET_COMPARE(motor->pwm_timer, motor->pwm_ch_reverse, 0U);
        } else if (u < 0.0f) {
            __HAL_TIM_SET_COMPARE(motor->pwm_timer, motor->pwm_ch_forward, 0U);
            __HAL_TIM_SET_COMPARE(motor->pwm_timer, motor->pwm_ch_reverse, duty);
        } else {
            __HAL_TIM_SET_COMPARE(motor->pwm_timer, motor->pwm_ch_forward, 0U);
            __HAL_TIM_SET_COMPARE(motor->pwm_timer, motor->pwm_ch_reverse, 0U);
        }
        return;
    }

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

    __HAL_TIM_SET_COMPARE(motor->pwm_timer, motor->pwm_ch_forward, duty);
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
#ifdef __HAL_DBGMCU_FREEZE_IWDG
    __HAL_DBGMCU_FREEZE_IWDG();
#endif

    uint32_t reload = APP_WATCHDOG_TIMEOUT_MS;
    if (reload == 0U) {
        reload = 1U;
    }
    if (reload > 4095U) {
        reload = 4095U;
    }

    /*
     * STM32F4 IWDG must be started so its LSI clock is running before waiting
     * for prescaler/reload register updates. The previous sequence waited on
     * IWDG->SR before starting IWDG, which could leave App_Init() stuck forever
     * while TIM10 continued to run in interrupts.
     */
    IWDG->KR = 0xCCCCU; /* Start IWDG / LSI. */
    IWDG->KR = 0x5555U; /* Enable PR/RLR writes. */
    IWDG->PR = 0x03U;   /* Prescaler /32, nominally ~1 ms/tick at 32 kHz LSI. */
    IWDG->RLR = reload - 1U;

    /* Bound the register-update wait so initialization can never deadlock. */
    uint32_t guard = 1000000U;
    while ((IWDG->SR != 0U) && (guard > 0U)) {
        guard--;
    }

    IWDG->KR = 0xAAAAU; /* Reload counter. */
    g_app_debug.watchdog_started = 1U;
    g_app_debug.watchdog_update_timeout = (guard == 0U) ? 1U : 0U;
}

static void App_WatchdogRefresh(void)
{
    IWDG->KR = 0xAAAAU;
}

void App_EmergencyShutdown(void)
{
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
        App_ZeroReferences();
        Motor_StopOutput(&motorWR);
        Motor_StopOutput(&motorWL);
        Motor_StopOutput(&motorBR);
        Motor_StopOutput(&motorBL);
        Motor_StopOutput(&motorCV);
        App_SetDriverEnable(false);

        armed = false;
        sysid_active = false;
        sysid_duty = 0.0f;

        if (!previous_estop_active) {
            Motor_ResetAllPID();
        }
    }

    previous_estop_active = active;
}

static void App_ZeroReferences(void)
{
    motorWR.ref_rpm = 0.0f;
    motorWL.ref_rpm = 0.0f;
    motorBR.ref_rpm = 0.0f;
    motorBL.ref_rpm = 0.0f;
    motorCV.ref_rpm = 0.0f;
}

static void App_Disarm(void)
{
    armed = false;
    sysid_active = false;
    sysid_duty = 0.0f;

    App_ZeroReferences();
    Motor_StopOutput(&motorWR);
    Motor_StopOutput(&motorWL);
    Motor_StopOutput(&motorBR);
    Motor_StopOutput(&motorBL);
    Motor_StopOutput(&motorCV);
    Motor_ResetAllPID();
    App_SetDriverEnable(false);
}

static bool App_TryArm(void)
{
    if (estop_active) {
        return false;
    }

    App_ZeroReferences();
    Motor_ResetAllPID();
    comm_timeout_active = false;
    last_setpoint_ms = HAL_GetTick();
    armed = true;
    App_SetDriverEnable(true);
    return true;
}

static void Sysid_Stop(bool disarm_after)
{
    sysid_active = false;
    sysid_duty = 0.0f;
    Motor_StopOutput(&motorWR);
    Motor_StopOutput(&motorWL);
    Motor_StopOutput(&motorBR);
    Motor_StopOutput(&motorBL);
    Motor_StopOutput(&motorCV);

    if (disarm_after) {
        App_Disarm();
    }
}

/* ============================== UART ====================================== */
static uint16_t CRC16_CCITT_FALSE(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;

    for (uint16_t i = 0U; i < len; ++i) {
        crc ^= (uint16_t)data[i] << 8U;

        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            if ((crc & 0x8000U) != 0U) {
                crc = (uint16_t)((crc << 1U) ^ 0x1021U);
            } else {
                crc <<= 1U;
            }
        }
    }

    return crc;
}

static uint16_t ReadU16LE(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8U));
}

static void WriteU16LE(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)(value & 0xFFU);
    p[1] = (uint8_t)((value >> 8U) & 0xFFU);
}

static uint32_t ReadU32LE(const uint8_t *p)
{
    return ((uint32_t)p[0]) |
           ((uint32_t)p[1] << 8U) |
           ((uint32_t)p[2] << 16U) |
           ((uint32_t)p[3] << 24U);
}

static void WriteU32LE(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)(value & 0xFFU);
    p[1] = (uint8_t)((value >> 8U) & 0xFFU);
    p[2] = (uint8_t)((value >> 16U) & 0xFFU);
    p[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

static void WriteI32LE(uint8_t *p, int32_t value)
{
    WriteU32LE(p, (uint32_t)value);
}

static float ReadFloatLE(const uint8_t *p)
{
    const uint32_t bits = ReadU32LE(p);
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

static void WriteFloatLE(uint8_t *p, float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    WriteU32LE(p, bits);
}

static Motor_t *MotorFromId(uint8_t motor_id)
{
    switch (motor_id) {
        case MOTOR_ID_WR: return &motorWR;
        case MOTOR_ID_WL: return &motorWL;
        case MOTOR_ID_BR: return &motorBR;
        case MOTOR_ID_BL: return &motorBL;
        case MOTOR_ID_CV: return &motorCV;
        default: return NULL;
    }
}

static int32_t EncoderCountFromId(uint8_t motor_id)
{
    switch (motor_id) {
        case MOTOR_ID_WR: return (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
        case MOTOR_ID_WL: return (int32_t)__HAL_TIM_GET_COUNTER(&htim5);
        case MOTOR_ID_BR: return (int32_t)(int16_t)__HAL_TIM_GET_COUNTER(&htim1);
        case MOTOR_ID_BL: return (int32_t)(int16_t)__HAL_TIM_GET_COUNTER(&htim4);
        case MOTOR_ID_CV: return cv_encoder_count;
        default: return 0;
    }
}

static void UART_ResetParser(void)
{
    rx_state = RX_WAIT_SOF1;
    rx_index = 0U;
    rx_payload_len = 0U;
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
                rx_state = RX_COLLECT_HEADER;
            } else if (b == SOF1) {
                rx_frame[0] = SOF1;
                rx_index = 1U;
            } else {
                UART_ResetParser();
            }
            break;

        case RX_COLLECT_HEADER:
            if (rx_index >= UART_HEADER_LEN) {
                UART_ResetParser();
                break;
            }

            rx_frame[rx_index++] = b;

            if (rx_index == UART_HEADER_LEN) {
                rx_payload_len = rx_frame[6];

                if (rx_payload_len > UART_MAX_PAYLOAD_LEN) {
                    UART_RecordInvalidFrame();
                    UART_ResetParser();
                    break;
                }

                rx_expected_len =
                    (uint8_t)(UART_HEADER_LEN + rx_payload_len + UART_CRC_LEN);
                rx_state = RX_COLLECT_FRAME;
            }
            break;

        case RX_COLLECT_FRAME:
            if (rx_index >= sizeof(rx_frame)) {
                UART_RecordInvalidFrame();
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

static bool UART_QueueFrameRaw(const uint8_t *frame, uint8_t len)
{
    if (frame == NULL || len == 0U || len > UART_FRAME_MAX_LEN) {
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

static bool UART_QueuePacket(uint8_t type, uint16_t seq,
                             const uint8_t *payload, uint8_t payload_len)
{
    if (payload_len > UART_MAX_PAYLOAD_LEN) {
        return false;
    }

    uint8_t frame[UART_FRAME_MAX_LEN];
    const uint8_t len =
        (uint8_t)(UART_HEADER_LEN + payload_len + UART_CRC_LEN);

    frame[0] = SOF1;
    frame[1] = SOF2;
    frame[2] = APP_PROTOCOL_VERSION;
    frame[3] = type;
    WriteU16LE(&frame[4], seq);
    frame[6] = payload_len;

    if (payload_len > 0U && payload != NULL) {
        memcpy(&frame[7], payload, payload_len);
    }

    const uint16_t crc =
        CRC16_CCITT_FALSE(&frame[2], (uint16_t)(5U + payload_len));
    WriteU16LE(&frame[7U + payload_len], crc);

    return UART_QueueFrameRaw(frame, len);
}

static void UART_QueueError(uint16_t request_seq, uint8_t request_type, uint8_t code)
{
    uint8_t payload[2] = {request_type, code};
    (void)UART_QueuePacket(TYPE_ERROR, request_seq, payload, sizeof(payload));
}

static void UART_QueueFeedback(void)
{
    uint8_t payload[50];
    uint8_t *p = payload;

    WriteU32LE(p, g_app_debug.control_tick); p += 4;
    WriteU32LE(p, g_app_debug.status_flags); p += 4;
    WriteU16LE(p, last_setpoint_seq); p += 2;

    WriteI32LE(p, EncoderCountFromId(MOTOR_ID_WR)); p += 4;
    WriteI32LE(p, EncoderCountFromId(MOTOR_ID_WL)); p += 4;
    WriteI32LE(p, EncoderCountFromId(MOTOR_ID_BR)); p += 4;
    WriteI32LE(p, EncoderCountFromId(MOTOR_ID_BL)); p += 4;
    WriteI32LE(p, EncoderCountFromId(MOTOR_ID_CV)); p += 4;

    WriteFloatLE(p, motorWR.rpm); p += 4;
    WriteFloatLE(p, motorWL.rpm); p += 4;
    WriteFloatLE(p, motorBR.rpm); p += 4;
    WriteFloatLE(p, motorBL.rpm); p += 4;
    WriteFloatLE(p, motorCV.rpm); p += 4;

    (void)UART_QueuePacket(TYPE_FEEDBACK, tx_sequence++, payload, sizeof(payload));
}

static void UART_QueueDiagnostics(uint16_t request_seq)
{
    uint8_t payload[32];
    uint8_t *p = payload;

    WriteU32LE(p, HAL_GetTick()); p += 4;
    WriteU32LE(p, g_app_debug.reset_flags_raw); p += 4;
    WriteU32LE(p, g_app_debug.uart_rx_frames_ok); p += 4;
    WriteU32LE(p, g_app_debug.uart_crc_errors); p += 4;
    WriteU32LE(p, g_app_debug.uart_invalid_frames); p += 4;
    WriteU32LE(p, g_app_debug.uart_errors); p += 4;
    WriteU32LE(p, g_app_debug.uart_tx_queue_drops); p += 4;

    *p++ = APP_FW_VERSION_MAJOR;
    *p++ = APP_FW_VERSION_MINOR;
    *p++ = APP_FW_VERSION_PATCH;
    *p++ = APP_PROTOCOL_VERSION;

    (void)UART_QueuePacket(TYPE_DIAGNOSTICS, request_seq, payload, sizeof(payload));
}

static void UART_QueueSysidSample(void)
{
    if (!sysid_active) {
        return;
    }

    Motor_t *motor = MotorFromId(sysid_motor_id);
    if (motor == NULL) {
        return;
    }

    uint8_t payload[23];
    uint8_t *p = payload;

    WriteU16LE(p, sysid_command_seq); p += 2;
    WriteU32LE(p, g_app_debug.control_tick); p += 4;
    *p++ = sysid_motor_id;
    WriteFloatLE(p, sysid_duty); p += 4;
    WriteFloatLE(p, motor->rpm); p += 4;
    WriteI32LE(p, EncoderCountFromId(sysid_motor_id)); p += 4;
    WriteU32LE(p, g_app_debug.status_flags);

    (void)UART_QueuePacket(TYPE_SYSID_SAMPLE, tx_sequence++, payload, sizeof(payload));
}

static void UART_QueuePIDResponse(uint16_t request_seq, uint8_t motor_id, const PIDF_t *pid)
{
    uint8_t payload[17];
    payload[0] = motor_id;
    WriteFloatLE(&payload[1], pid->Kp);
    WriteFloatLE(&payload[5], pid->Ki);
    WriteFloatLE(&payload[9], pid->Kd);
    WriteFloatLE(&payload[13], pid->Tf);
    (void)UART_QueuePacket(TYPE_PID_RESPONSE, request_seq, payload, sizeof(payload));
}

static void UART_QueueInfoResponse(uint16_t request_seq)
{
    const uint8_t payload[4] = {
        APP_FW_VERSION_MAJOR,
        APP_FW_VERSION_MINOR,
        APP_FW_VERSION_PATCH,
        APP_PROTOCOL_VERSION
    };
    (void)UART_QueuePacket(TYPE_INFO_RESPONSE, request_seq, payload, sizeof(payload));
}

static void UART_ProcessFrame(const uint8_t *frame, uint8_t len)
{
    if (len < (UART_HEADER_LEN + UART_CRC_LEN) ||
        frame[0] != SOF1 || frame[1] != SOF2) {
        UART_RecordInvalidFrame();
        return;
    }

    const uint8_t version = frame[2];
    const uint8_t type = frame[3];
    const uint16_t seq = ReadU16LE(&frame[4]);
    const uint8_t payload_len = frame[6];

    if (len != (uint8_t)(UART_HEADER_LEN + payload_len + UART_CRC_LEN)) {
        UART_RecordInvalidFrame();
        return;
    }

    const uint16_t received_crc = ReadU16LE(&frame[7U + payload_len]);
    const uint16_t calculated_crc =
        CRC16_CCITT_FALSE(&frame[2], (uint16_t)(5U + payload_len));

    if (received_crc != calculated_crc) {
        g_app_debug.uart_crc_errors++;
        return;
    }

    if (version != APP_PROTOCOL_VERSION) {
        UART_RecordInvalidFrame();
        UART_QueueError(seq, type, ERROR_BAD_VERSION);
        return;
    }

    g_app_debug.uart_rx_frames_ok++;
    const uint8_t *payload = &frame[7];

    switch (type) {
        case TYPE_ARM:
            if (payload_len != 0U) {
                UART_QueueError(seq, type, ERROR_BAD_LENGTH);
                return;
            }
            if (estop_active) {
                UART_QueueError(seq, type, ERROR_ESTOP_ACTIVE);
                return;
            }
            (void)App_TryArm();
            return;

        case TYPE_DISARM:
            if (payload_len != 0U) {
                UART_QueueError(seq, type, ERROR_BAD_LENGTH);
                return;
            }
            comm_timeout_active = false;
            App_Disarm();
            return;

        case TYPE_SETPOINT:
            if (payload_len != 20U) {
                UART_QueueError(seq, type, ERROR_BAD_LENGTH);
                return;
            }
            if (!armed || estop_active) {
                UART_QueueError(seq, type,
                                estop_active ? ERROR_ESTOP_ACTIVE : ERROR_NOT_ARMED);
                return;
            }
            {
                float v[MOTOR_COUNT];
                for (uint8_t i = 0U; i < MOTOR_COUNT; ++i) {
                    v[i] = ReadFloatLE(&payload[4U * i]);
                }

                if (!RefWithinLimit(v[0], APP_MAX_RPM_WR) ||
                    !RefWithinLimit(v[1], APP_MAX_RPM_WL) ||
                    !RefWithinLimit(v[2], APP_MAX_RPM_BR) ||
                    !RefWithinLimit(v[3], APP_MAX_RPM_BL) ||
                    !RefWithinLimit(v[4], APP_MAX_RPM_CV)) {
                    app_fault_flags |= APP_STATUS_INVALID_COMMAND;
                    UART_RecordInvalidFrame();
                    UART_QueueError(seq, type, ERROR_BAD_VALUE);
                    return;
                }

                motorWR.ref_rpm = v[0];
                motorWL.ref_rpm = v[1];
                motorBR.ref_rpm = v[2];
                motorBL.ref_rpm = v[3];
                motorCV.ref_rpm = v[4];
                last_setpoint_ms = HAL_GetTick();
                last_setpoint_seq = seq;
                comm_timeout_active = false;
            }
            return;

        case TYPE_SYSID_COMMAND:
            if (payload_len != 5U) {
                UART_QueueError(seq, type, ERROR_BAD_LENGTH);
                return;
            }
            if (!armed || estop_active) {
                UART_QueueError(seq, type,
                                estop_active ? ERROR_ESTOP_ACTIVE : ERROR_NOT_ARMED);
                return;
            }
            {
                const uint8_t motor_id = payload[0];
                const float duty = ReadFloatLE(&payload[1]);
                if (MotorFromId(motor_id) == NULL) {
                    UART_QueueError(seq, type, ERROR_BAD_MOTOR);
                    return;
                }
                if (!isfinite(duty) || duty < -1.0f || duty > 1.0f) {
                    UART_QueueError(seq, type, ERROR_BAD_VALUE);
                    return;
                }

                if (!sysid_active || sysid_motor_id != motor_id) {
                    App_ZeroReferences();
                    Motor_ResetAllPID();
                    Motor_StopOutput(&motorWR);
                    Motor_StopOutput(&motorWL);
                    Motor_StopOutput(&motorBR);
                    Motor_StopOutput(&motorBL);
                    Motor_StopOutput(&motorCV);
                }

                sysid_active = true;
                sysid_motor_id = motor_id;
                sysid_duty = duty;
                sysid_command_seq = seq;
                last_sysid_cmd_ms = HAL_GetTick();
            }
            return;

        case TYPE_SYSID_STOP:
            if (payload_len != 0U) {
                UART_QueueError(seq, type, ERROR_BAD_LENGTH);
                return;
            }
            Sysid_Stop(false);
            return;

        case TYPE_PID_SET:
            if (payload_len != 17U) {
                UART_QueueError(seq, type, ERROR_BAD_LENGTH);
                return;
            }
            {
                const uint8_t motor_id = payload[0];
                Motor_t *motor = MotorFromId(motor_id);
                if (motor == NULL) {
                    UART_QueueError(seq, type, ERROR_BAD_MOTOR);
                    return;
                }

                const float Kp = ReadFloatLE(&payload[1]);
                const float Ki = ReadFloatLE(&payload[5]);
                const float Kd = ReadFloatLE(&payload[9]);
                const float Tf = ReadFloatLE(&payload[13]);

                if (!isfinite(Kp) || !isfinite(Ki) || !isfinite(Kd) || !isfinite(Tf) ||
                    Kp < 0.0f || Ki < 0.0f || Kd < 0.0f || Tf < 0.0f ||
                    Kp > APP_PID_GAIN_MAX || Ki > APP_PID_GAIN_MAX ||
                    Kd > APP_PID_GAIN_MAX || Tf > APP_PID_TF_MAX_S) {
                    UART_QueueError(seq, type, ERROR_BAD_VALUE);
                    return;
                }

                PIDF_SetTuning(&motor->pid, Kp, Ki, Kd, Tf);
                UART_QueuePIDResponse(seq, motor_id, &motor->pid);
            }
            return;

        case TYPE_PID_GET:
            if (payload_len != 1U) {
                UART_QueueError(seq, type, ERROR_BAD_LENGTH);
                return;
            }
            {
                const uint8_t motor_id = payload[0];
                Motor_t *motor = MotorFromId(motor_id);
                if (motor == NULL) {
                    UART_QueueError(seq, type, ERROR_BAD_MOTOR);
                    return;
                }
                UART_QueuePIDResponse(seq, motor_id, &motor->pid);
            }
            return;

        case TYPE_DIAGNOSTICS:
            if (payload_len != 0U) {
                UART_QueueError(seq, type, ERROR_BAD_LENGTH);
                return;
            }
            UART_QueueDiagnostics(seq);
            return;

        case TYPE_INFO_REQUEST:
            if (payload_len != 0U) {
                UART_QueueError(seq, type, ERROR_BAD_LENGTH);
                return;
            }
            UART_QueueInfoResponse(seq);
            UART_QueueDiagnostics(seq);
            return;

        default:
            UART_RecordInvalidFrame();
            UART_QueueError(seq, type, ERROR_BAD_VALUE);
            return;
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
        __HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_HT);
    }
}

static void UART_ServiceTx(void)
{
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

    /*
     * TX intentionally uses polling rather than DMA.
     *
     * At 1 Mbaud a maximum-size protocol frame takes < 1 ms on the wire.
     * This runs only from App_Task() in the main loop, so TIM10 and encoder
     * interrupts still preempt it. RX remains DMA based.
     */
    const HAL_StatusTypeDef status =
        HAL_UART_Transmit(&huart6, tx_dma_frame, len, 2U);

    if (status == HAL_OK) {
        primask = App_EnterCritical();
        tx_tail = (uint8_t)((tail + 1U) % TX_QUEUE_DEPTH);
        g_app_debug.uart_tx_frames_ok++;
        App_ExitCritical(primask);
    } else {
        g_app_debug.uart_tx_errors++;
        app_fault_flags |= APP_STATUS_UART_ERROR_SEEN;
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
static void App_ControlUpdate(void)
{
    g_app_debug.control_tick++;

    App_SafetyService();

    const uint32_t now_cycles = DWT->CYCCNT;
    Motor_UpdateRPM_MT(&motorWR, now_cycles);
    Motor_UpdateRPM_MT(&motorWL, now_cycles);
    Motor_UpdateRPM_MT(&motorBR, now_cycles);
    Motor_UpdateRPM_MT(&motorBL, now_cycles);
    Motor_UpdateRPM_MT(&motorCV, now_cycles);

    if (estop_active) {
        App_UpdateDebugSnapshot();
        UART_QueueFeedback();
        return;
    }

    if (!armed) {
        App_ZeroReferences();
        Motor_StopOutput(&motorWR);
        Motor_StopOutput(&motorWL);
        Motor_StopOutput(&motorBR);
        Motor_StopOutput(&motorBL);
        Motor_StopOutput(&motorCV);
        App_UpdateDebugSnapshot();
        UART_QueueFeedback();
        return;
    }

    if (sysid_active) {
        if ((HAL_GetTick() - last_sysid_cmd_ms) > APP_TUNING_TIMEOUT_MS) {
            comm_timeout_active = true;
            Sysid_Stop(true);
            App_UpdateDebugSnapshot();
            UART_QueueFeedback();
            return;
        }

        Motor_StopOutput(&motorWR);
        Motor_StopOutput(&motorWL);
        Motor_StopOutput(&motorBR);
        Motor_StopOutput(&motorBL);
        Motor_StopOutput(&motorCV);

        Motor_t *motor = MotorFromId(sysid_motor_id);
        if (motor != NULL) {
            Motor_ApplyDutyNormalized(motor, sysid_duty);
        }

        App_UpdateDebugSnapshot();
        UART_QueueSysidSample();
        UART_QueueFeedback();
        return;
    }

    if ((HAL_GetTick() - last_setpoint_ms) > APP_COMM_TIMEOUT_MS) {
        comm_timeout_active = true;
        App_Disarm();
        App_UpdateDebugSnapshot();
        UART_QueueFeedback();
        return;
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

    App_UpdateDebugSnapshot();
    UART_QueueFeedback();
}

static void App_UpdateDebugSnapshot(void)
{
    uint32_t status = app_fault_flags;

    if (armed) {
        status |= APP_STATUS_ARMED;
    }
    if (estop_active) {
        status |= APP_STATUS_ESTOP;
    }
    if (comm_timeout_active) {
        status |= APP_STATUS_COMM_TIMEOUT;
    }
    if (sysid_active) {
        status |= APP_STATUS_SYSID_MODE;
    }

    g_app_debug.status_flags = status;
    g_app_debug.last_setpoint_seq = last_setpoint_seq;
    g_app_debug.sysid_command_seq = sysid_command_seq;
    g_app_debug.sysid_motor_id = sysid_motor_id;
    g_app_debug.sysid_duty = sysid_duty;

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
     * USART6 is expected to run at the protocol baud rate.
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

    /* Boot is always DISARMED regardless of ESTOP state. */
    armed = false;
    App_SetDriverEnable(false);
    last_setpoint_ms = HAL_GetTick();
    comm_timeout_active = true;
    last_setpoint_seq = 0U;

    sysid_active = false;
    sysid_motor_id = MOTOR_ID_WR;
    sysid_duty = 0.0f;
    sysid_command_seq = 0U;
    last_sysid_cmd_ms = HAL_GetTick();

    UART_StartReceiveToIdleDMA();

    /*
     * Start the watchdog before TIM10 so App_Init cannot leave the controller
     * ISR running while main() is still blocked in watchdog initialization.
     */
    App_WatchdogInit();

    /* TIM10 is the 100 Hz controller scheduler. */
    if (HAL_TIM_Base_Start_IT(&htim10) != HAL_OK) {
        Error_Handler();
    }

    App_UpdateDebugSnapshot();
}

void App_Task(void)
{
    static uint32_t last_watchdog_control_tick = 0U;

    g_app_debug.main_loop_count++;

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

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART6) {
        return;
    }

    g_app_debug.uart_errors++;
    app_fault_flags |= APP_STATUS_UART_ERROR_SEEN;

    HAL_UART_DMAStop(&huart6);
    UART_ResetParser();
    UART_StartReceiveToIdleDMA();
}
