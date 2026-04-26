#include "servo.h"

/* CubeMX 生成的 TIM2 句柄 */
extern TIM_HandleTypeDef htim2;

/* 当前 compare 记录，方便调试 */
static uint16_t s_servo_compare = 1500;

/* 
 * 1号舵机标定参数（按你前面那张表）
 * center = 306
 * slope  = 2.33 count/deg
 * min    = 113.5
 * max    = 498.5
 */
#define SERVO1_CENTER_COUNT   (306.0f)
#define SERVO1_SLOPE          (2.33f)
#define SERVO1_MIN_COUNT      (113.5f)
#define SERVO1_MAX_COUNT      (498.5f)

/* us 安全范围，可按舵机实际能力调整 */
#define SERVO_MIN_US          (500U)
#define SERVO_MAX_US          (2500U)


/*========================= 内部工具函数 =========================*/

static float Servo_ClampFloat(float x, float min_v, float max_v)
{
    if (x < min_v) return min_v;
    if (x > max_v) return max_v;
    return x;
}

static uint16_t Servo_ClampU16(uint16_t x, uint16_t min_v, uint16_t max_v)
{
    if (x < min_v) return min_v;
    if (x > max_v) return max_v;
    return x;
}

/*
 * PCA9685 count -> us
 *
 * 20ms / 4096 = 4.8828125us
 *
 * 例如：
 * count = 306 -> 1494us 左右
 */
static uint16_t Servo_CountToUs(float count)
{
    float us;

    if (count < 0.0f)
        count = 0.0f;
    if (count > 4095.0f)
        count = 4095.0f;

    us = count * 20000.0f / 4096.0f;

    if (us < (float)SERVO_MIN_US)
        us = (float)SERVO_MIN_US;
    if (us > (float)SERVO_MAX_US)
        us = (float)SERVO_MAX_US;

    return (uint16_t)(us + 0.5f);
}


/*========================= 对外接口 =========================*/

void Servo_TIM2_CH1_Start(void)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

    /* 上电后先打到中心附近 */
    Servo_TIM2_CH1_SetCount(SERVO1_CENTER_COUNT);
}

void Servo_TIM2_CH1_SetPulseUs(uint16_t pulse_us)
{
    pulse_us = Servo_ClampU16(pulse_us, SERVO_MIN_US, SERVO_MAX_US);

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse_us);

    s_servo_compare = pulse_us;
}

void Servo_TIM2_CH1_SetCount(float count)
{
    uint16_t pulse_us;

    count = Servo_ClampFloat(count, SERVO1_MIN_COUNT, SERVO1_MAX_COUNT);

    pulse_us = Servo_CountToUs(count);

    Servo_TIM2_CH1_SetPulseUs(pulse_us);
}

void Servo_TIM2_CH1_SetRelativeDeg(float rel_deg)
{
    float count;

    /*
     * 按你的表来：
     * count = center + slope * relative_deg
     *
     * 例如：
     * rel_deg = 0   -> count = 306
     * rel_deg = -84 -> 接近 110
     */
    count = SERVO1_CENTER_COUNT + SERVO1_SLOPE * rel_deg;

    Servo_TIM2_CH1_SetCount(count);
}

uint16_t Servo_TIM2_CH1_GetCompare(void)
{
    return s_servo_compare;
}