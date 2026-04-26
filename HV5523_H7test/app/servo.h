#ifndef _SERVO_H
#define _SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "tim.h"
#include <stdint.h>

/* 启动 TIM2 CH1 PWM 输出 */
void Servo_TIM2_CH1_Start(void);

/* 直接设置舵机脉宽，单位 us，例如 1500 表示 1.5ms */
void Servo_TIM2_CH1_SetPulseUs(uint16_t pulse_us);

/* 使用你表里的 0~4095 count 设置 */
void Servo_TIM2_CH1_SetCount(float count);

/* 使用“相对中心角”设置 */
void Servo_TIM2_CH1_SetRelativeDeg(float rel_deg);

/* 获取当前 compare 值 */
uint16_t Servo_TIM2_CH1_GetCompare(void);

#ifdef __cplusplus
}
#endif

#endif