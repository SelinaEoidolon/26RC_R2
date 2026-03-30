#ifndef __J3_CLAC_H_
#define __J3_CLAC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

	#define _USE_MATH_DEFINES
#include "math.h"

#define DEG_TO_RAD        M_PI / 180.0f 
#define RAD_TO_DEG        180.0f / M_PI 


#define JOINT1_MAX_ANGLE  60.0f*5.0f*3591.0f/187.0f//暂定
#define JOINT1_MAX_RAD    60.0f*5.0f*3591.0f/187.0f
#define JOINT1_MIN_ANGLE  60.0f*5.0f*3591.0f/187.0f
#define JOINT1_MIN_RAD    60.0f*5.0f*3591.0f/187.0f

#define JOINT2_MAX_ANGLE       60.0f*5.0f*3591.0f/187.0f
#define JOINT2_MAX_RAD         60.0f*5.0f*3591.0f/187.0f
#define JOINT2_MAX_TOTALANGLE  290000
#define JOINT2_MIN_ANGLE       60.0f*5.0f*3591.0f/187.0f
#define JOINT2_MIN_RAD         60.0f*5.0f*3591.0f/187.0f
#define JOINT2_MIN_TOTALANGLE  81920

#define JOINT3_MAX_ANGLE       60.0f*5.0f*3591.0f/187.0f
#define JOINT3_MAX_RAD         60.0f*5.0f*3591.0f/187.0f
#define JOINT3_MAX_TOTALANGLE  90000
#define JOINT3_MIN_ANGLE       60.0f*5.0f*3591.0f/187.0f
#define JOINT3_MIN_RAD         60.0f*5.0f*3591.0f/187.0f
#define JOINT3_MIN_TOTALANGLE  -160000

#define TOOL1_OFFSET_X    0.0f
#define TOOL1_OFFSET_Y    0.0f
#define TOOL1_OFFSET_Z    0.0f

#define TOOL2_OFFSET_X    0.0f
#define TOOL2_OFFSET_Y    0.0f
#define TOOL2_OFFSET_Z    0.0f



/* 一些编译环境里没有 M_PI，这里补一个 */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* 浮点比较阈值 */
#define IK_EPS 1e-6f

/**
 * @brief 逆运动学返回状态
 */
typedef enum
{
    IK_OK = 0,              /* 求解成功 */
    IK_ERR_NULL = -1,       /* 空指针 */
    IK_ERR_PARAM = -2,      /* 参数非法 */
    IK_ERR_UNREACHABLE = -3 /* 目标点不可达 */
} IK_Status_t;

/**
 * @brief 单组模型角结果
 *
 * 这组角是运动学模型中的关节角，不是直接发给电机的角。
 */
typedef struct
{
    float theta1;   /* rad，joint1模型角 */
    float theta2;   /* rad，joint2模型角 */
    float theta3;   /* rad，joint3模型角 */
    uint8_t valid;  /* 1=有效，0=无效 */
} IK_ModelSolution3R_t;

/**
 * @brief 单组电机目标角
 *
 * 这是已经加入零位偏置和方向修正后的“最终电机目标角”。
 */
typedef struct
{
    float q1_cmd;   /* rad，发给 joint1 电机的目标角 */
    float q2_cmd;   /* rad，发给 joint2 电机的目标角 */
    float q3_cmd;   /* rad，发给 joint3 电机的目标角 */
    uint8_t valid;  /* 1=有效，0=无效 */
} IK_MotorSolution3R_t;

/**
 * @brief 单个关节的标定参数
 *
 * motor_ref:
 *   上电时记录到的实际电机角
 *
 * model_ref:
 *   上电姿态在“运动学模型”中的参考角
 *
 * dir:
 *   方向系数，取 +1 或 -1
 *   用来处理“电机正方向”和“模型正方向”是否一致
 */
typedef struct
{
    float motor_ref;  /* rad */
    float model_ref;  /* rad */
    int8_t dir;       /* +1 / -1 */
} JointCalib_t;

/**
 * @brief 逆解总结果
 */
typedef struct
{
    IK_Status_t status;      /* 求解状态 */
    uint8_t solution_count;  /* 有效解数量，通常为 0 / 1 / 2 */
    uint8_t base_singular;   /* 1 表示 x=y=0，theta1 不唯一 */

    IK_ModelSolution3R_t model_sol[2];
    IK_MotorSolution3R_t motor_sol[2];
} IK_Result3R_WithOffset_t;

/**
 * @brief 角度归一化到 [-pi, pi)
 */
float IK_NormalizeAngle(float angle_rad);

/**
 * @brief 角度转弧度
 */
float IK_DegToRad(float deg);

/**
 * @brief 弧度转角度
 */
float IK_RadToDeg(float rad);

/**
 * @brief 把模型角转换成电机目标角
 *
 * 公式：
 *   q_cmd = motor_ref + dir * (theta_model - model_ref)
 */
float IK_ModelToMotorCmd(float theta_model, const JointCalib_t *cal);

/**
 * @brief 3R机械臂逆解（输入连杆末端目标坐标，输出模型角和电机目标角）
 *
 * 位置模型：
 *   px = cos(theta1) * (-a2*sin(theta2) - a3*sin(theta2 + theta3))
 *   py = sin(theta1) * (-a2*sin(theta2) - a3*sin(theta2 + theta3))
 *   pz = d1 + a2*cos(theta2) + a3*cos(theta2 + theta3)
 *
 * @param x, y, z
 *        连杆末端目标点坐标（基座坐标系）
 *
 * @param d1
 *        基座高度偏置
 *
 * @param a2, a3
 *        两段活动连杆长度
 *
 * @param theta1_hint
 *        当 x=y=0 时，theta1 几何上不唯一，此时使用该参考值（模型角）
 *
 * @param j1, j2, j3
 *        三个关节的标定参数
 *
 * @param result
 *        输出逆解结果
 */
IK_Status_t Arm3R_IK_WithOffset(float x,
                                float y,
                                float z,
                                float d1,
                                float a2,
                                float a3,
                                float theta1_hint,
                                const JointCalib_t *j1,
                                const JointCalib_t *j2,
                                const JointCalib_t *j3,
                                IK_Result3R_WithOffset_t *result);

#ifdef __cplusplus
}
#endif




#endif   




