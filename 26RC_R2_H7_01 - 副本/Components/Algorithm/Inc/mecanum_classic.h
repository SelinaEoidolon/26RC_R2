#ifndef MECANUM_KINEMATICS_H
#define MECANUM_KINEMATICS_H

typedef struct
{
    float vx;   // m/s
    float vy;   // m/s
    float vw;   // rad/s
} ChassisVel_t;

typedef struct
{
    float fl;
    float fr;
    float bl;
    float br;
} WheelSpeed_t;

typedef struct
{
    float wheel_radius;     // r  (m)
    float wheel_base;       // L  (m) 前后轮距
    float wheel_track;      // W  (m) 左右轮距
    float max_wheel_speed;  // 最大轮速 (rad/s 或 rpm，和输出一致)
} MecanumParam_t;

extern ChassisVel_t total_vel ;
extern WheelSpeed_t total_speed;
extern MecanumParam_t mecParam;

void Mecanum_Calc(
    const ChassisVel_t *chassis,
    const MecanumParam_t *param,
    WheelSpeed_t *wheel);

#endif
