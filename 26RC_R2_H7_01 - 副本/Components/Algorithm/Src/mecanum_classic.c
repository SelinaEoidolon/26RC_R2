#include "mecanum_classic.h"
//#include "pid_user.h"

ChassisVel_t total_vel = {0,0,0};
WheelSpeed_t total_speed = {0,0,0,0};
MecanumParam_t mecParam = {0.150f,0.5f,0.405f,1500.0f};


static float abs_f(float x)
{
    return (x >= 0.0f) ? x : -x;
}

static float max_f(float a, float b)
{
    return (a > b) ? a : b;
}

/**
 * @brief 麦克纳姆轮逆运动学
 */
//void Mecanum_Calc(
//    const ChassisVel_t *chassis,
//    const MecanumParam_t *param,
//    WheelSpeed_t *wheel)
//{
//    float r = param->wheel_radius;
//    float L = param->wheel_base * 0.5f;
//    float W = param->wheel_track * 0.5f;

//    float vx = chassis->vx;
//    float vy = chassis->vy;
//    float vw = chassis->vw;//chassis->vw为目标角速度pid，pid(陀螺仪读取值，目标值)

//    //逆解算
//    float k = (L + W) * vw;

//    wheel->fl = ( vx - vy - k ) / r;
//    wheel->fr = ( vx + vy + k ) / r;
//    wheel->bl = ( vx + vy - k ) / r;
//    wheel->br = ( vx - vy + k ) / r;

//    //速度归一化
//    float max_val = 0.0f;

//    max_val = max_f(max_val, abs_f(wheel->fl));
//    max_val = max_f(max_val, abs_f(wheel->fr));
//    max_val = max_f(max_val, abs_f(wheel->bl));
//    max_val = max_f(max_val, abs_f(wheel->br));

//    if (max_val > param->max_wheel_speed)
//    {
//        float scale = param->max_wheel_speed / max_val;

//        wheel->fl *= scale;
//        wheel->fr *= scale;
//        wheel->bl *= scale;
//        wheel->br *= scale;
//    }
//}

void Mecanum_Calc(
    const ChassisVel_t *chassis,
    const MecanumParam_t *param,
    WheelSpeed_t *wheel)
{
    float r = param->wheel_radius;       // 轮子半径
    float L = param->wheel_base * 0.5f;  // 轴距的一半（前后轮中心到底盘中心的距离）
    float W = param->wheel_track * 0.5f; // 轮距的一半（左右轮中心到底盘中心的距离）

    // 提取底盘速度（chassis->vy=前进/后退，chassis->vx=左/右平移，chassis->vw=自转）
    float chassis_vx = chassis->vx;  // 新X轴：向右为正
    float chassis_vy = chassis->vy;  // 新Y轴：向上（前进）为正
    float chassis_vw = -chassis->vw;  // 自转：逆时针（左转）为正//chassis->vw为目标角速度pid，pid(陀螺仪读取值，目标值)

    // 逆解算核心公式（适配新坐标系）
    float k = (L + W) * chassis_vw;  // 自转项系数

    // 轮子转速计算（rad/s）：fl=左前、fr=右前、bl=左后、br=右后
    wheel->fl = (chassis_vy + chassis_vx - k) / r;  // 左前轮
    wheel->fr = (chassis_vy - chassis_vx + k) / r;  // 右前轮
    wheel->bl = (chassis_vy - chassis_vx - k) / r;  // 左后轮
    wheel->br = (chassis_vy + chassis_vx + k) / r;  // 右后轮

    // 速度归一化：防止单个轮子转速超过最大限制
    float max_val = 0.0f;
    max_val = max_f(max_val, abs_f(wheel->fl));
    max_val = max_f(max_val, abs_f(wheel->fr));
    max_val = max_f(max_val, abs_f(wheel->bl));
    max_val = max_f(max_val, abs_f(wheel->br));

    if (max_val > param->max_wheel_speed)
    {
        float scale = param->max_wheel_speed / max_val;
        wheel->fl *= scale;
        wheel->fr *= scale;
        wheel->bl *= scale;
        wheel->br *= scale;
    }
}