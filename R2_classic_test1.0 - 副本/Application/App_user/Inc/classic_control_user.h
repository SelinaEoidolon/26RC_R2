#ifndef CLASSIC_CONTROL_USER_H  
#define CLASSIC_CONTROL_USER_H

#include "leg.h"
#include "mecanum_classic.h"
#include "include.h"
#include "imu.h"
#include "math.h"

typedef enum : uint8_t{
    RobotSystem = 0,  // 机器人坐标系指令（值为0，可省略，默认从0开始）
    WorldSystem       // 世界坐标系指令（值自动为1）
} mac_cmd;

extern float RX_theta ;
extern ChassisVel_t classic_speed;


uint8_t Classic_control(uint8_t mac_cmd,ChassisVel_t *total_vel,uint8_t leg_cmd);







#endif // CLASSIC_CONTROL_USER_H  



