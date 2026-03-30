#include "classic_control_user.h"
#include "leg.h"
#include "mecanum_classic.h"
#include "include.h"
#include "imu.h"
#include "math.h"

extern ChassisVel_t total_vel ;
extern WheelSpeed_t total_speed;
extern MecanumParam_t mecParam;
extern IMU_Data_t imu_data ;

//uint8_t mac_cmd = 0;
//uint8_t leg_cmd = 0;
//ChassisVel_t classic_speed;
//float RX_theta = 0;

/*
uint8_t mac_cmd 麦轮底盘运动模式指令
float V_x V_y V_yaw 底盘三个速度macArr

uint8_t leg_cmd 腿部运动指令
//float  highF highB 前后腿高度（暂定）
//float  theta[6] 前后腿高度对应角度
**/






