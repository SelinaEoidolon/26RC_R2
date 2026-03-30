#ifndef __PID_USER_H
#define __PID_USER_H
#include "pid.h"
#include "include.h"

void PID_devices_Init(void);

float PID_velocity_realize_1(float set_speed,int i);
float PID_position_realize_1(float set_pos,int i);
float pid_call_1(float position,int i);

float PID_velocity_realize_2(float set_speed,int i);
float PID_position_realize_2(float set_pos,int i);
float pid_call_2(float position,int i);

float PID_speed_yaw(float set_speed);
float PID_position_yaw(float set_pos);
float pid_call_yaw(float position);

#endif























