#include "pid_user.h"

extern motor_measure_t motor_fdcan2[8];
extern motor_measure_t motor_fdcan3[8];

pid_type_def pid_v_2[8],pid_pos_2[8];
pid_type_def pid_v_3[8],pid_pos_3[8];

float motor_speed_3508_pid_2[4]    = {5, 0.02, 0.1, 0.2};//3508参数
float motor_position_3508_pid_2[4] = {0.2, 0, 1, 0};
float motor_speed_2006_pid_2[4]    = {9, 0.1, 0, 0.3};//2006参数
float motor_position_2006_pid_2[4] = {0.2, 0, 0, 0};

float motor_speed_3508_pid_3[4]    = {5, 0.02, 0.1, 0.2};//3508参数
float motor_position_3508_pid_3[4] = {0.2, 0, 1, 0};
float motor_speed_2006_pid_3[4]    = {9, 0.1, 0, 0.3};//2006参数
float motor_position_2006_pid_3[4] = {0.2, 0, 0, 0};


#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }


//PID初始化
void PID_devices_Init(void)
{
	for(int i=0;i<4;i++)
	{

		
		PID_init(&pid_v_2[i], PID_POSITION, motor_speed_3508_pid_2, 10000, 6000);
		PID_init(&pid_pos_2[i], PID_POSITION, motor_position_3508_pid_2, 1000, 300);
		
		PID_init(&pid_v_3[i], PID_POSITION, motor_speed_3508_pid_3, 10000, 6000);
		PID_init(&pid_pos_3[i], PID_POSITION, motor_position_3508_pid_3, 1000, 300);
		
		
	}
	

	

}


float PID_velocity_realize_2(float set_speed,int i)
{
		PID_calc(&pid_v_2[i-1],motor_fdcan2[i-1].speed_rpm , set_speed);
		return pid_v_2[i-1].out;
}

float PID_position_realize_2(float set_pos,int i)
{

		PID_calc(&pid_pos_2[i-1],motor_fdcan2[i-1].total_angle , set_pos);
		return pid_pos_2[i-1].out;

}

float pid_call_2(float position,int i)
{
		return PID_velocity_realize_2(PID_position_realize_2(position,i),i);
}



float PID_velocity_realize_3(float set_speed,int i)
{
		PID_calc(&pid_v_3[i-1],motor_fdcan3[i-1].speed_rpm , set_speed);
		return pid_v_3[i-1].out;
}

float PID_position_realize_3(float set_pos,int i)
{

		PID_calc(&pid_pos_3[i-1],motor_fdcan3[i-1].total_angle , set_pos);
		return pid_pos_3[i-1].out;

}

float pid_call_3(float position,int i)
{
		return PID_velocity_realize_3(PID_position_realize_3(position,i),i);
}






