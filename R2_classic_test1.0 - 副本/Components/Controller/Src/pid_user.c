#include "pid_user.h"
#include "imu.h"


//extern motor_measure_t motor_fdcan1[8];
extern motor_measure_t motor_fdcan2[8];

//pid_type_def pid_v_1[8],pid_pos_1[8];
pid_type_def pid_v_2[8],pid_pos_2[8];

float motor_speed_3508_pid[3] = {10, 0.1, 0};//3508参数
float motor_position_3508_pid[3] = {0.2, 0, 1};
float motor_speed_2006_pid[3] = {9, 0.1, 0};//2006参数
float motor_position_2006_pid[3] = {0.2, 0, 0};

extern IMU_Data_t imu_data ;

pid_type_def pid_yaw_speed,pid_yaw_position;

float imu_yaw_speed_pid[3] = {10, 0.1, 0};//yaw轴参数暂定
float imu_yaw_position_pid[3] = {0.2, 0, 1};




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
//    PID_init(&pid_v_1[i], PID_POSITION, motor_speed_3508_pid, 10000, 6000);
//		PID_init(&pid_pos_1[i], PID_POSITION, motor_position_3508_pid, 1000, 300);
		
		PID_init(&pid_v_2[i], PID_POSITION, motor_speed_3508_pid, 10000, 6000);
		PID_init(&pid_pos_2[i], PID_POSITION, motor_position_3508_pid, 1000, 300);
	}
	
	PID_init(&pid_yaw_speed, PID_POSITION, imu_yaw_speed_pid, 400, 300);
	PID_init(&pid_yaw_position, PID_POSITION, imu_yaw_position_pid, 400, 300);
	
	while(0){
//	for(int i=4;i<8;i++)
//	{		
////    PID_init(&pid_v_1[i], PID_POSITION, motor_speed_3508_pid, 10000, 6000);
////		PID_init(&pid_pos_1[i], PID_POSITION, motor_position_3508_pid, 400, 300);
//		
//		PID_init(&pid_v_2[i], PID_POSITION, motor_speed_3508_pid, 10000, 6000);
//		PID_init(&pid_pos_2[i], PID_POSITION, motor_position_3508_pid, 400, 300);
//	}
	}
}

float PID_speed_yaw(float set_speed)
{
	
		PID_calc(&pid_yaw_speed,imu_data.gyro_z , set_speed);
		return pid_yaw_speed.out;
	
}

float PID_position_yaw(float set_pos)
{
	
		PID_calc(&pid_yaw_position,imu_data.yaw , set_pos);
		return pid_yaw_position.out;
	
}

float pid_call_yaw(float position)
{
	
		return PID_speed_yaw(PID_position_yaw(position));
	
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




//float PID_velocity_realize_1(float set_speed,int i)
//{
//		PID_calc(&pid_v_1[i-1],motor_fdcan1[i-1].speed_rpm , set_speed);
//		return pid_v_1[i-1].out;
//}

//float PID_position_realize_1(float set_pos,int i)
//{

//		PID_calc(&pid_pos_1[i-1],motor_fdcan1[i-1].total_angle , set_pos);
//		return pid_pos_1[i-1].out;

//}

//float pid_call_1(float position,int i)
//{
//		return PID_velocity_realize_1(PID_position_realize_1(position,i),i);
//}










