#include "cmsis_os.h"
#include "CAN_Task.h"
#include "include.h"
#include "dm_motor_ctrl.h"
#include "bsp_tick.h"
#include "bsp_mcu.h"
#include "leg.h"
#include "mecanum_classic.h"
#include "CRC.h"
#include "bsp_uart.h"


//static void FDCAN1_CMD_1(void);
extern WheelSpeed_t total_speed ;
//腿电机角度 rad
extern leg lf_leg ;//前腿
extern leg rf_leg ;

extern float lb_leg ;//后腿
extern float rb_leg ;


void CAN_Task(void const * argument)
{
  /* USER CODE BEGIN CAN_Task */
  /* Infinite loop */
  for(;;)
  {
//		FDCAN1_CMD_1();
		FDCAN2_CMD_1(PID_velocity_realize_2(-total_speed.fl,1),
				         PID_velocity_realize_2(total_speed.fr,2),
				         PID_velocity_realize_2(total_speed.bl,3),
				         PID_velocity_realize_2(-total_speed.br,4)
		);
//		FDCAN2_CMD_1(PID_velocity_realize_2(1000,1),
//				         PID_velocity_realize_2(1000,2),
//				         PID_velocity_realize_2(1000,3),
//				         PID_velocity_realize_2(1000,4)
//		);
//		FDCAN2_CMD_1(pid_call_2(218.463f,1),
//				         pid_call_2(218.463f,2),
//				         pid_call_2(218.463f,3),
//				         pid_call_2(218.463f,4)
//		);
		//四条腿发送任务在main定时器回调函数
		pos_ctrl(&hfdcan1,motor[Motor1].id,lf_leg.theta1,0.5);
		pos_ctrl(&hfdcan1,motor[Motor2].id,lf_leg.theta2,0.5);
		pos_ctrl(&hfdcan1,motor[Motor3].id,rf_leg.theta1,0.5);
		pos_ctrl(&hfdcan1,motor[Motor4].id,rf_leg.theta2,0.5);
		pos_ctrl(&hfdcan1,motor[Motor5].id,lb_leg,2);
		pos_ctrl(&hfdcan1,motor[Motor6].id,rb_leg,2);
		
//		pos_ctrl(&hfdcan1,motor[Motor1].id,0,5);
//		pos_ctrl(&hfdcan1,motor[Motor2].id,0,5);
//		pos_ctrl(&hfdcan1,motor[Motor3].id,0,5);
//		pos_ctrl(&hfdcan1,motor[Motor4].id,0,5);
//		pos_ctrl(&hfdcan1,motor[Motor5].id,0,5);
//		pos_ctrl(&hfdcan1,motor[Motor6].id,0,5);
//		pos_ctrl(&hfdcan1,motor[Motor1].id,2,5);
//		pos_ctrl(&hfdcan1,motor[Motor2].id,2,5);
//		pos_ctrl(&hfdcan1,motor[Motor3].id,2,5);
//		pos_ctrl(&hfdcan1,motor[Motor4].id,2,5);
//		pos_ctrl(&hfdcan1,motor[Motor5].id,2,5);
//		pos_ctrl(&hfdcan1,motor[Motor6].id,2,5);
		
		
		

		osDelay(10);
    osDelay(1);
  }
  /* USER CODE END CAN_Task */
}

//static void FDCAN1_CMD_1(void)
//{
//	read_all_motor_data(&motor[Motor1]);
//	read_all_motor_data(&motor[Motor2]);
//		
//	if(motor[Motor1].tmp.read_flag == 0 )
//		dm_motor_ctrl_send(&hfdcan1, &motor[Motor1]);
//		
//	if(motor[Motor2].tmp.read_flag == 0 )
//		dm_motor_ctrl_send(&hfdcan1, &motor[Motor2]);
//		
//}