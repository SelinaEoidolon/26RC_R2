#include "cmsis_os.h"
#include "CAN_Task.h"
#include "Control_Task.h"
#include "include.h"
#include "dm_motor_ctrl.h"
#include "bsp_tick.h"
#include "bsp_mcu.h"
#include "leg.h"
#include "mecanum_classic.h"
#include "CRC.h"
#include "bsp_uart.h"

extern float ctrl_j1,ctrl_j2,ctrl_j3;
extern float model_theta1,model_theta2,model_theta3;

extern WheelSpeed_t total_speed ;
//腿电机角度 rad
extern leg lf_leg ;//前腿
extern leg rf_leg ;

extern float lb_leg ;//后腿
extern float rb_leg ;


void CAN_Task(void const * argument){
	Delay_ms(5000);
	float Tnum = 360.0f / 8192.0f / 3591.0f *187.0f / 5.0f;
	for(;;)
    {
        FDCAN2_CMD_1(PID_velocity_realize_2(-total_speed.fl,1),
				     PID_velocity_realize_2(total_speed.fr,2),
				     PID_velocity_realize_2(total_speed.bl,3),
				     PID_velocity_realize_2(-total_speed.br,4)
		);

		FDCAN3_CMD_1(0,
		             pid_call_3(-ctrl_j2 / Tnum, 2), 
		             pid_call_3( ctrl_j3 / Tnum ,3), 
		             0
		);
		osDelay(1);
        pos_ctrl(&hfdcan1,motor[Motor1].id,lf_leg.theta1,0.5);
		pos_ctrl(&hfdcan1,motor[Motor2].id,lf_leg.theta2,0.5);
		pos_ctrl(&hfdcan1,motor[Motor3].id,rf_leg.theta1,0.5);
		pos_ctrl(&hfdcan1,motor[Motor4].id,rf_leg.theta2,0.5);
		pos_ctrl(&hfdcan1,motor[Motor5].id,lb_leg,2);
		pos_ctrl(&hfdcan1,motor[Motor6].id,rb_leg,2);
		osDelay(1);
        
        osDelay(1);
    }

	
}
//测试用例
	// FDCAN3_CMD_1(PID_velocity_realize_3(1500,1), 
	// 	         PID_velocity_realize_3(1500,2), 
	// 	         PID_velocity_realize_3(1500,3), 
	// 	         PID_velocity_realize_3(1500,4)
	// );
//		FDCAN3_CMD_1(0, 
//		             0, 
//		             0, 
//		             PID_velocity_realize_3(1500,4)
//		);
//		FDCAN3_CMD_1(0,
//		             pid_call_3(-ctrl_j2 / Tnum, 2), 
//		             pid_call_3( ctrl_j3 / Tnum ,3), 
//		             0
//		);

    // osDelay(1);


	// FDCAN2_CMD_1(PID_velocity_realize_2(1500,1), 
	// 	         PID_velocity_realize_2(1500,2), 
	// 	         PID_velocity_realize_2(1500,3), 
	// 	         PID_velocity_realize_2(1500,4)
	// );

    // osDelay(1);

