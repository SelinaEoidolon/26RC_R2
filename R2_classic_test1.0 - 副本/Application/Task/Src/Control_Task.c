#include "cmsis_os.h"
#include "Control_Task.h"
#include "bsp_mcu.h"
#include "include.h"
#include "dm_motor_ctrl.h"
#include "bsp_tick.h"
#include "leg.h"
#include "mecanum_classic.h"
#include "CRC.h"
#include "bsp_uart.h"
#include "quadruped_fsm.h"
#include "bsp_leg.h"

#include "usbd_cdc_if.h"
#include <stdio.h>
#include "bsp_usb.h"

extern uint8_t usb_Buf[300];

extern uint8_t bt_data[7];     // 蓝牙数据
extern ChassisVel_t total_vel ;
extern WheelSpeed_t total_speed;
extern MecanumParam_t mecParam;

//腿电机角度 rad
extern leg lf_leg ;//前腿
extern leg rf_leg ;

float  lf_last_theta1 = 0;
float  lf_last_theta2 = 0;
float  rf_last_theta1 = 0;
float  rf_last_theta2 = 0;


extern float lb_leg ;//后腿
extern float rb_leg ;

extern int8_t LEG_Cmd ;
extern float bleg_theta ;
extern float fleg_high ;

extern QFSM_Handle_t g_qfsm;//状态机实例

extern int8_t control_cmd ;

extern float legx ;
extern float legy ;
extern float leghtheta ;

float TOTAL_yaw = 0;
extern ChassisVel_t classic_speed;


void Control_Task(void const * argument)
{
  /* USER CODE BEGIN Control_Task */
	osDelay(1000);
	MCU_Init();
	MX_USB_DEVICE_Init();

	HAL_UART_Receive_IT(&huart10, &btReceiveData, 1);
	
//	QFSM_Init(&g_qfsm);
//  QFSM_Start(&g_qfsm);          // 先进入状态1，写入第一帧命令

  /* Infinite loop */
  for(;;)
  {
		//底盘麦轮控制
//    BT_Data_MAC_Process(&total_vel.vx,&total_vel.vy,&total_vel.vw,&LEG_Cmd,bt_data); 
////		BT_Data_DM_Process(&LEG_Cmd,bt_data);
//		Mecanum_Calc(&total_vel,&mecParam,&total_speed);
		
		
		//四条腿发送任务在main定时器回调函数
//		InverseKinematics(0,0,&lf_leg.theta1,&lf_leg.theta2);
//		backLeg_Left(0,&lb_leg);
//		
//		InverseKinematics(0,0,&rf_leg.theta1,&rf_leg.theta2);
//		backLeg_Right(0,&rb_leg);
//      __disable_irq();
		
//				lf_leg.theta1 = 0.0f;
//			  lf_leg.theta2 = 0.0f;
//				rf_leg.theta1 = 0.0f;
//			  rf_leg.theta2 = 0.0f;
//			
//			  lb_leg = 0.0f;
//			  rb_leg = 0.0f;
//		Delay_ms(5000);
//		InverseKinematics(45.0f,60.0f,&lf_leg.theta1,&lf_leg.theta2);
//		InverseKinematics(45.0f,60.0f,&rf_leg.theta1,&rf_leg.theta2);
		
//		if(key%2==1)
//		{
//			if(move_flag == 0)
//			{
//					lf_leg.theta1 = (LEG_Cmd * 4.0f)*3.14159f/180.0f;
//					lf_leg.theta2 = (-LEG_Cmd * 4.0f)*3.14159f/180.0f;
//					rf_leg.theta1 = (LEG_Cmd * 4.0f)*3.14159f/180.0f;
//					rf_leg.theta2 = (-LEG_Cmd * 4.0f)*3.14159f/180.0f;
//				
//					lb_leg = 4.0f * (-LEG_Cmd * 5.0f)*3.14159f/180.0f;
//					rb_leg = 4.0f * (LEG_Cmd * 5.0f)*3.14159f/180.0f;
//			}
//			else
//			{
//					lf_leg.theta1 = ((LEG_Cmd-move_val) * 4.0f)*3.14159f/180.0f;
//					lf_leg.theta2 = (-(LEG_Cmd-move_val) * 4.0f)*3.14159f/180.0f;
//					rf_leg.theta1 = ((LEG_Cmd+move_val) * 4.0f)*3.14159f/180.0f;
//					rf_leg.theta2 = (-(LEG_Cmd+move_val) * 4.0f)*3.14159f/180.0f;
//				
//					lb_leg = 4.0f * (-(LEG_Cmd-move_val) * 5.0f)*3.14159f/180.0f;
//					rb_leg = 4.0f * ((LEG_Cmd+move_val) * 5.0f)*3.14159f/180.0f;

//			}
//		}
//		else
//		{
//				lf_leg.theta1 = 0.0f;
//			  lf_leg.theta2 = 0.0f;
//				rf_leg.theta1 = 0.0f;
//			  rf_leg.theta2 = 0.0f;
//			
//			  lb_leg = 0.0f;
//			  rb_leg = 0.0f;
//		}
//      __enable_irq();
		 uint32_t n = CDC_App_Read(usb_Buf, sizeof(usb_Buf));

        // 2) 逐字节喂给你的协议解析状态机
        for (uint32_t i = 0; i < n; i++) {
            Receive(usb_Buf[i]);
        }

if (n > 0)
{
	if (CDC_App_Write((const uint8_t *)"return:", 7))
    {
        (void)CDC_App_Write(usb_Buf, n);
        (void)CDC_App_Write((const uint8_t *)"\r\n", 2);
    }
}
				
    BT_Data_MAC_Process(&total_vel.vx,&total_vel.vy,&total_vel.vw,&LEG_Cmd,bt_data); 
		Mecanum_Calc(&total_vel,&mecParam,&total_speed);
			if(move_flag == 0)
			{
				lf_leg.theta1 = 0.0f;
			  lf_leg.theta2 = 0.0f;
				rf_leg.theta1 = 0.0f;
			  rf_leg.theta2 = 0.0f;
				
			  lf_last_theta1 = 0.0f;
        lf_last_theta2 = 0.0f;
			  rf_last_theta1 = 0.0f;
        rf_last_theta2 = 0.0f;
			
			  lb_leg = 0.0f;
			  rb_leg = 0.0f;
			}
			else
			{
					InverseKinematics_Continuous(legx,legy,lf_last_theta1,lf_last_theta2,&lf_leg.theta1,&lf_leg.theta2);
					InverseKinematics_Continuous(legx,legy,rf_last_theta1,rf_last_theta2,&rf_leg.theta1,&rf_leg.theta2);
				
				  lf_last_theta1 = lf_leg.theta1;
          lf_last_theta2 = lf_leg.theta2;
				  rf_last_theta1 = rf_leg.theta1;
          rf_last_theta2 = rf_leg.theta2;
				
					lf_leg.theta1 = -lf_last_theta1;
					lf_leg.theta2 =  lf_last_theta2;
					rf_leg.theta1 =  rf_last_theta1;
					rf_leg.theta2 = -rf_last_theta2;
				
////					lf_leg.theta1 =  legx * 3.14159f/180.0f;
////					lf_leg.theta2 = -legx * 3.14159f/180.0f;
////					rf_leg.theta1 =  legx * 3.14159f/180.0f;
////					rf_leg.theta2 = -legx * 3.14159f/180.0f;

				
					lb_leg = 4.0f * (-leghtheta)*3.14159f/180.0f;
					rb_leg = 4.0f * ( leghtheta)*3.14159f/180.0f;
			}
//					lf_leg.theta1 =  0.0f;
//					lf_leg.theta2 =  0.0f;
//					rf_leg.theta1 =  0.0f;
//					rf_leg.theta2 =  0.0f;
//					
//					osDelay(3000);
//					
//					lf_leg.theta1 =  0.52f;
//					lf_leg.theta2 =  -0.12;
//					rf_leg.theta1 =  -0.52f;
//					rf_leg.theta2 =  0.12f;
//					

//					lf_leg.theta1 =  0.0f;
//					lf_leg.theta2 =  0.0f;
//					rf_leg.theta1 =  0.0f;
//					rf_leg.theta2 =  0.0f;
//					osDelay(5000);
//					
//					lf_leg.theta1 =  0.1f;
//					lf_leg.theta2 =  0.1f;
//					rf_leg.theta1 =  0.1f;
//					rf_leg.theta2 =  0.1f;
//					osDelay(5000);
					
    osDelay(1);
  }
  /* USER CODE END Control_Task */
}