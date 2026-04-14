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
#include "usbd_cdc_if.h"
#include <stdio.h>
#include "bsp_usb.h"
#include "arm_ik_3r_safe_stm32h7.h"
#include "arm_user.h"

extern uint8_t usb_Buf[USB_FRAME_BUF_SIZE];// USB接收缓冲区
extern uint8_t bt_data[11];     // 蓝牙数据

extern Arm3R_Handle_t g_arm_ik;
float ctrl_j1,ctrl_j2,ctrl_j3;
float model_theta1,model_theta2,model_theta3;

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

extern int8_t control_cmd ;

extern float legx ;
extern float legy ;
extern float leghtheta ;

static void USB_RX_task(void); 

void Control_Task(void const * argument){
	osDelay(1000);

	MX_USB_DEVICE_Init();
    HAL_UART_Receive_IT(&huart10, &btReceiveData, 1);

    MCU_Init();

	ArmIK_ComponentInit();

  for(;;)
  {
    USB_RX_task();
    BT_Data_MAC_Process(&total_vel.vx,&total_vel.vy,&total_vel.vw,&LEG_Cmd); 


	Mecanum_Calc(&total_vel,&mecParam,&total_speed);
	
	if(leg_flag == 0)
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
				
        lb_leg = 4.0f * (-leghtheta)*3.14159f/180.0f;
		rb_leg = 4.0f * ( leghtheta)*3.14159f/180.0f;
	}



		
	// model_theta1 = g_arm_ik.result.model.theta1 * 180.0f / 3.1415926f;
	// model_theta2 = g_arm_ik.result.model.theta2 * 180.0f / 3.1415926f;
	// model_theta3 = g_arm_ik.result.model.theta3 * 180.0f / 3.1415926f;
	
    // ctrl_j1  = g_arm_ik.result.ctrl.j1  * 180.0f / 3.1415926f;
    // ctrl_j2  = g_arm_ik.result.ctrl.j2  * 180.0f / 3.1415926f;
    // ctrl_j3  = g_arm_ik.result.ctrl.j3  * 180.0f / 3.1415926f;

	
    osDelay(1);
  }

	
}

static void USB_RX_task(void){
    uint32_t i;
    uint32_t read_len;

    read_len = CDC_App_Read(usb_Buf, sizeof(usb_Buf));
    for (i = 0; i < read_len; i++)
    {
        Receive(usb_Buf[i]);
    }
}




