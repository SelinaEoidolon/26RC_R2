#include "cmsis_os.h"
#include "bsp_usb.h"
#include "usbd_cdc_if.h"
#include "pid_user.h"
#include "fdcan_receive.h"
#include "bsp_fdcan.h"
#include "bsp_tick.h"
#include "include.h"
#include "bsp_uart.h"
#include "CRC.h"
#include "bsp_usb.h"
#include "arm_ik_3r_safe_stm32h7.h"
#include "arm_user.h"

float getlen;

extern uint8_t usb_Buf[USB_FRAME_BUF_SIZE];

extern Arm3R_Handle_t g_arm_ik;

uint8_t rx_buf[64];
float ctrl_j1,ctrl_j2,ctrl_j3;
float model_theta1,model_theta2,model_theta3;

void Control_Task(void const * argument){
	osDelay(1000);
	MX_USB_DEVICE_Init();
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_SET);
	FDCAN2_Filter_Init();
	FDCAN3_Filter_Init();
  PID_devices_Init();
	
	ArmIK_ComponentInit();

  
  for(;;)
  {
//		uint32_t n = CDC_App_Read(usb_Buf, sizeof(usb_Buf));

//        // 2) 逐字节喂给你的协议解析状态机
//    for (uint32_t i = 0; i < n; i++) {
//        Receive(usb_Buf[i]);
//    }

//    if (n > 0)
//    {
//	     if (CDC_App_Write((const uint8_t *)"return:", 7))
//       {
//          (void)CDC_App_Write(usb_Buf, n);
//          (void)CDC_App_Write((const uint8_t *)"\r\n", 2);
//       }
//    }
//		
//		ArmIK_ComponentStep(150.0f,50.0f,400.0f);
//		
//		g_arm_ik.result.model.theta1 = g_arm_ik.result.model.theta1 * 180.0f / 3.1415926f;
//		g_arm_ik.result.model.theta2 = g_arm_ik.result.model.theta2 * 180.0f / 3.1415926f;
//		g_arm_ik.result.model.theta3 = g_arm_ik.result.model.theta3 * 180.0f / 3.1415926f;
//	
//    g_arm_ik.result.ctrl.j1  = g_arm_ik.result.ctrl.j1  * 180.0f / 3.1415926f;
//    g_arm_ik.result.ctrl.j2  = g_arm_ik.result.ctrl.j2  * 180.0f / 3.1415926f;
//    g_arm_ik.result.ctrl.j3  = g_arm_ik.result.ctrl.j3  * 180.0f / 3.1415926f;

    uint32_t i;
    uint32_t read_len;

    read_len = CDC_App_Read(usb_Buf, sizeof(usb_Buf));
    for (i = 0; i < read_len; i++)
    {
        Receive(usb_Buf[i]);
    }
//		getlen = CDC_App_GetRxDropped();

    /* 推动 TX 环形缓冲往 USB 发 */
    CDC_App_TxTask();
		
		model_theta1 = g_arm_ik.result.model.theta1 * 180.0f / 3.1415926f;
		model_theta2 = g_arm_ik.result.model.theta2 * 180.0f / 3.1415926f;
		model_theta3 = g_arm_ik.result.model.theta3 * 180.0f / 3.1415926f;
	
    ctrl_j1  = g_arm_ik.result.ctrl.j1  * 180.0f / 3.1415926f;
    ctrl_j2  = g_arm_ik.result.ctrl.j2  * 180.0f / 3.1415926f;
    ctrl_j3  = g_arm_ik.result.ctrl.j3  * 180.0f / 3.1415926f;
		
    osDelay(1);
  }

	
}
void CAN_Task(void const * argument){
	Delay_ms(5000);
	float Tnum = 360.0f / 8192.0f / 3591.0f *187.0f / 5.0f;
	  for(;;)
  {
//		FDCAN3_CMD_1(PID_velocity_realize_3(1500,1), 
//		             PID_velocity_realize_3(1500,2), 
//		             PID_velocity_realize_3(1500,3), 
//		             PID_velocity_realize_3(1500,4)
//		);
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

    osDelay(10);

//			FDCAN3_CMD_1(0, 
//		               0, 
//		               0, 
//		               0
//		  );
//		FDCAN2_CMD_1(PID_velocity_realize_2(1500,1), 
//		             PID_velocity_realize_2(1500,2), 
//		             PID_velocity_realize_2(1500,3), 
//		             PID_velocity_realize_2(1500,4)
//		);

    osDelay(1);
		
    osDelay(1);
  }

	
}





