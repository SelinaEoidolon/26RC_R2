#include "cmsis_os.h"
#include "Control_Task.h"


extern uint8_t usb_Buf[USB_FRAME_BUF_SIZE];// USB���ջ�����
extern uint8_t bt_data[BT_FRAME_DATA_LEN];     // ��������

extern uint8_t USB_Task_flag;
extern uint8_t USART_Task_flag ;


//��е�ۿ���
extern Arm3R_Handle_t g_arm_ik;
float ctrl_j1,ctrl_j2,ctrl_j3;
float model_theta1,model_theta2,model_theta3;

float start_X = 0.0f;//启动保护
float start_Y = 0.0f;
float start_Z = 0.0f;

float start_X_USB = 0.0f;//USB启动保护
float start_Y_USB = 0.0f;
float start_Z_USB = 0.0f;




//�����ķ�ֵ��̿���
extern ChassisVel_t total_vel ;
extern WheelSpeed_t total_speed;
extern MecanumParam_t mecParam;


//�������
extern float legx ;//ǰ��
extern float legy ;
extern float leghtheta ;//����
//�ȵ���Ƕ� rad
extern leg lf_leg ;//ǰ��
extern leg rf_leg ;
float  lf_last_theta1 = 0;
float  lf_last_theta2 = 0;
float  rf_last_theta1 = 0;
float  rf_last_theta2 = 0;

extern float lb_leg ;//����
extern float rb_leg ;

//����ָ���ݶ�
extern int8_t control_cmd ;


//��������
static void USB_RX_task(void);             //usb���մ���

static void Mecanum_task(void);             //�����ķ�ֵ��̿��ƴ���
static void LEG_task(void);                       //������ƴ���
static void Arm_task(void);    //��е�ۿ��ƴ���

void Mecanum_task_USB(ChassisVel_t *chassis_user, MecanumParam_t *param_user, WheelSpeed_t *speed_user);    //�����ķ�ֵ��̿��ƴ�����ר�Ÿ�USB���ݽ������õĽӿ�
void LEG_task_USB(float legx,float legy,float h);    //������ƴ�����ר�Ÿ�USB���ݽ������õĽӿ�
void Arm_task_USB(float x,float y,float z);    //��е�ۿ��ƴ�����ר�Ÿ�USB���ݽ������õĽӿ�

void Control_Task(void const * argument){
	osDelay(1000);

	MX_USB_DEVICE_Init();
    HAL_UART_Receive_IT(&huart10, &btReceiveData, 1);

    MCU_Init();

	ArmEchoUart10_Init();
	ArmIK_ComponentInit();


  for(;;)
  {
    USB_RX_task();
    BT_Data_MAC_Process(&total_vel.vx,&total_vel.vy,&total_vel.vw,&LEG_Cmd); 

    Mecanum_task();
    osDelay(1);

	LEG_task();
    osDelay(1);

    // Arm_task();
    if(USART_Task_flag == 1)
    {
        Arm_task();
    }
    osDelay(1);
    // ArmIK_ComponentStep(150.0f,0.0f,400.0f);

    ctrl_j1  = g_arm_ik.result.ctrl.j1  * 180.0f / 3.1415926f;
    ctrl_j2  = g_arm_ik.result.ctrl.j2  * 180.0f / 3.1415926f;
    ctrl_j3  = g_arm_ik.result.ctrl.j3  * 180.0f / 3.1415926f;
    osDelay(1);
  }

	
}



static void USB_RX_task(void)
{
    uint32_t i;
    uint32_t read_len;

    read_len = CDC_App_Read(usb_Buf, sizeof(usb_Buf));
    for (i = 0; i < read_len; i++)
    {
        Receive(usb_Buf[i]);
    }
}

static void Mecanum_task(void) 
{
    Mecanum_Calc(&total_vel, &mecParam, &total_speed);
}

static void LEG_task(void)
{
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

}
static void Arm_task()
{
    const ArmIK_AppState_t *app;

    if(start_X == arm_X && start_Y == arm_Y && start_Z == arm_Z)
    {
		model_theta1 = 0.0f;
		model_theta2 = 0.0f;
		model_theta3 = 0.0f;

		ctrl_j1 = 0.0f;
		ctrl_j2 = 0.0f;
		ctrl_j3 = 0.0f;
    }
    else
    {
        ArmIK_ComponentStep(arm_X, arm_Y, arm_Z);

        /* ��ȡӦ�ò㵱ǰʵ��ά�ֵİ�ȫ��� */
        app = ArmIK_GetAppState();

        if (app->has_last_valid != 0U)
        {
		    if(arm_flag == 1)
            {
                model_theta1 = app->active_model.theta1;
                model_theta2 = app->active_model.theta2;
                model_theta3 = app->active_model.theta3;

                ctrl_j1 = app->active_motor_deg.j1_deg;
                ctrl_j2 = app->active_motor_deg.j2_deg;
                ctrl_j3 = app->active_motor_deg.j3_deg;
		    }
		    else 
		    {
			    model_theta1 = 0.0f;
			    model_theta2 = 0.0f;
			    model_theta3 = 0.0f;

			    ctrl_j1 = 0.0f;
			    ctrl_j2 = 0.0f;
			    ctrl_j3 = 0.0f;
		    }

	        if (ArmEchoUart10_IsBusy() == 0U)
            {
                ArmEchoUart10_StartSend_IT();
            }
        }
    }
}



void Mecanum_task_USB(ChassisVel_t *chassis_user, MecanumParam_t *param_user, WheelSpeed_t *speed_user)    //�����ķ�ֵ��̿��ƴ�����ר�Ÿ�USB���ݽ������õĽӿ�
{
    Mecanum_Calc(chassis_user, param_user, speed_user);
}

void LEG_task_USB(float legx,float legy,float h)
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
				
    lb_leg = 4.0f * (-h)*3.14159f/180.0f;
	rb_leg = 4.0f * ( h)*3.14159f/180.0f;

}
void Arm_task_USB(float x,float y,float z)
{
    const ArmIK_AppState_t *app;

    if(start_X_USB == x && start_Y_USB == y && start_Z_USB == z)
    {
		model_theta1 = 0.0f;
		model_theta2 = 0.0f;
		model_theta3 = 0.0f;

		ctrl_j1 = 0.0f;
		ctrl_j2 = 0.0f;
		ctrl_j3 = 0.0f;
    }
    else
    {
        ArmIK_ComponentStep(x, y, z);

        /* ��ȡӦ�ò㵱ǰʵ��ά�ֵİ�ȫ��� */
        app = ArmIK_GetAppState();

        if (app->has_last_valid != 0U)
        {
		    if(arm_flag == 1)
            {
                model_theta1 = app->active_model.theta1;
                model_theta2 = app->active_model.theta2;
                model_theta3 = app->active_model.theta3;

                ctrl_j1 = app->active_motor_deg.j1_deg;
                ctrl_j2 = app->active_motor_deg.j2_deg;
                ctrl_j3 = app->active_motor_deg.j3_deg;
		    }
		    else 
		    {
			    model_theta1 = 0.0f;
			    model_theta2 = 0.0f;
			    model_theta3 = 0.0f;

			    ctrl_j1 = 0.0f;
			    ctrl_j2 = 0.0f;
			    ctrl_j3 = 0.0f;
		    }

	        if (ArmEchoUart10_IsBusy() == 0U)
            {
                ArmEchoUart10_StartSend_IT();
            }
        }
    }
}
