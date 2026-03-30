#include "CRC.h"
#include "usart.h"
#include "struct_typedef.h"
#include "math.h"



// 定义串口句柄，根据你的硬件修改
extern UART_HandleTypeDef huart10;
extern float RX_theta ;

// 全局变量
//蓝牙变量
ParseState BT_Uart10 = STATE_WAIT_HEADER;//接收状态机
uint8_t bt_data[7];     // 存储6位原数据
uint8_t data_index = 0;  // 原数据接收计数
uint8_t checksum;        // 存储接收到的校验和
volatile uint8_t bt_parse_ok = 0;// 解析完成标志（供主循环读取）
uint8_t btReceiveData = 0;
int8_t key = 0;
int8_t start_mode =0;
int8_t move_flag = 0;
int8_t move_val = 0;

float legx = 0;
float legy = 0;
float leghtheta = 0;

void UART10_Receive(uint8_t receiveData)
{
    // 临界区保护：防止主循环与中断同时访问全局变量
    __disable_irq();

    switch (BT_Uart10)
    {
        case STATE_WAIT_HEADER:
            if (receiveData == 0xA5)
            {
                // 收到包头，初始化接收状态
                BT_Uart10 = STATE_RECV_DATA;
                data_index = 0;
                // 清空缓冲区（可选，增强鲁棒性）
                for (int i = 0; i < 7; i++) bt_data[i] = 0;
            }
            break;

        case STATE_RECV_DATA:
            bt_data[data_index++] = receiveData;
            if (data_index >= 7)
            {
                // 6字节原数据接收完成，进入校验和接收
                BT_Uart10 = STATE_RECV_CHECKSUM;
            }
            break;

        case STATE_RECV_CHECKSUM:
            checksum = receiveData;
            BT_Uart10 = STATE_RECV_TAIL;
            break;

        case STATE_RECV_TAIL:
            if (receiveData == 0x5A)
            {
                // 收到正确包尾，计算校验和
                uint8_t calc_checksum = 0;
                for (int i = 0; i < 7; i++)
                {
                    calc_checksum += bt_data[i];
                }
                calc_checksum &= 0xFF; // 取低8位

                if (calc_checksum == checksum)
                {
                    // 校验通过，设置解析完成标志
                    bt_parse_ok = 1;
                }
                else
                {
                    // 校验失败，可添加错误计数/日志
                }
            }
            // 无论包尾是否正确、校验是否通过，都回到等待包头
            BT_Uart10 = STATE_WAIT_HEADER;
            break;

        default:
            BT_Uart10 = STATE_WAIT_HEADER;
            break;
    }

    __enable_irq();
}


/**
byte0 byte1 byte 2 XYW
byte3 h
byte4 key
byte5 摇摆
**/
void BT_Data_MAC_Process(float *V_x,float *V_y,float *V_w,int8_t *cmd,uint8_t *byte)
{
    if (bt_parse_ok)
    {
        __disable_irq();
        bt_parse_ok = 0; // 清除标志
        __enable_irq();
			

        // byte0为X轴 ，byte1为用轴 ，byte2为自转角速度
			  *V_x = (int8_t)byte[0] / 128.0f * 256.0f;  // X轴速度：-100 ~ 100
        *V_y = (int8_t)byte[1] / 128.0f * 256.0f;  // Y轴速度：-100 ~ 100
        *V_w = (int8_t)byte[2] / 128.0f * 256.0f;   // 角速度：-50 ~ 50
			
			  legx = (float)(int8_t)byte[3] ;
			  legy = (float)(int8_t)byte[4] ;
        if(legy <= 0) legy = -legy;
			  
			  leghtheta = (float)(int8_t)byte[5] / 128.0f * 55.0f;
			
			if (fabsf(legx) >= 128.0f) legx = 128.0f;
			if (fabsf(legy) >= 128.0f) legy = 128.0f;
			if (fabsf(leghtheta)  >= 55.0f) leghtheta = 55.0f;

			  move_flag = (int8_t)byte[6];
//			  *cmd = (int8_t)byte[3];
//			  if((int8_t)byte[4] == 1)
//				{
//					key++;
//					if(key == 126) key = 0;
//				}
//				if((int8_t)byte[5]==0)
//				{
//					move_flag = 1;
//					move_val = 1;
//				}
//				if((int8_t)byte[5]==1)
//				{
//					move_flag = 0;
//					move_val = 0;
//				}
//				if((int8_t)byte[5]==2)
//				{
//					move_flag = 1;
//					move_val = -1;
//				}
        
    }
		
}
//void BT_Data_DM_Process(int8_t *cmd ,uint8_t *byte)
//{
//	    if (bt_parse_ok)
//    {
//        __disable_irq();
//        bt_parse_ok = 0; // 清除标志
//        __enable_irq();
//			
//			  

//       
//        
//    }
//}
//// 初始化函数（在main中调用）
//void uart_parse_init(void)
//{
//    // 开启第一个字节的中断接收
//    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
//}
// 数据包解析完成回调函数（可根据需求修改）
//void on_packet_received(uint8_t *data, uint8_t len)
//{
//    // 这里可以处理解析后的6字节原数据
//    // 例如：打印、存储或执行相应逻辑
//    for (int i = 0; i < len; i++)
//    {
//        // 示例：通过串口回显解析出的数据
//        HAL_UART_Transmit(&huart1, &data[i], 1, 100);
//    }
//}
