#include "stm32h7xx_hal.h"

// 解析状态枚举
typedef enum {
    STATE_WAIT_HEADER,  // 等待包头
    STATE_RECV_DATA,    // 接收原数据
    STATE_RECV_CHECKSUM,// 接收校验和
    STATE_RECV_TAIL     // 接收包尾
} ParseState;

// 全局变量
extern ParseState BT_Uart10;//接收状态机
extern uint8_t bt_data[7];     // 存储6位原数据
extern uint8_t data_index;  // 原数据接收计数
extern uint8_t checksum;        // 存储接收到的校验和
extern volatile uint8_t bt_parse_ok;// 解析完成标志（供主循环读取）
extern uint8_t btReceiveData; //中断接收数据

extern int8_t key;
extern int8_t move_flag;
extern int8_t move_val;
extern UART_HandleTypeDef huart10;
extern int8_t control_cmd ;

extern float legx ;
extern float legy ;
extern float leghtheta ;

void UART10_Receive(uint8_t receiveData);
void BT_Data_MAC_Process(float *V_x,float *V_y,float *V_w,int8_t *cmd,uint8_t *byte);
void BT_Data_DM_Process(int8_t *cmd ,uint8_t *byte);

