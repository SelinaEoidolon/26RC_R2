#include "CRC.h"
#include "usart.h"
#include "struct_typedef.h"
#include "math.h"
#include <string.h>

/* 串口句柄 */
extern UART_HandleTypeDef huart10;

/* 全局接收状态 */
ParseState BT_Uart10 = STATE_WAIT_HEADER;
uint8_t bt_data[BT_FRAME_DATA_LEN];
uint8_t data_index = 0;
uint8_t checksum = 0;
volatile uint8_t bt_parse_ok = 0;
uint8_t btReceiveData = 0;


/* 腿部控制量 */
int8_t leg_flag = 0;
float legx = 0.0f;
float legy = 0.0f;
float leghtheta = 0.0f;

/* 机械臂控制量 */
int8_t arm_flag = 0;
float arm_X = 0.0f;
float arm_Y = 0.0f;
float arm_Z = 0.0f;

/*
 * UART10 单字节接收状态机
 *
 * 帧格式：
 *   0xA5 + 11字节数据 + 1字节校验和 + 0x5A
 *
 * 校验规则：
 *   checksum = data[0] + data[1] + ... + data[10]
 *   取低8位
 */
void UART10_Receive(uint8_t receiveData)
{
    switch (BT_Uart10)
    {
        case STATE_WAIT_HEADER:
        {
            if (receiveData == 0xA5U)
            {
                BT_Uart10 = STATE_RECV_DATA;
                data_index = 0;
                memset(bt_data, 0, sizeof(bt_data));
            }
        }
        break;

        case STATE_RECV_DATA:
        {
            bt_data[data_index++] = receiveData;

            if (data_index >= BT_FRAME_DATA_LEN)
            {
                BT_Uart10 = STATE_RECV_CHECKSUM;
            }
        }
        break;

        case STATE_RECV_CHECKSUM:
        {
            checksum = receiveData;
            BT_Uart10 = STATE_RECV_TAIL;
        }
        break;

        case STATE_RECV_TAIL:
        {
            if (receiveData == 0x5AU)
            {
                uint8_t calc_checksum = 0U;
                uint8_t i;

                for (i = 0; i < BT_FRAME_DATA_LEN; i++)
                {
                    calc_checksum += bt_data[i];
                }

                if (calc_checksum == checksum)
                {
                    bt_parse_ok = 1U;
                }
            }

            /* 无论成功失败，回到等待包头 */
            BT_Uart10 = STATE_WAIT_HEADER;
            data_index = 0;
        }
        break;

        default:
        {
            BT_Uart10 = STATE_WAIT_HEADER;
            data_index = 0;
        }
        break;
    }
}

/*
 * 解析最新一帧遥控器数据
 *
 * 注意：
 * 这里不再从外面传 byte[] 进来，
 * 直接读取全局 bt_data，并且先做一份本地快照，
 * 避免中断又写入下一帧时把当前解析过程打断。
 */
void BT_Data_MAC_Process(float *V_x, float *V_y, float *V_w, int8_t *cmd)
{
    uint8_t frame[BT_FRAME_DATA_LEN];

    (void)cmd; /* 你当前这版代码没真正用到 cmd，这里先保留接口 */

    if ((V_x == 0) || (V_y == 0) || (V_w == 0))
    {
        return;
    }

    if (bt_parse_ok == 0U)
    {
        return;
    }

    __disable_irq();
    memcpy(frame, bt_data, sizeof(frame));
    bt_parse_ok = 0U;
    __enable_irq();

    /* 底盘控制 */
    *V_x = (int8_t)frame[0] / 128.0f * 256.0f;
    *V_y = (int8_t)frame[1] / 128.0f * 256.0f;
    *V_w = (int8_t)frame[2] / 128.0f * 256.0f;

    /* 腿部控制 */
    legx = (float)(int8_t)frame[3];
    legy = (float)(int8_t)frame[4];
    leghtheta = (float)(int8_t)frame[5] / 128.0f * 55.0f;

    if (fabsf(legx) >= 128.0f)      legx = (legx > 0.0f) ? 120.0f : -120.0f;
    if (fabsf(legy) >= 128.0f)      legy = (legy > 0.0f) ? 120.0f : -120.0f;
    if (fabsf(leghtheta) >= 55.0f)  leghtheta = (leghtheta > 0.0f) ? 55.0f : -55.0f;

    leg_flag = (int8_t)frame[6];

    /* 机械臂控制 */
    arm_flag = (int8_t)frame[7];

    if (arm_flag == 1)
    {
        arm_X = (float)(int8_t)frame[8] / 128.0f * 500.0f; /* 映射到 -500mm ~ +500mm */
        arm_Y = (float)(int8_t)frame[9] / 128.0f * 500.0f;
        arm_Z = (float)(int8_t)frame[10] / 128.0f * 500.0f;
    }
    else
    {
        arm_X = 0.0f;
        arm_Y = 0.0f;
        arm_Z = 0.0f;
    }
}