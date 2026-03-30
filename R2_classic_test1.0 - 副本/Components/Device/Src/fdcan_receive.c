#include "fdcan_receive.h"
#include "bsp_fdcan.h"

//extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;

//motor_measure_t motor_fdcan1[8];
motor_measure_t motor_fdcan2[8];



/* 解析电机数据 */
void get_motor_measure(motor_measure_t *ptr, uint8_t data[])
{
    ptr->last_angle    = ptr->angle;
    ptr->angle         = (uint16_t)(data[0] << 8 | data[1]);
    ptr->speed_rpm     = (int16_t)(data[2] << 8 | data[3]);
    ptr->given_current = (int16_t)(data[4] << 8 | data[5]);
    ptr->temperature   = data[6];

    if (ptr->angle - ptr->last_angle > 4096)
        ptr->round_cnt--;
    else if (ptr->angle - ptr->last_angle < -4096)
        ptr->round_cnt++;

    ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;
}

void get_motor_offset(motor_measure_t *ptr, uint8_t data[])
{
    ptr->angle = (uint16_t)(data[0] << 8 | data[1]);
    ptr->offset_angle = ptr->angle;
}

#define ABS(x)	( (x>0) ? (x) : (-x) )

void get_total_angle(motor_measure_t *p)
{
    int res1, res2, delta;
    if (p->angle < p->last_angle)
    {
        res1 = p->angle + 8192 - p->last_angle;
        res2 = p->angle - p->last_angle;
    }
    else
    {
        res1 = p->angle - 8192 - p->last_angle;
        res2 = p->angle - p->last_angle;
    }

    delta = (ABS(res1) < ABS(res2)) ? res1 : res2;
    p->total_angle += delta;
    p->last_angle = p->angle;
}

///* FDCAN Rx FIFO0 中断回调（HAL 会调用） */
//void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
//{
//    FDCAN_RxHeaderTypeDef rx_header;
//    uint8_t rx_data[8];

//    /* 如果是 FIFO0 新消息中断 */
//    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
//    {
//        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
//        {
//            /* 可选：错误处理 */
//            return;
//        }

//        /* 处理电机数据 ID */
//        if (rx_header.Identifier >= CAN_3508_M1_ID && rx_header.Identifier <= CAN_3508_M7_ID)
//        {
//            uint8_t i = (uint8_t)(rx_header.Identifier - CAN_3508_M1_ID);
//            motor_measure_t *m = NULL;

//            
//            m = &motor_fdcan2[i];

//            if (m)
//            {
//                m->msg_cnt++;
//                if (m->msg_cnt <= 50)
//                    get_motor_offset(m, rx_data);
//                else
//                    get_motor_measure(m, rx_data);
//            }
//        }
//        else
//        {
//            /* 其它 ID 可按需扩展处理 */
//        }
//    }
//}

 void FDCAN_Send(FDCAN_HandleTypeDef *hfdcan, uint16_t std_id,
                       int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
	  FDCAN_TxHeaderTypeDef fdcan_tx_header;
    uint8_t fdcan_send_data[8];
	
    fdcan_tx_header.Identifier = std_id;
    fdcan_tx_header.IdType = FDCAN_STANDARD_ID;
    fdcan_tx_header.TxFrameType = FDCAN_DATA_FRAME;
    fdcan_tx_header.DataLength = FDCAN_DLC_BYTES_8;
    fdcan_tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    fdcan_tx_header.BitRateSwitch = FDCAN_BRS_OFF;     /* 经典 CAN，不切换到 FD 数据相位 */
    fdcan_tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    fdcan_tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    fdcan_tx_header.MessageMarker = 0;

    fdcan_send_data[0] = (uint8_t)(m1 >> 8);
    fdcan_send_data[1] = (uint8_t)(m1 & 0xFF);
    fdcan_send_data[2] = (uint8_t)(m2 >> 8);
    fdcan_send_data[3] = (uint8_t)(m2 & 0xFF);
    fdcan_send_data[4] = (uint8_t)(m3 >> 8);
    fdcan_send_data[5] = (uint8_t)(m3 & 0xFF);
    fdcan_send_data[6] = (uint8_t)(m4 >> 8);
    fdcan_send_data[7] = (uint8_t)(m4 & 0xFF);

    /* 添加到 Tx FIFO/Q */
    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &fdcan_tx_header, fdcan_send_data) != HAL_OK)
    {
        /* 发送失败的处理（可选） */
    }
}

/* 对应的对外接口（与原逻辑等价，但改为 FDCAN_ 前缀） */
//void FDCAN1_CMD_1(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
//{
//    FDCAN_Send(&hfdcan1, 0x200, motor1, motor2, motor3, motor4);
//}

//void FDCAN1_CMD_2(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8)
//{
//    FDCAN_Send(&hfdcan1, 0x1FF, motor5, motor6, motor7, motor8);
//}

void FDCAN2_CMD_1(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    FDCAN_Send(&hfdcan2, 0x200, motor1, motor2, motor3, motor4);
}

void FDCAN2_CMD_2(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8)
{
    FDCAN_Send(&hfdcan2, 0x1FF, motor5, motor6, motor7, motor8);
}




