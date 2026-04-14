#include "bsp_fdcan.h"
#include "dm_motor_ctrl.h"
#include "fdcan_receive.h"

extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

extern motor_measure_t motor_fdcan2[8];
extern motor_measure_t motor_fdcan3[8];

/**
************************************************************************
* @brief:      	bsp_can_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN 使能
************************************************************************
**/
void bsp_can_init(void)
{
	
	can_filter_init();
	HAL_FDCAN_Start(&hfdcan1);                               //开启FDCAN
	HAL_FDCAN_ActivateNotification(&hfdcan1,
                                       0 | FDCAN_IT_RX_FIFO0_WATERMARK | FDCAN_IT_RX_FIFO0_WATERMARK
                                           | FDCAN_IT_TX_COMPLETE | FDCAN_IT_TX_FIFO_EMPTY | FDCAN_IT_BUS_OFF
                                           | FDCAN_IT_ARB_PROTOCOL_ERROR | FDCAN_IT_DATA_PROTOCOL_ERROR
                                           | FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING,
                                       0x00000F00);
}
/**
************************************************************************
* @brief:      	can_filter_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN滤波器初始化
************************************************************************
**/
void can_filter_init(void)
{
	FDCAN_FilterTypeDef fdcan_filter;
	
	fdcan_filter.IdType = FDCAN_STANDARD_ID;                       //标准ID
	fdcan_filter.FilterIndex = 0;                                  //滤波器索引                   
	fdcan_filter.FilterType = FDCAN_FILTER_MASK;                   
	fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;           //过滤器0关联到FIFO0  
	fdcan_filter.FilterID1 = 0x00;                               
	fdcan_filter.FilterID2 = 0x00;

	HAL_FDCAN_ConfigFilter(&hfdcan1,&fdcan_filter); 		 				  //接收ID2
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,FDCAN_REJECT,FDCAN_REJECT,FDCAN_REJECT_REMOTE,FDCAN_REJECT_REMOTE);
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 1);

}

void bsp_fdcan_set_baud(hcan_t *hfdcan, uint8_t mode, uint8_t baud)
{
	uint32_t nom_brp=0, nom_seg1=0, nom_seg2=0, nom_sjw=0;
	uint32_t dat_brp=0, dat_seg1=0, dat_seg2=0, dat_sjw=0;
	
	/*	nominal_baud = 80M/brp/(1+seg1+seg2)
		sample point = (1+seg1)/(1+seg1+sjw)  */
	if(mode == CAN_CLASS)
	{
		switch (baud)
		{
			case CAN_BR_125K: 	nom_brp=4 ; nom_seg1=139; nom_seg2=20; nom_sjw=20; break; // sample point 87.5%
			case CAN_BR_200K: 	nom_brp=2 ; nom_seg1=174; nom_seg2=25; nom_sjw=25; break; // sample point 87.5%
			case CAN_BR_250K: 	nom_brp=2 ; nom_seg1=139; nom_seg2=20; nom_sjw=20; break; // sample point 87.5%
			case CAN_BR_500K: 	nom_brp=1 ; nom_seg1=139; nom_seg2=20; nom_sjw=20; break; // sample point 87.5%
			case CAN_BR_1M:		nom_brp=1 ; nom_seg1=59 ; nom_seg2=20; nom_sjw=20; break; // sample point 75%
		}
		dat_brp=1 ; dat_seg1=29; dat_seg2=10; dat_sjw=10;	// 仲裁域默认1M
		hfdcan->Init.FrameFormat = FDCAN_FRAME_CLASSIC;
	}
	/*	data_baud	 = 80M/brp/(1+seg1+seg2)
		sample point = (1+seg1)/(1+seg1+sjw)  */
	if(mode == CAN_FD_BRS)
	{
		switch (baud)
		{
			case CAN_BR_2M: 	dat_brp=1 ; dat_seg1=29; dat_seg2=10; dat_sjw=10; break;	// sample point 75%
			case CAN_BR_2M5: 	dat_brp=1 ; dat_seg1=25; dat_seg2=6 ; dat_sjw=6 ; break;	// sample point 81.25%
			case CAN_BR_3M2: 	dat_brp=1 ; dat_seg1=19; dat_seg2=5 ; dat_sjw=5 ; break;	// sample point 80%
			case CAN_BR_4M: 	dat_brp=1 ; dat_seg1=14; dat_seg2=5 ; dat_sjw=5 ; break;	// sample point 75%
			case CAN_BR_5M:		dat_brp=1 ; dat_seg1=13; dat_seg2=2 ; dat_sjw=2 ; break;	// sample point 87.5%
		}
		nom_brp=1 ; nom_seg1=59 ; nom_seg2=20; nom_sjw=20; // 数据域默认1M
		hfdcan->Init.FrameFormat = FDCAN_FRAME_FD_BRS;
	}
	
	HAL_FDCAN_DeInit(hfdcan);
	
	hfdcan->Init.NominalPrescaler = nom_brp;
	hfdcan->Init.NominalTimeSeg1  = nom_seg1;
	hfdcan->Init.NominalTimeSeg2  = nom_seg2;
	hfdcan->Init.NominalSyncJumpWidth = nom_sjw;
	
	hfdcan->Init.DataPrescaler = dat_brp;
	hfdcan->Init.DataTimeSeg1  = dat_seg1;
	hfdcan->Init.DataTimeSeg2  = dat_seg2;
	hfdcan->Init.DataSyncJumpWidth = dat_sjw;
	
	HAL_FDCAN_Init(hfdcan);
}


/**
************************************************************************
* @brief:      	fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
* @param:       hfdcan：FDCAN句柄
* @param:       id：CAN设备ID
* @param:       data：发送的数据
* @param:       len：发送的数据长度
* @retval:     	void
* @details:    	发送数据
************************************************************************
**/
uint8_t fdcanx_send_data(hcan_t *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
{	
    FDCAN_TxHeaderTypeDef pTxHeader;
    pTxHeader.Identifier=id;
    pTxHeader.IdType=FDCAN_STANDARD_ID;
    pTxHeader.TxFrameType=FDCAN_DATA_FRAME;
	
	if(len<=8)
		pTxHeader.DataLength = len;
	else if(len==12)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_12;
	else if(len==16)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_16;
	else if(len==20)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_20;
	else if(len==24)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_24;
	else if(len==32)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_32;
	else if(len==48)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_48;
	else if(len==64)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_64;
	
    pTxHeader.ErrorStateIndicator=FDCAN_ESI_ACTIVE;
    pTxHeader.BitRateSwitch=FDCAN_BRS_ON;
    pTxHeader.FDFormat=FDCAN_FD_CAN;
    pTxHeader.TxEventFifoControl=FDCAN_NO_TX_EVENTS;
    pTxHeader.MessageMarker=0;
 
	if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &pTxHeader, data)!=HAL_OK) 
		return 1;//发送
	return 0;	
}
/**
************************************************************************
* @brief:      	fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint8_t *buf)
* @param:       hfdcan：FDCAN句柄
* @param:       buf：接收数据缓存
* @retval:     	接收的数据长度
* @details:    	接收数据
************************************************************************
**/
uint8_t fdcanx_receive(hcan_t *hfdcan, uint16_t *rec_id, uint8_t *buf)
{	
	FDCAN_RxHeaderTypeDef pRxHeader;
	uint8_t len;
	
	if(HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO0, &pRxHeader, buf)==HAL_OK)
	{
		*rec_id = pRxHeader.Identifier;
		if(pRxHeader.DataLength<=FDCAN_DLC_BYTES_8)
			len = pRxHeader.DataLength;
		else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_12)
			len = 12;
		else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_16)
			len = 16;
		else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_20)
			len = 20;
		else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_24)
			len = 24;
		else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_32)
			len = 32;
		else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_48)
			len = 48;
		else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_64)
			len = 64;
		
		return len;//接收数据
	}
	return 0;	
}







void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
	if(ErrorStatusITs & FDCAN_IR_BO)
	{
		CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
	}
	if(ErrorStatusITs & FDCAN_IR_EP)
	{
		// MX_FDCAN1_Init();
		// bsp_can_init();
	}
}


//dji
void FDCAN_Start(FDCAN_HandleTypeDef *hfdcan)
{
    /* 启动 FDCAN */
    if (HAL_FDCAN_Start(hfdcan) != HAL_OK)
    {
        Error_Handler();
    }

    /* 激活 FIFO0 新消息中断 */
    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
    }
}

void FDCAN2_Filter_Init(void)
{
    FDCAN_FilterTypeDef sFilterConfig;

    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 2;                        // 选择索引 14，便于与原来 bank 对照
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
    sFilterConfig.FilterID1 = 0x000;
    sFilterConfig.FilterID2 = 0x000;                      // mask 0 -> 接收所有
    if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
	FDCAN_Start(&hfdcan2);
}

void FDCAN3_Filter_Init(void)
{
    FDCAN_FilterTypeDef sFilterConfig;

    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 3;                        // 选择索引 14，便于与原来 bank 对照
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
    sFilterConfig.FilterID1 = 0x000;
    sFilterConfig.FilterID2 = 0x000;                      // mask 0 -> 接收所有
    if (HAL_FDCAN_ConfigFilter(&hfdcan3, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
	FDCAN_Start(&hfdcan3);
}

//fdcan_callback
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	FDCAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[64];
    if (hfdcan == &hfdcan1)
    {
		  fdcan1_rx_callback();
    }
}
//fdcan_callback
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
	FDCAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];

		if (hfdcan == &hfdcan2)
    {
		
      if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
      {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &rx_header, rx_data) != HAL_OK)
        {
            /* 可选：错误处理 */
            return;
        }

        /* 处理电机数据 ID */
				// DJI 3508 / M2006 电机反馈 ID 范围：0x201 ~ 0x207
        if (rx_header.Identifier >= CAN_3508_M1_ID && rx_header.Identifier <= CAN_3508_M7_ID)
        {
            uint8_t i = (uint8_t)(rx_header.Identifier - CAN_3508_M1_ID);
            motor_measure_t *m = NULL;

            
            m = &motor_fdcan2[i];

            if (m)
            {
                m->msg_cnt++;
                if (m->msg_cnt <= 50)
                    get_motor_offset(m, rx_data);
                else
                    get_motor_measure(m, rx_data);
            }
        }
        else
        {
            /* 其它 ID 可按需扩展处理 */
        }
      }
    }
		if (hfdcan == &hfdcan3)
    {
		/* 如果是 FIFO0 新消息中断 */
      if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
      {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &rx_header, rx_data) != HAL_OK)
        {
            /* 可选：错误处理 */
            return;
        }

        /* 处理电机数据 ID */
				// DJI 3508 / M2006 电机反馈 ID 范围：0x201 ~ 0x207
        if (rx_header.Identifier >= CAN_3508_M1_ID && rx_header.Identifier <= CAN_3508_M7_ID)
        {
            uint8_t i = (uint8_t)(rx_header.Identifier - CAN_3508_M1_ID);
            motor_measure_t *m = NULL;

            
            m = &motor_fdcan3[i];

            if (m)
            {
                m->msg_cnt++;
                if (m->msg_cnt <= 50)
                    get_motor_offset(m, rx_data);
                else
                    get_motor_measure(m, rx_data);
            }
        }
        else
        {
            /* 其它 ID 可按需扩展处理 */
        }
      }
    }

}
