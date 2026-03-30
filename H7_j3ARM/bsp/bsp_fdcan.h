#ifndef __BSP_FDCAN_H__
#define __BSP_FDCAN_H__
#include "main.h"
#include "fdcan.h"

#define hcan_t FDCAN_HandleTypeDef


//dji
void FDCAN_Start(FDCAN_HandleTypeDef *hfdcan);
void FDCAN2_Filter_Init(void);
void FDCAN3_Filter_Init(void);

#endif /* __BSP_FDCAN_H_ */

