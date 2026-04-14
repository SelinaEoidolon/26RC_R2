#ifndef BSP_MCU_H
#define BSP_MCU_H


static void DM_Moter_Init(void);
static void DJI_Moter_Init(void);
static void H7_power(void); 
static void DM_Moter1_Init(int Mode);
static void DM_Moter2_Init(int Mode);
static void DM_Moter3_Init(int Mode);
static void DM_Moter4_Init(int Mode);
static void DM_Moter5_Init(int Mode);
static void DM_Moter6_Init(int Mode);
	
extern void MCU_Init(void);

	
#endif //BSP_MCU_H

