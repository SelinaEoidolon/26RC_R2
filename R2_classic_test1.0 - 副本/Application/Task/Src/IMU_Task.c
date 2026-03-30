
#include "cmsis_os.h"
#include "IMU_Task.h"
#include "bsp_tick.h"
#include "wit_c_sdk.h"
#include "imu.h"


extern UART_HandleTypeDef huart7;

void IMU_Task(void const * argument)
{
  /* USER CODE BEGIN IMU_Task */
	IMU_Init();

  /* Infinite loop */
  for(;;)
  {
		osDelay(10);
		
    osDelay(1);
  }
  /* USER CODE END IMU_Task */
}
