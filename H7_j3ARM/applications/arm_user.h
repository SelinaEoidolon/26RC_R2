#include "arm_ik_3r_safe_stm32h7.h"
#include "bsp_usb.h"

#define ARM_IK_RESULT_OK            0U
#define ARM_IK_RESULT_UNREACHABLE   1U
#define ARM_IK_RESULT_UNSAFE        2U
#define ARM_IK_RESULT_PARAM_ERR     3U

extern Arm3R_Handle_t g_arm_ik;



void ArmIK_ComponentInit(void);
void ArmIK_ComponentStep(float x, float y, float z);




