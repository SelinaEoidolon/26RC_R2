#include "bsp_usb.h"
#include "arm_user.h"
#include "arm_ik_3r_safe_stm32h7.h"



/* 组件句柄 */
Arm3R_Handle_t g_arm_ik;

/*
 *解算结果回包
 */
static void ArmIK_SendResult(uint8_t status_code, uint8_t unsafe_reason)
{
    uint8_t tx_data[2];

    tx_data[0] = status_code;
    tx_data[1] = unsafe_reason;

    Send_Cmd_Data(USB_CMD_ARM_IK_RESULT, tx_data, 2);
}

/*
 * 组件初始化
 * 一般在 MX 初始化结束后、进入主循环前调用一次。
 */
void ArmIK_ComponentInit(void)
{
    Arm3R_Config_t cfg;

    cfg.link.d1 = 0.0f;
    cfg.link.a2 = 320.0f;
    cfg.link.a3 = 320.0f;

    cfg.j1_ref.model_ref = Arm3R_DegToRad(0.0f);
    cfg.j2_ref.model_ref = Arm3R_DegToRad(70.0f);
    cfg.j3_ref.model_ref = Arm3R_DegToRad(-145.0f);

    cfg.j1_ref.dir = +1;
    cfg.j2_ref.dir = -1;
    cfg.j3_ref.dir = +1;

    Arm3R_Init(&g_arm_ik, &cfg);
}

/*
 * 周期调用示例
 * 输入目标点，得到几何控制角，再交给你自己的电机控制层。
 */
void ArmIK_ComponentStep(float x, float y, float z)
{
    Arm3R_Status_t ret;
    const Arm3R_Result_t *res;
    Arm3R_CtrlAngles_t motor_ctrl;

    ret = Arm3R_Solve(&g_arm_ik, x, y, z, 0.0f);
    res = Arm3R_GetResult(&g_arm_ik);

    if (ret == ARM3R_OK)
    {
        /*
         * res->ctrl 是“几何控制角”
         * 如果你的电机层直接接收这套定义，就直接用 res->ctrl。
         *
         * 如果你的电机层需要考虑方向正负，可以先做一次 dir 映射。
         */
        Arm3R_GeomCtrlToMotorCtrl(&res->ctrl, &g_arm_ik.cfg, &motor_ctrl);

        /* ==================== 接这里 ==================== */
        /* 下面三句换成你自己的电机控制接口 */
        /* Motor_SetTargetRad(JOINT1, motor_ctrl.j1); */
        /* Motor_SetTargetRad(JOINT2, motor_ctrl.j2); */
        /* Motor_SetTargetRad(JOINT3, motor_ctrl.j3); */

        /* 回传：目标点可执行 */
        ArmIK_SendResult(ARM_IK_RESULT_OK, 0U);
    }
    else if (ret == ARM3R_ERR_UNREACHABLE)
    {
        /* 回传：目标点不可达 */
        ArmIK_SendResult(ARM_IK_RESULT_UNREACHABLE, 0U);
    }
    else if (ret == ARM3R_ERR_UNSAFE)
    {
        /* 回传：目标点可达但不安全，同时返回具体原因 */
        if (res != 0)
        {
            ArmIK_SendResult(ARM_IK_RESULT_UNSAFE, (uint8_t)res->unsafe_reason);
        }
        else
        {
            ArmIK_SendResult(ARM_IK_RESULT_UNSAFE, 0U);
        }
    }
    else
    {
        /* 参数错误、未初始化等 */
        ArmIK_SendResult(ARM_IK_RESULT_PARAM_ERR, 0U);
    }
}
