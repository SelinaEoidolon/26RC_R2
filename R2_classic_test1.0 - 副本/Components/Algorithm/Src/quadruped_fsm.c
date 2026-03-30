#include "quadruped_fsm.h"
#include "bsp_leg.h"


QFSM_Handle_t g_qfsm;//状态机实例

/**
 * @brief 进入指定状态，并执行该状态的“进入动作”
 *
 * @param fsm        状态机句柄
 * @param next_state 即将进入的新状态
 *
 * @note
 * 该函数是状态切换的核心：
 * 1. 更新当前状态
 * 2. 清零该状态内的节拍计数
 * 3. 执行进入该状态时应立即下发的腿部动作
 */
static void QFSM_EnterState(QFSM_Handle_t *fsm, QFSM_State_t next_state)
{
    if (fsm == 0)
        return;

    /* 更新状态为目标状态 */
    fsm->state = next_state;

    /* 进入新状态后，状态内计时器清零 */
    fsm->tick_in_state = 0;

    /* 根据不同状态，执行不同的“进入动作” */
    switch (next_state)
    {
    case QFSM_STATE1_ALL_LIFT:
        /*
         * 状态1：四条腿全部抬起
         *
         * 进入该状态时，立即给四条腿都下发 LIFT 命令
         */
        Leg_SetCommand(LEG_FL, LEG_CMD_LIFT);
        Leg_SetCommand(LEG_FR, LEG_CMD_LIFT);
        Leg_SetCommand(LEG_RL, LEG_CMD_LIFT);
        Leg_SetCommand(LEG_RR, LEG_CMD_LIFT);

        /* 将设置好的命令真正应用到底层硬件 */
        break;

    case QFSM_STATE2_FRONT_RETRACT:
        /*
         * 状态2：前两条腿收缩，后两条腿不变
         *
         * 前左/前右 -> RETRACT
         * 后左/后右 -> KEEP
         */
        Leg_SetCommand(LEG_FL, LEG_CMD_RETRACT);
        Leg_SetCommand(LEG_FR, LEG_CMD_RETRACT);
        Leg_SetCommand(LEG_RL, LEG_CMD_KEEP);
        Leg_SetCommand(LEG_RR, LEG_CMD_KEEP);

        /* 应用本状态动作 */
        break;

    case QFSM_STATE3_REAR_RETRACT:
        /*
         * 状态3：后两条腿收缩，前两条腿保持不变
         *
         * 前左/前右 -> KEEP
         * 后左/后右 -> RETRACT
         */
        Leg_SetCommand(LEG_FL, LEG_CMD_KEEP);
        Leg_SetCommand(LEG_FR, LEG_CMD_KEEP);
        Leg_SetCommand(LEG_RL, LEG_CMD_RETRACT);
        Leg_SetCommand(LEG_RR, LEG_CMD_RETRACT);

        /* 应用本状态动作 */
        break;

    case QFSM_DONE:
        /*
         * DONE：动作序列执行完成
         *
         * 这里将 running 置 false，表示状态机停止
         * 后续即使继续调用 QFSM_Process100ms()，也不会再推进状态
         */
        fsm->running = false;
        break;

    case QFSM_IDLE:
    default:
        /*
         * IDLE：空闲状态
         * 默认不做动作
         */
        break;
    }
}

/**
 * @brief 初始化状态机
 *
 * @param fsm 状态机句柄
 *
 * @note
 * 此函数一般在任务启动时调用一次。
 */
void QFSM_Init(QFSM_Handle_t *fsm)
{
    if (fsm == 0)
        return;

    /* 初始为空闲状态 */
    fsm->state = QFSM_IDLE;

    /* 当前状态内的节拍计数清零 */
    fsm->tick_in_state = 0;

    /*
     * 配置各状态默认持续时间
     * 单位是“100ms”
     *
     * 例如 5 表示持续 500ms
     */
    fsm->hold_tick_s1 = 50;
    fsm->hold_tick_s2 = 50;
    fsm->hold_tick_s3 = 50;

    /* 初始为未运行 */
    fsm->running = false;
}

/**
 * @brief 启动状态机
 *
 * @param fsm 状态机句柄
 *
 * @note
 * 调用后：
 * 1. running = true
 * 2. 立即切换到状态1并执行状态1动作
 */
void QFSM_Start(QFSM_Handle_t *fsm)
{
    if (fsm == 0)
        return;

    /* 标记状态机开始运行 */
    fsm->running = true;

    /* 从状态1开始执行 */
    QFSM_EnterState(fsm, QFSM_STATE1_ALL_LIFT);
}

/**
 * @brief 停止状态机
 *
 * @param fsm 状态机句柄
 *
 * @note
 * 只做逻辑层面的停止，不主动清硬件动作。
 * 如果你希望停止时让腿回到某个安全位置，可以在这里补充控制逻辑。
 */
void QFSM_Stop(QFSM_Handle_t *fsm)
{
    if (fsm == 0)
        return;

    /* 停止运行 */
    fsm->running = false;

    /* 状态恢复为空闲 */
    fsm->state = QFSM_IDLE;

    /* 清除状态内计数 */
    fsm->tick_in_state = 0;
}

/**
 * @brief 每 100ms 调用一次的状态机推进函数
 *
 * @param fsm 状态机句柄
 *
 * @note
 * 这是状态机的核心“时序推进”函数：
 * - 每调用一次，相当于过去了 100ms
 * - 先累计当前状态停留时间
 * - 达到阈值后切换到下一个状态
 */
void QFSM_Process100ms(QFSM_Handle_t *fsm)
{
    /* 空指针保护 */
    if ((fsm == 0) || (fsm->running == false))
        return;

    /* 
     * 每进入一次本函数，就说明又过去了一个 100ms 周期
     * 因此当前状态持续时间 +1
     */
    fsm->tick_in_state++;

    /* 根据当前状态决定是否需要切换 */
    switch (fsm->state)
    {
    case QFSM_STATE1_ALL_LIFT:
        /*
         * 状态1保持到指定时长后，切换到状态2
         */
        if (fsm->tick_in_state >= fsm->hold_tick_s1)
        {
            QFSM_EnterState(fsm, QFSM_STATE2_FRONT_RETRACT);
        }
        break;

    case QFSM_STATE2_FRONT_RETRACT:
        /*
         * 状态2保持到指定时长后，切换到状态3
         */
        if (fsm->tick_in_state >= fsm->hold_tick_s2)
        {
            QFSM_EnterState(fsm, QFSM_STATE3_REAR_RETRACT);
        }
        break;

    case QFSM_STATE3_REAR_RETRACT:
        /*
         * 状态3保持到指定时长后，切换到 DONE
         */
        if (fsm->tick_in_state >= fsm->hold_tick_s3)
        {
            QFSM_EnterState(fsm, QFSM_DONE);
        }
        break;

    case QFSM_DONE:
        /*
         * DONE 状态下不再继续推进
         * 因为 running 在进入 DONE 时已被置为 false
         */
        break;

    case QFSM_IDLE:
    default:
        /*
         * IDLE / 默认情况：不做任何处理
         */
        break;
    }
}

/**
 * @brief 获取当前状态
 *
 * @param fsm 状态机句柄
 * @return 当前状态；若参数非法则返回 IDLE
 */
QFSM_State_t QFSM_GetState(QFSM_Handle_t *fsm)
{
    if (fsm == 0)
        return QFSM_IDLE;

    return fsm->state;
}