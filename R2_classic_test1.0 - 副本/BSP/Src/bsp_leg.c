#include "bsp_leg.h"
#include "stm32h7xx.h" 




/**
 * @brief 保存四条腿当前“待执行”的命令
 *
 * 下标与 LegId_t 对应：
 * s_leg_cmd[LEG_FL] -> 前左腿命令
 * s_leg_cmd[LEG_FR] -> 前右腿命令
 * s_leg_cmd[LEG_RL] -> 后左腿命令
 * s_leg_cmd[LEG_RR] -> 后右腿命令
 *
 * 初始值全部为 KEEP，表示默认不对输出做额外修改。
 */
/* 
 * 当前腿部目标命令缓冲
 * 由状态机（可在TIM4中断中）写入
 * 由执行任务读取
 */
static volatile LegCmd_t s_leg_cmd[LEG_NUM] = {LEG_CMD_KEEP};

/* 
 * 命令版本号：
 * 每当命令发生变化，就递增一次
 * 执行任务可通过它判断是否有新命令
 */
static volatile uint32_t s_leg_cmd_version = 0;

/**
 * @brief 设置指定腿的控制命令
 *
 * @param leg 腿编号
 * @param cmd 要设置的命令
 *
 * @note
 * 1. 这里先做参数合法性判断，防止数组越界
 * 2. KEEP 的语义是“保持当前状态”，因此这里不覆盖原命令
 *    这样可以避免误把原本已经设好的动作清掉
 */
void Leg_SetCommand(LegId_t leg, LegCmd_t cmd)
{
    /* 参数检查：防止传入非法编号导致数组越界 */
    if (leg >= LEG_NUM)
        return;

    /* KEEP 表示保持当前状态，不改命令 */
    if (cmd == LEG_CMD_KEEP)
        return;

    /* 只有命令真的变化了，才更新并递增版本号 */
    if (s_leg_cmd[leg] != cmd)
    {
        s_leg_cmd[leg] = cmd;
        s_leg_cmd_version++;
    }
}

uint32_t Leg_GetCommandVersion(void)
{
    return s_leg_cmd_version;
}

void Leg_GetCommandSnapshot(LegCmd_t out_cmd[LEG_NUM])
{
    uint32_t primask;
    uint8_t i;

    if (out_cmd == 0)
        return;

    /* 
     * 短时间关中断，确保复制4个命令时不会被TIM4中断打断
     * 这样拿到的是一整帧一致的命令
     */
    primask = __get_PRIMASK();
    __disable_irq();

    for (i = 0; i < LEG_NUM; i++)
    {
        out_cmd[i] = (LegCmd_t)s_leg_cmd[i];
    }

    if (!primask)
    {
        __enable_irq();
    }
}

