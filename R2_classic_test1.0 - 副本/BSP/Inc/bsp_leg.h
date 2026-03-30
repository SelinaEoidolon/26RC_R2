#ifndef __BSP_LEG_H
#define __BSP_LEG_H

#include <stdint.h>

/**
 * @brief 四条腿的编号定义
 *
 * 命名说明：
 * FL = Front Left  前左
 * FR = Front Right 前右
 * RL = Rear Left   后左
 * RR = Rear Right  后右
 */
typedef enum
{
    LEG_FL = 0,   // 前左腿
    LEG_FR,       // 前右腿
    LEG_RL,       // 后左腿
    LEG_RR,       // 后右腿
    LEG_NUM       // 腿的总数量，便于数组大小定义和遍历
} LegId_t;

/**
 * @brief 腿部控制命令
 *
 * 说明：
 * KEEP    : 保持当前状态，不修改当前输出
 * LIFT    : 抬起
 * RETRACT : 收缩
 */
typedef enum
{
    LEG_CMD_KEEP = 0,   // 保持当前状态
    LEG_CMD_LIFT,       // 抬起
    LEG_CMD_RETRACT     // 收缩
} LegCmd_t;

/**
 * @brief 设置某条腿的目标命令
 *
 * @param leg 要控制的腿编号
 * @param cmd 目标命令
 *
 * @note
 * 这里仅仅是“写入目标命令”，不一定立即作用到硬件。
 * 真正把命令下发到硬件，通常通过 Leg_Apply() 统一执行。
 */
void Leg_SetCommand(LegId_t leg, LegCmd_t cmd);

/* 新增：获取命令版本号，用于判断命令是否更新 */
uint32_t Leg_GetCommandVersion(void);

/* 新增：读取当前命令快照（给其他任务按需使用） */
void Leg_GetCommandSnapshot(LegCmd_t out_cmd[LEG_NUM]);

#endif