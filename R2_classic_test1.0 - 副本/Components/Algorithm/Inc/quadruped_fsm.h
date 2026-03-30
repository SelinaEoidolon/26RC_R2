#ifndef __QUADRUPED_FSM_H
#define __QUADRUPED_FSM_H

#include <stdint.h>
#include <stdbool.h>
#include "bsp_leg.h"
 



/**
 * @brief 四足动作状态机的状态定义
 *
 * 状态流转顺序：
 * IDLE -> STATE1 -> STATE2 -> STATE3 -> DONE
 *
 * 各状态含义：
 * IDLE              : 空闲状态，状态机未运行
 * STATE1_ALL_LIFT   : 四条腿全部抬起
 * STATE2_FRONT_RETRACT : 前两条腿收缩，后两条腿保持不变
 * STATE3_REAR_RETRACT  : 后两条腿收缩
 * DONE              : 动作序列执行结束
 */
typedef enum
{
    QFSM_IDLE = 0,
    QFSM_STATE1_ALL_LIFT,        // 状态1：四条腿抬起
    QFSM_STATE2_FRONT_RETRACT,   // 状态2：前腿收缩，后腿不变
    QFSM_STATE3_REAR_RETRACT,    // 状态3：后腿收缩
    QFSM_DONE                    // 结束状态
} QFSM_State_t;

/**
 * @brief 状态机控制句柄
 *
 * 说明：
 * 用一个结构体保存状态机运行时的所有上下文，
 * 便于后续扩展为多个实例或增加更多控制参数。
 */
typedef struct
{
    QFSM_State_t state;      // 当前所处状态
    uint32_t tick_in_state;  // 当前状态已经持续了多少个“100ms节拍”

    /*
     * 以下三个参数决定每个状态保持的时长
     * 单位：100ms
     *
     * 举例：
     * hold_tick_s1 = 5 -> 状态1持续 5 * 100ms = 500ms
     */
    uint32_t hold_tick_s1;   // 状态1持续时长
    uint32_t hold_tick_s2;   // 状态2持续时长
    uint32_t hold_tick_s3;   // 状态3持续时长

    bool running;            // 状态机是否正在运行
} QFSM_Handle_t;


extern QFSM_Handle_t g_qfsm;//状态机实例
/**
 * @brief 初始化状态机对象
 *
 * @param fsm 状态机句柄指针
 *
 * @note
 * 该函数会：
 * - 设置初始状态为 IDLE
 * - 清空计数器
 * - 配置默认状态保持时间
 * - 标记为未运行
 */
void QFSM_Init(QFSM_Handle_t *fsm);

/**
 * @brief 启动状态机
 *
 * @param fsm 状态机句柄指针
 *
 * @note
 * 启动后立即进入状态1（四条腿抬起）
 */
void QFSM_Start(QFSM_Handle_t *fsm);

/**
 * @brief 停止状态机
 *
 * @param fsm 状态机句柄指针
 *
 * @note
 * 停止后状态切回 IDLE，并清除当前节拍计数
 */
void QFSM_Stop(QFSM_Handle_t *fsm);

/**
 * @brief 每 100ms 调用一次的状态机处理函数
 *
 * @param fsm 状态机句柄指针
 *
 * @note
 * 这是状态机的“节拍驱动入口”。
 * 推荐由 TIM4 100ms 中断通知任务后，在任务中调用本函数。
 */
void QFSM_Process100ms(QFSM_Handle_t *fsm);

/**
 * @brief 获取当前状态机状态
 *
 * @param fsm 状态机句柄指针
 * @return 当前状态
 */
QFSM_State_t QFSM_GetState(QFSM_Handle_t *fsm);

#endif