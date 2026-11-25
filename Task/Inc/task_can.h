#ifndef __TASK_CAN_H__
#define __TASK_CAN_H__

/**
 * ============================================================================
 *  task_can.h  ——  任务层
 *
 *  为什么任务层要单独一个文件：
 *    CubeMX 会在 freertos.c 里生成一个「弱定义」的 Task_Init。
 *    你在这里写「强定义」的同名函数，链接器会优先用你的，
 *    所以你一行都不用改 CubeMX 生成的文件，重新生成也不会丢代码。
 * ============================================================================
 */

#include "main.h"

/**
 * @brief  FreeRTOS 入口任务（覆盖 freertos.c 里的弱定义）
 * @note   顺序：建队列 → 初始化 CAN → 登记电机 → 建任务 → 删掉自己
 */
void App_Init(void);

/** @brief 发送任务：阻塞在队列上，来一帧发一帧 */
void Task_CAN(void *argument);

/** @brief 控制任务：1kHz，读反馈 → 算 PID → 发命令（大疆 6020） */
void Task_Motor(void *argument);

/** @brief 打印任务：低优先级，每 500ms 打一行，绝不放进控制任务 */
void Task_Print(void *argument);

#endif /* __TASK_CAN_H__ */
