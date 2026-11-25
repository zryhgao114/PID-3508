#ifndef __CAN_PORT_H__
#define __CAN_PORT_H__

/**
 * ============================================================================
 *  can_port.h  ——  CAN 硬件抽象层（JOINT / STM32H723VGT6 专用）
 *
 *  CubeMX 已经帮你配好了：引脚、时钟、位时序（1Mbps）、Message RAM 分区。
 *  它「没有」帮你做的四件事，全在这个文件里：
 *      1. 配置过滤器         HAL_FDCAN_ConfigFilter
 *      2. 启动外设           HAL_FDCAN_Start
 *      3. 使能接收中断       HAL_FDCAN_ActivateNotification
 *      4. 收发与分发         中断回调 + 发送队列
 *
 *  为什么这里自己声明 extern 而不是 include "fdcan.h"：
 *    万一你在 CubeMX 里没勾「每外设生成 .c/.h 对」，fdcan.h 就不存在，
 *    编译直接报错。自己声明一句 extern 最省事，也不依赖生成方式。
 * ============================================================================
 */

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* ---- 总线编号：调用时用这个，不要直接传 &hfdcan1 ---- */
#define CAN_BUS_1   1u
#define CAN_BUS_2   2u
#define CAN_BUS_3   3u

/* 是否启用第三路。不用就在 CubeMX 里取消 FDCAN3，并把这里改成 0 */
#define CAN_USE_BUS3   1

/* 发送队列深度：按 1ms 发 4 帧算，够缓冲 7ms 的突发 */
#define CAN_TX_QUEUE_LEN   30

/** 一帧待发送的数据，放进队列里传递 */
typedef struct
{
    uint8_t  bus;       /* CAN_BUS_1 / 2 / 3 */
    uint32_t id;        /* 11 位标准 ID */
    uint8_t  data[8];   /* 载荷，恒为 8 字节 */
} CanFrame_t;

/* CubeMX 生成的外设句柄，在这里声明一次，后面所有文件都能用 */
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
#if CAN_USE_BUS3
extern FDCAN_HandleTypeDef hfdcan3;
#endif

/* 发送队列。必须在 CAN_Port_Init 之前创建，顺序反了会在中断里用空队列 */
extern QueueHandle_t Queue_CanSend;

/* 计数器，用于判断"到底是没发出去还是没收进来" */
extern volatile uint32_t Can_RxCount[4];
extern volatile uint32_t Can_TxCount;

/* 发送失败次数：TX FIFO 满 / 总线错误 / 无 ACK 都会记在这里。
   排查时看这个：
     Can_TxCount 涨、Can_TxErrCount 不涨、电机不动 → 帧发出去了，电机那头的事
     Can_TxErrCount 一直涨                        → 线没接好 / 无 ACK / 无终端电阻 */
extern volatile uint32_t Can_TxErrCount;

/**
 * @brief  初始化全部 FDCAN：过滤器 + 全局过滤 + 启动 + 使能中断
 * @note   调用前 Queue_CanSend 必须已创建
 * @retval 0 成功，非 0 表示第几路失败
 */
uint8_t CAN_Port_Init(void);

/**
 * @brief  直接发送一帧（不进队列）。只用于上电自检
 */
HAL_StatusTypeDef CAN_Transmit(FDCAN_HandleTypeDef *hfdcan, uint32_t id, const uint8_t d[8]);

/**
 * @brief  把一帧丢进队列，由 Task_CAN 真正发出去
 * @retval 0 入队成功，1 队列满
 * @note   超时为 0：队列满时立刻返回失败，绝不阻塞控制任务
 */
uint8_t CAN_Send(uint8_t bus, uint32_t id, const uint8_t d[8]);

#endif /* __CAN_PORT_H__ */
