/**
 * ============================================================================
 *  can_port.c  ——  CAN 硬件抽象层实现（JOINT / STM32H723VGT6 专用）
 *
 *  三路 CAN 的 FIFO 分配：
 *      FDCAN1  →  RX FIFO0  →  CAN_BUS_1
 *      FDCAN2  →  RX FIFO1  →  CAN_BUS_2
 *      FDCAN3  →  RX FIFO0  →  CAN_BUS_3
 *
 *  为什么 CAN2 特意走 FIFO1：
 *    这样能顺带验证你 NVIC 里 IT0 / IT1 是不是都开了。只开 IT0 的话，
 *    CAN2 永远收不到，而且不报任何错误——这是最高频的坑，用一次就记住了。
 * ============================================================================
 */

#include "can_port.h"
#include "motor_dji.h"


QueueHandle_t Queue_CanSend = NULL;

volatile uint32_t Can_RxCount[4] = {0};
volatile uint32_t Can_TxCount = 0;
volatile uint32_t Can_TxErrCount = 0;

/* ===========================================================================
 *  一、初始化
 * =========================================================================*/

/**
 * @brief  给一路 FDCAN 配过滤器、启动、开接收中断
 * @param  hfdcan   外设句柄
 * @param  to_fifo  FDCAN_FILTER_TO_RXFIFO0 或 FDCAN_FILTER_TO_RXFIFO1
 * @retval 0 成功
 */
static uint8_t CAN_Port_InitOne(FDCAN_HandleTypeDef *hfdcan, uint32_t to_fifo)
{
    FDCAN_FilterTypeDef f;

    f.IdType       = FDCAN_STANDARD_ID;   /* 只关心 11 位标准帧 */
    f.FilterIndex  = 0;                   /* 每路 FDCAN 各有一套过滤器，索引都从 0 开始 */
    f.FilterType   = FDCAN_FILTER_MASK;   /* 掩码模式 */
    f.FilterConfig = to_fifo;             /* 匹配的帧进哪个 FIFO */
    f.FilterID1    = 0x000;               /* 期望 ID */
    f.FilterID2    = 0x000;               /* 掩码全 0 = 不比较任何位 = 所有帧都放行 */

    /* 掩码全 0 是调试期最省事的配法：先确认链路通，再收紧。
       等协议稳定了改成精确匹配，例如只收 0x205：
           FilterID1 = 0x205;   FilterID2 = 0x7FF;   */
    if (HAL_FDCAN_ConfigFilter(hfdcan, &f) != HAL_OK)
        return 1;

    /* 扩展帧过滤器：同样配一个全 0 掩码的，等于"扩展帧也全放行"。
       为什么需要：达妙电机在某些上位机配置下走的是 29 位扩展帧。
       只配标准帧却收到扩展帧时，现象是"一帧都进不来"，而且不报错。 */
    f.IdType      = FDCAN_EXTENDED_ID;
    f.FilterIndex = 0;                    /* 扩展帧的索引是另一套，也从 0 开始 */
    if (HAL_FDCAN_ConfigFilter(hfdcan, &f) != HAL_OK)
        return 1;

    /* H7 上这一句不写，很可能一帧都进不来。
       四个参数：不匹配的标准帧 / 不匹配的扩展帧 / 标准远程帧 / 扩展远程帧 怎么处理。
       这里全部拒绝 —— 现在掩码全 0 所以所有帧都"匹配"，走不到拒绝分支；
       以后收紧过滤时，这条会把无关帧挡在硬件层，CPU 少被打断。 */
    if (HAL_FDCAN_ConfigGlobalFilter(hfdcan,
                                     FDCAN_REJECT, FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK)
        return 1;

    /* 启动：不调用它，CAN 控制器一直停在初始化态，永远收发不了 */
    if (HAL_FDCAN_Start(hfdcan) != HAL_OK)
        return 1;

    /* 使能"新消息到达"中断：不调用它，帧会静默堆在 FIFO 里没人取 */
    if (to_fifo == FDCAN_FILTER_TO_RXFIFO0)
    {
        if (HAL_FDCAN_ActivateNotification(hfdcan,
                FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
            return 1;
    }
    else
    {
        if (HAL_FDCAN_ActivateNotification(hfdcan,
                FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)
            return 1;
    }

    /* 使能 Bus-Off 错误中断：某台电机异常会触发 Bus-Off，
       若不使能并自愈，会形成中断风暴拖死 CPU（见 ErrorStatusCallback） */
    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_BUS_OFF, 0) != HAL_OK)
        return 1;

    return 0;
}

uint8_t CAN_Port_Init(void)
{
    if (CAN_Port_InitOne(&hfdcan1, FDCAN_FILTER_TO_RXFIFO0) != 0) return 1;
    if (CAN_Port_InitOne(&hfdcan2, FDCAN_FILTER_TO_RXFIFO1) != 0) return 2;
#if CAN_USE_BUS3
    if (CAN_Port_InitOne(&hfdcan3, FDCAN_FILTER_TO_RXFIFO0) != 0) return 3;
#endif
    return 0;
}

/* ===========================================================================
 *  二、发送
 * =========================================================================*/

HAL_StatusTypeDef CAN_Transmit(FDCAN_HandleTypeDef *hfdcan, uint32_t id, const uint8_t d[8])
{
    FDCAN_TxHeaderTypeDef tx;

    tx.Identifier          = id;
    tx.IdType              = FDCAN_STANDARD_ID;   /* 标准帧 11 位 */
    tx.TxFrameType         = FDCAN_DATA_FRAME;    /* 数据帧，不是远程帧 */
    tx.DataLength          = FDCAN_DLC_BYTES_8;   /* 电机协议恒为 8 字节 */
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch       = FDCAN_BRS_OFF;       /* 经典 CAN，不开变速 */
    tx.FDFormat            = FDCAN_CLASSIC_CAN;   /* 不是 CAN-FD */
    tx.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker       = 0;

    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx, (uint8_t *)d) != HAL_OK)
    {
        /* 最常见的原因：TX FIFO 满（8 深）。
           如果这里一直失败而总线是通的，多半是没人 ACK —— 也就是
           CAN_H/CAN_L 接反、电机没上电、或者只接了板子这一端。 */
        Can_TxErrCount++;
        return HAL_ERROR;
    }

    Can_TxCount++;
    return HAL_OK;
}

uint8_t CAN_Send(uint8_t bus, uint32_t id, const uint8_t d[8])
{
    CanFrame_t f;
    uint8_t i;

    if (Queue_CanSend == NULL)
        return 1;

    f.bus = bus;
    f.id  = id;
    for (i = 0; i < 8; i++)
        f.data[i] = d[i];

    /* 超时 0：队列满就立刻失败，绝不阻塞调用者。
       控制任务里发不出去只是丢一帧，下一个周期还有机会。 */
    return (xQueueSend(Queue_CanSend, &f, 0) == pdTRUE) ? 0 : 1;
}

/* ===========================================================================
 *  三、接收：中断回调 → 分发
 *
 *  ⚠️ 以下函数运行在中断上下文：
 *      - 不能调用 HAL_Delay / vTaskDelay
 *      - 不能 printf（阻塞 + 不可重入）
 *      - 要把数据交给任务，只能用 xQueueSendFromISR
 * =========================================================================*/

/* ---- Bus-Off / 错误自愈（中断上下文，极轻量）----
 *  当某台电机异常（掉线、短接、发错误帧）时，FDCAN 会进入 Bus-Off，
 *  若不处理会持续触发错误中断，形成「中断风暴」把 CPU 拖死，
 *  现象是串口一个字都不出、电机全停。这里检测到 Bus-Off 就主动
 *  复位该路 FDCAN 控制器，避免风暴。 */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
    (void)ErrorStatusITs;

    /* 只有 Bus-Off 才需要复位（其他错误 HAL 自己能清） */
    if ((hfdcan->Instance->PSR & FDCAN_PSR_BO) != 0u)
    {
        Can_TxErrCount++;

        /* 复位控制器：Stop 再 Start，让 FDCAN 从 Bus-Off 恢复 */
        if (HAL_FDCAN_Stop(hfdcan) == HAL_OK)
        {
            (void)HAL_FDCAN_Start(hfdcan);
        }
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef rx;
    uint8_t d[8] = {0};

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0)
        return;

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx, d) != HAL_OK)
        return;

    /* 大疆走标准帧，所以这里不再按 IdType 过滤，
       驱动内部会自己判断"这条帧是不是我的"。
       MotorDJI_OnFrame 只看总线和负载里的电机 ID，帧 ID 长什么样都不影响。 */

    if (hfdcan->Instance == FDCAN1)
    {
        Can_RxCount[CAN_BUS_1]++;
        MotorDJI_OnFrame(CAN_BUS_1, rx.Identifier, d);
        
    }
#if CAN_USE_BUS3
    else if (hfdcan->Instance == FDCAN3)
    {
        Can_RxCount[CAN_BUS_3]++;
        MotorDJI_OnFrame(CAN_BUS_3, rx.Identifier, d);
        
    }
#endif
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    FDCAN_RxHeaderTypeDef rx;
    uint8_t d[8] = {0};

    if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) == 0)
        return;

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &rx, d) != HAL_OK)
        return;

    if (hfdcan->Instance == FDCAN2)
    {
        Can_RxCount[CAN_BUS_2]++;
        MotorDJI_OnFrame(CAN_BUS_2, rx.Identifier, d);
       
    }
}
