/**
 * ============================================================================
 *  WT931_Gyro.c  ——  维特智能 WT931 驱动实现（干净独立版）
 *
 *  相对老队员 Engineer_CusCtrlr 的 WT931_Gyro.c，做了三处修正：
 *
 *  1. 去耦合：老队员拿 Encoder_t（Gimbal_Gyro）存陀螺仪数据，本文件用独立
 *     的 WT931_Data_t，字段语义清晰，不碰电机/编码器结构体。
 *
 *  2. 多圈 Yaw 解算独立：老队员的跨圈判断写在 WT931_Process 里、且依赖
 *     Gimbal_Gyro.Angle_Z_Last，本文件把「Yaw 0~360 → 连续多圈 AllYaw」
 *     收敛进 WT931_Data_t，输出可直接喂给底盘自转闭环。
 *
 *  3. 串口/波特率可配：WT931_UART / WT931_BAUDRATE 宏，接线变了只改头文件。
 *
 *  协议要点（维特智能私有协议）：
 *    帧头 0x55，第二字节是帧类型，0x53 = 角度帧。
 *    角度帧数据区 11 字节，Roll/Pitch/Yaw 各 2 字节小端（short）。
 *    换算：raw(short) / 32768 * 180 = 角度（°）。
 *
 *  接收方式：UART + DMA + IDLE 中断。IDLE 表示一帧结束，配合 DMA 计数器
 *  得到本帧长度。这是接收变长/定长串口帧最省 CPU 的方式（老队员同款思路）。
 * ============================================================================
 */

#include "WT931_Gyro.h"
#include "dma.h"
#include "string.h"

WT931_Data_t WT931_Data;

/* ---- DMA 接收缓冲计数（IDLE 中断里读）---- */
static uint32_t wt931_rx_len = 0;

/**
 * @brief  初始化 WT931
 */
void WT931_Init(void)
{
    /* 数据清零 */
    WT931_Data.FrameCounter = 0;
    WT931_Data.Roll   = 0.0f;
    WT931_Data.Pitch  = 0.0f;
    WT931_Data.Yaw    = 0.0f;
    WT931_Data.YawCirnum = 0;
    WT931_Data.AllYaw    = 0.0f;
    WT931_Data.YawLast   = 0.0f;
    WT931_Data.FirstFrame = 0;

    for (uint16_t i = 0; i < WT931_RXBUFFER_LEN; i++)
        WT931_Data.RxBuffer[i] = 0;

    /* 波特率（若 CubeMX 里不是 115200，这里按宏校正；实际建议在 CubeMX 里改好） */
    /* 注：HAL 的 UART 初始化在 main 里已按 CubeMX 配置好波特率，
       这里不重复 Init，只负责使能中断 + 启动 DMA 接收。 */
    (void)WT931_BAUDRATE;

    /* 手动使能 UART7 的 NVIC 中断（CubeMX 未启用 UART7 全局中断，
       这里直接调 HAL_NVIC，无需重跑 CubeMX） */
    HAL_NVIC_SetPriority(UART7_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(UART7_IRQn);

    /* 使能 IDLE 中断（一帧结束的标志） */
    __HAL_UART_ENABLE_IT(WT931_UART, UART_IT_IDLE);

    /* 启动 DMA 接收 */
    HAL_UART_Receive_DMA(WT931_UART, WT931_Data.RxBuffer, WT931_RXBUFFER_LEN);
}

/**
 * @brief  UART 中断回调（IDLE 中断处理）
 * @note   在 HAL_UART_IRQHandler 里、或 stm32h7xx_it.c 的串口中断里调用。
 *         本工程统一在 stm32h7xx_it.c 里转发到本函数（见集成说明）。
 */
void WT931_CallBack(UART_HandleTypeDef *huart)
{
    DMA_HandleTypeDef *hdma_rx;

    if (huart != WT931_UART)
        return;

    /* 只处理 IDLE 中断（一帧收完） */
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) == RESET)
        return;

    __HAL_UART_CLEAR_IDLEFLAG(huart);

    /* 停止 DMA，读已收字节数 */
    HAL_UART_DMAStop(huart);
    hdma_rx = huart->hdmarx;
    if (hdma_rx != NULL)
        wt931_rx_len = WT931_RXBUFFER_LEN - __HAL_DMA_GET_COUNTER(hdma_rx);
    else
        wt931_rx_len = WT931_RXBUFFER_LEN;

    /* 解析（传入本帧长度做校验） */
    WT931_Process();

    /* 重新启动 DMA 接收 */
    HAL_UART_Receive_DMA(huart, WT931_Data.RxBuffer, WT931_RXBUFFER_LEN);
}

/**
 * @brief  解析一帧角度数据
 * @note   校验：帧头 0x55 + 类型 0x53 + 长度足够。
 */
void WT931_Process(void)
{
    WT931_AngleRaw_t raw;
    float yaw_now;

    /* 帧头校验 */
    if (WT931_Data.RxBuffer[0] != 0x55)
        return;

    /* 帧类型：0x53 = 角度帧 */
    if (WT931_Data.RxBuffer[1] != 0x53)
        return;

    /* 长度校验（至少 11 字节） */
    if (wt931_rx_len < WT931_RXBUFFER_LEN)
        return;

    WT931_Data.FrameCounter++;

    /* 数据区从第 2 字节开始：Roll[2..3] Pitch[4..5] Yaw[6..7]（小端 short） */
    raw.Angle_X = (short)((uint16_t)WT931_Data.RxBuffer[2]
                | ((uint16_t)WT931_Data.RxBuffer[3] << 8));
    raw.Angle_Y = (short)((uint16_t)WT931_Data.RxBuffer[4]
                | ((uint16_t)WT931_Data.RxBuffer[5] << 8));
    raw.Angle_Z = (short)((uint16_t)WT931_Data.RxBuffer[6]
                | ((uint16_t)WT931_Data.RxBuffer[7] << 8));

    /* 换算成角度（°），raw/32768*180 */
    WT931_Data.Roll  = (float)raw.Angle_X / 32768.0f * 180.0f;
    WT931_Data.Pitch = (float)raw.Angle_Y / 32768.0f * 180.0f;
    yaw_now          = (float)raw.Angle_Z / 32768.0f * 180.0f;

    /* Yaw 归正到 0~360（老队员 +180 的做法，让 0 在正前方） */
    yaw_now += 180.0f;
    if (yaw_now >= 360.0f)
        yaw_now -= 360.0f;
    if (yaw_now < 0.0f)
        yaw_now += 360.0f;
    WT931_Data.Yaw = yaw_now;

    /* ---- 跨圈判断：Yaw 从 330+ 跳到 30- 说明逆时针跨了一圈 ----
       输出连续多圈航向角 AllYaw，供底盘自转闭环用。 */
    if (WT931_Data.FirstFrame == 0)
    {
        WT931_Data.FirstFrame = 1;
        WT931_Data.YawCirnum  = 0;
    }
    else
    {
        if (WT931_Data.YawLast > 330.0f && yaw_now < 30.0f)
            WT931_Data.YawCirnum++;          /* 正向跨圈 */
        else if (WT931_Data.YawLast < 30.0f && yaw_now > 330.0f)
            WT931_Data.YawCirnum--;          /* 反向跨圈 */
    }

    WT931_Data.YawLast = yaw_now;
    WT931_Data.AllYaw  = (float)WT931_Data.YawCirnum * 360.0f + yaw_now;
}

/**
 * @brief  读连续航向角
 */
float WT931_GetAllYaw(void)
{
    return WT931_Data.AllYaw;
}
