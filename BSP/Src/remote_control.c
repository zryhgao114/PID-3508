/**
 * ============================================================================
 *  remote_control.c  ——  DJI DR16 DBUS 协议解析 + 失联检测
 *
 *  接收链路：接收机 → UART5(PC12 TX / PD2 RX) → DMA1_Stream2 → IDLE 中断。
 *  帧格式见 remote_control.h 头注释。
 *
 *  本工程只用摇杆 ch0~ch3 + 左右拨杆 sw[0]/sw[1]，鼠标/键盘/拨轮先不解析，
 *  但保留原始缓冲，后续要加直接补位即可。
 * ============================================================================
 */

#include "remote_control.h"

#include "dbg.h"

/* ---- UART5 / DMA 句柄（usart.c / dma.c 里 CubeMX 生成）-------------------- */
extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef hdma_uart5_rx;

/* ---- 全局实例 ------------------------------------------------------------ */
remote_control_t g_rc;

/* ---- DMA 接收缓冲：DBUS 帧 18 字节，收满一帧触发 IDLE -------------------- */
static volatile uint8_t s_rx_buf[RC_FRAME_LEN];

/* ---- 最后收帧时间戳（供失联判断）----------------------------------------- */
static volatile uint32_t s_last_frame_tick = 0u;

/* ===========================================================================
 *  初始化
 * =========================================================================*/
void RemoteControl_Init(void)
{
    g_rc.ch[0] = RC_CH_VALUE_OFFSET;
    g_rc.ch[1] = RC_CH_VALUE_OFFSET;
    g_rc.ch[2] = RC_CH_VALUE_OFFSET;
    g_rc.ch[3] = RC_CH_VALUE_OFFSET;
    g_rc.ch[4] = RC_CH_VALUE_OFFSET;
    g_rc.sw[0] = sw_offset;
    g_rc.sw[1] = sw_offset;
    g_rc.online = 0;
    g_rc.frame_count = 0;

    s_last_frame_tick = HAL_GetTick();

    /* 使能 UART5 接收 DMA + IDLE 中断 */
    __HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart5, (uint8_t *)s_rx_buf, RC_FRAME_LEN);
}

/* ===========================================================================
 *  中断回调：UART5 的 IDLE 中断里调用（放在 UART5_IRQHandler 的 USER CODE 段）
 * =========================================================================*/
void RemoteControl_IRQHandler(UART_HandleTypeDef *huart)
{
    if (huart->Instance != UART5)
        return;

    /* 仅处理 IDLE 中断（一帧收完） */
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET)
    {
        __HAL_UART_CLEAR_IDLEFLAG(huart);

        /* 停止当前 DMA 接收，算出实际收了多少字节 */
        HAL_UART_DMAStop(huart);

        uint32_t rx_len = RC_FRAME_LEN - __HAL_DMA_GET_COUNTER(huart->hdmarx);

        /* 只有完整 18 字节才算一帧有效帧，否则丢弃并重启接收 */
        if (rx_len == RC_FRAME_LEN)
        {
            /* ---- 解析：摇杆 ch0~ch3 + 拨杆 ---- */
            g_rc.ch[0] = (uint16_t)((s_rx_buf[0] | (s_rx_buf[1] << 8)) & 0x07FF);
            g_rc.ch[1] = (uint16_t)(((s_rx_buf[1] >> 3) | (s_rx_buf[2] << 5)) & 0x07FF);
            g_rc.ch[2] = (uint16_t)(((s_rx_buf[2] >> 6) | (s_rx_buf[3] << 2) | (s_rx_buf[4] << 10)) & 0x07FF);
            g_rc.ch[3] = (uint16_t)(((s_rx_buf[4] >> 1) | (s_rx_buf[5] << 7)) & 0x07FF);

            /* 拨杆：sw[0] 左 = (byte5 的 bit6~7)，sw[1] 右 = (byte5 的 bit4~5) */
            g_rc.sw[0] = (switch_state)(((s_rx_buf[5] >> 4) & 0x0C) >> 2);
            g_rc.sw[1] = (switch_state)((s_rx_buf[5] >> 4) & 0x03);

            g_rc.online = 1;
            g_rc.frame_count++;
            s_last_frame_tick = HAL_GetTick();
        }

        /* 重启 DMA 接收，等待下一帧 */
        HAL_UART_Receive_DMA(huart, (uint8_t *)s_rx_buf, RC_FRAME_LEN);
    }
}

/* ===========================================================================
 *  在线判断（任务侧轮询）
 * =========================================================================*/
uint8_t RemoteControl_IsOnline(void)
{
    uint32_t elapsed = HAL_GetTick() - s_last_frame_tick;

    if (g_rc.online && (elapsed < RC_OFFLINE_MS))
        return 1;

    /* 失联：清 online，并回中摇杆，确保即便立刻恢复也不会带旧指令冲出去 */
    g_rc.online = 0;
    g_rc.ch[0] = RC_CH_VALUE_OFFSET;
    g_rc.ch[1] = RC_CH_VALUE_OFFSET;
    g_rc.ch[2] = RC_CH_VALUE_OFFSET;
    g_rc.ch[3] = RC_CH_VALUE_OFFSET;
    g_rc.ch[4] = RC_CH_VALUE_OFFSET;
    return 0;
}

/* ===========================================================================
 *  读取通道（归一化 -1.0 ~ +1.0，右上正，死区内回 0）
 * =========================================================================*/
float RemoteControl_GetChannel(uint8_t channel)
{
    int32_t raw;
    int32_t diff;

    if (channel >= 5u)
        return 0.0f;

    raw  = (int32_t)g_rc.ch[channel];
    diff = raw - (int32_t)RC_CH_VALUE_OFFSET;

    if ((diff > -(int32_t)RC_CH_DEADBAND) && (diff < (int32_t)RC_CH_DEADBAND))
        return 0.0f;

    return (float)diff / RC_CH_RANGE;
}

/* ===========================================================================
 *  读取拨杆
 * =========================================================================*/
switch_state RemoteControl_GetSwitch(uint8_t sw)
{
    if (sw >= 2u)
        return sw_mid;
    return g_rc.sw[sw];
}

/* ===========================================================================
 *  双上保护：左右拨杆都拨到上位置
 * =========================================================================*/
uint8_t RemoteControl_IsDoubleUp(void)
{
    return (g_rc.sw[0] == sw_up && g_rc.sw[1] == sw_up) ? 1u : 0u;
}
