/**
 * ============================================================================
 *  dbg.c  ——  调试串口输出实现
 * ============================================================================
 */

#include "dbg.h"
#include <stdio.h>
#include <stdarg.h>

#define DBG_BUF_SIZE   160

/* ---- 调试串口选择 --------------------------------------------------------
 * 达妙 DM-MC-Board02 的调试口是 UART10：PE3 = TX / PE2 = RX。
 * 波特率由 main.c 的 USER CODE 2 在启动时设为 115200（兼容性优先），
 * 串口助手也要设 115200 8N1，否则看到的是空白或乱码。
 *
 * 想切回板载 USART1（PA9 = TX / PA10 = RX，115200）就把下面改成 huart1，
 * 同时把串口助手改成 115200。
 * ------------------------------------------------------------------------ */
#define DBG_UART   huart10

/**
 * @brief  裸寄存器发送 —— 替代 HAL_UART_Transmit
 *
 * 为什么不用 HAL 的函数：
 *   HAL_UART_Transmit 先看 huart->gState，再用 HAL_GetTick() 判超时。
 *   一旦状态机没回到 READY、或者 tick 停住，它就会卡在中间不出来。
 *
 * ★ 为什么等 TC 而不是 TXE
 *   TXE 只代表"还能再塞一个字节"，不代表上一个已经发到线上。
 *   连续猛写时，超出缓冲深度（FIFO 8 + 移位器 1 + TDR 1 ≈ 10）的部分
 *   会把还没发出去的字节覆盖掉 —— 现象就是"一串字符少了中间一段"。
 *   TC 代表整个字节连停止位都移完了，等它最稳，代价是每字节 87us。
 */
static void Dbg_RawSend(const char *p, uint16_t n)
{
    uint32_t stuck = 0u;

    DBG_UART.Instance->ICR = USART_ICR_ORECF | USART_ICR_FECF
                           | USART_ICR_NECF  | USART_ICR_PECF;

    while (n-- > 0u)
    {
        uint32_t guard = 2000000u;

        while (((DBG_UART.Instance->ISR & USART_ISR_TC) == 0u) && (guard != 0u))
            guard--;

        if (guard == 0u)
        {
            if (++stuck > 50u)
                return;                   /* 救不回来，放弃，别拖死调用者 */

            /* 抢救：清错误标志 + TE 重新使能，重试本字节 */
            DBG_UART.Instance->ICR = USART_ICR_ORECF | USART_ICR_FECF
                                   | USART_ICR_NECF  | USART_ICR_PECF;
            DBG_UART.Instance->CR1 &= ~USART_CR1_TE;
            __NOP(); __NOP(); __NOP(); __NOP();
            DBG_UART.Instance->CR1 |= USART_CR1_TE;
            continue;
        }

        DBG_UART.Instance->TDR = (uint8_t)(*p++);
        stuck = 0u;
    }

    /* 等最后一个字节也彻底移出 */
    {
        uint32_t guard = 2000000u;
        while (((DBG_UART.Instance->ISR & USART_ISR_TC) == 0u) && (guard != 0u))
            guard--;
    }
}

void Dbg_Print(const char *fmt, ...)
{
    char    buf[DBG_BUF_SIZE];
    va_list ap;
    int     n;

    va_start(ap, fmt);
    n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (n <= 0)
        return;

    if (n > (int)sizeof(buf) - 1)
        n = (int)sizeof(buf) - 1;

    Dbg_RawSend(buf, (uint16_t)n);
}

void Dbg_Hex(const char *tag, const uint8_t *p, uint8_t n)
{
    char    buf[DBG_BUF_SIZE];
    int     pos = 0;
    uint8_t i;

    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, "%s", tag);

    for (i = 0; i < n && pos < (int)sizeof(buf) - 4; i++)
        pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, " %02X", p[i]);

    pos += snprintf(&buf[pos], sizeof(buf) - (size_t)pos, "\r\n");

    if (pos > 0)
        Dbg_RawSend(buf, (uint16_t)pos);
}

/* ----------------------------------------------------------------------------
 *  心跳字符 —— 整个工程里最"抗造"的一条输出通路
 *
 *  不用 vsnprintf、不占栈、不依赖 FreeRTOS，就是往 TDR 塞一个字节然后等 TC。
 *  一个字节在 115200 下 87us 就出去了，就算 USB-TTL 很烂也塞得过去。
 *
 *  用它来判断"程序还活着吗"，比看任何一条格式化日志都可靠：
 *     看到这个字符  → MCU 确实执行到了那一行
 *     看不到        → 要么没执行到，要么串口物理链路真的断了
 * -------------------------------------------------------------------------- */
void Dbg_Ch(char c)
{
    uint32_t guard = 2000000u;

    while (((DBG_UART.Instance->ISR & USART_ISR_TC) == 0u) && (guard != 0u))
        guard--;

    if (guard == 0u)
    {
        /* 抢救一次：清错误标志 + TE 重新使能 */
        DBG_UART.Instance->ICR = USART_ICR_ORECF | USART_ICR_FECF
                               | USART_ICR_NECF  | USART_ICR_PECF;
        DBG_UART.Instance->CR1 &= ~USART_CR1_TE;
        __NOP(); __NOP(); __NOP(); __NOP();
        DBG_UART.Instance->CR1 |= USART_CR1_TE;
        return;
    }

    DBG_UART.Instance->TDR = (uint8_t)c;
}

/* ----------------------------------------------------------------------------
 *  断言失败处理 —— 配合 FreeRTOSConfig.h 里的 configASSERT
 *
 *  FreeRTOS 的断言原来展开成 taskDISABLE_INTERRUPTS(); for(;;);
 *  也就是"关中断 + 原地死循环 + 一声不吭"。真出问题时完全没法定位。
 *
 *  这里改成先把「文件名:行号」打出来再停机。
 *  刻意不用 Dbg_Print / vsnprintf：断言有可能正是因为栈不够才触发，
 *  再用一个要吃 200~300 字节栈的函数，会在打印途中二次崩掉。
 *  全部走 Dbg_RawSend（裸寄存器轮询），栈占用不到 64 字节，
 *  而且中断关掉也照样能发。
 * -------------------------------------------------------------------------- */
void Dbg_Assert_Fail(const char *file, int line)
{
    char       buf[64];
    int        n = 0;
    unsigned   v;
    const char *p;
    const char *base;

    buf[n++] = '\r';
    buf[n++] = '\n';
    buf[n++] = '!';
    buf[n++] = '!';
    buf[n++] = '!';
    buf[n++] = ' ';

    /* 文件名只取最后一段：../Task/Src/task_can.c → task_can.c */
    base = file;
    if (file != NULL)
    {
        for (p = file; *p != '\0'; p++)
            if ((*p == '/') || (*p == '\\'))
                base = p + 1;

        for (p = base; (*p != '\0') && (n < (int)sizeof(buf) - 14); p++)
            buf[n++] = *p;
    }

    buf[n++] = ':';

    /* 行号：手写十进制，避开 sprintf */
    v = (line < 0) ? 0u : (unsigned)line;
    {
        char tmp[10];
        int  k = 0;

        if (v == 0u)
            tmp[k++] = '0';
        while ((v > 0u) && (k < (int)sizeof(tmp)))
        {
            tmp[k++] = (char)('0' + (v % 10u));
            v /= 10u;
        }
        while (k > 0)
            buf[n++] = tmp[--k];
    }

    buf[n++] = '\r';
    buf[n++] = '\n';

    Dbg_RawSend(buf, (uint16_t)n);

    /* 停机。和原来的 configASSERT 行为一致：关中断后原地不动。
       用调试器挂上去，PC 就停在这里，栈回溯能看到是谁触发的。 */
    __disable_irq();
    for (;;)
    {
    }
}

/* ----------------------------------------------------------------------------
 *  可选：如果你更习惯用 printf，把下面这段取消注释，并在 Keil 里
 *  Options for Target → Target → 勾上 Use MicroLIB。
 *  注意：MicroLIB 的 printf 不可重入，只在一个任务里用才安全。
 * -------------------------------------------------------------------------- */
#if 0
int fputc(int ch, FILE *f)
{
    uint8_t c = (uint8_t)ch;
    HAL_UART_Transmit(&DBG_UART, &c, 1, 10);
    return ch;
}
#endif
