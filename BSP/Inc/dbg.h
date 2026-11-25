#ifndef __DBG_H__
#define __DBG_H__

/**
 * ============================================================================
 *  dbg.h  ——  调试串口输出（走 UART10，115200）
 *
 *  为什么不用 printf：
 *    1. printf 需要 Keil 勾 Use MicroLIB 并重定向 fputc，多一步配置
 *    2. 更重要的：printf 是阻塞的。115200 下发 50 个字符要 4.3ms，
 *       如果你在 1ms 的控制任务里打印，控制周期直接崩掉。
 *
 *  本文件的规矩：Dbg_Print 只能在低优先级任务里调用（Task_Print），
 *  绝不允许出现在中断回调或 1kHz 控制任务里。
 * ============================================================================
 */

#include "main.h"

/* 自己声明，不依赖 usart.h 是否被生成 */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart10;

/**
 * @brief  格式化打印，内部用 HAL_UART_Transmit 阻塞发送
 * @note   支持 %d %u %ld %lu %s %x，浮点请自己转成整数再打
 */
void Dbg_Print(const char *fmt, ...);

/**
 * @brief  按十六进制打印一段字节，用于核对原始帧
 * @param  tag  前缀标签
 * @param  p    数据指针
 * @param  n    长度
 */
void Dbg_Hex(const char *tag, const uint8_t *p, uint8_t n);

/**
 * @brief  只发一个字符，不做任何格式化
 * @note   心跳专用。vsnprintf 要占两三百字节栈，一旦哪里出问题就发不出东西；
 *         这个函数栈占用几乎为零，是"程序到底还活着吗"最可靠的探针。
 *
 *         约定（按启动顺序，一个字母 = 一个检查点）：
 *           K  外设初始化完成（main.c，调度器启动前）
 *           A  App_Init 完成，三个任务已创建（调度器启动前）
 *           T  Task_Motor 第一次被调度到  ← 调度器真的切过去了的最早证据
 *           P  Task_Print 第一次被调度到
 *           M  Task_Motor 的周期心跳，每 500 个 1ms 周期（0.5 秒）一个
 *           B  检测到 bus-off，正在尝试自愈（每次退避 500ms）
 *           S  defaultTask 第一次被调度到
 *
 *         ★ 注意 S 的位置：defaultTask 的优先级是 osPriorityIdle（1），
 *           是全部任务里最低的，而 Task_Motor 是 3。FreeRTOS 启动调度器时
 *           选的是【最高优先级就绪任务】，所以第一个跑的是 Task_Motor，
 *           不是 defaultTask。
 *           因此真实顺序是  K A T ... S  —— S 一定在最后，不代表"调度器
 *           刚起来"。判断调度器有没有切过去，请看 T，不要看 S。
 *
 *         正常上电应看到： K A T P（S 稍后）然后每 500ms 一行
 *                         "收,发,发错,lec" 和 M
 *         缺哪个字母，就卡在那一环，比看任何日志都快。
 */
void Dbg_Ch(char c);

/**
 * @brief  断言失败处理（由 FreeRTOSConfig.h 的 configASSERT 调用）
 * @param  file  触发断言的源文件（用 __FILE__ 传入）
 * @param  line  触发的行号（用 __LINE__ 传入）
 * @note   打印 "!!! ASSERT 文件名:行号" 后关中断停机。
 *         刻意不用 printf：栈占用不到 64 字节，中断关掉也能发，
 *         这样断言现场才留得下来。
 */
void Dbg_Assert_Fail(const char *file, int line);

#endif /* __DBG_H__ */
