#ifndef __REMOTE_CONTROL_H__
#define __REMOTE_CONTROL_H__

/**
 * ============================================================================
 *  remote_control.h  ——  DJI DR16 遥控器（接收机）DBUS 协议解析
 *
 *  参考：老队员 Engineer_CusCtrlr 的 RemoteControl_Comm.c + 接收机手册。
 *
 *  DBUS 帧（18 字节，100 kbps，8-N-1 经 DBUS 反相，接收机已还原成标准串口电平）：
 *    字节 0~5   ：通道 ch0~ch3（各 11bit，摇杆）
 *    字节 5 高4位：左右拨杆 sw[0] sw[1]（各 2bit）
 *    字节 6~13  ：鼠标 / 键盘
 *    字节 16~17 ：通道 ch4（拨轮）
 *
 *  通道中值 1024，量程 364~1684，死区 10，归一化 /660。
 *  拨杆值：sw_up=1（上）  sw_down=2（下）  sw_mid=3（中）。
 *
 *  安全约定（双上断电保护）：
 *    左拨杆(sw[0]) == sw_up 且 右拨杆(sw[1]) == sw_up  → 底盘断动力。
 *
 *  失联检测：超过 RC_OFFLINE_MS 没收到完整帧，判定遥控离线，底盘断动力。
 * ============================================================================
 */

#include "main.h"

/* ---- DBUS 帧参数 ---------------------------------------------------------- */
#define RC_FRAME_LEN           18u      /* DBUS 一帧长度 */
#define RC_CH_VALUE_OFFSET     1024u    /* 摇杆中值 */
#define RC_CH_DEADBAND         10u      /* 摇杆死区 */
#define RC_CH_RANGE            660.0f   /* 摇杆半量程（归一化分母） */

/* ---- 失联判据：超过这个毫秒数没收到帧即离线 ------------------------------ */
#define RC_OFFLINE_MS          100u     /* DR16 周期 14ms，100ms 约 7 帧 */

/* ---- 拨杆状态（与老队员 switch_state 对齐）------------------------------ */
typedef enum
{
    sw_offset,      /* 未赋值（上电默认，对齐老工程 sw_offset） */
    sw_up   = 1,    /* 上 */
    sw_down = 2,    /* 下 */
    sw_mid  = 3,    /* 中 */
    sw_counter
} switch_state;

/* ---- 遥控器解析结果（供上层读取）----------------------------------------- */
typedef struct
{
    uint16_t ch[5];             /* 通道原始值 0~1684（中 1024） */
    switch_state sw[2];         /* 左右拨杆 */

    volatile uint8_t online;    /* 1=在线，0=失联（由回调/任务刷新） */
    uint32_t frame_count;       /* 累计收帧数 */
} remote_control_t;

/* 全局实例（remote_control.c 定义） */
extern remote_control_t g_rc;

/**
 * @brief  遥控接收初始化（App_Init 里调用，调度器启动前）
 * @note   使能 UART5 的 DMA 接收 + IDLE 中断，接收 DBUS 帧。
 */
void RemoteControl_Init(void);

/**
 * @brief  遥控中断回调（UART5_IRQHandler / IDLE 中断里调用）
 * @param  huart  UART5 句柄
 */
void RemoteControl_IRQHandler(UART_HandleTypeDef *huart);

/**
 * @brief  遥控是否在线（任务侧轮询调用）
 * @retval 1 在线，0 失联
 * @note   内部依据 g_rc.online + 最后收帧时间戳做二次判断。
 */
uint8_t RemoteControl_IsOnline(void);

/**
 * @brief  读取摇杆通道归一化值
 * @param  channel  0~4（0 右横 / 1 右竖 / 2 左横 / 3 左竖 / 4 拨轮）
 * @retval -1.0 ~ +1.0（右上为正），死区内返回 0
 */
float RemoteControl_GetChannel(uint8_t channel);

/**
 * @brief  读取拨杆状态
 * @param  sw  0 左拨杆，1 右拨杆
 * @retval switch_state
 */
switch_state RemoteControl_GetSwitch(uint8_t sw);

/**
 * @brief  「双上」保护状态：左右拨杆同时拨到上位置
 * @retval 1 双上（应断动力），0 非双上
 */
uint8_t RemoteControl_IsDoubleUp(void);

#endif /* __REMOTE_CONTROL_H__ */
