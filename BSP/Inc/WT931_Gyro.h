#ifndef __WT931_GYRO_H__
#define __WT931_GYRO_H__

/**
 * ============================================================================
 *  WT931_Gyro.h  ——  维特智能 WT931 九轴陀螺仪驱动（麦轮自转闭环用）
 *
 *  对齐点：字段/流程参考老队员 Engineer_CusCtrlr 的 WT931_Gyro，但做了三处
 *  修正（见 WT931_Gyro.c 顶部注释）：
 *    1. 不再依赖 Encoder_t 存陀螺仪数据（老队员拿 Gimbal_Gyro 存，耦合绕）
 *    2. 连续多圈 Yaw 解算独立、干净，直接输出可用于底盘自转闭环的连续航向角
 *    3. 串口/波特率做成可配置宏，接线变了只改这里
 *
 *  WT931 协议要点（维特智能私有协议，帧头 0x55）：
 *    帧格式：0x55 + 0x53 + 数据区(11 字节，含 Roll/Pitch/Yaw 各 2 字节小端)
 *    角度换算：原始值(short, -32768~32767) → 角度 = raw / 32768 * 180（°）
 *
 *  本驱动只关心「角度帧 0x53」，解出 Roll / Pitch / Yaw，并做 Yaw 多圈累积，
 *  输出 AllYaw（连续航向角，不归零）—— 这是麦轮自转 w 轴闭环的反馈源。
 * ============================================================================
 */

#include "main.h"
#include "usart.h"

/* ---- 配置区：WT931 接哪个串口 / 波特率 ★接线后改这里★ ------------------- */
#define WT931_UART           (&huart7)        /* 默认 UART7（PE7=RX PE8=TX） */
#define WT931_BAUDRATE       115200u          /* WT931 默认 115200 */

/* ---- 数据帧长度（维特协议固定 11 字节）----------------------------------- */
#define WT931_RXBUFFER_LEN   11u

/* ---- 角度结构体 ---- */
typedef struct
{
    short Angle_X;      /* Roll 原始值 */
    short Angle_Y;      /* Pitch 原始值 */
    short Angle_Z;      /* Yaw 原始值 */
} WT931_AngleRaw_t;

/* ---- WT931 数据对象 ---- */
typedef struct
{
    uint32_t FrameCounter;          /* 收到角度帧数（在线判据） */

    uint8_t  RxBuffer[WT931_RXBUFFER_LEN];  /* 接收缓冲 */

    /* 三轴角度（°，解算后） */
    float Roll;                     /* 横滚 */
    float Pitch;                    /* 俯仰 */
    float Yaw;                      /* 偏航 0~360（已 +180 归正） */

    /* 连续多圈航向角（麦轮自转闭环反馈源） */
    int32_t YawCirnum;              /* Yaw 跨过 360 的圈数 */
    float   AllYaw;                 /* 连续航向角 = YawCirnum*360 + Yaw */
    float   YawLast;                /* 上一帧 Yaw（跨圈判断用） */

    uint8_t  FirstFrame;            /* 是否收到第一帧 */
} WT931_Data_t;

extern WT931_Data_t WT931_Data;

/**
 * @brief  初始化 WT931（使能 IDLE 中断 + 启动 DMA 接收）
 * @note   需在 UART + DMA 初始化完成后调用（App_Init 里）。
 */
void WT931_Init(void);

/**
 * @brief  UART 中断回调（在 HAL_UART_RxCpltCallback / IDLE 中断里调用）
 * @param  huart  触发中断的串口句柄
 * @note   仅当 huart == WT931_UART 时才处理，其他串口忽略。
 */
void WT931_CallBack(UART_HandleTypeDef *huart);

/**
 * @brief  解析一帧角度数据（内部调用，也可供轮询模式手动触发）
 */
void WT931_Process(void);

/**
 * @brief  读连续航向角（麦轮自转闭环直接用它）
 * @retval 连续 Yaw 角（°，不归零）
 */
float WT931_GetAllYaw(void);

#endif /* __WT931_GYRO_H__ */
