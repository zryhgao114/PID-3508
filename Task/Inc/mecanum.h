#ifndef __MECANUM_H__
#define __MECANUM_H__

/**
 * ============================================================================
 *  mecanum.h  ——  四轮麦轮底盘（Mecanum）解算 + 控制
 *
 *  这是「调麦轮」的主战场。基于对齐老队员后的 motor_t 骨架实现。
 *
 *  麦轮逆运动学（标准公式）：
 *    给定底盘中心三轴速度 (Vx, Vy, Wz)，四个轮子的转速为：
 *      V_fl = Vx - Vy - Wz * (Lx + Ly)
 *      V_fr = Vx + Vy + Wz * (Lx + Ly)
 *      V_rl = Vx + Vy - Wz * (Lx + Ly)
 *      V_rr = Vx - Vy + Wz * (Lx + Ly)
 *    其中 Lx 半轴距（前后轮距/2）、Ly 半轮距（左右轮距/2），单位一致。
 *
 *  约定（与老队员 Chassis.Speed_X/Y/W 语义对齐）：
 *    Vx = 前向速度（前进为正）
 *    Vy = 横向速度（右移为正）
 *    Wz = 自转角速度（逆时针为正）
 *
 *  本 demo 先用「转速开环」让底盘能平移/自转，验证四轮方向和安装正确，
 *  再逐步上速度环、位置环。这样能最快发现「哪个轮子装反了」。
 * ============================================================================
 */

#include "main.h"
#include "Motor.h"
#include "PID.h"

/* ---- 麦轮几何参数（单位 m，按实车量，这里给占位值）---------------------- */
#define MECANUM_HALF_WHEELBASE   0.12f   /* 前后半轴距 Lx */
#define MECANUM_HALF_TRACK       0.10f   /* 左右半轮距 Ly */
#define MECANUM_WHEEL_RADIUS     0.076f  /* 轮子半径（3508+麦轮约 76mm） */

/* ---- ★★★ 三电机模式开关 ★★★
 *  ID=4（后右轮 Mecanum_RR）硬件异常，本次任务只控制 ID=1/2/3 三台。
 *  =1 时：不注册 ID=4、后右轮转速强制 0、掉线保护只查三台。
 *  等 ID=4 电机修好后改回 0，即恢复四轮完整麦轮。 */
#define MECANUM_THREE_WHEEL_MODE  1

/* ---- ★★★ 两电机模式开关（临时调试用）★★★
 *  调试阶段 FR（前右，ID=2）也离线，先只控 FL（ID=1）+ RL（ID=3）两台验证。
 *  =1 时：FR 不参与控制、不纳入掉线保护，只控 FL/RL 两台。
 *  优先级高于 THREE_WHEEL_MODE，FR 修好后改回 0。 */
#define MECANUM_TWO_WHEEL_MODE    1

/* ---- 底盘速度上限（m/s 与 rad/s）----------------------------------------- */
#define MECANUM_MAX_LINEAR       1.0f
#define MECANUM_MAX_ANGULAR      3.0f

/* ---- 轮子转速上限（rps，输出轴）----------------------------------------- */
#define MECANUM_WHEEL_MAX_RPS    4.0f

/* ---- 四个轮子的电机实例（定义在 mecanum.c）------------------------------ */
extern motor_t Mecanum_FL;    /* 前左 */
extern motor_t Mecanum_FR;    /* 前右 */
extern motor_t Mecanum_RL;    /* 后左 */
extern motor_t Mecanum_RR;    /* 后右 */

extern pid_type Mecanum_FL_SpdPID, Mecanum_FR_SpdPID;
extern pid_type Mecanum_RL_SpdPID, Mecanum_RR_SpdPID;

/** 函数声明 */
void Mecanum_Init(void);
void Mecanum_SetVelocity(float vx, float vy, float wz);
void Mecanum_Control(void);
void Mecanum_Stop(void);

/**
 * @brief  自转航向保持（用 WT931 航向角做反馈，P 控制 w 轴）
 * @param  target_yaw_deg  目标航向角（°，连续值）
 * @note   调用前需已 WT931_Init；本函数把底盘锁定在目标航向，
 *         同时 vx/vy 仍可用 Mecanum_SetVelocity 单独设。
 */
void Mecanum_HoldYaw(float target_yaw_deg);

#endif /* __MECANUM_H__ */
