#ifndef __MOTION_MOTOR_H__
#define __MOTION_MOTOR_H__

/**
 * ============================================================================
 *  motion_motor.h  ——  电机实例 + 双环控制编排（应用层）
 *
 *  这就是你「调 3508 PID」的主战场。
 *
 *  层次（自上而下）：
 *     MotionMotor_Control()   位置环：目标角 → 目标转速
 *                             速度环：目标转速 → 电流
 *     MotorDJI_SetOutput()    把电流发给电调（协议层）
 *
 *  后续加电机：在「实例区」再 extern 一个 motor_t + 两个 pid_type，
 *  注册 + 控制各加一行即可。协议层已支持多实例，无需再动。
 * ============================================================================
 */

#include "main.h"
#include "Motor.h"
#include "PID.h"

/* ---- 3508 位置环目标（调试期写死，后续接遥控/上位机）----------------- */
/* ★ 调参期先用小角度（30°），避免大阶跃 + 积分饱和导致飞转。
   等位置环稳定后再逐步加大，最后再试 180°。 */
#define MOTION_TEST_TARGET_ANGLE   30.0f   /* 目标角度 ° */

/* ---- 速度环输出限幅（C620 电流 ±16384，这里留余量到 8000）----------- */
#define MOTION_SPEED_OUT_MAX       8000.0f
#define MOTION_SPEED_I_MAX         4000.0f

/* ---- 位置环输出（目标转速）限幅：单位 rps（输出轴）--------------------- */
#define MOTION_POS_OUT_MAX         4.0f     /* ±4 rps，温和起步 */
#define MOTION_POS_I_MAX           2.0f

/* ---- 电机实例（extern，定义在 motion_motor.c）-------------------------- */
extern motor_t   Motor_3508_1;

extern pid_type  Motor_3508_1_PosPID;
extern pid_type  Motor_3508_1_SpdPID;

/** 函数声明 */
void  MotionMotor_Init(void);
void  MotionMotor_Control(void);
void  MotionMotor_SetTarget(motor_t *m, float target_angle_deg);

#endif /* __MOTION_MOTOR_H__ */
