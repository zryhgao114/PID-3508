#ifndef __PID_H__
#define __PID_H__

/**
 * ============================================================================
 *  PID.h  ——  位置式 / 增量式 PID（双环共用）
 *
 *  结构体命名沿用老队员框架（pid_type / PID_POSITION / PID_DELTA /
 *  PID_Parameter_Init / Pid_cal），这样你读 Engineer_CusCtrlr 时能对上号；
 *  但内部实现做了三处修正（见 PID.c 顶部注释）：
 *    1. 增量式(PID_DELTA)不再用「输出累加」，而是每次由误差的差分直接算增量，
 *       避免 MaxOut 限幅后 Out 与真实输出的漂移。
 *    2. 位置式(PID_POSITION)补上积分抗饱和：Iout 超限时反向钳住 sum_error，
 *       防止电机卡死时积分无限累积、一松开就暴冲。
 *    3. 输出限幅统一放在最后，两处（积分、总输出）都限。
 *
 *  只用位置式就够了：位置环、速度环都走 PID_POSITION。
 *  PID_DELTA 留作参考，等你做「力矩增量式」这类场景时再用。
 * ============================================================================
 */

#include "main.h"

/** 工作模式 */
typedef enum
{
    PID_POSITION = 0,   /* 位置式：Out = Kp*e + Ki*Σe + Kd*(e-e_last) */
    PID_DELTA           /* 增量式：ΔOut = Kp*(e-e1) + Ki*e + Kd*(e-2e1+e2) */
} PID_MODE;

/** PID 结构体 */
typedef struct
{
    PID_MODE pidmode;

    /* 误差历史：error[0]=本次 error[1]=上次 error[2]=上上次 */
    float error[3];

    float sum_error;    /* 积分累加（位置式用） */
    float Dbuf[3];      /* 微分缓冲：0=本次 1=上次 2=上上次 */
    float last_out;     /* 上次输出（增量式用） */

    float kp, ki, kd;   /* PID 系数 */

    float Pout, Iout, Dout;   /* 比例/积分/微分分量（调试可读） */
    float Out;                /* 最终输出 */

    float dst;          /* 目标值（记录用） */
    float real;         /* 反馈值（记录用） */

    float MaxOut;       /* 输出限幅（>0） */
    float MaxIout;      /* 积分输出限幅（>0） */

    uint16_t DeadZone;  /* 死区（误差绝对值小于它则积分/输出不更新，0=不启用） */
} pid_type;

/** 函数声明 */
uint8_t PID_Parameter_Init(pid_type *pid, PID_MODE mode, const float Pidpara[],
                           float MaxOut, float MaxIout, uint16_t DeadZone);
uint8_t PID_Parameter_Update(pid_type *pid, const float Pidpara[]);
uint8_t PID_clear(pid_type *pid);
float   Pid_cal(pid_type *pid, float dst, float real);

#endif /* __PID_H__ */
