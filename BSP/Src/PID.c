

#include "PID.h"

/* 内部限幅宏（clamp） */
static float clampf(float v, float lo, float hi)
{
    if (v > hi) return hi;
    if (v < lo) return lo;
    return v;
}

/**
 * @brief  PID 初始化
 * @param  pid        结构体指针
 * @param  mode       PID_POSITION / PID_DELTA
 * @param  Pidpara    [0]=Kp [1]=Ki [2]=Kd
 * @param  MaxOut     输出限幅（>0）
 * @param  MaxIout    积分限幅（>0，位置式有效）
 * @param  DeadZone   死区（0=不启用）
 * @retval 1 成功，0 失败
 */
uint8_t PID_Parameter_Init(pid_type *pid, PID_MODE mode, const float Pidpara[],
                           float MaxOut, float MaxIout, uint16_t DeadZone)
{
    if (pid == NULL)
        return 0;

    pid->pidmode = mode;
    pid->kp = Pidpara[0];
    pid->ki = Pidpara[1];
    pid->kd = Pidpara[2];

    pid->MaxOut  = MaxOut;
    pid->MaxIout = MaxIout;
    pid->DeadZone = DeadZone;

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0]  = pid->Dbuf[1]  = pid->Dbuf[2]  = 0.0f;
    pid->sum_error = 0.0f;
    pid->last_out  = 0.0f;
    pid->Pout = pid->Iout = pid->Dout = pid->Out = 0.0f;
    pid->dst  = 0.0f;
    pid->real = 0.0f;

    return 1;
}

/**
 * @brief  运行时改 PID 参数（分段调参用）
 * @retval 1 成功，0 失败
 */
uint8_t PID_Parameter_Update(pid_type *pid, const float Pidpara[])
{
    if (pid == NULL)
        return 0;

    pid->kp = Pidpara[0];
    pid->ki = Pidpara[1];
    pid->kd = Pidpara[2];
    return 1;
}

/**
 * @brief  PID 计算
 * @param  pid   结构体指针
 * @param  dst   目标值
 * @param  real  反馈值
 * @retval 输出值（已限幅）
 */
float Pid_cal(pid_type *pid, float dst, float real)
{
    if (pid == NULL)
        return 0.0f;

    /* 滚动误差历史 */
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->dst  = dst;
    pid->real = real;
    pid->error[0] = dst - real;

    /* 死区：误差小到阈值内，直接把误差当 0（微分仍可算，但简单起见整体静默） */
    float e = pid->error[0];
    if (pid->DeadZone != 0u)
    {
        if (e > -((float)pid->DeadZone) && e < ((float)pid->DeadZone))
            e = 0.0f;
    }

    if (pid->pidmode == PID_POSITION)
    {
        /* 比例 */
        pid->Pout = pid->kp * e;

        /* 积分 + 抗饱和 */
        pid->sum_error += e;
        pid->Iout = pid->ki * pid->sum_error;

        if (pid->MaxIout > 0.0f)
        {
            /* Iout 超上限 → 反向钳 sum_error，防止积分无限累积 */
            if (pid->Iout >  pid->MaxIout)
            {
                pid->Iout = pid->MaxIout;
                pid->sum_error = pid->Iout / (pid->ki != 0.0f ? pid->ki : 1.0f);
            }
            else if (pid->Iout < -pid->MaxIout)
            {
                pid->Iout = -pid->MaxIout;
                pid->sum_error = pid->Iout / (pid->ki != 0.0f ? pid->ki : 1.0f);
            }
        }

        /* 微分：误差差分 */
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = pid->error[0] - pid->error[1];
        pid->Dout = pid->kd * pid->Dbuf[0];

        pid->Out = pid->Pout + pid->Iout + pid->Dout;
    }
    else /* PID_DELTA 增量式 */
    {
        pid->Pout = pid->kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->ki * e;

        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = pid->error[0] - 2.0f * pid->error[1] + pid->error[2];
        pid->Dout = pid->kd * pid->Dbuf[0];

        /* 返回的是「增量」，调用方自行累加；这里直接限幅并返回 */
        pid->Out = pid->Pout + pid->Iout + pid->Dout;
    }

    /* 统一输出限幅 */
    if (pid->MaxOut > 0.0f)
        pid->Out = clampf(pid->Out, -pid->MaxOut, pid->MaxOut);

    return pid->Out;
}

/**
 * @brief  PID 清零（切换模式/目标跳变时调用，防暴冲）
 * @retval 1 成功，0 失败
 */
uint8_t PID_clear(pid_type *pid)
{
    if (pid == NULL)
        return 0;

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->sum_error = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->Out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->last_out = 0.0f;
    return 1;
}
