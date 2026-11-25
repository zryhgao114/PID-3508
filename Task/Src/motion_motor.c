/**
 * ============================================================================
 *  motion_motor.c  ——  电机实例 + 双环控制实现
 *
 *  ★★★ 调 PID 只需要动这一个文件 ★★★
 *
 *  参数都在下方「PID 参数表」里，改数字重烧即可。
 *  调参顺序（空载、1kHz 控制周期）：
 *    1. 位置环 kp 从小往大（如 5 → 10 → 20），找到不震荡的最大值
 *    2. 速度环 kp 从小往大（如 30 → 60 → 120），电机响应变快但别超调震荡
 *    3. 速度环 ki 加一点消灭静差（0.1 起步）
 *    4. 微分先留 0，速度环加微分容易放大噪声
 *
 *  安全底线：
 *    · 上电静默、掉线发零、输出限幅（三层保护在 task_can.c 里，别删）
 *    · 空载调好再上机构
 * ============================================================================
 */

#include "motion_motor.h"
#include "motor_dji.h"
#include "dbg.h"

/* ===========================================================================
 *  一、电机实例与 PID 实例
 * =========================================================================*/

/* 3508 电机实例：CAN1、拨码 ID 1 */
motor_t Motor_3508_1;

pid_type Motor_3508_1_PosPID;   /* 位置环 */
pid_type Motor_3508_1_SpdPID;   /* 速度环 */

/* ===========================================================================
 *  二、PID 参数表（改这里）
 *
 *  3508 + C620 空载起步建议（从「稳」开始，再逐步加）：
 *    位置环 { kp, ki, kd } = { 1.5, 0.0, 0.0 }  输出限幅 ±3 rps
 *    速度环 { kp, ki, kd } = { 30.0, 0.0, 0.0 }  输出限幅 ±8000（电流）
 *
 *  ★ 调参顺序（内环→外环，每步只看一个量）：
 *    1. 速度环 ki 先给 0（纯 P），避免积分饱和飞车
 *    2. 速度环 kp 从小往大（30→60→100），电机响应变快，但别震荡
 *    3. 位置环 kp 从小往大（1.5→3→6），转到目标角不超调太多
 *    4. 两个环都稳了，再回头加速度环 ki（0.05 起步）消静差
 * =========================================================================*/
static const float PosPID_param[3] = { 1.0f, 0.0f, 0.0f };
static const float SpdPID_param[3] = { 120.0f, 0.0f, 0.0f };

/* ===========================================================================
 *  三、初始化：注册电机 + 初始化双环 PID
 * =========================================================================*/
void MotionMotor_Init(void)
{
    /* 注册 3508 电机（老队员 6 参签名 + MotorSetBus 指定总线）：
       type / 减速比 / Can_id；总线单独用 MotorSetBus 设 */
    MotortypeRegister(&Motor_3508_1, MOTOR_3508,
                      &Motor_3508_1_PosPID, &Motor_3508_1_SpdPID,
                      MOTOR3508_Reductionratio, 1);
    MotorSetBus(&Motor_3508_1, CAN_BUS_1);

    /* 挂进协议层注册表（算好反馈/控制 ID，开多圈解算） */
    MotorDJI_Register(&Motor_3508_1);

    /* 位置环：目标角 → 目标转速，输出限幅 ±8 rps */
    PID_Parameter_Init(&Motor_3508_1_PosPID, PID_POSITION, PosPID_param,
                       MOTION_POS_OUT_MAX, MOTION_POS_I_MAX, 0);

    /* 速度环：目标转速 → 电流，输出限幅 ±12000 */
    PID_Parameter_Init(&Motor_3508_1_SpdPID, PID_POSITION, SpdPID_param,
                       MOTION_SPEED_OUT_MAX, MOTION_SPEED_I_MAX, 0);

    /* 上电把当前位置当作零位，再设测试目标角，让位置环产生误差驱动电机。
       MOTION_TEST_TARGET_ANGLE 定义在 motion_motor.h（默认 180°）。
       调试期用固定目标角；后续接遥控/上位机时换成 MotionMotor_SetTarget()。 */
    MotorDJI_SetZero(&Motor_3508_1);
    Motor_3508_1.InitAngle    = 0.0f;
    Motor_3508_1.TargetAngle  = MOTION_TEST_TARGET_ANGLE;
    Motor_3508_1.Targetcirnum = 0;
}

/* ===========================================================================
 *  四、设定目标角度（供调试/上位机调用）
 * =========================================================================*/
void MotionMotor_SetTarget(motor_t *m, float target_angle_deg)
{
    if (m == NULL)
        return;

    m->TargetAngle = target_angle_deg;

    /* 目标跳变较大时清积分，避免暴冲 */
    if (m->PositionPID != NULL)
        m->PositionPID->sum_error = 0.0f;
    if (m->SpeedPID != NULL)
        m->SpeedPID->sum_error = 0.0f;
}

/* ===========================================================================
 *  五、双环控制（1ms 调用一次，在 Task_Motor 里）
 *
 *   位置环：Pid_cal(位置PID, 目标角, 实际连续角)  → 目标转速 rps
 *   速度环：Pid_cal(速度PID, 目标转速, 实际转速)  → 电流
 * =========================================================================*/
void MotionMotor_Control(void)
{
    motor_t *m = &Motor_3508_1;
    float target_speed;
    float out;

    if (m == NULL || m->PositionPID == NULL || m->SpeedPID == NULL)
        return;

    /* 位置环：目标角 → 目标转速（单位 rps） */
    target_speed = Pid_cal(m->PositionPID, m->TargetAngle, m->out_angle);

    /* 速度环：目标转速 → 电流（单位：电流原始值 ±16384） */
    out = Pid_cal(m->SpeedPID, target_speed, m->Realrotationrate);

    /* 输出（int16），发给协议层 */
    m->Out = (int16_t)out;

    /* 调试打印：tgt=目标角 out_ang=反馈角 tgt_spd=目标转速 real_spd=反馈转速 out=电流
       每 500 次打印一次，避免刷屏 */
    {
        static uint32_t dbg_cnt = 0;
        if ((++dbg_cnt % 500u) == 0u)
        {
            Dbg_Print("[DBG] tgt=%.1f ang=%.1f tspd=%.2f rspd=%.2f rawrpm=%d out=%d\n",
                      m->TargetAngle, m->out_angle, target_speed,
                      m->Realrotationrate, m->FeedbackData.RealSpeed, m->Out);
        }
    }
}
