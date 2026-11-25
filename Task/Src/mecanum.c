/**
 * ============================================================================
 *  mecanum.c  ——  四轮麦轮底盘解算 + 控制（对齐老队员 motor_t 骨架）
 *
 *  结构（自下而上）：
 *    Mecanum_Init          注册 4 台 3508 + 初始化速度环 PID
 *    Mecanum_SetVelocity   输入 (vx, vy, wz) → 算四轮目标转速
 *    Mecanum_Control       四轮速度环 → 电流 → MotorDJI_SetOutput
 *    Mecanum_Stop          四轮发零
 *
 *  ★ 第一阶段（调麦轮）建议：
 *    先用「转速开环」——直接按目标转速发电流，先验证四轮方向和安装；
 *    方向对了再切速度环闭环（打开 MECANUM_USE_SPEED_LOOP）。
 * ============================================================================
 */

#include "mecanum.h"
#include "motor_dji.h"
#include "WT931_Gyro.h"
#include "dbg.h"

/* ---- 是否启用速度环闭环（0=开环直发电流，1=速度环闭环）------------------ */
#define MECANUM_USE_SPEED_LOOP   0

/* ===========================================================================
 *  一、电机实例 + PID
 * =========================================================================*/

motor_t Mecanum_FL, Mecanum_FR, Mecanum_RL, Mecanum_RR;

pid_type Mecanum_FL_SpdPID, Mecanum_FR_SpdPID;
pid_type Mecanum_RL_SpdPID, Mecanum_RR_SpdPID;

/* ---- 速度环 PID 参数（保守起步）---- */
static const float SpdPID_param[3] = { 120.0f, 0.0f, 0.0f };

/* ---- 四轮当前目标转速（rps）---- */
static float fl_tgt_rps, fr_tgt_rps, rl_tgt_rps, rr_tgt_rps;

/* ===========================================================================
 *  二、初始化
 * =========================================================================*/
void Mecanum_Init(void)
{
    /* 麦轮 3508，拨码 ID 1~3，都在 CAN1（ID=4 后右轮异常，见 MECANUM_THREE_WHEEL_MODE） */
    MotortypeRegister(&Mecanum_FL, MOTOR_3508,
                      NULL, &Mecanum_FL_SpdPID, MOTOR3508_Reductionratio, 1);
#if !MECANUM_TWO_WHEEL_MODE
    MotortypeRegister(&Mecanum_FR, MOTOR_3508,
                      NULL, &Mecanum_FR_SpdPID, MOTOR3508_Reductionratio, 2);
#endif
    MotortypeRegister(&Mecanum_RL, MOTOR_3508,
                      NULL, &Mecanum_RL_SpdPID, MOTOR3508_Reductionratio, 3);

    MotorSetBus(&Mecanum_FL, CAN_BUS_1);
#if !MECANUM_TWO_WHEEL_MODE
    MotorSetBus(&Mecanum_FR, CAN_BUS_1);
#endif
    MotorSetBus(&Mecanum_RL, CAN_BUS_1);

#if !MECANUM_THREE_WHEEL_MODE
    /* 四轮完整模式：才注册 ID=4 后右轮 */
    MotortypeRegister(&Mecanum_RR, MOTOR_3508,
                      NULL, &Mecanum_RR_SpdPID, MOTOR3508_Reductionratio, 4);
    MotorSetBus(&Mecanum_RR, CAN_BUS_1);
#endif

    /* 挂进协议层注册表 */
    MotorDJI_Register(&Mecanum_FL);
#if !MECANUM_TWO_WHEEL_MODE
    MotorDJI_Register(&Mecanum_FR);
#endif
    MotorDJI_Register(&Mecanum_RL);
#if !MECANUM_THREE_WHEEL_MODE
    MotorDJI_Register(&Mecanum_RR);
#endif

    /* 速度环 PID 初始化（输出限幅 ±8000 电流） */
    PID_Parameter_Init(&Mecanum_FL_SpdPID, PID_POSITION, SpdPID_param, 8000.0f, 4000.0f, 0);
#if !MECANUM_TWO_WHEEL_MODE
    PID_Parameter_Init(&Mecanum_FR_SpdPID, PID_POSITION, SpdPID_param, 8000.0f, 4000.0f, 0);
#endif
    PID_Parameter_Init(&Mecanum_RL_SpdPID, PID_POSITION, SpdPID_param, 8000.0f, 4000.0f, 0);
#if !MECANUM_THREE_WHEEL_MODE
    PID_Parameter_Init(&Mecanum_RR_SpdPID, PID_POSITION, SpdPID_param, 8000.0f, 4000.0f, 0);
#endif

    /* 零位 */
    MotorDJI_SetZero(&Mecanum_FL);
#if !MECANUM_TWO_WHEEL_MODE
    MotorDJI_SetZero(&Mecanum_FR);
#endif
    MotorDJI_SetZero(&Mecanum_RL);
#if !MECANUM_THREE_WHEEL_MODE
    MotorDJI_SetZero(&Mecanum_RR);
#endif

    Mecanum_Stop();
}

/* ===========================================================================
 *  三、逆运动学：底盘速度 → 四轮转速
 * =========================================================================*/
void Mecanum_SetVelocity(float vx, float vy, float wz)
{
    float L = MECANUM_HALF_WHEELBASE + MECANUM_HALF_TRACK;  /* Lx + Ly */

    /* 线速度 → 轮子线速度（m/s）→ 转速（rps）= v / (2*pi*r) */
    float k = 1.0f / (2.0f * 3.14159265f * MECANUM_WHEEL_RADIUS);

    fl_tgt_rps = (vx - vy - wz * L) * k;
    fr_tgt_rps = (vx + vy + wz * L) * k;
    rl_tgt_rps = (vx + vy - wz * L) * k;
    rr_tgt_rps = (vx - vy + wz * L) * k;

#if MECANUM_THREE_WHEEL_MODE
    /* 三电机模式：后右轮 ID=4 异常，强制 0，不参与控制 */
    rr_tgt_rps = 0.0f;
#endif
#if MECANUM_TWO_WHEEL_MODE
    /* 两电机模式：前右轮 FR 离线，强制 0，不参与控制 */
    fr_tgt_rps = 0.0f;
#endif

    /* 限幅 */
    {
        #define CLAMP_RPS(x) do { \
            if ((x) >  MECANUM_WHEEL_MAX_RPS) (x) =  MECANUM_WHEEL_MAX_RPS; \
            if ((x) < -MECANUM_WHEEL_MAX_RPS) (x) = -MECANUM_WHEEL_MAX_RPS; \
        } while (0)
        CLAMP_RPS(fl_tgt_rps);
        CLAMP_RPS(fr_tgt_rps);
        CLAMP_RPS(rl_tgt_rps);
#if !MECANUM_THREE_WHEEL_MODE
        CLAMP_RPS(rr_tgt_rps);
#endif
    }
}

/* ===========================================================================
 *  四、控制（1ms 调用一次）
 * =========================================================================*/
void Mecanum_Control(void)
{
    int16_t out_fl, out_fr, out_rl, out_rr;

#if MECANUM_USE_SPEED_LOOP
    /* 速度环闭环：目标转速 → 电流 */
    out_fl = (int16_t)Pid_cal(&Mecanum_FL_SpdPID, fl_tgt_rps, Mecanum_FL.Realrotationrate);
    out_fr = (int16_t)Pid_cal(&Mecanum_FR_SpdPID, fr_tgt_rps, Mecanum_FR.Realrotationrate);
    out_rl = (int16_t)Pid_cal(&Mecanum_RL_SpdPID, rl_tgt_rps, Mecanum_RL.Realrotationrate);
    out_rr = 0;
#if !MECANUM_THREE_WHEEL_MODE
    out_rr = (int16_t)Pid_cal(&Mecanum_RR_SpdPID, rr_tgt_rps, Mecanum_RR.Realrotationrate);
#endif
#else
    /* 开环（第一阶段）：目标转速线性映射到电流（粗略标定，仅验方向） */
    #define RPS_TO_CURRENT  (1500.0f)   /* 每 rps 对应电流（粗略） */
    out_fl = (int16_t)(fl_tgt_rps * RPS_TO_CURRENT);
    out_fr = (int16_t)(fr_tgt_rps * RPS_TO_CURRENT);
    out_rl = (int16_t)(rl_tgt_rps * RPS_TO_CURRENT);
    out_rr = 0;
#if !MECANUM_THREE_WHEEL_MODE
    out_rr = (int16_t)(rr_tgt_rps * RPS_TO_CURRENT);
#endif
#endif

    MotorDJI_SetOutput(&Mecanum_FL, out_fl);
#if !MECANUM_TWO_WHEEL_MODE
    MotorDJI_SetOutput(&Mecanum_FR, out_fr);
#endif
    MotorDJI_SetOutput(&Mecanum_RL, out_rl);
#if !MECANUM_THREE_WHEEL_MODE
    MotorDJI_SetOutput(&Mecanum_RR, out_rr);
#endif

    /* 三电机模式下同帧合并凑不满 4 槽，必须显式把已填槽的帧发出去，
       否则 C620 收不到控制帧就不会回报反馈，形成死锁 */
    MotorDJI_Flush(CAN_BUS_1);

    /* 调试打印 */
    {
        static uint32_t dbg_cnt = 0;
        if ((++dbg_cnt % 500u) == 0u)
        {
            Dbg_Print("[MEC] tgt=%.2f,%.2f,%.2f,%.2f ang=%.0f,%.0f,%.0f,%.0f out=%d,%d,%d,%d\n",
                      fl_tgt_rps, fr_tgt_rps, rl_tgt_rps, rr_tgt_rps,
                      Mecanum_FL.out_angle, Mecanum_FR.out_angle,
                      Mecanum_RL.out_angle, Mecanum_RR.out_angle,
                      out_fl, out_fr, out_rl, out_rr);
        }
    }
}

/* ===========================================================================
 *  五、停车
 * =========================================================================*/
void Mecanum_Stop(void)
{
    fl_tgt_rps = fr_tgt_rps = rl_tgt_rps = rr_tgt_rps = 0.0f;

    MotorDJI_SetOutput(&Mecanum_FL, 0);
#if !MECANUM_TWO_WHEEL_MODE
    MotorDJI_SetOutput(&Mecanum_FR, 0);
#endif
    MotorDJI_SetOutput(&Mecanum_RL, 0);
#if !MECANUM_THREE_WHEEL_MODE
    MotorDJI_SetOutput(&Mecanum_RR, 0);
#endif

    /* 停车也要把 0 电流帧发出去（同 Mecanum_Control 的三电机 flush） */
    MotorDJI_Flush(CAN_BUS_1);
}

/* ===========================================================================
 *  六、自转航向保持（WT931 反馈 + P 控制）
 *
 *  把底盘锁定在目标航向：误差 = 目标航向 - 当前连续航向(WT931_GetAllYaw)，
 *  乘 kp 得到 w 轴角速度，再交给逆运动学分配四轮转速。
 *  这是麦轮「自转 w 轴」闭环的最小实现，后续可加积分/微分。
 * =========================================================================*/
#define MECANUM_YAW_KP   1.5f   /* 航向 P 增益（°/s 每 ° 误差），调参从这里开始 */

void Mecanum_HoldYaw(float target_yaw_deg)
{
    float yaw_now = WT931_GetAllYaw();
    float yaw_err = target_yaw_deg - yaw_now;

    /* 误差归一到 ±180，避免跨 360 时走远路 */
    while (yaw_err >  180.0f) yaw_err -= 360.0f;
    while (yaw_err < -180.0f) yaw_err += 360.0f;

    /* P 控制：误差 → w 轴角速度（rad/s） */
    float wz = yaw_err * MECANUM_YAW_KP * 0.0174533f;  /* 度→弧度 */

    /* 限幅 */
    if (wz >  MECANUM_MAX_ANGULAR) wz =  MECANUM_MAX_ANGULAR;
    if (wz < -MECANUM_MAX_ANGULAR) wz = -MECANUM_MAX_ANGULAR;

    /* 只控自转，平移为 0 */
    Mecanum_SetVelocity(0.0f, 0.0f, wz);

    /* 调试打印（可选，看到 yaw 收敛到目标就对了） */
    {
        static uint32_t yaw_dbg = 0;
        if ((++yaw_dbg % 500u) == 0u)
        {
            Dbg_Print("[YAW] tgt=%.1f now=%.1f err=%.1f wz=%.2f\n",
                      target_yaw_deg, yaw_now, yaw_err, wz);
        }
    }
}
