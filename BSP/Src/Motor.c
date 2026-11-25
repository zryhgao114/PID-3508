/**
 * ============================================================================
 *  Motor.c  ——  电机对象注册（对齐老队员 Engineer_CusCtrlr）
 *
 *  老队员原签名（6 参）：
 *     void MotortypeRegister(motor_t *mot, motor_type type,
 *                            pid_type *pospid, pid_type *speedpid,
 *                            uint8_t Reductionratio, uint32_t Can_id);
 *
 *  本文件保持这个签名不变（90% 对齐目标），只做一处增强：
 *    · 注册时额外把 rx_id / tx_id 按型号算好（老队员在发送处才算），
 *      这样协议层 MotorDJI 能按反馈 ID 分发，无需再查表。
 *    · bus（总线号）由 MotorSetBus() 单独指定，默认 CAN_BUS_1。
 *      因为老队员的 Can_id 语义里不含总线，总线归属由「挂在哪个
 *      handler」决定，这里用显式函数表达同样语义，将来继承机构代码时
 *      只需补一行 MotorSetBus()。
 * ============================================================================
 */

#include "Motor.h"

/* ---- 总线默认值（can_port.h 里的 CAN_BUS_1）---- */
#ifndef CAN_BUS_1
#define CAN_BUS_1   1u
#endif

/**
 * @brief  注册一台电机（老队员 6 参签名）
 */
void MotortypeRegister(motor_t *mot, motor_type type,
                       pid_type *pospid, pid_type *speedpid,
                       uint8_t Reductionratio, uint32_t Can_id)
{
    if (mot == NULL)
        return;

    mot->id             = Can_id;
    mot->type           = type;
    mot->Reductionratio = Reductionratio;
    mot->PositionPID    = pospid;
    mot->SpeedPID       = speedpid;

    /* 增强字段初始化（老队员无这些字段，置安全初值） */
    mot->bus            = CAN_BUS_1;       /* 默认总线 1，可用 MotorSetBus 改 */
    mot->dev_id         = (uint8_t)Can_id;
    mot->Error_id       = 0;
    mot->Mode           = 0;
    mot->First_Frame    = 0;
    mot->Online         = 0;

    mot->FeedbackData.FrameCounter        = 0;
    mot->FeedbackData.RealSpeed           = 0;
    mot->FeedbackData.Current             = 0;
    mot->FeedbackData.Voltage             = 0;
    mot->FeedbackData.Mechanical_Angle[0] = 0;
    mot->FeedbackData.Mechanical_Angle[1] = 0;
    mot->FeedbackData.Temperature         = 0;

    mot->Oricirnum        = 0;
    mot->Realcirnum       = 0;
    mot->Targetcirnum     = 0;
    mot->Realrotationrate_i = 0;
    mot->Realrotationrate = 0.0f;
    mot->Targetrotationrate = 0.0f;

    mot->InitAngle        = 0.0f;
    mot->RealAngle        = 0.0f;
    mot->TargetAngle      = 0.0f;
    mot->TargetMechAngle  = 0;

    mot->out_angle        = 0.0f;
    mot->Out              = 0;

    mot->Blockstate.BlockState = noblock;
    mot->Blockstate.Blocktime  = 0;

    /* 按型号算反馈/控制 ID（增强：老队员在发送处算，这里提前算好） */
    switch (type)
    {
    case MOTOR_3508:
    case MOTOR_2006:
        /* 3508/2006：反馈 0x200+ID；控制 0x200(ID1~4) / 0x1FF(ID5~8) */
        mot->rx_id = 0x200u + Can_id;
        mot->tx_id = (Can_id <= 4) ? 0x200u : 0x1FFu;
        break;

    case MOTOR_6020:
        /* GM6020：反馈 0x204+ID；电压控制 0x1FF(ID1~4) / 0x2FF(ID5~7) */
        mot->rx_id = 0x204u + Can_id;
        mot->tx_id = (Can_id <= 4) ? 0x1FFu : 0x2FFu;
        break;

    default:
        /* 达妙 / 灵控等由各自协议文件填充 */
        mot->rx_id = 0u;
        mot->tx_id = 0u;
        break;
    }
}

/**
 * @brief  指定电机挂载的总线（增强函数，老队员无）
 * @param  mot  电机
 * @param  bus  CAN_BUS_1/2/3
 * @note   需在 MotorDJI_Register 之前调用，让协议层按 bus+rx_id 分发。
 */
void MotorSetBus(motor_t *mot, uint8_t bus)
{
    if (mot != NULL)
        mot->bus = bus;
}
