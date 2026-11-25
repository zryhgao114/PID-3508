#ifndef __MOTOR_H__
#define __MOTOR_H__

/**
 * ============================================================================
 *  Motor.h  ——  电机抽象对象（对齐老队员 Engineer_CusCtrlr 的 motor_t）
 *
 *  对齐原则（本次重构的核心目标）：
 *    · 字段命名以老队员为准，保证将来「全盘继承」机构/云台/机械臂代码时
 *      字段名能直接对上，不用全局改名。
 *    · 老队员缺的、但实车验证过的「增强字段」作为补充共存，不冲突。
 *
 *  与老队员 motor_t 的对应关系：
 *    老队员字段                本文件（对齐后）
 *    --------------------     --------------------
 *    type / id                 同（id = Can_id 语义保留）
 *    Error_id / Mode           补回
 *    PositionPID / SpeedPID    同
 *    FeedbackData              同（Mechanical_Angle 改回数组 [2]）
 *    Reductionratio            同
 *    Oricirnum/Realcirnum      补回（多圈计数体系）
 *    Targetcirnum              补回
 *    Realrotationrate          同
 *    Targetrotationrate        补回
 *    InitAngle/RealAngle       同
 *    TargetAngle               同
 *    TargetMechAngle           补回
 *    Out                       同
 *    Blockstate                同
 *    First_Frame               同
 *    LK_ControlParam           补回（灵控电机参数块）
 *
 *  本文件增强字段（老队员没有，保留你验证过的实现）：
 *    bus / dev_id / rx_id / tx_id   协议层多实例分发用
 *    out_angle                       连续多圈角（补码差分解算，比 6700/1500 阈值稳）
 *    Online                          在线标志
 *
 *  用法：加一台电机 = 声明一个 motor_t + 两个 pid_type，注册一行。
 * ============================================================================
 */

#include "main.h"
#include "PID.h"

#define MOTOR_MAX_BLOCKTIME   (7000)

/* ---- 电机型号枚举（与老队员 motor_type 对齐，扩展位保留）---------------- */
typedef enum
{
    MOTOR_NONE = 0,
    MOTOR_2006,
    MOTOR_3508,
    MOTOR_4310,
    MOTOR_4315,
    MOTOR_6020,
    MOTOR_6623,
    MOTOR_9015,
    MOTOR_MF4005,
    MOTOR_MG4005,
    MOTOR_MG4010,
    MOTOR_MF4015,
    MOTOR_MS3008,
    MOTOR_MS5005,
    MOTOR_TYPE_COUNTER
} motor_type;

/* ---- 减速比（电机自带减速箱）--------------------------------------------- */
#define MOTOR3508_Reductionratio   19
#define MOTOR3508Lift_Reductionratio 27
#define MOTOR2006_Reductionratio   36
#define MOTOR6020_Reductionratio   1

/* ---- 输出限幅（按电调量程）----------------------------------------------- */
#define M3508_OUTMAX     15999   /* C620 电流 ±16384，留余量 */
#define M3508_KIMAX      10000   /* 3508 速度环积分上限 */
#define M3508_SPEEDMAX   10000
#define M2006_OUTMAX      9999   /* C610 电流 ±10000 */
#define M6020_OUTMAX     29999   /* GM6020 电压模式 ±30000 */

/* ---- 机械角 → 实际角换算（0~8191 → 0~360°）------------------------------ */
#define MechanicalAngle2RealAngle(Mechanical_Angle) \
    ((Mechanical_Angle) / 8191.0f * 360.0f)

/* ---- RS485 发送缓冲（老队员遗留，保留类型）------------------------------- */
typedef struct
{
    uint8_t Data[66];
} RS485Send_Type;

/* ---- 反馈数据（协议层填充）------------------------------------------------ */
typedef struct
{
    uint32_t    FrameCounter;       /* 收到帧数（在线判据） */
    int16_t     RealSpeed;          /* 转子转速 rpm（未除减速比） */
    int16_t     Current;            /* 实际电流原始值 */
    int16_t     Voltage;            /* 实际电压（6020 用） */
    uint16_t    Mechanical_Angle[2];/* 机械角 [0]:本次 [1]:上次，用于跨圈判断 */
    int16_t     Temperature;        /* 摄氏度 */
} Motor_Feedback_Data;

/* ---- 堵转状态 ------------------------------------------------------------- */
typedef enum
{
    noblock = 0,
    isblock,
    BlockState_Counter
} BlockState_t;

typedef struct
{
    BlockState_t BlockState;    /* 是否堵转 */
    uint16_t     Blocktime;     /* ms */
} BlockStruct_t;

/* ---- 灵控电机控制参数块（老队员 LK_ControlParam_t，后续 6020/灵控用）----- */
typedef struct
{
    uint16_t anglePid[3];
    uint16_t speedPid[3];
    uint16_t currentPid[3];
    uint16_t inputTorqueLimit;
    uint32_t inputSpeedLimit;
    int32_t  inputAngleLimit;
    int32_t  inputCurrentRamp;
    int32_t  inputSpeedRamp;
} LK_ControlParam_t;

/* ---- 电机对象（对齐老队员，字段顺序尽量贴近）----------------------------- */
typedef struct
{
    /* 身份 */
    motor_type  type;               /* 型号 */
    uint32_t    id;                 /* CAN id（=老队员 Can_id 语义） */
    uint8_t     Error_id;
    uint8_t     Mode;

    /* 增强：协议层多实例分发用（老队员无，本工程保留） */
    uint8_t     bus;                /* CAN_BUS_1/2/3 */
    uint8_t     dev_id;             /* 电调拨码 ID 1~8 */
    uint32_t    rx_id;              /* 反馈帧 ID（协议层算出） */
    uint32_t    tx_id;              /* 控制帧 ID（协议层算出） */

    /* 双环 PID */
    pid_type    *PositionPID;
    pid_type    *SpeedPID;

    /* 反馈 */
    Motor_Feedback_Data FeedbackData;

    /* 减速比 + 多圈计数（老队员圈数体系） */
    uint8_t     Reductionratio;     /* 减速比 */
    int16_t     Oricirnum;          /* 电机当前转过圈数（未折减速比） */
    int16_t     Realcirnum;         /* 输出轴实际圈数（折减速比后） */
    int16_t     Targetcirnum;       /* 目标圈数 */

    /* 转速 */
    int16_t     Realrotationrate_i; /* 输出轴转速 rps 的整型（老队员位） */
    float       Realrotationrate;   /* 输出轴转速 rps（速度环反馈，浮点） */
    float       Targetrotationrate; /* 目标转速 rps */

    /* 角度体系（老队员） */
    float       InitAngle;          /* 上电标定零位 */
    float       RealAngle;          /* 单圈实际角 0~360 */
    float       TargetAngle;        /* 目标角（相对 InitAngle） */
    int16_t     TargetMechAngle;    /* 目标机械角 */

    /* 增强：连续多圈角（补码差分解算，位置环反馈首选） */
    float       out_angle;          /* 输出轴连续多圈角，不归零 */

    /* 输出 */
    int16_t     Out;                /* 最终发给电调的电流/电压值 */

    /* 堵转 */
    BlockStruct_t Blockstate;

    uint8_t     First_Frame;        /* 是否收到第一帧 */
    uint8_t     Online;             /* 在线标志（增强） */

    LK_ControlParam_t LK_ControlParam;

} motor_t;

/**
 * @brief  注册一台电机（对齐老队员 6 参签名，内部补算 bus/dev_id/rx_id/tx_id）
 * @param  mot            电机对象
 * @param  type           型号（MOTOR_3508 / MOTOR_6020 ...）
 * @param  pospid         位置环 PID 指针
 * @param  speedpid       速度环 PID 指针
 * @param  Reductionratio 减速比
 * @param  Can_id         电调拨码 ID（1~8），总线在协议层登记时指定
 */
void MotortypeRegister(motor_t *mot, motor_type type,
                       pid_type *pospid, pid_type *speedpid,
                       uint8_t Reductionratio, uint32_t Can_id);

/**
 * @brief  指定电机挂载的总线（增强函数，老队员无）
 */
void MotorSetBus(motor_t *mot, uint8_t bus);

#endif /* __MOTOR_H__ */
