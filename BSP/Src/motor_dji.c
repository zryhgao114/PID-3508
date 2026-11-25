/**
 * ============================================================================
 *  motor_dji.c  ——  大疆电机协议层实现（多实例 + 同帧合并 + 老队员入口名）
 *
 *  对齐点（本次重构）：
 *    · 对外暴露老队员同名函数 DIJMotor_ParaHandle / DIJMotor_AngleHandle。
 *    · FeedbackData.Mechanical_Angle 改回数组 [2]，[1] 存上一帧，
 *      供老队员式「6700/1500 阈值跨圈判断」使用（本工程仍用更稳的
 *      补码差分，但数组字段补齐，将来机构代码能直接读）。
 *    · 保留本工程验证过的多实例注册表 + 同帧合并发送。
 *
 *  帧格式（8 字节 = 4 个 int16，大端）：
 *    字节 0~1  编码器机械角 0~8191
 *    字节 2~3  转速 rpm（转子，有符号）
 *    字节 4~5  实际电流（有符号）
 *    字节 6    温度 int8
 *    字节 7    保留
 * ============================================================================
 */

#include "motor_dji.h"

/* ===========================================================================
 *  注册表
 * =========================================================================*/
static motor_t *dj_reg[DJI_MOTOR_MAX];
static uint8_t  dj_reg_count = 0;

/* 每台电机的跨圈解算私有状态 */
typedef struct
{
    uint8_t  first;        /* 是否收到过第一帧 */
    uint16_t last_raw;     /* 上一帧编码器值 */
    int32_t  pos;          /* 累计编码器计数（转子），可正可负 */
    int32_t  zero_pos;     /* 零位偏移 */
    uint32_t last_tick;    /* 最后一帧时间戳 ms */
} dj_priv_t;

static dj_priv_t dj_priv[DJI_MOTOR_MAX];

/* ===========================================================================
 *  一、大端序辅助
 * =========================================================================*/
static int16_t be_i16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static void put_be_i16(uint8_t *p, int16_t v)
{
    p[0] = (uint8_t)((uint16_t)v >> 8);
    p[1] = (uint8_t)((uint16_t)v & 0xFF);
}

/* 向下取整除法：算圈数用 */
static int32_t floor_div(int32_t a, int32_t b)
{
    int32_t q = a / b;
    if ((a % b) != 0 && ((a < 0) != (b < 0)))
        q--;
    return q;
}

/* ===========================================================================
 *  二、登记
 * =========================================================================*/
uint8_t MotorDJI_Register(motor_t *m)
{
    if (m == NULL)
        return 2;

    if (m->type != MOTOR_3508 && m->type != MOTOR_2006 && m->type != MOTOR_6020)
        return 2;

    if (dj_reg_count >= DJI_MOTOR_MAX)
        return 1;

    dj_reg[dj_reg_count] = m;

    dj_priv[dj_reg_count].first     = 0;
    dj_priv[dj_reg_count].last_raw  = 0;
    dj_priv[dj_reg_count].pos       = 0;
    dj_priv[dj_reg_count].zero_pos  = 0;
    dj_priv[dj_reg_count].last_tick = 0;

    m->First_Frame = 0;
    m->Online      = 0;

    dj_reg_count++;
    return 0;
}

/* 按反馈 ID + 总线找到对应电机下标 */
static int8_t dj_find(uint8_t bus, uint32_t id)
{
    uint8_t i;
    for (i = 0; i < dj_reg_count; i++)
    {
        if (dj_reg[i] != NULL && dj_reg[i]->bus == bus && dj_reg[i]->rx_id == id)
            return (int8_t)i;
    }
    return -1;
}

/* 按电机指针找私有状态下标 */
static int8_t dj_find_mot(const motor_t *m)
{
    uint8_t i;
    for (i = 0; i < dj_reg_count; i++)
    {
        if (dj_reg[i] == m)
            return (int8_t)i;
    }
    return -1;
}

/* ===========================================================================
 *  三、解帧（中断上下文，必须短）
 *
 *  MotorDJI_OnFrame：can_port 中断调用 → 按 rx_id 找到电机 → 调
 *  DIJMotor_ParaHandle（老队员同名入口）。
 * =========================================================================*/
void MotorDJI_OnFrame(uint8_t bus, uint32_t id, uint8_t d[8])
{
    int8_t idx = dj_find(bus, id);
    if (idx < 0)
        return;
    DIJMotor_ParaHandle(dj_reg[idx], d);
}

/**
 * @brief  解析一帧反馈（对齐老队员 DIJMotor_ParaHandle）
 * @note   大疆 3508/2006/6020 帧格式相同，这里统一处理。
 */
void DIJMotor_ParaHandle(motor_t *mot, uint8_t adata[])
{
    int8_t idx;
    dj_priv_t *p;
    uint16_t raw;
    int16_t  delta;
    uint8_t  reduction;

    if (mot == NULL)
        return;

    idx = dj_find_mot(mot);
    if (idx < 0)
        return;

    p = &dj_priv[idx];
    reduction = (mot->Reductionratio != 0u) ? mot->Reductionratio : 1u;

    mot->FeedbackData.FrameCounter++;

    /* 机械角 [1] = 上一帧，[0] = 本次（老队员式跨圈判断依赖） */
    if (mot->First_Frame == 1)
        mot->FeedbackData.Mechanical_Angle[1] = mot->FeedbackData.Mechanical_Angle[0];

    /* 字节 0~1：编码器值 0~8191 */
    raw = (uint16_t)(((uint16_t)adata[0] << 8) | adata[1]);
    mot->FeedbackData.Mechanical_Angle[0] = raw;

    /* 字节 2~3：转速 rpm（有符号，转子转速） */
    mot->FeedbackData.RealSpeed = be_i16(&adata[2]);

    /* 字节 4~5：实际电流（有符号） */
    mot->FeedbackData.Current = be_i16(&adata[4]);

    /* 字节 6：温度 int8 */
    mot->FeedbackData.Temperature = (int16_t)(int8_t)adata[6];

    /* 转速：先存转子 rpm，多圈解算后再折算输出轴 rps */
    mot->Realrotationrate = (float)mot->FeedbackData.RealSpeed;
    mot->Realrotationrate_i = mot->FeedbackData.RealSpeed;

    /* ---- 补码差分多圈累积（本工程增强实现）---- */
    if (!p->first)
    {
        p->first    = 1;
        p->last_raw = raw;
        p->pos      = (int32_t)raw;
        p->zero_pos = p->pos;
        mot->First_Frame = 1;
    }
    else
    {
        delta = (int16_t)(uint16_t)(raw - p->last_raw);

        /* 超过半圈的跳变按「绕近路」修正 */
        if (delta > (int16_t)(DJI_ENCODER_CNT / 2u))
            delta = (int16_t)(delta - (int16_t)DJI_ENCODER_CNT);
        else if (delta < -(int16_t)(DJI_ENCODER_CNT / 2u))
            delta = (int16_t)(delta + (int16_t)DJI_ENCODER_CNT);

        p->last_raw = raw;
        p->pos += delta;
    }

    /* 多圈角解算（老队员同名入口） */
    DIJMotor_AngleHandle(mot);

    /* 输出轴转速 rps（转子 rpm ÷ 减速比 ÷ 60），速度环反馈用 */
    mot->Realrotationrate = (float)mot->FeedbackData.RealSpeed
                            / (float)reduction / 60.0f;

    p->last_tick = HAL_GetTick();
}

/**
 * @brief  多圈角解算（对齐老队员 DIJMotor_AngleHandle）
 * @note   本工程用补码差分累加的 out_angle 作为位置环反馈；
 *         同时维护老队员的 Oricirnum/Realcirnum/RealAngle 字段，
 *         保证机构代码直接读这些字段时数值一致。
 */
int8_t DIJMotor_AngleHandle(motor_t *mot)
{
    int8_t idx;
    dj_priv_t *p;
    int32_t rel;
    uint8_t reduction;
    float temp_angle;

    if (mot == NULL)
        return 0;

    idx = dj_find_mot(mot);
    if (idx < 0)
        return 0;

    p = &dj_priv[idx];
    reduction = (mot->Reductionratio != 0u) ? mot->Reductionratio : 1u;

    /* 相对零位的累计计数 */
    rel = p->pos - p->zero_pos;

    /* 圈数：输出轴圈数（折减速比） */
    mot->Realcirnum = (int16_t)floor_div(rel, (int32_t)DJI_ENCODER_CNT * reduction);

    /* 未折减速比的转子圈数（老队员 Oricirnum 语义） */
    mot->Oricirnum = (int16_t)floor_div(rel, (int32_t)DJI_ENCODER_CNT);

    /* 连续多圈角（增强字段，位置环反馈首选） */
    mot->out_angle = (float)rel * 360.0f
                     / (float)DJI_ENCODER_CNT / (float)reduction;

    /* 单圈角 0~360（老队员 RealAngle） */
    temp_angle = MechanicalAngle2RealAngle(mot->FeedbackData.Mechanical_Angle[0])
                 / (float)reduction;
    while (temp_angle >= 360.0f)
        temp_angle -= 360.0f;
    while (temp_angle < 0.0f)
        temp_angle += 360.0f;
    mot->RealAngle = temp_angle;

    return 0;
}

/* ===========================================================================
 *  四、发输出：同帧合并
 * =========================================================================*/

typedef struct
{
    uint8_t  bus;
    uint32_t tx_id;
    int16_t  slot[4];
    uint8_t  used;    /* 位掩码 */
} dj_tx_slot_t;

static dj_tx_slot_t dj_tx[DJI_MOTOR_MAX];

static int8_t dj_tx_find(uint8_t bus, uint32_t tx_id)
{
    uint8_t i;
    for (i = 0; i < DJI_MOTOR_MAX; i++)
    {
        if (dj_tx[i].used != 0u && dj_tx[i].bus == bus && dj_tx[i].tx_id == tx_id)
            return (int8_t)i;
    }
    for (i = 0; i < DJI_MOTOR_MAX; i++)
    {
        if (dj_tx[i].used == 0u)
        {
            dj_tx[i].bus   = bus;
            dj_tx[i].tx_id = tx_id;
            return (int8_t)i;
        }
    }
    return -1;
}

uint8_t MotorDJI_SetOutput(motor_t *m, int16_t out)
{
    int8_t  ti;
    uint8_t slot;
    uint8_t d[8];
    int     i;

    if (m == NULL)
        return 1;

    /* 限幅 */
    {
        int32_t max = (m->type == MOTOR_6020) ? DJI_VOLTAGE_MAX : DJI_CURRENT_MAX;
        if (out >  max) out = (int16_t)max;
        if (out < -max) out = (int16_t)(-max);
    }

    /* 槽位：ID 1~4 → 槽 0~3 */
    slot = (uint8_t)((m->dev_id - 1u) & 0x03u);

    ti = dj_tx_find(m->bus, m->tx_id);
    if (ti < 0)
        return 1;

    dj_tx[ti].slot[slot] = out;
    dj_tx[ti].used |= (uint8_t)(1u << slot);

    /* 攒满 4 槽发一帧 */
    if (dj_tx[ti].used == 0x0Fu)
    {
        for (i = 0; i < 4; i++)
            put_be_i16(&d[i * 2], dj_tx[ti].slot[i]);

        CAN_Send(dj_tx[ti].bus, dj_tx[ti].tx_id, d);

        dj_tx[ti].used = 0u;
        return 0;
    }

    /* 仅此一台（同 tx_id 只有一台）：直接发单帧，其余槽补 0 */
    {
        uint8_t same = 0;
        for (i = 0; i < (int)dj_reg_count; i++)
        {
            if (dj_reg[i] != NULL && dj_reg[i]->bus == m->bus
                && dj_reg[i]->tx_id == m->tx_id)
                same++;
        }

        if (same <= 1)
        {
            uint8_t dd[8] = {0,0,0,0,0,0,0,0};
            put_be_i16(&dd[slot * 2], out);
            dj_tx[ti].used = 0u;
            return CAN_Send(m->bus, m->tx_id, dd);
        }
    }

    return 0;
}

/**
 * @brief  把指定总线上「已攒但未发」的控制帧立即发出
 * @param  bus  总线号（CAN_BUS_1/2/3）
 * @note   同帧合并按 4 槽凑满才发，遇到「电机数不是 4 的整数倍」（如三电机
 *         模式缺 ID=4）时 used 永远凑不满 0x0F，一帧都发不出去。这时由上层
 *         在每轮设置完所有电机后调用本函数，把已填槽的帧强制发出，其余槽补 0。
 *         四轮完整模式凑满 4 槽时 used 已被清零，本函数不会重复发送。
 */
void MotorDJI_Flush(uint8_t bus)
{
    uint8_t i;
    uint8_t d[8];

    for (i = 0; i < DJI_MOTOR_MAX; i++)
    {
        if (dj_tx[i].used != 0u && dj_tx[i].bus == bus)
        {
            int s;
            for (s = 0; s < 4; s++)
                put_be_i16(&d[s * 2], dj_tx[i].slot[s]);

            CAN_Send(dj_tx[i].bus, dj_tx[i].tx_id, d);

            dj_tx[i].used = 0u;
        }
    }
}

/* ===========================================================================
 *  五、掉线检测
 * =========================================================================*/
uint8_t MotorDJI_IsOnline(const motor_t *m)
{
    int8_t idx;

    if (m == NULL || m->FeedbackData.FrameCounter == 0)
        return 0;

    idx = dj_find_mot(m);
    if (idx < 0)
        return 0;

    return ((HAL_GetTick() - dj_priv[idx].last_tick) < DJI_OFFLINE_MS) ? 1 : 0;
}

/* ===========================================================================
 *  六、零位
 * =========================================================================*/
void MotorDJI_SetZero(motor_t *m)
{
    int8_t idx = dj_find_mot(m);
    if (idx < 0)
        return;

    dj_priv[idx].zero_pos = dj_priv[idx].pos;
    m->out_angle = 0.0f;
    m->Realcirnum = 0;
    m->Oricirnum  = 0;
}
