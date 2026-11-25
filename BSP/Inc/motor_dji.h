#ifndef __MOTOR_DJI_H__
#define __MOTOR_DJI_H__

/**
 * ============================================================================
 *  motor_dji.h  ——  大疆电机（3508/2006/6020）协议层（对齐老队员入口名）
 *
 *  对齐点（本次重构）：
 *    · 反馈解析入口统一为老队员的 DIJMotor_ParaHandle(motor_t*, uint8_t[8])，
 *      内部再调用 DIJMotor_AngleHandle(motor_t*) 做多圈角解算。
 *      这样老队员 DIJ_Motor.c 的调用方式能原样对上。
 *    · 保留本工程验证过的增强实现：
 *        - 多实例注册表（按 bus+rx_id 分发）
 *        - 同帧合并发送（根治多电机互相清零）
 *        - 补码差分多圈解算（比老队员 6700/1500 阈值更稳）
 *
 *  分层（自下而上）：
 *    can_port.c 中断 → MotorDJI_OnFrame → DIJMotor_ParaHandle
 *                                          └─ DIJMotor_AngleHandle
 *    motion_motor.c 控制 → MotorDJI_SetOutput（同帧合并）
 * ============================================================================
 */

#include "main.h"
#include "Motor.h"
#include "can_port.h"

/* ---- 编码器参数 ---- */
#define DJI_ENCODER_CNT    8192u
#define DJI_ENCODER_MAX    (DJI_ENCODER_CNT - 1u)

/* ---- 掉线判定：超过这个毫秒没收到反馈就算掉线 ---- */
#define DJI_OFFLINE_MS     50u

/* ---- 注册表容量：最多支持这么多台大疆电机 ---- */
#define DJI_MOTOR_MAX      8u

/* ---- 输出量程 ---- */
#define DJI_VOLTAGE_MAX    30000    /* GM6020 电压模式上限 */
#define DJI_CURRENT_MAX    16384    /* 电流模式上限 */

/**
 * @brief  登记一台大疆电机进注册表（多实例）
 * @param  m   motor_t 对象（需已 MotortypeRegister 填好 type/bus/dev_id）
 * @retval 0 成功，1 注册表满，2 类型不支持
 */
uint8_t MotorDJI_Register(motor_t *m);

/**
 * @brief  收到一帧时由 can_port 中断调用，内部按 rx_id 分发
 * @note   中断上下文，禁止阻塞
 */
void MotorDJI_OnFrame(uint8_t bus, uint32_t id, uint8_t d[8]);

/**
 * @brief  解析一帧反馈，写入 mot->FeedbackData（对齐老队员入口名）
 * @param  mot    电机
 * @param  adata  8 字节反馈帧
 * @note   老队员 DIJ_Motor.c 同名函数；本工程保留此名以对齐。
 */
void DIJMotor_ParaHandle(motor_t *mot, uint8_t adata[]);

/**
 * @brief  多圈角解算（对齐老队员入口名）
 * @param  mot  电机
 * @retval 0 无变化/正常，±1 跨圈
 */
int8_t DIJMotor_AngleHandle(motor_t *mot);

/**
 * @brief  给某台电机设置输出（同帧合并）
 * @param  m    电机
 * @param  out  输出值（3508/2006 电流 ±16384；6020 电压 ±30000）
 */
uint8_t MotorDJI_SetOutput(motor_t *m, int16_t out);

/**
 * @brief  把指定总线上已攒但未发出的控制帧立即发出（补 0 槽）
 * @param  bus  总线号（CAN_BUS_1/2/3）
 * @note   电机数不是 4 的整数倍（如三电机缺 ID=4）时，同帧合并永远凑不满
 *         4 槽，需在每轮 SetOutput 结束后调用本函数强制发出。
 */
void MotorDJI_Flush(uint8_t bus);

/**
 * @brief  在线检测（1ms 控制任务里每周期调用）
 * @retval 1 在线，0 掉线
 */
uint8_t MotorDJI_IsOnline(const motor_t *m);

/**
 * @brief  把电机当前角度归零（SetZero）
 */
void MotorDJI_SetZero(motor_t *m);

#endif /* __MOTOR_DJI_H__ */
