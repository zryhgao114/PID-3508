/**
 * ============================================================================
 *  task_can.c  ——  三个任务的实现
 *
 *  ★★★ 上电前只需要改这三行 ★★★
 * ============================================================================
 */

#include "task_can.h"
#include "can_port.h"
#include "motor_dji.h"
#include "motion_motor.h"
#include "mecanum.h"
#include "WT931_Gyro.h"
#include "remote_control.h"

#include "dbg.h"

/* ---------------------------------------------------------------------------
 *  运动模式选择（下一步调麦轮的核心开关）
 *     0 = 单电机调 PID（3508 位置环，沿用之前的工作）
 *     1 = 四轮麦轮底盘（mecanum.c）
 * -------------------------------------------------------------------------*/
#define MOTION_MODE_MOTOR   0
#define MOTION_MODE_MECANUM 1
#define MOTION_MODE         MOTION_MODE_MECANUM   /* ★ 切这里 ★：麦轮遥控模式 */

/* ---------------------------------------------------------------------------
 *  ① 电机挂在哪条总线、拨码是几号
 *  注：电机类型/ID/减速比现在在 motion_motor.c 的 MotortypeRegister 里注册，
 *      这里的 MOTOR_BUS 只用于 Bus-Off 自愈选句柄，请与注册值保持一致。
 * -------------------------------------------------------------------------*/
#define MOTOR_BUS        CAN_BUS_1     /* 先用 CAN1 试 */
#define MOTOR_ID         1             /* 电调上拨码设的 ID */

/* ---------------------------------------------------------------------------
 *  ② 验证阶段 —— 只改这一个数字，就能决定"电机动不动"
 *
 *     1 = 只收不发。串口能看到角度跟着手转，但电机不动、不抖。
 *         ★ 第一次烧录必须停在这里，先把"收得到帧"这件事确认掉。
 *     2 = 允许发电流（进入位置环闭环）。
 *
 *  分两步走的原因：收不到帧时去调输出，你永远不知道问题在收还是在发。
 * -------------------------------------------------------------------------*/
#define MOTOR_TEST_STAGE      2
#define MOTOR_OUTPUT_ENABLE   (MOTOR_TEST_STAGE >= 2)

/* 上电后先静默这么久再出力：留时间看串口、确认电机是空载、手离开转子 */
#define MOTOR_START_DELAY_MS  3000

/* ---------------------------------------------------------------------------
 *  任务优先级与栈
 *
 *  优先级数字越大越优先，但必须小于 configMAX_PRIORITIES（默认 7）。
 *  Task_Init 自己跑在 defaultTask（osPriorityNormal），
 *  所以这里所有任务的优先级都不超过 3，避免"建任务时把自己饿死"。
 *
 *  ★ 入口函数签名是 void Task_Init(void *argument) —— 对应 CMSIS-RTOS **V2**。
 *    FW_H7 V1.13.0 起，ST 已经删掉了 CMSIS_RTOS_V1 包装层，
 *    只能用 V2；V1 的签名是 void const *argument，写成那样会报类型冲突。
 *
 *  栈单位是字（word），256 words = 1KB，384 words = 1.5KB。
 * -------------------------------------------------------------------------*/
#define MOTOR_TASK_PRIO    3
#define MOTOR_TASK_STACK   256

#define CAN_TASK_PRIO      2
#define CAN_TASK_STACK     256

#define PRINT_TASK_PRIO    1
#define PRINT_TASK_STACK   384

#define MOTOR_PERIOD_MS    1        /* 1kHz 控制周期 */
#define PRINT_PERIOD_MS    500

/* ===========================================================================
 *  一、入口任务
 *
 *  五步的顺序不能变，每一步后面都写了原因。
 * =========================================================================*/
void App_Init(void)
{
    /* 本函数由 freertos.c 的 USER CODE BEGIN RTOS_THREADS 段调用，
       此时调度器尚未启动，但 xQueueCreate / xTaskCreate 已可用。 */

    /* ① 先建发送队列。
          放到 CAN 初始化之后的话，中断可能抢先到达，
          回调里拿到的是空指针队列 → HardFault。 */
    Queue_CanSend = xQueueCreate(CAN_TX_QUEUE_LEN, sizeof(CanFrame_t));
    if (Queue_CanSend == NULL)
        Error_Handler();

    /* ② 再初始化 CAN：过滤器 + 全局过滤 + Start + 使能中断
          ★ 调试期不直接 Error_Handler()：那个函数是 __disable_irq(); while(1);
            一旦某一路 CAN 起不来，程序会静默死在这里，串口一个字都不发，
            你会误以为是串口坏了。这里改成把错误码打出来再继续，
            至少能知道卡在哪一路。稳定后可以改回 Error_Handler()。 */
    {
        uint8_t err = CAN_Port_Init();

        if (err != 0)
        {
            Dbg_Print("[FATAL] CAN_Port_Init err=%u  (1=FDCAN1 2=FDCAN2 3=FDCAN3)\r\n",
                      (unsigned)err);
        }
    }

    /* ③ 登记电机（算出反馈 ID / 控制 ID）
          现在走 motor_t 抽象对象：注册电机 + 初始化双环 PID。 */
#if MOTION_MODE == MOTION_MODE_MOTOR
    MotionMotor_Init();
#elif MOTION_MODE == MOTION_MODE_MECANUM
    Mecanum_Init();
#endif

    /* ③b 初始化 WT931 陀螺仪（麦轮自转闭环用，单电机模式也可留作调试）
          ★ 需在 UART7(PE7/PE8) 接 WT931，且其波特率配 115200。
          UART7 的 NVIC 中断由 WT931_Init 内部手动使能。 */
    WT931_Init();

    /* ③c 初始化遥控器（DR16 DBUS，UART5 接收）
          ★ 需在 UART5(PC12 TX / PD2 RX) 接接收机。
          上电后遥控「离线」状态，双上保护、失联保护都依赖它。 */
    RemoteControl_Init();


    /* ④ 建工作任务
          ★ 必须检查返回值。xTaskCreate 失败只返回 errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY，
            不会报任何错：任务不存在，调度器照常跑，串口一片安静 —— 和"调度器没起来"
            长得一模一样。configTOTAL_HEAP_SIZE 只有 15KB，六个任务（含 idle/timer）
            已经吃掉一半，以后再加任务很容易在这里悄悄失败。 */
    {
        BaseType_t ok = pdPASS;

        /* 注意别写成 ok &= xTaskCreate(...)：失败返回值是 -1，1 & -1 还是 1，
           与运算会把失败"与"掉。必须逐个比较。 */
        if (xTaskCreate(Task_Print, "print", PRINT_TASK_STACK, NULL, PRINT_TASK_PRIO, NULL) != pdPASS)
            ok = pdFAIL;
        if (xTaskCreate(Task_CAN, "can", CAN_TASK_STACK, NULL, CAN_TASK_PRIO, NULL) != pdPASS)
            ok = pdFAIL;
        if (xTaskCreate(Task_Motor, "motor", MOTOR_TASK_STACK, NULL, MOTOR_TASK_PRIO, NULL) != pdPASS)
            ok = pdFAIL;

        if (ok != pdPASS)
        {
            Dbg_Print("[FATAL] xTaskCreate 失败：FreeRTOS 堆不够（configTOTAL_HEAP_SIZE=%u）\r\n",
                      (unsigned)configTOTAL_HEAP_SIZE);
        }
    }

    /* 心跳 2/4：'A' = 队列、CAN、电机、三个任务全部建完。
       看到 K 看不到 A → 卡在本函数里（大概率是 CAN_Port_Init 或 xTaskCreate）。
       注意：此刻调度器还没启动，这行一定会执行完才会进 osKernelStart。 */
    Dbg_Ch('A');
}

/* ===========================================================================
 *  二、发送任务
 *
 *  99% 的时间阻塞在 xQueueReceive 上，不占 CPU。
 *  这样设计的好处：发送可能因为总线忙而失败，但绝不会拖慢控制任务。
 * =========================================================================*/
void Task_CAN(void *argument)
{
    CanFrame_t f;

    (void)argument;

    for (;;)
    {
        if (xQueueReceive(Queue_CanSend, &f, portMAX_DELAY) != pdTRUE)
            continue;

        switch (f.bus)
        {
        case CAN_BUS_1: CAN_Transmit(&hfdcan1, f.id, f.data); break;
        case CAN_BUS_2: CAN_Transmit(&hfdcan2, f.id, f.data); break;
#if CAN_USE_BUS3
        case CAN_BUS_3: CAN_Transmit(&hfdcan3, f.id, f.data); break;
#endif
        default: break;
        }
    }
}

/* ===========================================================================
 *  三、控制任务（1kHz）—— 位置环闭环
 *
 *  调用链：读反馈 → 位置环(目标角→目标转速) → 速度环(目标转速→电流)
 *           → 限幅/爬升 → MotorDJI_SetOutput → 发给电调
 *
 *  保留的三层保护（防事故，加了 PID 也不能删）：
 *      · 上电静默    上电瞬间不输出，留时间确认空载、手离开转子
 *      · 掉线发零    收不到反馈就发 0，防电机拿着旧指令一直转（飞车）
 *      · 输出爬升    目标从 0 线性爬升，避免冲击电流打齿
 * =========================================================================*/
void Task_Motor(void *argument)
{
    TickType_t last_wake;
    uint32_t   start_tick;
    uint32_t   hb = 0;

    (void)argument;

    Dbg_Ch('T');

    last_wake  = xTaskGetTickCount();
    start_tick = HAL_GetTick();

    for (;;)
    {
        /* ---- Bus Off 自愈（同前） ---- */
        {
            static uint32_t bo_tick = 0;
            FDCAN_HandleTypeDef *hcan = (MOTOR_BUS == CAN_BUS_2) ? &hfdcan2 : &hfdcan1;

            if ((hcan->Instance->PSR & FDCAN_PSR_BO) != 0u)
            {
                uint32_t now = HAL_GetTick();
                if ((now - bo_tick) >= 500u)
                {
                    bo_tick = now;
                    (void)HAL_FDCAN_Stop(hcan);
                    (void)HAL_FDCAN_Start(hcan);
                    Dbg_Ch('B');
                }
            }
        }

#if MOTOR_OUTPUT_ENABLE
        {
            int16_t out = 0;
            uint8_t ready = ((HAL_GetTick() - start_tick) >= MOTOR_START_DELAY_MS) ? 1u : 0u;

#if MOTION_MODE == MOTION_MODE_MOTOR
            /* ---- 单电机调 PID 模式 ---- */
            /* 掉线保护：收不到反馈立刻发 0 */
            if (MotorDJI_IsOnline(&Motor_3508_1) && ready)
            {
                /* 位置环 + 速度环 算电流 */
                MotionMotor_Control();

                out = Motor_3508_1.Out;
            }
            else
            {
                /* 静默期 / 掉线：输出 0（电机有保持力矩但不转） */
                out = 0;
            }

            MotorDJI_SetOutput(&Motor_3508_1, out);

#elif MOTION_MODE == MOTION_MODE_MECANUM
            /* ---- 四轮麦轮底盘模式（遥控器控制 + 双上断电保护）----
               运动来源：遥控器摇杆
                   ch0（右横）→ 横向 Vy
                   ch1（右竖）→ 纵向 Vx
                   ch2（左横）→ 自转 Wz
               保护链（四层，任一触发都断动力）：
                   ① 四轮任一掉线
                   ② 遥控器失联
                   ③ 双上保护（左右拨杆都拨到上）
                   ④ 上电静默期未过 */
            {
                uint8_t all_online = MotorDJI_IsOnline(&Mecanum_FL)
#if !MECANUM_TWO_WHEEL_MODE
                                   && MotorDJI_IsOnline(&Mecanum_FR)
#endif
                                   && MotorDJI_IsOnline(&Mecanum_RL);
#if !MECANUM_THREE_WHEEL_MODE
                /* 四轮完整模式才要求后右轮在线 */
                all_online = all_online && MotorDJI_IsOnline(&Mecanum_RR);
#endif

                uint8_t rc_online   = RemoteControl_IsOnline();
                uint8_t double_up   = RemoteControl_IsDoubleUp();

                if (all_online && rc_online && !double_up && ready)
                {
                    /* 摇杆 → 底盘三轴速度（归一化 -1~1 → 实际 m/s、rad/s） */
                    float vx = RemoteControl_GetChannel(1) * MECANUM_MAX_LINEAR;   /* 右竖：前/后 */
                    float vy = RemoteControl_GetChannel(0) * MECANUM_MAX_LINEAR;   /* 右横：左/右 */
                    float wz = RemoteControl_GetChannel(2) * MECANUM_MAX_ANGULAR;  /* 左横：自转 */

                    Mecanum_SetVelocity(vx, vy, wz);
                    Mecanum_Control();
                }
                else
                {
                    Mecanum_Stop();
                }
            }
#endif
        }
#endif

        /* 心跳 'M' */
        if (++hb >= 500u)
        {
            hb = 0u;
            Dbg_Ch('M');
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(MOTOR_PERIOD_MS));
    }
}



/* ===========================================================================
 *  五、打印任务（低优先级，500ms 一次）
 *
 *  把打印单独拆出来，是为了让 1kHz 控制任务永远不被串口拖慢。
 *  115200 下打 60 个字符要 5ms，如果放在控制任务里，5 个周期全废。
 * =========================================================================*/
void Task_Print(void *argument)
{
    TickType_t last_wake;

    (void)argument;

    /* 进入标记：'P'。看到 P 说明打印任务也被调度到了。
       P 后面紧跟一行 "收,发,发错,lec"，那才是真正的数据。 */
    Dbg_Ch('P');

    /* ★ 这里不再打任何抬头。
       串口每次只送得出去十来个字节，抬头两行就四十多个字节，
       额度全被它吃掉，真正要看的数据一个都到不了。
       第一行输出就是数据本身 —— 看到它，就说明任务在跑。 */

    last_wake = xTaskGetTickCount();

    for (;;)
    {
        /* 打印 PID 调参关键量（一行 5 个数，控制在 40 字节内）：
             cnt   收到帧数（/100）。不涨 = 一帧都没收到（查接线/ID）
             ang   连续多圈角 °（位置环反馈，手转应连续变化）
             tgt   目标角 °（当前写死，见 motion_motor.h）
             spd   输出轴转速 rps（速度环反馈）
             out   最终电流指令（发给电调）
           第二行：
             收,发,发错,lec   （CAN 链路健康度） */
        {
            uint32_t psr = hfdcan1.Instance->PSR;

#if MOTION_MODE == MOTION_MODE_MOTOR
            Dbg_Print("%lu,%d,%d,%d,%d\n",
                      (unsigned long)(Motor_3508_1.FeedbackData.FrameCounter / 100u),
                      (int)(Motor_3508_1.out_angle * 10.0f),
                      (int)(Motor_3508_1.TargetAngle * 10.0f),
                      (int)(Motor_3508_1.Realrotationrate * 100.0f),
                      (int)Motor_3508_1.Out);
#elif MOTION_MODE == MOTION_MODE_MECANUM
#if MECANUM_THREE_WHEEL_MODE
            Dbg_Print("FL:%d,%d,%d FR:%d,%d,%d RL:%d,%d,%d\n",
                      (int)(Mecanum_FL.FeedbackData.FrameCounter / 100u),
                      (int)(Mecanum_FL.out_angle * 10.0f),
                      (int)Mecanum_FL.Out,
                      (int)(Mecanum_FR.FeedbackData.FrameCounter / 100u),
                      (int)(Mecanum_FR.out_angle * 10.0f),
                      (int)Mecanum_FR.Out,
                      (int)(Mecanum_RL.FeedbackData.FrameCounter / 100u),
                      (int)(Mecanum_RL.out_angle * 10.0f),
                      (int)Mecanum_RL.Out);
#else
            Dbg_Print("FL:%d,%d,%d FR:%d,%d,%d RL:%d,%d,%d RR:%d,%d,%d\n",
                      (int)(Mecanum_FL.FeedbackData.FrameCounter / 100u),
                      (int)(Mecanum_FL.out_angle * 10.0f),
                      (int)Mecanum_FL.Out,
                      (int)(Mecanum_FR.FeedbackData.FrameCounter / 100u),
                      (int)(Mecanum_FR.out_angle * 10.0f),
                      (int)Mecanum_FR.Out,
                      (int)(Mecanum_RL.FeedbackData.FrameCounter / 100u),
                      (int)(Mecanum_RL.out_angle * 10.0f),
                      (int)Mecanum_RL.Out,
                      (int)(Mecanum_RR.FeedbackData.FrameCounter / 100u),
                      (int)(Mecanum_RR.out_angle * 10.0f),
                      (int)Mecanum_RR.Out);
#endif

            /* 遥控器状态：收帧数、在线、双上、三轴归一化（×1000） */
            Dbg_Print("RC:%lu,%d,%d,%d,%d,%d,%d\n",
                      (unsigned long)g_rc.frame_count,
                      (int)RemoteControl_IsOnline(),
                      (int)RemoteControl_IsDoubleUp(),
                      (int)(RemoteControl_GetChannel(0) * 1000.0f),
                      (int)(RemoteControl_GetChannel(1) * 1000.0f),
                      (int)(RemoteControl_GetChannel(2) * 1000.0f),
                      (int)(g_rc.sw[0] * 10 + g_rc.sw[1]));
#endif

            Dbg_Print("%lu,%lu,%lu,%lu\n",
                      (unsigned long)(Can_RxCount[CAN_BUS_1] / 100u),
                      (unsigned long)(Can_TxCount / 100u),
                      (unsigned long)Can_TxErrCount,
                      (unsigned long)(psr & 0x7u));
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PRINT_PERIOD_MS));
    }
}
