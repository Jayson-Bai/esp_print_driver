#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
 * 第一阶段挤出控制接口边界：
 * 1. 上位机下发的是“累计绝对 E 位置”，不是本周期增量；后续绝对位置可覆盖此前
 *    中间报文丢失的影响，但永久丢失且没有后续报文的最后一个 E 无法由 MCU 恢复。
 * 2. 上位机在序列化之前已经丢失的 E 精度，MCU 无法反向恢复。
 * 3. 当前执行器是开环 STEP/DIR 控制；没有送料编码器时，只能核算已发出的脉冲，
 *    不能判断电机物理失步或耗材打滑。
 * 4. MCU 在本地完成使能、加减速、制动和换向，不等待上位机处理状态回报。
 */

/* 故障位可按位组合；是否阻断电机执行应读取状态中的 hard_fault_flags。 */
typedef enum {
    EXTRUDER_FAULT_NONE             = 0,       /* 无故障 */
    EXTRUDER_FAULT_UART_FIFO_OVF    = 1U << 0, /* UART 硬件 FIFO 溢出 */
    EXTRUDER_FAULT_UART_BUFFER_FULL = 1U << 1, /* UART 驱动接收缓冲区已满 */
    EXTRUDER_FAULT_LINE_QUEUE_FULL  = 1U << 2, /* 完整文本行队列已满 */
    EXTRUDER_FAULT_CMD_QUEUE_FULL   = 1U << 3, /* 已解析命令队列已满 */
    EXTRUDER_FAULT_RMT_SUBMIT       = 1U << 4, /* STEP 脉冲提交失败 */
    EXTRUDER_FAULT_RESET_TIMEOUT    = 1U << 5, /* 绝对位置复位屏障超时 */
    EXTRUDER_FAULT_RMT_DONE         = 1U << 6, /* STEP 脉冲完成回调异常 */
    EXTRUDER_FAULT_LAG_LIMIT        = 1U << 7, /* 目标与已提交步数长期偏差过大 */
    EXTRUDER_FAULT_PROTOCOL         = 1U << 8, /* 参数超出固件协议范围 */
} extruder_fault_t;

/* MCU 本地挤出运动状态；状态变化不需要上位机确认。 */
typedef enum {
    EXTRUDER_STATE_IDLE = 0,    /* 无待执行步数，空闲 */
    EXTRUDER_STATE_ENABLE_WAIT, /* 驱动器使能后的建立时间 */
    EXTRUDER_STATE_ACCEL,       /* 按本地加速度限制升速 */
    EXTRUDER_STATE_RUN,         /* 稳态运行 */
    EXTRUDER_STATE_DECEL,       /* 减速或反转前制动 */
    EXTRUDER_STATE_DIR_CHANGE,  /* 等待脉冲队列排空后安全换向 */
    EXTRUDER_STATE_HOLD,        /* 会话仍有效，当前位置保持 */
    EXTRUDER_STATE_FAULT,       /* 硬故障制动状态 */
    EXTRUDER_STATE_RESETTING,   /* 正在建立新的绝对位置原点 */
} extruder_state_t;

/* 状态快照中的步数均为有符号累计值，正负号对应送料/回抽方向。 */
typedef struct {
    int64_t target_steps;                /* 最新绝对目标折算后的步数 */
    int64_t submitted_steps;             /* 已提交给 RMT 的累计步数 */
    int64_t completed_steps;             /* RMT 已确认完成的累计步数 */
    int64_t host_total_um;                /* 上位机累计 E，单位为 10^-6 mm */
    int64_t last_command_us;              /* 最近 E 报文接收时刻，MCU 单调微秒时钟 */
    int64_t feedforward_valid_until_us;   /* 当前前馈速度估计的有效截止时刻 */
    float feedforward_steps_s;            /* 根据绝对 E 样本估算的前馈速度，step/s */
    float velocity_steps_s;               /* 当前带方向的规划速度，step/s */
    float phase;                          /* 尚不足一个整步的相位累计值 */
    uint32_t last_seq;                    /* 最近接收的上位机序号，仅用于估速和诊断 */
    uint32_t fault_flags;                 /* 全部累计故障位 */
    uint32_t hard_fault_flags;            /* 会使执行器制动并进入 FAULT 的故障位 */
    uint32_t rmt_submit_errors;            /* RMT 提交失败累计次数 */
    uint32_t rmt_done_errors;              /* RMT 完成回调异常累计次数 */
    uint32_t queue_depth;                  /* 当前尚未完成的 RMT 批次数 */
    int direction_sign;                   /* 当前物理方向：-1、0 或 +1 */
    bool enabled;                         /* 驱动器 EN 当前是否有效 */
    bool host_hold_active;                /* 串口绝对 E 会话是否仍保持驱动器 */
    bool host_sample_valid;               /* 是否已经收到可用的绝对 E 样本 */
    extruder_state_t state;               /* MCU 本地运动状态 */
} extruder_status_t;

/* 启动两个挤出器共用的本地规划任务；重复调用不会重复创建任务。 */
void extruder_task_start(void);

/*
 * 兼容旧 UI 的浮点入口。正式串口链路应优先使用下面的整数接口，避免累计 E
 * 在 float 转换中继续损失精度。
 */
void extruder_set_absolute(int tool_id, float total_mm);

/*
 * 设置累计绝对 E 目标。tool_id 只接受 1 或 2；total_um 的实际单位是
 * 10^-6 mm（沿用现有变量名，并非通常意义上的 1 微米）。seq 不作为丢弃命令的
 * 严格门禁，允许上位机暂停、去重、跳号或 uint32_t 回绕。
 */
bool extruder_set_absolute_um(int tool_id, int64_t total_um, uint32_t seq);

/*
 * 与 extruder_set_absolute_um() 相同，但由调用方提供完整报文到达时刻 rx_us；
 * rx_us 使用 esp_timer 单调微秒时钟，供 MCU 估算相邻绝对 E 的速度。
 */
bool extruder_set_absolute_um_at(int tool_id, int64_t total_um, uint32_t seq,
                                 int64_t rx_us);

/* 异步申请重置绝对位置账本；函数返回不表示复位已经完成。 */
void extruder_reset_absolute(int tool_id);

/*
 * 同步复位屏障：等待在途 RMT 脉冲结算后再建立新原点。返回 true 才表示完成；
 * 超时返回 false，并记录 RESET_TIMEOUT 硬故障，不能把超时当作复位成功。
 */
bool extruder_reset_absolute_wait(int tool_id, uint32_t timeout_ms);

/* UI 点动入口；dir_sign >= 0 为正向，dir_sign < 0 为反向。 */
void extruder_ui_start(int tool_id, int dir_sign);
/* 结束 UI 点动并按本地减速逻辑停车。正式串口 E 会自动取得 UI 点动所有权。 */
void extruder_ui_stop(int tool_id);

/*
 * 将 UART/队列链路故障按位记入两个挤出器状态；该通知仅记录本地状态，
 * 不依赖上位机读取或回传确认。
 */
void extruder_notify_transport_fault(uint32_t fault_flags);

/* 原子读取指定挤出器的状态快照；参数非法时返回 false。 */
bool extruder_get_status(int tool_id, extruder_status_t *status);

/* 仅表示本地任务、RMT 通道和编码器已初始化，不代表物理送料没有失步。 */
bool extruder_is_ready(int tool_id);
