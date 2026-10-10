/** ivision 风格定点控制（替代版头文件）
 *
 * 公开接口、状态枚举与原 position_control.h 一致，调用方无需改动；
 * 配置结构尾部追加了 10 个规划参数（菜单可改、Flash v3 保存），
 * 需与本文件夹内的 Mymenu.c / Flash.c / Flash.h 配套替换。
 * 区别：XY 不再使用位置反馈（原 xy_kp/xy_kd）与位置比例航向环（原 yaw_kp），
 * 改为 ivision 式开环速度规划（sqrt 制动曲线 + 斜坡 + 方向滤波）与
 * YawProfiled 航向规划（远端 sqrt / 近端线性带 / 陀螺阻尼），
 * 位置闭环职责下放给已有的四轮速度环。配置中的三个增益字段仅为
 * 兼容保留，运行时不读取。
 *
 * Goal XY uses the fixed Navigation/Zero frame; yaw is relative to that origin.
 * +X forward at Zero, +Y left, positive yaw counter-clockwise. */
#ifndef MCAR_POSITION_CONTROL_H
#define MCAR_POSITION_CONTROL_H

#include "navigation_fusion.h"

typedef struct {
    float x_cm, y_cm, yaw_deg;
} position_goal_t;

typedef struct {
    float xy_kp;             /* 未使用：ivision 式开环规划无位置比例，仅保留兼容 */
    float xy_kd;             /* 未使用：无位置速度阻尼项，仅保留兼容 */
    float yaw_kp;            /* 未使用：航向改为 sqrt 曲线+线性带，仅保留兼容 */
    float max_speed_cmps;    /* 平移最大速度 cm/s */
    float max_omega_radps;   /* 最大角速度 rad/s */
    float max_accel_cmps2;   /* 平移最大加速度（加速斜坡）cm/s² */
    float max_alpha_radps2;  /* 最大角加速度 rad/s² */
    float xy_tolerance_cm;   /* 位置到达容差，同时是弹簧静止死区半径 */
    float yaw_tolerance_deg; /* 航向到达容差，同时是航向规划死区 */
    /* ---- ivision 式规划参数（菜单 Position 页可改，随 SaveCfg 掉电保存） ---- */
    float brake_limit;       /* 刹车能力 = max_accel × 本比例 */
    float brake_ceiling_cmps2; /* 附着刹车上限 cm/s²，与上项取小 */
    float short_segment_cm;  /* 锁定段长 ≤ 该值 → 抬升起步加速斜坡 */
    float short_boost_gain;  /* 短段加速抬升倍率（只影响加速斜坡） */
    float approach_zone_cm;  /* 末端接近区上限 cm，0=关闭双段刹车 */
    float approach_ratio;    /* 接近区 ≤ 锁定段长×该比例 */
    float approach_acc_k;    /* 接近区刹车 = 刹车能力×该比例（更缓曲线） */
    float yaw_lin_band_rad;  /* 航向线性带半宽 rad，保证近端等效增益有界 */
    float yaw_kd;            /* 静止转向陀螺阻尼 */
    float yaw_kd_translate;  /* 平移时陀螺阻尼 */
} position_config_t;

#define POSITION_CONFIG_DEFAULT {2.0f, 0.2f, 2.0f, 20.0f, 1.0f, 40.0f, 2.0f, 2.0f, 3.0f, \
                                 0.90f, 450.0f, 40.0f, 1.40f, 10.0f, 0.25f, 0.45f, \
                                 0.22f, 0.22f, 0.36f}

enum {
    POSITION_BAD_CONFIG = -2,
    POSITION_NO_POSE = -1,
    POSITION_IDLE = 0,
    POSITION_MOVING = 1,
    POSITION_SETTLING = 2,
    POSITION_REACHED = 3
};

typedef struct {
    int32_t status;
    float distance_cm, yaw_error_deg;
    float vx_cmps, vy_cmps, omega_radps; /* effective Drive body command */
} position_output_t;

typedef struct {
    position_output_t output;
    position_goal_t previous_goal;
    float world_vx_cmps, world_vy_cmps; /* 一阶滤波后的世界系速度（ivision alpha=0.25） */
    float previous_yaw_deg, settled_seconds;
    bool have_previous;
    /* ivision 式规划器内部状态 */
    float plan_v_cmps;         /* 平移标量速度参考（Trajectory.current_v） */
    float plan_vw_radps;       /* 航向角速度参考（YawProfiled.current_vw） */
    float initial_distance_cm; /* 启动/换目标瞬间锁定的段长，用于短段提升与接近区比例 */
    bool have_initial;
} position_control_t;

void position_control_reset(position_control_t *control);
/* Pure controller, called once per encoder tick. Invalid input returns zero
 * command with a negative status. */
void position_control_update(position_control_t *control,
                             const position_config_t *config,
                             const position_goal_t *goal,
                             const navigation_snapshot_t *pose, float dt);

#endif
