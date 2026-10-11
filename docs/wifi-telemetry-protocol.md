# WiFi-SPI 遥测、上传变量订阅与滑杆调参

/**
 * @file path_follow.c
 * @brief 路径跟随与底盘速度规划实现文件�? *
 * 本文件负责将上层给出的路径点或拐点序列转换为底盘可执行的连续速度命令�? * 控制过程依次经过路径几何分析、标量速度规划、姿态控制和车体系速度合成�? * 当前实现兼容网格地图产生的横平竖直路径，并保留启动偏移、位姿校正等辅助动作接口�? */


#ifndef M_PI
#define M_PI 3.1415926
#endif


//#define YAW_ACCELERATE_FORWAROD_VALUE 0.0f

/** @brief 拐点/最终目标点前馈外推基础量，单位 m�?*/
#define PATH_TARGET_FEEDFORWARD_BASE_M 0.000f
/** @brief 拐点/最终目标点前馈外推段长增量系数，单�?m/m�?*/
#define PATH_TARGET_FEEDFORWARD_GAIN_PER_M 0.000f
/** @brief 拐点/最终目标点前馈外推上限，单�?m�?*/
#define PATH_TARGET_FEEDFORWARD_MAX_M 0.020f
/** @brief 拐点/最终目标点前馈外推占当前段长的最大比例�?*/
#define PATH_TARGET_FEEDFORWARD_SEGMENT_RATIO_MAX 0.25f
/** @brief 原地转向命中容差后，需要连续保持稳定的控制周期数�?*/
#define PATH_FOLLOW_ROTATE_SETTLE_MS 80U
#define PATH_FOLLOW_UPDATE_RATE_HZ MOTOR_ENCODER_SAMPLE_HZ
#define PATH_FOLLOW_UPDATE_DT_S MOTOR_ENCODER_SAMPLE_DT_S
#define PATH_FOLLOW_ROTATE_SETTLE_CYCLES ((PATH_FOLLOW_ROTATE_SETTLE_MS * PATH_FOLLOW_UPDATE_RATE_HZ + 999U) / 1000U)
#define PATH_FOLLOW_TERMINAL_YAW_TIMEOUT_MS 1000U
#define PATH_FOLLOW_TERMINAL_YAW_TIMEOUT_CYCLES ((PATH_FOLLOW_TERMINAL_YAW_TIMEOUT_MS * PATH_FOLLOW_UPDATE_RATE_HZ + 999U) / 1000U)
/* Final arrival requires yaw and all four wheel speeds to remain in tolerance continuously. */
#define PATH_FOLLOW_FINAL_REACHED_SETTLE_MS 100U
#define PATH_FOLLOW_FINAL_REACHED_SETTLE_CYCLES ((PATH_FOLLOW_FINAL_REACHED_SETTLE_MS * PATH_FOLLOW_UPDATE_RATE_HZ + 999U) / 1000U)
/** @brief 原地转向到位判定容差，单�?deg�?*/
#define PATH_FOLLOW_ROTATE_TOL_DEG 0.8f
#define PATH_FOLLOW_YAW_PID_DEADBAND_DEG 0.1f
/** @brief 姿态角速度输出最大变化率，单�?deg/s^2�?*/
#define PATH_FOLLOW_YAW_OMEGA_SLEW_DEGPS2 100000.0f
/** @brief IMU Z 轴角速度到世界航向正方向的符号修正�?*/
#define PATH_FOLLOW_GYRO_Z_TO_YAW_SIGN 1.0f

控制请求：ASCII，区分大小写。原有 SUB/RATE/STREAM/LIST?/GET? 每条以 LF `\n` 结尾，CRLF 也可；新增滑杆包以 `]` 结束，可不带换行。逐飞 SPI 接收是字节流，本实现接受分段与连续多条命令；建议在同一个 UDP 数据报中发送一条完整命令。命令正文最多 1023 字节，滑杆包长度包含左右方括号。单个配置操作串行等待应答，不要同时用多个客户端改变订阅。

#define PATH_DISTANCE_COMP_MIN_APPLY_M 0.0005f
#define PATH_DISTANCE_COMP_MIN_CMD_M 0.0001f

#define PATH_CROSS_AXIS_DISTANCE_COMP_ENABLE 1
#define PATH_CROSS_AXIS_COMP_MIN_APPLY_M 0.00005f
#define PATH_CROSS_AXIS_COMP_LIMIT_M 10000.000f
#define PATH_FOLLOW_COMP_TABLE_COUNT_OF(array) (sizeof(array) / sizeof((array)[0]))

/**
 * @brief 分段线性距离补偿表中的一个采样点�? *        distance_m：名义（目标）距离，单位 m�? *        error_cm：实测误差，单位 cm（正值表示走短，负值表示走多）�? */
/* 前进距离补偿：前移实测整体走长，因此主方向 error_cm 为负值。 */
static path_follow_distance_comp_sample_t g_path_comp_fwd_table[] = {
    {0.0f,   0.00f},
    {0.2f,  -3.10f},
    {0.4f,  -7.26f},
    {0.6f,  -10.57f},
    {0.8f,  -10.23f},
    {1.0f, -10.43f},
    {1.2f, -10.00f},
    {1.4f, -14.26f},
    {1.6f, -12.75f},
    {1.8f, -11.33f},
    {2.0f, -14.80f},
    {2.2f, -15.66f},
    {2.4f, -16.83f},
    {2.6f, -16.30f},
    {2.8f, -17.00f}
};

/* 后退距离补偿：后移实测整体走长，因此主方向 error_cm 为负值。 */
static path_follow_distance_comp_sample_t g_path_comp_back_table[] = {
    {0.0f,   0.00f},
    {0.2f,  -5.10f},
    {0.4f,  -9.00f},
    {0.6f, -10.80f},
    {0.8f, -12.43f},
    {1.0f, -13.90f},
    {1.2f, -15.36f},
    {1.4f, -15.62f},
    {1.6f, -12.23f},
    {1.8f, -12.72f},
    {2.0f, -14.95f},
    {2.2f, -14.97f},
    {2.4f, -15.40f},
    {2.6f, -15.63f},
    {2.8f, -17.40f}
};

/* 左移距离补偿：左移实测整体走短，因此主方向 error_cm 为正值。 */
static path_follow_distance_comp_sample_t g_path_comp_left_table[] = {
    {0.0f,   0.00f},
    {0.2f,   1.26f},
    {0.4f,   2.66f},
    {0.6f,   4.10f},
    {0.8f,   7.40f},
    {1.0f,   9.96f},
    {1.2f,  11.40f},
    {1.4f,  15.13f},
    {1.6f,  20.66f},
    {1.8f,  26.26f},
    {2.0f,  29.83f},
    {2.2f,  34.56f},
    {2.4f,  37.40f},
    {2.6f,  45.46f},
    {2.8f,  49.50f}
};

## 滑杆数据包：修改一个参数

只需要三个字段，UTF-8/ASCII 文本均可，但类型和参数名使用下面的 ASCII 名称：

```text
[slider,参数名,数值]
```

例如滑杆改变位置环 Kp：

```text
[slider,pos_xy_kp,2.5]
```

也接受带双引号的 JSON 数组，数值必须是数字，不能是字符串：

```json
["slider", "pos_xy_kp", 2.5]
```

三字段分别是控件类型 `slider`、参数名 `pos_xy_kp`、要写入的值 `2.5`。空格可以省略。以 `]` 立即结包，不需要额外发送 `\n`；同一接收流中可连续出现多包，也可和原有换行命令混用。解析仅支持这种简单数组，不支持嵌套数组、字符串转义、额外字段或任意内存地址。

成功后返回实际写入的 float32 值：

```text
MCAR SLIDER pos_xy_kp 2.5\n
```

失败返回 `MCAR ERR <原因>\n`，例如：

```text
MCAR ERR unknown parameter
MCAR ERR parameter out of range
MCAR ERR parameter requires Run off
MCAR ERR expected [slider,parameter,number]
```

每包修改一个参数，校验失败不修改。只更新 RAM，重启恢复启动配置。轮速 PID 修改会同时更新初始化参数和实际 PID，并清除该轮 PID 历史状态；重复发送同一值且运行参数已经一致时不重复清除。调参不启动电机，不改变模式。

### 已支持的参数名

| 参数名 | 范围 | 单位/作用 |
| --- | --- | --- |
| `pos_xy_kp` | 0.01～20 | 平移位置比例，1/s |
| `pos_xy_kd` | 0～5 | 平移速度阻尼系数 |
| `pos_yaw_kp` | 0.01～20 | 航向比例，1/s |
| `pos_max_speed_cmps` | 1～100 | 平移最大速度，cm/s |
| `pos_max_omega_radps` | 0.05～3 | 最大角速度，rad/s |
| `pos_max_accel_cmps2` | 1～300 | 平移最大加速度，cm/s² |
| `pos_max_alpha_radps2` | 0.05～10 | 最大角加速度，rad/s² |
| `pos_xy_tolerance_cm` | 0.5～20 | 位置到达容差，cm |
| `pos_yaw_tolerance_deg` | 0.5～20 | 航向到达容差，度 |
| `ul_kp,ul_ki,ul_kd` | 各 0～1000 | 左前轮速度 PID |
| `ur_kp,ur_ki,ur_kd` | 各 0～1000 | 右前轮速度 PID |
| `dl_kp,dl_ki,dl_kd` | 各 0～1000 | 左后轮速度 PID |
| `dr_kp,dr_ki,dr_kd` | 各 0～1000 | 右后轮速度 PID |
| `goal_x_cm,goal_y_cm` | 各 -10000～10000 | 目标坐标，cm；仅 Run=Off 可改 |
| `goal_yaw_deg` | -180～180 | 目标航向，度；仅 Run=Off 可改 |
| `scale_x,scale_y` | 各 0.01～10 | 定位比例；仅 Run=Off 可改，修改后定位会重新 Zero |

数值支持普通小数、负数和科学计数法，例如 `-125`、`2.5`、`1e-2`。NaN、Infinity、十六进制数、带引号的数值和超出范围的值不接受。实际应用值以应答为准。

新增参数在 `wifispi.c` 的 `g_slider_parameters` 中加一条映射即可，例如：

```c
SLIDER("my_gain", &my_gain, 0.0f, 10.0f, 0),
```

五项依次是上位机使用的名称、可写 `float` 变量地址、最小值、最大值、是否仅允许 Run=Off 写入。运行控制每 10 ms 从这些参数读取新值，赋值在短临界区内完成，网络收发不关中断。当前只支持 `slider` 控件类型。

上位机界面可直接将“滑杆对应的参数名”和“滑杆当前数值”组成上述数组发送到模块 IP:5001；接收 `MCAR SLIDER ...` 作为确认，不进入 JustFloat 绘图。现有 GUI 的原控制解析器尚不认识 `SLIDER` 应答，由你新增的调参界面接收处理即可。拖动时建议限频，释放滑杆时再发送最终值；UDP 无序号，固件按实际收到的顺序应用，必要时用已有同名遥测通道核对。

## 公开变量及单位

/* 前进/后退段的 Y 串轴补偿表，单位 cm；+Y 为左移，-Y 为右移。 */
static path_follow_distance_comp_sample_t g_path_cross_fwd_y_table[] = {
    {0.0f,   0.00f},
    {0.2f,   0.00f},
    {0.4f,   0.03f},
    {0.6f,  -0.37f},
    {0.8f,  -2.90f},
    {1.0f,  -1.33f},
    {1.2f,   0.06f},
    {1.4f,  -1.83f},
    {1.6f,  -4.80f},
    {1.8f,  -3.83f},
    {2.0f,  -8.10f},
    {2.2f,  -6.63f},
    {2.4f,   2.60f},
    {2.6f,  -3.26f},
    {2.8f,  -5.43f}
};

static path_follow_distance_comp_sample_t g_path_cross_back_y_table[] = {
    {0.0f,   0.00f},
    {0.2f,   0.00f},
    {0.4f,   0.16f},
    {0.6f,  -1.00f},
    {0.8f,   0.80f},
    {1.0f,   0.10f},
    {1.2f,   0.76f},
    {1.4f,   1.20f},
    {1.6f,   3.73f},
    {1.8f,   1.50f},
    {2.0f,   6.02f},
    {2.2f,   3.12f},
    {2.4f,   3.66f},
    {2.6f,   2.80f},
    {2.8f,   3.93f}
};

/* 左移/右移段的 X 串轴补偿表，单位 cm；+X 为前进，-X 为后退。 */
static path_follow_distance_comp_sample_t g_path_cross_left_x_table[] = {
    {0.0f,   0.00f},
    {0.2f,   0.00f},
    {0.4f,   2.73f},
    {0.6f,   5.33f},
    {0.8f,   8.36f},
    {1.0f,   7.56f},
    {1.2f,  11.83f},
    {1.4f,  10.70f},
    {1.6f,  13.06f},
    {1.8f,  14.40f},
    {2.0f,  19.66f},
    {2.2f,  25.33f},
    {2.4f,  20.86f},
    {2.6f,  25.63f},
    {2.8f,  28.10f}
};

static path_follow_distance_comp_sample_t g_path_cross_right_x_table[] = {
    {0.0f,   0.00f},
    {0.2f,  -1.67f},
    {0.4f,  -4.86f},
    {0.6f,  -6.66f},
    {0.8f,  -9.73f},
    {1.0f,  -17.03f},
    {1.2f,  -15.11f},
    {1.4f,  -13.36f},
    {1.6f,  -24.11f},
    {1.8f, -27.46f},
    {2.0f, -28.85f},
    {2.2f, -39.85f},
    {2.4f, -41.36f},
    {2.6f, -44.66f},
    {2.8f, -55.63f}
};

/** @brief 非终点路径段在段末保留的过渡速度，单�?cm/s�?*/
#define SCURVE_SEGMENT_END_SPEED_CMPS 0.0f
/** @brief S 曲线 band 0 的距离上限，单位 m�?*/
#define PATH_FOLLOW_SCURVE_BAND0_UPPER_M 0.30f
/** @brief S 曲线 band 1 的距离上限，单位 m�?*/
#define PATH_FOLLOW_SCURVE_BAND1_UPPER_M 0.50f
/** @brief S 曲线 band 2 的距离上限，单位 m�?*/
#define PATH_FOLLOW_SCURVE_BAND2_UPPER_M 0.70f
/** @brief S 曲线 band 3 的距离上限，单位 m�?*/
#define PATH_FOLLOW_SCURVE_BAND3_UPPER_M 0.90f
/** @brief S 曲线最后一�?band 的兜底距离上限，单位 m�?*/
#define PATH_FOLLOW_SCURVE_LAST_UPPER_M 1000.0f
/** @brief 当前未锁定任�?band 时的无效索引�?*/
#define PATH_FOLLOW_INVALID_SCURVE_BAND_IDX 0xFFU
/** @brief 启动偏移与定位修正临时路径的离散分辨率，单位 m�?*/
#define PRESTART_OFFSET_RESOLUTION_M 0.01f
#define PATH_FOLLOW_TEMP_POINT_INDEX_LIMIT_F 127.49f
/** @brief 拐点附近是否启用双轴 PID 保持修正�? 启用�? 关闭�?*/
#define PATH_FOLLOW_ENABLE_CORNER_DUAL_AXIS_TRIM 1
/** @brief 启用拐点 handover 时，拐点双轴保持修正的权重缩放系数�?*/
#define PATH_FOLLOW_CORNER_HOLD_TRIM_SCALE 0.50f
/** @brief 普通中间拐点是否启用速度向量 handover�? 启用�? 回退到原始简单段末速度保留�?*/
#define PATH_FOLLOW_ENABLE_CORNER_HANDOVER 0
/** @brief 线段投影到达终点的切向容差，单位 m�?*/
#define PATH_FOLLOW_SEGMENT_ALONG_TOL_M 0.008f
/** @brief 线段切换允许的横向误差容差，单位 m�?*/
#define PATH_FOLLOW_SEGMENT_LATERAL_TOL_M 0.008f
/** @brief 明显越过段末后允许兜底切段的超出量，单位 m�?*/
#define PATH_FOLLOW_SEGMENT_OVERRUN_TOL_M 0.050f
#define PATH_FOLLOW_DIAGONAL_MIN_OFF_AXIS_M 0.03f
#if PATH_FOLLOW_ENABLE_CORNER_HANDOVER
/* [CornerHandover重构] corner handover 进入距离占前后短段长度的比例�?*/
#define PATH_CORNER_HANDOVER_ENTER_RATIO 0.30f
/* [CornerHandover重构] corner handover 进入距离下限，单�?m�?*/
#define PATH_CORNER_HANDOVER_ENTER_MIN_M 0.03f
/* [CornerHandover重构] corner handover 进入距离上限，单�?m�?*/
#define PATH_CORNER_HANDOVER_ENTER_MAX_M 0.15f
/* [CornerHandover重构] 普通中间拐点允许正式切段的提前量比例�?*/
#define PATH_CORNER_HANDOVER_COMMIT_RATIO 0.12f
/* [CornerHandover重构] 普通中间拐点允许正式切段的提前量下限，单位 m�?*/
#define PATH_CORNER_HANDOVER_COMMIT_MIN_M 0.015f
/* [CornerHandover重构] 普通中间拐点允许正式切段的提前量上限，单位 m�?*/
#define PATH_CORNER_HANDOVER_COMMIT_MAX_M 0.03f
/* [CornerHandover重构] 保证 enter / commit 窗口之间至少留出的距离，单位 m�?*/
#define PATH_CORNER_HANDOVER_WINDOW_GAP_MIN_M 0.005f
/* [CornerHandover重构] 普通中间拐点正式切段时的最小横向误差门限，单位 m�?*/
#define PATH_CORNER_COMMIT_LATERAL_GATE_MIN_M 0.015f
/* [CornerHandover重构] 普通中间拐点正式切段时横向误差门限相对 pos_tol 的放大系数�?*/
#define PATH_CORNER_COMMIT_LATERAL_GATE_SCALE 1.20f
/* [CornerHandover重构] 交接中段对标量速度做轻微压低，避免拐点切向速度过硬�?*/
#define PATH_CORNER_HANDOVER_K_DROP 0.16f
/* [CornerHandover重构] 短段判定参考长度，取两格与固定阈值中的较大者，单位 m�?*/
#define PATH_CORNER_END_SPEED_SHORT_SEG_REF_M 0.20f
/* [CornerHandover重构] 短段场景下段末保留速度的最小长度缩放系数�?*/
#define PATH_CORNER_END_SPEED_SHORT_SEG_MIN_SCALE 0.60f
/* [CornerHandover重构] 90 度拐角相对默认段末速度的最大压低比例�?*/
#define PATH_CORNER_END_SPEED_TURN_DROP_90DEG 0.45f
/* [CornerHandover重构] 普通中间拐点段末保留速度的最小缩放系数�?*/
#define PATH_CORNER_END_SPEED_MIN_SCALE 0.30f
#endif

/**
 * @brief 世界坐标系下的二维位姿�? */
typedef struct
{
    float x_m;      // 世界坐标�?X，单�?m
    float y_m;      // 世界坐标�?Y，单�?m
    float yaw_deg;
} pose2d_t;

/**
 * @brief 姿态控制模式枚举�? */
typedef enum
{
    PATH_FOLLOW_HEADING_FIXED = 0
} path_follow_heading_mode_t;

typedef enum
{
    PATH_DISTANCE_COMP_DIR_FWD = 0,
    PATH_DISTANCE_COMP_DIR_BACK,
    PATH_DISTANCE_COMP_DIR_LEFT,
    PATH_DISTANCE_COMP_DIR_RIGHT
} path_follow_distance_comp_dir_t;

/**
 * @brief 速度规划参数集合�? */
typedef struct
{
    float segment_end_speed_cmps;   // 非终点路径段在段末保留的目标速度，单�?cm/s
} path_follow_speed_cfg_t;

/**
 * @brief 当前路径段锁定的 S 曲线 band 运行参数�? */
typedef struct
{
    uint8 band_idx;         // 当前路径段锁定的 band 索引
    float max_speed_cmps;   // 当前路径段锁定的最大速度，单�?cm/s
    float accel_cmpss;      // 当前路径段锁定的最大加速度，单�?cm/s^2
    float jerk_cmpsss;      // 当前路径段锁定的最�?jerk，单�?cm/s^3
} path_follow_scurve_runtime_cfg_t;

/**
 * @brief 路径几何层输出�? */
typedef struct
{
    Point target_point;
    float target_x_m;
    float target_y_m;
    float delta_x_m;       // 目标相对当前位置�?X 方向误差，单�?m
    float delta_y_m;       // 目标相对当前位置�?Y 方向误差，单�?m
    float distance_m;      // 当前目标点欧氏距离，单位 m
    float dir_x;           // 当前线段切向单位向量 X 分量
    float dir_y;           // 当前线段切向单位向量 Y 分量
    float segment_start_x_m;   // 当前线段起点 X 坐标，单�?m
    float segment_start_y_m;   // 当前线段起点 Y 坐标，单�?m
    float segment_dir_x;       // 当前线段切向单位向量 X 分量
    float segment_dir_y;       // 当前线段切向单位向量 Y 分量
    float segment_normal_x;    // 当前线段法向单位向量 X 分量
    float segment_normal_y;    // 当前线段法向单位向量 Y 分量
    float segment_length_m;    // 当前线段长度，单�?m
    float segment_progress_m;  // 当前位置在线段上的投影进度，单位 m
    float segment_remaining_m; // 当前线段切向剩余距离，单�?m
    float lateral_error_m;     // 当前位置到线段的有符号横向误差，单位 m
    uint8 segment_axis;
} path_follow_geometry_t;

typedef struct
{
    const Point *path;
    size_t target_idx;
    Point target_point;
    float start_x_m;
    float start_y_m;
    float target_x_m;
    float target_y_m;
    float segment_dir_x;
    float segment_dir_y;
    float segment_normal_x;
    float segment_normal_y;
    float segment_length_m;
    uint8 route_axis;
    uint8 active;
} path_follow_segment_lock_t;

/**
 * @brief 速度规划层输出�? */
typedef struct
{
    float safety_cap_cmps; // 仅用于异常保护的安全速度上限，单�?cm/s
    float end_speed_cmps;  // 当前段末端目标速度，单�?cm/s
    float ref_speed_cmps;  // 本周期最终标量速度参考，单位 cm/s
} path_follow_speed_plan_t;

/**
 * @brief 当前路径段的标准 7 �?jerk-limited S 曲线参数�? *
 * @note 本结构描述的是“一条路径段一次性规划完成”的完整 profile�? *       后续控制周期只按累计时间采样，不再滚动重建�? */
typedef struct
{
    float s;      // 当前路径段总位移，单位 cm
    float x0;     // 轨迹起点位置，单�?cm，当前实现固定为 0
    float x1;     // 轨迹终点位置，单�?cm
    float v0;     // 段起点速度，单�?cm/s
    float v1;     // 段终点速度，单�?cm/s
    float vmax;   // 配置给定的最大速度约束，单�?cm/s
    float amax;   // 本条轨迹最终采用的最大加速度，单�?cm/s^2
    float jmax;   // 本条轨迹采用的最�?jerk，单�?cm/s^3
    float vlim;   // 本条轨迹实际可达到的峰值速度，单�?cm/s
    float alima;  // 加速侧实际峰值加速度，单�?cm/s^2
    float alimd;  // 减速侧实际峰值减速度幅值，单位 cm/s^2
    float Tj1;    // 加速侧 jerk 作用时间，单�?s
    float Ta;     // 整个加速阶段时长，单位 s
    float Tv;     // 匀速阶段时长，单位 s
    float Tj2;    // 减速侧 jerk 作用时间，单�?s
    float Td;     // 整个减速阶段时长，单位 s
    float T;      // 轨迹总时长，单位 s
    uint8 valid;
} path_follow_scurve_profile_t;

/**
 * @brief 姿态层输出�? */
typedef struct
{
    float target_yaw_deg;   // 当前姿态层目标航向，单�?deg
    float omega_cmd_radps;  // 当前姿态层输出角速度，单�?rad/s
    float omega_ref_degps;
} path_follow_attitude_plan_t;

/**
 * @brief 车体运动命令缓存�? */
typedef struct
{
    float vx_world_cmps;  // 世界�?X 方向速度命令，单�?cm/s
    float vy_world_cmps;  // 世界�?Y 方向速度命令，单�?cm/s
    float vx_body_cmps;   // 车体�?X 方向速度命令，单�?cm/s
    float vy_body_cmps;   // 车体�?Y 方向速度命令，单�?cm/s
} path_follow_motion_cmd_t;

/**
 * @brief 调试状态快照�? */
typedef struct
{
    float distance_m;       // 当前目标距离，单�?m
    float dir_x;            // 当前线段切向单位向量 X 分量
    float dir_y;            // 当前线段切向单位向量 Y 分量
    float segment_start_x_m;
    float segment_start_y_m;
    float segment_dir_x;
    float segment_dir_y;
    float segment_normal_x;
    float segment_normal_y;
    float segment_length_m;
    float segment_progress_m;
    float segment_remaining_m;
    float lateral_error_m;
    float speed_ref_cmps;   // 当前标量速度参考，单位 cm/s
    float target_yaw_deg;   // 当前目标航向，单�?deg
    float omega_cmd_radps;  // 当前角速度输出，单�?rad/s
    float vx_world_cmps;    // 当前世界�?X 速度命令，单�?cm/s
    float vy_world_cmps;    // 当前世界�?Y 速度命令，单�?cm/s
    uint8 segment_axis;
    float omega_ref_degps;
} path_follow_debug_state_t;

/**
 * @brief 路径跟随模块运行上下文�? */
typedef struct
{
    const Point *path;     // 规划路径指针
    size_t steps;          // 路径长度
    size_t idx;
    float grid_m;
    float default_grid_m;
    float pulses_per_meter;
    float pos_tol_m;       // 位置容差
    float yaw_tol_deg;     // 原地转向到位容差
    float max_v_mps;       // 线速度上限 m/s
    float max_w_rad;       // 航向控制内部角速度上限，单�?deg/s
    float imu_to_world_yaw_offset_deg; // IMU 航向到世界航向的偏置
    float target_yaw_deg;
    uint8 heading_mode;
    pose2d_t pose;
    path_follow_speed_cfg_t speed_cfg;
    path_follow_scurve_runtime_cfg_t active_scurve_cfg; // 当前路径段锁定的 S 曲线参数
    path_follow_scurve_profile_t active_profile;// 当前目标段的整段�?S 曲线 profile
    path_follow_segment_lock_t segment_lock;
    path_follow_debug_state_t debug;
    path_follow_motion_telemetry_t telemetry;
    float profile_time_s;                       // 当前 profile 已执行的累计时间
    float last_ref_speed_cmps;
    float last_omega_cmd_degps;
    size_t profile_target_idx;                 // 当前 profile 绑定的目标点索引
    size_t pause_indices[PATH_FOLLOW_MAX_PAUSE_POINTS]; // 需要在到点后暂停的 corner 索引列表
    size_t pause_count;                        // pause_indices 中的有效元素个数
    size_t pause_cursor;                       // 下一个待触发的暂停点游标
    uint32 pause_cycles_cfg;                   // 每次暂停统一持续的控制周期数
    uint32 pause_cycles_remaining;
    uint8 pause_events_enabled;
    uint8 paused;
    uint8 rotate_only_active;
    uint8 rotate_hold_after_reach;
    uint8 rotate_in_tol_cycles;                // 原地转向连续命中容差的周期数
    uint8 terminal_yaw_settle_active;          // terminal yaw settle before final stop
    uint32 terminal_yaw_settle_cycles;
    uint32 terminal_yaw_in_tol_cycles;
    uint8 yaw_rate_only_active;                // 1: 当前仅执行固定航向角速度调试
    float yaw_rate_target_degps;               // 固定航向角速度调试目标，单�?deg/s
    uint8 motion_reached;
    uint8 profile_active;                      // 1: 当前段已有有�?profile
#if PATH_FOLLOW_ENABLE_CORNER_HANDOVER
    uint8 corner_handover_active;
    size_t corner_handover_idx;
    float corner_enter_distance_m;             // 当前拐点进入 handover 的距离阈值，单位 m
    float corner_commit_distance_m;            // 当前拐点允许正式切段的提前量，单�?m
#endif
    uint8 active;                              // 1: 正在跟随
} path_follow_ctx_t;

/** @brief 路径跟随模块全局上下文�?*/
static path_follow_ctx_t g_ctx = {0};
static path_follow_xy_remember_entry_t g_xy_remember[PATH_FOLLOW_XY_REMEMBER_CAPACITY] = {0};
static size_t g_xy_remember_count = 0U;
static size_t g_xy_remember_write_idx = 0U;

static void path_follow_invalidate_segment_lock(void)
{
    g_ctx.segment_lock = (path_follow_segment_lock_t){0};
}

static uint8 path_follow_comp_resolve_table(path_follow_comp_table_id_t table_id,
                                            path_follow_distance_comp_sample_t **table,
                                            size_t *count)
{
    if (table == NULL || count == NULL)
    {
        return 0U;
    }

    switch (table_id)
    {
    case PATH_FOLLOW_COMP_TABLE_MAIN_FWD:
        *table = g_path_comp_fwd_table;
        *count = PATH_FOLLOW_COMP_TABLE_COUNT_OF(g_path_comp_fwd_table);
        break;
    case PATH_FOLLOW_COMP_TABLE_MAIN_BACK:
        *table = g_path_comp_back_table;
        *count = PATH_FOLLOW_COMP_TABLE_COUNT_OF(g_path_comp_back_table);
        break;
    case PATH_FOLLOW_COMP_TABLE_MAIN_LEFT:
        *table = g_path_comp_left_table;
        *count = PATH_FOLLOW_COMP_TABLE_COUNT_OF(g_path_comp_left_table);
        break;
    case PATH_FOLLOW_COMP_TABLE_MAIN_RIGHT:
        *table = g_path_comp_right_table;
        *count = PATH_FOLLOW_COMP_TABLE_COUNT_OF(g_path_comp_right_table);
        break;
    case PATH_FOLLOW_COMP_TABLE_CROSS_FWD_Y:
        *table = g_path_cross_fwd_y_table;
        *count = PATH_FOLLOW_COMP_TABLE_COUNT_OF(g_path_cross_fwd_y_table);
        break;
    case PATH_FOLLOW_COMP_TABLE_CROSS_BACK_Y:
        *table = g_path_cross_back_y_table;
        *count = PATH_FOLLOW_COMP_TABLE_COUNT_OF(g_path_cross_back_y_table);
        break;
    case PATH_FOLLOW_COMP_TABLE_CROSS_LEFT_X:
        *table = g_path_cross_left_x_table;
        *count = PATH_FOLLOW_COMP_TABLE_COUNT_OF(g_path_cross_left_x_table);
        break;
    case PATH_FOLLOW_COMP_TABLE_CROSS_RIGHT_X:
        *table = g_path_cross_right_x_table;
        *count = PATH_FOLLOW_COMP_TABLE_COUNT_OF(g_path_cross_right_x_table);
        break;
    default:
        *table = NULL;
        *count = 0U;
        return 0U;
    }

    return 1U;
}

size_t path_follow_comp_sample_count(path_follow_comp_table_id_t table_id)
{
    path_follow_distance_comp_sample_t *table;
    size_t count;

    if (!path_follow_comp_resolve_table(table_id, &table, &count))
    {
        return 0U;
    }

    (void)table;
    return count;
}

uint8 path_follow_comp_get_sample(path_follow_comp_table_id_t table_id,
                                  size_t sample_index,
                                  path_follow_distance_comp_sample_t *sample)
{
    path_follow_distance_comp_sample_t *table;
    size_t count;

    if (sample == NULL ||
        !path_follow_comp_resolve_table(table_id, &table, &count) ||
        sample_index >= count)
    {
        return 0U;
    }

    *sample = table[sample_index];
    return 1U;
}

uint8 path_follow_comp_set_error_cm(path_follow_comp_table_id_t table_id,
                                    size_t sample_index,
                                    float error_cm)
{
    path_follow_distance_comp_sample_t *table;
    size_t count;

    if (!(error_cm >= -10000.0f && error_cm <= 10000.0f) ||
        !path_follow_comp_resolve_table(table_id, &table, &count) ||
        sample_index >= count)
    {
        return 0U;
    }

    table[sample_index].error_cm = error_cm;
    path_follow_invalidate_segment_lock();
    return 1U;
}

/* 仅在斜向运动时修正麦�?X/Y 耦合；纯 X/Y 保持原里程计比例�?*/
static float g_odom_diag_major_gain = 0.0f;
static float g_odom_diag_minor_gain = 0.0f;
static float g_cmd_diag_major_reduce_gain = 0.0f;
static float g_cmd_diag_minor_boost_gain = 0.0f;
/** @brief 默认�?S 曲线 band 配置表�?*/
static const path_follow_scurve_band_cfg_t g_path_follow_scurve_band_default_cfg[PATH_FOLLOW_SCURVE_BAND_COUNT] = {
    {PATH_FOLLOW_SCURVE_BAND0_UPPER_M, 4.50f, 8.00f, 15.00f},
    {PATH_FOLLOW_SCURVE_BAND1_UPPER_M, 4.50f, 8.00f, 15.00f},
    {PATH_FOLLOW_SCURVE_BAND2_UPPER_M, 4.50f, 8.00f, 15.00f},
    {PATH_FOLLOW_SCURVE_BAND3_UPPER_M, 4.50f, 8.00f, 15.00f},
    {PATH_FOLLOW_SCURVE_LAST_UPPER_M, 4.50f, 8.00f, 15.00f},
};
/** @brief 对外可调�?S 曲线 band 配置表�?*/

path_follow_scurve_band_cfg_t g_path_follow_scurve_band_cfg[PATH_FOLLOW_SCURVE_BAND_COUNT] = {
    {PATH_FOLLOW_SCURVE_BAND0_UPPER_M, 4.50f, 8.00f, 15.00f},
    {PATH_FOLLOW_SCURVE_BAND1_UPPER_M, 4.50f, 8.00f, 15.00f},
    {PATH_FOLLOW_SCURVE_BAND2_UPPER_M, 4.50f, 8.00f, 15.00f},
    {PATH_FOLLOW_SCURVE_BAND3_UPPER_M, 4.50f, 8.00f, 15.00f},
    {PATH_FOLLOW_SCURVE_LAST_UPPER_M, 4.50f, 8.00f, 15.00f},
};
/** @brief 历史 X 轴位置环 PID，当前主要保留参数与兼容接口�?*/
tagPID_T pid_world_x;
tagPID_T pid_stay;
/** @brief 航向控制 PID�?*/
tagPID_T pid_yaw;
/** @brief 预留的姿态扩�?PID�?*/
tagPID_T pid_accel_yaw;

/** @brief 历史位置环初始化参数；当前仅保留 X 轴兼容项�?*/
static PIDInitStruct pid_world_init;
static PIDInitStruct pid_stay_init;
/** @brief 航向 PID 初始化参数�?*/
static PIDInitStruct pid_yaw_init;
/** @brief 预留姿态扩�?PID 初始化参数�?*/
static PIDInitStruct pid_accel_yaw_init;
/** @brief 当前路径段主轴标记：0 停止�? X 段，2 Y 段，3 斜段�?*/
uint8 car_direction = 0;
/** @brief 外层保留的停车等待标志�?*/
uint8 wait_stop = 0;
/** @brief 发车前左移偏置量，单�?m�?*/
float prestart_move_left_m = 1.00f;
/** @brief 发车前右移偏置量，单�?m�?*/
float prestart_move_right_m = 0.00f;
/** @brief 发车前前移偏置量，单�?m�?*/
float prestart_move_forward_m = 0.00f;
/** @brief 发车前后移偏置量，单�?m�?*/
float prestart_move_backward_m = 0.00f;
/** @brief 单目标模式使用的两点临时路径缓存�?*/
static Point g_single_target_path[2];
/** @brief 发车前偏移动作使用的两点直线路径缓存�?*/
static Point g_prestart_offset_path[2];
/** @brief 位姿修正动作使用的临时路径缓存�?*/
static Point g_pose_correction_path[3];

/**
 * @brief 对输入值做对称限幅�? *
 * @param v 待限幅输入值�? * @param limit 正向与负向共用的限幅绝对值�? * @return 限幅后的结果�? */
static float clamp_sym(float v, float limit)
{
    if (v > limit)
    {
        return limit;
    }
    if (v < -limit)
    {
        return -limit;
    }
    return v;
}

static float path_follow_sign_nonzero(float value)
{
    return (value < 0.0f) ? -1.0f : 1.0f;
}

static float path_follow_actual_yaw_rate_degps(void)
{
    return PATH_FOLLOW_GYRO_Z_TO_YAW_SIGN *
           icm_data.gyro_z * (180.0f / (float)M_PI);
}

float path_follow_get_actual_yaw_rate_degps(void)
{
    return path_follow_actual_yaw_rate_degps();
}

float path_follow_get_yaw_rate_debug_target_degps(void)
{
    return g_ctx.yaw_rate_target_degps;
}

static float path_follow_apply_yaw_omega_slew(float target_omega_degps)
{
    float max_delta_degps = PATH_FOLLOW_YAW_OMEGA_SLEW_DEGPS2 * PATH_FOLLOW_UPDATE_DT_S;
    float prev_omega_degps;
    float delta_degps;
    float omega_degps;

    target_omega_degps = clamp_sym(target_omega_degps, g_ctx.max_w_rad);
    if (max_delta_degps <= 0.0f)
    {
        g_ctx.last_omega_cmd_degps = target_omega_degps;
        return target_omega_degps;
    }

    prev_omega_degps = clamp_sym(g_ctx.last_omega_cmd_degps, g_ctx.max_w_rad);
    delta_degps = target_omega_degps - prev_omega_degps;
    delta_degps = clamp_sym(delta_degps, max_delta_degps);
    omega_degps = clamp_sym(prev_omega_degps + delta_degps, g_ctx.max_w_rad);
    g_ctx.last_omega_cmd_degps = omega_degps;

    return omega_degps;
}

/**
 * @brief 将弧度归一化到 (-pi, pi] 区间�? *
 * @param rad 原始弧度值�? * @return 归一化后的弧度值�? */
static float wrap_pi(float rad)
{
    while (rad > (float)M_PI)
    {
        rad -= 2.0f * (float)M_PI;
    }
    while (rad < -(float)M_PI)
    {
        rad += 2.0f * (float)M_PI;
    }
    return rad;
}

static float path_follow_wrap_deg(float deg)
{
    while (deg > 180.0f)
    {
        deg -= 360.0f;
    }
    while (deg < -180.0f)
    {
        deg += 360.0f;
    }
    return deg;
}

static float path_follow_world_yaw_from_imu(float imu_yaw_deg)
{
    return path_follow_wrap_deg(path_follow_wrap_deg(imu_yaw_deg) +
                                g_ctx.imu_to_world_yaw_offset_deg);
}

static float path_follow_yaw_error_deg(float current_yaw_deg, float target_yaw_deg)
{
    return path_follow_wrap_deg(target_yaw_deg - current_yaw_deg);
}

/**
 * @brief �?m/s 转成 cm/s�? */
static float path_follow_mps_to_cmps(float value_mps)
{
    return value_mps * 100.0f;
}

/**
 * @brief �?m/s^2 转成 cm/s^2�? */
static float path_follow_mps2_to_cmpss(float value_mps2)
{
    return value_mps2 * 100.0f;
}

/**
 * @brief �?m/s^3 转成 cm/s^3�? */
static float path_follow_mps3_to_cmpsss(float value_mps3)
{
    return value_mps3 * 100.0f;
}

/**
 * @brief 复位 S 曲线 band 配置到默认值�? */
void path_follow_reset_scurve_band_defaults(void)
{
    uint8 i;

    for (i = 0U; i < PATH_FOLLOW_SCURVE_BAND_COUNT; ++i)
    {
        g_path_follow_scurve_band_cfg[i] = g_path_follow_scurve_band_default_cfg[i];
    }
}

/**
 * @brief 校正对外可调�?S 曲线 band 配置，非法值回退到默认值�? */
void path_follow_sanitize_scurve_band_cfg(void)
{
    uint8 i;

    for (i = 0U; i < PATH_FOLLOW_SCURVE_BAND_COUNT; ++i)
    {
        g_path_follow_scurve_band_cfg[i].distance_upper_m =
            g_path_follow_scurve_band_default_cfg[i].distance_upper_m;

        if (!(g_path_follow_scurve_band_cfg[i].vmax_mps > 0.0f))
        {
            g_path_follow_scurve_band_cfg[i].vmax_mps =
                g_path_follow_scurve_band_default_cfg[i].vmax_mps;
        }
        if (!(g_path_follow_scurve_band_cfg[i].amax_mps2 > 0.0f))
        {
            g_path_follow_scurve_band_cfg[i].amax_mps2 =
                g_path_follow_scurve_band_default_cfg[i].amax_mps2;
        }
        if (!(g_path_follow_scurve_band_cfg[i].jmax_mps3 > 0.0f))
        {
            g_path_follow_scurve_band_cfg[i].jmax_mps3 =
                g_path_follow_scurve_band_default_cfg[i].jmax_mps3;
        }
    }
}

/**
 * @brief 复位当前路径段锁定的 S 曲线 band 参数�? */
static void path_follow_reset_active_scurve_cfg(void)
{
    g_ctx.active_scurve_cfg.band_idx = PATH_FOLLOW_INVALID_SCURVE_BAND_IDX;
    g_ctx.active_scurve_cfg.max_speed_cmps = 0.0f;
    g_ctx.active_scurve_cfg.accel_cmpss = 0.0f;
    g_ctx.active_scurve_cfg.jerk_cmpsss = 0.0f;
}

/**
 * @brief 根据段长选择当前路径段应锁定�?S 曲线 band�? *
 * @param distance_m 当前路径段总长度，单位 m�? * @param runtime_cfg 选中的运行参数输出�? */
static void path_follow_select_scurve_band(float distance_m,
                                           path_follow_scurve_runtime_cfg_t *runtime_cfg)
{
    uint8 i;

    if (runtime_cfg == NULL)
    {
        return;
    }

    path_follow_sanitize_scurve_band_cfg();
    distance_m = fmaxf(distance_m, 0.0f);

    for (i = 0U; i < PATH_FOLLOW_SCURVE_BAND_COUNT; ++i)
    {
        if (distance_m <= g_path_follow_scurve_band_cfg[i].distance_upper_m)
        {
            runtime_cfg->band_idx = i;
            runtime_cfg->max_speed_cmps = path_follow_mps_to_cmps(g_path_follow_scurve_band_cfg[i].vmax_mps);
            runtime_cfg->accel_cmpss = path_follow_mps2_to_cmpss(g_path_follow_scurve_band_cfg[i].amax_mps2);
            runtime_cfg->jerk_cmpsss = path_follow_mps3_to_cmpsss(g_path_follow_scurve_band_cfg[i].jmax_mps3);
            return;
        }
    }

    runtime_cfg->band_idx = PATH_FOLLOW_SCURVE_BAND_COUNT - 1U;
    runtime_cfg->max_speed_cmps = path_follow_mps_to_cmps(g_path_follow_scurve_band_cfg[PATH_FOLLOW_SCURVE_BAND_COUNT - 1U].vmax_mps);
    runtime_cfg->accel_cmpss = path_follow_mps2_to_cmpss(g_path_follow_scurve_band_cfg[PATH_FOLLOW_SCURVE_BAND_COUNT - 1U].amax_mps2);
    runtime_cfg->jerk_cmpsss = path_follow_mps3_to_cmpsss(g_path_follow_scurve_band_cfg[PATH_FOLLOW_SCURVE_BAND_COUNT - 1U].jmax_mps3);
}

/**
 * @brief 根据当前上下文中的最大速度/角速度更新 PID 输出限幅�? *
 * @note 当前主链路未主动调用该函数，但保留它用于后续在线调参或恢复旧控制器时复用�? */
static void path_follow_update_pid_limits(void)
{
    // �?max_v/max_w 更新 PID 限幅
    pid_world_init.fMax_Iout = g_ctx.max_v_mps * 100.0f;
    pid_world_init.fMax_Out = g_ctx.max_v_mps * 100.0f;
    PID_Update(&pid_world_x, &pid_world_init);

    pid_yaw_init.fMax_Iout = g_ctx.max_w_rad;
    pid_yaw_init.fMax_Out = g_ctx.max_w_rad;
    PID_Update(&pid_yaw, &pid_yaw_init);

    pid_accel_yaw_init.fMax_Iout = g_ctx.max_w_rad;
    pid_accel_yaw_init.fMax_Out = g_ctx.max_w_rad;
    PID_Update(&pid_accel_yaw, &pid_accel_yaw_init);
}

/**
 * @brief 为单条路径段一次性构造标�?7 �?jerk-limited S 曲线 profile�? *
 * @param profile 输出 profile�? * @param distance_cm 当前路径段总位移，单位 cm�? * @param v0_cmps 起始速度，单�?cm/s�? * @param v1_cmps 末端速度，单�?cm/s�? * @param v_max_cmps 最大速度约束，单�?cm/s�? * @param a_max_cmpss 最大加速度约束，单�?cm/s^2�? * @param j_max_cmpsss 最�?jerk 约束，单�?cm/s^3�? * @return `1` 表示构造成功，`0` 表示在给定边界条件和约束下当前段不可行�? */
static uint8 path_follow_build_scurve_profile(path_follow_scurve_profile_t *profile,
                                              float distance_cm,
                                              float v0_cmps,
                                              float v1_cmps,
                                              float v_max_cmps,
                                              float a_max_cmpss,
                                              float j_max_cmpsss)
{
    float T1;
    float T2;
    float Tjs;
    float Tj1;
    float Tj2;
    float Ta;
    float Td;
    float Tv;
    float Tj;
    float delta;
    float a_work;

    if (profile == NULL)
    {
        return 0U;
    }

    *profile = (path_follow_scurve_profile_t){0};

    if (distance_cm <= 0.0f ||
        v_max_cmps <= 0.0f ||
        a_max_cmpss <= 0.0f ||
        j_max_cmpsss <= 0.0f)
    {
        return 0U;
    }

    v0_cmps = fminf(fmaxf(v0_cmps, 0.0f), v_max_cmps);
    v1_cmps = fminf(fmaxf(v1_cmps, 0.0f), v_max_cmps);

    T1 = sqrtf(fabsf(v1_cmps - v0_cmps) / j_max_cmpsss);
    T2 = v_max_cmps / j_max_cmpsss;
    Tjs = fminf(T1, T2);

    if ((T1 <= T2 && distance_cm < (Tjs * (v0_cmps + v1_cmps))) ||
        (T1 > T2 && distance_cm < (0.5f * (v0_cmps + v1_cmps) *
                                   (Tjs + fabsf(v1_cmps - v0_cmps) / a_max_cmpss))))
    {
        return 0U;
    }

    if ((v_max_cmps - v0_cmps) * j_max_cmpsss < (a_max_cmpss * a_max_cmpss))
    {
        Tj1 = sqrtf(fmaxf(v_max_cmps - v0_cmps, 0.0f) / j_max_cmpsss);
        Ta = 2.0f * Tj1;
        profile->alima = j_max_cmpsss * Tj1;
    }
    else
    {
        Tj1 = a_max_cmpss / j_max_cmpsss;
        Ta = Tj1 + (v_max_cmps - v0_cmps) / a_max_cmpss;
        profile->alima = a_max_cmpss;
    }

    if ((v_max_cmps - v1_cmps) * j_max_cmpsss < (a_max_cmpss * a_max_cmpss))
    {
        Tj2 = sqrtf(fmaxf(v_max_cmps - v1_cmps, 0.0f) / j_max_cmpsss);
        Td = 2.0f * Tj2;
        profile->alimd = j_max_cmpsss * Tj2;
    }
    else
    {
        Tj2 = a_max_cmpss / j_max_cmpsss;
        Td = Tj2 + (v_max_cmps - v1_cmps) / a_max_cmpss;
        profile->alimd = a_max_cmpss;
    }

    Tv = distance_cm / v_max_cmps -
         Ta * 0.5f * (1.0f + v0_cmps / v_max_cmps) -
         Td * 0.5f * (1.0f + v1_cmps / v_max_cmps);
    if (Tv > 0.0f)
    {
        profile->s = distance_cm;
        profile->x0 = 0.0f;
        profile->x1 = distance_cm;
        profile->v0 = v0_cmps;
        profile->v1 = v1_cmps;
        profile->vmax = v_max_cmps;
        profile->amax = a_max_cmpss;
        profile->jmax = j_max_cmpsss;
        profile->vlim = v_max_cmps;
        profile->Tj1 = Tj1;
        profile->Tj2 = Tj2;
        profile->Ta = Ta;
        profile->Td = Td;
        profile->Tv = Tv;
        profile->T = Ta + Tv + Td;
        profile->valid = 1U;
        return 1U;
    }

    Tv = 0.0f;
    Tj = a_max_cmpss / j_max_cmpsss;
    Tj1 = Tj;
    Tj2 = Tj;
    delta = powf(a_max_cmpss, 4) / powf(j_max_cmpsss, 2) +
            2.0f * (v0_cmps * v0_cmps + v1_cmps * v1_cmps) +
            a_max_cmpss * (4.0f * distance_cm -
                           2.0f * a_max_cmpss / j_max_cmpsss * (v0_cmps + v1_cmps));
    delta = fmaxf(delta, 0.0f);
    Ta = (powf(a_max_cmpss, 2) / j_max_cmpsss - 2.0f * v0_cmps + sqrtf(delta)) /
         (2.0f * a_max_cmpss);
    Td = (powf(a_max_cmpss, 2) / j_max_cmpsss - 2.0f * v1_cmps + sqrtf(delta)) /
         (2.0f * a_max_cmpss);
    if (Ta > 2.0f * Tj && Td > 2.0f * Tj)
    {
        profile->s = distance_cm;
        profile->x0 = 0.0f;
        profile->x1 = distance_cm;
        profile->v0 = v0_cmps;
        profile->v1 = v1_cmps;
        profile->vmax = v_max_cmps;
        profile->amax = a_max_cmpss;
        profile->jmax = j_max_cmpsss;
        profile->Tj1 = Tj1;
        profile->Tj2 = Tj2;
        profile->Ta = Ta;
        profile->Td = Td;
        profile->Tv = Tv;
        profile->T = Ta + Td;
        profile->alima = a_max_cmpss;
        profile->alimd = a_max_cmpss;
        profile->vlim = v0_cmps + (Ta - Tj1) * profile->alima;
        profile->valid = 1U;
        return 1U;
    }

    a_work = a_max_cmpss;
    while (Ta < 2.0f * Tj || Td < 2.0f * Tj)
    {
        if (Ta > 0.0f && Td > 0.0f)
        {
            a_work *= 0.99f;
            if (a_work <= 0.0f)
            {
                return 0U;
            }

            Tj = a_work / j_max_cmpsss;
            Tj1 = Tj;
            Tj2 = Tj;
            delta = powf(a_work, 4) / powf(j_max_cmpsss, 2) +
                    2.0f * (v0_cmps * v0_cmps + v1_cmps * v1_cmps) +
                    a_work * (4.0f * distance_cm -
                              2.0f * a_work / j_max_cmpsss * (v0_cmps + v1_cmps));
            delta = fmaxf(delta, 0.0f);
            Ta = (powf(a_work, 2) / j_max_cmpsss - 2.0f * v0_cmps + sqrtf(delta)) /
                 (2.0f * a_work);
            Td = (powf(a_work, 2) / j_max_cmpsss - 2.0f * v1_cmps + sqrtf(delta)) /
                 (2.0f * a_work);
            continue;
        }

        if ((v0_cmps + v1_cmps) <= 0.0f)
        {
            return 0U;
        }

        if (Ta <= 0.0f)
        {
            float numer;
            float denom;
            Ta = 0.0f;
            Tj1 = 0.0f;
            Td = 2.0f * distance_cm / (v0_cmps + v1_cmps);
            numer = j_max_cmpsss * distance_cm -
                    sqrtf(fmaxf(j_max_cmpsss * (j_max_cmpsss * distance_cm * distance_cm +
                                                (v1_cmps + v0_cmps) * (v1_cmps + v0_cmps) *
                                                (v1_cmps - v0_cmps)),
                                0.0f));
            denom = j_max_cmpsss * (v1_cmps + v0_cmps);
            if (fabsf(denom) <= 1e-6f)
            {
                return 0U;
            }
            Tj2 = numer / denom;
        }
        else
        {
            float numer;
            float denom;
            Td = 0.0f;
            Tj2 = 0.0f;
            Ta = 2.0f * distance_cm / (v0_cmps + v1_cmps);
            numer = j_max_cmpsss * distance_cm -
                    sqrtf(fmaxf(j_max_cmpsss * (j_max_cmpsss * distance_cm * distance_cm -
                                                (v1_cmps + v0_cmps) * (v1_cmps + v0_cmps) *
                                                (v1_cmps - v0_cmps)),
                                0.0f));
            denom = j_max_cmpsss * (v1_cmps + v0_cmps);
            if (fabsf(denom) <= 1e-6f)
            {
                return 0U;
            }
            Tj1 = numer / denom;
        }

        profile->s = distance_cm;
        profile->x0 = 0.0f;
        profile->x1 = distance_cm;
        profile->v0 = v0_cmps;
        profile->v1 = v1_cmps;
        profile->vmax = v_max_cmps;
        profile->amax = a_work;
        profile->jmax = j_max_cmpsss;
        profile->Tj1 = Tj1;
        profile->Tj2 = Tj2;
        profile->Ta = Ta;
        profile->Td = Td;
        profile->Tv = 0.0f;
        profile->T = Ta + Td;
        profile->alima = j_max_cmpsss * Tj1;
        profile->alimd = j_max_cmpsss * Tj2;
        profile->vlim = v0_cmps + (Ta - Tj1) * profile->alima;
        profile->valid = 1U;
        return 1U;
    }

    profile->s = distance_cm;
    profile->x0 = 0.0f;
    profile->x1 = distance_cm;
    profile->v0 = v0_cmps;
    profile->v1 = v1_cmps;
    profile->vmax = v_max_cmps;
    profile->amax = a_work;
    profile->jmax = j_max_cmpsss;
    profile->Tj1 = Tj1;
    profile->Tj2 = Tj2;
    profile->Ta = Ta;
    profile->Td = Td;
    profile->Tv = 0.0f;
    profile->T = Ta + Td;
    profile->alima = j_max_cmpsss * Tj1;
    profile->alimd = j_max_cmpsss * Tj2;
    profile->vlim = v0_cmps + (Ta - Tj1) * profile->alima;
    profile->valid = 1U;
    return 1U;
}

static uint8 path_follow_target_requires_pause(size_t target_idx);
static uint8 path_follow_is_route_corner_target(void);
static uint8 path_follow_get_segment_points(const Point **prev_point,
                                            const Point **curr_point,
                                            const Point **next_point);
#if PATH_FOLLOW_ENABLE_CORNER_HANDOVER
static void path_follow_reset_corner_handover_state(void);
static uint8 path_follow_compute_segment_progress_m(const pose2d_t *pose,
                                                    const Point *seg_start,
                                                    const Point *seg_end,
                                                    float *progress_m,
                                                    float *segment_length_m,
                                                    float *normal_error_m);
static float path_follow_compute_corner_enter_distance_m(float prev_length_m,
                                                         float next_length_m);
static float path_follow_compute_corner_commit_distance_m(float prev_length_m,
                                                          float next_length_m);
static float path_follow_compute_corner_end_speed_cmps(void);
#endif

/**
 * @brief 按当前路径段的累计执行时间采样标�?S 曲线目标速度�? *
 * @param t_s 从当�?profile 起点开始累计的全局时间，单�?s�? * @param profile �?`path_follow_build_scurve_profile()` 构造的 profile�? * @return 当前采样时刻的目标速度，单�?cm/s�? */
static float path_follow_sample_scurve_velocity(float t_s,
                                                const path_follow_scurve_profile_t *profile)
{
    if (profile == NULL || !profile->valid)
    {
        return 0.0f;
    }
    if (t_s <= 0.0f)
    {
        return profile->v0;
    }
    if (t_s >= profile->T)
    {
        return profile->v1;
    }

    if (t_s < profile->Tj1)
    {
        return profile->v0 + profile->jmax * t_s * t_s * 0.5f;
    }
    if (t_s < (profile->Ta - profile->Tj1))
    {
        return profile->v0 + profile->alima * (t_s - profile->Tj1 * 0.5f);
    }
    if (t_s < profile->Ta)
    {
        float dt = profile->Ta - t_s;
        return profile->vlim - profile->jmax * dt * dt * 0.5f;
    }
    if (t_s < (profile->Ta + profile->Tv))
    {
        return profile->vlim;
    }
    if (t_s < (profile->T - profile->Td + profile->Tj2))
    {
        float dt = t_s - profile->T + profile->Td;
        return profile->vlim - profile->jmax * dt * dt * 0.5f;
    }
    if (t_s < (profile->T - profile->Tj2))
    {
        return profile->vlim - profile->alimd *
               (t_s - profile->T + profile->Td - profile->Tj2 * 0.5f);
    }

    {
        float dt = profile->T - t_s;
        return profile->v1 + profile->jmax * dt * dt * 0.5f;
    }
}

/**
 * @brief 获取当前目标相关的前一角点、当前角点和后一角点�? *
 * @note 若某一侧不存在，对应输出会被置�?NULL�? *
 * @param prev_point 前一角点输出�? * @param curr_point 当前目标角点输出�? * @param next_point 后一角点输出�? * @return `1` 表示当前至少存在有效 curr_point，`0` 表示当前路径上下文无效�? */
static uint8 path_follow_get_segment_points(const Point **prev_point,
                                            const Point **curr_point,
                                            const Point **next_point)
{
    if (prev_point != NULL)
    {
        *prev_point = NULL;
    }
    if (curr_point != NULL)
    {
        *curr_point = NULL;
    }
    if (next_point != NULL)
    {
        *next_point = NULL;
    }

    if (g_ctx.path == NULL || g_ctx.idx >= g_ctx.steps)
    {
        return 0U;
    }

    if (prev_point != NULL && g_ctx.idx > 0U)
    {
        *prev_point = &g_ctx.path[g_ctx.idx - 1U];
    }
    if (curr_point != NULL)
    {
        *curr_point = &g_ctx.path[g_ctx.idx];
    }
    if (next_point != NULL && (g_ctx.idx + 1U) < g_ctx.steps)
    {
        *next_point = &g_ctx.path[g_ctx.idx + 1U];
    }

    return 1U;
}

#if PATH_FOLLOW_ENABLE_CORNER_HANDOVER
/**
 * @brief 计算当前位置�?prev->curr 段上的投影进度、段长和横向误差�? *
 * @param pose 当前位姿�? * @param seg_start 当前段起�?prev�? * @param seg_end 当前段终�?curr�? * @param progress_m 输出当前投影进度 s，单�?m�? * @param segment_length_m 输出当前段长�?L，单�?m�? * @param normal_error_m 输出当前横向误差 e_n，单�?m�? * @return `1` 表示计算成功，`0` 表示输入非法或段长退化�? */
static uint8 path_follow_compute_segment_progress_m(const pose2d_t *pose,
                                                    const Point *seg_start,
                                                    const Point *seg_end,
                                                    float *progress_m,
                                                    float *segment_length_m,
                                                    float *normal_error_m)
{
    float start_x_m;
    float start_y_m;
    float end_x_m;
    float end_y_m;
    float seg_dx_m;
    float seg_dy_m;
    float seg_norm_m;
    float seg_dir_x;
    float seg_dir_y;
    float rel_x_m;
    float rel_y_m;

    if (progress_m != NULL)
    {
        *progress_m = 0.0f;
    }
    if (segment_length_m != NULL)
    {
        *segment_length_m = 0.0f;
    }
    if (normal_error_m != NULL)
    {
        *normal_error_m = 0.0f;
    }

    if (pose == NULL || seg_start == NULL || seg_end == NULL ||
        progress_m == NULL || segment_length_m == NULL || normal_error_m == NULL)
    {
        return 0U;
    }

    start_x_m = seg_start->row * g_ctx.grid_m;
    start_y_m = seg_start->col * g_ctx.grid_m;
    end_x_m = seg_end->row * g_ctx.grid_m;
    end_y_m = seg_end->col * g_ctx.grid_m;
    seg_dx_m = end_x_m - start_x_m;
    seg_dy_m = end_y_m - start_y_m;
    seg_norm_m = sqrtf(seg_dx_m * seg_dx_m + seg_dy_m * seg_dy_m);
    if (seg_norm_m <= 1e-6f)
    {
        return 0U;
    }

    seg_dir_x = seg_dx_m / seg_norm_m;
    seg_dir_y = seg_dy_m / seg_norm_m;
    rel_x_m = pose->x_m - start_x_m;
    rel_y_m = pose->y_m - start_y_m;

    *progress_m = rel_x_m * seg_dir_x + rel_y_m * seg_dir_y;
    *segment_length_m = seg_norm_m;
    *normal_error_m = rel_x_m * (-seg_dir_y) + rel_y_m * seg_dir_x;
    return 1U;
}

/**
 * @brief 根据前后段较短长度计算进�?handover 的距离阈值�? *
 * @param prev_length_m 前一段长度，单位 m�? * @param next_length_m 后一段长度，单位 m�? * @return 建议�?handover 进入距离，单�?m�? */
static float path_follow_compute_corner_enter_distance_m(float prev_length_m,
                                                         float next_length_m)
{
    float min_length_m = fmaxf(fminf(prev_length_m, next_length_m), 0.0f);
    float enter_distance_m = PATH_CORNER_HANDOVER_ENTER_RATIO * min_length_m;

    enter_distance_m = fmaxf(enter_distance_m, PATH_CORNER_HANDOVER_ENTER_MIN_M);
    enter_distance_m = fminf(enter_distance_m, PATH_CORNER_HANDOVER_ENTER_MAX_M);
    return enter_distance_m;
}

/**
 * @brief 根据前后段较短长度计算普通中间拐点允许正式切段的提前量�? *
 * @param prev_length_m 前一段长度，单位 m�? * @param next_length_m 后一段长度，单位 m�? * @return 建议的正式切段距离，单位 m�? */
static float path_follow_compute_corner_commit_distance_m(float prev_length_m,
                                                          float next_length_m)
{
    float min_length_m = fmaxf(fminf(prev_length_m, next_length_m), 0.0f);
    float commit_distance_m = PATH_CORNER_HANDOVER_COMMIT_RATIO * min_length_m;

    commit_distance_m = fmaxf(commit_distance_m, PATH_CORNER_HANDOVER_COMMIT_MIN_M);
    commit_distance_m = fminf(commit_distance_m, PATH_CORNER_HANDOVER_COMMIT_MAX_M);
    return commit_distance_m;
}

/**
 * @brief 按转角与前后段长度估算普通中间拐点的段末保留速度�? *
 * @note 最终点�?pause 点仍由上层逻辑返回 0；这里仅针对普通中间拐点给出通过速度�? *
 * @return 当前普通中间拐点建议保留的段末速度，单�?cm/s�? */
static float path_follow_compute_corner_end_speed_cmps(void)
{
    const Point *prev_point = NULL;
    const Point *curr_point = NULL;
    const Point *next_point = NULL;
    float prev_dx_m;
    float prev_dy_m;
    float next_dx_m;
    float next_dy_m;
    float prev_length_m;
    float next_length_m;
    float min_length_m;
    float prev_dir_x;
    float prev_dir_y;
    float next_dir_x;
    float next_dir_y;
    float turn_cos;
    float turn_severity;
    float base_speed_cmps;
    float short_seg_ref_m;
    float length_scale;
    float angle_scale;
    float min_speed_cmps;
    float end_speed_cmps;

    if ((g_ctx.idx + 1U) >= g_ctx.steps)
    {
        return 0.0f;
    }
    if (path_follow_target_requires_pause(g_ctx.idx))
    {
        return 0.0f;
    }
    if (!path_follow_is_route_corner_target())
    {
        return g_ctx.speed_cfg.segment_end_speed_cmps;
    }
    if (!path_follow_get_segment_points(&prev_point, &curr_point, &next_point) ||
        prev_point == NULL || curr_point == NULL || next_point == NULL)
    {
        return g_ctx.speed_cfg.segment_end_speed_cmps;
    }

    prev_dx_m = (curr_point->row - prev_point->row) * g_ctx.grid_m;
    prev_dy_m = (curr_point->col - prev_point->col) * g_ctx.grid_m;
    next_dx_m = (next_point->row - curr_point->row) * g_ctx.grid_m;
    next_dy_m = (next_point->col - curr_point->col) * g_ctx.grid_m;
    prev_length_m = sqrtf(prev_dx_m * prev_dx_m + prev_dy_m * prev_dy_m);
    next_length_m = sqrtf(next_dx_m * next_dx_m + next_dy_m * next_dy_m);
    if (prev_length_m <= 1e-6f || next_length_m <= 1e-6f)
    {
        return g_ctx.speed_cfg.segment_end_speed_cmps;
    }

    prev_dir_x = prev_dx_m / prev_length_m;
    prev_dir_y = prev_dy_m / prev_length_m;
    next_dir_x = next_dx_m / next_length_m;
    next_dir_y = next_dy_m / next_length_m;
    turn_cos = prev_dir_x * next_dir_x + prev_dir_y * next_dir_y;
    turn_cos = fminf(fmaxf(turn_cos, -1.0f), 1.0f);
    /* 直行时为 0�?0 度及以上拐角�?1 处理�?*/
    turn_severity = fminf(fmaxf(1.0f - turn_cos, 0.0f), 1.0f);

    base_speed_cmps = fmaxf(g_ctx.speed_cfg.segment_end_speed_cmps, 0.0f);
    if (base_speed_cmps <= 0.0f)
    {
        return 0.0f;
    }

    min_length_m = fminf(prev_length_m, next_length_m);
    short_seg_ref_m = fmaxf(PATH_CORNER_END_SPEED_SHORT_SEG_REF_M,
                            g_ctx.default_grid_m * 2.0f);
    length_scale = min_length_m / short_seg_ref_m;
    length_scale = fminf(fmaxf(length_scale, PATH_CORNER_END_SPEED_SHORT_SEG_MIN_SCALE), 1.0f);
    angle_scale = 1.0f - PATH_CORNER_END_SPEED_TURN_DROP_90DEG * turn_severity;
    min_speed_cmps = base_speed_cmps * PATH_CORNER_END_SPEED_MIN_SCALE;

    end_speed_cmps = base_speed_cmps * angle_scale * length_scale;
    end_speed_cmps = fmaxf(end_speed_cmps, min_speed_cmps);
    end_speed_cmps = fminf(end_speed_cmps, base_speed_cmps);
    return end_speed_cmps;
}
#endif

/**
 * @brief 计算当前路径段末端希望保留的过渡速度�? *
 * @return 当前段末端目标速度，单�?cm/s�? */
static float path_follow_compute_segment_end_speed(void)
{
    if ((g_ctx.idx + 1U) >= g_ctx.steps)
    {
        return 0.0f;
    }
    if (path_follow_target_requires_pause(g_ctx.idx))
    {
        return 0.0f;
    }

#if PATH_FOLLOW_ENABLE_CORNER_HANDOVER
    if (path_follow_is_route_corner_target())
    {
        return path_follow_compute_corner_end_speed_cmps();
    }
#endif

    return g_ctx.speed_cfg.segment_end_speed_cmps;
}

/**
 * @brief 根据当前剩余距离计算一个仅用于异常保护的安全速度上限�? *
 * @param distance_m 当前段剩余距离，单位 m�? * @param end_speed_cmps 当前段末端目标速度，单�?cm/s�? * @param max_speed_cmps 当前路径段锁定的最大速度，单�?cm/s�? * @param accel_cmpss 当前路径段锁定的最大加速度，单�?cm/s^2�? * @return 按最大加速度约束换算得到的安全速度上限，单�?cm/s�? */
static float path_follow_compute_brake_speed_cap(float distance_m,
                                                 float end_speed_cmps,
                                                 float max_speed_cmps,
                                                 float accel_cmpss)
{
    float distance_cm = fmaxf(distance_m, 0.0f) * 100.0f;
    accel_cmpss = fmaxf(accel_cmpss, 1.0f);
    float speed_sq = end_speed_cmps * end_speed_cmps +
                     2.0f * accel_cmpss * distance_cm;

    return fminf(max_speed_cmps,
                 sqrtf(fmaxf(speed_sq, 0.0f)));
}

/**
 * @brief �?profile 构造失败时，用单拍加速度限幅生成保守速度作为异常保护�? *
 * @param current_speed_cmps 当前参考速度，单�?cm/s�? * @param safety_speed_cmps 当前允许的安全速度上限，单�?cm/s�? * @param accel_cmpss 当前路径段锁定的最大加速度，单�?cm/s^2�? * @return 下一拍的速度参考，单位 cm/s�? */
static float path_follow_compute_profile_fault_speed(float current_speed_cmps,
                                                     float safety_speed_cmps,
                                                     float accel_cmpss)
{
    float accel_step = fmaxf(accel_cmpss, 1.0f) * PATH_FOLLOW_UPDATE_DT_S;

    if (current_speed_cmps < safety_speed_cmps)
    {
        current_speed_cmps += accel_step;
        if (current_speed_cmps > safety_speed_cmps)
        {
            current_speed_cmps = safety_speed_cmps;
        }
    }
    else
    {
        current_speed_cmps -= accel_step;
        if (current_speed_cmps < safety_speed_cmps)
        {
            current_speed_cmps = safety_speed_cmps;
        }
    }

    return fmaxf(current_speed_cmps, 0.0f);
}

/**
 * @brief 清空对外可见的调试状态缓存�? *
 * @note 该函数不会影响路径、位姿和 PID，仅清除状态查询与屏显用的调试量�? */
static void path_follow_clear_debug_state(void)
{
    g_ctx.debug.distance_m = 0.0f;
    g_ctx.debug.dir_x = 0.0f;
    g_ctx.debug.dir_y = 0.0f;
    g_ctx.debug.segment_start_x_m = 0.0f;
    g_ctx.debug.segment_start_y_m = 0.0f;
    g_ctx.debug.segment_dir_x = 0.0f;
    g_ctx.debug.segment_dir_y = 0.0f;
    g_ctx.debug.segment_normal_x = 0.0f;
    g_ctx.debug.segment_normal_y = 0.0f;
    g_ctx.debug.segment_length_m = 0.0f;
    g_ctx.debug.segment_progress_m = 0.0f;
    g_ctx.debug.segment_remaining_m = 0.0f;
    g_ctx.debug.lateral_error_m = 0.0f;
    g_ctx.debug.speed_ref_cmps = 0.0f;
    g_ctx.debug.target_yaw_deg = 0.0f;
    g_ctx.debug.omega_cmd_radps = 0.0f;
    g_ctx.debug.omega_ref_degps = 0.0f;
    g_ctx.debug.vx_world_cmps = 0.0f;
    g_ctx.debug.vy_world_cmps = 0.0f;
    g_ctx.debug.segment_axis = 0U;
}

static void path_follow_clear_motion_telemetry(void)
{
    g_ctx.telemetry = (path_follow_motion_telemetry_t){0};
}

static void path_follow_refresh_motion_telemetry_state(void)
{
    g_ctx.telemetry.target_idx = g_ctx.idx;
    g_ctx.telemetry.active = (g_ctx.active ||
                              g_ctx.terminal_yaw_settle_active ||
                              g_ctx.rotate_only_active ||
                              g_ctx.yaw_rate_only_active) ? 1U : 0U;
    g_ctx.telemetry.paused = g_ctx.paused;
    g_ctx.telemetry.yaw_only_active = (g_ctx.rotate_only_active ||
                                       g_ctx.yaw_rate_only_active ||
                                       g_ctx.terminal_yaw_settle_active) ? 1U : 0U;
}

static void path_follow_store_motion_plan(float distance_m,
                                          float speed_ref_cmps,
                                          float plan_vx_world_cmps,
                                          float plan_vy_world_cmps,
                                          float plan_w_radps,
                                          uint8 segment_axis,
                                          float lateral_error_m)
{
    g_ctx.telemetry.distance_m = distance_m;
    g_ctx.telemetry.speed_ref_cmps = speed_ref_cmps;
    g_ctx.telemetry.plan_vx_world_cmps = plan_vx_world_cmps;
    g_ctx.telemetry.plan_vy_world_cmps = plan_vy_world_cmps;
    g_ctx.telemetry.plan_w_radps = plan_w_radps;
    g_ctx.telemetry.segment_axis = segment_axis;
    g_ctx.telemetry.lateral_error_m = lateral_error_m;
    path_follow_refresh_motion_telemetry_state();
}

void path_follow_xy_remember_clear(void)
{
    memset(g_xy_remember, 0, sizeof(g_xy_remember));
    g_xy_remember_count = 0U;
    g_xy_remember_write_idx = 0U;
}

size_t path_follow_xy_remember_count(void)
{
    return g_xy_remember_count;
}

uint8 path_follow_xy_remember_get(size_t index, path_follow_xy_remember_entry_t *entry)
{
    size_t start_idx;
    size_t read_idx;

    if (entry == NULL || index >= g_xy_remember_count)
    {
        return 0U;
    }

    start_idx = (g_xy_remember_write_idx + PATH_FOLLOW_XY_REMEMBER_CAPACITY - g_xy_remember_count) %
                PATH_FOLLOW_XY_REMEMBER_CAPACITY;
    read_idx = (start_idx + index) % PATH_FOLLOW_XY_REMEMBER_CAPACITY;
    *entry = g_xy_remember[read_idx];
    return 1U;
}

static void path_follow_remember_xy_at_target(const path_follow_geometry_t *geometry)
{
    path_follow_xy_remember_entry_t *entry;

    if (geometry == NULL)
    {
        return;
    }

    entry = &g_xy_remember[g_xy_remember_write_idx];
    entry->target_idx = g_ctx.idx;
    entry->target_x_m = geometry->target_x_m;
    entry->target_y_m = geometry->target_y_m;
    entry->actual_x_m = g_ctx.pose.x_m;
    entry->actual_y_m = g_ctx.pose.y_m;
    entry->error_x_m = g_ctx.pose.x_m - geometry->target_x_m;
    entry->error_y_m = g_ctx.pose.y_m - geometry->target_y_m;
    entry->distance_m = geometry->distance_m;

    g_xy_remember_write_idx++;
    if (g_xy_remember_write_idx >= PATH_FOLLOW_XY_REMEMBER_CAPACITY)
    {
        g_xy_remember_write_idx = 0U;
    }
    if (g_xy_remember_count < PATH_FOLLOW_XY_REMEMBER_CAPACITY)
    {
        g_xy_remember_count++;
    }
}
#if PATH_FOLLOW_ENABLE_CORNER_HANDOVER
/**
 * @brief 清空普通中间拐�?handover 的运行态缓存�? */
static void path_follow_reset_corner_handover_state(void)
{
    g_ctx.corner_handover_active = 0U;
    g_ctx.corner_handover_idx = SIZE_MAX;
    g_ctx.corner_enter_distance_m = 0.0f;
    g_ctx.corner_commit_distance_m = 0.0f;
}
#endif

/**
 * @brief 使当前路径段�?S 曲线 profile 失效，但保留上一拍速度参考�? */
static void path_follow_invalidate_active_profile(void)
{
    g_ctx.active_profile = (path_follow_scurve_profile_t){0};
    g_ctx.profile_time_s = 0.0f;
    g_ctx.profile_target_idx = 0U;
    g_ctx.profile_active = 0U;
}

/**
 * @brief 复位整段�?S 曲线执行状态�? */
static void path_follow_reset_motion_profile_state(void)
{
    path_follow_invalidate_active_profile();
#if PATH_FOLLOW_ENABLE_CORNER_HANDOVER
    path_follow_reset_corner_handover_state();
#endif
    path_follow_reset_active_scurve_cfg();
    path_follow_invalidate_segment_lock();
    g_ctx.last_ref_speed_cmps = 0.0f;
}


/**
 * @brief 清空本模块内部控制状态�? *
 * 该函数会清除当前�?S 曲线 profile、调试状态以及平移位置相�?PID 的历史量�? * 用于新路径装载、位姿重置或整条路径结束等需要重新起控的场景�? */
static void path_follow_reset_control_state(void)
{
    path_follow_reset_motion_profile_state();
    path_follow_clear_debug_state();
    path_follow_clear_motion_telemetry();
    g_ctx.rotate_in_tol_cycles = 0U;
    g_ctx.terminal_yaw_settle_active = 0U;
    g_ctx.terminal_yaw_settle_cycles = 0U;
    g_ctx.terminal_yaw_in_tol_cycles = 0U;
    g_ctx.yaw_rate_only_active = 0U;
    g_ctx.yaw_rate_target_degps = 0.0f;
    g_ctx.last_omega_cmd_degps = 0.0f;
    PID_Clear(&pid_world_x);
    PID_Clear(&pid_stay);
    PID_Clear(&pid_yaw);
    PID_Clear(&pid_accel_yaw);
}

static void path_follow_clear_hold_state(void)
{
    g_ctx.motion_reached = 0U;
}

static uint8 path_follow_ensure_segment_lock(void);
static uint8 path_follow_resolve_segment_axis(float delta_x_m, float delta_y_m);
static void path_follow_apply_target_feedforward(path_follow_geometry_t *geometry);
static float path_follow_compensate_distance_m(float target_distance_m,
                                               path_follow_distance_comp_dir_t comp_dir);
static float path_follow_compensate_body_component_m(float target_component_abs_m,
                                                     path_follow_distance_comp_dir_t comp_dir);
static void path_follow_apply_distance_compensation(path_follow_geometry_t *geometry,
                                                    float start_x_m,
                                                    float start_y_m);
static float path_follow_cross_axis_distance_trim_m(
        float main_abs_m,
        const path_follow_distance_comp_sample_t *tbl,
        size_t n);
static void path_follow_apply_cross_axis_distance_compensation(uint8 main_axis,
                                                               float dx_body,
                                                               float dy_body,
                                                               float *comp_dx_body,
                                                               float *comp_dy_body);

static uint8 path_follow_ensure_segment_lock(void)
{
    const float eps_m = 1e-6f;
    path_follow_segment_lock_t *lock = &g_ctx.segment_lock;
    path_follow_geometry_t geometry = {0};
    Point target;
    float start_x_m;
    float start_y_m;
    float seg_dx_m;
    float seg_dy_m;
    float seg_len_m;
    float distance_m;
    uint8 route_axis;

    if (g_ctx.path == NULL || g_ctx.idx >= g_ctx.steps)
    {
        return 0U;
    }

    if (lock->active && lock->path == g_ctx.path && lock->target_idx == g_ctx.idx)
    {
        return 1U;
    }

    target = g_ctx.path[g_ctx.idx];
    geometry.target_point = target;
    geometry.target_x_m = target.row * g_ctx.grid_m;
    geometry.target_y_m = target.col * g_ctx.grid_m;

    if (g_ctx.idx > 0U)
    {
        Point start = g_ctx.path[g_ctx.idx - 1U];
        start_x_m = start.row * g_ctx.grid_m;
        start_y_m = start.col * g_ctx.grid_m;
    }
    else
    {
        start_x_m = g_ctx.pose.x_m;
        start_y_m = g_ctx.pose.y_m;
    }

    route_axis = path_follow_resolve_segment_axis(geometry.target_x_m - start_x_m,
                                                  geometry.target_y_m - start_y_m);
    path_follow_apply_target_feedforward(&geometry);
    path_follow_apply_distance_compensation(&geometry, start_x_m, start_y_m);

    geometry.delta_x_m = geometry.target_x_m - g_ctx.pose.x_m;
    geometry.delta_y_m = geometry.target_y_m - g_ctx.pose.y_m;
    distance_m = sqrtf(geometry.delta_x_m * geometry.delta_x_m +
                       geometry.delta_y_m * geometry.delta_y_m);

    seg_dx_m = geometry.target_x_m - start_x_m;
    seg_dy_m = geometry.target_y_m - start_y_m;
    seg_len_m = sqrtf(seg_dx_m * seg_dx_m + seg_dy_m * seg_dy_m);
    if (seg_len_m <= eps_m)
    {
        if (distance_m <= eps_m)
        {
            path_follow_invalidate_segment_lock();
            return 0U;
        }
        start_x_m = g_ctx.pose.x_m;
        start_y_m = g_ctx.pose.y_m;
        seg_dx_m = geometry.delta_x_m;
        seg_dy_m = geometry.delta_y_m;
        seg_len_m = distance_m;
    }

    *lock = (path_follow_segment_lock_t){0};
    lock->path = g_ctx.path;
    lock->target_idx = g_ctx.idx;
    lock->target_point = target;
    lock->start_x_m = start_x_m;
    lock->start_y_m = start_y_m;
    lock->target_x_m = geometry.target_x_m;
    lock->target_y_m = geometry.target_y_m;
    lock->segment_length_m = seg_len_m;
    lock->segment_dir_x = seg_dx_m / seg_len_m;
    lock->segment_dir_y = seg_dy_m / seg_len_m;
    lock->segment_normal_x = -lock->segment_dir_y;
    lock->segment_normal_y = lock->segment_dir_x;
    lock->route_axis = route_axis;
    lock->active = 1U;

    return 1U;
}

static uint8 path_follow_fill_segment_geometry(path_follow_geometry_t *geometry)
{
    const path_follow_segment_lock_t *lock;
    float rel_x_m;
    float rel_y_m;

    if (geometry == NULL || g_ctx.path == NULL || g_ctx.idx >= g_ctx.steps)
    {
        return 0U;
    }

    if (!path_follow_ensure_segment_lock())
    {
        return 0U;
    }

    lock = &g_ctx.segment_lock;
    geometry->target_point = lock->target_point;
    geometry->target_x_m = lock->target_x_m;
    geometry->target_y_m = lock->target_y_m;
    geometry->segment_start_x_m = lock->start_x_m;
    geometry->segment_start_y_m = lock->start_y_m;
    geometry->segment_length_m = lock->segment_length_m;
    geometry->segment_dir_x = lock->segment_dir_x;
    geometry->segment_dir_y = lock->segment_dir_y;
    geometry->segment_normal_x = lock->segment_normal_x;
    geometry->segment_normal_y = lock->segment_normal_y;
    geometry->segment_axis = lock->route_axis;

    geometry->delta_x_m = geometry->target_x_m - g_ctx.pose.x_m;
    geometry->delta_y_m = geometry->target_y_m - g_ctx.pose.y_m;
    geometry->distance_m = sqrtf(geometry->delta_x_m * geometry->delta_x_m +
                                 geometry->delta_y_m * geometry->delta_y_m);

    if (geometry->segment_axis != 0U)
    {
        geometry->dir_x = geometry->segment_dir_x;
        geometry->dir_y = geometry->segment_dir_y;
    }
    else
    {
        geometry->dir_x = 0.0f;
        geometry->dir_y = 0.0f;
    }

    rel_x_m = g_ctx.pose.x_m - geometry->segment_start_x_m;
    rel_y_m = g_ctx.pose.y_m - geometry->segment_start_y_m;
    geometry->segment_progress_m = rel_x_m * geometry->segment_dir_x +
                                   rel_y_m * geometry->segment_dir_y;
    geometry->lateral_error_m = rel_x_m * geometry->segment_normal_x +
                                rel_y_m * geometry->segment_normal_y;
    geometry->segment_remaining_m = geometry->segment_length_m - geometry->segment_progress_m;
    if (geometry->segment_remaining_m < 0.0f)
    {
        geometry->segment_remaining_m = 0.0f;
    }

    return 1U;
}

static uint8 path_follow_segment_reached(const path_follow_geometry_t *geometry)
{
    (void)geometry;

    if (g_ctx.profile_active && g_ctx.profile_target_idx == g_ctx.idx)
    {
        return (g_ctx.profile_time_s >= g_ctx.active_profile.T) ? 1U : 0U;
    }

    return 0U;
}

static float path_follow_speed_distance_m(const path_follow_geometry_t *geometry)
{
    if (geometry == NULL)
    {
        return 0.0f;
    }
    if (geometry->segment_length_m <= 1e-6f)
    {
        return fmaxf(geometry->distance_m, 0.0f);
    }
    return fmaxf(geometry->segment_remaining_m, 0.0f);
}
static uint8 path_follow_is_true_diagonal_segment(const path_follow_geometry_t *geometry)
{
    float abs_x_m;
    float abs_y_m;

    if (geometry == NULL)
    {
        return 0U;
    }

    abs_x_m = fabsf(geometry->target_x_m - geometry->segment_start_x_m);
    abs_y_m = fabsf(geometry->target_y_m - geometry->segment_start_y_m);
    return (fminf(abs_x_m, abs_y_m) > PATH_FOLLOW_DIAGONAL_MIN_OFF_AXIS_M) ? 1U : 0U;
}

static uint8 path_follow_finish_path(path_follow_output_t *out,
                                     const path_follow_geometry_t *geometry);

static void path_follow_reset_pause_runtime(void)
{
    g_ctx.paused = 0U;
    g_ctx.pause_cycles_remaining = 0U;
    g_ctx.pause_cursor = 0U;
}

static void path_follow_begin_terminal_yaw_settle(void)
{
    g_ctx.terminal_yaw_settle_active = 1U;
    g_ctx.terminal_yaw_settle_cycles = 0U;
    g_ctx.terminal_yaw_in_tol_cycles = 0U;
    g_ctx.motion_reached = 0U;
    path_follow_invalidate_active_profile();
    g_ctx.last_ref_speed_cmps = 0.0f;
    car_direction = 0U;
}

/**
 * @brief 清空暂停事件配置及其运行态�? */
static void path_follow_clear_pause_config(void)
{
    size_t i;

    for (i = 0U; i < PATH_FOLLOW_MAX_PAUSE_POINTS; ++i)
    {
        g_ctx.pause_indices[i] = SIZE_MAX;
    }
    g_ctx.pause_count = 0U;
    g_ctx.pause_cycles_cfg = 0U;
    path_follow_reset_pause_runtime();
}

/**
 * @brief 将暂停游标追赶到当前目标索引，跳过已经过去的旧事件�? *
 * @param current_idx 当前路径目标索引�? */
static void path_follow_sync_pause_cursor(size_t current_idx)
{
    while (g_ctx.pause_cursor < g_ctx.pause_count &&
           g_ctx.pause_indices[g_ctx.pause_cursor] < current_idx)
    {
        g_ctx.pause_cursor++;
    }
}

/**
 * @brief 判断当前目标 corner 是否需要在到点后暂停�? *
 * @param target_idx 当前目标 corner 索引�? * @return `1` 表示�?target_idx 是下一个待触发暂停点，`0` 表示不是�? */
static uint8 path_follow_target_requires_pause(size_t target_idx)
{
    if (!g_ctx.pause_events_enabled || g_ctx.paused || 0U == g_ctx.pause_cycles_cfg)
    {
        return 0U;
    }

    path_follow_sync_pause_cursor(target_idx);
    if (g_ctx.pause_cursor >= g_ctx.pause_count)
    {
        return 0U;
    }

    return (g_ctx.pause_indices[g_ctx.pause_cursor] == target_idx) ? 1U : 0U;
}

/**
 * @brief 进入一个“到点后静止等待”的暂停窗口�? *
 * @param target_idx 当前触发暂停�?corner 索引，仅用于调试语义保持�? */
static void path_follow_enter_pause(size_t target_idx)
{
    (void)target_idx;

    g_ctx.paused = 1U;
    g_ctx.pause_cycles_remaining = (g_ctx.pause_cycles_cfg > 0U) ? g_ctx.pause_cycles_cfg : 1U;
    car_direction = 0U;
    /* Pause resumes with a fresh segment profile, so clear the current motion state here. */
    path_follow_reset_control_state();
}

/**
 * @brief 处理暂停窗口的倒计时与恢复逻辑�? *
 * @param out 输出结构体。暂停期间保�?active=1 且三轴速度�?0�? * @return `1` 表示本周期已由暂停逻辑完全处理，`0` 表示当前不在暂停态�? */
static uint8 path_follow_handle_pause(path_follow_output_t *out)
{
    if (!g_ctx.paused)
    {
        return 0U;
    }

    car_direction = 0U;
    path_follow_clear_debug_state();
    path_follow_store_motion_plan(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0U, 0.0f);

    if (out != NULL)
    {
        out->active = 1U;
        out->reached = 0U;
        out->vx_cmd = 0.0f;
        out->vy_cmd = 0.0f;
        out->omega_cmd = 0.0f;
        out->target_idx = g_ctx.idx;
    }

    if (g_ctx.pause_cycles_remaining > 0U)
    {
        g_ctx.pause_cycles_remaining--;
    }
    if (g_ctx.pause_cycles_remaining > 0U)
    {
        return 1U;
    }

    g_ctx.paused = 0U;
    if (g_ctx.pause_cursor < g_ctx.pause_count &&
        g_ctx.pause_indices[g_ctx.pause_cursor] == g_ctx.idx)
    {
        g_ctx.pause_cursor++;
    }

    if ((g_ctx.idx + 1U) < g_ctx.steps)
    {
            g_ctx.idx++;
        /* The next segment must rebuild its own profile from standstill after pause release. */
        path_follow_reset_control_state();
        return 1U;
    }

    path_follow_begin_terminal_yaw_settle();
    if (out != NULL)
    {
        out->active = 1U;
        out->reached = 0U;
        out->target_idx = g_ctx.idx;
    }
    return 1U;
}

/**
 * @brief 装载一条新的路径到路径跟随上下文�? *
 * 该函数负责统一处理路径切换时的状态重置、网格分辨率切换�? * 起点重置以及首目标点跳过策略�? *
 * @param path 路径点数组指针�? * @param steps 路径点数量�? * @param grid_m 路径对应网格边长，单�?m�? * @param reset_pose_to_first 是否将当前位姿重置到路径首点�? * @param skip_first_target 是否跳过路径首点，直接跟随下一目标点�? * @param pause_events_enabled 该路径是否允许触发预配置的暂停事件�? */
static void path_follow_apply_path(const Point *path,
                                   size_t steps,
                                   float grid_m,
                                   uint8 reset_pose_to_first,
                                   uint8 skip_first_target,
                                   uint8 pause_events_enabled)
{
    path_follow_reset_control_state();
    path_follow_reset_pause_runtime();
    path_follow_clear_hold_state();
    g_ctx.path = path;
    g_ctx.steps = steps;
    g_ctx.grid_m = (grid_m > 0.0f) ? grid_m : g_ctx.default_grid_m;
    g_ctx.idx = 0;
    g_ctx.pause_events_enabled = pause_events_enabled;
    g_ctx.rotate_only_active = 0U;
    g_ctx.rotate_hold_after_reach = 0U;
    g_ctx.rotate_in_tol_cycles = 0U;
    g_ctx.yaw_rate_only_active = 0U;
    g_ctx.yaw_rate_target_degps = 0.0f;

    if (path && steps > 0U)
    {
        if (reset_pose_to_first)
        {
            g_ctx.pose.x_m = path[0].row * g_ctx.grid_m;
            g_ctx.pose.y_m = path[0].col * g_ctx.grid_m;
        }

        if (skip_first_target && steps > 1U)
        {
            g_ctx.idx = 1U;
        }

        /* Each new motion starts from the current heading, unless the caller overrides it later. */
        g_ctx.target_yaw_deg = path_follow_wrap_deg(g_ctx.pose.yaw_deg);
        g_ctx.active = 1U;
    }
    else
    {
        g_ctx.active = 0U;
    }
}

static float path_follow_resolve_temp_grid(float requested_grid_m,
                                                float start_x_m,
                                                float start_y_m,
                                                float target_x_m,
                                                float target_y_m)
{
    float grid_m = requested_grid_m;
    float max_abs_m;
    float min_required_grid_m;

    if (grid_m <= 0.0f)
    {
        return requested_grid_m;
    }

    max_abs_m = fabsf(start_x_m);
    max_abs_m = fmaxf(max_abs_m, fabsf(start_y_m));
    max_abs_m = fmaxf(max_abs_m, fabsf(target_x_m));
    max_abs_m = fmaxf(max_abs_m, fabsf(target_y_m));
    if (max_abs_m <= 0.0f)
    {
        return grid_m;
    }

    min_required_grid_m = max_abs_m / PATH_FOLLOW_TEMP_POINT_INDEX_LIMIT_F;
    if (grid_m < min_required_grid_m)
    {
        grid_m = min_required_grid_m;
    }

    return grid_m;
}

/**
 * @brief 生成一条起点到目标点的两点直线路径并启动跟随�? *
 * 该函数用于发车前固定偏移场景，让底盘按当前位置到目标点的直线方向
 * 一次性完成位移，而不是拆成前�?左右两个分段动作�? *
 * @param target_x_m 临时目标�?X 坐标，单�?m�? * @param target_y_m 临时目标�?Y 坐标，单�?m�? * @param path_buffer 临时路径缓存�? * @param buffer_capacity 路径缓存容量�? * @param grid_m 生成临时路径时使用的栅格分辨率，单位 m�? */
static void path_follow_start_linear_move(float target_x_m,
                                          float target_y_m,
                                          Point *path_buffer,
                                          size_t buffer_capacity,
                                          float grid_m)
{
    const float eps_m = 0.001f;
    float start_x_m = g_ctx.pose.x_m;
    float start_y_m = g_ctx.pose.y_m;

    if (path_buffer == NULL || buffer_capacity < 2U || grid_m <= 0.0f)
    {
        return;
    }

    if (fabsf(target_x_m - start_x_m) <= eps_m &&
        fabsf(target_y_m - start_y_m) <= eps_m)
    {
        path_follow_apply_path(NULL, 0U, grid_m, 0U, 0U, 0U);
        return;
    }

    grid_m = path_follow_resolve_temp_grid(grid_m,
                                          start_x_m,
                                          start_y_m,
                                          target_x_m,
                                          target_y_m);

    path_buffer[0].row = (int)lroundf(start_x_m / grid_m);
    path_buffer[0].col = (int)lroundf(start_y_m / grid_m);
    path_buffer[1].row = (int)lroundf(target_x_m / grid_m);
    path_buffer[1].col = (int)lroundf(target_y_m / grid_m);

    path_follow_apply_path(path_buffer, 2U, grid_m, 0U, 1U, 0U);
}

/**
 * @brief 生成一条只包含水平/竖直两段的临时路径并启动跟随�? *
 * 该函数主要用于发车前固定偏移和外部位姿修正场景，
 * 会自动根据位移方向决定先�?X 还是先走 Y�? *
 * @param target_x_m 临时目标�?X 坐标，单�?m�? * @param target_y_m 临时目标�?Y 坐标，单�?m�? * @param path_buffer 临时路径缓存�? * @param buffer_capacity 路径缓存容量�? * @param grid_m 生成临时路径时使用的栅格分辨率，单位 m�? */
static void path_follow_start_axis_move(float target_x_m,
                                        float target_y_m,
                                        Point *path_buffer,
                                        size_t buffer_capacity,
                                        float grid_m)
{
    const float eps_m = 0.001f;
    float start_x_m = g_ctx.pose.x_m;
    float start_y_m = g_ctx.pose.y_m;
    float delta_x_m = target_x_m - start_x_m;
    float delta_y_m = target_y_m - start_y_m;
    uint8 move_x_first = (fabsf(delta_x_m) >= fabsf(delta_y_m)) ? 1U : 0U;
    size_t steps = 1U;

    if (path_buffer == NULL || buffer_capacity < 3U)
    {
        return;
    }

    if (fabsf(delta_x_m) <= eps_m && fabsf(delta_y_m) <= eps_m)
    {
        path_follow_apply_path(NULL, 0U, grid_m, 0U, 0U, 0U);
        return;
    }

    grid_m = path_follow_resolve_temp_grid(grid_m,
                                          start_x_m,
                                          start_y_m,
                                          target_x_m,
                                          target_y_m);

    path_buffer[0].row = (int)lroundf(start_x_m / grid_m);
    path_buffer[0].col = (int)lroundf(start_y_m / grid_m);

    if (move_x_first)
    {
        if (fabsf(delta_x_m) > eps_m)
        {
            path_buffer[steps].row = (int)lroundf(target_x_m / grid_m);
            path_buffer[steps].col = (int)lroundf(start_y_m / grid_m);
            steps++;
        }

        if (fabsf(delta_y_m) > eps_m)
        {
            path_buffer[steps].row = (int)lroundf(target_x_m / grid_m);
            path_buffer[steps].col = (int)lroundf(target_y_m / grid_m);
            steps++;
        }
    }
    else
    {
        if (fabsf(delta_y_m) > eps_m)
        {
            path_buffer[steps].row = (int)lroundf(start_x_m / grid_m);
            path_buffer[steps].col = (int)lroundf(target_y_m / grid_m);
            steps++;
        }

        if (fabsf(delta_x_m) > eps_m)
        {
            path_buffer[steps].row = (int)lroundf(target_x_m / grid_m);
            path_buffer[steps].col = (int)lroundf(target_y_m / grid_m);
            steps++;
        }
    }

    path_follow_apply_path(path_buffer, steps, grid_m, 0U, 1U, 0U);
}

/**
 * @brief 初始化路径跟随模块�? *
 * 该函数完成上下文默认值、速度规划参数以及相关 PID 的初始化�? *
 * @param grid_size_m 默认路径网格边长，单�?m�? * @param pulses_per_meter 编码器每米脉冲数，用于里程计换算�? */
void path_follow_init(float grid_size_m, float pulses_per_meter)
{
    // 初始化参数和 PID
    g_ctx.grid_m = grid_size_m;
    g_ctx.default_grid_m = grid_size_m;
    g_ctx.pulses_per_meter = pulses_per_meter;
    g_ctx.pos_tol_m = 0.004f;
    g_ctx.yaw_tol_deg = PATH_FOLLOW_ROTATE_TOL_DEG;
    g_ctx.max_v_mps = 4.0f;    // 最大线速度
    g_ctx.max_w_rad = 10000.0f;    // 航向控制内部角速度上限，单�?deg/s
    g_ctx.imu_to_world_yaw_offset_deg = 0.0f;
    g_ctx.target_yaw_deg = 0.0f;
    g_ctx.heading_mode = PATH_FOLLOW_HEADING_FIXED;
    g_ctx.pose.x_m = 0.0f;
    g_ctx.pose.y_m = 0.0f;
    g_ctx.pose.yaw_deg = 0.0f;
    g_ctx.speed_cfg.segment_end_speed_cmps = SCURVE_SEGMENT_END_SPEED_CMPS;
    g_ctx.path = NULL;
    g_ctx.steps = 0;
    g_ctx.idx = 0;
    g_ctx.rotate_only_active = 0U;
    g_ctx.rotate_hold_after_reach = 0U;
    g_ctx.rotate_in_tol_cycles = 0U;
    g_ctx.yaw_rate_only_active = 0U;
    g_ctx.yaw_rate_target_degps = 0.0f;
    path_follow_clear_hold_state();
    g_ctx.active = 0;
    g_ctx.pause_events_enabled = 0U;
    path_follow_reset_scurve_band_defaults();
    path_follow_reset_motion_profile_state();
    path_follow_clear_debug_state();
    path_follow_clear_motion_telemetry();
    path_follow_clear_pause_config();

    pid_world_init.fKp = 3.7f;  // 位置误差到速度输出
    pid_world_init.fKi = 0.0f;
    pid_world_init.fKd = 2.0f;
    pid_world_init.fMax_Iout = g_ctx.max_v_mps*100.0f;
    pid_world_init.fMax_Out = g_ctx.max_v_mps*100.0f;
    pid_world_init.alpha = 0.9f;

    pid_stay_init.fKp = 3.9f;  // 位置误差到速度输出
    pid_stay_init.fKi = 0.0f;
    pid_stay_init.fKd = 3.4f;
    pid_stay_init.fMax_Iout = 200.0f;
    pid_stay_init.fMax_Out = 200.0f;
    pid_stay_init.alpha = 0.9f;

    pid_yaw_init.fKp = 10.7f;  // 航向角外环：角度误差到目标角速度
    pid_yaw_init.fKi = 0.002f;
    pid_yaw_init.fKd = 110.0f;
    pid_yaw_init.fMax_Iout = 5000.0f;
    pid_yaw_init.fMax_Out = 10000.0f;
    pid_yaw_init.alpha = 0.9f;

    pid_accel_yaw_init.fKp = 1.00f;  // 航向角速度内环：目标角速度到最终角速度命令
    pid_accel_yaw_init.fKi = 0.007f;
    pid_accel_yaw_init.fKd = 0.00f;
    pid_accel_yaw_init.fMax_Iout = 5000.0f;
    pid_accel_yaw_init.fMax_Out = 10000.0f;
    pid_accel_yaw_init.alpha = 0.9f;

    PID_Init(&pid_world_x, &pid_world_init);
    PID_Init(&pid_stay, &pid_stay_init);
    PID_Init(&pid_yaw, &pid_yaw_init);
    PID_Init(&pid_accel_yaw, &pid_accel_yaw_init);
}

/**
 * @brief 重置当前里程计位姿�? *
 * 重置位姿时会同时清空平移控制状态，避免旧路径残留的 PID 历史量继续作用�? *
 * @param x_m 新的 X 坐标，单�?m�? * @param y_m 新的 Y 坐标，单�?m�? * @param yaw_deg 新的航向角，单位 deg�? */
void path_follow_reset_pose(float x_m, float y_m, float yaw_deg)
{
    float imu_yaw_deg = path_follow_wrap_deg(eulerAngle.yaw);

    g_ctx.imu_to_world_yaw_offset_deg = path_follow_wrap_deg(path_follow_wrap_deg(yaw_deg) -
                                                             imu_yaw_deg);
    g_ctx.pose.x_m = x_m;
    g_ctx.pose.y_m = y_m;
    g_ctx.pose.yaw_deg = path_follow_world_yaw_from_imu(imu_yaw_deg);
    path_follow_clear_hold_state();
    path_follow_reset_control_state();
}

/**
 * @brief 强制停止路径跟随并清空运行状态�? *
 * 该接口用于外部异常保护或紧急停机场景�? * 它会保留当前位姿，但清除路径、暂停、旋转控制以及相�?PID 历史量�? */
void path_follow_stop(void)
{
    g_ctx.active = 0U;
    g_ctx.rotate_only_active = 0U;
    g_ctx.rotate_hold_after_reach = 0U;
    g_ctx.rotate_in_tol_cycles = 0U;
    g_ctx.yaw_rate_only_active = 0U;
    g_ctx.yaw_rate_target_degps = 0.0f;
    g_ctx.path = NULL;
    g_ctx.steps = 0U;
    g_ctx.idx = 0U;
    g_ctx.pause_events_enabled = 0U;
    g_ctx.target_yaw_deg = 0.0f;
    car_direction = 0U;
    wait_stop = 0U;
    path_follow_clear_pause_config();
    path_follow_clear_hold_state();
    path_follow_reset_control_state();
    PID_Clear(&pid_yaw);
    PID_Clear(&pid_accel_yaw);
}

/**
 * @brief 设置外部位姿输入接口�? *
 * @param x_m 外部位置 X，单�?m�? * @param y_m 外部位置 Y，单�?m�? * @param valid 外部位置是否有效�? *
 * @note 当前版本保留接口但未在本模块内部直接使用�? */
void path_follow_set_external_position(float x_m, float y_m, uint8 valid)
{
    (void)x_m;
    (void)y_m;
    (void)valid;
}

/**
 * @brief 设置一条新的跟随路径�? *
 * @param path 路径点数组指针�? * @param steps 路径点数量�? */
void path_follow_set_path(const Point *path, size_t steps)
{
    path_follow_set_path_pause_enabled(path, steps, 1U);
}

/**
 * @brief 设置一条新的跟随路径，并显式指定该路径是否允许触发 pause 事件�? *
 * @param path 路径点数组指针�? * @param steps 路径点数量�? * @param pause_events_enabled `1` 表示允许使用已配置的 pause 索引，`0` 表示本路径禁�?pause�? */
void path_follow_set_path_pause_enabled(const Point *path, size_t steps, uint8 pause_events_enabled)
{
    path_follow_apply_path(path, steps, g_ctx.default_grid_m, 0U, 1U, pause_events_enabled);
}

/**
 * @brief 配置普通地�?corner_path 上的暂停事件列表�? *
 * @param pause_indices 需要暂停的 target corner 索引数组，可�?NULL�? * @param pause_count pause_indices 中的元素个数�? * @param pause_ms 每个暂停点统一停留时长，单�?ms；传 0 可清空配置�? */
void path_follow_set_pause_indices(const size_t *pause_indices, size_t pause_count, uint32 pause_ms)
{
    size_t copy_count = 0U;
    size_t i;

    path_follow_clear_pause_config();
    if (pause_indices == NULL || pause_count == 0U || pause_ms == 0U)
    {
        return;
    }

    if (pause_count > PATH_FOLLOW_MAX_PAUSE_POINTS)
    {
        pause_count = PATH_FOLLOW_MAX_PAUSE_POINTS;
    }

    for (i = 0U; i < pause_count; ++i)
    {
        g_ctx.pause_indices[copy_count++] = pause_indices[i];
    }
    g_ctx.pause_count = copy_count;
    g_ctx.pause_cycles_cfg = (pause_ms * (uint32)PATH_FOLLOW_UPDATE_RATE_HZ + 999U) / 1000U;
    if (g_ctx.pause_cycles_cfg == 0U)
    {
        g_ctx.pause_cycles_cfg = 1U;
    }
}

/**
 * @brief 设置单个离散目标点并生成临时两点路径�? *
 * @param target_row 目标点所在网格行号�? * @param target_col 目标点所在网格列号�? */
void path_follow_set_target(int target_row, int target_col)
{
    g_single_target_path[0].row = (int)lroundf(g_ctx.pose.x_m / g_ctx.default_grid_m);
    g_single_target_path[0].col = (int)lroundf(g_ctx.pose.y_m / g_ctx.default_grid_m);
    g_single_target_path[1].row = target_row;
    g_single_target_path[1].col = target_col;
    path_follow_apply_path(g_single_target_path, 2U, g_ctx.default_grid_m, 0U, 1U, 0U);
}

void path_follow_set_target_yaw(float target_yaw_deg)
{
    g_ctx.target_yaw_deg = path_follow_wrap_deg(target_yaw_deg);
}

void path_follow_hold_current_yaw(void)
{
    path_follow_set_target_yaw(g_ctx.pose.yaw_deg);
}

static void path_follow_start_rotate_to_yaw_mode(float target_yaw_deg, uint8 hold_after_reach)
{
    path_follow_reset_control_state();
    path_follow_clear_hold_state();
    PID_Clear(&pid_yaw);
    g_ctx.path = NULL;
    g_ctx.steps = 0U;
    g_ctx.idx = 0U;
    g_ctx.active = 0U;
    g_ctx.paused = 0U;
    g_ctx.pause_cycles_remaining = 0U;
    g_ctx.rotate_only_active = 1U;
    g_ctx.yaw_rate_only_active = 0U;
    g_ctx.yaw_rate_target_degps = 0.0f;
    g_ctx.rotate_hold_after_reach = hold_after_reach ? 1U : 0U;
    g_ctx.rotate_in_tol_cycles = 0U;
    g_ctx.pause_events_enabled = 0U;
    path_follow_set_target_yaw(target_yaw_deg);
}

void path_follow_start_rotate_to_yaw(float target_yaw_deg)
{
    path_follow_start_rotate_to_yaw_mode(target_yaw_deg, 0U);
}

void path_follow_start_rotate_to_yaw_hold(float target_yaw_deg)
{
    path_follow_start_rotate_to_yaw_mode(target_yaw_deg, 1U);
}

void path_follow_start_yaw_rate_debug(float target_omega_degps)
{
    path_follow_reset_control_state();
    path_follow_clear_hold_state();
    g_ctx.path = NULL;
    g_ctx.steps = 0U;
    g_ctx.idx = 0U;
    g_ctx.active = 0U;
    g_ctx.paused = 0U;
    g_ctx.pause_cycles_remaining = 0U;
    g_ctx.rotate_only_active = 0U;
    g_ctx.rotate_hold_after_reach = 0U;
    g_ctx.pause_events_enabled = 0U;
    g_ctx.yaw_rate_target_degps = clamp_sym(target_omega_degps, g_ctx.max_w_rad);
    g_ctx.yaw_rate_only_active = 1U;
    PID_Clear(&pid_yaw);
    PID_Clear(&pid_accel_yaw);
}

/**
 * @brief 启动发车前固定偏移动作�? *
 * @param delta_x_m X 方向偏移量，单位 m�? * @param delta_y_m Y 方向偏移量，单位 m�? */
void path_follow_start_offset_move(float delta_x_m, float delta_y_m)
{
    float start_x_m = g_ctx.pose.x_m;
    float start_y_m = g_ctx.pose.y_m;
    path_follow_start_linear_move(start_x_m + delta_x_m,
                                  start_y_m + delta_y_m,
                                  g_prestart_offset_path,
                                  sizeof(g_prestart_offset_path) / sizeof(g_prestart_offset_path[0]),
                                  PRESTART_OFFSET_RESOLUTION_M);
}

void path_follow_start_offset_move_hold(float delta_x_m, float delta_y_m)
{
    path_follow_start_offset_move(delta_x_m, delta_y_m);
    if (g_ctx.active)
    {
        g_ctx.motion_reached = 0U;
        return;
    }

    g_ctx.motion_reached = 1U;
}

void path_follow_release_hold(void)
{
}

void path_follow_start_point_move(float target_x_m, float target_y_m)
{
    float delta_x_m = target_x_m - g_ctx.pose.x_m;
    float delta_y_m = target_y_m - g_ctx.pose.y_m;

    if (path_follow_resolve_segment_axis(delta_x_m, delta_y_m) == 3U)
    {
        path_follow_start_linear_move(target_x_m,
                                      target_y_m,
                                      g_pose_correction_path,
                                      sizeof(g_pose_correction_path) / sizeof(g_pose_correction_path[0]),
                                      PRESTART_OFFSET_RESOLUTION_M);
        return;
    }

    path_follow_start_axis_move(target_x_m,
                                target_y_m,
                                g_pose_correction_path,
                                sizeof(g_pose_correction_path) / sizeof(g_pose_correction_path[0]),
                                PRESTART_OFFSET_RESOLUTION_M);
}

/**
 * @brief 启动位姿修正动作�? *
 * 该函数会生成一条短临时路径，使底盘按当前位置缓慢靠近外部修正目标�? *
 * @param target_x_m 修正目标 X 坐标，单�?m�? * @param target_y_m 修正目标 Y 坐标，单�?m�? */
void path_follow_start_pose_correction(float target_x_m, float target_y_m)
{
    path_follow_start_point_move(target_x_m, target_y_m);
}

static uint8 path_follow_get_current_segment_delta(float *delta_x_m, float *delta_y_m)
{
    Point target;
    float target_x_m;
    float target_y_m;
    float start_x_m;
    float start_y_m;

    if (delta_x_m == NULL || delta_y_m == NULL ||
        !g_ctx.active || g_ctx.path == NULL || g_ctx.idx >= g_ctx.steps)
    {
        return 0U;
    }

    target = g_ctx.path[g_ctx.idx];
    target_x_m = target.row * g_ctx.grid_m;
    target_y_m = target.col * g_ctx.grid_m;

    if (g_ctx.idx > 0U)
    {
        Point start = g_ctx.path[g_ctx.idx - 1U];
        start_x_m = start.row * g_ctx.grid_m;
        start_y_m = start.col * g_ctx.grid_m;
    }
    else
    {
        start_x_m = g_ctx.pose.x_m;
        start_y_m = g_ctx.pose.y_m;
    }

    *delta_x_m = target_x_m - start_x_m;
    *delta_y_m = target_y_m - start_y_m;
    return 1U;
}

static uint8 path_follow_get_current_segment_axis(void)
{
    float delta_x_m;
    float delta_y_m;

    if (!path_follow_get_current_segment_delta(&delta_x_m, &delta_y_m))
    {
        return 0U;
    }

    return path_follow_resolve_segment_axis(delta_x_m, delta_y_m);
}

static void path_follow_apply_diagonal_odom_comp(float *vx_body, float *vy_body)
{
    /* 斜向里程计补偿归零：重新校准前不改编码器解算出的 vx/vy�?*/
    (void)vx_body;
    (void)vy_body;
}
/**
 * @brief 根据四轮编码器和当前航向估算世界系位姿�? *
 * @param yaw_deg 当前 IMU 航向角，单位 deg�? */
void path_follow_update_odometry(float yaw_deg)
{
    float world_yaw_deg;

    if (g_ctx.pulses_per_meter <= 0.0f)
    {
        g_ctx.telemetry.actual_vx_world_cmps = 0.0f;
        g_ctx.telemetry.actual_vy_world_cmps = 0.0f;
        g_ctx.telemetry.actual_w_radps = 0.0f;
        path_follow_refresh_motion_telemetry_state();
        return;
    }

    float ppm_x = ((float)pulse_per_meter_x > 0.0f) ? (float)pulse_per_meter_x : g_ctx.pulses_per_meter;
    float ppm_y = ((float)pulse_per_meter_y > 0.0f) ? (float)pulse_per_meter_y : g_ctx.pulses_per_meter;
    float ppm_rot = g_ctx.pulses_per_meter;
    float count_to_x_mps = ((float)MOTOR_ENCODER_SAMPLE_HZ) / ppm_x;
    float count_to_y_mps = ((float)MOTOR_ENCODER_SAMPLE_HZ) / ppm_y;
    float count_to_rot_mps = ((float)MOTOR_ENCODER_SAMPLE_HZ) / ppm_rot;

    float vx_body = 0.25f *
                    ((float)up_L_all + (float)up_R_all + (float)down_L_all + (float)down_R_all) *
                    count_to_x_mps;
    float vy_body = 0.25f *
                    (-(float)up_L_all + (float)up_R_all + (float)down_L_all - (float)down_R_all) *
                    count_to_y_mps;

    path_follow_apply_diagonal_odom_comp(&vx_body, &vy_body);

    float omega_body = (-(float)up_L_all + (float)up_R_all - (float)down_L_all + (float)down_R_all) *
                       count_to_rot_mps / (2 * D_X + 2 * D_Y);
    float dt = MOTOR_ENCODER_SAMPLE_DT_S;
    float yaw_rad;
    float cos_yaw;
    float sin_yaw;
    float vx_world;
    float vy_world;

    world_yaw_deg = path_follow_world_yaw_from_imu(yaw_deg);
    yaw_rad = world_yaw_deg * ((float)M_PI / 180.0f);

    cos_yaw = cosf(yaw_rad);
    sin_yaw = sinf(yaw_rad);

    vx_world = vx_body * cos_yaw - vy_body * sin_yaw;
    vy_world = vx_body * sin_yaw + vy_body * cos_yaw;

    g_ctx.pose.x_m += vx_world * dt;
    g_ctx.pose.y_m += vy_world * dt;
    g_ctx.pose.yaw_deg = world_yaw_deg;
    g_ctx.telemetry.actual_vx_world_cmps = path_follow_mps_to_cmps(vx_world);
    g_ctx.telemetry.actual_vy_world_cmps = path_follow_mps_to_cmps(vy_world);
    g_ctx.telemetry.actual_w_radps = omega_body;
    path_follow_refresh_motion_telemetry_state();
}

/**
 * @brief 根据目标点误差判断当前路径段主轴方向�? *
 * @param delta_x_m 目标点相对当前位置的 X 方向误差，单�?m�? * @param delta_y_m 目标点相对当前位置的 Y 方向误差，单�?m�? * @return `0` 静止，`1` X 段，`2` Y 段，`3` 斜段�? */
static uint8 path_follow_resolve_segment_axis(float delta_x_m, float delta_y_m)
{
    float delta_x_abs = fabsf(delta_x_m);
    float delta_y_abs = fabsf(delta_y_m);
    const float axis_eps_m = 0.001f;

    if (delta_x_abs <= axis_eps_m && delta_y_abs <= axis_eps_m)
    {
        return 0U;
    }
    if (delta_y_abs <= axis_eps_m)
    {
        return 1U;
    }
    if (delta_x_abs <= axis_eps_m)
    {
        return 2U;
    }
    if (fminf(delta_x_abs, delta_y_abs) > PATH_FOLLOW_DIAGONAL_MIN_OFF_AXIS_M)
    {
        return 3U;
    }

    return (delta_x_abs >= delta_y_abs) ? 1U : 2U;
}


/**
 * @brief 将路径跟随状态切换为完成�? *
 * @param out 输出结构体指针，用于回写 `reached` 标志�? * @return 固定返回 `0`，便于在调用点直接作为失�?结束分支使用�? */
static uint8 path_follow_finish_path(path_follow_output_t *out,
                                     const path_follow_geometry_t *geometry)
{
    (void)geometry;

    g_ctx.active = 0U;
    car_direction = 0U;
    path_follow_reset_control_state();
    path_follow_clear_hold_state();
    g_ctx.motion_reached = 1U;
    if (out != NULL)
    {
        out->active = 0U;
        out->reached = 1U;
        out->vx_cmd = 0.0f;
        out->vy_cmd = 0.0f;
        out->omega_cmd = 0.0f;
    }
    return 0U;
}
/**
 * @brief 判断当前目标点是否允许应用目标前馈外推�? *
 * 仅对中间转向拐点、最后目标点以及两点临时路径目标生效，避免完整栅格路径中的普通直行点被逐格外推�? */
static uint8 path_follow_target_feedforward_enabled(void)
{
    Point prev_point;
    Point curr_point;
    Point next_point;
    int prev_dr;
    int prev_dc;
    int next_dr;
    int next_dc;

    if ((PATH_TARGET_FEEDFORWARD_BASE_M <= 0.0f &&
         PATH_TARGET_FEEDFORWARD_GAIN_PER_M <= 0.0f) ||
        PATH_TARGET_FEEDFORWARD_MAX_M <= 0.0f ||
        PATH_TARGET_FEEDFORWARD_SEGMENT_RATIO_MAX <= 0.0f)
    {
        return 0U;
    }
    if (g_ctx.path == NULL || g_ctx.steps < 2U || g_ctx.idx == 0U || g_ctx.idx >= g_ctx.steps)
    {
        return 0U;
    }

    if ((g_ctx.idx + 1U) >= g_ctx.steps)
    {
        return 1U;
    }

    prev_point = g_ctx.path[g_ctx.idx - 1U];
    curr_point = g_ctx.path[g_ctx.idx];
    next_point = g_ctx.path[g_ctx.idx + 1U];

    prev_dr = curr_point.row - prev_point.row;
    prev_dc = curr_point.col - prev_point.col;
    next_dr = next_point.row - curr_point.row;
    next_dc = next_point.col - curr_point.col;

    return (prev_dr != next_dr || prev_dc != next_dc) ? 1U : 0U;
}

/**
 * @brief 根据路径段长度计算目标前馈外推距离�? */
static float path_follow_compute_target_feedforward_m(float seg_len_m)
{
    float extra_len_m;
    float feedforward_m;
    float segment_cap_m;

    if (seg_len_m <= 0.0f ||
        PATH_TARGET_FEEDFORWARD_MAX_M <= 0.0f ||
        PATH_TARGET_FEEDFORWARD_SEGMENT_RATIO_MAX <= 0.0f)
    {
        return 0.0f;
    }

    extra_len_m = fmaxf(seg_len_m - g_ctx.default_grid_m, 0.0f);
    feedforward_m = PATH_TARGET_FEEDFORWARD_BASE_M +
                    PATH_TARGET_FEEDFORWARD_GAIN_PER_M * extra_len_m;
    feedforward_m = fmaxf(feedforward_m, 0.0f);
    feedforward_m = fminf(feedforward_m, PATH_TARGET_FEEDFORWARD_MAX_M);

    segment_cap_m = seg_len_m * PATH_TARGET_FEEDFORWARD_SEGMENT_RATIO_MAX;
    feedforward_m = fminf(feedforward_m, segment_cap_m);

    return fmaxf(feedforward_m, 0.0f);
}

/**
 * @brief 按上一段运动方向外推目标点，抵消到点容差造成的提前停车�? */
static void path_follow_apply_target_feedforward(path_follow_geometry_t *geometry)
{
    Point prev_point;
    Point curr_point;
    float seg_dx_m;
    float seg_dy_m;
    float seg_len_m;
    float feedforward_m;

    if (geometry == NULL || !path_follow_target_feedforward_enabled())
    {
        return;
    }

    prev_point = g_ctx.path[g_ctx.idx - 1U];
    curr_point = g_ctx.path[g_ctx.idx];
    seg_dx_m = (curr_point.row - prev_point.row) * g_ctx.grid_m;
    seg_dy_m = (curr_point.col - prev_point.col) * g_ctx.grid_m;
    seg_len_m = sqrtf(seg_dx_m * seg_dx_m + seg_dy_m * seg_dy_m);
    if (seg_len_m <= 0.001f)
    {
        return;
    }

    feedforward_m = path_follow_compute_target_feedforward_m(seg_len_m);
    if (feedforward_m <= 0.0f)
    {
        return;
    }

    geometry->target_x_m += feedforward_m * seg_dx_m / seg_len_m;
    geometry->target_y_m += feedforward_m * seg_dy_m / seg_len_m;
}

/**
 * @brief 对补偿表进行分段线性插值�? *
 * @param tbl   表格数组指针（必须按 distance_m 升序排列）�? * @param n     表格条目数量�? * @param d_m   名义距离，单�?m�? * @return      插值得到的 error_cm 值�? */
static float path_follow_interp_comp_table(
        const path_follow_distance_comp_sample_t *tbl, size_t n, float d_m)
{
    size_t i;

    if (n == 0U)
    {
        return 0.0f;
    }
    if (d_m <= tbl[0].distance_m)
    {
        return tbl[0].error_cm;
    }
    if (d_m >= tbl[n - 1U].distance_m)
    {
        return tbl[n - 1U].error_cm;
    }

    for (i = 1U; i < n; ++i)
    {
        if (d_m <= tbl[i].distance_m)
        {
            float t = (d_m - tbl[i - 1U].distance_m) /
                      (tbl[i].distance_m - tbl[i - 1U].distance_m);
            return tbl[i - 1U].error_cm +
                   t * (tbl[i].error_cm - tbl[i - 1U].error_cm);
        }
    }

    return tbl[n - 1U].error_cm;
}

/**
 * @brief 对车体系单个分量的绝对值应用距离补偿�? *
 * 前进、后退、左移：查表插值，cmd = target_abs + error_cm / 100�? * 右移：保留原版线性公�?(k, b) 加残差修正�? *
 * @param target_component_abs_m  车体系分量的绝对值（>= 0），单位 m�? * @param comp_dir                方向标识�? * @return 补偿后的绝对分量，单�?m�? */
static float path_follow_compensate_body_component_m(float target_component_abs_m,
                                                     path_follow_distance_comp_dir_t comp_dir)
{
#if PATH_DISTANCE_COMP_ENABLE
    float cmd_m;
    float error_cm;

    if (target_component_abs_m < PATH_DISTANCE_COMP_MIN_APPLY_M)
    {
        return target_component_abs_m;
    }

    switch (comp_dir)
    {
    case PATH_DISTANCE_COMP_DIR_FWD:
        error_cm = path_follow_interp_comp_table(
                g_path_comp_fwd_table,
                sizeof(g_path_comp_fwd_table) / sizeof(g_path_comp_fwd_table[0]),
                target_component_abs_m);
        cmd_m = target_component_abs_m + error_cm / 100.0f;
        break;

    case PATH_DISTANCE_COMP_DIR_BACK:
        error_cm = path_follow_interp_comp_table(
                g_path_comp_back_table,
                sizeof(g_path_comp_back_table) / sizeof(g_path_comp_back_table[0]),
                target_component_abs_m);
        cmd_m = target_component_abs_m + error_cm / 100.0f;
        break;

    case PATH_DISTANCE_COMP_DIR_LEFT:
        error_cm = path_follow_interp_comp_table(
                g_path_comp_left_table,
                sizeof(g_path_comp_left_table) / sizeof(g_path_comp_left_table[0]),
                target_component_abs_m);
        cmd_m = target_component_abs_m + error_cm / 100.0f;
        break;

    case PATH_DISTANCE_COMP_DIR_RIGHT:
    default:
        error_cm = path_follow_interp_comp_table(
                g_path_comp_right_table,
                sizeof(g_path_comp_right_table) / sizeof(g_path_comp_right_table[0]),
                target_component_abs_m);
        cmd_m = target_component_abs_m + error_cm / 100.0f;
        break;
    }

    if (cmd_m < PATH_DISTANCE_COMP_MIN_CMD_M)
    {
        cmd_m = PATH_DISTANCE_COMP_MIN_CMD_M;
    }
    return cmd_m;
#else
    (void)comp_dir;
    return target_component_abs_m;
#endif
}

/**
 * @brief 兼容接口（整段距离公式）。
 *        内部直接转发 path_follow_compensate_body_component_m()，
 *        前进、后退、左移、右移全部按表格插值补偿。
 */
static float path_follow_compensate_distance_m(float target_distance_m,
                                               path_follow_distance_comp_dir_t comp_dir)
{
    return path_follow_compensate_body_component_m(target_distance_m, comp_dir);
}


/**
 * @brief 计算串轴距离补偿量。
 *
 * 该函数只根据本段主方向距离查表，不使用实时横向误差。
 * 表中 error_cm 直接表示需要额外给副轴的补偿量，正负号遵循车体系：
 *   +X 前进，-X 后退，+Y 左移，-Y 右移。
 */
static float path_follow_cross_axis_distance_trim_m(
        float main_abs_m,
        const path_follow_distance_comp_sample_t *tbl,
        size_t n)
{
#if PATH_CROSS_AXIS_DISTANCE_COMP_ENABLE
    float trim_m;

    if (main_abs_m < PATH_CROSS_AXIS_COMP_MIN_APPLY_M || tbl == NULL || n == 0U)
    {
        return 0.0f;
    }

    trim_m = path_follow_interp_comp_table(tbl, n, main_abs_m) / 100.0f;
    trim_m = clamp_sym(trim_m, PATH_CROSS_AXIS_COMP_LIMIT_M);
    return trim_m;
#else
    (void)main_abs_m;
    (void)tbl;
    (void)n;
    return 0.0f;
#endif
}

/**
 * @brief 对纯前后/纯左右段叠加副轴距离补偿�? *
 * - X 段（前进/后退）：补偿 Y 分量，用来抵消左右串轴�? * - Y 段（左移/右移）：补偿 X 分量，用来抵消前后串轴�? * - 斜向段：暂不处理，避免破坏斜�?vx/vy 比例�? */
static void path_follow_apply_cross_axis_distance_compensation(uint8 main_axis,
                                                               float dx_body,
                                                               float dy_body,
                                                               float *comp_dx_body,
                                                               float *comp_dy_body)
{
    float cross_trim_m;

    if (comp_dx_body == NULL || comp_dy_body == NULL)
    {
        return;
    }

    if (main_axis == 1U)
    {
        /* 前进/后退时，副轴补偿 Y；+Y 为左移，-Y 为右移。 */
        if (dx_body >= 0.0f)
        {
            cross_trim_m = path_follow_cross_axis_distance_trim_m(
                    fabsf(dx_body),
                    g_path_cross_fwd_y_table,
                    sizeof(g_path_cross_fwd_y_table) / sizeof(g_path_cross_fwd_y_table[0]));
        }
        else
        {
            cross_trim_m = path_follow_cross_axis_distance_trim_m(
                    fabsf(dx_body),
                    g_path_cross_back_y_table,
                    sizeof(g_path_cross_back_y_table) / sizeof(g_path_cross_back_y_table[0]));
        }
        *comp_dy_body += cross_trim_m;
    }
    else if (main_axis == 2U)
    {
        /* 左移/右移时，副轴补偿 X；+X 为前进，-X 为后退。 */
        if (dy_body >= 0.0f)
        {
            cross_trim_m = path_follow_cross_axis_distance_trim_m(
                    fabsf(dy_body),
                    g_path_cross_left_x_table,
                    sizeof(g_path_cross_left_x_table) / sizeof(g_path_cross_left_x_table[0]));
        }
        else
        {
            cross_trim_m = path_follow_cross_axis_distance_trim_m(
                    fabsf(dy_body),
                    g_path_cross_right_x_table,
                    sizeof(g_path_cross_right_x_table) / sizeof(g_path_cross_right_x_table[0]));
        }
        *comp_dx_body += cross_trim_m;
    }
}

/**
 * @brief 对路径段目标点应用车体系分量距离补偿�? *
 * 算法流程�? *  1. 计算段起点到目标点的世界坐标位移
 *     （可能已包含前馈偏移）�? *  2. 转为车体系（使用当前车头角）�? *       dx_body =  dx * cos_yaw + dy * sin_yaw   (+X 为前�?
 *       dy_body = -dx * sin_yaw + dy * cos_yaw   (+Y 为左�?
 *  3. 分别补偿两个车体系分量：
 *       - dx_body > 0 ：前进补偿表
 *       - dx_body < 0 ：后退补偿�? *       - dy_body > 0 ：左移补偿表
 *       - dy_body < 0 ：右移补偿表
 *  4. 补偿后车体系向量转回世界坐标�? *       new_target_x = start_x + comp_dx_body * cos_yaw - comp_dy_body * sin_yaw
 *       new_target_y = start_y + comp_dx_body * sin_yaw + comp_dy_body * cos_yaw
 */
static void path_follow_apply_distance_compensation(path_follow_geometry_t *geometry,
                                                    float start_x_m,
                                                    float start_y_m)
{
    float dx_m;
    float dy_m;
    float yaw_rad;
    float cos_yaw;
    float sin_yaw;
    float dx_body;
    float dy_body;
    float comp_dx_body;
    float comp_dy_body;
    uint8 main_axis;

    if (geometry == NULL)
    {
        return;
    }

    dx_m = geometry->target_x_m - start_x_m;
    dy_m = geometry->target_y_m - start_y_m;

    if (fabsf(dx_m) <= 1e-6f && fabsf(dy_m) <= 1e-6f)
    {
        return;
    }

    /* 世界坐标�?�?车体系（与命令合成及菜单中的约定一致）*/
    yaw_rad = g_ctx.pose.yaw_deg * ((float)M_PI / 180.0f);
    cos_yaw = cosf(yaw_rad);
    sin_yaw = sinf(yaw_rad);

    dx_body =  dx_m * cos_yaw + dy_m * sin_yaw;
    dy_body = -dx_m * sin_yaw + dy_m * cos_yaw;
    main_axis = path_follow_resolve_segment_axis(dx_body, dy_body);

    /* compensate X component */
    if (fabsf(dx_body) >= PATH_DISTANCE_COMP_MIN_APPLY_M)
    {
        float abs_comp = path_follow_compensate_body_component_m(
                fabsf(dx_body),
                (dx_body >= 0.0f) ? PATH_DISTANCE_COMP_DIR_FWD : PATH_DISTANCE_COMP_DIR_BACK);
        comp_dx_body = (dx_body >= 0.0f) ? abs_comp : -abs_comp;
    }
    else
    {
        comp_dx_body = dx_body;
    }

    /* compensate Y component */
    if (fabsf(dy_body) >= PATH_DISTANCE_COMP_MIN_APPLY_M)
    {
        float abs_comp = path_follow_compensate_body_component_m(
                fabsf(dy_body),
                (dy_body >= 0.0f) ? PATH_DISTANCE_COMP_DIR_LEFT : PATH_DISTANCE_COMP_DIR_RIGHT);
        comp_dy_body = (dy_body >= 0.0f) ? abs_comp : -abs_comp;
    }
    else
    {
        comp_dy_body = dy_body;
    }

    /* 串轴距离补偿：在主方向距离补偿之后，额外给副轴一个固定目标量�?*/
    path_follow_apply_cross_axis_distance_compensation(main_axis,
                                                       dx_body,
                                                       dy_body,
                                                       &comp_dx_body,
                                                       &comp_dy_body);

    /* 车体�?�?世界坐标�?*/
    geometry->target_x_m = start_x_m + comp_dx_body * cos_yaw - comp_dy_body * sin_yaw;
    geometry->target_y_m = start_y_m + comp_dx_body * sin_yaw + comp_dy_body * cos_yaw;
}

/**
 * @brief 为当前周期准备路径几何信息�? *
 * 该函数会跳过已经到达的目标点，必要时自动推进到下一个拐点；
 * 若整条路径执行完毕，则在内部完成结束收尾�? *
 * @param geometry 几何信息输出指针�? * @param out 本周期控制输出指针，用于路径结束时回写状态�? * @return `1` 表示成功得到有效几何信息，`0` 表示当前无有效控制目标�? */
static uint8 path_follow_prepare_geometry(path_follow_geometry_t *geometry, path_follow_output_t *out)
{
    if (geometry == NULL)
    {
        return 0U;
    }

    while (g_ctx.active && g_ctx.path && g_ctx.idx < g_ctx.steps)
    {
        uint8 pause_target;
        uint8 geometry_ok;

        *geometry = (path_follow_geometry_t){0};
        path_follow_sync_pause_cursor(g_ctx.idx);
        geometry_ok = path_follow_fill_segment_geometry(geometry);
        pause_target = path_follow_target_requires_pause(g_ctx.idx);

        if (geometry_ok && !path_follow_segment_reached(geometry))
        {
            car_direction = geometry->segment_axis;
            return 1U;
        }

        if ((g_ctx.idx + 1U) < g_ctx.steps)
        {
            path_follow_remember_xy_at_target(geometry);
            if (pause_target)
            {
                path_follow_enter_pause(g_ctx.idx);
                return 0U;
            }
#if PATH_FOLLOW_ENABLE_CORNER_HANDOVER
            path_follow_reset_corner_handover_state();
#endif
            path_follow_invalidate_active_profile();
            g_ctx.idx++;
            continue;
        }

        if (pause_target)
        {
            path_follow_enter_pause(g_ctx.idx);
            return 0U;
        }
        path_follow_begin_terminal_yaw_settle();
        return 0U;
    }

    car_direction = 0U;
    return 0U;
}

/**
 * @brief 为当前路径段构造一次性的标准 S 曲线 profile�? *
 * @note 该函数只在进入新路径段、切换目标点�?profile 被显式失效时调用�? *       正常执行期不会重复重�?profile�? *
 * @param geometry 当前周期路径几何信息�? * @param speed_plan 速度规划结果输出，用于回填段末速度与安全速度上限�? * @return `1` 表示当前�?profile 构造成功，`0` 表示构造失败�? */
static uint8 path_follow_build_active_profile(const path_follow_geometry_t *geometry,
                                              path_follow_speed_plan_t *speed_plan)
{
    path_follow_scurve_profile_t profile = {0};
    path_follow_scurve_runtime_cfg_t selected_cfg = {0};
    float speed_distance_m;
    float start_speed_cmps;

    if (geometry == NULL || speed_plan == NULL)
    {
        return 0U;
    }

    speed_distance_m = path_follow_speed_distance_m(geometry);
    path_follow_select_scurve_band(speed_distance_m, &selected_cfg);
    g_ctx.active_scurve_cfg = selected_cfg;

    speed_plan->end_speed_cmps = fminf(path_follow_compute_segment_end_speed(),
                                       g_ctx.active_scurve_cfg.max_speed_cmps);
    speed_plan->safety_cap_cmps = path_follow_compute_brake_speed_cap(speed_distance_m,
                                                                      speed_plan->end_speed_cmps,
                                                                      g_ctx.active_scurve_cfg.max_speed_cmps,
                                                                      g_ctx.active_scurve_cfg.accel_cmpss);

    start_speed_cmps = fmaxf(g_ctx.last_ref_speed_cmps, 0.0f);
    start_speed_cmps = fminf(start_speed_cmps, speed_plan->safety_cap_cmps);

    if (!path_follow_build_scurve_profile(&profile,
                                          speed_distance_m * 100.0f,
                                          start_speed_cmps,
                                          speed_plan->end_speed_cmps,
                                          g_ctx.active_scurve_cfg.max_speed_cmps,
                                          g_ctx.active_scurve_cfg.accel_cmpss,
                                          g_ctx.active_scurve_cfg.jerk_cmpsss))
    {
        path_follow_invalidate_active_profile();
        return 0U;
    }

    g_ctx.active_profile = profile;
    g_ctx.profile_time_s = 0.0f;
    g_ctx.profile_target_idx = g_ctx.idx;
    g_ctx.profile_active = 1U;
    speed_plan->end_speed_cmps = profile.v1;
    return 1U;
}

/**
 * @brief 规划当前周期的标量速度参考�? *
 * 当前实现采用“整段一次规�?+ 按累计时间连续采样”的标准 S 曲线执行方式�? * 1. 仅在进入新路径段时构造一次完�?profile�? * 2. profile 内固定该段的起点速度、终点速度与约束；
 * 3. 每个控制周期只推进该段的累计时间并采样速度�? * 4. 当前剩余距离只作为安全监护层，不再参与主轨迹滚动重建�? * 5. 只有 profile 构造失败时，才退化到异常保护速度�? *
 * @param geometry 当前周期路径几何信息�? * @param speed_plan 速度规划结果输出�? */
static void path_follow_plan_speed(const path_follow_geometry_t *geometry,
                                   path_follow_speed_plan_t *speed_plan)
{
    const float sample_dt_s = PATH_FOLLOW_UPDATE_DT_S;
    uint8 build_ok = 1U;
    uint8 profile_sampled = 0U;
    float speed_distance_m;

    if (geometry == NULL || speed_plan == NULL)
    {
        return;
    }

    speed_distance_m = path_follow_speed_distance_m(geometry);
    speed_plan->end_speed_cmps = path_follow_compute_segment_end_speed();
    speed_plan->safety_cap_cmps = 0.0f;
    speed_plan->ref_speed_cmps = 0.0f;

    if (!g_ctx.profile_active || g_ctx.profile_target_idx != g_ctx.idx)
    {
        build_ok = path_follow_build_active_profile(geometry, speed_plan);
    }
    else
    {
        speed_plan->end_speed_cmps = g_ctx.active_profile.v1;
        speed_plan->safety_cap_cmps = path_follow_compute_brake_speed_cap(speed_distance_m,
                                                                          speed_plan->end_speed_cmps,
                                                                          g_ctx.active_scurve_cfg.max_speed_cmps,
                                                                          g_ctx.active_scurve_cfg.accel_cmpss);
    }

    if (g_ctx.profile_active)
    {
        g_ctx.profile_time_s += sample_dt_s;
        if (g_ctx.profile_time_s > g_ctx.active_profile.T)
        {
            g_ctx.profile_time_s = g_ctx.active_profile.T;
        }

        speed_plan->ref_speed_cmps = path_follow_sample_scurve_velocity(g_ctx.profile_time_s,
                                                                        &g_ctx.active_profile);
        profile_sampled = 1U;
        speed_plan->end_speed_cmps = g_ctx.active_profile.v1;
        speed_plan->safety_cap_cmps = path_follow_compute_brake_speed_cap(speed_distance_m,
                                                                          speed_plan->end_speed_cmps,
                                                                          g_ctx.active_scurve_cfg.max_speed_cmps,
                                                                          g_ctx.active_scurve_cfg.accel_cmpss);
    }
    else if (!build_ok)
    {
        speed_plan->ref_speed_cmps = path_follow_compute_profile_fault_speed(g_ctx.last_ref_speed_cmps,
                                                                             speed_plan->safety_cap_cmps,
                                                                             g_ctx.active_scurve_cfg.accel_cmpss);
    }

    speed_plan->ref_speed_cmps = clamp_sym(speed_plan->ref_speed_cmps,
                                           g_ctx.active_scurve_cfg.max_speed_cmps);
    if (!profile_sampled)
    {
        speed_plan->ref_speed_cmps = fminf(speed_plan->ref_speed_cmps,
                                            speed_plan->safety_cap_cmps);
    }
    speed_plan->ref_speed_cmps = fmaxf(speed_plan->ref_speed_cmps, 0.0f);

    g_ctx.last_ref_speed_cmps = speed_plan->ref_speed_cmps;
}


/**
 * @brief 判断当前目标点是否属于普通路线中的中间拐点�? *
 * @note 这里显式排除启动偏移/位姿修正使用的临时细网格路径，只在正常地图路径上生效�? *
 * @return `1` 表示当前目标是普通路线中仍有后续段的拐点，`0` 表示不是�? */
static uint8 path_follow_is_route_corner_target(void)
{
    const float grid_eps_m = 1e-6f;
    Point prev_point;
    Point curr_point;
    Point next_point;
    int d1_row;
    int d1_col;
    int d2_row;
    int d2_col;

    if (g_ctx.path == NULL || g_ctx.steps < 3U)
    {
        return 0U;
    }
    if (g_ctx.idx == 0U || (g_ctx.idx + 1U) >= g_ctx.steps)
    {
        return 0U;
    }
    if (fabsf(g_ctx.grid_m - g_ctx.default_grid_m) > grid_eps_m)
    {
        return 0U;
    }

    prev_point = g_ctx.path[g_ctx.idx - 1U];
    curr_point = g_ctx.path[g_ctx.idx];
    next_point = g_ctx.path[g_ctx.idx + 1U];

    d1_row = curr_point.row - prev_point.row;
    d1_col = curr_point.col - prev_point.col;
    d2_row = next_point.row - curr_point.row;
    d2_col = next_point.col - curr_point.col;

    return (d1_row != d2_row || d1_col != d2_col) ? 1U : 0U;
}

#if PATH_FOLLOW_ENABLE_CORNER_HANDOVER
/**
 * @brief [CornerHandover移植] �?0~1 线性进度整形成五次 smoothstep 进度�? *
 * @param t 线性进度，范围期望�?[0, 1]�? * @return S 形整形后的进度，范围�?[0, 1]�? */
static float path_follow_s_curve_01(float t)
{
    if (t <= 0.0f)
    {
        return 0.0f;
    }
    if (t >= 1.0f)
    {
        return 1.0f;
    }

    return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
}

/**
 * @brief [CornerHandover重构] 在普通中间拐点前平滑交接世界系速度方向�? *
 * @note 这里不改变原有段�?S 曲线的标量速度规划，只在普通中间拐点上
 *       按上一段投影进度进�?handover，并在切段前保持该运行态锁存�? *
 * @param geometry 当前周期路径几何信息�? * @param ref_speed_cmps 当前周期标量速度参考，单位 cm/s�? * @param vx_world_cmps 世界�?X 方向速度输出�? * @param vy_world_cmps 世界�?Y 方向速度输出�? * @return `1` 表示本周期已应用拐点交接，`0` 表示未应用�? */
static uint8 path_follow_try_corner_handover(const path_follow_geometry_t *geometry,
                                             float ref_speed_cmps,
                                             float *vx_world_cmps,
                                             float *vy_world_cmps)
{
    const float eps = 1e-6f;
    const Point *prev_point = NULL;
    const Point *curr_point = NULL;
    const Point *next_point = NULL;
    float segment_progress_m = 0.0f;
    float segment_length_m = 0.0f;
    float lateral_error_m = 0.0f;
    float remaining_along_m;
    float prev_dx_m;
    float prev_dy_m;
    float next_dx_m;
    float next_dy_m;
    float prev_length_m;
    float next_length_m;
    float enter_distance_m;
    float commit_distance_m;
    float t;
    float k;
    float v_scale;
    float turn_cos;
    float turn_severity;
    float k_drop;
    float old_x;
    float old_y;
    float new_x;
    float new_y;
    float mix_x;
    float mix_y;
    float mix_norm;

    if (geometry == NULL || vx_world_cmps == NULL || vy_world_cmps == NULL)
    {
        return 0U;
    }
    if (!path_follow_is_route_corner_target())
    {
        return 0U;
    }
    if (path_follow_target_requires_pause(g_ctx.idx))
    {
        return 0U;
    }
    if (!path_follow_get_segment_points(&prev_point, &curr_point, &next_point) ||
        prev_point == NULL || curr_point == NULL || next_point == NULL)
    {
        return 0U;
    }
    if (!path_follow_compute_segment_progress_m(&g_ctx.pose,
                                                prev_point,
                                                curr_point,
                                                &segment_progress_m,
                                                &segment_length_m,
                                                &lateral_error_m))
    {
        return 0U;
    }
    (void)lateral_error_m;

    prev_dx_m = (curr_point->row - prev_point->row) * g_ctx.grid_m;
    prev_dy_m = (curr_point->col - prev_point->col) * g_ctx.grid_m;
    next_dx_m = (next_point->row - curr_point->row) * g_ctx.grid_m;
    next_dy_m = (next_point->col - curr_point->col) * g_ctx.grid_m;
    prev_length_m = sqrtf(prev_dx_m * prev_dx_m + prev_dy_m * prev_dy_m);
    next_length_m = sqrtf(next_dx_m * next_dx_m + next_dy_m * next_dy_m);
    if (prev_length_m <= eps || next_length_m <= eps)
    {
        return 0U;
    }

    enter_distance_m = path_follow_compute_corner_enter_distance_m(prev_length_m, next_length_m);
    commit_distance_m = path_follow_compute_corner_commit_distance_m(prev_length_m, next_length_m);
    if (enter_distance_m <= (commit_distance_m + PATH_CORNER_HANDOVER_WINDOW_GAP_MIN_M))
    {
        enter_distance_m = commit_distance_m + PATH_CORNER_HANDOVER_WINDOW_GAP_MIN_M;
    }

    if (g_ctx.corner_handover_active)
    {
        if (g_ctx.corner_handover_idx != g_ctx.idx)
        {
            path_follow_reset_corner_handover_state();
        }
        else
        {
            if (g_ctx.corner_enter_distance_m > 0.0f)
            {
                enter_distance_m = g_ctx.corner_enter_distance_m;
            }
            if (g_ctx.corner_commit_distance_m > 0.0f)
            {
                commit_distance_m = g_ctx.corner_commit_distance_m;
            }
        }
    }
    if (!g_ctx.corner_handover_active)
    {
        g_ctx.corner_handover_idx = g_ctx.idx;
        g_ctx.corner_enter_distance_m = enter_distance_m;
        g_ctx.corner_commit_distance_m = commit_distance_m;
    }
    if (enter_distance_m <= (commit_distance_m + PATH_CORNER_HANDOVER_WINDOW_GAP_MIN_M))
    {
        enter_distance_m = commit_distance_m + PATH_CORNER_HANDOVER_WINDOW_GAP_MIN_M;
    }

    remaining_along_m = segment_length_m - segment_progress_m;
    if (remaining_along_m < 0.0f)
    {
        remaining_along_m = 0.0f;
    }

    if (!g_ctx.corner_handover_active)
    {
        if (remaining_along_m >= enter_distance_m)
        {
            return 0U;
        }
        g_ctx.corner_handover_active = 1U;
    }

    old_x = geometry->dir_x;
    old_y = geometry->dir_y;
    new_x = next_dx_m / next_length_m;
    new_y = next_dy_m / next_length_m;
    turn_cos = old_x * new_x + old_y * new_y;
    turn_cos = fminf(fmaxf(turn_cos, -1.0f), 1.0f);
    /* 直行时为 0�?0 度及以上拐角�?1 处理�?*/
    turn_severity = fminf(fmaxf(1.0f - turn_cos, 0.0f), 1.0f);

    t = (enter_distance_m - remaining_along_m) / (enter_distance_m - commit_distance_m);
    t = fminf(fmaxf(t, 0.0f), 1.0f);
    k = path_follow_s_curve_01(t);
    k_drop = PATH_CORNER_HANDOVER_K_DROP * turn_severity;
    v_scale = 1.0f - k_drop * 4.0f * k * (1.0f - k);
    v_scale = fmaxf(v_scale, 0.0f);

    mix_x = old_x * (1.0f - k) + new_x * k;
    mix_y = old_y * (1.0f - k) + new_y * k;
    mix_norm = sqrtf(mix_x * mix_x + mix_y * mix_y);
    if (mix_norm <= eps)
    {
        return 0U;
    }

    mix_x /= mix_norm;
    mix_y /= mix_norm;
    *vx_world_cmps = ref_speed_cmps * v_scale * mix_x;
    *vy_world_cmps = ref_speed_cmps * v_scale * mix_y;
    return 1U;
}
#endif

static void path_follow_apply_diagonal_command_comp(const path_follow_geometry_t *geometry,
                                                    path_follow_motion_cmd_t *motion_cmd)
{
    /* 斜向命令补偿归零：保持原�?vx/vy 比例，重新校准前不做额外增减�?*/
    (void)geometry;
    (void)motion_cmd;
}


static void path_follow_build_path_coordinate_speed(const path_follow_speed_plan_t *speed_plan,
                                                     float *path_speed_cmps)
{
    float max_path_speed_cmps;

    if (path_speed_cmps == NULL)
    {
        return;
    }

    *path_speed_cmps = 0.0f;

    if (speed_plan == NULL)
    {
        return;
    }

    max_path_speed_cmps = g_ctx.active_scurve_cfg.max_speed_cmps;
    if (max_path_speed_cmps <= 0.0f)
    {
        max_path_speed_cmps = g_ctx.max_v_mps * 100.0f;
    }

    *path_speed_cmps = clamp_sym(speed_plan->ref_speed_cmps, max_path_speed_cmps);
}

static void path_follow_limit_world_speed(path_follow_motion_cmd_t *motion_cmd)
{
    float max_speed_cmps;
    float speed_norm;
    float scale;

    if (motion_cmd == NULL)
    {
        return;
    }

    max_speed_cmps = g_ctx.active_scurve_cfg.max_speed_cmps;
    if (max_speed_cmps <= 0.0f)
    {
        max_speed_cmps = path_follow_mps_to_cmps(g_path_follow_scurve_band_default_cfg[PATH_FOLLOW_SCURVE_BAND_COUNT - 1U].vmax_mps);
    }

    speed_norm = sqrtf(motion_cmd->vx_world_cmps * motion_cmd->vx_world_cmps +
                       motion_cmd->vy_world_cmps * motion_cmd->vy_world_cmps);
    if (speed_norm <= max_speed_cmps || speed_norm <= 0.0f)
    {
        return;
    }

    scale = max_speed_cmps / speed_norm;
    motion_cmd->vx_world_cmps *= scale;
    motion_cmd->vy_world_cmps *= scale;
}

static void path_follow_transform_world_to_body(float yaw_deg,
                                                path_follow_motion_cmd_t *motion_cmd)
{
    float world_yaw_deg;
    float yaw_rad;
    float cos_yaw;
    float sin_yaw;

    if (motion_cmd == NULL)
    {
        return;
    }

    world_yaw_deg = path_follow_world_yaw_from_imu(yaw_deg);
    yaw_rad = world_yaw_deg * ((float)M_PI / 180.0f);
    cos_yaw = cosf(yaw_rad);
    sin_yaw = sinf(yaw_rad);

    motion_cmd->vx_body_cmps = motion_cmd->vx_world_cmps * cos_yaw +
                               motion_cmd->vy_world_cmps * sin_yaw;
    motion_cmd->vy_body_cmps = -motion_cmd->vx_world_cmps * sin_yaw +
                               motion_cmd->vy_world_cmps * cos_yaw;
}

/**
 * @brief 根据几何方向和标量速度生成运动命令�? *
 * @param geometry 当前周期路径几何信息�? * @param speed_plan 当前周期速度规划结果�? * @param yaw_deg 当前航向角，单位 deg�? * @param motion_cmd 运动命令输出�? */
static void path_follow_build_motion_command(const path_follow_geometry_t *geometry,
                                             const path_follow_speed_plan_t *speed_plan,
                                             float yaw_deg,
                                             path_follow_motion_cmd_t *motion_cmd)
{
    float path_speed_cmps;

    if (geometry == NULL || speed_plan == NULL || motion_cmd == NULL)
    {
        return;
    }

    path_follow_build_path_coordinate_speed(speed_plan,
                                             &path_speed_cmps);

    motion_cmd->vx_world_cmps = path_speed_cmps * geometry->dir_x;
    motion_cmd->vy_world_cmps = path_speed_cmps * geometry->dir_y;

#if PATH_FOLLOW_ENABLE_CORNER_HANDOVER
    /*
     * Corner handover is disabled by default. If enabled later, it still only changes
     * the main path direction near a normal route corner; scalar path speed remains
     * the primary motion synthesis path.
     */
    (void)path_follow_try_corner_handover(geometry,
                                          path_speed_cmps,
                                          &motion_cmd->vx_world_cmps,
                                          &motion_cmd->vy_world_cmps);
#endif

    path_follow_apply_diagonal_command_comp(geometry, motion_cmd);
    path_follow_limit_world_speed(motion_cmd);

    path_follow_transform_world_to_body(yaw_deg, motion_cmd);
}

/**
 * @brief 规划当前周期的姿态控制输出�? *
 * @param yaw_deg 当前航向角，单位 deg�? * @param attitude_plan 姿态规划结果输出�? */
static void path_follow_plan_attitude(float yaw_deg, path_follow_attitude_plan_t *attitude_plan)
{
    float world_yaw_deg;
    float omega_cmd_degps;
    float yaw_error_deg;

    if (attitude_plan == NULL)
    {
        return;
    }

    attitude_plan->target_yaw_deg = g_ctx.target_yaw_deg;
    if (g_ctx.heading_mode != PATH_FOLLOW_HEADING_FIXED)
    {
        attitude_plan->target_yaw_deg = 0.0f;
    }

    attitude_plan->target_yaw_deg = path_follow_wrap_deg(attitude_plan->target_yaw_deg);
    world_yaw_deg = path_follow_world_yaw_from_imu(yaw_deg);
    yaw_error_deg = path_follow_yaw_error_deg(world_yaw_deg, attitude_plan->target_yaw_deg);
    if (fabsf(yaw_error_deg) < PATH_FOLLOW_YAW_PID_DEADBAND_DEG)
    {
        omega_cmd_degps = 0.0f;
        PID_Clear(&pid_yaw);
    }
    else
    {
        omega_cmd_degps = (float)PID_Location_Calculate(&pid_yaw, 0.0f, yaw_error_deg);
        omega_cmd_degps = clamp_sym(omega_cmd_degps, g_ctx.max_w_rad);
    }
    omega_cmd_degps = path_follow_apply_yaw_omega_slew(omega_cmd_degps);
    attitude_plan->omega_ref_degps = omega_cmd_degps;
    attitude_plan->omega_cmd_radps = omega_cmd_degps * ((float)M_PI / 180.0f);
}

static uint8 path_follow_handle_terminal_yaw_settle(float yaw_deg, path_follow_output_t *out)
{
    path_follow_attitude_plan_t attitude_plan = {0};
    float world_yaw_deg;
    float target_yaw_deg;
    float yaw_error_deg;
    uint8 yaw_in_tol;
    uint8 wheels_in_tol;

    if (!g_ctx.terminal_yaw_settle_active)
    {
        return 0U;
    }

    /* Entering this state already guarantees that the final S-curve time has ended. */
    path_follow_plan_attitude(yaw_deg, &attitude_plan);
    world_yaw_deg = path_follow_world_yaw_from_imu(yaw_deg);
    target_yaw_deg = g_ctx.target_yaw_deg;
    if (g_ctx.heading_mode != PATH_FOLLOW_HEADING_FIXED)
    {
        target_yaw_deg = 0.0f;
    }
    target_yaw_deg = path_follow_wrap_deg(target_yaw_deg);
    yaw_error_deg = path_follow_yaw_error_deg(world_yaw_deg, target_yaw_deg);

    yaw_in_tol = (fabsf(yaw_error_deg) <= g_ctx.yaw_tol_deg) ? 1U : 0U;
    wheels_in_tol = motor_wheel_feedback_is_settled();

    if (yaw_in_tol && wheels_in_tol)
    {
        if (g_ctx.terminal_yaw_in_tol_cycles < PATH_FOLLOW_FINAL_REACHED_SETTLE_CYCLES)
        {
            g_ctx.terminal_yaw_in_tol_cycles++;
        }
    }
    else
    {
        g_ctx.terminal_yaw_in_tol_cycles = 0U;
    }

    if (g_ctx.terminal_yaw_settle_cycles < 0xFFFFFFFFU)
    {
        g_ctx.terminal_yaw_settle_cycles++;
    }

    path_follow_clear_debug_state();
    g_ctx.debug.target_yaw_deg = attitude_plan.target_yaw_deg;
    g_ctx.debug.omega_cmd_radps = attitude_plan.omega_cmd_radps;
    g_ctx.debug.omega_ref_degps = attitude_plan.omega_ref_degps;
    path_follow_store_motion_plan(0.0f,
                                  0.0f,
                                  0.0f,
                                  0.0f,
                                  attitude_plan.omega_cmd_radps,
                                  0U,
                                  0.0f);
    car_direction = 0U;

    if (g_ctx.terminal_yaw_in_tol_cycles >= PATH_FOLLOW_FINAL_REACHED_SETTLE_CYCLES)
    {
        (void)path_follow_finish_path(out, NULL);
        return 1U;
    }

    if (out != NULL)
    {
        out->vx_cmd = 0.0f;
        out->vy_cmd = 0.0f;
        out->omega_cmd = attitude_plan.omega_cmd_radps;
        out->active = 1U;
        out->reached = 0U;
        out->target_idx = g_ctx.idx;
    }
    return 1U;
}

static void path_follow_plan_yaw_rate_debug(path_follow_attitude_plan_t *attitude_plan)
{
    float actual_omega_degps;
    float omega_cmd_degps;

    if (attitude_plan == NULL)
    {
        return;
    }

    attitude_plan->target_yaw_deg = g_ctx.pose.yaw_deg;
    actual_omega_degps = path_follow_actual_yaw_rate_degps();
    omega_cmd_degps = (float)PID_Location_Calculate(&pid_accel_yaw,
                                                    actual_omega_degps,
                                                    g_ctx.yaw_rate_target_degps);
    omega_cmd_degps = path_follow_apply_yaw_omega_slew(omega_cmd_degps);
    attitude_plan->omega_ref_degps = g_ctx.yaw_rate_target_degps;
    attitude_plan->omega_cmd_radps = omega_cmd_degps * ((float)M_PI / 180.0f);
}

/**
 * @brief 保存当前周期的调试状态快照�? *
 * @param geometry 当前周期路径几何信息�? * @param speed_plan 当前周期速度规划结果�? * @param attitude_plan 当前周期姿态规划结果�? * @param motion_cmd 当前周期运动命令�? */
static void path_follow_store_debug_state(const path_follow_geometry_t *geometry,
                                          const path_follow_speed_plan_t *speed_plan,
                                          const path_follow_attitude_plan_t *attitude_plan,
                                          const path_follow_motion_cmd_t *motion_cmd)
{
    if (geometry == NULL || speed_plan == NULL || attitude_plan == NULL || motion_cmd == NULL)
    {
        path_follow_clear_debug_state();
        path_follow_clear_motion_telemetry();
        return;
    }

    g_ctx.debug.distance_m = geometry->distance_m;
    g_ctx.debug.dir_x = geometry->dir_x;
    g_ctx.debug.dir_y = geometry->dir_y;
    g_ctx.debug.segment_start_x_m = geometry->segment_start_x_m;
    g_ctx.debug.segment_start_y_m = geometry->segment_start_y_m;
    g_ctx.debug.segment_dir_x = geometry->segment_dir_x;
    g_ctx.debug.segment_dir_y = geometry->segment_dir_y;
    g_ctx.debug.segment_normal_x = geometry->segment_normal_x;
    g_ctx.debug.segment_normal_y = geometry->segment_normal_y;
    g_ctx.debug.segment_length_m = geometry->segment_length_m;
    g_ctx.debug.segment_progress_m = geometry->segment_progress_m;
    g_ctx.debug.segment_remaining_m = geometry->segment_remaining_m;
    g_ctx.debug.lateral_error_m = geometry->lateral_error_m;
    g_ctx.debug.speed_ref_cmps = speed_plan->ref_speed_cmps;
    g_ctx.debug.target_yaw_deg = attitude_plan->target_yaw_deg;
    g_ctx.debug.omega_cmd_radps = attitude_plan->omega_cmd_radps;
    g_ctx.debug.omega_ref_degps = attitude_plan->omega_ref_degps;
    g_ctx.debug.vx_world_cmps = motion_cmd->vx_world_cmps;
    g_ctx.debug.vy_world_cmps = motion_cmd->vy_world_cmps;
    g_ctx.debug.segment_axis = geometry->segment_axis;
    path_follow_store_motion_plan(geometry->distance_m,
                                  speed_plan->ref_speed_cmps,
                                  motion_cmd->vx_world_cmps,
                                  motion_cmd->vy_world_cmps,
                                  attitude_plan->omega_cmd_radps,
                                  geometry->segment_axis,
                                  geometry->lateral_error_m);
}

/**
 * @brief 路径跟随主更新函数�? *
 * 该函数是路径跟随控制链的主入口，每个控制周期调用一次�? * 它会完成位姿更新、目标点推进、几何分析、速度规划、姿态规划以及车体系输出合成�? *
 * @param yaw_deg 当前航向角，单位 deg�? * @param out 输出结构体，返回当前周期车体系速度命令与状态标志�? */
void path_follow_update(float yaw_deg, path_follow_output_t *out)
{
    path_follow_geometry_t geometry = {0};
    path_follow_speed_plan_t speed_plan = {0};
    path_follow_attitude_plan_t attitude_plan = {0};
    path_follow_motion_cmd_t motion_cmd = {0};
    float yaw_error_deg = 0.0f;

    g_ctx.pose.yaw_deg = path_follow_world_yaw_from_imu(yaw_deg);

    if (out)
    {
        out->active = 0;
        out->reached = 0;
        out->vx_cmd = 0;
        out->vy_cmd = 0;
        out->omega_cmd = 0;
        out->target_idx = g_ctx.idx;
    }

    if ((!g_ctx.active || NULL == g_ctx.path || 0 == g_ctx.steps) &&
        !g_ctx.terminal_yaw_settle_active &&
        !g_ctx.rotate_only_active &&
        !g_ctx.yaw_rate_only_active)
    {
        path_follow_clear_debug_state();
        path_follow_store_motion_plan(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0U, 0.0f);
        car_direction = 0U;
        return;
    }

    if (g_ctx.yaw_rate_only_active)
    {
        path_follow_plan_yaw_rate_debug(&attitude_plan);
        path_follow_clear_debug_state();
        g_ctx.debug.target_yaw_deg = g_ctx.pose.yaw_deg;
        g_ctx.debug.omega_cmd_radps = attitude_plan.omega_cmd_radps;
        g_ctx.debug.omega_ref_degps = attitude_plan.omega_ref_degps;
        path_follow_store_motion_plan(0.0f,
                                      0.0f,
                                      0.0f,
                                      0.0f,
                                      attitude_plan.omega_cmd_radps,
                                      0U,
                                      0.0f);
        car_direction = 0U;

        if (out)
        {
            out->vx_cmd = 0.0f;
            out->vy_cmd = 0.0f;
            out->omega_cmd = attitude_plan.omega_cmd_radps;
            out->active = 1U;
        }
        return;
    }

    if (g_ctx.rotate_only_active)
    {
        path_follow_plan_attitude(yaw_deg, &attitude_plan);
        yaw_error_deg = path_follow_yaw_error_deg(g_ctx.pose.yaw_deg,
                                                  attitude_plan.target_yaw_deg);

        path_follow_clear_debug_state();
        g_ctx.debug.target_yaw_deg = attitude_plan.target_yaw_deg;
        g_ctx.debug.omega_cmd_radps = attitude_plan.omega_cmd_radps;
        g_ctx.debug.omega_ref_degps = attitude_plan.omega_ref_degps;
        path_follow_store_motion_plan(0.0f,
                                      0.0f,
                                      0.0f,
                                      0.0f,
                                      attitude_plan.omega_cmd_radps,
                                      0U,
                                      0.0f);
        car_direction = 0U;

        if (fabsf(yaw_error_deg) <= g_ctx.yaw_tol_deg)
        {
            if (g_ctx.rotate_in_tol_cycles < PATH_FOLLOW_ROTATE_SETTLE_CYCLES)
            {
                g_ctx.rotate_in_tol_cycles++;
            }
            if (g_ctx.rotate_in_tol_cycles >= PATH_FOLLOW_ROTATE_SETTLE_CYCLES &&
                !g_ctx.rotate_hold_after_reach)
            {
                g_ctx.rotate_only_active = 0U;
                g_ctx.rotate_hold_after_reach = 0U;
                PID_Clear(&pid_yaw);
                path_follow_reset_control_state();
                if (out)
                {
                    out->reached = 1U;
                    out->active = 0U;
                }
                return;
            }
        }
        else
        {
            g_ctx.rotate_in_tol_cycles = 0U;
        }

        if (out)
        {
            out->vx_cmd = 0.0f;
            out->vy_cmd = 0.0f;
            out->omega_cmd = attitude_plan.omega_cmd_radps;
            out->active = 1U;
        }
        return;
    }

    if (path_follow_handle_terminal_yaw_settle(yaw_deg, out))
    {
        return;
    }

    if (path_follow_handle_pause(out))
    {
        return;
    }

    if (!path_follow_prepare_geometry(&geometry, out))
    {
        if (path_follow_handle_pause(out))
        {
            return;
        }
        if (path_follow_handle_terminal_yaw_settle(yaw_deg, out))
        {
            return;
        }
        return;
    }

    path_follow_plan_speed(&geometry, &speed_plan);
    path_follow_build_motion_command(&geometry, &speed_plan, yaw_deg, &motion_cmd);
    path_follow_plan_attitude(yaw_deg, &attitude_plan);
    path_follow_store_debug_state(&geometry, &speed_plan, &attitude_plan, &motion_cmd);

    if (out)
    {
        out->vx_cmd = motion_cmd.vx_body_cmps;
        out->vy_cmd = motion_cmd.vy_body_cmps;
        out->omega_cmd = attitude_plan.omega_cmd_radps;
        out->active = 1U;
        out->target_idx = g_ctx.idx;
    }
}

/**
 * @brief 路径跟随测试入口�? *
 * @param yaw_deg 当前航向角，单位 deg�? * @param out 输出结构体�? *
 * @note 当前实现直接复用正式更新函数�? */
void path_follow_update_test(float yaw_deg, path_follow_output_t *out)
{
    path_follow_update(yaw_deg, out);
}

/**
 * @brief 将路径跟随输出接入到底盘运动学逆解�? *
 * 该函数负责把 `path_follow_update()` 输出的三轴速度命令
 * 写入全局速度缓存，并进一步生成四轮目标编码器速度�? */
void distance_speed_strategy(void)
{
        // 车体系速度单轴测试：单�?cm/s，omega 单位 rad/s
    // #if 1
    //     speed_three_array[0] = 0.0f;  // Vx�? 前进�? 后退
    //     speed_three_array[1] = 200.0f;   // Vy�? 左移�? 右移
    //     speed_three_array[2] = 0.0f;   // omega�?/- 旋转

    //     Kinematics_Inverse(speed_three_array, speed_encoder);
    //     return;
    // #endif
    path_follow_output_t pf = {0};

    path_follow_update(eulerAngle.yaw, &pf);

    if (pf.active)
    {
        speed_three_array[0] = pf.vx_cmd;
        speed_three_array[1] = pf.vy_cmd;
        speed_three_array[2] = pf.omega_cmd;
    }
    else
    {
        speed_three_array[0] = 0.0f;
        speed_three_array[1] = 0.0f;
        speed_three_array[2] = 0.0f;
    }

    Kinematics_Inverse(speed_three_array, speed_encoder);
}

/**
 * @brief 获取路径跟随模块当前状态�? *
 * @param status 状态输出结构体�? */
void path_follow_get_status(path_follow_status_t *status)
{
    if (NULL == status)
    {
        return;
    }

    status->x_m = g_ctx.pose.x_m;
    status->y_m = g_ctx.pose.y_m;
    status->yaw_deg = g_ctx.pose.yaw_deg;
    status->active = (g_ctx.active ||
                      g_ctx.terminal_yaw_settle_active ||
                      g_ctx.rotate_only_active ||
                      g_ctx.yaw_rate_only_active) ? 1U : 0U;
    status->reached = (g_ctx.motion_reached || status->active == 0U) ? 1U : 0U;
    status->paused = g_ctx.paused;
    status->yaw_only_active = (g_ctx.rotate_only_active ||
                               g_ctx.yaw_rate_only_active ||
                               g_ctx.terminal_yaw_settle_active) ? 1U : 0U;
    status->holding_position = 0U;
    status->target_idx = g_ctx.idx;
    status->distance_m = g_ctx.debug.distance_m;
    status->dir_x = g_ctx.debug.dir_x;
    status->dir_y = g_ctx.debug.dir_y;
    status->segment_start_x_m = g_ctx.debug.segment_start_x_m;
    status->segment_start_y_m = g_ctx.debug.segment_start_y_m;
    status->segment_dir_x = g_ctx.debug.segment_dir_x;
    status->segment_dir_y = g_ctx.debug.segment_dir_y;
    status->segment_normal_x = g_ctx.debug.segment_normal_x;
    status->segment_normal_y = g_ctx.debug.segment_normal_y;
    status->segment_length_m = g_ctx.debug.segment_length_m;
    status->segment_progress_m = g_ctx.debug.segment_progress_m;
    status->segment_remaining_m = g_ctx.debug.segment_remaining_m;
    status->lateral_error_m = g_ctx.debug.lateral_error_m;
    status->speed_ref_cmps = g_ctx.debug.speed_ref_cmps;
    status->segment_axis = g_ctx.debug.segment_axis;
    if (g_ctx.yaw_rate_only_active)
    {
        status->target_yaw_deg = g_ctx.pose.yaw_deg;
        status->yaw_error_deg = 0.0f;
        status->omega_ref = g_ctx.yaw_rate_target_degps * ((float)M_PI / 180.0f);
    }
    else
    {
        status->target_yaw_deg = g_ctx.target_yaw_deg;
        status->yaw_error_deg = path_follow_yaw_error_deg(g_ctx.pose.yaw_deg,
                                                          g_ctx.target_yaw_deg);
        status->omega_ref = g_ctx.debug.omega_ref_degps * ((float)M_PI / 180.0f);
    }
    status->omega_cmd = g_ctx.debug.omega_cmd_radps;

    if (g_ctx.path && g_ctx.idx < g_ctx.steps)
    {
        status->target_x_m = g_ctx.path[g_ctx.idx].row * g_ctx.grid_m;
        status->target_y_m = g_ctx.path[g_ctx.idx].col * g_ctx.grid_m;
    }
    else
    {
        status->target_x_m = 0.0f;
        status->target_y_m = 0.0f;
    }
}

void path_follow_get_motion_telemetry(path_follow_motion_telemetry_t *telemetry)
{
    if (telemetry == NULL)
    {
        return;
    }

    *telemetry = g_ctx.telemetry;
}

/**
 * @brief �?IPS200 屏幕上显示当前路径跟随状态�? */
void path_follow_draw_status(void)
{
    path_follow_status_t st = {0};
    path_follow_get_status(&st);

    // ips200_show_string(x, y, "Pose x y yaw:");
    ips200_show_string(0,112,"st_x_m");
    ips200_show_float(70, 112, st.x_m, 2, 4);
    ips200_show_string(0,128,"st_y_m");
    ips200_show_float(70, 128, st.y_m, 2, 4);
    ips200_show_string(0,144,"st_yaw");
    ips200_show_float(70, 144, st.yaw_deg, 3, 2);
    ips200_show_string(128,144,"imu");
    ips200_show_float(168, 144, eulerAngle.yaw, 3, 2);

    // ips200_show_string(x, y + 16, "Target idx/x/y:");
    ips200_show_string(0,160,"target_idx");
    ips200_show_uint(90, 160, st.target_idx, 3);
    ips200_show_string(0,176,"target_x_m");
    ips200_show_float(90, 176, st.target_x_m, 2, 3);
    ips200_show_string(0,192,"target_y_m");
    ips200_show_float(90, 192, st.target_y_m, 2, 3);
    ips200_show_string(0,208,"speed_ref");
    ips200_show_float(90, 208, st.speed_ref_cmps, 3, 1);
    ips200_show_string(0,224,"seg_axis");
    ips200_show_uint(90, 224, st.segment_axis, 1);

}

/**
 * @brief 计算两个离散点之间的朝向角�? *
 * @param from 起点�? * @param to 终点�? * @return 从起点指向终点的角度，单�?deg�? */
float path_follow_heading_deg(Point from, Point to)
{
    float dx = (float)(to.row - from.row);
    float dy = (float)(to.col - from.col);
    if (dx == 0.0f && dy == 0.0f)
    {
        return 0.0f;
    }
    float angle_rad = atan2f(dy, dx);
    return angle_rad * (180.0f / (float)M_PI);
}

/**
 * @brief 从完整离散路径中提取拐点序列�? *
 * @param path 原始离散路径�? * @param path_steps 原始路径点数量�? * @param corner_buffer 拐点输出缓存�? * @param corner_capacity 拐点缓存容量�? * @return 成功提取到的拐点数量�? */
size_t path_follow_extract_corners(const Point *path, size_t path_steps,
                                   Point *corner_buffer, size_t corner_capacity)
{
    size_t corner_count;
    size_t i;

    if (path == NULL || path_steps == 0U || corner_buffer == NULL || corner_capacity == 0U)
    {
        return 0U;
    }

    if (path_steps == 1U)
    {
        corner_buffer[0] = path[0];
        return 1U;
    }

    corner_count = 0U;
    if (corner_count >= corner_capacity)
    {
        return 0U;
    }
    corner_buffer[corner_count++] = path[0];

    for (i = 1U; i < path_steps - 1U; ++i)
    {
        int dx1 = path[i].col - path[i - 1U].col;
        int dy1 = path[i].row - path[i - 1U].row;
        int dx2 = path[i + 1U].col - path[i].col;
        int dy2 = path[i + 1U].row - path[i].row;

        if (dx1 != dx2 || dy1 != dy2)
        {
            if (corner_count >= corner_capacity)
            {
                return 0U;
            }
            corner_buffer[corner_count++] = path[i];
        }
    }

    if (corner_count >= corner_capacity)
    {
        return 0U;
    }
    corner_buffer[corner_count++] = path[path_steps - 1U];

    return corner_count;
}
