/** 六轴姿态解算：标定、Mahony/Madgwick、四元数与欧拉角。
 * 文件名按需求保留 attitiude。配置继续使用原 config.h，避免改变工程参数。
 * 输入：角速度 rad/s，加速度 g，时间 s；输出角度 deg。
 */
#ifndef MCAR_ATTITIUDE_H
#define MCAR_ATTITIUDE_H
#include "config.h"
#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    unsigned count; /* 连续合格静止样本数 */
    float sum_g[3], sum_a[3]; /* 角速度/加速度累加和 */
    float min_g[3], max_g[3]; /* 角速度窗口极值，deg/s */
    float min_a[3], max_a[3]; /* 加速度窗口极值，g */
    float bias_dps[3], initial_accel[3]; /* 陀螺静止零偏和初始重力 */
    float bias_z_ref_dps; /* 标定完成时的 z 零偏，跟踪限幅基准 */
} ImuCalibration;

typedef struct {
    float q[4]; /* w,x,y,z，传感器坐标到参考坐标 */
    float integral[3]; /* PI 积分角速度修正，rad/s */
    float accel_norm; /* 当前加速度模长，g */
    int accel_used; /* 1=加速度参与融合；0=门限拒绝 */
} Mahony6;
typedef struct {
    float q[4];
    float accel_norm; /* 当前加速度模长，g */
    int accel_used; /* 1=加速度参与融合；0=门限拒绝 */
} Madgwick6;

/* 运动时丢弃整段标定窗口；收满样本返回 1。 */
void calibration_reset(ImuCalibration *c);
int calibration_push(ImuCalibration *c, const float gyro_dps[3], const float accel_g[3]);
/* 深度静止期间缓慢跟踪 z 轴陀螺零偏（温漂补偿）；发生更新返回 1。 */
int calibration_track_gyro_z(ImuCalibration *c, float raw_gz_dps, float dt);

/* 保留仓库原公共数学接口。静止 +Z=+1g，初始 yaw=0。 */
int attitude_init(float q[4], float *accel_norm, const float accel_g[3]);
void attitude_euler(const float q[4], float rpy_deg[3]);
/* Cube 的 Ry*Rx*Rz 分解，不可与常规 ZYX 欧拉角混用。 */
void attitude_cube_euler(const float q[4], float xyz_deg[3]);

/* 更新函数成功返回 1，失败返回 0；失败不提交新状态。 */
int mahony6_init(Mahony6 *s, const float a[3]);
int mahony6_update(Mahony6 *s, const float gyro_rad_s[3], const float a[3], float dt);
void mahony6_euler(const Mahony6 *s, float e[3]);
void mahony6_cube_euler(const Mahony6 *s, float e[3]);
int madgwick6_init(Madgwick6 *s, const float a[3]);
int madgwick6_update(Madgwick6 *s, const float gyro_rad_s[3], const float a[3], float dt);
void madgwick6_euler(const Madgwick6 *s, float e[3]);
void madgwick6_cube_euler(const Madgwick6 *s, float e[3]);

/* 仍按原 AHRS_METHOD 编译期选型，两种算法的原接口均保留。 */
/* Quaternion nominal state, local angle error + X/Y residual gyro bias.
 * Z bias remains handled by wheel-confirmed stationary calibration in imu.c.
 * Fixed-size storage; no allocator or matrix library required. */
typedef struct {
    float q[4], bias_xy[2], covariance[5][5], accel_lpf[3];
    float accel_norm, innovation_nis;
    int accel_used;
} Ekf6;
int ekf6_init(Ekf6 *s, const float a[3]);
int ekf6_update(Ekf6 *s, const float g[3], const float a[3], float dt);
void ekf6_euler(const Ekf6 *s, float e[3]);
void ekf6_cube_euler(const Ekf6 *s, float e[3]);
#if AHRS_METHOD == AHRS_METHOD_EKF
typedef Ekf6 Ahrs6;
#define ahrs6_init ekf6_init
#define ahrs6_update ekf6_update
#define ahrs6_euler ekf6_euler
#define ahrs6_cube_euler ekf6_cube_euler
#elif AHRS_METHOD == AHRS_METHOD_MAHONY
typedef Mahony6 Ahrs6;
#define ahrs6_init mahony6_init
#define ahrs6_update mahony6_update
#define ahrs6_euler mahony6_euler
#define ahrs6_cube_euler mahony6_cube_euler
#elif AHRS_METHOD == AHRS_METHOD_MADGWICK
typedef Madgwick6 Ahrs6;
#define ahrs6_init madgwick6_init
#define ahrs6_update madgwick6_update
#define ahrs6_euler madgwick6_euler
#define ahrs6_cube_euler madgwick6_cube_euler
#else
#error Invalid AHRS_METHOD
#endif
#ifdef __cplusplus
}
#endif
#endif
