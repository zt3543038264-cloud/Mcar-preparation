#ifndef MCAR_IMU_CONFIG_H
#define MCAR_IMU_CONFIG_H

/* Six-axis AHRS selection, ported from rt1064_imu_vofa. */
#define AHRS_METHOD_MAHONY       1
#define AHRS_METHOD_MADGWICK     2
#ifndef AHRS_METHOD
#define AHRS_METHOD              AHRS_METHOD_MAHONY
#endif
#if AHRS_METHOD != AHRS_METHOD_MAHONY && AHRS_METHOD != AHRS_METHOD_MADGWICK
#error AHRS_METHOD must be AHRS_METHOD_MAHONY or AHRS_METHOD_MADGWICK
#endif

#ifndef MADGWICK_BETA
#define MADGWICK_BETA            0.05f
#endif

#define AHRS_KP                  2.0f
#define AHRS_KI                  0.02f
#define AHRS_INTEGRAL_LIMIT      0.08726646f

#define ACC_NORM_MIN             0.85f
#define ACC_NORM_MAX             1.15f
#define MAX_SAMPLE_DT            0.020f

#define CALIBRATION_SAMPLES      400u
#define CAL_GYRO_LIMIT_DPS       10.0f
#define CAL_GYRO_SPAN_DPS        1.5f
#define CAL_ACCEL_SPAN_G         0.08f
#define IMU_TIMEOUT_MS           100u

/* WiFi-SPI2.0 UDP settings ported from rt1064_imu_vofa. */
/* SPI1 D12-D15 is free with the current Motor.h wiring. */
#ifndef IMU_WIFI_ENABLED
#define IMU_WIFI_ENABLED         1
#endif
#if IMU_WIFI_ENABLED != 0 && IMU_WIFI_ENABLED != 1
#error IMU_WIFI_ENABLED must be 0 or 1
#endif
#define IMU_WIFI_SSID            "HDUASC"
#define IMU_WIFI_PASSWORD        "zyz520520"
#define IMU_WIFI_TARGET_IP       "192.168.0.113"
#define IMU_WIFI_TARGET_PORT     "8081"
#define IMU_WIFI_LOCAL_PORT      "5001"
#define IMU_WIFI_STARTUP_DELAY_MS 300u
#define IMU_WIFI_INIT_ATTEMPTS    3u
#define IMU_WIFI_RETRY_DELAY_MS   500u
#define IMU_WIFI_PERIOD_MS        10u
#if IMU_WIFI_PERIOD_MS < 2 || IMU_WIFI_PERIOD_MS > 1000
#error IMU_WIFI_PERIOD_MS must be between 2 and 1000
#endif

#define DEG_TO_RAD               0.01745329251994329577f
#define RAD_TO_DEG               57.29577951308232088f

#endif
