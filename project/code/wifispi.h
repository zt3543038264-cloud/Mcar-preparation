/** WiFi-SPI2.0 JustFloat 遥测；主循环解析上位机 UDP 订阅命令。
 */
#ifndef MCAR_WIFISPI_H
#define MCAR_WIFISPI_H
#include "config.h"
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
#define VOFA_MAX_CHANNELS 40u
#define VOFA_TAIL_BYTES 4u
#define VOFA_FRAME_BYTES(count) (4u*(count)+VOFA_TAIL_BYTES)
#define VOFA_MAX_FRAME_BYTES VOFA_FRAME_BYTES(VOFA_MAX_CHANNELS)
/* 状态：0 未开始，1 就绪，2/3/4 初始化阶段；-1/-2/-3 初始化错误，
 * -4 发送错误，-5 禁用。保留原变量名和菜单数据绑定。 */
extern volatile int imu_wifi_status;
extern volatile uint32_t imu_wifi_tx_packets; /* 累计成功发送帧数 */
extern volatile uint32_t imu_wifi_init_attempts; /* 本次初始化尝试次数 */
extern volatile int imu_wifi_last_error; /* 原初始化最近错误，成功重试不清除 */
extern volatile uint32_t wifi_telemetry_channel_count, wifi_telemetry_period_ms;
extern volatile uint32_t wifi_telemetry_commands, wifi_telemetry_command_errors;
extern volatile uint32_t wifi_telemetry_stream_enabled;

/* 显式小端 float32 + 00 00 80 7F；无效参数返回 0，且不写输出。 */
size_t vofa_pack(uint8_t *out,size_t out_capacity,const float *channels,size_t count);
int wifispi_init(void); /* 启动阶段，可能阻塞 */
int wifispi_send_floats(const float *channels,size_t count); /* 1=成功 */
/* IMU 三通道专用接口；实际发送 16 字节 JustFloat，不是旧 12 字节包。 */
int wifispi_send_imu(const float angles_deg[3]);
void wifispi_telemetry_init(void);
/* 主循环；遥测配置 + [slider,参数名,数值]，也接受 JSON 引号。 */
void wifispi_telemetry_service(void);

/* 至少一个通道；表达式只求值一次，整数自动转 float。不得在中断中发送。 */
#define wifi_justfloat(...) \
    wifispi_send_floats((const float[]){__VA_ARGS__}, \
    sizeof((const float[]){__VA_ARGS__})/sizeof(float))
#ifdef __cplusplus
}
#endif
#endif
