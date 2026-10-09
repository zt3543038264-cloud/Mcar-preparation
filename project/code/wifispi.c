/** 网络驱动、JustFloat 打包及可订阅遥测。
 * 收发均为同步操作，只在主循环且开中断时调用。
 */
#include "wifispi.h"
#include "imu.h"
#if IMU_WIFI_ENABLED
#include "zf_common_clock.h"
#include "zf_common_interrupt.h"
#include "zf_device_wifi_spi.h"
#include "zf_driver_delay.h"
#include "zf_driver_gpio.h"
#include "zf_driver_pwm.h"
#include "Motor.h"
#include "app_navigation.h"
#include "app_control.h"
#include "PID_config.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#endif
#include <string.h>

volatile int imu_wifi_status;
volatile uint32_t imu_wifi_tx_packets, imu_wifi_init_attempts;
volatile int imu_wifi_last_error;
volatile uint32_t wifi_telemetry_channel_count, wifi_telemetry_period_ms;
volatile uint32_t wifi_telemetry_commands, wifi_telemetry_command_errors;
volatile uint32_t wifi_telemetry_stream_enabled;

/* 编译期确认 float32；协议不依赖主机字节序。 */
typedef char float_must_be_32_bits[(sizeof(float)==4)?1:-1];
static void pack_float32_le(uint8_t *out,float value)
{
    uint32_t bits;
    unsigned j;
    memcpy(&bits,&value,4);
    for (j=0;j<4;++j) out[j]=(uint8_t)(bits>>(8*j));
}
size_t vofa_pack(uint8_t *out,size_t out_capacity,const float *channels,size_t count)
{
    size_t i,frame_bytes;
    if (out==NULL || channels==NULL || count==0u || count>VOFA_MAX_CHANNELS) return 0u;
    frame_bytes=VOFA_FRAME_BYTES(count);
    if (out_capacity<frame_bytes) return 0u;
    for (i=0u;i<count;++i) pack_float32_le(&out[4u*i],channels[i]);
    out[4u*count]=0x00u;
    out[4u*count+1u]=0x00u;
    out[4u*count+2u]=0x80u;
    out[4u*count+3u]=0x7fu;
    return frame_bytes;
}
#if IMU_WIFI_ENABLED
#include "zf_driver_spi.h"
/* SPI1 当前使用 D12-D15。允许现有 C/D2-D3 电机接线，禁止退回旧接线。 */
#define WIFI_SPI1_DIR_CONFLICT(pin) ((pin)==D12 || (pin)==D13 || (pin)==D14 || (pin)==D15)
#define WIFI_SPI1_PWM_CONFLICT(pin) ((pin)==PWM1_MODULE0_CHA_D12 || (pin)==PWM1_MODULE0_CHB_D13 || \
                                   (pin)==PWM1_MODULE1_CHA_D14 || (pin)==PWM1_MODULE1_CHB_D15)
typedef char wifi_spi1_motor_pin_conflict[(WIFI_SPI_INDEX!=SPI_1 ||
    !(WIFI_SPI1_DIR_CONFLICT(MOTOR1_DIR) || WIFI_SPI1_DIR_CONFLICT(MOTOR2_DIR) ||
      WIFI_SPI1_DIR_CONFLICT(MOTOR3_DIR) || WIFI_SPI1_DIR_CONFLICT(MOTOR4_DIR) ||
      WIFI_SPI1_PWM_CONFLICT(MOTOR1_PWM) || WIFI_SPI1_PWM_CONFLICT(MOTOR2_PWM) ||
      WIFI_SPI1_PWM_CONFLICT(MOTOR3_PWM) || WIFI_SPI1_PWM_CONFLICT(MOTOR4_PWM)))?1:-1];
static int wifi_ready;
static uint32 g_last_wifi_tx_ticks, g_last_wifi_rx_ticks;

/* 单一表定义公开变量名、通道 ID 和取值；只能订阅，不能写控制变量。
 * nav 的 XY/速度是 Zero 固定坐标，cmd 为车体坐标；所有通道转为 float32。
 * getter 及表达式全部在短临界区执行，字符串解析和 SPI 收发均在开中断时。 */
#define TELEMETRY_VARIABLES(X) \
    X(roll_deg, imu_roll_deg) X(pitch_deg, imu_pitch_deg) X(yaw_deg, imu_yaw_deg) \
    X(x_cm, nav.x_m*100.0f) X(y_cm, nav.y_m*100.0f) X(nav_yaw_deg, nav.yaw_deg) \
    X(vx_cmps, nav.vx_mps*100.0f) X(vy_cmps, nav.vy_mps*100.0f) \
    X(ax_mps2, nav.ax_mps2) X(ay_mps2, nav.ay_mps2) \
    X(nav_status, nav.status) X(nav_valid, nav.valid) X(bias_ready, nav.bias_ready) \
    X(stationary, nav.stationary) X(slipping, nav.slipping) X(enc_weight, nav.encoder_weight) \
    X(enc_rejected, nav.rejected_encoder_samples) \
    X(goal_x_cm, motor_position_goal.x_cm) X(goal_y_cm, motor_position_goal.y_cm) \
    X(goal_yaw_deg, motor_position_goal.yaw_deg) \
    X(cmd_vx_cmps, motor_cmd_vx_cmps) X(cmd_vy_cmps, motor_cmd_vy_cmps) \
    X(cmd_omega_radps, motor_cmd_omega_radps) X(pos_status, pos.status) \
    X(distance_cm, pos.distance_cm) X(yaw_error_deg, pos.yaw_error_deg) \
    X(run, motor_run_enabled) X(position_enabled, motor_position_enabled) \
    X(open_loop, motor_pwm_test_enabled) X(scale_x, navigation_scale_x) X(scale_y, navigation_scale_y) \
    X(accel_x_g, imu_accel_g[0]) X(accel_y_g, imu_accel_g[1]) X(accel_z_g, imu_accel_g[2]) \
    X(gyro_x_dps, imu_gyro_dps[0]) X(gyro_y_dps, imu_gyro_dps[1]) X(gyro_z_dps, imu_gyro_dps[2]) \
    X(imu_status, imu_attitude_status) X(imu_cal_percent, imu_calibration_percent) \
    X(ul_raw, motor.raw_counts[0]) X(ur_raw, motor.raw_counts[1]) \
    X(dl_raw, motor.raw_counts[2]) X(dr_raw, motor.raw_counts[3]) \
    X(ul_filt, motor.filtered_counts[0]) X(ur_filt, motor.filtered_counts[1]) \
    X(dl_filt, motor.filtered_counts[2]) X(dr_filt, motor.filtered_counts[3]) \
    X(ul_total, motor.cumulative_raw_counts[0]) X(ur_total, motor.cumulative_raw_counts[1]) \
    X(dl_total, motor.cumulative_raw_counts[2]) X(dr_total, motor.cumulative_raw_counts[3]) \
    X(ul_cmps, motor.wheel_speed_cmps[0]) X(ur_cmps, motor.wheel_speed_cmps[1]) \
    X(dl_cmps, motor.wheel_speed_cmps[2]) X(dr_cmps, motor.wheel_speed_cmps[3]) \
    X(ul_target, motor.target_counts[0]) X(ur_target, motor.target_counts[1]) \
    X(dl_target, motor.target_counts[2]) X(dr_target, motor.target_counts[3]) \
    X(ul_pwm, motor.final_pwm[0]) X(ur_pwm, motor.final_pwm[1]) \
    X(dl_pwm, motor.final_pwm[2]) X(dr_pwm, motor.final_pwm[3]) \
    X(control_ticks, motor.control_ticks)
#define VARIABLE_ID(name, expr) TV_##name,
enum { TELEMETRY_VARIABLES(VARIABLE_ID) TV_COUNT };
#undef VARIABLE_ID
#define VARIABLE_NAME(name, expr) #name,
static const char *const g_variable_names[]={TELEMETRY_VARIABLES(VARIABLE_NAME)};
#undef VARIABLE_NAME
typedef char variable_ids_fit_byte[(TV_COUNT<=256)?1:-1];
static uint8_t g_channels[VOFA_MAX_CHANNELS];
static char g_command[1024];
static size_t g_command_length;
static int g_command_discard;
static int g_slider_receiving;

/* A slider names one public parameter, never an address. Add mappings here
 * when another application variable should be editable from the host. */
typedef struct {
    const char *name;
    volatile float *value;
    float minimum, maximum;
    int needs_stop;
    tagPID_T *pid;
    PIDInitStruct *pid_init;
} slider_parameter_t;
#define SLIDER(name, address, low, high, stop) {name, address, low, high, stop, NULL, NULL}
#define SLIDER_PID(name, wheel, term) \
    {name, &wheel##PidInitStruct.term, 0, 1000, 0, &wheel##pid, &wheel##PidInitStruct}
static const slider_parameter_t g_slider_parameters[] = {
    SLIDER("scale_x", &navigation_scale_x, 0.01f, 10, 1),
    SLIDER("scale_y", &navigation_scale_y, 0.01f, 10, 1),
    SLIDER("pos_xy_kp", &motor_position_config.xy_kp, 0.01f, 20, 0),
    SLIDER("pos_xy_kd", &motor_position_config.xy_kd, 0, 5, 0),
    SLIDER("pos_yaw_kp", &motor_position_config.yaw_kp, 0.01f, 20, 0),
    SLIDER("pos_max_speed_cmps", &motor_position_config.max_speed_cmps, 1, 100, 0),
    SLIDER("pos_max_omega_radps", &motor_position_config.max_omega_radps, 0.05f, 3, 0),
    SLIDER("pos_max_accel_cmps2", &motor_position_config.max_accel_cmps2, 1, 300, 0),
    SLIDER("pos_max_alpha_radps2", &motor_position_config.max_alpha_radps2, 0.05f, 10, 0),
    SLIDER("pos_xy_tolerance_cm", &motor_position_config.xy_tolerance_cm, 0.5f, 20, 0),
    SLIDER("pos_yaw_tolerance_deg", &motor_position_config.yaw_tolerance_deg, 0.5f, 20, 0),
    SLIDER("goal_x_cm", &motor_position_goal.x_cm, -10000, 10000, 1),
    SLIDER("goal_y_cm", &motor_position_goal.y_cm, -10000, 10000, 1),
    SLIDER("goal_yaw_deg", &motor_position_goal.yaw_deg, -180, 180, 1),
    SLIDER_PID("ul_kp", UL, fKp), SLIDER_PID("ul_ki", UL, fKi), SLIDER_PID("ul_kd", UL, fKd),
    SLIDER_PID("ur_kp", UR, fKp), SLIDER_PID("ur_ki", UR, fKi), SLIDER_PID("ur_kd", UR, fKd),
    SLIDER_PID("dl_kp", DL, fKp), SLIDER_PID("dl_ki", DL, fKi), SLIDER_PID("dl_kd", DL, fKd),
    SLIDER_PID("dr_kp", DR, fKp), SLIDER_PID("dr_ki", DR, fKi), SLIDER_PID("dr_kd", DR, fKd)
};
#undef SLIDER
#undef SLIDER_PID

static void telemetry_reset(void)
{
    g_channels[0]=TV_roll_deg; g_channels[1]=TV_pitch_deg; g_channels[2]=TV_yaw_deg;
    wifi_telemetry_channel_count=3;
    wifi_telemetry_period_ms=IMU_WIFI_PERIOD_MS;
    wifi_telemetry_stream_enabled=1;
    wifi_telemetry_commands=wifi_telemetry_command_errors=0;
    g_command_length=0; g_command_discard=0;
    g_slider_receiving=0;
}

static void send_reply(const char *text)
{
    if (wifi_spi_send_buffer((const uint8 *)text,(uint32)strlen(text))!=0 ||
        wifi_spi_udp_send_now()!=0) imu_wifi_status=-4;
}
static void command_error(const char *reason)
{
    char reply[96];
    ++wifi_telemetry_command_errors;
    (void)snprintf(reply,sizeof(reply),"MCAR ERR %s\n",reason);
    send_reply(reply);
}
static char *trim(char *text)
{
    char *end;
    while (*text==' ' || *text=='\t') ++text;
    end=text+strlen(text);
    while (end>text && (end[-1]==' ' || end[-1]=='\t')) --end;
    *end='\0';
    return text;
}
static char *slider_token(char *text)
{
    size_t length;
    text=trim(text); length=strlen(text);
    if (length>=2 && text[0]=='"' && text[length-1]=='"') {
        text[length-1]='\0'; ++text;
    }
    if (*text=='\0' || strchr(text,'"')!=NULL) return NULL;
    return text;
}
/* Decimal number, optionally signed and with exponent; not a general JSON
 * parser. Strings/escapes, extra fields, NaN/Inf and hex numbers are rejected. */
static int slider_number(const char *text, float *out)
{
    const char *p=text;
    char *end;
    if (*p=='-' || *p=='+') ++p;
    if (*p<'0' || *p>'9') return 0;
    while (*p>='0' && *p<='9') ++p;
    if (*p=='.') {
        ++p;
        if (*p<'0' || *p>'9') return 0;
        while (*p>='0' && *p<='9') ++p;
    }
    if (*p=='e' || *p=='E') {
        ++p;
        if (*p=='+' || *p=='-') ++p;
        if (*p<'0' || *p>'9') return 0;
        while (*p>='0' && *p<='9') ++p;
    }
    if (*p!='\0') return 0;
    *out=strtof(text,&end);
    return *end=='\0' && isfinite(*out);
}
static void process_slider(char *command)
{
    char *kind, *name, *number, *comma;
    size_t length=strlen(command), i;
    float value;
    const slider_parameter_t *parameter=NULL;
    uint32 primask;
    char reply[96];
    if (length<2 || command[length-1]!=']') goto bad_packet;
    command[length-1]='\0'; kind=command+1;
    comma=strchr(kind,',');
    if (comma==NULL) goto bad_packet;
    *comma='\0'; name=comma+1;
    comma=strchr(name,',');
    if (comma==NULL) goto bad_packet;
    *comma='\0'; number=trim(comma+1);
    kind=slider_token(kind); name=slider_token(name);
    if (kind==NULL || name==NULL || !slider_number(number,&value)) goto bad_packet;
    if (strcmp(kind,"slider")!=0) { command_error("unknown control type"); return; }
    for (i=0;i<sizeof(g_slider_parameters)/sizeof(g_slider_parameters[0]);++i) {
        if (strcmp(name,g_slider_parameters[i].name)==0) { parameter=&g_slider_parameters[i]; break; }
    }
    if (parameter==NULL) { command_error("unknown parameter"); return; }
    if (value<parameter->minimum || value>parameter->maximum) {
        command_error("parameter out of range"); return;
    }
    primask=interrupt_global_disable();
    if (parameter->needs_stop && motor_run_enabled) {
        interrupt_global_enable(primask);
        command_error("parameter requires Run off"); return;
    }
    {
        int changed=(*parameter->value!=value);
        *parameter->value=value;
        if (parameter->pid!=NULL && (changed ||
            parameter->pid->fKp!=parameter->pid_init->fKp ||
            parameter->pid->fKi!=parameter->pid_init->fKi ||
            parameter->pid->fKd!=parameter->pid_init->fKd)) {
            PID_Update(parameter->pid,parameter->pid_init);
            PID_Clear(parameter->pid);
        }
    }
    value=*parameter->value;
    interrupt_global_enable(primask);
    (void)snprintf(reply,sizeof(reply),"MCAR SLIDER %s %.9g\n",name,(double)value);
    send_reply(reply);
    return;
bad_packet:
    command_error("expected [slider,parameter,number]");
}
static void reply_names(int all)
{
    /* Maximum names + delimiters are checked at append time. One UDP datagram. */
    char reply[1400];
    size_t used=0, i, count=all?TV_COUNT:wifi_telemetry_channel_count;
    int n=snprintf(reply,sizeof(reply),all?"MCAR LIST ":"MCAR SUB ");
    used=(size_t)n;
    for (i=0;i<count;++i) {
        n=snprintf(reply+used,sizeof(reply)-used,"%s%s",i?",":"",
                   g_variable_names[all?i:g_channels[i]]);
        if (n<0 || (size_t)n>=sizeof(reply)-used-1) {
            command_error("reply too long"); return;
        }
        used+=(size_t)n;
    }
    reply[used++]='\n'; reply[used]='\0';
    send_reply(reply);
}
static void process_command(char *line)
{
    char *command=trim(line), *arg, *end;
    uint8_t selected[VOFA_MAX_CHANNELS];
    size_t count=0, id;
    unsigned long number;
    char reply[80];
    ++wifi_telemetry_commands;
    if (*command=='[') { process_slider(command); return; }
    if (strcmp(command,"LIST?")==0) { reply_names(1); return; }
    if (strcmp(command,"GET?")==0) {
        reply_names(0);
        (void)snprintf(reply,sizeof(reply),"MCAR RATE %lu\n",(unsigned long)wifi_telemetry_period_ms);
        send_reply(reply);
        (void)snprintf(reply,sizeof(reply),"MCAR STREAM %lu\n",(unsigned long)wifi_telemetry_stream_enabled);
        send_reply(reply); return;
    }
    arg=strpbrk(command," \t");
    if (arg==NULL) { command_error("expected SUB/RATE/STREAM/LIST?/GET?"); return; }
    *arg++='\0'; arg=trim(arg);
    if (strcmp(command,"SUB")==0) {
        /* Validate the entire list before replacing anything. Empty fields,
         * duplicate names, unknown names and >40 channels leave the old list intact. */
        while (1) {
            char *comma=strchr(arg,',');
            size_t j;
            if (comma!=NULL) *comma='\0';
            arg=trim(arg);
            if (*arg=='\0' || count==VOFA_MAX_CHANNELS) { command_error("invalid channel count"); return; }
            for (id=0;id<TV_COUNT;++id) if (strcmp(arg,g_variable_names[id])==0) break;
            if (id==TV_COUNT) { command_error("unknown variable"); return; }
            for (j=0;j<count;++j) if (selected[j]==id) { command_error("duplicate variable"); return; }
            selected[count++]=(uint8_t)id;
            if (comma==NULL) break;
            arg=comma+1;
        }
        memcpy(g_channels,selected,count);
        wifi_telemetry_channel_count=(uint32_t)count;
        reply_names(0); return;
    }
    if (*arg<'0' || *arg>'9') { command_error("expected unsigned integer"); return; }
    number=strtoul(arg,&end,10);
    if (*end!='\0') { command_error("invalid integer"); return; }
    if (strcmp(command,"RATE")==0 && number>=2 && number<=1000) {
        wifi_telemetry_period_ms=(uint32_t)number;
        (void)snprintf(reply,sizeof(reply),"MCAR RATE %lu\n",number);
    } else if (strcmp(command,"STREAM")==0 && number<=1) {
        wifi_telemetry_stream_enabled=(uint32_t)number;
        (void)snprintf(reply,sizeof(reply),"MCAR STREAM %lu\n",number);
    } else { command_error("RATE 2..1000 or STREAM 0/1"); return; }
    send_reply(reply);
}
static void receive_commands(void)
{
    uint8_t bytes[256];
    uint32 i, count=wifi_spi_read_buffer(bytes,sizeof(bytes));
    for (i=0;i<count;++i) {
        uint8_t ch=bytes[i];
        if (ch=='[' && !g_slider_receiving && !g_command_discard) {
            g_command_length=0; g_slider_receiving=1;
        }
        if (ch=='\n' || (g_slider_receiving && ch==']')) {
            if (ch==']' && !g_command_discard) {
                if (g_command_length==sizeof(g_command)-1) g_command_discard=1;
                else g_command[g_command_length++]=(char)ch;
            }
            if (g_command_discard) {
                ++wifi_telemetry_commands;
                command_error("invalid or oversized line");
            }
            else if (g_command_length!=0) {
                g_command[g_command_length]='\0'; process_command(g_command);
            }
            g_command_length=0; g_command_discard=0; g_slider_receiving=0;
        } else if (ch=='\r') {
            /* Accept both LF and CRLF from the existing host text sender. */
        } else if (g_command_discard) {
            /* Recover only at the next newline. */
        } else if ((ch<32 && ch!='\t') || ch>126 || g_command_length==sizeof(g_command)-1) {
            g_command_discard=1;
        } else g_command[g_command_length++]=(char)ch;
    }
}
static size_t capture_channels(float channels[VOFA_MAX_CHANNELS])
{
    navigation_snapshot_t nav;
    position_output_t pos;
    motor_speed_debug_snapshot_t motor;
    size_t i, count=wifi_telemetry_channel_count;
    uint32 primask=interrupt_global_disable();
    app_navigation_get_snapshot(&nav);
    app_control_get_position_snapshot(&pos);
    motor_speed_debug_get_snapshot(&motor);
#define VARIABLE_VALUE(name, expr) (float)(expr),
    const float values[]={TELEMETRY_VARIABLES(VARIABLE_VALUE)};
#undef VARIABLE_VALUE
    for (i=0;i<count;++i) channels[i]=values[g_channels[i]];
    interrupt_global_enable(primask);
    return count;
}
static int imu_wifi_init_once(void)
{
    memset(wifi_spi_version,0,sizeof(wifi_spi_version));
    memset(wifi_spi_ip_addr_port,0,sizeof(wifi_spi_ip_addr_port));
    imu_wifi_status=2;
    if (wifi_spi_init(0,0)!=0 || strncmp(wifi_spi_version,"V2",2)!=0) return -1;
    imu_wifi_status=3;
    if (wifi_spi_wifi_connect(IMU_WIFI_SSID,IMU_WIFI_PASSWORD)!=0) return -2;
    imu_wifi_status=4;
    if (wifi_spi_socket_connect("UDP",IMU_WIFI_TARGET_IP,
                               IMU_WIFI_TARGET_PORT,IMU_WIFI_LOCAL_PORT)!=0) return -3;
    return 1;
}
#endif
int wifispi_init(void)
{
#if IMU_WIFI_ENABLED
    uint32_t attempt;
    int result;
    wifi_ready=0;
    imu_wifi_status=0; imu_wifi_init_attempts=0; imu_wifi_last_error=0;
    /* 保留原初始化统计语义：启用模式重新初始化不清发送累计数。 */
    system_delay_ms(IMU_WIFI_STARTUP_DELAY_MS);
    for (attempt=0;attempt<IMU_WIFI_INIT_ATTEMPTS;++attempt) {
        if (attempt!=0) system_delay_ms(IMU_WIFI_RETRY_DELAY_MS);
        ++imu_wifi_init_attempts;
        result=imu_wifi_init_once();
        if (result==1) { wifi_ready=1; imu_wifi_status=1; return 1; }
        imu_wifi_last_error=result; imu_wifi_status=result;
    }
    return 0;
#else
    /* 禁用时绝不初始化 SPI/GPIO，防止覆盖电机引脚复用。 */
    imu_wifi_status=-5; imu_wifi_init_attempts=0;
    imu_wifi_last_error=0; imu_wifi_tx_packets=0;
    return 0;
#endif
}
int wifispi_send_floats(const float *channels,size_t count)
{
#if IMU_WIFI_ENABLED
    uint8_t packet[VOFA_MAX_FRAME_BYTES];
    size_t packet_bytes;
    if (!wifi_ready) return 0;
    packet_bytes=vofa_pack(packet,sizeof(packet),channels,count);
    if (packet_bytes==0u) return 0;
    /* send_buffer 返回未发送字节数；send_now 结束本 UDP 数据报。 */
    if (wifi_spi_send_buffer(packet,(uint32_t)packet_bytes)!=0 ||
        wifi_spi_udp_send_now()!=0) { imu_wifi_status=-4; return 0; }
    imu_wifi_status=1; ++imu_wifi_tx_packets;
    return 1;
#else
    (void)channels; (void)count;
    return 0;
#endif
}
int wifispi_send_imu(const float angles_deg[3])
{
    return wifispi_send_floats(angles_deg,3u);
}
void wifispi_telemetry_init(void)
{
#if IMU_WIFI_ENABLED
    telemetry_reset();
#endif
    (void)wifispi_init();
#if IMU_WIFI_ENABLED
    g_last_wifi_tx_ticks=g_last_wifi_rx_ticks=DWT->CYCCNT;
#endif
}
void wifispi_telemetry_service(void)
{
#if IMU_WIFI_ENABLED
    uint32 now;
    float channels[VOFA_MAX_CHANNELS];
    size_t count;
    if (!wifi_ready) return;
    now=DWT->CYCCNT;
    /* Poll independently of streaming so STREAM 0 and slow RATE cannot lock
     * the host out. Limit SPI polling to 100 Hz in the main loop. */
    if ((uint32)(now-g_last_wifi_rx_ticks)>=(system_clock/1000u)*10u) {
        g_last_wifi_rx_ticks=now; receive_commands();
    }
    now=DWT->CYCCNT;
    if (!wifi_telemetry_stream_enabled ||
        (uint32)(now-g_last_wifi_tx_ticks)<(system_clock/1000u)*wifi_telemetry_period_ms) return;
    g_last_wifi_tx_ticks=now;
    count=capture_channels(channels);
    (void)wifispi_send_floats(channels,count);
#endif
}
