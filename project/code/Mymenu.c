#include "Mymenu.h"
#include "Motor.h"
#include "Flash.h"
#include "route_follow.h"
#include "PID_config.h"
#include "app_control.h"
#include "app_navigation.h"
#include "imu.h"
#include "wifispi.h"
#include "menu.h"
#include "zf_common_font.h"
#include "zf_common_interrupt.h"
#include "zf_device_ips200.h"
#include "zf_device_key.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define MENU_COLUMNS 30
#define MENU_LINE_HEIGHT 16
#define MENU_VISIBLE_LINES 7
#define MENU_STEP_COUNT 5u

static Menu_Item g_root;
static Menu_Item *g_pointer;
static float g_steps[MENU_STEP_COUNT]={0.01f,0.1f,1.0f,10.0f,100.0f};
static uint8_t g_step_index=2u;
static volatile bool g_refresh_pending;
static bool g_imu_recalibrate;
static Menu_Item *g_encoder_folder;
static bool g_encoder_zero;
static uint8_t g_refresh_ticks;
static motor_speed_debug_snapshot_t g_motor_snapshot;
static int32 g_encoder_zero_counts[MOTOR_WHEEL_COUNT];
static Menu_Item *g_navigation_folder;
static bool g_navigation_zero;
static navigation_snapshot_t g_navigation_snapshot;
static float g_navigation_x_cm, g_navigation_y_cm;
static Menu_Item *g_position_folder;
static position_output_t g_position_snapshot;
static bool g_flash_save;
static char g_route_names[ROUTE_MAX_NODES][3][8];

/* 240 像素 / 8 像素字体 = 30 字符，补空格清除旧文本。 */
static void menu_show_line(uint16 y,const char *text)
{
    char bounded[MENU_COLUMNS+1];
    snprintf(bounded,sizeof(bounded),"%-*.*s",MENU_COLUMNS,MENU_COLUMNS,text);
    ips200_show_string(0,y,bounded);
}
static void menu_motor_snapshot(void)
{
    uint32 primask=interrupt_global_disable();
    motor_speed_debug_get_snapshot(&g_motor_snapshot);
    app_navigation_get_snapshot(&g_navigation_snapshot);
    app_control_get_position_snapshot(&g_position_snapshot);
    interrupt_global_enable(primask);
    g_navigation_x_cm=g_navigation_snapshot.x_m*100.0f;
    g_navigation_y_cm=g_navigation_snapshot.y_m*100.0f;
}
static void menu_create(void)
{
    Menu_Item *pwm_test=Create_Menu_Folder_dynamic(&g_root,"PWM_Test");
    Menu_Item *drive=Create_Menu_Folder_dynamic(&g_root,"Drive");
    Menu_Item *encoder=Create_Menu_Folder_dynamic(&g_root,"Encoder");
    Menu_Item *navigation=Create_Menu_Folder_dynamic(&g_root,"Navigation");
    Menu_Item *imu=Create_Menu_Folder_dynamic(&g_root,"IMU");
    Menu_Item *sensor=Create_Menu_Folder_dynamic(&g_root,"Sensor");
    Menu_Item *wifi=Create_Menu_Folder_dynamic(&g_root,"WiFi");
    Menu_Item *pid=Create_Menu_Folder_dynamic(&g_root,"PID");
    Menu_Item *position=Create_Menu_Folder_dynamic(&g_root,"Position");
    Menu_Item *pid_ul,*pid_ur,*pid_dl,*pid_dr;
    g_encoder_folder=encoder;
    g_navigation_folder = navigation;
    g_position_folder = position;

    Create_Menu_File_dynamic(position, "Enable", (void *)&motor_position_enabled, bool_Box);
    Create_Menu_File_dynamic(position, "Run", (void *)&motor_run_enabled, bool_Box);
    Create_Menu_File_dynamic(position, "TargetX_cm", (void *)&motor_position_goal.x_cm, float_Box);
    Create_Menu_File_dynamic(position, "TargetY_cm", (void *)&motor_position_goal.y_cm, float_Box);
    Create_Menu_File_dynamic(position, "TargetYaw", (void *)&motor_position_goal.yaw_deg, float_Box);
    Create_Menu_Readonly_dynamic(position, "State", &g_position_snapshot.status, int32_Box);
    Create_Menu_File_dynamic(position, "MaxV_cmps", (void *)&motor_position_config.max_speed_cmps, float_Box);
    Create_Menu_File_dynamic(position, "MaxOmega", (void *)&motor_position_config.max_omega_radps, float_Box);
    Create_Menu_File_dynamic(position, "TolXY_cm", (void *)&motor_position_config.xy_tolerance_cm, float_Box);
    Create_Menu_File_dynamic(position, "TolYaw_deg", (void *)&motor_position_config.yaw_tolerance_deg, float_Box);
    Create_Menu_File_dynamic(position, "Acc_cmps2", (void *)&motor_position_config.max_accel_cmps2, float_Box);
    Create_Menu_File_dynamic(position, "Alpha", (void *)&motor_position_config.max_alpha_radps2, float_Box);
    /* ivision 式开环规划参数（替代原 XY_Kp/XY_Kd/Yaw_Kp，后三者已不参与控制） */
    Create_Menu_File_dynamic(position, "BrakeLim", (void *)&motor_position_config.brake_limit, float_Box);
    Create_Menu_File_dynamic(position, "BrakeCap", (void *)&motor_position_config.brake_ceiling_cmps2, float_Box);
    Create_Menu_File_dynamic(position, "ShortSeg", (void *)&motor_position_config.short_segment_cm, float_Box);
    Create_Menu_File_dynamic(position, "ShortBst", (void *)&motor_position_config.short_boost_gain, float_Box);
    Create_Menu_File_dynamic(position, "AppZone", (void *)&motor_position_config.approach_zone_cm, float_Box);
    Create_Menu_File_dynamic(position, "AppRatio", (void *)&motor_position_config.approach_ratio, float_Box);
    Create_Menu_File_dynamic(position, "AppAccK", (void *)&motor_position_config.approach_acc_k, float_Box);
    Create_Menu_File_dynamic(position, "YawBand", (void *)&motor_position_config.yaw_lin_band_rad, float_Box);
    Create_Menu_File_dynamic(position, "YawKd", (void *)&motor_position_config.yaw_kd, float_Box);
    Create_Menu_File_dynamic(position, "YawTrKd", (void *)&motor_position_config.yaw_kd_translate, float_Box);
    Create_Menu_Readonly_dynamic(position, "ErrXY_cm", &g_position_snapshot.distance_cm, float_Box);
    Create_Menu_Readonly_dynamic(position, "ErrYaw_deg", &g_position_snapshot.yaw_error_deg, float_Box);

    Create_Menu_Readonly_dynamic(navigation, "State", &g_navigation_snapshot.status, int32_Box);
    Create_Menu_Readonly_dynamic(navigation, "X_cm", &g_navigation_x_cm, float_Box);
    Create_Menu_Readonly_dynamic(navigation, "Y_cm", &g_navigation_y_cm, float_Box);
    Create_Menu_Readonly_dynamic(navigation, "Yaw_deg", &g_navigation_snapshot.yaw_deg, float_Box);
    Create_Menu_File_dynamic(navigation, "Mount_deg", (void *)&navigation_mount_deg, float_Box);
    Create_Menu_File_dynamic(navigation, "YawFlip", (void *)&navigation_yaw_reversed, bool_Box);
    Create_Menu_File_dynamic(navigation, "Zero", &g_navigation_zero, bool_Box);
    Create_Menu_File_dynamic(navigation, "ScaleX", (void *)&navigation_scale_x, float_Box);
    Create_Menu_File_dynamic(navigation, "ScaleY", (void *)&navigation_scale_y, float_Box);

    Create_Menu_File_dynamic(drive,"Run",(void *)&motor_run_enabled,bool_Box);
    Create_Menu_File_dynamic(drive,"Vx_cmps",(void *)&motor_cmd_vx_cmps,float_Box);
    Create_Menu_File_dynamic(drive,"Vy_cmps",(void *)&motor_cmd_vy_cmps,float_Box);
    Create_Menu_File_dynamic(drive,"Omega",(void *)&motor_cmd_omega_radps,float_Box);
    Create_Menu_File_dynamic(pwm_test,"OpenLoop",(void *)&motor_pwm_test_enabled,bool_Box);
    Create_Menu_File_dynamic(pwm_test,"Run",(void *)&motor_run_enabled,bool_Box);
    Create_Menu_File_dynamic(pwm_test,"UL_PWM",(void *)&motor_test_pwm[MOTOR_WHEEL_UL],int16_Box);
    Create_Menu_File_dynamic(pwm_test,"UR_PWM",(void *)&motor_test_pwm[MOTOR_WHEEL_UR],int16_Box);
    Create_Menu_File_dynamic(pwm_test,"DL_PWM",(void *)&motor_test_pwm[MOTOR_WHEEL_DL],int16_Box);
    Create_Menu_File_dynamic(pwm_test,"DR_PWM",(void *)&motor_test_pwm[MOTOR_WHEEL_DR],int16_Box);
    Create_Menu_Readonly_dynamic(encoder,"UL",&up_L_all,int16_Box);
    Create_Menu_Readonly_dynamic(encoder,"UR",&up_R_all,int16_Box);
    Create_Menu_Readonly_dynamic(encoder,"DL",&down_L_all,int16_Box);
    Create_Menu_Readonly_dynamic(encoder,"DR",&down_R_all,int16_Box);
    Create_Menu_File_dynamic(encoder,"ZeroTotal",&g_encoder_zero,bool_Box);
    Create_Menu_Readonly_dynamic(imu,"Status",(void *)&imu_attitude_status,int32_Box);
    Create_Menu_Readonly_dynamic(imu,"CalPct",(void *)&imu_calibration_percent,float_Box);
    Create_Menu_Readonly_dynamic(imu,"Roll",(void *)&imu_roll_deg,float_Box);
    Create_Menu_Readonly_dynamic(imu,"Pitch",(void *)&imu_pitch_deg,float_Box);
    Create_Menu_Readonly_dynamic(imu,"Yaw",(void *)&imu_yaw_deg,float_Box);
    Create_Menu_File_dynamic(imu,"Recal",&g_imu_recalibrate,bool_Box);
    Create_Menu_Readonly_dynamic(sensor,"Gx_dps",(void *)&imu_gyro_dps[0],float_Box);
    Create_Menu_Readonly_dynamic(sensor,"Gy_dps",(void *)&imu_gyro_dps[1],float_Box);
    Create_Menu_Readonly_dynamic(sensor,"Gz_dps",(void *)&imu_gyro_dps[2],float_Box);
    Create_Menu_Readonly_dynamic(sensor,"Ax_g",(void *)&imu_accel_g[0],float_Box);
    Create_Menu_Readonly_dynamic(sensor,"Ay_g",(void *)&imu_accel_g[1],float_Box);
    Create_Menu_Readonly_dynamic(sensor,"Az_g",(void *)&imu_accel_g[2],float_Box);
    Create_Menu_Readonly_dynamic(wifi,"Status",(void *)&imu_wifi_status,int32_Box);
    Create_Menu_Readonly_dynamic(wifi,"Packets",(void *)&imu_wifi_tx_packets,uint32_Box);
    Create_Menu_Readonly_dynamic(wifi,"Attempts",(void *)&imu_wifi_init_attempts,uint32_Box);
    Create_Menu_Readonly_dynamic(wifi,"LastErr",(void *)&imu_wifi_last_error,int32_Box);
    Create_Menu_Readonly_dynamic(wifi,"Channels",(void *)&wifi_telemetry_channel_count,uint32_Box);
    Create_Menu_Readonly_dynamic(wifi,"Period_ms",(void *)&wifi_telemetry_period_ms,uint32_Box);
    Create_Menu_Readonly_dynamic(wifi,"Stream",(void *)&wifi_telemetry_stream_enabled,uint32_Box);
    Create_Menu_Readonly_dynamic(wifi,"Commands",(void *)&wifi_telemetry_commands,uint32_Box);
    Create_Menu_Readonly_dynamic(wifi,"CmdErr",(void *)&wifi_telemetry_command_errors,uint32_Box);
    pid_ul=Create_Menu_Folder_dynamic(pid,"UL");
    pid_ur=Create_Menu_Folder_dynamic(pid,"UR");
    pid_dl=Create_Menu_Folder_dynamic(pid,"DL");
    pid_dr=Create_Menu_Folder_dynamic(pid,"DR");
    Create_Menu_File_dynamic(pid_ul,"Kp",&ULpid.fKp,float_Box);
    Create_Menu_File_dynamic(pid_ul,"Ki",&ULpid.fKi,float_Box);
    Create_Menu_File_dynamic(pid_ul,"Kd",&ULpid.fKd,float_Box);
    Create_Menu_File_dynamic(pid_ur,"Kp",&URpid.fKp,float_Box);
    Create_Menu_File_dynamic(pid_ur,"Ki",&URpid.fKi,float_Box);
    Create_Menu_File_dynamic(pid_ur,"Kd",&URpid.fKd,float_Box);
    Create_Menu_File_dynamic(pid_dl,"Kp",&DLpid.fKp,float_Box);
    Create_Menu_File_dynamic(pid_dl,"Ki",&DLpid.fKi,float_Box);
    Create_Menu_File_dynamic(pid_dl,"Kd",&DLpid.fKd,float_Box);
    Create_Menu_File_dynamic(pid_dr,"Kp",&DRpid.fKp,float_Box);
    Create_Menu_File_dynamic(pid_dr,"Ki",&DRpid.fKi,float_Box);
    Create_Menu_File_dynamic(pid_dr,"Kd",&DRpid.fKd,float_Box);

    /* Route 路线跟随：按节点顺序调用位置外环逐点行驶。
     * Nodes 为生效节点数；N?X_cm/N?Y_cm/N?Yaw 为节点坐标（Zero 坐标系）。
     * 超出节点数的槽位不执行，可留空。 */
    Menu_Item *route=Create_Menu_Folder_dynamic(&g_root,"Route");
    Create_Menu_File_dynamic(route,"Run",(void *)&route_run_flag,bool_Box);
    Create_Menu_File_dynamic(route,"Nodes",(void *)&route_node_count,int16_Box);
    Create_Menu_Readonly_dynamic(route,"Idx",(void *)&route_current_idx,int32_Box);
    Create_Menu_Readonly_dynamic(route,"State",(void *)&route_state,int32_Box);
    for(unsigned i=0;i<ROUTE_MAX_NODES;++i) {
        snprintf(g_route_names[i][0],sizeof(g_route_names[i][0]),"N%dX_cm",i+1);
        snprintf(g_route_names[i][1],sizeof(g_route_names[i][1]),"N%dY_cm",i+1);
        snprintf(g_route_names[i][2],sizeof(g_route_names[i][2]),"N%dYaw",i+1);
        Create_Menu_File_dynamic(route,g_route_names[i][0],&route_nodes[i].x_cm,float_Box);
        Create_Menu_File_dynamic(route,g_route_names[i][1],&route_nodes[i].y_cm,float_Box);
        Create_Menu_File_dynamic(route,g_route_names[i][2],&route_nodes[i].yaw_deg,float_Box);
    }

    /* 根目录保存项：置 On 即把当前 PID/位置环/导航参数写入 Flash */
    g_flash_save=false;
    Create_Menu_File_dynamic(route,"SaveCfg",&g_flash_save,bool_Box);
    Create_Menu_File_dynamic(&g_root,"SaveCfg",&g_flash_save,bool_Box);
}
static void menu_format_value(const Menu_Item *item,char *buffer,size_t size)
{
    switch(item->kind) {
    case int32_Box: snprintf(buffer,size,"%ld",(long)*(int32_t *)item->data); break;
    case uint32_Box: snprintf(buffer,size,"%lu",(unsigned long)*(uint32_t *)item->data); break;
    case int16_Box: snprintf(buffer,size,"%d",(int)*(int16_t *)item->data); break;
    case uint16_Box: snprintf(buffer,size,"%u",(unsigned)*(uint16_t *)item->data); break;
    case int8_Box: snprintf(buffer,size,"%d",(int)*(int8_t *)item->data); break;
    case uint8_Box: snprintf(buffer,size,"%u",(unsigned)*(uint8_t *)item->data); break;
    case float_Box: snprintf(buffer,size,"%.3f",(double)*(float *)item->data); break;
    case bool_Box: snprintf(buffer,size,"%s",*(bool *)item->data?"On":"Off"); break;
    default: snprintf(buffer,size,"[folder]"); break;
    }
}
static void menu_draw(void)
{
    Menu_Item *item=g_pointer->Father->First_Son;
    char line[MENU_COLUMNS+1],value[14];
    uint8_t row;
    unsigned first_row=g_pointer->rank>MENU_VISIBLE_LINES ?
                       (unsigned)g_pointer->rank-MENU_VISIBLE_LINES : 0u;
    snprintf(line,sizeof(line),"%-20s <%5.2f>",g_pointer->Father->name,(double)(g_pointer->data==(void *)&route_node_count ? 1.0f : g_steps[g_step_index]));
    menu_show_line(0,line);
    if(g_pointer->Father==g_encoder_folder) {
        menu_show_line(MENU_LINE_HEIGHT," Wheel Raw Filt Total");
        for(row=0u;row<MOTOR_WHEEL_COUNT;++row) {
            snprintf(line,sizeof(line),"%c%-2s %6d%6d%12ld",item==g_pointer?'>':' ',item->name,
                     g_motor_snapshot.raw_counts[row],g_motor_snapshot.filtered_counts[row],
                     (long)(g_motor_snapshot.cumulative_raw_counts[row]-g_encoder_zero_counts[row]));
            menu_show_line((uint16)((row+2u)*MENU_LINE_HEIGHT),line);
            item=item->Next_Brother;
        }
        snprintf(line,sizeof(line),"%c%c%-12s %12s",item==g_pointer?'>':' ',item->selected?'*':' ',item->name,"Off");
        menu_show_line(6u*MENU_LINE_HEIGHT,line);
        menu_show_line(7u*MENU_LINE_HEIGHT,"Raw/Filt=count/10ms");
        snprintf(line,sizeof(line),"cm/s UL:%7.2f UR:%7.2f",(double)g_motor_snapshot.wheel_speed_cmps[0],(double)g_motor_snapshot.wheel_speed_cmps[1]);
        menu_show_line(128,line);
        snprintf(line,sizeof(line),"cm/s DL:%7.2f DR:%7.2f",(double)g_motor_snapshot.wheel_speed_cmps[2],(double)g_motor_snapshot.wheel_speed_cmps[3]);
        menu_show_line(144,line);
    } else {
        menu_show_line(128,""); menu_show_line(144,"");
        for(unsigned skip=0;skip<first_row;++skip) item=item->Next_Brother;
        for(row=0u;row<MENU_VISIBLE_LINES;++row) {
            if(row+first_row<g_pointer->Father->sons) {
                menu_format_value(item,value,sizeof(value));
                snprintf(line,sizeof(line),"%c%c%-12s %12s",item==g_pointer?'>':' ',item->selected?'*':' ',item->name,value);
                item=item->Next_Brother;
            } else snprintf(line,sizeof(line),"%30s","");
            menu_show_line((uint16)((row+1u)*MENU_LINE_HEIGHT),line);
        }
        if (g_pointer->Father == g_navigation_folder) {
            snprintf(line, sizeof(line), "Vcm/s X:%7.2f Y:%7.2f",
                     (double)(g_navigation_snapshot.vx_mps * 100.0f),
                     (double)(g_navigation_snapshot.vy_mps * 100.0f));
            menu_show_line(128, line);
            snprintf(line, sizeof(line), "Valid:%d Bias:%d Slip:%d Rest:%d",
                     g_navigation_snapshot.valid, g_navigation_snapshot.bias_ready,
                     g_navigation_snapshot.slipping, g_navigation_snapshot.stationary);
            menu_show_line(144, line);
        }
        if (g_pointer->Father == g_position_folder) {
            snprintf(line, sizeof(line), "Vx%5.1f Vy%5.1f W%5.2f",
                     (double)g_position_snapshot.vx_cmps,
                     (double)g_position_snapshot.vy_cmps,
                     (double)g_position_snapshot.omega_radps);
            menu_show_line(128, line);
            snprintf(line, sizeof(line), "XY%6.1f,%6.1f Yaw%6.1f",
                     (double)g_navigation_x_cm, (double)g_navigation_y_cm,
                     (double)g_navigation_snapshot.yaw_deg);
            menu_show_line(144, line);
        }
    }
    snprintf(line,sizeof(line),"IMU:%ld Cal:%5.1f%% ",(long)imu_attitude_status,(double)imu_calibration_percent);
    menu_show_line(160,line);
    snprintf(line,sizeof(line),"RPY:%7.2f %7.2f %7.2f",(double)imu_roll_deg,(double)imu_pitch_deg,(double)imu_yaw_deg);
    menu_show_line(176,line);
    snprintf(line,sizeof(line),"ENC:%5d %5d %5d %5d ",g_motor_snapshot.filtered_counts[0],g_motor_snapshot.filtered_counts[1],g_motor_snapshot.filtered_counts[2],g_motor_snapshot.filtered_counts[3]);
    menu_show_line(192,line);
    snprintf(line,sizeof(line),"PWM:%5d %5d %5d %5d ",g_motor_snapshot.final_pwm[0],g_motor_snapshot.final_pwm[1],g_motor_snapshot.final_pwm[2],g_motor_snapshot.final_pwm[3]);
    menu_show_line(208,line);
    snprintf(line,sizeof(line),"K1=enter K3=back K2/K4=move");
    menu_show_line(224,line);
}
static float menu_clamp(float value,float low,float high)
{ return value<low?low:(value>high?high:value); }
static void menu_adjust(int direction)
{
    float delta=g_steps[g_step_index]*(float)direction;
    if(!g_pointer->editable) return;
    if(g_pointer->kind==bool_Box) {
        bool enabled=direction>0;
        uint32 primask = interrupt_global_disable();
        if (g_pointer->data == (void *)&motor_position_enabled) {
            /* One menu action selects position mode; Run remains a separate action. */
            motor_run_enabled = false;
            if (enabled) motor_pwm_test_enabled = false;
        } else if (g_pointer->data == (void *)&motor_pwm_test_enabled) {
            motor_run_enabled = false;
            if (enabled) motor_position_enabled = false;
        }
        *(bool *)g_pointer->data = enabled;
        interrupt_global_enable(primask);
        if(g_pointer->data==&g_imu_recalibrate && enabled) {
            imu_request_recalibration();
            g_imu_recalibrate=false;
        } else if(g_pointer->data==&g_encoder_zero && enabled) {
            /* 只重置显示基线，不改变编码器反馈、PWM 或 PID。 */
            menu_motor_snapshot();
            for(unsigned wheel=0;wheel<MOTOR_WHEEL_COUNT;++wheel)
                g_encoder_zero_counts[wheel]=g_motor_snapshot.cumulative_raw_counts[wheel];
            g_encoder_zero=false;
        } else if(g_pointer->data==&g_navigation_zero && enabled) {
            app_navigation_request_reset();
            g_navigation_zero=false;
        } else if(g_pointer->data==&g_flash_save && enabled) {
            uint32 primask;
            g_flash_save=false;
            /* 擦写耗时约几十至上百毫秒且期间关中断：先停车，避免控制空窗 */
            primask=interrupt_global_disable();
            motor_run_enabled=false;
            interrupt_global_enable(primask);
            menu_flash_save_current();
        } else if(g_pointer->data==(void *)&route_run_flag) {
            /* Route/Run：On 启动路线（自动切入位置模式），Off 停车取消 */
            if(enabled) route_follow_start(); else route_follow_stop();
        }
        return;
    }
    if(g_pointer->kind==float_Box) {
        float value=*(float *)g_pointer->data+delta;
        if (g_pointer->data == (void *)&navigation_mount_deg)
        {
            value = menu_clamp(value, -180.0f, 180.0f);
        }
        else if (g_pointer->data == (void *)&navigation_scale_x ||
                 g_pointer->data == (void *)&navigation_scale_y)
            value = menu_clamp(value, 0.1f, 5.0f);
        else if (g_pointer->data == (void *)&motor_position_goal.x_cm ||
                 g_pointer->data == (void *)&motor_position_goal.y_cm)
            value = menu_clamp(value, -10000.0f, 10000.0f);
        else if (g_pointer->data == (void *)&motor_position_goal.yaw_deg)
            value = menu_clamp(value, -180.0f, 180.0f);
        else if (g_pointer->data == (void *)&motor_position_config.xy_kp ||
                 g_pointer->data == (void *)&motor_position_config.yaw_kp)
            value = menu_clamp(value, 0.01f, 20.0f);
        else if (g_pointer->data == (void *)&motor_position_config.xy_kd)
            value = menu_clamp(value, 0.0f, 5.0f);
        else if (g_pointer->data == (void *)&motor_position_config.max_speed_cmps)
            value = menu_clamp(value, 1.0f, 500.0f);
        else if (g_pointer->data == (void *)&motor_position_config.max_omega_radps)
            value = menu_clamp(value, 0.05f, 3.0f);
        else if (g_pointer->data == (void *)&motor_position_config.max_accel_cmps2)
            value = menu_clamp(value, 1.0f, 300.0f);
        else if (g_pointer->data == (void *)&motor_position_config.max_alpha_radps2)
            value = menu_clamp(value, 0.05f, 10.0f);
        else if (g_pointer->data == (void *)&motor_position_config.xy_tolerance_cm ||
                 g_pointer->data == (void *)&motor_position_config.yaw_tolerance_deg)
            value = menu_clamp(value, 0.5f, 20.0f);
        else if (g_pointer->data == (void *)&motor_position_config.brake_limit)
            value = menu_clamp(value, 0.05f, 2.0f);
        else if (g_pointer->data == (void *)&motor_position_config.brake_ceiling_cmps2)
            value = menu_clamp(value, 50.0f, 5000.0f);
        else if (g_pointer->data == (void *)&motor_position_config.short_segment_cm)
            value = menu_clamp(value, 1.0f, 200.0f);
        else if (g_pointer->data == (void *)&motor_position_config.short_boost_gain)
            value = menu_clamp(value, 1.0f, 3.0f);
        else if (g_pointer->data == (void *)&motor_position_config.approach_zone_cm)
            value = menu_clamp(value, 0.0f, 50.0f);
        else if (g_pointer->data == (void *)&motor_position_config.approach_ratio)
            value = menu_clamp(value, 0.0f, 1.0f);
        else if (g_pointer->data == (void *)&motor_position_config.approach_acc_k)
            value = menu_clamp(value, 0.05f, 0.95f);
        else if (g_pointer->data == (void *)&motor_position_config.yaw_lin_band_rad)
            value = menu_clamp(value, 0.02f, 1.0f);
        else if (g_pointer->data == (void *)&motor_position_config.yaw_kd ||
                 g_pointer->data == (void *)&motor_position_config.yaw_kd_translate)
            value = menu_clamp(value, 0.0f, 2.0f);
        else if (g_pointer->data == (void *)&motor_cmd_vx_cmps ||
            g_pointer->data == (void *)&motor_cmd_vy_cmps)
        {
            value = menu_clamp(value, -300.0f, 300.0f);
        }
        else if (g_pointer->data == (void *)&motor_cmd_omega_radps)
        {
            value = menu_clamp(value, -20.0f, 20.0f);
        }
        else if (route_follow_is_node_field(g_pointer->data))
        {
            /* Route 节点：X/Y ±10000cm，Yaw ±180°，允许负值 */
            value = route_follow_clamp_node_field(g_pointer->data, value);
        }
        else
        {
            value = menu_clamp(value, 0.0f, 1000.0f);
        }
        *(float *)g_pointer->data=value;
        if(route_follow_is_node_field(g_pointer->data)) route_follow_config_edited();
    } else if(g_pointer->kind==int16_Box) {
        int step=g_pointer->data==(void *)&route_node_count ? 1 : (int)g_steps[g_step_index],value;
        if(step<1) step=1;
        value=*(int16_t *)g_pointer->data+step*direction;
        if(g_pointer->data==(void *)&route_node_count) {
            value=Limit_int(1,value,ROUTE_MAX_NODES);
            *(int16_t *)g_pointer->data=(int16_t)value;
            route_follow_config_edited();
        } else {
            *(int16_t *)g_pointer->data=(int16_t)Limit_int(LIMIT_PWM_MIN,value,LIMIT_PWM_MAX);
        }
    }
}
void Menu_Init(void)
{
    ips200_set_dir(IPS200_PORTAIT);
    ips200_set_font(IPS200_8X16_FONT);
    ips200_set_color(RGB565_WHITE,RGB565_BLACK);
    ips200_init(IPS200_TYPE_SPI);
    ips200_clear(); key_init(20u);
    memset(&g_root,0,sizeof(g_root));
    g_root.name="MCAR"; g_root.kind=MENU_Folder;
    menu_create(); g_pointer=g_position_folder->First_Son;
    All_Folder_Menu_Init(&g_root);
    g_refresh_ticks=0;
    memset(g_encoder_zero_counts,0,sizeof(g_encoder_zero_counts));
    g_encoder_zero=false; g_refresh_pending=true;
}
void Menu_Tick_20ms(void)
{
    if(++g_refresh_ticks>=5u) { g_refresh_ticks=0; g_refresh_pending=true; }
}
void Menu_Show(void)
{
    if(!g_refresh_pending) return;
    g_refresh_pending=false;
    menu_motor_snapshot(); menu_draw();
}
void Menu_Switch(void)
{
    key_state_enum enter=key_get_state(KEY_1),up=key_get_state(KEY_2);
    key_state_enum back=key_get_state(KEY_3),down=key_get_state(KEY_4);
    bool repeat_numeric=g_pointer->selected && g_pointer->editable &&
                        (g_pointer->kind==float_Box || g_pointer->kind==int16_Box);
    bool up_repeat=up==KEY_LONG_PRESS || up==KEY_REPEAT_PRESS;
    bool down_repeat=down==KEY_LONG_PRESS || down==KEY_REPEAT_PRESS;
    if(enter!=KEY_SHORT_PRESS && up!=KEY_SHORT_PRESS && back!=KEY_SHORT_PRESS &&
       down!=KEY_SHORT_PRESS && !up_repeat && !down_repeat) return;
    g_refresh_pending=true;
    if(up==KEY_SHORT_PRESS || (repeat_numeric && up_repeat)) {
        if(g_pointer->selected) menu_adjust(1);
        else g_pointer=g_pointer->Last_Brother;
    } else if(down==KEY_SHORT_PRESS || (repeat_numeric && down_repeat)) {
        if(g_pointer->selected) menu_adjust(-1);
        else g_pointer=g_pointer->Next_Brother;
    } else if(enter==KEY_SHORT_PRESS) {
        if(g_pointer->kind==MENU_Folder && g_pointer->First_Son!=NULL)
            g_pointer=g_pointer->First_Son;
        else if(g_pointer->editable && !g_pointer->selected) g_pointer->selected=true;
        else if(g_pointer->editable)
            g_step_index=(uint8_t)((g_step_index+MENU_STEP_COUNT-1u)%MENU_STEP_COUNT);
    } else if(back==KEY_SHORT_PRESS) {
        if(g_pointer->selected) g_pointer->selected=false;
        else if(g_pointer->Father->Father!=NULL) g_pointer=g_pointer->Father;
    }
    key_clear_all_state();
}
