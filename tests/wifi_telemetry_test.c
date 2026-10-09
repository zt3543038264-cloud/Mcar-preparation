/* Actual enabled transport/command service with recorded UDP datagrams.
 * No hardware is driven. Verify malformed commands cannot touch motor control. */
#include "wifispi.h"
#include "imu.h"
#include "Motor.h"
#include "app_control.h"
#include "app_navigation.h"
#include "PID_config.h"
#include "zf_common_clock.h"
#include "zf_device_wifi_spi.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

host_dwt_t host_dwt;
host_core_debug_t host_core_debug;
uint32 system_clock=600000000u;
volatile int32 imu_attitude_status=1;
volatile float imu_roll_deg=12, imu_pitch_deg=-4, imu_yaw_deg=30;
volatile float imu_accel_g[3]={0,0,1}, imu_gyro_dps[3]={1,2,3};
volatile float imu_calibration_percent=100;
volatile bool motor_run_enabled, motor_position_enabled=true, motor_pwm_test_enabled;
volatile float motor_cmd_vx_cmps=5, motor_cmd_vy_cmps=6, motor_cmd_omega_radps=0.2f;
volatile position_goal_t motor_position_goal={100,100,90};
volatile position_config_t motor_position_config=POSITION_CONFIG_DEFAULT;
volatile float navigation_scale_x=0.52f, navigation_scale_y=0.61f;
char wifi_spi_version[12], wifi_spi_ip_addr_port[25];
static navigation_snapshot_t nav={.status=2,.x_m=0.25f,.y_m=-0.5f,.yaw_deg=40,.valid=true};
static position_output_t pos={.status=1,.distance_cm=25,.yaw_error_deg=50};
static motor_speed_debug_snapshot_t motor={.raw_counts={10,11,12,6},.final_pwm={100,200,300,400}};
static unsigned irq_disabled, fail_stage, init_calls, udp_calls, rx_calls;
static unsigned fail_send;
static uint8_t outgoing[1400];
static uint32 outgoing_size;
static uint8_t datagrams[32][1400];
static uint32 sizes[32], packet_count;
static uint8_t incoming[4096];
static size_t incoming_size;
uint32 interrupt_global_disable(void) { assert(!irq_disabled); irq_disabled=1; return 7; }
void interrupt_global_enable(uint32 mask) { assert(irq_disabled && mask==7); irq_disabled=0; }
void app_navigation_get_snapshot(navigation_snapshot_t *out) { assert(irq_disabled); *out=nav; }
void app_control_get_position_snapshot(position_output_t *out) { assert(irq_disabled); *out=pos; }
void motor_speed_debug_get_snapshot(motor_speed_debug_snapshot_t *out) { assert(irq_disabled); *out=motor; }
void system_delay_ms(uint32 ms) { assert(!irq_disabled); (void)ms; }
uint8 wifi_spi_init(char *ssid,char *password) {
    assert(!irq_disabled && ssid==NULL && password==NULL); ++init_calls;
    strcpy(wifi_spi_version,"V2.0"); return fail_stage==1;
}
uint8 wifi_spi_wifi_connect(char *ssid,char *password) {
    assert(!irq_disabled && ssid!=NULL && password!=NULL); return fail_stage==2;
}
uint8 wifi_spi_socket_connect(char *type,char *ip,char *port,char *local) {
    assert(!irq_disabled && strcmp(type,"UDP")==0 && ip && port && strcmp(local,"5001")==0);
    return fail_stage==3;
}
uint32 wifi_spi_send_buffer(const uint8 *data,uint32 size) {
    assert(!irq_disabled && size<=sizeof(outgoing));
    if(fail_send) return size;
    memcpy(outgoing,data,size); outgoing_size=size; return 0;
}
uint8 wifi_spi_udp_send_now(void) {
    assert(!irq_disabled && packet_count<32); ++udp_calls;
    memcpy(datagrams[packet_count],outgoing,outgoing_size);
    sizes[packet_count++]=outgoing_size; return 0;
}
uint32 wifi_spi_read_buffer(uint8 *data,uint32 length) {
    size_t count=incoming_size<length?incoming_size:length;
    assert(!irq_disabled); ++rx_calls;
    memcpy(data,incoming,count); memmove(incoming,incoming+count,incoming_size-count);
    incoming_size-=count; return (uint32)count;
}
static void advance(unsigned ms) {
    host_dwt.CYCCNT+=(system_clock/1000u)*ms;
    wifispi_telemetry_service();
}
static void queue_bytes(const void *bytes,size_t count) {
    assert(count+incoming_size<=sizeof(incoming));
    memcpy(incoming+incoming_size,bytes,count); incoming_size+=count;
}
static void command(const char *line) {
    packet_count=0; queue_bytes(line,strlen(line));
    do { advance(10); } while(incoming_size);
}
static int has_reply(const char *expected) {
    for(unsigned i=0;i<packet_count;++i)
        if(sizes[i]==strlen(expected) && memcmp(datagrams[i],expected,sizes[i])==0) return 1;
    return 0;
}
static float last_value(unsigned channel,unsigned count) {
    float value; unsigned i=packet_count-1;
    assert(sizes[i]==VOFA_FRAME_BYTES(count));
    assert(memcmp(datagrams[i]+4*count,"\0\0\x80\x7f",4)==0);
    memcpy(&value,datagrams[i]+4*channel,4); return value;
}
static void test_slider_packets(void) {
    command("[slider,pos_xy_kp,2.5]");
    assert(motor_position_config.xy_kp==2.5f && has_reply("MCAR SLIDER pos_xy_kp 2.5\n"));
    command("[\"slider\",\"pos_xy_kp\",2.75]\n");
    assert(motor_position_config.xy_kp==2.75f && has_reply("MCAR SLIDER pos_xy_kp 2.75\n"));
    command(" [ slider , goal_x_cm , -1.25e2 ]\r\n");
    assert(motor_position_goal.x_cm==-125 && has_reply("MCAR SLIDER goal_x_cm -125\n"));
    command("[slider,pos_xy_kp,"); assert(packet_count==0);
    command("3.0][slider,pos_xy_kd,0.5]RATE 20\n");
    assert(motor_position_config.xy_kp==3 && motor_position_config.xy_kd==0.5f);
    assert(has_reply("MCAR SLIDER pos_xy_kp 3\n") && has_reply("MCAR SLIDER pos_xy_kd 0.5\n"));
    assert(has_reply("MCAR RATE 20\n"));
    const char *bad_packets[]={
        "[slider,pos_xy_kp,NaN]", "[slider,pos_xy_kp,Infinity]", "[slider,pos_xy_kp,1e100]",
        "[slider,pos_xy_kp,\"4\"]", "[slider,pos_xy_kp,4,5]", "[slider,pos_xy_kp,4x]",
        "[slider,pos_xy_kp,0x1p0]", "[slider,pos_xy_kp,1e]", "[slider,pos_xy_kp,]",
        "[slider,pos_xy_kp]", "[slider,\"pos_xy_kp,4]", "[slider,,4]",
        "[slider,pos_xy_kp,4\n", "[slider,pos_xy_kp,[slider,pos_xy_kp,4]]\n"
    };
    for(unsigned i=0;i<sizeof(bad_packets)/sizeof(bad_packets[0]);++i) {
        command(bad_packets[i]);
        assert(motor_position_config.xy_kp==3);
        assert(has_reply("MCAR ERR expected [slider,parameter,number]\n"));
    }
    command("[slider,pos_xy_kp,20.1]");
    assert(has_reply("MCAR ERR parameter out of range\n") && motor_position_config.xy_kp==3);
    command("[slider,not_a_parameter,4]"); assert(has_reply("MCAR ERR unknown parameter\n"));
    command("[slider,run,1]"); assert(!motor_run_enabled && has_reply("MCAR ERR unknown parameter\n"));
    command("[button,pos_xy_kp,4]"); assert(has_reply("MCAR ERR unknown control type\n"));
    motor_run_enabled=true;
    command("[slider,scale_x,0.6]");
    assert(navigation_scale_x==0.52f && has_reply("MCAR ERR parameter requires Run off\n"));
    command("[slider,goal_y_cm,200]");
    assert(motor_position_goal.y_cm==100 && has_reply("MCAR ERR parameter requires Run off\n"));
    command("[slider,pos_xy_kp,4]"); assert(motor_position_config.xy_kp==4);
    ULpid.fError[0]=91;
    command("[slider,ul_kp,11]");
    assert(ULPidInitStruct.fKp==11 && ULpid.fKp==11 && ULpid.fError[0]==0);
    ULpid.fError[0]=91;
    command("[slider,ul_kp,11]"); assert(ULpid.fError[0]==91);
    assert(motor_run_enabled && motor_position_enabled && !motor_pwm_test_enabled);
    motor_run_enabled=false;
    command("[slider,scale_x,0.6][slider,goal_y_cm,200][slider,dr_kd,30]");
    assert(navigation_scale_x==0.6f && motor_position_goal.y_cm==200);
    assert(DRPidInitStruct.fKd==30 && DRpid.fKd==30 && !motor_run_enabled);
    char oversized_slider[1100]; memset(oversized_slider,'x',sizeof(oversized_slider));
    oversized_slider[0]='[';queue_bytes(oversized_slider,sizeof(oversized_slider));
    command("][slider,pos_xy_kp,5]");
    assert(has_reply("MCAR ERR invalid or oversized line\n") && motor_position_config.xy_kp==5);
    command("[slider,pos_xy_kp,6]");
    const char invalid_slider[]={0,'7',']'};queue_bytes(invalid_slider,sizeof(invalid_slider));command("\n");
    assert(motor_position_config.xy_kp==6);
    /* Reject a binary byte in an unfinished frame, then recover on ']'. */
    command("[slider,pos_xy_kp,");queue_bytes(invalid_slider,sizeof(invalid_slider));command("\n");
    assert(has_reply("MCAR ERR invalid or oversized line\n") && motor_position_config.xy_kp==6);
    command("RATE 2\n");
    puts("slider packets passed: quoted/plain, fragmented/concatenated, gains, bounds, Run isolation");
}
int main(void) {
    assert(IMU_WIFI_ENABLED==1);
    wifispi_telemetry_init();
    assert(imu_wifi_status==1 && init_calls==1 && wifi_telemetry_channel_count==3);
    advance(9); assert(packet_count==0 && rx_calls==0);
    advance(1); assert(last_value(0,3)==12 && last_value(2,3)==30);
    command(" SUB x_cm, y_cm, nav_yaw_deg,dr_raw,scale_x,scale_y \r\n");
    assert(has_reply("MCAR SUB x_cm,y_cm,nav_yaw_deg,dr_raw,scale_x,scale_y\n"));
    assert(last_value(0,6)==25 && last_value(1,6)==-50 && last_value(2,6)==40);
    assert(last_value(3,6)==6 && last_value(4,6)==0.52f && last_value(5,6)==0.61f);
    command("SUB x_cm,no_such_variable\n");
    assert(has_reply("MCAR ERR unknown variable\n") && wifi_telemetry_channel_count==6);
    assert(last_value(1,6)==-50);
    command("SUB x_cm,x_cm\n"); assert(has_reply("MCAR ERR duplicate variable\n"));
    command("SUB x_cm,\n"); assert(has_reply("MCAR ERR invalid channel count\n"));
    command("SUB \n"); assert(wifi_telemetry_channel_count==6);
    command("RATE 0\nRATE -1\nRATE 1001\nRATE 2x\nRATE 42949672960\n");
    assert(wifi_telemetry_period_ms==10);
    command("RATE 2\n"); assert(wifi_telemetry_period_ms==2);
    packet_count=0; advance(1); assert(packet_count==0); advance(1); assert(packet_count==1);
    command("STREAM 0\n"); assert(has_reply("MCAR STREAM 0\n") && packet_count==1);
    packet_count=0; advance(1000); assert(packet_count==0);
    command("GET?\n"); assert(packet_count==3 && has_reply("MCAR RATE 2\n") && has_reply("MCAR STREAM 0\n"));
    command("SUB x_"); assert(packet_count==0);
    command("cm,y_cm\nRATE 1000\n");
    assert(has_reply("MCAR SUB x_cm,y_cm\n") && wifi_telemetry_period_ms==1000);
    command("LIST?\n");
    assert(packet_count==1 && sizes[0]<1400 && memcmp(datagrams[0],"MCAR LIST ",10)==0);
    char list[1400], selection[1024]="SUB ";
    memcpy(list,datagrams[0]+10,sizes[0]-10); list[sizes[0]-11]='\0';
    char *name=strtok(list,",");
    for(unsigned i=0;i<41;++i) {
        assert(name); if(i) strcat(selection,","); strcat(selection,name);
        if(i==39) { char max[1024]; strcpy(max,selection); strcat(max,"\n"); command(max); assert(wifi_telemetry_channel_count==40); }
        name=strtok(NULL,",");
    }
    strcat(selection,"\n"); command(selection);
    assert(has_reply("MCAR ERR invalid channel count\n") && wifi_telemetry_channel_count==40);
    char oversized[1200]; memset(oversized,'a',sizeof(oversized));
    queue_bytes(oversized,sizeof(oversized)); command("\nSUB dr_pwm,ul_raw\n");
    assert(wifi_telemetry_channel_count==2 && has_reply("MCAR ERR invalid or oversized line\n"));
    const char invalid[]={ 'S','U','B',' ',0,'x','\n' };
    queue_bytes(invalid,sizeof(invalid)); command("GET?\n");
    assert(has_reply("MCAR ERR invalid or oversized line\n"));
    command("RUN 1\nSUB motor_run_enabled\nSTREAM 2\n");
    assert(!motor_run_enabled && motor_position_enabled && !motor_pwm_test_enabled);
    assert(motor_position_goal.x_cm==100 && navigation_scale_x==0.52f);
    test_slider_packets();
    command("RATE 10\nSTREAM 1\n"); assert(last_value(0,2)==400 && last_value(1,2)==10);
    packet_count=0; host_dwt.CYCCNT=0xfffffff0u; wifispi_telemetry_service();
    packet_count=0; advance(10); assert(packet_count==1); /* DWT wrap */
    fail_send=1; advance(10); assert(imu_wifi_status==-4);
    fail_send=0; packet_count=0; advance(10); assert(imu_wifi_status==1);
    for(fail_stage=1;fail_stage<=3;++fail_stage) {
        wifispi_telemetry_init(); assert(imu_wifi_status==-(int)fail_stage);
        assert(imu_wifi_init_attempts==3 && imu_wifi_last_error==-(int)fail_stage);
        packet_count=0; advance(1000); assert(packet_count==0);
    }
    puts("wifi_telemetry_test passed"); return 0;
}
