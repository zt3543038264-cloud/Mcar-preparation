/* Reuse transport fakes; exercise diagnostics without legacy goal-bound assumptions. */
#define main legacy_wifi_main
#include "wifi_telemetry_test.c"
#undef main
int main(void) {
    wifispi_telemetry_init();
    command("SUB gyro_z_integral_deg\n");
    assert(last_value(0,1)==12.5f);
    command("SUB imu_dt_ms,imu_solve_us,imu_solve_max_us,imu_accel_used,ekf_nis,ekf_bias_x_dps,ekf_bias_y_dps,imu_seq,imu_method,accel_norm_g\n");
    const float expected[]={5,21,35,1,0.25f,0.1f,-0.2f,1234,3,1.01f};
    for(unsigned i=0;i<10;++i) assert(last_value(i,10)==expected[i]);
    imu_accel_used=0; imu_ekf_nis=-1;
    command("GET?\n"); assert(last_value(3,10)==0 && last_value(4,10)==-1);
    command("SUB roll_deg,pitch_deg,yaw_deg,gyro_x_dps,gyro_y_dps,gyro_z_dps,accel_x_g,accel_y_g,accel_z_g,gz_bias_dps,still_s,imu_status,nav_yaw_deg,cmd_omega_radps,ul_cmps,ur_cmps,dl_cmps,dr_cmps,ul_target,ur_target,dl_target,dr_target,imu_dt_ms,imu_solve_us,imu_solve_max_us,imu_accel_used,ekf_nis,ekf_bias_x_dps,ekf_bias_y_dps,imu_seq,imu_method,accel_norm_g\n");
    assert(wifi_telemetry_channel_count==32 && sizes[packet_count-1]==VOFA_FRAME_BYTES(32));
    command("LIST?\n");
    int found=0;
    for(unsigned i=0;i<packet_count;++i) if(sizes[i]>10 && memcmp(datagrams[i],"MCAR LIST ",10)==0) {
        assert(sizes[i]<1400); found=1;
    }
    assert(found && !motor_run_enabled);
    puts("IMU diagnostics UDP: exact values, unavailable NIS, 32-channel subscription, LIST size passed");
    return 0;
}
