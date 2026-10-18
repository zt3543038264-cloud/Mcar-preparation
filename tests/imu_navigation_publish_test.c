/* Exercise real imu.c + attitude.c with SPI samples and a simulated DWT clock.
 * Real device/SPI headers provide register addresses and enum definitions. */
#include "imu.h"
#include "config.h"
#include "zf_common_clock.h"
#include "zf_device_imu660ra.h"
#include "zf_driver_gpio.h"
#include "zf_driver_spi.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

host_dwt_t host_dwt;
host_core_debug_t host_core_debug;
uint32 system_clock=600000000u;
static uint8 registers[128], ready=1, cs_level=1, init_result;
static uint32 irq_disabled;
static int16 raw_accel[3]={0,0,4096}, raw_gyro[3]={64,-64,128};

uint8 imu660ra_init(void) { return init_result; }
void gpio_set_level(gpio_pin_enum pin,uint8 level)
{ assert(pin==C20); cs_level=level; }
void system_delay_ms(uint32 ms) { DWT->CYCCNT+=(system_clock/1000u)*ms; }
uint32 interrupt_global_disable(void)
{ assert(!irq_disabled); irq_disabled=1; return 0; }
void interrupt_global_enable(uint32 primask)
{ assert(irq_disabled && primask==0); irq_disabled=0; }
void spi_write_8bit_register(spi_index_enum spi,const uint8 reg,const uint8 value)
{ assert(spi==SPI_4 && cs_level==0 && reg<128); registers[reg]=value; }
void spi_read_8bit_registers(spi_index_enum spi,const uint8 reg,uint8 *data,uint32 len)
{
    assert(spi==SPI_4 && cs_level==0);
    memset(data,0,len);
    uint8 address=reg & 0x7fu;
    if(len==2) {
        data[1]=address==0x03u ? (ready?0xC0u:0u) : registers[address];
    } else {
        assert(address==IMU660RA_ACC_ADDRESS && len==13);
        for(unsigned i=0;i<3;++i) {
            uint16 a=(uint16)raw_accel[i],g=(uint16)raw_gyro[i];
            data[1+2*i]=(uint8)a; data[2+2*i]=(uint8)(a>>8);
            data[7+2*i]=(uint8)g; data[8+2*i]=(uint8)(g>>8);
        }
    }
}
static void tick(void)
{
    DWT->CYCCNT+=system_clock/200u;
    imu_update_5ms();
    assert(cs_level==1);
}
static void calibrate(void)
{
    assert(imu_attitude_status==IMU_ATTITUDE_CALIBRATING);
    for(unsigned i=0;i<CALIBRATION_SAMPLES+1u;++i) tick();
    assert(imu_attitude_status==IMU_ATTITUDE_RUNNING);
}
int main(void)
{
    imu_navigation_sample_t first, sample;
    assert(!imu_get_navigation_sample(&sample));
    imu_init();
    assert(!imu_get_navigation_sample(&sample));
    assert(imu_sample_sequence==0 && imu_accel_used==-1 && imu_ekf_nis==-1);
    assert(imu_gyro_z_integral_deg==0);
    calibrate();
    assert(imu_sample_sequence==1 && fabsf(imu_sample_dt_ms-5)<0.0001f);
    assert(imu_get_navigation_sample(&first) && first.sequence==1u);
    assert(first.generation>0 && fabsf(first.dt_s-0.005f)<1e-6f);
    assert(first.accel_g[2]==1 && first.accel_g[0]==0);
    for(unsigned i=0;i<3;++i) assert(fabsf(first.gyro_dps[i])<0.0001f);
    assert(imu_gyro_dps[0]>0.9f); /* Raw sensor debug remains uncorrected. */
    raw_gyro[2]+=656; raw_accel[0]=410;
    tick();
    assert(imu_get_navigation_sample(&sample) && sample.sequence==2u);
    assert(sample.generation==first.generation);
    assert(fabsf(sample.gyro_dps[2]-10.0f)<0.0001f);
    assert(sample.yaw_deg>0 && sample.yaw_deg==imu_yaw_deg);
    assert(sample.roll_deg==imu_roll_deg && sample.pitch_deg==imu_pitch_deg);
    assert(sample.accel_g[0]==imu_accel_g[0] && fabsf(sample.accel_g[0]-0.1f)<0.001f);
    assert(imu_sample_sequence==2 && imu_accel_used==1);
    assert(fabsf(imu_gyro_z_integral_deg-.05f)<1e-5f);
#if AHRS_METHOD == AHRS_METHOD_EKF
    assert(imu_ekf_nis>=0);
#else
    assert(imu_ekf_nis==-1);
#endif
    raw_accel[2]=8192; tick();
    assert(imu_sample_sequence==3 && imu_accel_used==0 && imu_ekf_nis==-1);
    assert(imu_solve_us>=0 && imu_solve_max_us>=imu_solve_us);
    raw_accel[2]=4096;
    ready=0; tick();
    assert(imu_get_navigation_sample(&sample) && sample.sequence==3u);
    assert(imu_sample_sequence==3);
    for(unsigned i=0;i<21;++i) tick();
    assert(imu_attitude_status==IMU_ATTITUDE_TIMEOUT);
    assert(!imu_get_navigation_sample(&sample));
    ready=1; raw_gyro[2]=128; raw_accel[0]=0;
    imu_request_recalibration(); imu_service();
    assert(!imu_get_navigation_sample(&sample));
    assert(imu_sample_sequence==0 && imu_accel_used==-1 && imu_ekf_nis==-1);
    assert(imu_gyro_z_integral_deg==0);
    calibrate();
    assert(imu_get_navigation_sample(&sample) && sample.sequence==1u);
    assert(sample.generation!=first.generation && fabsf(sample.yaw_deg)<0.001f);
    DWT->CYCCNT+=system_clock/20u; tick();
    assert(imu_attitude_status==IMU_ATTITUDE_BAD_DT);
    assert(!imu_get_navigation_sample(&sample));
    init_result=1; imu_init();
    assert(imu_attitude_status==IMU_ATTITUDE_INIT_FAILED);
    assert(!imu_get_navigation_sample(&sample));
    printf("IMU publish passed (method %u): coherent frame, corrected gyro, fresh sequence, generation, timeout/dt faults\n",
           (unsigned)imu_attitude_method);
    return 0;
}
