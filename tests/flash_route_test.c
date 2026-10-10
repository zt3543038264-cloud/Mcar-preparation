/* Real Flash serializer/loader against an in-memory device, no hardware writes. */
#include "Flash.h"
#include "app_control.h"
#include "app_navigation.h"
#include "PID_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

flash_data_union flash_union_buffer[FLASH_DATA_BUFFER_SIZE];
static uint32 storage[FLASH_DATA_BUFFER_SIZE];
static unsigned present, erased, writes, recoveries, resets, irq_disabled, fail_erase, fail_write;
static uint16 last_write_length;
volatile bool motor_run_enabled, motor_position_enabled, motor_pwm_test_enabled;
volatile position_goal_t motor_position_goal;
volatile position_config_t motor_position_config = POSITION_CONFIG_DEFAULT;
volatile float navigation_mount_deg = 180, navigation_scale_x = 0.52f, navigation_scale_y = 0.61f;
volatile bool navigation_yaw_reversed;
uint32 interrupt_global_disable(void) { assert(!irq_disabled); irq_disabled=1; return 7; }
void interrupt_global_enable(uint32 previous) { assert(irq_disabled && previous==7); irq_disabled=0; }
void app_navigation_request_reset(void) { ++resets; }
void imu_recover_after_stall(void) { assert(!irq_disabled); ++recoveries; }
void app_control_get_position_snapshot(position_output_t *out) { memset(out,0,sizeof(*out)); }
void flash_buffer_clear(void) { memset(flash_union_buffer,0,sizeof(flash_union_buffer)); }
uint8 flash_check(uint32 sector, flash_page_enum page)
{ assert(sector==FLASH_SECTION_INDEX && page==FLASH_PAGE_INDEX); return (uint8)present; }
uint8 flash_erase_page(uint32 sector, flash_page_enum page)
{
    assert(!irq_disabled); (void)flash_check(sector,page); ++erased;
    if(fail_erase) return 1;
    memset(storage,0xff,sizeof(storage)); present=0; return 0;
}
uint8 flash_write_page(uint32 sector, flash_page_enum page,const uint32 *data,uint16 length)
{
    assert(!irq_disabled); (void)flash_check(sector,page); ++writes;
    last_write_length=length;
    if(fail_write) return 1;
    assert(length<=FLASH_DATA_BUFFER_SIZE); memcpy(storage,data,length*sizeof(uint32)); present=1; return 0;
}
void flash_read_page_to_buffer(uint32 sector, flash_page_enum page)
{ assert(!irq_disabled); (void)flash_check(sector,page); memcpy(flash_union_buffer,storage,sizeof(storage)); }
static void checksum(unsigned index)
{
    uint32 value=storage[0]^storage[1]^0xa5a55a5au;
    for(unsigned i=2;i<index;++i) value^=storage[i];
    storage[index]=value;
}
static void set_word(unsigned index,float value) { memcpy(&storage[index],&value,4); }
static void assert_nodes(const route_node_t *expected)
{
    for(unsigned i=0;i<ROUTE_MAX_NODES;++i) {
        assert(route_nodes[i].x_cm==expected[i].x_cm);
        assert(route_nodes[i].y_cm==expected[i].y_cm);
        assert(route_nodes[i].yaw_deg==expected[i].yaw_deg);
    }
}
int main(void)
{
    static const route_node_t defaults[ROUTE_MAX_NODES]=ROUTE_DEFAULT_NODES;
    route_node_t expected[ROUTE_MAX_NODES];
    menu_flash_config_t config, sentinel;
    PID_Init(&ULpid,&ULPidInitStruct); PID_Init(&URpid,&URPidInitStruct);
    PID_Init(&DLpid,&DLPidInitStruct); PID_Init(&DRpid,&DRPidInitStruct);
    assert(!Data_load_from_flash(&config));
    for(unsigned i=0;i<ROUTE_MAX_NODES;++i) {
        expected[i]=(route_node_t){-125.0f+(float)i*60.0f,240.0f-(float)i*90.0f,-180.0f+(float)i*45.0f};
        route_nodes[i]=expected[i];
    }
    route_node_count=6;
    ULpid.fKp=13.5f;
    route_follow_start(); route_state=ROUTE_ADVANCE; motor_run_enabled=true;
    assert(menu_flash_save_current());
    assert(!route_run_flag && !motor_run_enabled && route_state==ROUTE_IDLE);
    assert(last_write_length==53 && storage[1]==2 && storage[27]==6 && recoveries==1);
    assert(Data_load_from_flash(&config));
    assert(config.route_node_count==6 && config.wheel_kp[0]==13.5f);
    memset(route_nodes,0,sizeof(route_nodes)); route_node_count=1;
    ULpid.fKp=1;
    route_follow_init(); /* Same order as startup: runtime init before load. */
    assert(menu_flash_load_current());
    assert(route_node_count==6 && ULpid.fKp==13.5f && resets==1);
    assert_nodes(expected);
    assert(!route_run_flag && !motor_run_enabled && route_state==ROUTE_IDLE);
    route_follow_tick_10ms(); assert(!motor_run_enabled);
    /* Editing a value and saving again must overwrite the previous route. */
    route_nodes[7].yaw_deg=179; expected[7].yaw_deg=179;
    assert(menu_flash_save_current()); assert(erased==1);
    assert(Data_load_from_flash(&config));
    unsigned writes_before=writes, erases_before=erased;
    config.route_node_count=0; assert(!Data_save_to_flash(&config));
    config.route_node_count=9; assert(!Data_save_to_flash(&config));
    config.route_node_count=6; config.route_nodes[7].yaw_deg=NAN;
    assert(!Data_save_to_flash(&config));
    config.route_nodes[7].yaw_deg=181; assert(!Data_save_to_flash(&config));
    config.route_nodes[7].yaw_deg=179; config.route_nodes[7].x_cm=10001;
    assert(!Data_save_to_flash(&config));
    assert(writes==writes_before && erased==erases_before);
    assert(Data_load_from_flash(&config)); sentinel=config;
    storage[28]^=1; assert(!Data_load_from_flash(&config));
    assert(!memcmp(&config,&sentinel,sizeof(config))); storage[28]^=1;
    storage[27]=257; checksum(52); assert(!Data_load_from_flash(&config));
    storage[27]=6; set_word(51,INFINITY); checksum(52);
    assert(!Data_load_from_flash(&config));
    assert(menu_flash_save_current()); /* Replace corrupted stored data. */
    /* Version 1 has only PID/navigation data, at checksum word 27. */
    storage[1]=1; checksum(27);
    route_node_count=1; memset(route_nodes,0,sizeof(route_nodes));
    assert(menu_flash_load_current());
    assert(route_node_count==3 && ULpid.fKp==13.5f); assert_nodes(defaults);
    assert(!motor_run_enabled && !route_run_flag);
    assert(menu_flash_save_current()); assert(storage[1]==2 && storage[27]==3);
    storage[1]=99; checksum(52); assert(!Data_load_from_flash(&config));
    storage[1]=2; checksum(52);
    fail_erase=1; assert(!menu_flash_save_current()); fail_erase=0;
    fail_write=1; assert(!menu_flash_save_current()); fail_write=0;
    assert(!motor_run_enabled && !route_run_flag && !irq_disabled);
    assert(menu_flash_save_current());
    assert(Data_clear_flash() && !Data_load_from_flash(&config));
    puts("Flash route passed: all 8 nodes/count, reboot, v1 migration, checksum/ranges, stopped state, I/O errors");
    return 0;
}
