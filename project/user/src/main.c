#include "zf_common_headfile.h"
#include "Mymenu.h"
#include "app_control.h"
#include "Flash.h"
#include "route_follow.h"
#include "wifispi.h"
#include "imu.h"

#define PIT_SHARED_IRQ_PRIORITY 2u

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);
    debug_init();
    system_delay_ms(300);

    app_control_init();
    route_follow_init();
    /* Flash 参数存取：先初始化 FlexSPI ROM 驱动，再尝试加载上次保存的参数。
     * 首次运行或校验失败时返回 0，继续使用代码中的默认值。 */
    flash_init();
    menu_flash_load_current();
    imu_init();
    Menu_Init();

    /* CH0: 200 Hz IMU；CH1: 100 Hz 电机控制；CH2: 50 Hz 按键扫描。 */
    pit_ms_init(PIT_CH0, 5);
    pit_ms_init(PIT_CH1, 10);
    pit_ms_init(PIT_CH2, 20);
    interrupt_set_priority(PIT_IRQn, PIT_SHARED_IRQ_PRIORITY);
    interrupt_global_enable(0);
    /* Networking can block on startup. Keep IMU/control ticks running. */
    wifispi_telemetry_init();

    while (1)
    {
        /* 阻塞重初始化和 UDP 同步发送只在主循环执行。 */
        imu_service();
        wifispi_telemetry_service();
        Menu_Switch();
        Menu_Show();
    }
}
