#include "route_follow.h"
#include "app_control.h"

#include <stddef.h>

/* 默认路线：(0,0) -> (20,40) -> (50,80)，可在菜单 Route 中修改。
 * 坐标是 Navigation/Zero 坐标系下的 cm；先静止 Zero，再开 Run。 */
bool route_run_flag;
volatile int16_t route_node_count = ROUTE_DEFAULT_NODE_COUNT;
volatile int32_t route_state = ROUTE_IDLE;
volatile int32_t route_current_idx;
route_node_t route_nodes[ROUTE_MAX_NODES] = ROUTE_DEFAULT_NODES;

/* 切入位置模式后等待的控制周期数：一拍被模式切换逻辑消耗，再留一拍稳定 */
#define ROUTE_ARM_SETTLE_TICKS 2U

static uint32 route_arm_ticks;

void route_follow_init(void)
{
    route_run_flag = false;
    route_state = ROUTE_IDLE;
    route_current_idx = 0;
    route_arm_ticks = 0U;
}

static uint8 route_count_clamped(void)
{
    int16_t count = route_node_count;

    if (count < 1) count = 1;
    if (count > ROUTE_MAX_NODES) count = ROUTE_MAX_NODES;
    return (uint8)count;
}

static void route_stop_internal(void)
{
    motor_run_enabled = false;
    route_run_flag = false;
    route_state = ROUTE_IDLE;
}

void route_follow_start(void)
{
    /* 运行中重复置 On 不重启 */
    if (route_state == ROUTE_ARMING || route_state == ROUTE_RUNNING ||
        route_state == ROUTE_ADVANCE)
    {
        return;
    }

    route_node_count = (int16_t)route_count_clamped();
    route_current_idx = 0;
    route_arm_ticks = 0U;
    route_state = ROUTE_ARMING;
    route_run_flag = true;
}

void route_follow_stop(void)
{
    route_stop_internal();
}

void route_follow_config_edited(void)
{
    if (route_state == ROUTE_ARMING || route_state == ROUTE_RUNNING ||
        route_state == ROUTE_ADVANCE)
    {
        route_stop_internal();
    }
}

uint8 route_follow_is_node_field(const void *ptr)
{
    const uint8 *p = (const uint8 *)ptr;

    return (p >= (const uint8 *)route_nodes &&
            p < (const uint8 *)(route_nodes + ROUTE_MAX_NODES)) ? 1U : 0U;
}

float route_follow_clamp_node_field(const void *field, float value)
{
    size_t offset = (size_t)((const uint8 *)field - (const uint8 *)route_nodes);

    if (!route_follow_is_node_field(field))
    {
        return value;
    }
    if (offset % sizeof(route_node_t) == offsetof(route_node_t, yaw_deg))
    {
        if (value > 180.0f) value = 180.0f;
        if (value < -180.0f) value = -180.0f;
    }
    else
    {
        if (value > 10000.0f) value = 10000.0f;
        if (value < -10000.0f) value = -10000.0f;
    }
    return value;
}

static void route_set_goal(uint8 idx)
{
    motor_position_goal.x_cm = route_nodes[idx].x_cm;
    motor_position_goal.y_cm = route_nodes[idx].y_cm;
    motor_position_goal.yaw_deg = route_nodes[idx].yaw_deg;
}

void route_follow_tick_10ms(void)
{
    position_output_t snapshot;
    uint8 count;
    int32_t state = route_state;

    if (state == ROUTE_IDLE || state == ROUTE_DONE || state == ROUTE_FAULT)
    {
        return;
    }

    count = route_count_clamped();

    if (state == ROUTE_ARMING)
    {
        /* 强制位置模式：模式切换那一拍 app_control 会清零 Run，
         * 等切换完成后再写目标并开 Run。 */
        motor_pwm_test_enabled = false;
        motor_position_enabled = true;
        motor_run_enabled = false;
        route_arm_ticks++;
        if (route_arm_ticks >= ROUTE_ARM_SETTLE_TICKS)
        {
            route_set_goal(0U);
            motor_run_enabled = true;
            route_state = ROUTE_RUNNING;
        }
        return;
    }

    if (state == ROUTE_ADVANCE)
    {
        /* 上一拍位置环已到点并自动关 Run；这一拍换新目标并重新开 Run */
        route_set_goal((uint8)route_current_idx);
        motor_run_enabled = true;
        route_state = ROUTE_RUNNING;
        return;
    }

    /* ROUTE_RUNNING */
    if (!motor_position_enabled || motor_pwm_test_enabled)
    {
        /* 运行中被手动切走模式：放弃路线 */
        route_run_flag = false;
        route_state = ROUTE_IDLE;
        return;
    }

    app_control_get_position_snapshot(&snapshot);
    if (snapshot.status == POSITION_REACHED)
    {
        if (route_current_idx + 1 >= count)
        {
            /* 终点到达，app_control 已关 Run 并清零输出 */
            route_run_flag = false;
            route_state = ROUTE_DONE;
        }
        else
        {
            route_current_idx++;
            route_state = ROUTE_ADVANCE;
        }
        return;
    }
    if (snapshot.status < 0)
    {
        /* 定位无效或参数非法：停车并报错，重新 Run 可再次启动 */
        motor_run_enabled = false;
        route_run_flag = false;
        route_state = ROUTE_FAULT;
        return;
    }
    if (!motor_run_enabled)
    {
        /* 非到点/非故障却被关了 Run（手动关闭等）：取消路线 */
        route_run_flag = false;
        route_state = ROUTE_IDLE;
    }
}
