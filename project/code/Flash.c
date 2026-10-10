#include "Flash.h"
#include "PID_config.h"
#include "app_control.h"
#include "app_navigation.h"
#include "imu.h"
#include "zf_common_interrupt.h"

#include <string.h>

/* v3 adds the ivision-style position planner parameters. v2 has the route,
 * v1 keeps PID/navigation and uses default nodes; both load with planner
 * defaults. Run and runtime state are never persisted. Unknown versions
 * are rejected. */
#define MENU_FLASH_MAGIC 0x4D454E55U
#define MENU_FLASH_VERSION 3U
#define MENU_FLASH_VERSION_V2 2U
#define MENU_FLASH_LEGACY_VERSION 1U
#define MENU_FLASH_CHECK_XOR 0xA5A55A5AU

#define MENU_FLASH_WORD_MAGIC 0U
#define MENU_FLASH_WORD_VERSION 1U
#define MENU_FLASH_WORD_KP_FIRST 2U   /* +MOTOR_WHEEL_COUNT */
#define MENU_FLASH_WORD_KI_FIRST 6U   /* +MOTOR_WHEEL_COUNT */
#define MENU_FLASH_WORD_KD_FIRST 10U  /* +MOTOR_WHEEL_COUNT */
#define MENU_FLASH_WORD_XY_KP 14U
#define MENU_FLASH_WORD_XY_KD 15U
#define MENU_FLASH_WORD_YAW_KP 16U
#define MENU_FLASH_WORD_MAX_SPEED 17U
#define MENU_FLASH_WORD_MAX_OMEGA 18U
#define MENU_FLASH_WORD_MAX_ACCEL 19U
#define MENU_FLASH_WORD_MAX_ALPHA 20U
#define MENU_FLASH_WORD_TOL_XY 21U
#define MENU_FLASH_WORD_TOL_YAW 22U
#define MENU_FLASH_WORD_MOUNT_DEG 23U
#define MENU_FLASH_WORD_SCALE_X 24U
#define MENU_FLASH_WORD_SCALE_Y 25U
#define MENU_FLASH_WORD_FLAGS 26U
#define MENU_FLASH_LEGACY_CHECKSUM 27U
#define MENU_FLASH_WORD_ROUTE_COUNT 27U
#define MENU_FLASH_WORD_ROUTE_FIRST 28U
#define MENU_FLASH_WORD_CHECKSUM_V2 (MENU_FLASH_WORD_ROUTE_FIRST + 3U * ROUTE_MAX_NODES)
#define MENU_FLASH_WORD_COUNT_V2 (MENU_FLASH_WORD_CHECKSUM_V2 + 1U)
/* v3：路线块之后追加 10 个规划参数字，校验字后移 */
#define MENU_FLASH_WORD_PLANNER_FIRST MENU_FLASH_WORD_CHECKSUM_V2
#define MENU_FLASH_WORD_BRAKE_LIMIT (MENU_FLASH_WORD_PLANNER_FIRST + 0U)
#define MENU_FLASH_WORD_BRAKE_CEILING (MENU_FLASH_WORD_PLANNER_FIRST + 1U)
#define MENU_FLASH_WORD_SHORT_SEGMENT (MENU_FLASH_WORD_PLANNER_FIRST + 2U)
#define MENU_FLASH_WORD_SHORT_BOOST (MENU_FLASH_WORD_PLANNER_FIRST + 3U)
#define MENU_FLASH_WORD_APP_ZONE (MENU_FLASH_WORD_PLANNER_FIRST + 4U)
#define MENU_FLASH_WORD_APP_RATIO (MENU_FLASH_WORD_PLANNER_FIRST + 5U)
#define MENU_FLASH_WORD_APP_ACC_K (MENU_FLASH_WORD_PLANNER_FIRST + 6U)
#define MENU_FLASH_WORD_YAW_BAND (MENU_FLASH_WORD_PLANNER_FIRST + 7U)
#define MENU_FLASH_WORD_YAW_KD (MENU_FLASH_WORD_PLANNER_FIRST + 8U)
#define MENU_FLASH_WORD_YAW_KD_TRN (MENU_FLASH_WORD_PLANNER_FIRST + 9U)
#define MENU_FLASH_WORD_CHECKSUM (MENU_FLASH_WORD_PLANNER_FIRST + 10U)
#define MENU_FLASH_WORD_COUNT (MENU_FLASH_WORD_CHECKSUM + 1U)

#define MENU_FLAG_YAW_REVERSED (1UL << 0)

static uint32 menu_flash_float_to_word(float value)
{
    uint32 word;
    memcpy(&word, &value, sizeof(word));
    return word;
}

static float menu_flash_word_to_float(uint32 word)
{
    float value;
    memcpy(&value, &word, sizeof(value));
    return value;
}

/* 校验字在缓冲区内计算，保存前和读回后各调用一次；范围随版本不同 */
static uint32 menu_flash_checksum(void)
{
    uint32 version = flash_union_buffer[MENU_FLASH_WORD_VERSION].uint32_type;
    uint32 end;
    uint32 checksum = MENU_FLASH_MAGIC ^ version ^ MENU_FLASH_CHECK_XOR;
    uint32 index;

    if (version == MENU_FLASH_LEGACY_VERSION) end = MENU_FLASH_LEGACY_CHECKSUM;
    else if (version == MENU_FLASH_VERSION_V2) end = MENU_FLASH_WORD_CHECKSUM_V2;
    else end = MENU_FLASH_WORD_CHECKSUM;

    for (index = MENU_FLASH_WORD_KP_FIRST; index < end; index++)
    {
        checksum ^= flash_union_buffer[index].uint32_type;
    }
    return checksum;
}

static uint8 menu_flash_buffer_valid(void)
{
    uint32 version = flash_union_buffer[MENU_FLASH_WORD_VERSION].uint32_type;
    uint32 check;

    if (version == MENU_FLASH_LEGACY_VERSION) check = MENU_FLASH_LEGACY_CHECKSUM;
    else if (version == MENU_FLASH_VERSION_V2) check = MENU_FLASH_WORD_CHECKSUM_V2;
    else check = MENU_FLASH_WORD_CHECKSUM;
    return (flash_union_buffer[MENU_FLASH_WORD_MAGIC].uint32_type == MENU_FLASH_MAGIC &&
            (version == MENU_FLASH_VERSION || version == MENU_FLASH_VERSION_V2 ||
             version == MENU_FLASH_LEGACY_VERSION) &&
            flash_union_buffer[check].uint32_type == menu_flash_checksum()) ? 1U : 0U;
}

/* 拒绝 NaN、无穷大和明显离谱的数值，防止坏数据直接进入 PID */
static uint8 menu_flash_float_valid(float value)
{
    return (value == value && value > -1.0e9f && value < 1.0e9f) ? 1U : 0U;
}

static uint8 menu_flash_config_valid(const menu_flash_config_t *config)
{
    uint8 wheel;

    for (wheel = 0U; wheel < MOTOR_WHEEL_COUNT; wheel++)
    {
        if (!menu_flash_float_valid(config->wheel_kp[wheel]) || config->wheel_kp[wheel] < 0.0f) return 0U;
        if (!menu_flash_float_valid(config->wheel_ki[wheel]) || config->wheel_ki[wheel] < 0.0f) return 0U;
        if (!menu_flash_float_valid(config->wheel_kd[wheel]) || config->wheel_kd[wheel] < 0.0f) return 0U;
    }
    if (!menu_flash_float_valid(config->xy_kp) || config->xy_kp <= 0.0f) return 0U;
    if (!menu_flash_float_valid(config->xy_kd) || config->xy_kd < 0.0f) return 0U;
    if (!menu_flash_float_valid(config->yaw_kp) || config->yaw_kp <= 0.0f) return 0U;
    if (!menu_flash_float_valid(config->max_speed_cmps) || config->max_speed_cmps <= 0.0f) return 0U;
    if (!menu_flash_float_valid(config->max_omega_radps) || config->max_omega_radps <= 0.0f) return 0U;
    if (!menu_flash_float_valid(config->max_accel_cmps2) || config->max_accel_cmps2 <= 0.0f) return 0U;
    if (!menu_flash_float_valid(config->max_alpha_radps2) || config->max_alpha_radps2 <= 0.0f) return 0U;
    if (!menu_flash_float_valid(config->xy_tolerance_cm) || config->xy_tolerance_cm <= 0.0f) return 0U;
    if (!menu_flash_float_valid(config->yaw_tolerance_deg) || config->yaw_tolerance_deg <= 0.0f) return 0U;
    if (!menu_flash_float_valid(config->brake_limit) ||
        config->brake_limit < 0.05f || config->brake_limit > 2.0f) return 0U;
    if (!menu_flash_float_valid(config->brake_ceiling_cmps2) ||
        config->brake_ceiling_cmps2 < 50.0f || config->brake_ceiling_cmps2 > 5000.0f) return 0U;
    if (!menu_flash_float_valid(config->short_segment_cm) ||
        config->short_segment_cm < 1.0f || config->short_segment_cm > 200.0f) return 0U;
    if (!menu_flash_float_valid(config->short_boost_gain) ||
        config->short_boost_gain < 1.0f || config->short_boost_gain > 3.0f) return 0U;
    if (!menu_flash_float_valid(config->approach_zone_cm) ||
        config->approach_zone_cm < 0.0f || config->approach_zone_cm > 50.0f) return 0U;
    if (!menu_flash_float_valid(config->approach_ratio) ||
        config->approach_ratio < 0.0f || config->approach_ratio > 1.0f) return 0U;
    if (!menu_flash_float_valid(config->approach_acc_k) ||
        config->approach_acc_k < 0.05f || config->approach_acc_k > 0.95f) return 0U;
    if (!menu_flash_float_valid(config->yaw_lin_band_rad) ||
        config->yaw_lin_band_rad < 0.02f || config->yaw_lin_band_rad > 1.0f) return 0U;
    if (!menu_flash_float_valid(config->yaw_kd) ||
        config->yaw_kd < 0.0f || config->yaw_kd > 2.0f) return 0U;
    if (!menu_flash_float_valid(config->yaw_kd_translate) ||
        config->yaw_kd_translate < 0.0f || config->yaw_kd_translate > 2.0f) return 0U;
    if (!menu_flash_float_valid(config->mount_deg) ||
        config->mount_deg < -180.0f || config->mount_deg > 180.0f) return 0U;
    if (!menu_flash_float_valid(config->scale_x) ||
        config->scale_x < 0.1f || config->scale_x > 5.0f) return 0U;
    if (!menu_flash_float_valid(config->scale_y) ||
        config->scale_y < 0.1f || config->scale_y > 5.0f) return 0U;
    if (config->route_node_count < 1U || config->route_node_count > ROUTE_MAX_NODES) return 0U;
    for (wheel = 0U; wheel < ROUTE_MAX_NODES; wheel++)
    {
        const route_node_t *node = &config->route_nodes[wheel];
        if (!menu_flash_float_valid(node->x_cm) || node->x_cm < -10000.0f || node->x_cm > 10000.0f) return 0U;
        if (!menu_flash_float_valid(node->y_cm) || node->y_cm < -10000.0f || node->y_cm > 10000.0f) return 0U;
        if (!menu_flash_float_valid(node->yaw_deg) || node->yaw_deg < -180.0f || node->yaw_deg > 180.0f) return 0U;
    }
    return 1U;
}

uint8 Data_save_to_flash(const menu_flash_config_t *config)
{
    uint32 flags = 0U;
    uint8 wheel;

    if (config == NULL || !menu_flash_config_valid(config))
    {
        return 0U;
    }

    flash_buffer_clear();
    flash_union_buffer[MENU_FLASH_WORD_MAGIC].uint32_type = MENU_FLASH_MAGIC;
    flash_union_buffer[MENU_FLASH_WORD_VERSION].uint32_type = MENU_FLASH_VERSION;
    for (wheel = 0U; wheel < MOTOR_WHEEL_COUNT; wheel++)
    {
        flash_union_buffer[MENU_FLASH_WORD_KP_FIRST + wheel].uint32_type = menu_flash_float_to_word(config->wheel_kp[wheel]);
        flash_union_buffer[MENU_FLASH_WORD_KI_FIRST + wheel].uint32_type = menu_flash_float_to_word(config->wheel_ki[wheel]);
        flash_union_buffer[MENU_FLASH_WORD_KD_FIRST + wheel].uint32_type = menu_flash_float_to_word(config->wheel_kd[wheel]);
    }
    flash_union_buffer[MENU_FLASH_WORD_XY_KP].uint32_type = menu_flash_float_to_word(config->xy_kp);
    flash_union_buffer[MENU_FLASH_WORD_XY_KD].uint32_type = menu_flash_float_to_word(config->xy_kd);
    flash_union_buffer[MENU_FLASH_WORD_YAW_KP].uint32_type = menu_flash_float_to_word(config->yaw_kp);
    flash_union_buffer[MENU_FLASH_WORD_MAX_SPEED].uint32_type = menu_flash_float_to_word(config->max_speed_cmps);
    flash_union_buffer[MENU_FLASH_WORD_MAX_OMEGA].uint32_type = menu_flash_float_to_word(config->max_omega_radps);
    flash_union_buffer[MENU_FLASH_WORD_MAX_ACCEL].uint32_type = menu_flash_float_to_word(config->max_accel_cmps2);
    flash_union_buffer[MENU_FLASH_WORD_MAX_ALPHA].uint32_type = menu_flash_float_to_word(config->max_alpha_radps2);
    flash_union_buffer[MENU_FLASH_WORD_TOL_XY].uint32_type = menu_flash_float_to_word(config->xy_tolerance_cm);
    flash_union_buffer[MENU_FLASH_WORD_TOL_YAW].uint32_type = menu_flash_float_to_word(config->yaw_tolerance_deg);
    flash_union_buffer[MENU_FLASH_WORD_MOUNT_DEG].uint32_type = menu_flash_float_to_word(config->mount_deg);
    flash_union_buffer[MENU_FLASH_WORD_SCALE_X].uint32_type = menu_flash_float_to_word(config->scale_x);
    flash_union_buffer[MENU_FLASH_WORD_SCALE_Y].uint32_type = menu_flash_float_to_word(config->scale_y);
    if (config->yaw_reversed) flags |= MENU_FLAG_YAW_REVERSED;
    flash_union_buffer[MENU_FLASH_WORD_FLAGS].uint32_type = flags;
    flash_union_buffer[MENU_FLASH_WORD_ROUTE_COUNT].uint32_type = config->route_node_count;
    for (wheel = 0U; wheel < ROUTE_MAX_NODES; wheel++)
    {
        uint32 first = MENU_FLASH_WORD_ROUTE_FIRST + 3U * wheel;
        flash_union_buffer[first].uint32_type = menu_flash_float_to_word(config->route_nodes[wheel].x_cm);
        flash_union_buffer[first + 1U].uint32_type = menu_flash_float_to_word(config->route_nodes[wheel].y_cm);
        flash_union_buffer[first + 2U].uint32_type = menu_flash_float_to_word(config->route_nodes[wheel].yaw_deg);
    }
    flash_union_buffer[MENU_FLASH_WORD_BRAKE_LIMIT].uint32_type = menu_flash_float_to_word(config->brake_limit);
    flash_union_buffer[MENU_FLASH_WORD_BRAKE_CEILING].uint32_type = menu_flash_float_to_word(config->brake_ceiling_cmps2);
    flash_union_buffer[MENU_FLASH_WORD_SHORT_SEGMENT].uint32_type = menu_flash_float_to_word(config->short_segment_cm);
    flash_union_buffer[MENU_FLASH_WORD_SHORT_BOOST].uint32_type = menu_flash_float_to_word(config->short_boost_gain);
    flash_union_buffer[MENU_FLASH_WORD_APP_ZONE].uint32_type = menu_flash_float_to_word(config->approach_zone_cm);
    flash_union_buffer[MENU_FLASH_WORD_APP_RATIO].uint32_type = menu_flash_float_to_word(config->approach_ratio);
    flash_union_buffer[MENU_FLASH_WORD_APP_ACC_K].uint32_type = menu_flash_float_to_word(config->approach_acc_k);
    flash_union_buffer[MENU_FLASH_WORD_YAW_BAND].uint32_type = menu_flash_float_to_word(config->yaw_lin_band_rad);
    flash_union_buffer[MENU_FLASH_WORD_YAW_KD].uint32_type = menu_flash_float_to_word(config->yaw_kd);
    flash_union_buffer[MENU_FLASH_WORD_YAW_KD_TRN].uint32_type = menu_flash_float_to_word(config->yaw_kd_translate);
    flash_union_buffer[MENU_FLASH_WORD_CHECKSUM].uint32_type = menu_flash_checksum();

    if (flash_check(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX) &&
        flash_erase_page(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX) != 0U)
    {
        return 0U;
    }
    /* 只写入实际使用的字，缩短关中断时间（整页写约 16ms，这里约 1ms）。
     * 擦除仍是最大耗时项，保存后需调用 imu_recover_after_stall() 恢复 IMU。 */
    if (flash_write_page(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX,
                         (const uint32 *)&flash_union_buffer[0], MENU_FLASH_WORD_COUNT) != 0U)
    {
        return 0U;
    }

    flash_read_page_to_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);
    return menu_flash_buffer_valid();
}

uint8 Data_load_from_flash(menu_flash_config_t *config)
{
    menu_flash_config_t loaded;
    static const route_node_t defaults[ROUTE_MAX_NODES] = ROUTE_DEFAULT_NODES;
    uint32 flags;
    uint8 wheel;

    if (config == NULL || !flash_check(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX))
    {
        return 0U;
    }

    flash_read_page_to_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);
    if (!menu_flash_buffer_valid())
    {
        return 0U;
    }

    for (wheel = 0U; wheel < MOTOR_WHEEL_COUNT; wheel++)
    {
        loaded.wheel_kp[wheel] = menu_flash_word_to_float(flash_union_buffer[MENU_FLASH_WORD_KP_FIRST + wheel].uint32_type);
        loaded.wheel_ki[wheel] = menu_flash_word_to_float(flash_union_buffer[MENU_FLASH_WORD_KI_FIRST + wheel].uint32_type);
        loaded.wheel_kd[wheel] = menu_flash_word_to_float(flash_union_buffer[MENU_FLASH_WORD_KD_FIRST + wheel].uint32_type);
    }
    loaded.xy_kp = menu_flash_word_to_float(flash_union_buffer[MENU_FLASH_WORD_XY_KP].uint32_type);
    loaded.xy_kd = menu_flash_word_to_float(flash_union_buffer[MENU_FLASH_WORD_XY_KD].uint32_type);
    loaded.yaw_kp = menu_flash_word_to_float(flash_union_buffer[MENU_FLASH_WORD_YAW_KP].uint32_type);
    loaded.max_speed_cmps = menu_flash_word_to_float(flash_union_buffer[MENU_FLASH_WORD_MAX_SPEED].uint32_type);
    loaded.max_omega_radps = menu_flash_word_to_float(flash_union_buffer[MENU_FLASH_WORD_MAX_OMEGA].uint32_type);
    loaded.max_accel_cmps2 = menu_flash_word_to_float(flash_union_buffer[MENU_FLASH_WORD_MAX_ACCEL].uint32_type);
    loaded.max_alpha_radps2 = menu_flash_word_to_float(flash_union_buffer[MENU_FLASH_WORD_MAX_ALPHA].uint32_type);
    loaded.xy_tolerance_cm = menu_flash_word_to_float(flash_union_buffer[MENU_FLASH_WORD_TOL_XY].uint32_type);
    loaded.yaw_tolerance_deg = menu_flash_word_to_float(flash_union_buffer[MENU_FLASH_WORD_TOL_YAW].uint32_type);
    loaded.mount_deg = menu_flash_word_to_float(flash_union_buffer[MENU_FLASH_WORD_MOUNT_DEG].uint32_type);
    loaded.scale_x = menu_flash_word_to_float(flash_union_buffer[MENU_FLASH_WORD_SCALE_X].uint32_type);
    loaded.scale_y = menu_flash_word_to_float(flash_union_buffer[MENU_FLASH_WORD_SCALE_Y].uint32_type);
    flags = flash_union_buffer[MENU_FLASH_WORD_FLAGS].uint32_type;
    loaded.yaw_reversed = (flags & MENU_FLAG_YAW_REVERSED) ? 1U : 0U;

    if (flash_union_buffer[MENU_FLASH_WORD_VERSION].uint32_type == MENU_FLASH_LEGACY_VERSION)
    {
        loaded.route_node_count = ROUTE_DEFAULT_NODE_COUNT;
        memcpy(loaded.route_nodes, defaults, sizeof(defaults));
    }
    else
    {
        uint32 count = flash_union_buffer[MENU_FLASH_WORD_ROUTE_COUNT].uint32_type;
        if (count < 1U || count > ROUTE_MAX_NODES) return 0U;
        loaded.route_node_count = (uint8)count;
        for (wheel = 0U; wheel < ROUTE_MAX_NODES; wheel++)
        {
            uint32 first = MENU_FLASH_WORD_ROUTE_FIRST + 3U * wheel;
            loaded.route_nodes[wheel].x_cm = menu_flash_word_to_float(flash_union_buffer[first].uint32_type);
            loaded.route_nodes[wheel].y_cm = menu_flash_word_to_float(flash_union_buffer[first + 1U].uint32_type);
            loaded.route_nodes[wheel].yaw_deg = menu_flash_word_to_float(flash_union_buffer[first + 2U].uint32_type);
        }
    }
    if (flash_union_buffer[MENU_FLASH_WORD_VERSION].uint32_type == MENU_FLASH_VERSION)
    {
        loaded.brake_limit = menu_flash_word_to_float(flash_union_buffer[MENU_FLASH_WORD_BRAKE_LIMIT].uint32_type);
        loaded.brake_ceiling_cmps2 = menu_flash_word_to_float(flash_union_buffer[MENU_FLASH_WORD_BRAKE_CEILING].uint32_type);
        loaded.short_segment_cm = menu_flash_word_to_float(flash_union_buffer[MENU_FLASH_WORD_SHORT_SEGMENT].uint32_type);
        loaded.short_boost_gain = menu_flash_word_to_float(flash_union_buffer[MENU_FLASH_WORD_SHORT_BOOST].uint32_type);
        loaded.approach_zone_cm = menu_flash_word_to_float(flash_union_buffer[MENU_FLASH_WORD_APP_ZONE].uint32_type);
        loaded.approach_ratio = menu_flash_word_to_float(flash_union_buffer[MENU_FLASH_WORD_APP_RATIO].uint32_type);
        loaded.approach_acc_k = menu_flash_word_to_float(flash_union_buffer[MENU_FLASH_WORD_APP_ACC_K].uint32_type);
        loaded.yaw_lin_band_rad = menu_flash_word_to_float(flash_union_buffer[MENU_FLASH_WORD_YAW_BAND].uint32_type);
        loaded.yaw_kd = menu_flash_word_to_float(flash_union_buffer[MENU_FLASH_WORD_YAW_KD].uint32_type);
        loaded.yaw_kd_translate = menu_flash_word_to_float(flash_union_buffer[MENU_FLASH_WORD_YAW_KD_TRN].uint32_type);
    }
    else
    {
        /* v1/v2 旧存档：规划参数取编译期默认值 */
        static const position_config_t pos_defaults = POSITION_CONFIG_DEFAULT;
        loaded.brake_limit = pos_defaults.brake_limit;
        loaded.brake_ceiling_cmps2 = pos_defaults.brake_ceiling_cmps2;
        loaded.short_segment_cm = pos_defaults.short_segment_cm;
        loaded.short_boost_gain = pos_defaults.short_boost_gain;
        loaded.approach_zone_cm = pos_defaults.approach_zone_cm;
        loaded.approach_ratio = pos_defaults.approach_ratio;
        loaded.approach_acc_k = pos_defaults.approach_acc_k;
        loaded.yaw_lin_band_rad = pos_defaults.yaw_lin_band_rad;
        loaded.yaw_kd = pos_defaults.yaw_kd;
        loaded.yaw_kd_translate = pos_defaults.yaw_kd_translate;
    }
    if (!menu_flash_config_valid(&loaded)) return 0U;
    *config = loaded;
    return 1U;
}

uint8 Data_clear_flash(void)
{
    flash_buffer_clear();
    return (flash_erase_page(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX) == 0U) ? 1U : 0U;
}

uint8 menu_flash_save_current(void)
{
    menu_flash_config_t config;
    uint8 wheel;
    uint32 primask;
    int16 count;
    static tagPID_T *const wheel_pid[MOTOR_WHEEL_COUNT] = {&ULpid, &URpid, &DLpid, &DRpid};

    primask = interrupt_global_disable();
    route_follow_stop();
    for (wheel = 0U; wheel < MOTOR_WHEEL_COUNT; wheel++)
    {
        config.wheel_kp[wheel] = wheel_pid[wheel]->fKp;
        config.wheel_ki[wheel] = wheel_pid[wheel]->fKi;
        config.wheel_kd[wheel] = wheel_pid[wheel]->fKd;
    }
    config.xy_kp = motor_position_config.xy_kp;
    config.xy_kd = motor_position_config.xy_kd;
    config.yaw_kp = motor_position_config.yaw_kp;
    config.max_speed_cmps = motor_position_config.max_speed_cmps;
    config.max_omega_radps = motor_position_config.max_omega_radps;
    config.max_accel_cmps2 = motor_position_config.max_accel_cmps2;
    config.max_alpha_radps2 = motor_position_config.max_alpha_radps2;
    config.xy_tolerance_cm = motor_position_config.xy_tolerance_cm;
    config.yaw_tolerance_deg = motor_position_config.yaw_tolerance_deg;
    config.brake_limit = motor_position_config.brake_limit;
    config.brake_ceiling_cmps2 = motor_position_config.brake_ceiling_cmps2;
    config.short_segment_cm = motor_position_config.short_segment_cm;
    config.short_boost_gain = motor_position_config.short_boost_gain;
    config.approach_zone_cm = motor_position_config.approach_zone_cm;
    config.approach_ratio = motor_position_config.approach_ratio;
    config.approach_acc_k = motor_position_config.approach_acc_k;
    config.yaw_lin_band_rad = motor_position_config.yaw_lin_band_rad;
    config.yaw_kd = motor_position_config.yaw_kd;
    config.yaw_kd_translate = motor_position_config.yaw_kd_translate;
    config.mount_deg = navigation_mount_deg;
    config.scale_x = navigation_scale_x;
    config.scale_y = navigation_scale_y;
    config.yaw_reversed = navigation_yaw_reversed ? 1U : 0U;

    count = route_node_count;
    memcpy(config.route_nodes, route_nodes, sizeof(config.route_nodes));
    interrupt_global_enable(primask);
    if (count < 1 || count > ROUTE_MAX_NODES) return 0U;
    config.route_node_count = (uint8)count;
    uint8 result = Data_save_to_flash(&config);
    /* 擦写期间全局关中断数十毫秒以上，IMU 采样间隔超限会锁止到 BAD_DT(-3)，
     * 这里恢复采样时间基准；标定和姿态保留，导航不需要重新 Zero。 */
    imu_recover_after_stall();
    return result;
}

uint8 menu_flash_load_current(void)
{
    menu_flash_config_t config;
    uint8 wheel;
    uint32 primask;
    static tagPID_T *const wheel_pid[MOTOR_WHEEL_COUNT] = {&ULpid, &URpid, &DLpid, &DRpid};

    if (!Data_load_from_flash(&config))
    {
        return 0U;
    }

    primask = interrupt_global_disable();
    route_follow_stop();
    for (wheel = 0U; wheel < MOTOR_WHEEL_COUNT; wheel++)
    {
        wheel_pid[wheel]->fKp = config.wheel_kp[wheel];
        wheel_pid[wheel]->fKi = config.wheel_ki[wheel];
        wheel_pid[wheel]->fKd = config.wheel_kd[wheel];
    }
    motor_position_config.xy_kp = config.xy_kp;
    motor_position_config.xy_kd = config.xy_kd;
    motor_position_config.yaw_kp = config.yaw_kp;
    motor_position_config.max_speed_cmps = config.max_speed_cmps;
    motor_position_config.max_omega_radps = config.max_omega_radps;
    motor_position_config.max_accel_cmps2 = config.max_accel_cmps2;
    motor_position_config.max_alpha_radps2 = config.max_alpha_radps2;
    motor_position_config.xy_tolerance_cm = config.xy_tolerance_cm;
    motor_position_config.yaw_tolerance_deg = config.yaw_tolerance_deg;
    motor_position_config.brake_limit = config.brake_limit;
    motor_position_config.brake_ceiling_cmps2 = config.brake_ceiling_cmps2;
    motor_position_config.short_segment_cm = config.short_segment_cm;
    motor_position_config.short_boost_gain = config.short_boost_gain;
    motor_position_config.approach_zone_cm = config.approach_zone_cm;
    motor_position_config.approach_ratio = config.approach_ratio;
    motor_position_config.approach_acc_k = config.approach_acc_k;
    motor_position_config.yaw_lin_band_rad = config.yaw_lin_band_rad;
    motor_position_config.yaw_kd = config.yaw_kd;
    motor_position_config.yaw_kd_translate = config.yaw_kd_translate;
    navigation_mount_deg = config.mount_deg;
    navigation_scale_x = config.scale_x;
    navigation_scale_y = config.scale_y;
    navigation_yaw_reversed = config.yaw_reversed ? true : false;
    /* 安装参数在 reset_state 时被快照进融合模块，加载后要重建一次定位起点 */
    route_node_count = config.route_node_count;
    memcpy(route_nodes, config.route_nodes, sizeof(route_nodes));
    route_current_idx = 0;
    app_navigation_request_reset();
    interrupt_global_enable(primask);
    return 1U;
}
