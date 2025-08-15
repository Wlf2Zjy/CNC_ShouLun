#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/pcnt.h"
#include "esp_log.h"

// ==================== 硬件引脚定义 ====================
#define ENCODER_A GPIO_NUM_18
#define ENCODER_B GPIO_NUM_19

#define LEFT_SW1 GPIO_NUM_3
#define LEFT_SW2 GPIO_NUM_4
#define LEFT_SW3 GPIO_NUM_5
#define LEFT_SW4 GPIO_NUM_9

#define RIGHT_SW1 GPIO_NUM_0
#define RIGHT_SW2 GPIO_NUM_23
#define RIGHT_SW3 GPIO_NUM_20

static const char *TAG = "ENCODER";
static pcnt_unit_t pcnt_unit = PCNT_UNIT_0;

// ==================== 四轴累计计数（单位：步） ====================
// 累计总位移
static volatile float axis_counts[4] = {0, 0, 0, 0};
// 上次输出时的累计值，用于计算本次运动的增量
static volatile float axis_last_report[4] = {0, 0, 0, 0};
// 当前选择的轴索引：0=X, 1=Y, 2=Z, 3=A
static volatile int current_axis = 0;
// 右拨档倍率
static volatile float right_multiplier = 1.0f;

// ==================== 编码器初始化 ====================
static void encoder_init(void) {
    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = ENCODER_A,
        .ctrl_gpio_num = ENCODER_B,
        .lctrl_mode = PCNT_MODE_KEEP,
        .hctrl_mode = PCNT_MODE_REVERSE,
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_DEC,
        .counter_h_lim = 32767,
        .counter_l_lim = -32768,
        .unit = pcnt_unit,
        .channel = PCNT_CHANNEL_0
    };

    pcnt_unit_config(&pcnt_config);
    pcnt_set_filter_value(pcnt_unit, 1000); // 硬件滤波，防止抖动
    pcnt_filter_enable(pcnt_unit);
    pcnt_counter_pause(pcnt_unit);
    pcnt_counter_clear(pcnt_unit);
    pcnt_counter_resume(pcnt_unit);
}

// ==================== 拨档初始化 ====================
static void switch_init(void) {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,
        .pull_down_en = 0
    };

    // 左拨档
    io_conf.pin_bit_mask = (1ULL << LEFT_SW1) | (1ULL << LEFT_SW2) |
                           (1ULL << LEFT_SW3) | (1ULL << LEFT_SW4);
    gpio_config(&io_conf);

    // 右拨档
    io_conf.pin_bit_mask = (1ULL << RIGHT_SW1) | (1ULL << RIGHT_SW2) |
                           (1ULL << RIGHT_SW3);
    gpio_config(&io_conf);
}

// ==================== 拨档读取（原始值） ====================
static char read_left_switch_raw(void) {
    if (gpio_get_level(LEFT_SW1) == 0) return 'X';
    if (gpio_get_level(LEFT_SW2) == 0) return 'Y';
    if (gpio_get_level(LEFT_SW3) == 0) return 'Z';
    if (gpio_get_level(LEFT_SW4) == 0) return 'A';
    return 0;
}

static float read_right_switch_raw(void) {
    if (gpio_get_level(RIGHT_SW1) == 0) return 0.1f;
    if (gpio_get_level(RIGHT_SW2) == 0) return 1.0f;
    if (gpio_get_level(RIGHT_SW3) == 0) return 5.0f;
    return 0.0f;
}

// ==================== 防抖函数宏 ====================
#define SWITCH_SAMPLES 10
#define SWITCH_DELAY_MS 10

#define DEFINE_SWITCH_STABLE(type) \
static type read_switch_stable_##type(type (*read_func)(void), type last_value) { \
    type stable_value = last_value; \
    int count_same = 0; \
    for (int i = 0; i < SWITCH_SAMPLES; i++) { \
        type val = read_func(); \
        if (val == stable_value) { \
            count_same++; \
        } else { \
            stable_value = val; \
            count_same = 1; \
        } \
        vTaskDelay(pdMS_TO_TICKS(SWITCH_DELAY_MS)); \
    } \
    return (count_same >= 3) ? stable_value : last_value; \
}

DEFINE_SWITCH_STABLE(char)
DEFINE_SWITCH_STABLE(float)

// ==================== 编码器轮询任务 ====================
// 每 50ms 检查一次，如果运动结束则输出相对于上次运动的增量
static void encoder_poll_task(void *arg) {
    int16_t raw_count = 0;
    bool moving = false; // 是否在运动中

    while (1) {
        pcnt_get_counter_value(pcnt_unit, &raw_count);

        if (raw_count != 0) {
            // 有脉冲 → 累加总位移
            pcnt_counter_clear(pcnt_unit);
            float scaled_steps = (raw_count / 2.0f) * right_multiplier;
            axis_counts[current_axis] += scaled_steps;
            moving = true;
        } else {
            // 没有脉冲
            if (moving) {
                // 刚停止 → 计算本次增量
                const char axis_names[4] = {'X', 'Y', 'Z', 'A'};
                float delta = axis_counts[current_axis] - axis_last_report[current_axis];

                ESP_LOGI(TAG, "轴: %c, 本次增量: %.2f",
                         axis_names[current_axis], delta);

                // 记录本次结束后的累计值，供下次增量计算
                axis_last_report[current_axis] = axis_counts[current_axis];

                moving = false;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50)); // 每 50ms 检查一次
    }
}

// ==================== 拨档任务（更新轴和倍率） ====================
static void switch_task(void *arg) {
    char left_pos = 0;
    float right_pos = 1.0f;
    while (1) {
        char new_left = read_switch_stable_char(read_left_switch_raw, left_pos);
        float new_right = read_switch_stable_float(read_right_switch_raw, right_pos);

        if (new_left != left_pos || new_right != right_pos) {
            if (new_left != 0) {
                left_pos = new_left;
                switch (new_left) {
                    case 'X': current_axis = 0; break;
                    case 'Y': current_axis = 1; break;
                    case 'Z': current_axis = 2; break;
                    case 'A': current_axis = 3; break;
                }
                ESP_LOGI(TAG, "切换到轴: %c", new_left);
            }
            if (new_right != 0.0f) {
                right_pos = new_right;
                right_multiplier = right_pos;
                ESP_LOGI(TAG, "倍率调整: ×%.1f", right_pos);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ==================== 主程序入口 ====================
void app_main(void) {
    ESP_LOGI(TAG, "初始化旋转编码器 + 拨档开关");

    encoder_init();
    switch_init();

    xTaskCreate(encoder_poll_task, "encoder_poll_task", 4096, NULL, 5, NULL);
    xTaskCreate(switch_task, "switch_task", 2048, NULL, 5, NULL);
}
