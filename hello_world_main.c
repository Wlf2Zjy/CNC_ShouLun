#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/pcnt.h"
#include "esp_log.h"

#define ENCODER_A GPIO_NUM_18
#define ENCODER_B GPIO_NUM_19

// 左拨档 4 档
#define LEFT_SW1 GPIO_NUM_3
#define LEFT_SW2 GPIO_NUM_4
#define LEFT_SW3 GPIO_NUM_5
#define LEFT_SW4 GPIO_NUM_9

// 右拨档 3 档
#define RIGHT_SW1 GPIO_NUM_0
#define RIGHT_SW2 GPIO_NUM_23
#define RIGHT_SW3 GPIO_NUM_20

static const char *TAG = "ENCODER";
static pcnt_unit_t pcnt_unit = PCNT_UNIT_0;

// ================= 编码器初始化 =================
static void encoder_init(void) {
    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = ENCODER_A,   // A 相
        .ctrl_gpio_num = ENCODER_B,    // B 相
        .lctrl_mode = PCNT_MODE_KEEP,    // B 相低电平时保持方向
        .hctrl_mode = PCNT_MODE_REVERSE, // B 相高电平时反向
        .pos_mode = PCNT_COUNT_INC,      // 上升沿 +1
        .neg_mode = PCNT_COUNT_DEC,      // 下降沿 -1
        .counter_h_lim = 32767,
        .counter_l_lim = -32768,
        .unit = pcnt_unit,
        .channel = PCNT_CHANNEL_0
    };

    pcnt_unit_config(&pcnt_config);

    // 启用硬件滤波器（1us）
    pcnt_set_filter_value(pcnt_unit, 1000);
    pcnt_filter_enable(pcnt_unit);

    // 清零并启动
    pcnt_counter_pause(pcnt_unit);
    pcnt_counter_clear(pcnt_unit);
    pcnt_counter_resume(pcnt_unit);
}

//  拨档初始化 
static void switch_init(void) {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,
        .pull_down_en = 0
    };

    // 左拨档
    io_conf.pin_bit_mask = (1ULL << LEFT_SW1) | (1ULL << LEFT_SW2) | (1ULL << LEFT_SW3) | (1ULL << LEFT_SW4);
    gpio_config(&io_conf);

    // 右拨档
    io_conf.pin_bit_mask = (1ULL << RIGHT_SW1) | (1ULL << RIGHT_SW2) | (1ULL << RIGHT_SW3);
    gpio_config(&io_conf);
}

// 读取左拨档
static int read_left_switch_raw(void) {
    if (gpio_get_level(LEFT_SW1) == 0) return 1;
    if (gpio_get_level(LEFT_SW2) == 0) return 2;
    if (gpio_get_level(LEFT_SW3) == 0) return 3;
    if (gpio_get_level(LEFT_SW4) == 0) return 4;
    return 0; // 未选中
}

// 读取右拨档
static int read_right_switch_raw(void) {
    if (gpio_get_level(RIGHT_SW1) == 0) return 1;
    if (gpio_get_level(RIGHT_SW2) == 0) return 2;
    if (gpio_get_level(RIGHT_SW3) == 0) return 3;
    return 0; // 未选中
}

// 软件防抖，确保连续读取多次一致
static int read_switch_stable(int (*read_func)(void), int last_value) {
    int stable_value = last_value;
    int count_same = 0;
    int samples = 10;  // 采样次数
    int delay_ms = 10; // 每次间隔

    for (int i = 0; i < samples; i++) {
        int val = read_func();
        if (val == stable_value) {
            count_same++;
        } else {
            stable_value = val;
            count_same = 1;
        }
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }

    // 至少连续 3 次一致才更新
    return (count_same >= 3) ? stable_value : last_value;
}

// ================= 主程序 =================
void app_main(void) {
    ESP_LOGI(TAG, "初始化旋转编码器 + 拨档开关");
    encoder_init();
    switch_init();

    int16_t count = 0;
    int16_t last_count = 0;
    int left_pos = 0, right_pos = 0;

    while (1) {
        // ======== 编码器读取 ========
        pcnt_get_counter_value(pcnt_unit, &count);
        count /= 2; // 每两个脉冲算一步
        if (count != last_count) {
            int delta = count - last_count;
            if (delta > 0) {
                ESP_LOGI(TAG, "方向: 顺时针, 步数: %d, 总计: %d", delta, count);
            } else {
                ESP_LOGI(TAG, "方向: 逆时针, 步数: %d, 总计: %d", -delta, count);
            }
            last_count = count;
        }

        // ======== 拨档读取 ========

        int new_left = read_switch_stable(read_left_switch_raw, left_pos);
        int new_right = read_switch_stable(read_right_switch_raw, right_pos);

        if (new_left != left_pos || new_right != right_pos) {
            ESP_LOGI(TAG, "左拨档: %d 档, 右拨档: %d 档", new_left, new_right);
            left_pos = new_left;
            right_pos = new_right;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

