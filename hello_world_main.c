#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/pcnt.h"
#include "esp_log.h"

#define ENCODER_A GPIO_NUM_18
#define ENCODER_B GPIO_NUM_19

static const char *TAG = "ENCODER";
static pcnt_unit_t pcnt_unit = PCNT_UNIT_0;

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

void app_main(void) {
    ESP_LOGI(TAG, "初始化旋转编码器");
    encoder_init();

    int16_t count = 0;
    int16_t last_count = 0;

    while (1) {
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
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
