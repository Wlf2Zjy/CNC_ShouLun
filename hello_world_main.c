/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"

#include <stdio.h>
#include "driver/gpio.h"
#include "esp_log.h"

#define ENCODER_A GPIO_NUM_18
#define ENCODER_B GPIO_NUM_19

static const char *TAG = "ENCODER";

volatile long encoder_count = 0;
volatile int last_encoded = 0;

static void IRAM_ATTR encoder_isr_handler(void* arg) {
    static int lastA = 0;
    int A = gpio_get_level(ENCODER_A);
    int B = gpio_get_level(ENCODER_B);

    // 只在 A 相从低变高时判断方向
    if (A != lastA && A == 1) {
        if (B == 0) {
            encoder_count++; // 顺时针
        } else {
            encoder_count--; // 逆时针
        }
    }
    lastA = A;
}


void app_main(void) {
    ESP_LOGI(TAG, "启动旋转编码器测试");

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << ENCODER_A) | (1ULL << ENCODER_B),
        .pull_down_en = 0,
        .pull_up_en = 1
    };
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(ENCODER_A, encoder_isr_handler, NULL);
    gpio_isr_handler_add(ENCODER_B, encoder_isr_handler, NULL);

    long last_count = 0;

    while (1) {
        if (encoder_count != last_count) {
            long delta = encoder_count - last_count;
            if (delta > 0) {
                ESP_LOGI(TAG, "方向: 顺时针, 步数: %ld, 总计: %ld", delta, encoder_count);
            } else {
                ESP_LOGI(TAG, "方向: 逆时针, 步数: %ld, 总计: %ld", -delta, encoder_count);
            }
            last_count = encoder_count;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}


