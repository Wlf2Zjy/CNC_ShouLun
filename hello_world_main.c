#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/pcnt.h"
#include "esp_log.h"

// ==================== 硬件引脚定义 ====================
// 编码器 A/B 相输入
#define ENCODER_A GPIO_NUM_18
#define ENCODER_B GPIO_NUM_19

// 左拨档（选择当前控制的轴 X/Y/Z/A）
#define LEFT_SW1 GPIO_NUM_3
#define LEFT_SW2 GPIO_NUM_4
#define LEFT_SW3 GPIO_NUM_5
#define LEFT_SW4 GPIO_NUM_9

// 右拨档（倍率选择：0.1、1、5）
#define RIGHT_SW1 GPIO_NUM_0
#define RIGHT_SW2 GPIO_NUM_23
#define RIGHT_SW3 GPIO_NUM_20

static const char *TAG = "ENCODER"; // 用于 ESP_LOG 打印分类

static pcnt_unit_t pcnt_unit = PCNT_UNIT_0; // 使用 PCNT 单元 0

// ==================== 全局变量 ====================
// 四个轴的累计计数（单位：步）
static volatile float axis_counts[4] = {0, 0, 0, 0}; //暂未用到
// 当前选择的轴索引：0=X, 1=Y, 2=Z, 3=A
static volatile int current_axis = 0;
// 右拨档倍率（默认为 1 倍）
static volatile float right_multiplier = 1.0f;

// ==================== 编码器初始化 ====================
// 配置 ESP32 PCNT（脉冲计数器）来读取旋转编码器的脉冲
static void encoder_init(void) {
    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = ENCODER_A,       // A 相输入
        .ctrl_gpio_num = ENCODER_B,        // B 相作为方向控制
        .lctrl_mode = PCNT_MODE_KEEP,      // B=低时不反转计数方向
        .hctrl_mode = PCNT_MODE_REVERSE,   // B=高时反转计数方向
        .pos_mode = PCNT_COUNT_INC,        // A 相上升沿计数 +1
        .neg_mode = PCNT_COUNT_DEC,        // A 相下降沿计数 -1
        .counter_h_lim = 32767,            // 计数上限
        .counter_l_lim = -32768,           // 计数下限
        .unit = pcnt_unit,                 // 使用的 PCNT 单元
        .channel = PCNT_CHANNEL_0          // 使用的通道
    };

    // 应用配置
    pcnt_unit_config(&pcnt_config);

    // 设置数字滤波，过滤高频毛刺
    pcnt_set_filter_value(pcnt_unit, 1000);
    pcnt_filter_enable(pcnt_unit);

    // 清零并启动计数
    pcnt_counter_pause(pcnt_unit);
    pcnt_counter_clear(pcnt_unit);
    pcnt_counter_resume(pcnt_unit);
}

// ==================== 拨档初始化 ====================
// 配置所有拨档开关为输入 + 上拉
static void switch_init(void) {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE, // 不使用中断
        .mode = GPIO_MODE_INPUT,        // 输入模式
        .pull_up_en = 1,                 // 上拉电阻
        .pull_down_en = 0
    };

    // 左拨档（X/Y/Z/A 选择）
    io_conf.pin_bit_mask = (1ULL << LEFT_SW1) | (1ULL << LEFT_SW2) |
                           (1ULL << LEFT_SW3) | (1ULL << LEFT_SW4);
    gpio_config(&io_conf);

    // 右拨档（倍率选择）
    io_conf.pin_bit_mask = (1ULL << RIGHT_SW1) | (1ULL << RIGHT_SW2) |
                           (1ULL << RIGHT_SW3);
    gpio_config(&io_conf);
}

// ==================== 拨档读取函数 ====================
// 左拨档原始值（未防抖），返回 'X','Y','Z','A' 或 0
static char read_left_switch_raw(void) {
    if (gpio_get_level(LEFT_SW1) == 0) return 'X';
    if (gpio_get_level(LEFT_SW2) == 0) return 'Y';
    if (gpio_get_level(LEFT_SW3) == 0) return 'Z';
    if (gpio_get_level(LEFT_SW4) == 0) return 'A';
    return 0;
}

// 右拨档原始值（未防抖），返回倍率 0.1、1.0、5.0
static float read_right_switch_raw(void) {
    if (gpio_get_level(RIGHT_SW1) == 0) return 0.1f;
    if (gpio_get_level(RIGHT_SW2) == 0) return 1.0f;
    if (gpio_get_level(RIGHT_SW3) == 0) return 5.0f;
    return 0.0f;
}

// ==================== 防抖读取模板宏 ====================
#define SWITCH_SAMPLES 10     // 采样次数
#define SWITCH_DELAY_MS 10    // 每次采样间隔

// 生成稳定开关读取函数（char 型 / float 型都可用）
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

// 生成 char 和 float 类型的防抖读取函数
DEFINE_SWITCH_STABLE(char)
DEFINE_SWITCH_STABLE(float)

// ==================== 编码器轮询任务 ====================
// 每 50 ms 检查一次编码器，如果检测到停止，则输出一次
static void encoder_poll_task(void *arg) {
    int16_t raw_count = 0;
    bool moving = false; // 当前是否在运动

    while (1) {
        // 读取编码器计数值
        pcnt_get_counter_value(pcnt_unit, &raw_count);

        if (raw_count != 0) {
            // 有脉冲 → 清零计数并累计到当前轴
            pcnt_counter_clear(pcnt_unit);
            float scaled_steps = (raw_count / 2.0f) * right_multiplier; // 每两脉冲一个步
            axis_counts[current_axis] += scaled_steps;
            moving = true;
        } else {
            // 没有脉冲
            if (moving) {
                // 刚刚停止时 → 输出一次最终位置
                const char axis_names[4] = {'X', 'Y', 'Z', 'A'};
                ESP_LOGI(TAG, "轴: %c, 当前位置: %.2f",
                         axis_names[current_axis],
                         axis_counts[current_axis]);
                moving = false;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50)); // 每 50ms 轮询一次
    }
}

// ==================== 拨档检测任务 ====================
// 持续检测左右拨档变化，并更新当前控制轴和倍率
static void switch_task(void *arg) {
    char left_pos = 0;        // 当前左拨档位置
    float right_pos = 1.0f;   // 当前右拨档倍率

    while (1) {
        // 读取防抖后的拨档值
        char new_left = read_switch_stable_char(read_left_switch_raw, left_pos);
        float new_right = read_switch_stable_float(read_right_switch_raw, right_pos);

        // 如果有变化 → 更新状态
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
        vTaskDelay(pdMS_TO_TICKS(20)); // 每 20ms 检查一次
    }
}

// ==================== 主程序入口 ====================
void app_main(void) {
    ESP_LOGI(TAG, "初始化旋转编码器 + 拨档开关");

    // 初始化硬件
    encoder_init();
    switch_init();

    // 创建任务
    xTaskCreate(encoder_poll_task, "encoder_poll_task", 4096, NULL, 5, NULL);
    xTaskCreate(switch_task, "switch_task", 2048, NULL, 5, NULL);
}
