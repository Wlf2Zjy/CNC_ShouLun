#include <stdio.h>
#include "freertos/FreeRTOS.h"   // FreeRTOS实时操作系统核心库
#include "freertos/task.h"
#include "driver/gpio.h"         // ESP32 GPIO驱动程序
#include "driver/pcnt.h"        // ESP32脉冲计数器驱动程序
#include "esp_log.h"
#include <math.h>

// ==================== 硬件引脚定义 ====================
#define ENCODER_A GPIO_NUM_18  // 编码器A相引脚
#define ENCODER_B GPIO_NUM_19  // 编码器B相引脚

// 左拨档开关引脚（选择轴）
#define LEFT_SW1 GPIO_NUM_3    // X轴选择开关
#define LEFT_SW2 GPIO_NUM_4    // Y轴选择开关
#define LEFT_SW3 GPIO_NUM_5    // Z轴选择开关
#define LEFT_SW4 GPIO_NUM_9    // A轴选择开关

// 右拨档开关引脚（选择倍率）
#define RIGHT_SW1 GPIO_NUM_0   // ×0.1倍率选择开关
#define RIGHT_SW2 GPIO_NUM_23  // ×1.0倍率选择开关
#define RIGHT_SW3 GPIO_NUM_20  // ×5.0倍率选择开关

static const char *TAG = "ENCODER";  // 日志标签
static pcnt_unit_t pcnt_unit = PCNT_UNIT_0;  // 使用的脉冲计数器单元

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
.pulse_gpio_num = ENCODER_A,  // 脉冲输入引脚
        .ctrl_gpio_num = ENCODER_B,   // 方向控制引脚
        .lctrl_mode = PCNT_MODE_KEEP, // 控制线低电平时计数模式保持不变
        .hctrl_mode = PCNT_MODE_REVERSE, // 控制线高电平时反转计数方向
        .pos_mode = PCNT_COUNT_INC,   // 正向计数模式（递增）
        .neg_mode = PCNT_COUNT_DEC,   // 反向计数模式（递减）
        .counter_h_lim = 32767,       // 计数器上限（16位有符号整数）
        .counter_l_lim = -32768,      // 计数器下限（16位有符号整数）
        .unit = pcnt_unit,            // 使用的计数器单元
        .channel = PCNT_CHANNEL_0      // 使用的计数器通道
    };

    pcnt_unit_config(&pcnt_config);   //初始化计数器
    pcnt_set_filter_value(pcnt_unit, 1000); // 硬件滤波，防止抖动
    pcnt_filter_enable(pcnt_unit);
    pcnt_counter_pause(pcnt_unit);
    pcnt_counter_clear(pcnt_unit);
    pcnt_counter_resume(pcnt_unit);
}

// ==================== 拨档初始化 ====================
static void switch_init(void) {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,  // 禁用中断
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,   // 启用上拉
        .pull_down_en = 0  // 禁用下拉
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
    return 0;  //无开关按下
}

static float read_right_switch_raw(void) {
    if (gpio_get_level(RIGHT_SW1) == 0) return 0.1f;
    if (gpio_get_level(RIGHT_SW2) == 0) return 1.0f;
    if (gpio_get_level(RIGHT_SW3) == 0) return 5.0f;
    return 0.0f;
}

// ==================== 防抖函数宏 ====================
#define SWITCH_SAMPLES 5     // 采样次数
#define SWITCH_DELAY_MS 10   // 每次采样间隔（毫秒）

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
    const char axis_names[4] = {'X', 'Y', 'Z', 'A'};
    int inactive_counter = 0;
    const int INACTIVE_THRESHOLD = 2; // 100ms无活动视为停止

    while (1) {
        pcnt_get_counter_value(pcnt_unit, &raw_count);
        pcnt_counter_clear(pcnt_unit); // 清零计数器

        if (raw_count != 0) {
            // 除以2.0f是因为每个完整周期有2个脉冲（A相和B相）
            float scaled_steps = (raw_count / 2.0f) * right_multiplier;
            axis_counts[current_axis] += scaled_steps;
            
            // 实时输出增量
           // ESP_LOGI(TAG, "轴: %c, 倍率: ×%.1f, 实时增量: %.2f", 
                     //axis_names[current_axis], right_multiplier, scaled_steps);
            
            inactive_counter = 0; // 重置不活动计数器
        } else {
            if (inactive_counter < INACTIVE_THRESHOLD) {
                inactive_counter++;
            } else if (inactive_counter == INACTIVE_THRESHOLD) {
                // 运动结束处理
                float total_delta = axis_counts[current_axis] - axis_last_report[current_axis];
                if (fabsf(total_delta) > 0.001f) {
                    ESP_LOGI(TAG, "[运动结束] 轴: %c,倍率: ×%.1f, 总位移: %.2f", 
                             axis_names[current_axis], right_multiplier, total_delta);
                    axis_last_report[current_axis] = axis_counts[current_axis];
                }
                inactive_counter++; // 防止重复输出
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));   // 每50ms轮询一次
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

    // 编码器轮询任务（优先级5，栈大小4096字节）
    xTaskCreate(encoder_poll_task, "encoder_poll_task", 4096, NULL, 5, NULL);
    // 拨档任务（优先级5，栈大小2048字节）
    xTaskCreate(switch_task, "switch_task", 2048, NULL, 5, NULL);
}
