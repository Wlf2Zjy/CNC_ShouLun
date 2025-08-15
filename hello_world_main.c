#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/pcnt.h"
#include "driver/timer.h"
#include "esp_log.h"

// ==================== 硬件引脚定义 ====================
#define ENCODER_A GPIO_NUM_18 // 编码器 A 相信号输入
#define ENCODER_B GPIO_NUM_19 // 编码器 B 相信号输入

// 左拨档 4 档（低电平有效）
#define LEFT_SW1 GPIO_NUM_3
#define LEFT_SW2 GPIO_NUM_4
#define LEFT_SW3 GPIO_NUM_5
#define LEFT_SW4 GPIO_NUM_9

// 右拨档 3 档（低电平有效）
#define RIGHT_SW1 GPIO_NUM_0
#define RIGHT_SW2 GPIO_NUM_23
#define RIGHT_SW3 GPIO_NUM_20

// 定时器配置
#define TIMER_GROUP        TIMER_GROUP_0  // 使用定时器组 0
#define TIMER_IDX          TIMER_0        // 使用定时器 0
#define TIMER_INTERVAL_MS  50             // 定时器采样周期（毫秒）

// 日志 TAG
static const char *TAG = "ENCODER";

// PCNT（脉冲计数器）单元
static pcnt_unit_t pcnt_unit = PCNT_UNIT_0;

// 用队列传输定时器采样到的步数增量
static QueueHandle_t encoder_queue;

// ==================== 编码器初始化（PCNT 增量模式） ====================
static void encoder_init(void) {
    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = ENCODER_A,   // A 相输入
        .ctrl_gpio_num = ENCODER_B,    // B 相输入（方向控制）
        .lctrl_mode = PCNT_MODE_KEEP,    // B 低电平时方向不变
        .hctrl_mode = PCNT_MODE_REVERSE, // B 高电平时反向
        .pos_mode = PCNT_COUNT_INC,      // A 上升沿计数 +1
        .neg_mode = PCNT_COUNT_DEC,      // A 下降沿计数 -1
        .counter_h_lim = 32767,          // 计数上限
        .counter_l_lim = -32768,         // 计数下限
        .unit = pcnt_unit,               // 使用 PCNT 单元 0
        .channel = PCNT_CHANNEL_0        // 使用通道 0
    };

    pcnt_unit_config(&pcnt_config); // 初始化 PCNT 单元

    pcnt_set_filter_value(pcnt_unit, 1000); // 设置滤波时间 1000ns = 1us
    pcnt_filter_enable(pcnt_unit);          // 启用滤波器

    pcnt_counter_pause(pcnt_unit); // 暂停计数器
    pcnt_counter_clear(pcnt_unit); // 清零计数器
    pcnt_counter_resume(pcnt_unit);// 启动计数器
}

// ==================== 拨档初始化 ====================
static void switch_init(void) {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE, // 不使用中断
        .mode = GPIO_MODE_INPUT,        // 输入模式
        .pull_up_en = 1,                 // 开启上拉
        .pull_down_en = 0                // 关闭下拉
    };

    // 左拨档配置
    io_conf.pin_bit_mask = (1ULL << LEFT_SW1) | (1ULL << LEFT_SW2) |
                           (1ULL << LEFT_SW3) | (1ULL << LEFT_SW4);
    gpio_config(&io_conf);

    // 右拨档配置
    io_conf.pin_bit_mask = (1ULL << RIGHT_SW1) | (1ULL << RIGHT_SW2) |
                           (1ULL << RIGHT_SW3);
    gpio_config(&io_conf);
}

// ==================== 拨档读取（原始值） ====================
// 低电平表示按下/选中
static int read_left_switch_raw(void) {
    if (gpio_get_level(LEFT_SW1) == 0) return 1;
    if (gpio_get_level(LEFT_SW2) == 0) return 2;
    if (gpio_get_level(LEFT_SW3) == 0) return 3;
    if (gpio_get_level(LEFT_SW4) == 0) return 4;
    return 0; // 没有拨到任何档
}

static int read_right_switch_raw(void) {
    if (gpio_get_level(RIGHT_SW1) == 0) return 1;
    if (gpio_get_level(RIGHT_SW2) == 0) return 2;
    if (gpio_get_level(RIGHT_SW3) == 0) return 3;
    return 0; // 没有拨到任何档
}

// ==================== 软件防抖 ====================
// 通过多次采样判断开关状态是否稳定
static int read_switch_stable(int (*read_func)(void), int last_value) {
    int stable_value = last_value;  // 稳定值初始化为上一次值
    int count_same = 0;             // 连续相同次数计数
    const int samples = 10;         // 采样次数
    const int delay_ms = 10;        // 每次采样间隔

    for (int i = 0; i < samples; i++) {
        int val = read_func();
        if (val == stable_value) {
            count_same++;
        } else {
            stable_value = val;
            count_same = 1;
        }
        vTaskDelay(pdMS_TO_TICKS(delay_ms)); // 延时
    }
    return (count_same >= 3) ? stable_value : last_value; // 至少连续3次一致才更新
}

// ==================== 定时器中断回调（采集编码器增量） ====================
static bool IRAM_ATTR encoder_timer_isr_callback(void *args) {
    int16_t raw_count = 0;

    // 读取当前 PCNT 计数值
    pcnt_get_counter_value(pcnt_unit, &raw_count);

    // 清零计数器以便下一次采样获取增量
    pcnt_counter_clear(pcnt_unit);

    // 每两个脉冲算一步
    int steps = raw_count / 2;

    // 如果有步数变化，则通过队列传递给任务
    if (steps != 0) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(encoder_queue, &steps, &xHigherPriorityTaskWoken);
        return xHigherPriorityTaskWoken == pdTRUE;
    }
    return pdFALSE;
}

// ==================== 定时器初始化 ====================
static void encoder_timer_init(void) {
    timer_config_t config = {
        .divider = 80,              // 定时器计数分频（80MHz / 80 = 1MHz，即 1 tick = 1us）
        .counter_dir = TIMER_COUNT_UP, // 向上计数
        .counter_en = TIMER_PAUSE,     // 初始暂停
        .alarm_en = TIMER_ALARM_EN,    // 开启报警（中断）
        .auto_reload = true            // 自动重装载
    };

    timer_init(TIMER_GROUP, TIMER_IDX, &config); // 初始化定时器
    timer_set_counter_value(TIMER_GROUP, TIMER_IDX, 0); // 清零计数器
    timer_set_alarm_value(TIMER_GROUP, TIMER_IDX, TIMER_INTERVAL_MS * 1000); // 50ms 定时
    timer_enable_intr(TIMER_GROUP, TIMER_IDX); // 开启中断
    timer_isr_callback_add(TIMER_GROUP, TIMER_IDX, encoder_timer_isr_callback, NULL, 0); // 绑定回调
    timer_start(TIMER_GROUP, TIMER_IDX); // 启动定时器
}

// ==================== 编码器处理任务 ====================
// 负责接收定时器传来的增量并打印
static void encoder_task(void *arg) {
    int steps;
    while (1) {
        // 阻塞等待队列数据
        if (xQueueReceive(encoder_queue, &steps, portMAX_DELAY)) {
            if (steps > 0) {
                ESP_LOGI(TAG, "方向: 顺时针, 移动步数: %d", steps);
            } else {
                ESP_LOGI(TAG, "方向: 逆时针, 移动步数: %d", -steps);
            }
        }
    }
}

// ==================== 拨档处理任务 ====================
static void switch_task(void *arg) {
    int left_pos = 0, right_pos = 0;
    while (1) {
        int new_left = read_switch_stable(read_left_switch_raw, left_pos);
        int new_right = read_switch_stable(read_right_switch_raw, right_pos);

        // 如果有变化则打印
        if (new_left != left_pos || new_right != right_pos) {
            ESP_LOGI(TAG, "左拨档: %d 档, 右拨档: %d 档", new_left, new_right);
            left_pos = new_left;
            right_pos = new_right;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ==================== 主程序 ====================
void app_main(void) {
    ESP_LOGI(TAG, "初始化旋转编码器 + 拨档开关（定时器+队列版本）");

    // 创建队列（存放步数）
    encoder_queue = xQueueCreate(10, sizeof(int));

    // 初始化硬件
    encoder_init();
    switch_init();
    encoder_timer_init();

    // 创建任务
    xTaskCreate(encoder_task, "encoder_task", 2048, NULL, 5, NULL);
    xTaskCreate(switch_task, "switch_task", 2048, NULL, 5, NULL);
}
