#include <stdio.h>
#include "freertos/FreeRTOS.h"   // FreeRTOS实时操作系统核心库
#include "freertos/task.h"
#include "driver/gpio.h"         // ESP32 GPIO驱动程序
#include "driver/pcnt.h"        // ESP32脉冲计数器驱动程序
#include "esp_log.h"
#include <math.h>
#include "driver/uart.h"         // UART 驱动
#include <string.h>


// ==================== 硬件引脚定义 ====================
#define ENCODER_A GPIO_NUM_1  // 编码器A相引脚
#define ENCODER_B GPIO_NUM_0  // 编码器B相引脚

/*A 相和 B 相是旋转编码器输出的两个脉冲信号：

A 相（CLK）：输出脉冲用来计数旋转的步数，每转动一下就会跳变一次。

B 相（DT）：用来判断旋转方向。它和 A 相的跳变先后顺序不同，表示顺时针或逆时针。

简单记忆法：

A 相跳变 = 数一格

看 B 相此时是高还是低 = 判断是加还是减

例子说明：

如果 A 先变、B 后变，就是顺时针（数值增加）

如果 B 先变、A 后变，就是逆时针（数值减少）

*/

// 左拨档开关引脚（选择轴）
#define LEFT_SW1 GPIO_NUM_4    // X轴选择开关
#define LEFT_SW2 GPIO_NUM_5    // Y轴选择开关
#define LEFT_SW3 GPIO_NUM_3    // Z轴选择开关
#define LEFT_SW4 GPIO_NUM_2    // A轴选择开关

// 右拨档开关引脚（选择倍率）
#define RIGHT_SW1 GPIO_NUM_9   // ×0.1倍率选择开关
#define RIGHT_SW2 GPIO_NUM_18  // ×1.0倍率选择开关
#define RIGHT_SW3 GPIO_NUM_19  // ×5.0倍率选择开关

#define UART_PORT_NUM      UART_NUM_0
#define UART_TX_PIN        GPIO_NUM_21
#define UART_RX_PIN        GPIO_NUM_22
#define UART_BAUD_RATE     115200

#define ESTOP_PIN GPIO_NUM_23   // 急停按钮引脚
#define ESTOP_DEBOUNCE_MS 50    // 防抖时间 50ms

#define FUNC_BTN_PIN GPIO_NUM_20   // 功能按键

static const char *TAG = "ENCODER";  // 日志标签
static pcnt_unit_t pcnt_unit = PCNT_UNIT_0;  // 使用的脉冲计数器单元
static volatile bool estop_triggered = false;  // 急停状态标志（true = 已急停，false = 正常）
static volatile uint32_t last_estop_tick = 0;  // 记录上次中断触发时间（tick）
static void IRAM_ATTR estop_isr_handler(void* arg);
static void IRAM_ATTR func_btn_isr_handler(void* arg);

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

// ==================== UART 初始化 ====================
static void uart_init(void) {
    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_driver_install(UART_PORT_NUM, 1024, 0, 0, NULL, 0);
    uart_param_config(UART_PORT_NUM, &uart_config);
    uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

// ==================== 功能按键初始化 ====================
static void func_btn_init(void) {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,  // 下降沿触发（按下）
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << FUNC_BTN_PIN),
        .pull_up_en = 1,   // 开启上拉
        .pull_down_en = 0
    };
    gpio_config(&io_conf);

    // 注册中断
    gpio_isr_handler_add(FUNC_BTN_PIN, func_btn_isr_handler, NULL);
}

// ==================== 急停GPIO初始化 ====================
static void estop_init(void) {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_ANYEDGE,  // 上升沿+下降沿都触发
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << ESTOP_PIN),
        .pull_up_en = 1,   // 开启上拉
        .pull_down_en = 0
    };
    gpio_config(&io_conf);

    // 安装GPIO中断服务
    gpio_install_isr_service(0);
    // 注册中断处理函数
    gpio_isr_handler_add(ESTOP_PIN, estop_isr_handler, NULL);
}

// ==================== 急停中断服务函数 ====================
static void IRAM_ATTR estop_isr_handler(void* arg) {
    uint32_t now_tick = xTaskGetTickCountFromISR();
    if ((now_tick - last_estop_tick) < pdMS_TO_TICKS(ESTOP_DEBOUNCE_MS)) {
        return;  // 在防抖时间内，忽略
    }
    last_estop_tick = now_tick;

    int level = gpio_get_level(ESTOP_PIN);

    if (level == 0) {  
        // 按钮按下（假设低电平有效）
        estop_triggered = true;
        const char stop_cmd = 0x18;  // GRBL 急停指令
        uart_write_bytes(UART_PORT_NUM, &stop_cmd, 1);
        //ESP_EARLY_LOGW(TAG, "急停触发，发送0x18");
    } else {  
        // 按钮松开
        estop_triggered = false;
        const char *unlock_cmd = "$X\n";  // GRBL 解锁指令
        uart_write_bytes(UART_PORT_NUM, unlock_cmd, strlen(unlock_cmd));
        //ESP_EARLY_LOGI(TAG, "急停解除，发送$X");
    }
}

// ==================== 功能按键中断回调 ====================
static void IRAM_ATTR func_btn_isr_handler(void* arg) {
    // 这里直接发一个测试指令 "1\n"
    const char *test_cmd = "1\n";
    uart_write_bytes(UART_PORT_NUM, test_cmd, strlen(test_cmd));
    //ESP_EARLY_LOGI(TAG, "功能按键触发，发送测试指令: %s", test_cmd);
}

// ==================== 发送指令帧 ====================
static void send_command_frame(float scaled_steps, int axis_index) {
    if (estop_triggered) {
        // 急停状态下不再发送运动指令
        return;
    }
    char cmd[128];
    float x = 0, y = 0, z = 0,A = 0;
    float feedrate = 3000.0f;   // 默认移动速度

    switch (axis_index) {
        case 0: x = scaled_steps; break;
        case 1: y = scaled_steps; break;
        case 2: z = scaled_steps; break;
        case 3: A = scaled_steps; break; 
    }

    snprintf(cmd, sizeof(cmd), "$J=G21G91X%.2fY%.2fZ%.2fA%.2fF%.1f\n",
             x, y, z, A,feedrate);

    uart_write_bytes(UART_PORT_NUM, cmd, strlen(cmd));
    //ESP_LOGI(TAG, "发送指令: %s", cmd);
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
/*拨档档位的防抖，只有大于三次是同一个档位才输出*/
DEFINE_SWITCH_STABLE(char)
DEFINE_SWITCH_STABLE(float)

// ==================== 编码器轮询任务 ====================
//每20ms检测一次，有位移就输出，没有位移就不输出
static void encoder_poll_task(void *arg) {
    int16_t raw_count = 0;
    const char axis_names[4] = {'X', 'Y', 'Z', 'A'};
    while (1) {
        // 获取当前脉冲计数值
        pcnt_get_counter_value(pcnt_unit, &raw_count);
        
        // 如果有脉冲，处理并输出增量
        if (raw_count != 0) {
            // // 除以2.0f是因为每个完整周期有2个脉冲（A相和B相）
            float scaled_steps = (raw_count / 2.0f) * right_multiplier;
            
            // 累加到当前轴的总位移
            axis_counts[current_axis] += scaled_steps;
            
            // 输出增量信息
            //ESP_LOGI(TAG, "轴: %c, 倍率: ×%.1f, 增量: %.2f", 
                    // axis_names[current_axis], right_multiplier, scaled_steps);
            // 发送指令帧
                    send_command_frame(scaled_steps, current_axis);
                    axis_last_report[current_axis] = axis_counts[current_axis];
            // 清除计数器
            pcnt_counter_clear(pcnt_unit);
        }
        
        // 等待20ms进行下一次检测
        vTaskDelay(pdMS_TO_TICKS(20));
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
                //ESP_LOGI(TAG, "切换到轴: %c", new_left);
            }
            if (new_right != 0.0f) {
                right_pos = new_right;
                right_multiplier = right_pos;
                //ESP_LOGI(TAG, "倍率调整: ×%.1f", right_pos);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ==================== 主程序入口 ====================
void app_main(void) {
    ESP_LOGI(TAG, "初始化旋转编码器 + 拨档开关");

    uart_init();
    encoder_init();
    switch_init();
    estop_init();   // 初始化急停按键
    func_btn_init();   // 初始化功能按键
    //serial3_init();

    // 编码器轮询任务（优先级5，栈大小4096字节）
    xTaskCreate(encoder_poll_task, "encoder_poll_task", 4096, NULL, 5, NULL);
    // 拨档任务（优先级5，栈大小2048字节）
    xTaskCreate(switch_task, "switch_task", 2048, NULL, 5, NULL);
}
