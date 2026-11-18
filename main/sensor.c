/*
 * 传感器模块 (sensor.c): 4 路红外循迹 + HC-SR04 超声波
 */

#include "sensor.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

/* ================================================================== */
/*                             初始化                                  */
/* ================================================================== */

void sensor_init(void)
{
    /* 红外循迹: 输入, 上拉 */
    gpio_config_t io_in = {
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,   /* 多数红外模块悬空时默认输出需上拉 */
        .pin_bit_mask = (1ULL << PIN_IR1) | (1ULL << PIN_IR2) |
                        (1ULL << PIN_IR3) | (1ULL << PIN_IR4),
    };
    gpio_config(&io_in);

    /* 超声波 ECHO 输入, TRIG 输出并初始拉低 */
    io_in.pin_bit_mask = 1ULL << PIN_ECHO;
    io_in.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_in);

    gpio_config_t io_out = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << PIN_TRIG,
    };
    gpio_config(&io_out);
    gpio_set_level(PIN_TRIG, 0);            /* TRIG 初始拉低 */
}

/* ================================================================== */
/*                             红外循迹                                */
/* ================================================================== */

/*
 * 读 4 路红外, 返回位图 bit3~bit0 = IR1~IR4, 1 = 检测到黑线
 * 注: 若你的模块是低电平 = 黑线, 在这里把电平取反即可
 */
int ir_read_all(void)
{
    int v = 0;
    if (gpio_get_level(PIN_IR1)) v |= 0x8;
    if (gpio_get_level(PIN_IR2)) v |= 0x4;
    if (gpio_get_level(PIN_IR3)) v |= 0x2;
    if (gpio_get_level(PIN_IR4)) v |= 0x1;
    return v;
}

/* ================================================================== */
/*                             超声波测距                              */
/* ================================================================== */

/*
 * 时序: TRIG 拉低(保证初始状态) -> 拉高 >10us -> 拉低
 *       等 Echo 变高开始计时, Echo 变低停止
 *       距离 = 时间(us) * 0.034(cm/us) / 2
 */
float ultrasonic_get_distance(void)
{
    int64_t start, t;

    /* 1. TRIG 先确保拉低, 保持稳定初始状态 */
    gpio_set_level(PIN_TRIG, 0);
    esp_rom_delay_us(5);

    /* 2. TRIG 拉高 >10us */
    gpio_set_level(PIN_TRIG, 1);
    esp_rom_delay_us(15);
    gpio_set_level(PIN_TRIG, 0);

    /* 3. 等待 Echo 变高, 开始计时 */
    start = esp_timer_get_time();
    while (gpio_get_level(PIN_ECHO) == 0) {
        if (esp_timer_get_time() - start > ECHO_TIMEOUT_US) {
            return 999.0f;      /* 没回波, 视为前方无障碍 */
        }
    }
    start = esp_timer_get_time();

    /* 4. 等待 Echo 变低, 停止计时 */
    while (gpio_get_level(PIN_ECHO) == 1) {
        t = esp_timer_get_time();
        if (t - start > ECHO_TIMEOUT_US) {
            return 999.0f;
        }
    }
    t = esp_timer_get_time();

    /* 5. 计算距离 */
    return (float)(t - start) * 0.034f / 2.0f;
}
