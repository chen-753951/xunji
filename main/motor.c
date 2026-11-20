/*
 * 电机驱动模块 (motor.c)
 * IN1~IN4 控方向, ENA/ENB 走 PWM 开环调速
 */

#include "motor.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

/* ================================================================== */
/*                             初始化                                  */
/* ================================================================== */

void motor_init(void)
{
    /* 方向脚 IN1~IN4 输出 */
    gpio_config_t io_out = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << PIN_IN1) | (1ULL << PIN_IN2) |
                        (1ULL << PIN_IN3) | (1ULL << PIN_IN4),
    };
    gpio_config(&io_out);

    /* PWM 定时器 */
    ledc_timer_config_t timer_conf = {
        .speed_mode      = LEDC_MODE,
        .timer_num       = LEDC_TIMER_0,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz         = LEDC_FREQUENCY,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer_conf);

    /* 左右两个 PWM 通道 */
    const int chs[2] = {LEDC_CH_LEFT, LEDC_CH_RIGHT};
    const gpio_num_t en_pins[2] = {PIN_ENA, PIN_ENB};
    for (int i = 0; i < 2; i++) {
        ledc_channel_config_t ch_conf = {
            .speed_mode = LEDC_MODE,
            .channel    = chs[i],
            .timer_sel  = LEDC_TIMER_0,
            .gpio_num   = en_pins[i],
            .duty       = 0,
            .hpoint     = 0,
        };
        ledc_channel_config(&ch_conf);
    }
}

/* ================================================================== */
/*                             电机控制                                */
/* ================================================================== */

/* 设置左电机: dir=1 正转 / 0 反转, duty=0 时停止 */
void motor_left(int dir, uint32_t duty)
{
    if (duty == 0) {
        gpio_set_level(PIN_IN1, 0);
        gpio_set_level(PIN_IN2, 0);
    } else if (dir) {
        gpio_set_level(PIN_IN1, 1);
        gpio_set_level(PIN_IN2, 0);
    } else {
        gpio_set_level(PIN_IN1, 0);
        gpio_set_level(PIN_IN2, 1);
    }
    ledc_set_duty(LEDC_MODE, LEDC_CH_LEFT, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CH_LEFT);
}

/* 设置右电机: dir=1 正转 / 0 反转, duty=0 时停止 */
void motor_right(int dir, uint32_t duty)
{
    if (duty == 0) {
        gpio_set_level(PIN_IN3, 0);
        gpio_set_level(PIN_IN4, 0);
    } else if (dir) {
        gpio_set_level(PIN_IN3, 1);
        gpio_set_level(PIN_IN4, 0);
    } else {
        gpio_set_level(PIN_IN3, 0);
        gpio_set_level(PIN_IN4, 1);
    }
    ledc_set_duty(LEDC_MODE, LEDC_CH_RIGHT, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CH_RIGHT);
}

void car_stop(void)
{
    motor_left(1, 0);
    motor_right(1, 0);
}

void car_forward(void)
{
    motor_left(1, SPEED_AVG);
    motor_right(1, SPEED_AVG);
}

/* 带符号差速: duty>0 正转, <0 反转, =0 停止(滑行); 绝对值为 PWM 占空比 */
void car_drive(int duty_l, int duty_r)
{                   //方向                      +               速度
    motor_left(duty_l >= 0 ? 1 : 0, (uint32_t)(duty_l >= 0 ? duty_l : -duty_l));
    motor_right(duty_r >= 0 ? 1 : 0, (uint32_t)(duty_r >= 0 ? duty_r : -duty_r));
}

/* 右转: 左轮正转, 右轮反转(原地右转), 避障用 */
void car_turn_right(void)
{
    motor_left(1, SPEED_AVOID);
    motor_right(0, SPEED_AVOID);
}
