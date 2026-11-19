#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

/*
 * 电机驱动(如 TB6612 / L298N):
 *  - IN1~IN4 控制两个电机方向
 *  - ENA/ENB 接 PWM 控制速度 (开环, 直接给定占空比)
 */

/* 引脚分配 (可按实际接线修改) */
#define PIN_IN1   1      /* 左电机 IN1 */
#define PIN_IN2   2      /* 左电机 IN2 */
#define PIN_IN3   41     /* 右电机 IN3 */
#define PIN_IN4   42     /* 右电机 IN4 */
#define PIN_ENA   39     /* 左电机使能/PWM */
#define PIN_ENB   40     /* 右电机使能/PWM */

/* PWM 配置 */
#define LEDC_MODE         0                    /* 低速模式 */
#define LEDC_DUTY_RES     10                   /* 10bit: 占空比 0~1023 */
#define LEDC_FREQUENCY    1000                 /* 1kHz */
#define LEDC_CH_LEFT      0                    /* 左电机通道 */
#define LEDC_CH_RIGHT     1                    /* 右电机通道 */

/* 速度参数 (开环, 按车况调整) */
#define SPEED_AVG      300    /* 平均速度: 直行速度/原地转基准占空比 0~1023 */
#define SPEED_AVOID    550     /* 避障右转速度 */

/* 初始化 PWM 和方向引脚 */
void motor_init(void);

/* 左/右电机: dir=1 正转 / 0 反转, duty=0 时停止 */
void motor_left(int dir, uint32_t duty);
void motor_right(int dir, uint32_t duty);

/* 整车动作 */
void car_stop(void);
void car_forward(void);
void car_drive(int duty_l, int duty_r);             /* 带符号差速: >0正转 <0反转 =0停 */
void car_turn_right(void);                          /* 原地右转(避障用) */

#endif
