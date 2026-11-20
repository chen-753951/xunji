#ifndef DUOJI_H
#define DUOJI_H

#include <stdbool.h>

/*
 * 遥控模块 (duoji): WiFi AP + TCP 服务器 + 2 路舵机(PWM)
 *  - 手机连热点后通过 TCP 发指令, 控制小车移动 + 舵机抓取
 *  - 遥控期间 main.c 的循迹/避障自动逻辑失效
 */

/* 舵机引脚 (可按实际接线修改) */
#define PIN_SERVO_ARM   17      /* 抬臂舵机 */
#define PIN_SERVO_CLAW  18      /* 抓取爪舵机 */

/* 舵机编号 */
#define SERVO_ARM   0
#define SERVO_CLAW  1

/* WiFi 热点配置 */
#define AP_SSID     "xunji-car"
#define AP_PASS     "12345678"

/* TCP 服务器端口 */
#define TCP_PORT    3333

/* 遥控电机速度 (开环占空比 0~1023) */
#define REMOTE_SPEED    400

/* 初始化舵机 PWM + WiFi AP + TCP 服务器, 并启动遥控任务 */
void duoji_init(void);

/* 是否处于遥控模式 (手机首次连上后永久锁定, 断联只停车等重连) */
bool duoji_is_remote(void);

/* 设置舵机角度: servo_id = SERVO_ARM/SERVO_CLAW, angle 0~180 */
void duoji_set_servo(int servo_id, int angle);

#endif
