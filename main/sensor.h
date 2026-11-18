#ifndef SENSOR_H
#define SENSOR_H

/*
 * 传感器: 4 路红外循迹 + HC-SR04 超声波
 *  - 红外输出高电平 = 检测到黑线, 排列从车头左到右: IR1 IR2 IR3 IR4
 *  - 超声波: TRIG 拉高 >10us 后拉低, ECHO 高电平时间为声波往返时间
 *      距离(cm) = 高电平时间(us) * 0.034 / 2
 */

/* 红外引脚分配 (可按实际接线修改) */
#define PIN_IR1   4      /* 最左 */
#define PIN_IR2   5      /* 中左 */
#define PIN_IR3   6      /* 中右 */
#define PIN_IR4   7      /* 最右 */

/* 超声波引脚 */
#define PIN_TRIG  15
#define PIN_ECHO  16

/* Echo 等待超时 ~5m */
#define ECHO_TIMEOUT_US   30000

/* 初始化红外输入脚 + 超声波 TRIG/ECHO 脚 */
void sensor_init(void);

/*
 * 读 4 路红外, 返回位图 bit3~bit0 = IR1~IR4, 1 = 检测到黑线
 * 例: 0b0110 = 中间两个压线
 */
int ir_read_all(void);

/* 获取超声波距离(cm), 超时/无回波返回 999 (视为前方无障碍) */
float ultrasonic_get_distance(void);

#endif
