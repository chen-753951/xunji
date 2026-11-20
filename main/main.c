/*
 * ESP32-S3 循迹 + 超声波避障小车 (main.c: 状态机与循迹决策)
 *
 *  - 4 路红外循迹: 高电平 = 黑线  (sensor.c)
 *  - HC-SR04 超声波测距           (sensor.c)
 *  - 双电机 PWM 开环调速          (motor.c)
 *
 * 状态机:
 *  - 循迹态: 距离 >= 10cm, 按红外状态巡线; 距离 < 10cm 进避障态
 *  - 避障态: 第一次识别到障碍置标志位并原地左转(FIRST_TURN_LEFT_MS);
 *            之后再遇到障碍一律原地右转(AVOID_TURN_RIGHT_MS);
 *            无障碍时按循迹速度直行找线;
 *            IR2/IR3 出现 0->1 跳变(重新压到黑线)时:
 *            先制动消惯性, 再原地左转(REALIGN_TURN_MS)对线, 切回循迹态
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "motor.h"
#include "sensor.h"
#include "duoji.h"

/* 避障距离阈值 */
#define OBSTACLE_DIST_CM    25.0f

/* 控制周期 */
#define CTRL_PERIOD_MS      10

/* ========================= 状态机 ========================= */
typedef enum {
    MODE_TRACK = 0,     /* 循迹 */
    MODE_AVOID  = 1,    /* 避障(右转找线) */
} car_mode_t;

static car_mode_t s_mode = MODE_TRACK;

/* 避障标志: 第一次识别到障碍置1, 之后避障只右转 */
static int s_avoid_flag = 0;
/* 避障态 IR2/IR3 上一拍状态(只看 bit2/bit1): 用来检测 0->1 跳变 */
static int s_avoid_last_mid = 0;

/* ========================= 避障参数 ========================= */
#define AVOID_SPEED          365     /* 首次避障转身速度 */
#define AVOID_TURN_SPEED     700     /* 后续避障右转/对线转身速度: 两轮正负600 */
#define FIRST_TURN_LEFT_MS   450    /* 首次避障: 原地左转时长 */
#define AVOID_TURN_RIGHT_MS  200    /* 后续避障: 每次原地右转时长 */

/* 避障动作统一先制动消惯性再转身/对线 (力度独立于循迹态刹车) */
#define AVOID_BRAKE_MS       300    /* 避障转身前制动时长 */
#define AVOID_BRAKE_DUTY     (-150)  /* 避障转身前制动占空比 */

/* 避障->循迹 对线: IR2/IR3 跳变瞬间先制动消惯性,
 * 再一律原地左转对线, 完成后切回循迹 */
#define REALIGN_TURN_MS      500    /* 对线左转时长 */

/* 原地左转: 左轮反转, 右轮正转; 原地右转相反. 两轮速度相同 */
static void car_pivot_left(int duty)  { car_drive(-duty, duty); }
static void car_pivot_right(int duty) { car_drive(duty, -duty); }

/* ========================= 权重式循迹参数 ========================= */
#define STEER_GAIN     120      /* 权重 -> 占空比换算系数 */
#define STEER_MAX      3       /* 偏差限幅, 决定最大转弯力度 */

/* 消惯性刹车: 只有最外侧探头 IR1(最左)/IR4(最右) 的信号发生跳变
 * (压到线或离开线)时, 两轮才以 BRAKE_DUTY 反转保持 BRAKE_TIME_MS 消惯性,
 * 非阻塞. IR2/IR3 的信号变化不刹车, 直接按加权循迹处理.
 * 刹车期间持续刷新信号基线, 刹完不会连环刹车 */
#define BRAKE_DUTY      (-300)   /* 刹车占空比: 反转制动 */
#define BRAKE_TIME_MS   200     /* 刹车保持时长 */

static int64_t s_brake_until_us = 0;    /* 刹车截止时刻(us), 0=空闲 */
static int s_last_s = -1;               /* 上一拍红外信号, -1=上电首拍不刹车 */
static int s_brake_error = 0;           /* 刹车触发瞬间记录的加权偏差 */
static int s_pivot_hold = 0;            /* 1=按记录偏差持续原地转中, 直到 s=0000 */

/*
 * 权重表: IR1~IR4 从左到右的位置权重
 * 负 = 线偏左(需左转), 正 = 线偏右(需右转)
 * 离中间越远权重越大 -> 偏离越远转弯力度越大
 * 想改成别的力度组合只改这张表即可
 */
static const int8_t ir_weight[4] = {3, 2, -2, -3};   /* IR1 最左 ~ IR4 最右 */

/* 按偏差原地转: error>0 右转, error<0 左转, error=0 两轮同速直行 */
static void pivot_by_error(int error)
{
    int diff = error * STEER_GAIN;

    int duty_l, duty_r;
    if (error > 0) {
        /* 线偏右 -> 原地右转: 左轮 = 基准 + 差分 (正转), 右轮取负 (反转) */
        duty_l =  400 + diff;
        duty_r = -(400 + diff);
    } else if (error < 0) {
        /* 线偏左 -> 原地左转: 左轮取负 (反转), 右轮 = 基准 + |差分| (正转) */
        duty_l = -(400 - diff);
        duty_r =  400 - diff;
    } else {
        /* 偏差为0: 两轮同速直行 */
        duty_l = SPEED_AVG;
        duty_r = SPEED_AVG;
    }

    /* 带符号限幅 -1023~1023: 正=正转 负=反转 */
    if (duty_l > 1023) duty_l = 1023;
    if (duty_l < -1023) duty_l = -1023;
    if (duty_r > 1023) duty_r = 1023;
    if (duty_r < -1023) duty_r = -1023;

    car_drive(duty_l, duty_r);
}

/* 求红外状态 s 的加权偏差并限幅: 线偏左<0, 偏右>0 (对线/循迹共用权重表) */
static int error_of(int s)
{
    int err = 0;
    if (s & 0x8) err += ir_weight[0];   /* IR1 最左 */
    if (s & 0x4) err += ir_weight[1];   /* IR2 中左 */
    if (s & 0x2) err += ir_weight[2];   /* IR3 中右 */
    if (s & 0x1) err += ir_weight[3];   /* IR4 最右 */
    if (err > STEER_MAX) err = STEER_MAX;
    if (err < -STEER_MAX) err = -STEER_MAX;
    return err;
}

/* ================================================================== */
/*                             循迹逻辑                                */
/* ================================================================== */

/*
 * 权重式循迹 + 原地差速转向:
 *  0. IR1/IR4 信号跳变: 刹车 0.4s 并记录此刻加权偏差, 刹完后按记录偏差
 *     持续原地转, 一直转到 s=0000 (线转出探头视野)才交回循迹逻辑
 *  1. 把压线的传感器权重累加成偏差 error (线偏左 error<0, 偏右 error>0)
 *  2. 差分速度 diff = error * 系数
 *  3. error=0 (双中压线): 两轮都以平均速度直行
 *  4. error!=0: 原地转, 内侧轮反转:
 *     线偏右 -> 左轮 = 平均速度 + diff (正转), 右轮 = -(平均速度 + diff) (反转)
 *     线偏左 -> 左右镜像: 左轮反转, 右轮正转, 大小同为 平均速度 + |diff|
 *
 * 力度举例 (权重 {-3,-3,3,3}, 平均速度 400, 系数 120):
 *  0010 IR3 压线   error=+3  左轮 +760 右轮 -760  原地右转
 *  0100 IR2 压线   error=-3  左轮 -760 右轮 +760  原地左转
 *  0110 双中压线   error=0   左轮 +400 右轮 +400  直行
 */
static void track_control(void)
{
    static int last_error = 0;      /* 全丢线时沿用上次的偏差去找线 */
    int64_t now = esp_timer_get_time();

    /* 刹车保持期: 两轮反转消惯性, 到时后恢复循迹 (非阻塞) */
    if (now < s_brake_until_us) {
        car_drive(BRAKE_DUTY, BRAKE_DUTY);
        s_last_s = ir_read_all();       /* 刷新信号基线, 刹完不会连环刹车 */
        return;
    }

    int s = ir_read_all();

    /* 刹车结束: 按刹车前记录的加权偏差持续原地转, 一直转到 s=0000
     * (线完全转出探头视野)才交回循迹逻辑; 期间的信号变化不触发刹车 */
    if (s_pivot_hold) {
        s_last_s = s;
        if (s != 0b0000) {
            pivot_by_error(s_brake_error);
            return;
        }
        s_pivot_hold = 0;
        printf("pivot done (s=0000) -> track\n");
        /* 落到下面的循迹逻辑: 0000 直行, 再次压线后按循迹处理 */
    }

    /* 只有 IR1(最左)/IR4(最右) 信号跳变: 刹车消惯性,
     * 并记录此刻的加权偏差, 刹完车按它持续原地转到 s=0000 */
    if (s_last_s != -1 && ((s ^ s_last_s) & 0b1001)) {
        int prev = s_last_s;
        s_last_s = s;
        int err = 0;
        if (s & 0x8) err += ir_weight[0];
        if (s & 0x4) err += ir_weight[1];
        if (s & 0x2) err += ir_weight[2];
        if (s & 0x1) err += ir_weight[3];
        if (err > STEER_MAX) err = STEER_MAX;
        if (err < -STEER_MAX) err = -STEER_MAX;
        s_brake_error = err;
        s_pivot_hold = 1;
        s_brake_until_us = now + BRAKE_TIME_MS * 1000;
        car_drive(BRAKE_DUTY, BRAKE_DUTY);
        printf("IR1/4 edge %d->%d: brake %.2fs, err=%d\n",
               prev, s, BRAKE_TIME_MS / 1000.0f, err);
        return;
    }
    s_last_s = s;

    /* 全压线(直角/宽线): 直行 */
    if (s == 0b1111) {
        car_forward();

        return;
    }
    if (s == 0b0000) {
        car_forward();

        return;
    }
    /* 加权求偏差 */
    int error = 0;
    if (s & 0x8) error += ir_weight[0];     /* IR1 最左 */
    if (s & 0x4) error += ir_weight[1];     /* IR2 中左 */
    if (s & 0x2) error += ir_weight[2];     /* IR3 中右 */
    if (s & 0x1) error += ir_weight[3];     /* IR4 最右 */

    /* 偏差限幅 (如 1100 累加得 -4, 限到 -4) */
    if (error > STEER_MAX) error = STEER_MAX;
    if (error < -STEER_MAX) error = -STEER_MAX;
    if (s != 0) last_error = error;         /* 有效压线时记忆偏差 */
    else        error = last_error;         /* 丢线时按记忆偏差继续找 */

    /* 双中压线, 偏差为0: 直行 */
    if (error == 0) {
        car_drive(SPEED_AVG, SPEED_AVG);
        return;
    }

    /* 偏差(IR1/IR4 跳变时已在上面刹过车): 按原地转修正 */
    pivot_by_error(error);
}

/* ================================================================== */
/*                             主任务                                  */
/* ================================================================== */

static void car_task(void *arg)
{
    float dist;

    while (1) {
        /* 遥控模式: 手机首次连上后永久锁定, 断联只停车等待重连,
         * 循迹+避障自动逻辑永久失效 */
        if (duoji_is_remote()) {
            vTaskDelay(pdMS_TO_TICKS(CTRL_PERIOD_MS));
            continue;
        }

        /* 每个控制周期测一次距离 */
        dist = ultrasonic_get_distance();

        if (s_mode == MODE_AVOID) {
            int s = ir_read_all();
            int mid = s & 0b0110;

            /* 基线统一在此更新: 跳变检测用的上一拍值先存下来 */
            int mid_prev = s_avoid_last_mid;
            s_avoid_last_mid = mid;

            if (mid && !mid_prev) {
                /* IR2/IR3 的 0->1 跳变 = 重新压到黑线:
                 * 先制动消惯性, 再一律原地左转对线, 完成后回循迹 */
                printf("line jump (IR2/3 0->1, err=%d): brake %dms\n",
                       error_of(s), AVOID_BRAKE_MS);
                car_drive(AVOID_BRAKE_DUTY, AVOID_BRAKE_DUTY);
                vTaskDelay(pdMS_TO_TICKS(AVOID_BRAKE_MS));
                
                car_pivot_right(700);
                vTaskDelay(pdMS_TO_TICKS(250));
                s_mode = MODE_TRACK;
                /* 清掉循迹态刹车/原地转残留, 回循迹后重新采样 */
                s_brake_until_us = 0;
                s_pivot_hold = 0;
                s_last_s = -1;
            } else {
                if (dist < OBSTACLE_DIST_CM) {
                    /* 后续遇到障碍: 先制动消惯性, 再原地右转一段再找路 */
                    printf("avoiding, dist=%.1fcm: brake %dms\n",
                           dist, AVOID_BRAKE_MS);
                    car_drive(AVOID_BRAKE_DUTY, AVOID_BRAKE_DUTY);
                    vTaskDelay(pdMS_TO_TICKS(AVOID_BRAKE_MS));
                    printf("avoiding: pivot %dms\n", AVOID_TURN_RIGHT_MS);
                    car_pivot_left(AVOID_TURN_SPEED);
                    vTaskDelay(pdMS_TO_TICKS(AVOID_TURN_RIGHT_MS));
                    /* 转身过程不采样, 以转完后的信号作为新基线, 防止误判跳变 */
                    s_avoid_last_mid = ir_read_all() & 0b0110;
                } else {
                    /* 无障碍: 按循迹速度直行找线 */
                    car_forward();
                }
            }
        } else {
            /* 循迹状态: 距离 < 10cm 时无论在不在黑线上都进入避障 */
            if (dist < OBSTACLE_DIST_CM) {
                s_mode = MODE_AVOID;
                printf("obstacle %.1fcm -> avoid mode\n", dist);
                if (!s_avoid_flag) {
                    /* 第一次识别到障碍: 置标志位1, 原地左转 */
                    s_avoid_flag = 1;
                    printf("first obstacle: pivot left %dms\n", FIRST_TURN_LEFT_MS);
                    car_pivot_right(AVOID_SPEED);
                    vTaskDelay(pdMS_TO_TICKS(FIRST_TURN_LEFT_MS));
                }
                /* 后续避障不在这里转, 由避障态下一周期统一右转,
                 * 否则会和避障态的右转连转两次 */
                /* 转身后刷新基线, 转身造成的信号变化不算"重新找到线" */
                s_avoid_last_mid = ir_read_all() & 0b0110;
            } else {
                track_control();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(CTRL_PERIOD_MS));
    }
}

void app_main(void)
{
    motor_init();
    sensor_init();
    duoji_init();
    car_stop();
    printf("car ready, start tracking\n");

    xTaskCreate(car_task, "car_task", 4096, NULL, 5, NULL);
}
