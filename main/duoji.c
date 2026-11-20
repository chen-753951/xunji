/*
 * 遥控模块 (duoji.c): WiFi AP + TCP 服务器 + 1 路舵机
 *
 *  - 舵机: LEDC 50Hz PWM (timer1, 14bit), 0.5~2.5ms 脉宽对应 0~180 度
 *  - WiFi: ESP32 开热点(AP), 手机连上后通过 TCP 发文本指令控制
 *  - 手机首次连上后 s_remote_mode=1 永久锁定, 断联只停车等待重连, 不切回自动
 *
 * 指令协议 (每条一行, 可带 \n 或 \r\n 结尾):
 *   F 前进  B 后退  L 原地左转  R 原地右转  S 停止
 *   M <L> <R>   直接给左右轮占空比 (-1023~1023)
 *   A <deg>     舵机角度 (0~180)
 *   G <deg>     舵机角度 (0~180, 与 A 等效)
 */

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "driver/ledc.h"
#include "lwip/sockets.h"
#include "duoji.h"
#include "motor.h"

/* 舵机 PWM 配置: 电机已占 timer0/ch0,1, 舵机用 timer1/ch2 */
#define SERVO_LEDC_MODE    LEDC_LOW_SPEED_MODE
#define SERVO_LEDC_TIMER   LEDC_TIMER_1
#define SERVO_CH           LEDC_CHANNEL_2

/* 50Hz(20ms)下 14bit(0~16383): 0.5ms 脉宽=410, 2.5ms 脉宽=2048 */
#define SERVO_DUTY_MIN     410
#define SERVO_DUTY_MAX     2048

/* 遥控模式标志: 手机首次连上后置 true 并永久锁定;
 * 断联不清除, 只停车等待重连, 不会切回自动模式 */
static volatile bool s_remote_mode = false;

/* ================================================================== */
/*                             舵机控制                                */
/* ================================================================== */

static void servo_init(void)
{
    /* 独立定时器: 50Hz, 14bit */
    ledc_timer_config_t timer = {
        .speed_mode      = SERVO_LEDC_MODE,
        .timer_num       = SERVO_LEDC_TIMER,
        .duty_resolution = LEDC_TIMER_14_BIT,
        .freq_hz         = 50,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t ch = {
        .speed_mode = SERVO_LEDC_MODE,
        .channel    = SERVO_CH,
        .timer_sel  = SERVO_LEDC_TIMER,
        .gpio_num   = PIN_SERVO,
        .duty       = 0,
        .hpoint     = 0,
    };
    ledc_channel_config(&ch);

    /* 上电默认中位角, 避免舵机猛地打到极限 */
    duoji_set_servo(90);
}

void duoji_set_servo(int angle)
{
    if (angle < 0)   angle = 0;
    if (angle > 180) angle = 180;

    uint32_t duty = SERVO_DUTY_MIN + (SERVO_DUTY_MAX - SERVO_DUTY_MIN) * angle / 180;
    ledc_set_duty(SERVO_LEDC_MODE, SERVO_CH, duty);
    ledc_update_duty(SERVO_LEDC_MODE, SERVO_CH);
}

/* ================================================================== */
/*                             WiFi 热点                               */
/* ================================================================== */

static void wifi_ap_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    //初始化网络接口
    ESP_ERROR_CHECK(esp_netif_init());//初始化 TCP/IP 网络抽象层
    ESP_ERROR_CHECK(esp_event_loop_create_default());//创建事件循环，WiFi 的“连上/断开”等事件都通过它分发。
    esp_netif_create_default_wifi_ap();     /* AP 默认 IP 192.168.4.1 */

    // 初始化 WiFi 驱动
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // 配置 WiFi AP 参数
    wifi_config_t wifi_config = {
        .ap = {
            .max_connection = 1,             /* 一次只连一个手机 */
            .authmode       = WIFI_AUTH_WPA2_PSK,
            .channel        = 1,
        },
    };
    strlcpy((char *)wifi_config.ap.ssid,     AP_SSID, sizeof(wifi_config.ap.ssid));
    strlcpy((char *)wifi_config.ap.password, AP_PASS, sizeof(wifi_config.ap.password));
    wifi_config.ap.ssid_len = strlen(AP_SSID);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    printf("WiFi AP ready: %s -> 192.168.4.1:%d\n", AP_SSID, TCP_PORT);
}

/* ================================================================== */
/*                             指令解析                                */
/* ================================================================== */

static int clamp_duty(int v)
{
    if (v > 1023)  v = 1023;
    if (v < -1023) v = -1023;
    return v;
}

static void handle_command(const char *line)
{
    int a = 0, b = 0;

    switch (line[0]) {
    case 'F': case 'f':
        car_drive(REMOTE_SPEED, REMOTE_SPEED);          /* 前进 */
        break;
    case 'B': case 'b':
        car_drive(-REMOTE_SPEED, -REMOTE_SPEED);        /* 后退 */
        break;
    case 'L': case 'l':
        car_drive(-REMOTE_SPEED, REMOTE_SPEED);         /* 原地左转 */
        break;
    case 'R': case 'r':
        car_drive(REMOTE_SPEED, -REMOTE_SPEED);         /* 原地右转 */
        break;
    case 'S': case 's':
        car_stop();                                     /* 停止 */
        break;
    case 'M': case 'm':
        if (sscanf(line, "M %d %d", &a, &b) == 2 || sscanf(line, "m %d %d", &a, &b) == 2) {
            car_drive(clamp_duty(a), clamp_duty(b));
        }
        break;
    case 'A': case 'a':
        if (sscanf(line, "A %d", &a) == 1 || sscanf(line, "a %d", &a) == 1) {
            duoji_set_servo(a);
        }
        break;
    case 'G': case 'g':
        if (sscanf(line, "G %d", &a) == 1 || sscanf(line, "g %d", &a) == 1) {
            duoji_set_servo(a);
        }
        break;
    default:
        printf("unknown cmd: %s\n", line);
        break;
    }
}

/* ================================================================== */
/*                             遥控任务                                */
/* ================================================================== */

static void duoji_task(void *arg)
{
    int listen_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_sock < 0) {
        printf("socket() failed\n");
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port   = htons(TCP_PORT);
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(listen_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        printf("bind() failed\n");
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }
    if (listen(listen_sock, 1) < 0) {
        printf("listen() failed\n");
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }
    printf("TCP server listening on :%d\n", TCP_PORT);

    char rx[256];
    char line_buf[128];
    int  line_len = 0;

    while (1) {
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        int client = accept(listen_sock, (struct sockaddr *)&client_addr, &client_len);
        if (client < 0) {
            continue;
        }
        printf("phone connected, enter remote mode\n");
        s_remote_mode = true;
        car_stop();   /* 连上后立刻停车，防止沿用自动模式最后一拍 PWM */

        /* 1s 接收超时, 便于及时检测断连 */
        struct timeval tv = { .tv_sec = 1, .tv_usec = 0 };
        setsockopt(client, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        while (1) {
            int n = recv(client, rx, sizeof(rx) - 1, 0);
            if (n == 0) {
                break;                              /* 正常断开 */
            }
            if (n < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    continue;                       /* 超时, 继续等 */
                }
                break;                              /* 出错断开 */
            }
            rx[n] = '\0';
            for (int i = 0; i < n; i++) {
                char c = rx[i];
                if (c == '\n' || c == '\r') {
                    if (line_len > 0) {
                        line_buf[line_len] = '\0';
                        handle_command(line_buf);
                        line_len = 0;
                    }
                } else if (line_len < (int)sizeof(line_buf) - 1) {
                    line_buf[line_len++] = c;
                }
            }
        }

        printf("phone disconnected, car stop, waiting reconnect...\n");
        car_stop();                                 /* 断开立即停车 */
        close(client);
        /* 保持 s_remote_mode=true: 回到 accept 等待重连, 不切回自动 */
    }
}

/* ================================================================== */
/*                             对外接口                                */
/* ================================================================== */

void duoji_init(void)
{
    servo_init();
    wifi_ap_init();
    xTaskCreate(duoji_task, "duoji_task", 8192, NULL, 5, NULL);
}

bool duoji_is_remote(void)
{
    return s_remote_mode;
}
