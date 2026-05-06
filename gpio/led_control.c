#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>
#include <string.h>
#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <gpiod.h>

#define PIN_GREEN 27
#define PIN_BLUE  22
#define PIN_RED   17
#define CHECK_INTERVAL_SEC 3

static const char *NETWORK_IFACES[] = { "wlan0", "usb0" };
static const int NUM_IFACES = sizeof(NETWORK_IFACES) / sizeof(NETWORK_IFACES[0]);

// v2: 使用 gpiod_line_request 替代 gpiod_line
struct gpiod_line_request *req_green = NULL;
struct gpiod_line_request *req_blue = NULL;

volatile bool has_signal = false;
volatile bool keep_running = true;

/* ====== 网络状态检测 (sysfs, 无外部依赖) ======
 * 遍历所有监控接口，任一连通即返回 true */
bool check_network_status(void) {
    for (int i = 0; i < NUM_IFACES; i++) {
        const char *iface = NETWORK_IFACES[i];
        char path[128];
        char buf[16] = {0};
        int fd;

        /* Layer 1: 物理载波检测 */
        snprintf(path, sizeof(path), "/sys/class/net/%s/carrier", iface);
        fd = open(path, O_RDONLY);
        if (fd >= 0) {
            read(fd, buf, sizeof(buf) - 1);
            close(fd);
            if (buf[0] != '1') continue;   /* 此接口无载波，查下一个 */
        } else {
            continue;                       /* 接口不存在，跳过 */
        }

        /* Layer 2: 操作状态确认 */
        memset(buf, 0, sizeof(buf));
        snprintf(path, sizeof(path), "/sys/class/net/%s/operstate", iface);
        fd = open(path, O_RDONLY);
        if (fd >= 0) {
            read(fd, buf, sizeof(buf) - 1);
            close(fd);
            if (strncmp(buf, "up", 2) == 0) return true;  /* 此接口已连通 */
        }
    }
    return false;  /* 所有接口均未连通 */
}

void sigint_handler(int sig) {
    printf("\n捕获到退出信号，正在清理 GPIO...\n");
    keep_running = false;
}

void* run_led_thread(void* arg) {
    /* 绿灯：运行指示，程序启动后常亮（低电平=亮） */
    if (req_green) gpiod_line_request_set_value(req_green, PIN_GREEN, 0);
    while (keep_running) { sleep(1); }
    return NULL;
}

void* signal_led_thread(void* arg) {
    int period_us = 20000;   /* 20ms 周期，50Hz PWM，肉眼可见呼吸 */
    int duty = 0, step = 1;

    while (keep_running) {
        if (!req_blue) { usleep(100000); continue; }

        if (has_signal) {
            gpiod_line_request_set_value(req_blue, PIN_BLUE, 0);  /* 低电平常亮 */
            usleep(100000);
        } else {
            int on_time = (duty * period_us) / 100;
            int off_time = period_us - on_time;
            if (on_time > 0) {
                gpiod_line_request_set_value(req_blue, PIN_BLUE, 0);  /* 低电平=亮 */
                usleep(on_time);
            }
            if (off_time > 0) {
                gpiod_line_request_set_value(req_blue, PIN_BLUE, 1);  /* 高电平=灭 */
                usleep(off_time);
            }
            duty += step;
            if (duty >= 100) { duty = 100; step = -1; }
            else if (duty <= 0) { duty = 0; step = 1; }
        }
    }
    return NULL;
}

int main() {
    signal(SIGINT, sigint_handler);

    // v2: 使用 gpiod_chip_open(path)
    struct gpiod_chip *chip = gpiod_chip_open("/dev/gpiochip4");
    if (!chip) {
        chip = gpiod_chip_open("/dev/gpiochip0");
    }
    if (!chip) {
        perror("无法打开 GPIO 芯片");
        return -1;
    }

    // v2: 构建请求配置
    struct gpiod_request_config *req_cfg = gpiod_request_config_new();
    struct gpiod_line_settings *settings = gpiod_line_settings_new();
    struct gpiod_line_config *line_cfg = gpiod_line_config_new();

    gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_OUTPUT);
    gpiod_line_settings_set_output_value(settings, GPIOD_LINE_VALUE_ACTIVE);   /* 初始高电平=灭灯 */

    // 配置引脚（3 个 LED 共用一个 request）
    unsigned int offsets[] = { PIN_GREEN, PIN_BLUE, PIN_RED };
    gpiod_line_config_add_line_settings(line_cfg, offsets, 3, settings);

    // 发起请求
    req_green = gpiod_chip_request_lines(chip, req_cfg, line_cfg);
    if (!req_green) {
        perror("无法请求 GPIO 行");
        gpiod_chip_close(chip);
        return -1;
    }
    req_blue = req_green;  /* 同一个 request 包含所有引脚 */

    /* 红灯：电源指示，启动后立即常亮（低电平=亮） */
    gpiod_line_request_set_value(req_green, PIN_RED, 0);

    pthread_t thread_run, thread_signal;
    pthread_create(&thread_run, NULL, run_led_thread, NULL);
    pthread_create(&thread_signal, NULL, signal_led_thread, NULL);

    printf("基于 libgpiod v2 的程序已启动！(按 Ctrl+C 退出)\n");
    printf("[LED] 红灯(Pin11)=电源常亮 | 绿灯(Pin13)=运行闪烁 | 蓝灯(Pin15)=网络状态\n");
    printf("[主线程] 监控接口:");
    for (int i = 0; i < NUM_IFACES; i++) printf(" %s", NETWORK_IFACES[i]);
    printf(" (检测间隔: %ds, 任一连通即视为有网)\n", CHECK_INTERVAL_SEC);

    while (keep_running) {
        bool new_status = check_network_status();
        if (new_status != has_signal) {
            has_signal = new_status;
            printf("[主线程] ★ 网络状态变更: %s\n",
                   has_signal ? "已连接 -> 蓝灯常亮" : "断开 -> 蓝灯呼吸");
        } else {
            printf("[主线程] 网络状态: %s (无变化)\n",
                   has_signal ? "已连接" : "未连接");
        }
        for (int i = 0; i < CHECK_INTERVAL_SEC; i++) {
            if (!keep_running) break;
            sleep(1);
        }
    }

    pthread_join(thread_run, NULL);
    pthread_join(thread_signal, NULL);

    // 清理（全部高电平=灭灯）
    gpiod_line_request_set_value(req_green, PIN_GREEN, 1);
    gpiod_line_request_set_value(req_blue, PIN_BLUE, 1);
    gpiod_line_request_set_value(req_green, PIN_RED, 1);

    gpiod_line_request_release(req_green);  // 一个 release 即可
    gpiod_line_config_free(line_cfg);
    gpiod_line_settings_free(settings);
    gpiod_request_config_free(req_cfg);
    gpiod_chip_close(chip);

    printf("资源释放完毕，安全退出。\n");
    return 0;
}
