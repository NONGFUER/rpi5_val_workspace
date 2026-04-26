#include <gpiod.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

int main() {
    const char *chip_path = "/dev/gpiochip0";
    unsigned int offset = 12; // GPIO 12
    struct gpiod_chip *chip;
    struct gpiod_line_settings *settings;
    struct gpiod_line_config *line_cfg;
    struct gpiod_request_config *req_cfg;
    struct gpiod_line_request *request;
    enum gpiod_line_value value = GPIOD_LINE_VALUE_ACTIVE;

    // 1. 打开 GPIO 芯片
    chip = gpiod_chip_open(chip_path);
    if (!chip) {
        perror("打开芯片失败");
        return EXIT_FAILURE;
    }

    // 2. 配置引脚设置 (输出模式)
    settings = gpiod_line_settings_new();
    gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_OUTPUT);

    // 3. 配置行映射
    line_cfg = gpiod_line_config_new();
    gpiod_line_config_add_line_settings(line_cfg, &offset, 1, settings);

    // 4. 配置请求信息
    req_cfg = gpiod_request_config_new();
    gpiod_request_config_set_consumer(req_cfg, "led-blink-c-v2");

    // 5. 请求控制权
    request = gpiod_chip_request_lines(chip, req_cfg, line_cfg);
    if (!request) {
        perror("请求 GPIO 线失败");
        return EXIT_FAILURE;
    }

    printf("C程序：GPIO 17 正在闪烁...\n");

    while (1) {
        gpiod_line_request_set_value(request, offset, value);
        printf("电平状态: %s\n", value == GPIOD_LINE_VALUE_ACTIVE ? "高" : "低");
        
        // 翻转电平
        value = (value == GPIOD_LINE_VALUE_ACTIVE) ? 
                 GPIOD_LINE_VALUE_INACTIVE : GPIOD_LINE_VALUE_ACTIVE;
        usleep(500000); // 500ms
    }

    // 释放资源（实际运行中 Ctrl+C 退出，正式产品建议捕捉信号进行清理）
    gpiod_line_request_release(request);
    gpiod_chip_close(chip);
    return 0;
}