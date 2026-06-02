/**
 * ============================================================================
 * @file    modbus_scale_test.c
 * @brief   Modbus RTU 称重读取调试工具 (手动触发 / 实时更新 双模式)
 *
 * 功能说明：
 *   模式1 - 手动触发: 按回车键发送一次请求, 逐帧观察收发数据
 *   模式2 - 实时更新: 自动循环读取, 原地刷新显示重量值
 *
 *  通信参数: 9600bps 8N1, 从站地址 0x01
 * 编译: gcc -Wall -Wextra -o modbus_scale_test modbus_scale_test.c
 * 运行: sudo ./modbus_scale_test [/dev/ttyAMA0] [manual|realtime]   Ctrl+C 退出
 *
 * @author  AI Assistant
 * @date    2026-05-22
 * ============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdint.h>
#include <signal.h>

static volatile sig_atomic_t g_run = 1;

static void on_sigint(int sig) { (void)sig; g_run = 0; }

/* ======================== CRC-16 Modbus ======================== */
uint16_t crc16_modbus(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i];
        for (int j = 0; j < 8; j++) {
            if ((crc & 0x0001) != 0) { crc >>= 1; crc ^= 0xA001; }
            else                          { crc >>= 1; }
        }
    }
    return crc;
}

/* ======================== 常量定义 ======================== */
#define SLAVE_ADDR       0x01
#define FUNC_READ_HOLD   0x03
#define REG_START_ADDR   0x0010
#define REG_TOTAL_COUNT  9       /* 9个寄存器 = 18字节数据 */
#define EXPECTED_RX_LEN  23      /* 01 03 12 [18B data] CRC CRC */

/* ======================== 串口初始化 ======================== */
static int uart_init(const char *dev_path) {
    int fd = open(dev_path, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) { perror("open uart"); return -1; }

    struct termios cfg = {0};
    tcgetattr(fd, &cfg);

    cfsetispeed(&cfg, B9600);
    cfsetospeed(&cfg, B9600);

    cfg.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);  /* 无校验、1停止位、无流控 */
    cfg.c_cflag |= CS8 | CREAD | CLOCAL;                   /* 8数据位、允许读 */

    cfg.c_iflag &= ~(IXON | IXOFF | IXANY | INLCR | ICRNL | IGNCR);
    cfg.c_oflag &= ~OPOST;
    cfg.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);       /* RAW模式 */
    cfg.c_cc[VMIN]  = 0;
    cfg.c_cc[VTIME] = 5;                                   /* 500ms超时 */

    if (tcsetattr(fd, TCSANOW, &cfg) != 0) { perror("tcsetattr"); close(fd); return -1; }

    /* 回读确认波特率 */
    struct termios actual;
    tcgetattr(fd, &actual);
    speed_t ispeed = cfgetispeed(&actual);
    printf("[UART] 设备=%s | 波特率=", dev_path);
    switch (ispeed) {
        case B4800:   printf("4800\n"); break;
        case B9600:   printf("9600\n"); break;
        case B19200:  printf("19200\n"); break;
        case B115200: printf("115200\n"); break;
        default:      printf("%ld(未知)\n", (long)ispeed); break;
    }

    tcflush(fd, TCIOFLUSH);
    return fd;
}

/* ======================== 单次收发 ======================== */

/**
 * do_one_query - 发一次请求, 收一次响应, 打印原始数据
 * @return: 0=OK, -1=超时无响应, -2=CRC错误
 */
static int do_one_query(int fd) {
    /* ---- 构造请求帧 ---- */
    uint8_t tx[8];
    tx[0] = SLAVE_ADDR;
    tx[1] = FUNC_READ_HOLD;
    tx[2] = (REG_START_ADDR >> 8) & 0xFF;
    tx[3] = REG_START_ADDR & 0xFF;
    tx[4] = (REG_TOTAL_COUNT >> 8) & 0xFF;
    tx[5] = REG_TOTAL_COUNT & 0xFF;

    uint16_t crc = crc16_modbus(tx, 6);
    tx[6] = crc & 0xFF;
    tx[7] = (crc >> 8) & 0xFF;

    printf("\nTX (%dB): ", 8);
    for (int i = 0; i < 8; i++) printf("%02X ", tx[i]);
    fflush(stdout);

    /* ---- 清缓冲区 ---- */
    tcflush(fd, TCIFLUSH);

    /* ---- 发送 ---- */
    ssize_t wlen = write(fd, tx, 8);
    fsync(fd);
    if (wlen != 8)
        fprintf(stderr, "\n[WARN] write 返回 %zd\n", wlen);

    /* ---- 等待从站响应 (给从站一点处理时间) ---- */
    usleep(30000);  /* 30ms 等从站准备数据 */

    /* ---- 接收: 有啥收啥, 不做帧头过滤 ---- */
    uint8_t rx[256] = {0};
    int total = 0;
    int retry = 0;
    const int max_retry = 100;

    while (total < EXPECTED_RX_LEN && retry < max_retry) {
        ssize_t n = read(fd, rx + total, EXPECTED_RX_LEN - total);
        if (n > 0) {
            total += n;
        }
        retry++;
        if (n <= 0) usleep(20000);
    }

    /* ---- 打印接收原始数据 ---- */
    printf("\nRX (%dB): ", total);
    for (int i = 0; i < total; i++) printf("%02X ", rx[i]);

    if (total <= 0) return -1;

    /* ---- CRC校验 ---- */
    if (total >= EXPECTED_RX_LEN) {
        uint16_t calc_crc = crc16_modbus(rx, (uint16_t)(total - 2));
        uint16_t recv_crc = (uint16_t)(rx[total - 2] | (rx[total - 1] << 8));
        if (calc_crc != recv_crc) {
            printf("| CRC错 calc=0x%04X recv=0x%04X\n", calc_crc, recv_crc);
            return -2;
        }
        printf("| CRC OK\n");
    } else {
        printf("| 帧不完整\n");
        return -1;
    }

    /* ---- 解析显示 (如果CRC通过) ---- */
    const uint8_t *d = &rx[3];

    /* ADC Value - int32 LE */
    {
        int32_t v = (int32_t)((uint32_t)d[0] | ((uint32_t)d[1]<<8) |
                             ((uint32_t)d[2]<<16) | ((uint32_t)d[3]<<24));
        printf("  ADC_Value     = 0x%08X (%d)\n", (uint32_t)v, v);
    }
    /*
     EmptyLoad - int32 LE */
    {
        int32_t v = (int32_t)((uint32_t)d[4] | ((uint32_t)d[5]<<8) |
                             ((uint32_t)d[6]<<16) | ((uint32_t)d[7]<<24));
        printf("  EmptyLoad     = 0x%08X (%d)\n", (uint32_t)v, v);
    
    }
    /* SCALE1 - float LE */
    {
        uint32_t raw = (uint32_t)d[8] | ((uint32_t)d[9]<<8) |
                       ((uint32_t)d[10]<<16) | ((uint32_t)d[11]<<24);
        float fv; memcpy(&fv, &raw, sizeof(fv));
        printf("  SCALE1        = %.4f\n", fv);
    }
    /* WEIGHT - int32 LE */
    {
        int32_t v = (int32_t)((uint32_t)d[12] | ((uint32_t)d[13]<<8) |
                             ((uint32_t)d[14]<<16) | ((uint32_t)d[15]<<24));
        printf("  WEIGHT        = %d g\n", v);
    }
    /* Status - uint16 BE */
    {
        uint16_t v = (uint16_t)((uint16_t)(d[16]<<8) | d[17]);
        printf("  ContAndStatus = 0x%04X\n", v);
    }

    return 0;
}

/* ======================== 实时模式 ======================== */

/**
 * do_realtime_loop - 实时循环读取, 原地刷新显示
 */
static void do_realtime_loop(int fd) {
    int cycle = 0;

    printf("\n[实时模式] 正在读取... (Ctrl+C 停止)\n");

    while (g_run) {
        /* ---- 构造请求帧 ---- */
        uint8_t tx[8];
        tx[0] = SLAVE_ADDR;
        tx[1] = FUNC_READ_HOLD;
        tx[2] = (REG_START_ADDR >> 8) & 0xFF;
        tx[3] = REG_START_ADDR & 0xFF;
        tx[4] = (REG_TOTAL_COUNT >> 8) & 0xFF;
        tx[5] = REG_TOTAL_COUNT & 0xFF;

        uint16_t crc = crc16_modbus(tx, 6);
        tx[6] = crc & 0xFF;
        tx[7] = (crc >> 8) & 0xFF;

        tcflush(fd, TCIFLUSH);
        write(fd, tx, 8);
        fsync(fd);

        usleep(30000);

        /* ---- 接收响应 ---- */
        uint8_t rx[256] = {0};
        int total = 0;
        int retry = 0;

        while (total < EXPECTED_RX_LEN && retry < 100) {
            ssize_t n = read(fd, rx + total, EXPECTED_RX_LEN - total);
            if (n > 0) total += n;
            retry++;
            if (n <= 0) usleep(20000);
        }

        ++cycle;

        if (total >= EXPECTED_RX_LEN) {
            uint16_t calc_crc = crc16_modbus(rx, (uint16_t)(total - 2));
            uint16_t recv_crc = (uint16_t)(rx[total - 2] | (rx[total - 1] << 8));

            if (calc_crc == recv_crc) {
                const uint8_t *d = &rx[3];

                int32_t adc   = (int32_t)((uint32_t)d[0]  | ((uint32_t)d[1]<<8)  |
                                          ((uint32_t)d[2]<<16) | ((uint32_t)d[3]<<24));
                int32_t empty = (int32_t)((uint32_t)d[4]  | ((uint32_t)d[5]<<8)  |
                                          ((uint32_t)d[6]<<16) | ((uint32_t)d[7]<<24));
                uint32_t sraw = (uint32_t)d[8]  | ((uint32_t)d[9]<<8)  |
                               ((uint32_t)d[10]<<16) | ((uint32_t)d[11]<<24);
                float scale; memcpy(&scale, &sraw, sizeof(scale));
                int32_t weight= (int32_t)((uint32_t)d[12] | ((uint32_t)d[13]<<8)  |
                                          ((uint32_t)d[14]<<16) | ((uint32_t)d[15]<<24));
                uint16_t status=(uint16_t)((uint16_t)(d[16]<<8) | d[17]);

                printf("\r[%04d] ADC=%d  Empty=%d  SCALE=%.4f  WEIGHT=%d g  Status=0x%04X   ",
                       cycle, adc, empty, scale, weight, status);
                fflush(stdout);
            } else {
                printf("\r[%04d] CRC错误                                    ", cycle);
                fflush(stdout);
            }
        } else {
            printf("\r[%04d] 无响应/帧不完整                              ", cycle);
            fflush(stdout);
        }

        usleep(200000);  /* 200ms 间隔 */
    }

    printf("\n\n实时模式退出, 共 %d 次查询\n", cycle);
}

/* 设置终端为非规范模式 (用于检测回车键按下) */
static void set_raw_mode(int fd_stdin, struct termios *orig_attr) {
    struct termios new_attr;
    tcgetattr(fd_stdin, orig_attr);
    new_attr = *orig_attr;
    new_attr.c_lflag &= ~(ICANON | ECHO);  /* 非标准输入、关闭回显 */
    new_attr.c_cc[VMIN] = 0;
    new_attr.c_cc[VTIME] = 0;
    tcsetattr(fd_stdin, TCSANOW, &new_attr);
}

static void restore_mode(int fd_stdin, struct termios *orig_attr) {
    tcsetattr(fd_stdin, TCSANOW, orig_attr);
}

/* ======================== 主程序 ======================== */
int main(int argc, char **argv) {
    const char *uart_dev = "/dev/ttyAMA0";
    const char *mode     = "manual";  /* 默认手动模式 */

    /* 解析参数:  [串口] [模式] */
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "manual") == 0 || strcmp(argv[i], "realtime") == 0)
            mode = argv[i];
        else
            uart_dev = argv[i];
    }

    signal(SIGINT, on_sigint);

    int fd = uart_init(uart_dev);
    if (fd < 0) {
        fprintf(stderr, "无法打开串口 %s\n", uart_dev);
        return EXIT_FAILURE;
    }

    printf("========================================\n");
    printf("  Modbus 称重调试工具\n");
    printf("  串口: %s @ 9600bps 8N1\n", uart_dev);
    printf("  模式: %s\n", mode);
    printf("  退出: Ctrl+C\n");
    printf("========================================\n");

    if (strcmp(mode, "realtime") == 0) {
        do_realtime_loop(fd);
    } else {
        /* ---- 手动触发模式 ---- */
        struct termios orig_stdin;
        set_raw_mode(STDIN_FILENO, &orig_stdin);

        int cycle = 0;
        printf("  操作: 按 Enter 发送一次查询\n");

        while (g_run) {
            char buf[64];
            ssize_t nr = read(STDIN_FILENO, buf, sizeof(buf));
            if (nr > 0 && (buf[0] == '\n' || buf[0] == '\r')) {
                printf("----- 第 %04d 次 -----", ++cycle);
                do_one_query(fd);
                printf("\n按 Enter 继续...\n");
            }
            usleep(10000);
        }

        restore_mode(STDIN_FILENO, &orig_stdin);
        printf("\n退出, 共完成 %d 次查询\n", cycle);
    }

    close(fd);
    return EXIT_SUCCESS;
}
