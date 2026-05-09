/**
 * ============================================================================
 * @file    feigong_scale.c
 * @brief   费工电子秤 Modbus RTU — 称重/去皮/校准
 *
 *  寄存器映射:
 *    0x0000-0x0001 Net_Weight  R  int32   净重(g)
 *    0x0002        Status_Word R  uint16  状态字(bit0:稳定 bit1:过载 bit2:负重 bit3:去皮)
 *    0x0003-0x0004 ADC_Raw     R  int32   ADC原始值
 *    0x0010        System_Cmd  RW uint16  命令(1:去皮,2:校准)
 *    0x0020-0x0021 Cal_Factor  RW int32   校准系数K
 *
 *  编译: gcc -Wall -Wextra -o feigong_scale feigong_scale.c
 *  运行: sudo ./feigong_scale [/dev/ttyAMA0]
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

/* ======================== 全局控制 ======================== */
static volatile sig_atomic_t g_run = 1;

/* ======================== Modbus CRC-16 ======================== */
static uint16_t crc16_modbus(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i];
        for (int j = 0; j < 8; j++)
            crc = (crc & 0x0001) ? (crc >> 1) ^ 0xA001 : crc >> 1;
    }
    return crc;
}

/* ======================== 大端序转换 ======================== */
static int32_t be_to_int32(const uint8_t *p) {
    return (int32_t)((uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 |
                     (uint32_t)p[2] << 8 | (uint32_t)p[3]);
}
static uint16_t be_to_uint16(const uint8_t *p) {
    return ((uint16_t)p[0] << 8) | p[1];
}

/* ======================== 常量定义 ======================== */
#define SLAVE_ADDR      0x01
#define FUNC_READ       0x03
#define FUNC_WRITE      0x06

/* 寄存器地址 */
#define REG_WEIGHT       0x0000   /* Net_Weight (int32, 2个寄存器) */
#define REG_STATUS       0x0002   /* Status (uint16, 1个寄存器) */
#define REG_ADC          0x0003   /* ADC_Raw (int32, 2个寄存器) */
#define REG_CMD          0x0010   /* System_Cmd (uint16) */
#define REG_CAL          0x0020   /* Cal_Factor_K (int32, 2个寄存器) */

/* 命令值 */
#define CMD_TARE         1
#define CMD_CALIBRATE    2

/* ======================== 信号处理 ======================== */
static void on_sigint(int sig) { (void)sig; g_run = 0; }

/* ======================== 串口初始化 (9600 8N1) ======================== */
static int uart_init(const char *dev) {
    int fd = open(dev, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) { perror("open uart"); return -1; }

    struct termios cfg = {0};
    tcgetattr(fd, &cfg);

    cfsetispeed(&cfg, B9600);
    cfsetospeed(&cfg, B9600);

    cfg.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
    cfg.c_cflag |= CS8 | CREAD | CLOCAL;
    cfg.c_iflag &= ~(IXON | IXOFF | IXANY | INLCR | ICRNL | IGNCR);
    cfg.c_oflag &= ~OPOST;
    cfg.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    cfg.c_cc[VMIN]  = 0;
    cfg.c_cc[VTIME] = 10;  /* 1.0s 读超时（加大，避免设备慢响应时截断） */

    if (tcsetattr(fd, TCSANOW, &cfg) != 0) { perror("tcsetattr"); close(fd); return -1; }
    tcflush(fd, TCIOFLUSH);
    return fd;
}

/* ======================== Modbus FC03 读保持寄存器 ======================== */

/**
 * @return  0=OK  -1=无响应/长度不足  -2=CRC错  -3=异常响应码
 *
 * 稳定性措施:
 *   1. 发送前 tcflush 清空输入缓冲区
 *   2. 帧头校验：首字节必须是从站地址(01)
 *   3. 失败自动重试（最多2次），重试期间静默
 *   4. 只在最终结果时统一打印 TX/RX
 */
static int modbus_read(int fd, uint16_t reg_addr, uint16_t reg_count,
                       const uint8_t **out_data, int *out_dlen) {
    /* 构造请求帧 */
    uint8_t tx[8] = {
        SLAVE_ADDR, FUNC_READ,
        (reg_addr >> 8) & 0xFF, reg_addr & 0xFF,
        (reg_count >> 8) & 0xFF, reg_count & 0xFF,
        0, 0
    };
    uint16_t crc = crc16_modbus(tx, 6);
    tx[6] = crc & 0xFF;
    tx[7] = (crc >> 8) & 0xFF;

    static uint8_t rx[64];  /* static 以便返回指针有效 */
    int final_rlen = 0;
    int final_ret = -1;
    int attempts = 0;

    /* 重试: 静默执行，只在最后打印一次 */
    for (int attempt = 1; attempt <= 2; attempt++) {
        tcflush(fd, TCIFLUSH);

        if (write(fd, tx, 8) != 8) return -1;
        fsync(fd);
        usleep(attempt == 1 ? 100000 : 200000);

        memset(rx, 0, sizeof(rx));
        int rlen = (int)read(fd, rx, sizeof(rx));

        if (rlen <= 0) { if (attempt < 2) continue; break; }

        /* 帧头校验 */
        if (rx[0] != SLAVE_ADDR) { if (attempt < 2) continue; final_rlen = rlen; break; }

        int expect_len = 3 + (int)reg_count * 2 + 2;
        if (rlen < expect_len) { if (attempt < 2) continue; final_rlen = rlen; break; }

        /* CRC 校验 */
        uint16_t calc_crc = crc16_modbus(rx, (uint16_t)(expect_len - 2));
        uint16_t recv_crc = rx[expect_len - 2] | (rx[expect_len - 1] << 8);
        if (calc_crc != recv_crc) { if (attempt < 2) continue; final_rlen = rlen; final_ret = -2; break; }

        /* 异常检查 */
        if (rx[1] & 0x80) {
            final_rlen = rlen;
            final_ret = -3;
            break;
        }

        /* 成功 */
        final_rlen = rlen;
        final_ret = 0;
        attempts = attempt;
        break;
    }

    /* ===== 统一打印结果 ===== */
    printf("  TX: ");
    for (int i = 0; i < 8; i++) printf("%02X ", tx[i]);
    if (attempts > 1) printf("(第%d次)", attempts);
    printf("\n");

    if (final_rlen <= 0) {
        printf("  RX: 无响应\n");
        return -1;
    }
    printf("  RX(%dB): ", final_rlen);
    for (int i = 0; i < final_rlen; i++) printf("%02X ", rx[i]);
    printf("\n");

    if (final_ret == -2) {
        int el = 3 + (int)reg_count * 2 + 2;
        uint16_t cc = crc16_modbus(rx, (uint16_t)(el - 2));
        uint16_t rc = rx[el - 2] | (rx[el - 1] << 8);
        printf("  CRC错误: calc=0x%04X recv=0x%04X\n", cc, rc);
        return -2;
    }
    if (final_ret == -3) {
        printf("  异常响应: 错误码=0x%02X\n", rx[2]);
        return -3;
    }
    if (rx[0] != SLAVE_ADDR) {
        printf("  帧头异常(非从站地址)\n");
        return -1;
    }
    if (final_rlen < 3 + (int)reg_count * 2 + 2) {
        printf("  数据不足\n");
        return -1;
    }

    *out_data = &rx[3];
    *out_dlen = rx[2];
    return 0;
}

/* ======================== Modbus FC06 写单个寄存器 ======================== */

/**
 * 静默重试 + 最终统一打印
 */
static int modbus_write(int fd, uint16_t reg_addr, uint16_t value) {
    uint8_t tx[8] = {
        SLAVE_ADDR, FUNC_WRITE,
        (reg_addr >> 8) & 0xFF, reg_addr & 0xFF,
        (value >> 8) & 0xFF, value & 0xFF,
        0, 0
    };
    uint16_t crc = crc16_modbus(tx, 6);
    tx[6] = crc & 0xFF;
    tx[7] = (crc >> 8) & 0xFF;

    uint8_t rx[32] = {0};
    int final_rlen = 0;
    int final_ret = -1;
    int attempts = 0;

    /* 静默重试 */
    for (int attempt = 1; attempt <= 2; attempt++) {
        tcflush(fd, TCIFLUSH);

        if (write(fd, tx, 8) != 8) return -1;
        fsync(fd);
        usleep(attempt == 1 ? 100000 : 200000);

        memset(rx, 0, sizeof(rx));
        int rlen = (int)read(fd, rx, sizeof(rx));

        if (rlen <= 0) { if (attempt < 2) continue; break; }
        if (rx[0] != SLAVE_ADDR) { if (attempt < 2) continue; final_rlen = rlen; break; }

        /* 从站返回03数据帧 */
        if (rlen >= 15 && rx[1] == FUNC_READ) {
            uint16_t c = crc16_modbus(rx, 13);
            uint16_t r = rx[13] | (rx[14] << 8);
            if (c == r) { final_rlen = rlen; final_ret = 0; attempts = attempt; break; }
            if (attempt < 2) continue;
            final_rlen = rlen; final_ret = -2; break;
        }

        /* 标准06回显帧 */
        if (rlen < 8) { if (attempt < 2) continue; final_rlen = rlen; break; }

        uint16_t cc = crc16_modbus(rx, 6), rc = rx[6] | (rx[7] << 8);
        if (cc != rc) { if (attempt < 2) continue; final_rlen = rlen; final_ret = -2; break; }
        if (rx[1] & 0x80) { final_rlen = rlen; final_ret = -3; break; }

        final_rlen = rlen;
        final_ret = 0;
        attempts = attempt;
        break;
    }

    /* ===== 统一打印结果 ===== */
    printf("  TX: ");
    for (int i = 0; i < 8; i++) printf("%02X ", tx[i]);
    if (attempts > 1) printf("(第%d次)", attempts);
    printf("\n");

    if (final_rlen <= 0) {
        printf("  RX: 无响应\n");
        return -1;
    }
    printf("  RX(%dB): ", final_rlen);
    for (int i = 0; i < final_rlen; i++) printf("%02X ", rx[i]);
    printf("\n");

    if (final_ret == -2) { printf("  CRC错误\n"); return -2; }
    if (final_ret == -3) { printf("  异常响应\n"); return -3; }
    if (rx[0] != SLAVE_ADDR) { printf("  帧头异常\n"); return -1; }
    if (final_rlen < 8) { printf("  响应长度不足\n"); return -1; }

    printf("  [OK] 从站确认成功\n");
    return 0;
}

/* ======================== 业务函数 ======================== */

/**
 * [1] 模拟称重 — 只读净重+状态+ADC (0x0000~0x0004, 5个寄存器)
 */
static int do_weighing(int fd) {
    const uint8_t *d; int dlen;

    int r = modbus_read(fd, REG_WEIGHT, 5, &d, &dlen);
    if (r != 0) return r;

    int32_t weight  = be_to_int32(d + 0);          /* Net_Weight */
    uint16_t status = be_to_uint16(d + 4);          /* Status */
    int32_t adc     = be_to_int32(d + 6);          /* ADC_Raw */

    printf("[净重:%d g | ADC:%d | 状态:0x%04X [%s%s%s%s]]\n",
           weight, adc, status,
           (status & 0x01) ? "稳定" : "波动",
           (status & 0x02) ? " 过载" : "",
           (status & 0x04) ? " 负重" : "",
           (status & 0x08) ? " 去皮" : "");
    return 0;
}

/**
 * [2] 读 System_Cmd (0x0010, 1个寄存器)
 */
static void do_read_cmd(int fd) {
    printf(">>> 读取 System_Cmd ...\n");
    const uint8_t *d; int dlen;
    int r = modbus_read(fd, REG_CMD, 1, &d, &dlen);
    if (r == 0)
        printf("  System_Cmd = %d\n\n", be_to_uint16(d));
}

/**
 * [3] 读 Cal_Factor_K (0x0020~0x0021, 2个寄存器→int32)
 */
static void do_read_cal_k(int fd) {
    printf(">>> 读取 Cal_Factor_K ...\n");
    const uint8_t *d; int dlen;
    int r = modbus_read(fd, REG_CAL, 2, &d, &dlen);
    if (r == 0)
        printf("  Cal_Factor_K = %d (0x%08X)\n\n", be_to_int32(d), (uint32_t)be_to_int32(d));
}

/**
 * [4] 去皮 — 写 System_Cmd = 1
 */
static void do_tare(int fd) {
    printf(">>> 执行去皮...\n");
    modbus_write(fd, REG_CMD, CMD_TARE);
}

/**
 * [5] 校准 — 写 System_Cmd = 2
 */
static void do_calibrate(int fd) {
    printf(">>> 执行校准...\n");
    modbus_write(fd, REG_CMD, CMD_CALIBRATE);
}

/* ======================== 主程序 ======================== */

static void print_menu(void) {
    printf("\n========================================\n");
    printf("  飞功电子秤 Modbus RTU V2.1\n");
    printf("  [1] 模拟称重 (持续读取重量)\n");
    printf("  [2] 读 System_Cmd\n");
    printf("  [3] 读 Cal_Factor_K\n");
    printf("  [4] 去皮 (Tare)\n");
    printf("  [5] 校准 (Calibrate)\n");
    printf("  [q] 退出\n");
    printf("========================================\n");
    printf("> ");
}

int main(int argc, char **argv) {
    const char *dev = (argc > 1) ? argv[1] : "/dev/ttyAMA0";

    signal(SIGINT, on_sigint);

    int fd = uart_init(dev);
    if (fd < 0) { fprintf(stderr, "无法打开串口 %s\n", dev); return EXIT_FAILURE; }

    printf("\n========================================\n");
    printf("  串口: %s @ 9600bps 8N1 | 从站: 0x%02X\n",
           dev, SLAVE_ADDR);
    printf("========================================\n");

    char line[64];

    while (g_run) {
        print_menu();
        if (!fgets(line, sizeof(line), stdin)) break;

        switch (line[0]) {

        case '1': {
            printf("[持续称重中... Ctrl+C 返回菜单]\n");
            while (g_run) {
                if (do_weighing(fd) != 0) { usleep(3000000); continue; }
                usleep(3000000);
            }
            g_run = 1;
            break;
        }

        case '2':
            do_read_cmd(fd);
            break;

        case '3':
            do_read_cal_k(fd);
            break;

        case '4':
            do_tare(fd);
            break;

        case '5':
            do_calibrate(fd);
            break;

        case 'q': case 'Q':
            goto done;

        default:
            printf("无效选项，请输入 1-5 或 q\n");
        }
    }

done:
    printf("退出程序\n");
    close(fd);
    return EXIT_SUCCESS;
}
