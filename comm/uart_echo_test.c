/**
 * ============================================================================
 * @file    uart_echo_test.c
 * @brief   Modbus RTU 从站模拟器 - 模拟烧录设备, 支持连接测试/读写SN/擦除SN
 *
 * 协议规则:
 *   1. 连接测试:   FC=03, 读 0x0000, len=1  → 返回 0xABCD
 *   2. 读取SN:     FC=03, 读 0x0002, len=4  → 返回8字节SN (或全FF)
 *   3. 写入SN:     FC=10, 写 0x0002, len=4  → ACK + 更新本地SN
 *   4. 擦除SN:     FC=06, 写 0x0001, val=0x00FF → 清空SN + 原样回传
 *
 * 编译: gcc -Wall -Wextra -o uart_echo_test uart_echo_test.c
 * 运行: sudo ./uart_echo_test [/dev/ttyAMA0] [9600]    Ctrl+C 退出
 *
 * @author  AI Assistant
 * @date    2026-06-01
 * ============================================================================
 */

#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdint.h>
#include <errno.h>
#include <signal.h>
#include <time.h>

/* ======================== 全局状态 ======================== */
static volatile sig_atomic_t g_run = 1;
static uint8_t g_sn[8];          /* 模拟序列号, 初始为空(全0xFF) */
static int g_sn_valid = 0;       /* SN是否已写入(非空) */
static int g_pkt_cnt = 0;

#define SLAVE_ADDR      0x01

/* 寄存器地址定义 */
#define REG_CONN_TEST   0x0000
#define REG_ERASE_CMD   0x0001
#define REG_SN_START    0x0002

/* 默认SN值 (示例) */
#define DEFAULT_SN { 0x00, 0x04, 0xB2, 0x37, 0xC5, 0x13, 0x98, 0xC1 }

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

static void append_crc(uint8_t *buf, int data_len) {
    uint16_t crc = crc16_modbus(buf, (uint16_t)data_len);
    buf[data_len]     = crc & 0xFF;
    buf[data_len + 1] = (crc >> 8) & 0xFF;
}

static int check_crc(const uint8_t *buf, int total_len) {
    if (total_len < 4) return 0;
    uint16_t calc = crc16_modbus(buf, (uint16_t)(total_len - 2));
    uint16_t recv = (uint16_t)(buf[total_len - 2] | (buf[total_len - 1] << 8));
    return calc == recv ? 1 : 0;
}

/* ======================== 串口初始化 ======================== */
static int uart_init(const char *dev_path, int baud) {
    int fd = open(dev_path, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) { perror("open uart"); return -1; }

    struct termios cfg = {0};
    tcgetattr(fd, &cfg);
    cfsetispeed(&cfg, baud);
    cfsetospeed(&cfg, baud);

    cfg.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
    cfg.c_cflag |= CS8 | CREAD | CLOCAL;
    cfg.c_iflag &= ~(IXON | IXOFF | IXANY | INLCR | ICRNL | IGNCR);
    cfg.c_oflag &= ~OPOST;
    cfg.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    cfg.c_cc[VMIN]  = 0;
    cfg.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &cfg) != 0) {
        perror("tcsetattr"); close(fd); return -1;
    }
    tcflush(fd, TCIOFLUSH);
    return fd;
}

static const char* baud_name(speed_t baud) {
    switch (baud) {
        case B4800:   return "4800";
        case B9600:   return "9600";
        case B19200:  return "19200";
        case B38400:  return "38400";
        case B57600:  return "57600";
        case B115200: return "115200";
        default:      return "未知";
    }
}

static speed_t str_to_baud(const char *s) {
    if (!s) return B9600;
    switch (atoi(s)) {
        case 4800:   return B4800;
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        default:     return B9600;
    }
}

/* ======================== 打印辅助 ======================== */
static void print_hex(const uint8_t *data, int len, const char *prefix) {
    printf("%s", prefix);
    for (int i = 0; i < len; i++) printf("%02X ", data[i]);
}

static void print_sn(void) {
    printf("SN=");
    for (int i = 0; i < 8; i++) printf("%02X", g_sn[i]);
    printf(" (%s)", g_sn_valid ? "已写入" : "空/擦除");
}

/* ======================== Modbus 响应处理 ======================== */

/**
 * 处理功能码 03 - 读保持寄存器
 * 返回响应帧长度 (含CRC), 0表示异常
 */
static int handle_fc03(const uint8_t *rx, int rx_len, uint8_t *tx) {
    (void)rx_len;
    uint16_t reg_addr  = (uint16_t)((rx[2] << 8) | rx[3]);
    uint16_t reg_count = (uint16_t)((rx[4] << 8) | rx[5]);

    tx[0] = SLAVE_ADDR;
    tx[1] = 0x03;

    /* --- 连接测试: 读 0x0000, len=1 --- */
    if (reg_addr == REG_CONN_TEST && reg_count == 1) {
        tx[2] = 2;              /* 数据长度 2字节 */
        tx[3] = 0xAB;
        tx[4] = 0xCD;
        append_crc(tx, 5);
        print_hex(tx, 7, "[FC03 ConnTest] TX:");
        printf("\n");
        return 7;
    }

    /* --- 读取SN: 读 0x0002, len=4 --- */
    if (reg_addr == REG_SN_START && reg_count == 4) {
        tx[2] = 8;              /* 数据长度 8字节 */
        memcpy(&tx[3], g_sn, 8); /* 当前SN或全FF */
        append_crc(tx, 11);
        print_hex(tx, 13, "[FC03 ReadSN]   TX:");
        printf(" ");
        print_sn();
        printf("\n");
        return 13;
    }

    /* 不支持的地址 -> 异常响应 */
    tx[1] = 0x83;               /* 异常功能码 */
    tx[2] = 0x02;               /* 非法数据地址 */
    append_crc(tx, 3);
    print_hex(tx, 5, "[FC03 Error]    TX: Illegal address\n");
    return 5;
}

/**
 * 处理功能码 06 - 写单个寄存器
 * 返回响应帧长度 (含CRC), 0表示异常
 */
static int handle_fc06(const uint8_t *rx, int rx_len, uint8_t *tx) {
    (void)rx_len;
    uint16_t reg_addr = (uint16_t)((rx[2] << 8) | rx[3]);
    uint16_t value    = (uint16_t)((rx[4] << 8) | rx[5]);

    /* --- 擦除命令: 写 0x0001, val=0xFFFF --- */
    if (reg_addr == REG_ERASE_CMD && value == 0x00FF) {
        memset(g_sn, 0xFF, sizeof(g_sn));
        g_sn_valid = 0;
        /* 原样回传请求报文 */
        memcpy(tx, rx, 8);      /* 含CRC原样返回 */
        print_hex(tx, 8, "[FC06 Erase]     TX:");
        printf(" ");
        print_sn();
        printf("\n");
        return 8;
    }

    /* 其他写操作不支持 -> 异常 */
    tx[0] = SLAVE_ADDR;
    tx[1] = 0x86;
    tx[2] = 0x02;
    append_crc(tx, 3);
    print_hex(tx, 5, "[FC06 Error]    TX: Not erase command\n");
    return 5;
}

/**
 * 处理功能码 10 (0x10) - 写多个寄存器
 * 返回响应帧长度 (含CRC), 0表示异常
 */
static int handle_fc10(const uint8_t *rx, int rx_len, uint8_t *tx) {
    uint16_t reg_addr  = (uint16_t)((rx[2] << 8) | rx[3]);
    uint16_t reg_count = (uint16_t)((rx[4] << 8) | rx[5]);
    uint8_t  byte_cnt  = rx[6];

    /* --- 写入SN: 地址 0x0002, 长度 4 (8字节) --- */
    if (reg_addr == REG_SN_START && reg_count == 4 && byte_cnt == 8 && rx_len >= 17) {
        memcpy(g_sn, &rx[7], 8);  /* 提取8字节数据写入SN */
        g_sn_valid = 1;

        /* ACK响应: 回传地址+数量+CRC */
        tx[0] = SLAVE_ADDR;
        tx[1] = 0x10;
        tx[2] = (reg_addr >> 8) & 0xFF;
        tx[3] = reg_addr & 0xFF;
        tx[4] = (reg_count >> 8) & 0xFF;
        tx[5] = reg_count & 0xFF;
        append_crc(tx, 6);

        print_hex(rx, rx_len, "[FC10 WriteSN]   RX:");
        printf(" | ");
        print_sn();
        print_hex(tx, 8, "\n                  TX: ACK");
        printf("\n");
        return 8;
    }

    /* 不支持 -> 异常 */
    tx[0] = SLAVE_ADDR;
    tx[1] = 0x90;
    tx[2] = 0x02;
    append_crc(tx, 3);
    print_hex(tx, 5, "[FC10 Error]    TX: Illegal address\n");
    return 5;
}

/**
 * process_frame - 解析并处理一帧完整的Modbus RTU请求
 * @return: 响应帧长度 (含CRC), 0=不响应/错误
 */
static int process_frame(const uint8_t *frame, int frame_len, uint8_t *resp) {
    /* 最小长度检查: addr + func + 至少2字节 + 2CRC = 6 */
    if (frame_len < 6) return 0;

    /* 地址匹配 */
    if (frame[0] != SLAVE_ADDR) return 0;

    /* CRC 校验 */
    if (!check_crc(frame, frame_len)) {
        printf("[ERR] CRC错误\n");
        return 0;
    }

    uint8_t fc = frame[1];

    switch (fc) {
        case 0x03:
            return handle_fc03(frame, frame_len, resp);
        case 0x06:
            return handle_fc06(frame, frame_len, resp);
        case 0x10:
            return handle_fc10(frame, frame_len, resp);
        default:
            /* 未知功能码 -> 异常响应 */
            resp[0] = SLAVE_ADDR;
            resp[1] = fc | 0x80;
            resp[2] = 0x01;  /* 非法功能码 */
            append_crc(resp, 3);
            print_hex(resp, 5, "[Error]          TX: Unknown FC\n");
            return 5;
    }
}

/* ======================== 主循环 ======================== */
int main(int argc, char **argv) {
    const char *dev_path = "/dev/ttyAMA0";
    const char *baud_str = "9600";

    for (int i = 1; i < argc; i++) {
        dev_path = argv[i];
        if (i + 1 < argc && argv[i + 1][0] != '/')
            baud_str = argv[++i];
    }

    speed_t baud = str_to_baud(baud_str);
    signal(SIGINT, on_sigint);

    int fd = uart_init(dev_path, baud);
    if (fd < 0) {
        fprintf(stderr, "无法打开串口 %s\n", dev_path);
        return EXIT_FAILURE;
    }

    /* 初始化默认SN (全FF = 空/未烧录) */
    memset(g_sn, 0xFF, sizeof(g_sn));
    g_sn_valid = 0;

    printf("========================================\n");
    printf("  Modbus RTU 从站模拟器\n");
    printf("  设备: %s @ %sbps 8N1\n", dev_path, baud_name(baud));
    printf("  从站地址: 0x%02X\n", SLAVE_ADDR);
    printf("  支持: FC03(读) / FC06(写单) / FC10(写多)\n");
    printf("  ");
    print_sn();
    printf("\n  退出: Ctrl+C\n");
    printf("========================================\n");

    uint8_t rx_buf[256];
    uint8_t tx_buf[256];
    int rx_pos = 0;
    struct timespec last_rx_ts = {0};

    while (g_run) {
        ssize_t n = read(fd, &rx_buf[rx_pos], sizeof(rx_buf) - (size_t)rx_pos);
        if (n > 0) {
            clock_gettime(CLOCK_MONOTONIC, &last_rx_ts);
            rx_pos += n;

            /* 尝试解析完整帧 (最小帧长判断) */
            while (rx_pos >= 6) {
                /* 先找帧头: 必须是本从站地址 */
                if (rx_buf[0] != SLAVE_ADDR) {
                    /* 跳过非目标地址字节 */
                    memmove(rx_buf, rx_buf + 1, (size_t)(--rx_pos));
                    continue;
                }

                /* 根据功能码估算最小帧长来决定是否可以尝试解析 */
                uint8_t fc = rx_buf[1];
                int min_len = 0;
                switch (fc) {
                    case 0x03: min_len = 8; break;  /* 1+1+2+2+2crc */
                    case 0x06: min_len = 8; break;  /* 1+1+2+2+2crc */
                    case 0x10: {
                        if (rx_pos >= 7)
                            min_len = 9 + rx_buf[6];  /* hdr(7) + data + 2crc */
                        else
                            min_len = 256;             /* 等更多数据 */
                        break;
                    }
                    default:   min_len = 8; break;     /* 未知FC,按最短处理 */
                }

                if (rx_pos < min_len) break;  /* 等更多数据 */

                /* 有足够数据, 尝试处理 */
                ++g_pkt_cnt;
                printf("\n[%04d RX %dB] ", g_pkt_cnt, rx_pos);
                for (int i = 0; i < rx_pos; i++)
                    printf("%02X ", rx_buf[i]);

                int tx_len = process_frame(rx_buf, rx_pos, tx_buf);

                if (tx_len > 0) {
                    ssize_t wn = write(fd, tx_buf, (size_t)tx_len);
                    fsync(fd);
                    printf("[TX %zdB]\n", wn);
                } else {
                    printf("[无响应]\n");
                }

                /* 清缓冲区准备下一帧 */
                rx_pos = 0;
            }
        } else if (n < 0 && errno != EAGAIN) {
            perror("read error");
            break;
        }

        /* 帧超时检测: 如果有残留数据且超过50ms没收到新数据, 清掉残帧 */
        if (rx_pos > 0) {
            struct timespec now;
            clock_gettime(CLOCK_MONOTONIC, &now);
            long elapsed_ms = (now.tv_sec - last_rx_ts.tv_sec) * 1000 +
                              (now.tv_nsec - last_rx_ts.tv_nsec) / 1000000L;
            if (elapsed_ms > 50) {
                /* 残帧丢弃 */
                printf("[WARN] 丢弃残帧 %dB\n", rx_pos);
                rx_pos = 0;
            }
        }

        usleep(3000);  /* 3ms 轮询间隔 */
    }

    printf("\n\n退出, 共处理 %d 个请求包\n", g_pkt_cnt);
    close(fd);
    return EXIT_SUCCESS;
}
