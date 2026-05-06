/**
 * ============================================================================
 * @file    modbus_poll_weight.c
 * @brief   Modbus RTU 轮询读取万工称重传感器 HX711 全部参数
 *
 * 功能说明：
 *   通过 RS485 串口，以 Modbus RTU 功能码0x03一次性读取从站全部参数。
 *   从站返回16字节数据，按小端序(LE)解析为以下5个字段：
 *
 *     偏移  字节数  类型        字段名              打印格式示例
 *     +0     4      int32 LE    ADC Value           0x0835601
 *     +4     4      int32 LE    EmptyLoad_Value     0x0800601
 *     +8     4      float LE    SCALE1              214.5263
 *    +12     2      int16 LE    WEIGHT               125
 *    +14     2      uint16 LE   ContAndStatus       0x0003
 *
 * 实际响应帧示例（原始字节）：
 *   TX: 01 03 00 10 00 08 CRC CRC
 *   RX: 01 03 10 01 56 83 00 01 06 80 00 BC 86 56 43 00 7D 00 03 AF 1A
 *                     |--ADC(LE)-| |--Empty(LE)-| |-SCALE1(LE)-| |WGT| |STS|
 *
 * 通信参数：4800bps 8N1, 从站地址 0x01, 轮询间隔 1s/次
 * 编译： gcc -Wall -Wextra -o modbus_poll_weight modbus_poll_weight.c
 * 运行： sudo ./modbus_poll_weight [/dev/ttyAMA0]   Ctrl+C 退出
 *
 * @author  AI Assistant
 * @date    2026-05-06
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

/* ======================== 全局控制标志 ======================== */
static volatile sig_atomic_t g_run = 1;

/* ======================== Modbus CRC-16 (逐位移位法) ========================
 * 多项式: 0x8005, 初始值: 0xFFFF, 输出反转(异或 0xA001)
 * 参考: Modbus over Serial Line Specification V1.02 §6.5.1
 * ============================================================================ */
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

/* ======================== 小端序(LE) 数据类型转换 ========================
 * 万工模块响应数据为小端序(Little-Endian): 低字节在前，高字节在后
 * 例如原始字节 [01 56 83 00] 解析为 int32 = 0x00835601
 * ============================================================================ */

/**
 * le_bytes_to_int32 - 4字节小端序 → int32_t
 * @param p: 数据起始指针
 * @return: 有符号32位整数 (小端)
 */
static int32_t le_to_int32(const uint8_t *p) {
    return (int32_t)((uint32_t)p[0]          |
                     ((uint32_t)p[1] << 8)  |
                     ((uint32_t)p[2] << 16) |
                     ((uint32_t)p[3] << 24));
}

/**
 * le_bytes_to_int16 - 2字节小端序 → int16_t
 * @param p: 数据起始指针
 * @return: 有符号16位整数 (小端)
 */
static int16_t le_to_int16(const uint8_t *p) {
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

/**
 * le_bytes_to_uint16 - 2字节小端序 → uint16_t
 * @param p: 数据起始指针
 * @return: 无符号16位整数 (小端)
 */
static uint16_t le_to_uint16(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

/**
 * be_bytes_to_int16 - 2字节大端序 → int16_t
 * @param p: 数据起始指针
 * @return: 有符号16位整数 (大端)
 */
static int16_t be_to_int16(const uint8_t *p) {
    return (int16_t)((uint16_t)(p[0] << 8) | (uint16_t)p[1]);
}

/**
 * be_bytes_to_uint16 - 2字节大端序 → uint16_t
 * @param p: 数据起始指针
 * @return: 无符号16位整数 (大端)
 */
static uint16_t be_to_uint16(const uint8_t *p) {
    return (uint16_t)((uint16_t)(p[0] << 8) | (uint16_t)p[1]);
}

/**
 * le_to_float - 4字节小端序 IEEE754 → float
 * @param p: 数据起始指针
 * @return: 单精度浮点数 (小端)
 *
 * 通过 memcpy 避免严格别名规则(strict aliasing)问题。
 * 例: [BC 86 56 43] → 214.5263
 */
static float le_to_float(const uint8_t *p) {
    uint32_t raw = (uint32_t)p[0]          |
                   ((uint32_t)p[1] << 8)  |
                   ((uint32_t)p[2] << 16) |
                   ((uint32_t)p[3] << 24);
    float val;
    memcpy(&val, &raw, sizeof(val));
    return val;
}

/* ======================== 寄存器与常量定义 ======================== */
#define SLAVE_ADDR       0x01    /**< 从站地址 */
#define FUNC_READ_HOLD   0x03    /**< 功能码: 读保持寄存器 */
#define REG_START_ADDR   0x0010  /**< 起始寄存器地址 (ADC Value) */
#define REG_TOTAL_COUNT  8       /**< 总共读取 8 个保持寄存器 (16字节) */
/**< 8个寄存器 × 2字节 = 16B: 4+4+4+2+2 = ADC+Empty+SCALE1+WEIGHT+STATUS */

/** @brief 各字段在响应数据区中的字节偏移量 */
#define OFFS_ADC_VALUE         0   /* [+0]  ADC原始值      4B int32 LE */
#define OFFS_EMPTYLOAD_VALUE   4   /* [+4]  去皮值          4B int32 LE */
#define OFFS_SCALE1            8   /* [+8]  校准系数        4B float  LE */
#define OFFS_WEIGHT            12  /* [+12] 实时重量        2B int16 LE */
#define OFFS_CONT_AND_STATUS   14  /* [+14] 状态标志        2B uint16 LE */

/* ======================== 信号处理 ======================== */
static void on_sigint(int sig) { (void)sig; g_run = 0; }

/* ======================== 串口初始化 ======================== */
static int uart_init(const char *dev_path) {
    int fd = open(dev_path, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) { perror("open uart"); return -1; }

    struct termios cfg = {0};
    tcgetattr(fd, &cfg);

    cfsetispeed(&cfg, B4800);
    cfsetospeed(&cfg, B4800);

    cfg.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);  /* 无校验、1停止位、清大小、无流控 */
    cfg.c_cflag |= CS8 | CREAD | CLOCAL;                   /* 8数据位、允许读、忽略DCD */

    cfg.c_iflag &= ~(IXON | IXOFF | IXANY | INLCR | ICRNL | IGNCR);  /* 关软流控、关CR转换 */
    cfg.c_oflag &= ~OPOST;
    cfg.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);                  /* RAW模式、无回显 */
    cfg.c_cc[VMIN]  = 0;
    cfg.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &cfg) != 0) { perror("tcsetattr"); close(fd); return -1; }
    tcflush(fd, TCIOFLUSH);
    return fd;
}

/* ======================== Modbus 收发 ======================== */

/**
 * modbus_read_all_params - 发送单次请求读取全部 HX711 参数并接收响应
 *
 * @param fd:      串口文件描述符
 * @param tx_buf:  [输出] 发送帧缓冲 (至少8字节)
 * @param tx_len:  [输出] 发送帧长度
 * @param rx_buf:  [输出] 接收缓冲 (调用者分配 >= 256字节)
 * @param rx_len:  [输出] 实际接收字节数
 * @return: 0=成功, -1=无响应, -2=CRC校验失败
 *
 * 请求帧: 01 03 00 10 00 08 CRCL CRCH  (从0x0010起读8个寄存器=16字节)
 * 正常响应: 01 03 10 [16字节数据] CRCL CRCH
 */
static int modbus_read_all_params(int fd,
                                   uint8_t *tx_buf, uint8_t *tx_len,
                                   uint8_t *rx_buf, int *rx_len) {
    /* ---- 构造请求帧 ---- */
    uint8_t tx[8];
    tx[0] = SLAVE_ADDR;
    tx[1] = FUNC_READ_HOLD;
    tx[2] = (REG_START_ADDR >> 8) & 0xFF;   /* 起始地址高 */
    tx[3] = REG_START_ADDR & 0xFF;           /* 起始地址低 */
    tx[4] = (REG_TOTAL_COUNT >> 8) & 0xFF;   /* 寄存器数量高 */
    tx[5] = REG_TOTAL_COUNT & 0xFF;          /* 寄存器数量低 */

    uint16_t crc = crc16_modbus(tx, 6);
    tx[6] = crc & 0xFF;
    tx[7] = (crc >> 8) & 0xFF;

    memcpy(tx_buf, tx, sizeof(tx));
    *tx_len = sizeof(tx);

    /* ---- 发送 ---- */
    ssize_t wlen = write(fd, tx, sizeof(tx));
    if (wlen != (ssize_t)sizeof(tx))
        fprintf(stderr, "[WARN] write: expected %zd, got %zd\n", sizeof(tx), wlen);
    fsync(fd);

    /* ---- 等待响应 (100ms余量) ---- */
    usleep(100000);

    memset(rx_buf, 0, 256);
    *rx_len = (int)read(fd, rx_buf, 255);
    if (*rx_len <= 0) return -1;

    /* ---- CRC校验 ---- */
    uint16_t calc_crc = crc16_modbus(rx_buf, (uint16_t)(*rx_len - 2));
    uint16_t recv_crc = (uint16_t)(rx_buf[*rx_len - 2] | (rx_buf[*rx_len - 1] << 8));
    if (calc_crc != recv_crc) {
        fprintf(stderr, "[ERR] CRC mismatch: calc=0x%04X recv=0x%04X\n",
                calc_crc, recv_crc);
        return -2;
    }
    return 0;
}

/* ======================== 主程序 ======================== */
int main(int argc, char **argv) {
    const char *uart_dev = (argc > 1) ? argv[1] : "/dev/ttyAMA0";

    signal(SIGINT, on_sigint);

    int fd = uart_init(uart_dev);
    if (fd < 0) {
        fprintf(stderr, "无法打开串口 %s\n", uart_dev);
        return EXIT_FAILURE;
    }

    printf("========================================\n");
    printf("  Modbus RTU -> HX711 全参数轮询\n");
    printf("  串口: %s  @ 4800bps 8N1\n", uart_dev);
    printf("  从站: 0x%02X | 读%d reg(16B) | 1s/轮\n",
           SLAVE_ADDR, REG_TOTAL_COUNT);
    printf("========================================\n\n");

    int cycle = 0;

    while (g_run) {
        printf("---------- 第 %04d 轮 ----------\n", ++cycle);

        uint8_t tx[8], rx[256];
        uint8_t tlen;
        int rlen = 0;

        int ret = modbus_read_all_params(fd, tx, &tlen, rx, &rlen);

        /* --- 打印发送帧 --- */
        printf("  TX(%dB): ", tlen);
        for (uint8_t i = 0; i < tlen; i++) printf("%02X ", tx[i]);

        if (ret == -1) {
            printf("| 没数据\n\n");
            usleep(900000);
            continue;
        }
        if (ret == -2) {
            printf("| RX(%dB): ", rlen);
            for (int i = 0; i < rlen; i++) printf("%02X ", rx[i]);
            printf("(CRC错误)\n\n");
            usleep(900000);
            continue;
        }

        /* --- 打印接收帧原始数据 --- */
        printf("\n  RX(%dB): ", rlen);
        for (int i = 0; i < rlen; i++) printf("%02X ", rx[i]);
        printf("\n");

        /* --- 检查异常响应 --- */
        if (rx[1] & 0x80) {
            printf("  异常码 = 0x%02X\n\n", rx[2]);
            usleep(900000);
            continue;
        }

        /* --- 正常响应: 解析5个字段 (Modbus Poll 风格) ---
         * 响应结构: [地址][功能码][字节数N][数据区(N字节)][CRC]
         * 数据区从 rx[3] 开始 */
        const uint8_t *d = &rx[3];  /* 指向数据区首字节 */

        /* [+0] ADC Value - int32 小端 → 十六进制显示 */
        {
            int32_t v = le_to_int32(d + OFFS_ADC_VALUE);
            printf("HX711_Parameters.ADC Value       = 0x%06lX;\n",
                   (unsigned long)(uint32_t)v);
        }

        /* [+4] EmptyLoad Value - int32 小端 → 十六进制显示 */
        {
            int32_t v = le_to_int32(d + OFFS_EMPTYLOAD_VALUE);
            printf("HX711_Parameters.EmptyLoad_Value = 0x%06lX;\n",
                   (unsigned long)(uint32_t)v);
        }

        /* [+8] SCALE1 - float 小端 → 浮点数显示 */
        {
            float v = le_to_float(d + OFFS_SCALE1);
            printf("HX711_Parameters.SCALE1           = %.4f;\n", v);
        }

        /* [+12] WEIGHT - int16 大端 → 整数显示 */
        {
            int16_t v = be_to_int16(d + OFFS_WEIGHT);
            printf("HX711_Parameters.WEIGHT           = %d;\n", (int)v);
        }

        /* [+14] ContAndStatus - uint16 大端 → 十六进制显示 */
        {
            uint16_t v = be_to_uint16(d + OFFS_CONT_AND_STATUS);
            printf("HX711_Parameters.ContAndStatus    = 0x%04X;\n", v);
        }

        printf("\n");
        usleep(900000);  /* 补齐约1秒总间隔 */
    }

    printf("共完成 %d 轮查询\n", cycle);
    close(fd);
    return EXIT_SUCCESS;
}
