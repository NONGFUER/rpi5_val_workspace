/**
 * ============================================================================
 * Modbus RTU 网络秤测试程序
 * 基于 XGS-STM32-HX711-1CH-A 模块协议 v2.1
 * 
 * 功能：
 *   - 功能码 0x03: 读保持寄存器 (ADC原始值、去皮值、校准系数、实时重量、状态)
 *   - 功能码 0x06: 写单个寄存器 (去皮、校准、控制)
 *   - CRC16 校验 (MODBUS 标准)
 *   - 异常响应检测与处理
 *   - 完整测试用例套件
 *
 * 编译：gcc -o modbus_scale_test modbus_scale_test.c -Wall
 * 运行：./modbus_scale_test [串口设备]
 *       默认串口: /dev/ttyAMA0
 * ============================================================================
 */

#define _DEFAULT_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdint.h>
#include <time.h>
#include <errno.h>
#include <sys/time.h>

/* ======================== 配置参数 ======================== */
#define MODBUS_SLAVE_ADDR        0x01        // 从机地址
#define UART_BAUD_RATE           B4800       // 波特率 4800
#define UART_DEVICE_DEFAULT      "/dev/ttyAMA0"
#define TX_RX_DELAY_US           100000      // 发送后等待 100ms
#define RESPONSE_TIMEOUT_S       2           // 接收超时 2秒
#define POLL_INTERVAL_MS         150         // 轮询间隔 >= 100ms (HX711要求)
#define MAX_RETRIES              3           // 最大重试次数
#define STABLE_CHECK_COUNT       10          // 稳定状态连续检查次数

/* ======================== 寄存器地址定义 ================= */
#define REG_ADC_RAW              0x0000      // ADC 原始值 (只读, 2个寄存器)
#define REG_TARE                 0x0002      // 去皮值 (读写, 2个寄存器)
#define REG_CALIBRATION          0x0004      // 校准系数 (读写, 2个寄存器)
#define REG_WEIGHT               0x0006      // 实时重量-克 (只读, 1个寄存器)
#define REG_STATUS_CTRL          0x0007      // 控制与状态 (读写, 1个寄存器)

/* 状态寄存器位定义 */
#define STATUS_BIT_AUTO_TARE     (1 << 0)    // Bit0: 自动去皮使能
#define STATUS_BIT_STABLE        (1 << 1)    // Bit1: 稳定状态
#define STATUS_BIT_CONT_OUTPUT   (1 << 2)    // Bit2: 允许连续输出

/* ======================== 功能码 ======================== */
#define FUNC_READ_HOLDING        0x03
#define FUNC_WRITE_SINGLE        0x06
#define FUNC_ERROR_MASK          0x80

/* ======================== 异常码 ======================== */
#define ERR_ILLEGAL_FUNC         0x01
#define ERR_ILLEGAL_ADDR         0x02
#define ERR_ILLEGAL_DATA         0x03

/* ======================== 帧长度限制 ==================== */
#define MAX_FRAME_SIZE           256
#define MAX_RESPONSE_SIZE        256

/* ======================== 全局统计 ====================== */
static int g_total_tests = 0;
static int g_passed_tests = 0;
static int g_failed_tests = 0;

/* ============================================================
 *  CRC-16/MODBUS 计算 (查表法 - 高效实现)
 *  多项式: 0x8005, 初始值: 0xFFFF, 出异或: 无
 *  输出: 低字节在前 (小端模式)
 * ============================================================ */
static const uint8_t crc_hi_table[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
};

static const uint8_t crc_lo_table[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2,
    0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04,
    0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E,
    0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8,
    0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
    0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
    0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6,
    0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10,
    0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32,
    0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE,
    0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
    0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA,
    0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C,
    0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0,
    0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62,
    0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
    0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE,
    0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA,
    0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C,
    0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76,
    0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
    0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
    0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54,
    0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E,
    0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98,
    0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A,
    0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86,
    0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};

uint16_t crc16_modbus(const uint8_t *data, uint16_t len) {
    uint8_t crc_hi = 0xFF;
    uint8_t crc_lo = 0xFF;
    for (uint16_t i = 0; i < len; i++) {
        uint8_t idx = crc_hi ^ data[i];
        crc_hi = crc_lo ^ crc_hi_table[idx];
        crc_lo = crc_lo_table[idx];
    }
    return ((uint16_t)crc_hi << 8) | crc_lo;
}

/* ============================================================
 *  串口初始化与配置
 *  参数: 4800bps, 8N1, 原始模式
 * ============================================================ */
int uart_open(const char *device) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        perror("[错误] 无法打开串口");
        return -1;
    }

    /* 设置为阻塞模式 */
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags & ~O_NDELAY);

    struct termios options;
    memset(&options, 0, sizeof(options));
    tcgetattr(fd, &options);

    /* 波特率 4800 */
    cfsetispeed(&options, UART_BAUD_RATE);
    cfsetospeed(&options, UART_BAUD_RATE);

    /* 8N1: 8数据位, 无校验, 1停止位 */
    options.c_cflag &= ~PARENB;     // 无校验
    options.c_cflag &= ~CSTOPB;     // 1停止位
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;         // 8数据位
    options.c_cflag |= CLOCAL | CREAD; // 忽略 modem 控制, 使能接收

    /* 关闭流控 */
    options.c_cflag &= ~CRTSCTS;
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP |
                          INLCR | IGNCR | ICRNL);

    /* 原始输出 */
    options.c_oflag &= ~OPOST;
    options.c_oflag &= ~ONLCR;

    /* 原始输入 */
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_lflag &= ~(IEXTEN | ECHOK | ECHOCTL | ECHOKE);

    /* 超时设置: 读超时 100ms*10=1s, 最少收到1字节即返回 */
    options.c_cc[VMIN]  = 0;
    options.c_cc[VTIME] = 10;       // 1秒超时 (单位: 100ms)

    tcflush(fd, TCIFLUSH);
    if (tcsetattr(fd, TCSANOW, &options) != 0) {
        perror("[错误] 串口配置失败");
        close(fd);
        return -1;
    }

    printf("[信息] 串口已打开: %s @ 4800bps 8N1\n", device);
    return fd;
}

void uart_close(int fd) {
    if (fd >= 0) close(fd);
}

/* ============================================================
 *  工具函数: 打印十六进制数据
 * ============================================================ */
void print_hex(const char *label, const uint8_t *data, int len) {
    printf("%s[%d]: ", label, len);
    for (int i = 0; i < len; i++) {
        printf("%02X ", data[i]);
    }
    printf("\n");
}

/* ============================================================
 *  Modbus RTU 帧构建
 *  
 *  帧格式 (大端字节序):
 *  [地址码(1)] [功能码(1)] [数据(n)] [CRC_LO(1)] [CRC_HI(1)]
 * ============================================================ */

/**
 * 构建读保持寄存器请求帧 (功能码 0x03)
 * 帧结构: [Addr][Func][StartHi][StartLo][QtyHi][QtyLo][CRC]
 */
int build_read_request(uint8_t slave_addr, uint16_t reg_addr,
                       uint16_t reg_count, uint8_t *out_frame, int out_size) {
    if (out_size < 8) return -1;

    int idx = 0;
    out_frame[idx++] = slave_addr;
    out_frame[idx++] = FUNC_READ_HOLDING;
    out_frame[idx++] = (reg_addr >> 8) & 0xFF;   // 起始地址高字节
    out_frame[idx++] = reg_addr & 0xFF;            // 起始地址低字节
    out_frame[idx++] = (reg_count >> 8) & 0xFF;   // 寄存器数量高字节
    out_frame[idx++] = reg_count & 0xFF;           // 寄存器数量低字节

    /* 附加 CRC (低字节在前) */
    uint16_t crc = crc16_modbus(out_frame, idx);
    out_frame[idx++] = crc & 0xFF;
    out_frame[idx++] = (crc >> 8) & 0xFF;

    return idx;  // 返回帧长度
}

/**
 * 构建写单个寄存器请求帧 (功能码 0x06)
 * 帧结构: [Addr][Func][RegHi][RegLo][ValHi][ValLo][CRC]
 */
int build_write_request(uint8_t slave_addr, uint16_t reg_addr,
                        uint16_t value, uint8_t *out_frame, int out_size) {
    if (out_size < 8) return -1;

    int idx = 0;
    out_frame[idx++] = slave_addr;
    out_frame[idx++] = FUNC_WRITE_SINGLE;
    out_frame[idx++] = (reg_addr >> 8) & 0xFF;
    out_frame[idx++] = reg_addr & 0xFF;
    out_frame[idx++] = (value >> 8) & 0xFF;       // 写入值高字节
    out_frame[idx++] = value & 0xFF;               // 写入值低字节

    uint16_t crc = crc16_modbus(out_frame, idx);
    out_frame[idx++] = crc & 0xFF;
    out_frame[idx++] = (crc >> 8) & 0xFF;

    return idx;
}

/* ============================================================
 *  Modbus RTU 响应解析
 * ============================================================ */

/** 异常码描述表 */
static const char *exception_code_desc(uint8_t code) {
    switch (code) {
        case ERR_ILLEGAL_FUNC:  return "非法功能码";
        case ERR_ILLEGAL_ADDR:  return "非法寄存器地址";
        case ERR_ILLEGAL_DATA:  return "数据值超出范围";
        default:                return "未知异常";
    }
}

/**
 * 解析读响应 (功能码 0x03 正常响应)
 * 帧结构: [Addr][Func][ByteCount][Data...][CRC]
 * 返回: 数据字节数, -1表示异常响应, -2表示CRC错误, -3表示格式错误
 */
int parse_read_response(const uint8_t *resp, int resp_len,
                        uint16_t **out_data, int *out_reg_count) {
    if (resp_len < 5) {
        printf("[解析] 响应帧过短 (%d 字节)\n", resp_len);
        return -3;
    }

    /* 检查是否为异常响应 (功能码最高位为1) */
    if (resp[1] & FUNC_ERROR_MASK) {
        uint8_t err_code = resp[2];
        printf("[异常] 功能码 0x%02X 异常响应, 错误码: 0x%02X - %s\n",
               resp[1] & 0x7F, err_code, exception_code_desc(err_code));
        return -1;
    }

    /* 验证 CRC */
    uint16_t calc_crc = crc16_modbus(resp, resp_len - 2);
    uint16_t recv_crc = resp[resp_len - 2] | (resp[resp_len - 1] << 8);
    if (calc_crc != recv_crc) {
        printf("[CRC错误] 计算值: %04X, 收到值: %04X\n", calc_crc, recv_crc);
        return -2;
    }

    uint8_t byte_count = resp[2];
    if (resp_len < (int)(3 + byte_count + 2)) {
        printf("[解析] 数据长度不匹配 (声明%d字节, 实际%d字节)\n",
               byte_count, resp_len - 5);
        return -3;
    }

    *out_reg_count = byte_count / 2;
    *out_data = (uint16_t *)&resp[3];  // 指向数据区 (大端序)

    return byte_count;
}

/**
 * 解析写响应 (功能码 0x06 响应 = 请求回显)
 * 返回: 0成功, 负数为错误
 */
int parse_write_response(const uint8_t *resp, int resp_len,
                         uint16_t exp_addr, uint16_t exp_value) {
    if (resp_len < 8) {
        printf("[解析] 写响应帧过短\n");
        return -3;
    }

    /* 检查异常响应 */
    if (resp[1] & FUNC_ERROR_MASK) {
        uint8_t err_code = resp[2];
        printf("[异常] 功能码 0x%02X 异常响应, 错误码: 0x%02X - %s\n",
               resp[1] & 0x7F, err_code, exception_code_desc(err_code));
        return -1;
    }

    /* 验证 CRC */
    uint16_t calc_crc = crc16_modbus(resp, resp_len - 2);
    uint16_t recv_crc = resp[resp_len - 2] | (resp[resp_len - 1] << 8);
    if (calc_crc != recv_crc) {
        printf("[CRC错误] 计算值: %04X, 收到值: %04X\n", calc_crc, recv_crc);
        return -2;
    }

    /* 验证回显 */
    uint16_t ack_addr  = (resp[2] << 8) | resp[3];
    uint16_t ack_value = (resp[4] << 8) | resp[5];

    if (ack_addr != exp_addr || ack_value != exp_value) {
        printf("[回显不匹配] 地址: 期望%04X 实际%04X, 值: 期望%04X 实际%04X\n",
               exp_addr, ack_addr, exp_value, ack_value);
        return -4;
    }

    return 0;
}

/* ============================================================
 *  底层通信函数
 * ============================================================ */

/**
 * 发送帧并接收响应 (带重试机制)
 * 返回: 接收到的字节数, 负数表示错误
 */
int modbus_transaction(int fd, const uint8_t *tx_frame, int tx_len,
                       uint8_t *rx_buffer, int rx_buffer_size) {
    for (int retry = 0; retry < MAX_RETRIES; retry++) {
        if (retry > 0) {
            printf("  [重试] 第 %d/%d 次...\n", retry + 1, MAX_RETRIES);
            usleep(50000);  // 重试前等待 50ms
        }

        /* 清空接收缓冲 */
        tcflush(fd, TCIFLUSH);

        /* 发送请求帧 */
        print_hex("TX", tx_frame, tx_len);
        ssize_t written = write(fd, tx_frame, tx_len);
        if (written != tx_len) {
            printf("[发送错误] 期望写%d字节, 实际写%zd字节\n", tx_len, written);
            continue;
        }

        /* 等待从机响应 */
        usleep(TX_RX_DELAY_US);

        /* 读取响应 */
        memset(rx_buffer, 0, rx_buffer_size);
        int total_read = 0;

        struct timeval tv_start, tv_now;
        gettimeofday(&tv_start, NULL);

        while (1) {
            gettimeofday(&tv_now, NULL);
            double elapsed = (tv_now.tv_sec - tv_start.tv_sec) +
                             (tv_now.tv_usec - tv_start.tv_usec) / 1000000.0;
            if (elapsed > RESPONSE_TIMEOUT_S) break;

            ssize_t n = read(fd, rx_buffer + total_read, rx_buffer_size - total_read - 1);
            if (n > 0) {
                total_read += n;
                /* 检查是否已接收完整帧 (最小帧长: 5字节) */
                if (total_read >= 5) break;
            }
            usleep(10000);  // 10ms 轮询间隔
        }

        if (total_read > 0) {
            print_hex("RX", rx_buffer, total_read);
            return total_read;
        }

        printf("[超时] 未接收到响应\n");
    }
    return -1;
}

/* ============================================================
 *  高级 API 函数
 * ============================================================ */

/**
 * 读保持寄存器 (封装完整流程)
 * 返回: 读取的寄存器数量, 负数表示错误
 */
int modbus_read_holding(int fd, uint8_t addr, uint16_t reg_addr,
                        uint16_t reg_count, uint16_t *result_values) {
    uint8_t tx_buf[MAX_FRAME_SIZE];
    uint8_t rx_buf[MAX_RESPONSE_SIZE];
    
    int frame_len = build_read_request(addr, reg_addr, reg_count, 
                                        tx_buf, sizeof(tx_buf));
    if (frame_len < 0) return -10;

    int rx_len = modbus_transaction(fd, tx_buf, frame_len, rx_buf, sizeof(rx_buf));
    if (rx_len < 0) return -11;

    uint16_t *data_ptr = NULL;
    int reg_cnt = 0;
    int data_bytes = parse_read_response(rx_buf, rx_len, &data_ptr, &reg_cnt);

    if (data_bytes < 0) return data_bytes;

    /* 复制结果 (处理大端序转换) */
    for (int i = 0; i < reg_cnt && i < (int)reg_count; i++) {
        result_values[i] = (data_ptr[i * 2] << 8) | data_ptr[i * 2 + 1];
    }

    return reg_cnt;
}

/**
 * 写单个寄存器 (封装完整流程)
 * 返回: 0成功, 负数表示错误
 */
int modbus_write_single(int fd, uint8_t addr, uint16_t reg_addr, uint16_t value) {
    uint8_t tx_buf[MAX_FRAME_SIZE];
    uint8_t rx_buf[MAX_RESPONSE_SIZE];

    int frame_len = build_write_request(addr, reg_addr, value, tx_buf, sizeof(tx_buf));
    if (frame_len < 0) return -10;

    int rx_len = modbus_transaction(fd, tx_buf, frame_len, rx_buf, sizeof(rx_buf));
    if (rx_len < 0) return -11;

    return parse_write_response(rx_buf, rx_len, reg_addr, value);
}

/* ============================================================
 *  测试框架
 * ============================================================ */
void test_result(const char *test_name, int passed) {
    g_total_tests++;
    if (passed) {
        g_passed_tests++;
        printf("  ✓ PASS: %s\n", test_name);
    } else {
        g_failed_tests++;
        printf("  ✗ FAIL: %s\n", test_name);
    }
}

void print_separator(void) {
    printf("─────────────────────────────────────────────────────\n");
}

/* ============================================================
 *  测试用例集合
 * ============================================================ */

/**
 * 测试1: 读取 ADC 原始值 (寄存器 0x0000)
 * 验证能否正常获取 HX711 的 ADC 原始采样值
 */
void test_read_adc_raw(int fd) {
    printf("\n[测试1] 读取 ADC 原始值 (寄存器 0x0000)\n");
    print_separator();

    uint16_t values[2] = {0};
    int ret = modbus_read_holding(fd, MODBUS_SLAVE_ADDR, REG_ADC_RAW, 2, values);

    if (ret == 2) {
        uint32_t adc_raw = ((uint32_t)values[0] << 16) | values[1];
        printf("  ADC 原始值 (32位): 0x%08X (%u)\n", adc_raw, adc_raw);
        test_result("ADC原始值读取成功", 1);
        test_result("ADC值非零检查", (adc_raw != 0) ? 1 : 0);  // 警告级
    } else {
        printf("  读取失败, 返回码: %d\n", ret);
        test_result("ADC原始值读取成功", 0);
    }
}

/**
 * 测试2: 读取实时重量 (寄存器 0x0006)
 * 验证称重数据的正确读取
 */
void test_read_weight(int fd) {
    printf("\n[测试2] 读取实时重量 (寄存器 0x0006)\n");
    print_separator();

    uint16_t values[1] = {0};
    int ret = modbus_read_holding(fd, MODBUS_SLAVE_ADDR, REG_WEIGHT, 1, values);

    if (ret == 1) {
        int16_t weight = (int16_t)values[0];  // 有符号
        printf("  当前重量: %d 克\n", weight);
        test_result("重量读取成功", 1);
    } else {
        printf("  读取失败, 返回码: %d\n", ret);
        test_result("重量读取成功", 0);
    }
}

/**
 * 测试3: 连续多次读取重量 (稳定性测试)
 * 验证在静止状态下重量值的波动范围
 */
void test_read_weight_stability(int fd) {
    printf("\n[测试3] 重量稳定性测试 (连续读取 %d 次)\n", STABLE_CHECK_COUNT);
    print_separator();
    printf("  请确保秤盘上无物体或已放置稳定物体...\n");
    sleep(1);

    int weights[STABLE_CHECK_COUNT];
    int success_count = 0;

    for (int i = 0; i < STABLE_CHECK_COUNT; i++) {
        uint16_t values[1] = {0};
        int ret = modbus_read_holding(fd, MODBUS_SLAVE_ADDR, REG_WEIGHT, 1, values);

        if (ret == 1) {
            weights[i] = (int16_t)values[0];
            success_count++;
            printf("  [%2d] 重量: %d g\n", i + 1, weights[i]);
        } else {
            weights[i] = 0;
            printf("  [%2d] 读取失败\n", i + 1);
        }

        usleep(POLL_INTERVAL_MS * 1000);  // HX711 要求 >= 100ms
    }

    if (success_count >= STABLE_CHECK_COUNT - 2) {  // 允许最多2次失败
        int min_w = weights[0], max_w = weights[0];
        for (int i = 1; i < STABLE_CHECK_COUNT; i++) {
            if (weights[i] < min_w) min_w = weights[i];
            if (weights[i] > max_w) max_w = weights[i];
        }
        int range = max_w - min_w;
        printf("  波动范围: %d g (Min=%d, Max=%d)\n", range, min_w, max_w);
        test_result("连续读取成功率", 1);
        test_result("波动范围 <= 5g", (range <= 5) ? 1 : 0);
    } else {
        test_result("连续读取成功率", 0);
    }
}

/**
 * 测试4: 读取控制与状态寄存器 (寄存器 0x0007)
 * 检查各状态位的含义
 */
void test_read_status(int fd) {
    printf("\n[测试4] 读取控制与状态寄存器 (0x0007)\n");
    print_separator();

    uint16_t values[1] = {0};
    int ret = modbus_read_holding(fd, MODBUS_SLAVE_ADDR, REG_STATUS_CTRL, 1, values);

    if (ret == 1) {
        uint16_t status = values[0];
        printf("  状态寄存器值: 0x%04X\n", status);
        printf("  └─ Bit0 (自动去皮):    %s\n",
               (status & STATUS_BIT_AUTO_TARE) ? "使能" : "禁用");
        printf("  └─ Bit1 (稳定状态):    %s\n",
               (status & STATUS_BIT_STABLE) ? "已稳定" : "未稳定");
        printf("  └─ Bit2 (连续输出):    %s\n",
               (status & STATUS_BIT_CONT_OUTPUT) ? "允许" : "禁止");

        test_result("状态寄存器读取成功", 1);
        
        /* 额外验证: 如果稳定位为1, 说明当前处于稳定状态 */
        if (status & STATUS_BIT_STABLE) {
            printf("  >> 提示: 当前称重已达到稳定状态\n");
        }
    } else {
        printf("  读取失败, 返回码: %d\n", ret);
        test_result("状态寄存器读取成功", 0);
    }
}

/**
 * 测试5: 读取去皮值 (寄存器 0x0002)
 */
void test_read_tare(int fd) {
    printf("\n[测试5] 读取去皮值 (寄存器 0x0002)\n");
    print_separator();

    uint16_t values[2] = {0};
    int ret = modbus_read_holding(fd, MODBUS_SLAVE_ADDR, REG_TARE, 2, values);

    if (ret == 2) {
        uint32_t tare = ((uint32_t)values[0] << 16) | values[1];
        printf("  当前去皮值: %u (0x%08X)\n", tare, tare);
        test_result("去皮值读取成功", 1);
    } else {
        printf("  读取失败, 返回码: %d\n", ret);
        test_result("去皮值读取成功", 0);
    }
}

/**
 * 测试6: 读取校准系数 (寄存器 0x0004)
 */
void test_read_calibration(int fd) {
    printf("\n[测试6] 读取校准系数 (寄存器 0x0004)\n");
    print_separator();

    uint16_t values[2] = {0};
    int ret = modbus_read_holding(fd, MODBUS_SLAVE_ADDR, REG_CALIBRATION, 2, values);

    if (ret == 2) {
        uint32_t cal = ((uint32_t)values[0] << 16) | values[1];
        printf("  校准系数: %u (0x%08X)\n", cal, cal);
        test_result("校准系数读取成功", 1);
    } else {
        printf("  读取失败, 返回码: %d\n", ret);
        test_result("校准系数读取成功", 0);
    }
}

/**
 * 测试7: 执行自动去皮操作
 * 流程: 将控制寄存器 Bit0 置1 -> MCU 自动记录当前重量作为皮重 -> 读取验证
 */
void test_auto_tare(int fd) {
    printf("\n[测试7] 自动去皮操作\n");
    print_separator();
    printf("  步骤1: 发送自动去皮指令 (置位状态寄存器 Bit0)...\n");

    int ret = modbus_write_single(fd, MODBUS_SLAVE_ADDR, 
                                  REG_STATUS_CTRL, STATUS_BIT_AUTO_TARE);
    if (ret == 0) {
        test_result("自动去皮指令发送成功", 1);
    } else {
        printf("  发送失败, 返回码: %d\n", ret);
        test_result("自动去皮指令发送成功", 0);
        return;
    }

    /* 等待 MCU 处理去皮操作 */
    printf("  等待 MCU 处理...\n");
    usleep(200000);  // 200ms

    /* 读取去皮值进行验证 */
    printf("  步骤2: 验证去皮值是否已更新...\n");
    uint16_t tare_values[2] = {0};
    ret = modbus_read_holding(fd, MODBUS_SLAVE_ADDR, REG_TARE, 2, tare_values);
    if (ret == 2) {
        uint32_t tare = ((uint32_t)tare_values[0] << 16) | tare_values[1];
        printf("  新去皮值: %u\n", tare);
        test_result("去皮值已更新", (tare > 0) ? 1 : 0);
    } else {
        test_result("去皮值已更新", 0);
    }
}

/**
 * 测试8: 写入去皮值 (直接设置去皮值为指定值)
 */
static uint16_t g_test8_tare_val = 200;  // 默认测试值
void test_write_tare_value(int fd) {
    printf("\n[测试8] 写入去皮值 = %d\n", g_test8_tare_val);
    print_separator();
    uint16_t tare_value = g_test8_tare_val;

    int ret = modbus_write_single(fd, MODBUS_SLAVE_ADDR, REG_TARE, tare_value);
    if (ret == 0) {
        test_result("去皮值写入成功", 1);

        /* 回读验证 */
        uint16_t values[1] = {0};
        ret = modbus_read_holding(fd, MODBUS_SLAVE_ADDR, REG_TARE, 1, values);
        if (ret == 1 && values[0] == tare_value) {
            test_result("去皮值回读验证通过", 1);
        } else {
            test_result("去皮值回读验证通过", 0);
        }
    } else {
        printf("  写入失败, 返回码: %d\n", ret);
        test_result("去皮值写入成功", 0);
    }
}

/**
 * 测试9: 异常情况测试 - 非法功能码
 * 发送一个不支持的功能码, 验证从机返回异常响应
 */
void test_exception_illegal_function(int fd) {
    printf("\n[测试9] 异常测试 - 非法功能码 (0x42)\n");
    print_separator();

    /* 手动构建一个非法功能码请求 */
    uint8_t tx_buf[] = {MODBUS_SLAVE_ADDR, 0x42, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00};
    uint16_t crc = crc16_modbus(tx_buf, 6);
    tx_buf[6] = crc & 0xFF;
    tx_buf[7] = (crc >> 8) & 0xFF;

    uint8_t rx_buf[MAX_RESPONSE_SIZE];
    int rx_len = modbus_transaction(fd, tx_buf, 8, rx_buf, sizeof(rx_buf));

    if (rx_len > 0 && (rx_buf[1] & FUNC_ERROR_MASK)) {
        printf("  收到异常响应, 错误码: 0x%02X (%s)\n",
               rx_buf[2], exception_code_desc(rx_buf[2]));
        test_result("非法功能码异常检测", (rx_buf[2] == ERR_ILLEGAL_FUNC) ? 1 : 0);
    } else if (rx_len == 0) {
        printf("  从机无响应 (可能被静默忽略)\n");
        test_result("非法功能码异常检测", 0);  // 也算一种有效行为
    } else {
        printf("  意外: 收到正常响应或无效响应\n");
        print_hex("RX", rx_buf, rx_len);
        test_result("非法功能码异常检测", 0);
    }
}

/**
 * 测试10: 异常情况测试 - 非法寄存器地址
 * 尝试读取不存在的寄存器地址范围
 */
void test_exception_illegal_address(int fd) {
    printf("\n[测试10] 异常测试 - 非法寄存器地址 (0x00FF)\n");
    print_separator();

    uint16_t values[1] = {0};
    int ret = modbus_read_holding(fd, MODBUS_SLAVE_ADDR, 0x00FF, 1, values);

    if (ret == -1) {  // parse_read_response 返回 -1 表示异常响应
        test_result("非法地址异常检测", 1);
    } else if (ret == -11) {  // 超时无响应
        printf("  从机无响应 (可能被静默忽略)\n");
        test_result("非法地址异常检测", 0);
    } else {
        printf("  意外: 读取到数据 (该地址可能存在)\n");
        test_result("非法地址异常检测", 0);
    }
}

/**
 * 测试11: CRC 校验测试 - 人为破坏 CRC 验证从机行为
 * 注意: 此测试仅观察从机如何处理 CRC 错误的帧
 */
void test_crc_error_handling(int fd) {
    printf("\n[测试11] CRC 错误处理测试\n");
    print_separator();

    /* 构建正常请求帧, 然后破坏 CRC */
    uint8_t tx_buf[8];
    build_read_request(MODBUS_SLAVE_ADDR, REG_WEIGHT, 1, tx_buf, sizeof(tx_buf));
    
    printf("  原始帧: ");
    for (int i = 0; i < 8; i++) printf("%02X ", tx_buf[i]);
    printf("\n");

    /* 破坏 CRC (翻转低位) */
    tx_buf[6] ^= 0xFF;
    printf("  破坏后: ");
    for (int i = 0; i < 8; i++) printf("%02X ", tx_buf[i]);
    printf("\n");

    /* 发送破坏后的帧 */
    uint8_t rx_buf[MAX_RESPONSE_SIZE];
    tcflush(fd, TCIFLUSH);
    write(fd, tx_buf, 8);
    usleep(TX_RX_DELAY_US);

    int rx_len = read(fd, rx_buf, sizeof(rx_buf));
    if (rx_len > 0) {
        printf("  从机有响应 (可能未严格校验CRC): ");
        for (int i = 0; i < rx_len; i++) printf("%02X ", rx_buf[i]);
        printf("\n");
        test_result("CRC错误时从机静默", 0);
    } else {
        printf("  从机无响应 (正确行为: CRC错误时应丢弃帧)\n");
        test_result("CRC错误时从机静默", 1);
    }
}

/**
 * 测试12: 综合测试 - 完整的称重工作流
 * 模拟实际使用场景: 去皮 -> 放置物品 -> 等待稳定 -> 读取重量
 */
void test_complete_workflow(int fd) {
    printf("\n[测试12] 综合工作流测试\n");
    print_separator();
    printf("  请按以下步骤操作:\n");
    printf("  1. 确保秤盘上有已知重量的物体\n");
    printf("  2. 等待 5 秒开始测试...\n");

    for (int i = 5; i > 0; i--) {
        printf("  %d...\n", i);
        sleep(1);
    }

    int pass_all = 1;

    /* Step 1: 先执行一次去皮 */
    printf("\n  >> Step 1: 执行自动去皮...\n");
    int ret = modbus_write_single(fd, MODBUS_SLAVE_ADDR,
                                  REG_STATUS_CTRL, STATUS_BIT_AUTO_TARE);
    if (ret != 0) {
        printf("     去皮指令失败!\n");
        pass_all = 0;
    }
    usleep(300000);

    /* Step 2: 检查稳定状态 */
    printf("  >> Step 2: 等待并检查稳定状态...\n");
    int stable = 0;
    for (int i = 0; i < 20; i++) {
        uint16_t status_val[1] = {0};
        modbus_read_holding(fd, MODBUS_SLAVE_ADDR, REG_STATUS_CTRL, 1, status_val);
        if (status_val[0] & STATUS_BIT_STABLE) {
            stable = 1;
            printf("     已稳定! (第 %d 次检查)\n", i + 1);
            break;
        }
        usleep(POLL_INTERVAL_MS * 1000);
    }
    test_result("稳定状态检测", stable);

    /* Step 3: 读取最终重量 */
    printf("  >> Step 3: 读取当前净重...\n");
    uint16_t weight_val[1] = {0};
    ret = modbus_read_holding(fd, MODBUS_SLAVE_ADDR, REG_WEIGHT, 1, weight_val);
    if (ret == 1) {
        int16_t final_weight = (int16_t)weight_val[0];
        printf("     最终净重: %d g\n", final_weight);
        test_result("重量读取成功", 1);
    } else {
        test_result("重量读取成功", 0);
        pass_all = 0;
    }

    test_result("综合工作流", pass_all);
}

/**
 * 测试13: 批量寄存器扫描 (一次性读取多个寄存器)
 * 验证连续寄存器读取功能
 */
void test_batch_register_scan(int fd) {
    printf("\n[测试13] 批量寄存器扫描 (0x0000 ~ 0x0007)\n");
    print_separator();

    /* 一次性读取所有寄存器 (共8个: 0x0000-0x0007) */
    uint16_t values[8] = {0};
    int ret = modbus_read_holding(fd, MODBUS_SLAVE_ADDR, 0x0000, 8, values);

    if (ret == 8) {
        printf("  ADC原始值(Hi):  0x%04X (%d)\n", values[0], values[0]);
        printf("  ADC原始值(Lo):  0x%04X (%d)\n", values[1], values[1]);
        printf("  去皮值(Hi):     0x%04X (%d)\n", values[2], values[2]);
        printf("  去皮值(Lo):     0x%04X (%d)\n", values[3], values[3]);
        printf("  校准系数(Hi):   0x%04X (%d)\n", values[4], values[4]);
        printf("  校准系数(Lo):   0x%04X (%d)\n", values[5], values[5]);
        printf("  实时重量:       0x%04X (%d)\n", values[6], (int16_t)values[6]);
        printf("  控制与状态:     0x%04X\n", values[7]);

        /* 解析状态位 */
        printf("  └─ 状态详情:\n");
        printf("     ├─ 自动去皮: %s\n", (values[7] & STATUS_BIT_AUTO_TARE) ? "ON" : "OFF");
        printf("     ├─ 稳定状态: %s\n", (values[7] & STATUS_BIT_STABLE) ? "YES" : "NO");
        printf("     └─ 连续输出: %s\n", (values[7] & STATUS_BIT_CONT_OUTPUT) ? "ON" : "OFF");

        test_result("批量读取成功", 1);
    } else {
        printf("  批量读取失败, 返回码: %d (可能不支持一次读取过多寄存器)\n", ret);
        test_result("批量读取成功", 0);
    }
}

/* ============================================================
 *  主程序入口
 * ============================================================ */
void print_banner(void) {
    printf("\n");
    printf("╔══════════════════════════════════════════════════════╗\n");
    printf("║    Modbus RTU 网络秤测试程序 v1.0                    ║\n");
    printf("║    协议: XGS-STM32-HX711-1CH-A v2.1                 ║\n");
    printf("║    接口: UART 4800bps 8N1                           ║\n");
    printf("╚══════════════════════════════════════════════════════╝\n");
}

void print_usage(const char *prog) {
    printf("用法: %s [选项]\n", prog);
    printf("选项:\n");
    printf("  -d <设备>   串口设备路径 (默认: %s)\n", UART_DEVICE_DEFAULT);
    printf("  -a <地址>   从机地址 (默认: 0x%02X)\n", MODBUS_SLAVE_ADDR);
    printf("  -t <编号>   运行指定测试 (1-13), 不指定则运行全部\n");
    printf("  -h          显示帮助信息\n");
    printf("\n可用测试:\n");
    printf("  1  - 读取ADC原始值\n");
    printf("  2  - 读取实时重量\n");
    printf("  3  - 重量稳定性测试\n");
    printf("  4  - 读取控制与状态寄存器\n");
    printf("  5  - 读取去皮值\n");
    printf("  6  - 读取校准系数\n");
    printf("  7  - 自动去皮操作\n");
    printf("  8  - 写入去皮值\n");
    printf("  9  - 异常测试: 非法功能码\n");
    printf("  10 - 异常测试: 非法地址\n");
    printf("  11 - CRC错误处理测试\n");
    printf("  12 - 综合工作流测试\n");
    printf("  13 - 批量寄存器扫描\n");
}

typedef void (*test_func_t)(int);

int main(int argc, char *argv[]) {
    const char *uart_device = UART_DEVICE_DEFAULT;
    uint8_t slave_addr = MODBUS_SLAVE_ADDR;
    int specific_test = 0;  // 0 = 运行全部

    /* 参数解析 */
    int opt;
    while ((opt = getopt(argc, argv, "d:a:t:h")) != -1) {
        switch (opt) {
            case 'd':
                uart_device = optarg;
                break;
            case 'a':
                slave_addr = (uint8_t)strtol(optarg, NULL, 16);
                break;
            case 't':
                specific_test = atoi(optarg);
                break;
            case 'h':
                print_usage(argv[0]);
                return 0;
            default:
                print_usage(argv[0]);
                return 1;
        }
    }

    print_banner();
    printf("[配置] 串口: %s, 从机地址: 0x%02X\n", uart_device, slave_addr);

    /* 打开串口 */
    int fd = uart_open(uart_device);
    if (fd < 0) {
        fprintf(stderr, "\n致命错误: 无法初始化串口, 退出。\n");
        return 1;
    }

    /* 定义测试函数表 */
    test_func_t tests[] = {
        test_read_adc_raw,          // 1
        test_read_weight,           // 2
        test_read_weight_stability, // 3
        test_read_status,           // 4
        test_read_tare,             // 5
        test_read_calibration,      // 6
        test_auto_tare,             // 7
        test_write_tare_value,      // 8
        test_exception_illegal_function,  // 9
        test_exception_illegal_address,   // 10
        test_crc_error_handling,    // 11
        test_complete_workflow,     // 12
        test_batch_register_scan    // 13
    };
    int num_tests = sizeof(tests) / sizeof(tests[0]);

    /* 执行测试 */
    if (specific_test > 0 && specific_test <= num_tests) {
        printf("\n▶ 运行测试 %d...\n", specific_test);
        tests[specific_test - 1](fd);
    } else {
        printf("\n▶ 运行全部 %d 个测试...\n", num_tests);
        for (int i = 0; i < num_tests; i++) {
            tests[i](fd);
        }
    }

    /* 打印测试总结 */
    printf("\n");
    printf("╔══════════════════════════════════════════════════════╗\n");
    printf("║                    测试结果汇总                      ║\n");
    printf("╠══════════════════════════════════════════════════════╣\n");
    printf("║  总计: %2d  │  通过: %2d  │  失败: %2d               ║\n",
           g_total_tests, g_passed_tests, g_failed_tests);
    printf("╚══════════════════════════════════════════════════════╝\n");

    if (g_failed_tests > 0) {
        printf("\n⚠ 有 %d 个测试未通过, 请检查硬件连接和模块配置。\n", g_failed_tests);
    } else {
        printf("\n✓ 所有测试通过! Modbus 通信正常。\n");
    }

    uart_close(fd);
    return (g_failed_tests > 0) ? 1 : 0;
}
