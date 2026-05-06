/**
 * ============================================================================
 * 电子秤 Modbus RTU 测试联调程序
 * 基于 mobus2.md 协议规范 V1.1
 *
 * 物理层: UART TTL, 9600bps, 8-N-1
 * 功能码: 03(读保持寄存器), 06(写单个寄存器)
 *
 * 寄存器映射 (mobus2.md V1.1):
 *   0x0000~0x0001  Net_Weight    净重值(32位补码, 单位g)      R
 *   0x0002         Status_Word   设备状态字                    R
 *   0x0003~0x0004  ADC_Raw       传感器原始ADC采样值           R
 *   0x0010         System_Cmd    命令寄存器(1:去皮;2:校准)    RW
 *   0x0020~0x0021  Cal_Factor    校准系数K(32位定点数)        RW
 *
 * 编译: gcc -o modbus_v11_test modbus_v11_test.c -Wall
 * 运行: ./modbus_v11_test [选项]
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
#define MODBUS_SLAVE_ADDR        0x01
#define UART_BAUD_RATE           B9600
#define UART_DEVICE_DEFAULT      "/dev/ttyAMA0"
#define TX_RX_DELAY_US           100000
#define RESPONSE_TIMEOUT_S       2
#define POLL_INTERVAL_MS         150
#define MAX_RETRIES              3
#define STABLE_CHECK_COUNT       10

/* ======================== 寄存器地址 (mobus2.md V1.1) ======= */
#define REG_NET_WEIGHT_H         0x0000    // 净重高16位 (Int16)
#define REG_NET_WEIGHT_L         0x0001    // 净重低16位
#define REG_STATUS_WORD          0x0002    // 状态字 (Uint16)
#define REG_ADC_RAW_H            0x0003    // ADC高16位 (Int16)
#define REG_ADC_RAW_L            0x0004    // ADC低16位
#define REG_SYSTEM_CMD           0x0010    // 系统命令寄存器 (Uint16)
#define REG_CAL_FACTOR_H         0x0020    // 校准系数K高16位 (Int16)
#define REG_CAL_FACTOR_L         0x0021    // 校准系数低16位

/* ======================== 系统命令定义 ==================== */
#define CMD_TARE                 0x0001    // 去皮命令
#define CMD_CALIBRATE            0x0002    // 校准命令

/* ======================== 状态字 Bit 定义 ================= */
#define STATUS_BIT_STABLE        (1 << 0)  // Bit0: 稳定
#define STATUS_BIT_OVERLOAD      (1 << 1)  // Bit1: 过载
#define STATUS_BIT_NEGATIVE      (1 << 2)  // Bit2: 负重
#define STATUS_BIT_TARED         (1 << 3)  // Bit3: 已去皮

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
 *  CRC-16/MODBUS 计算 (位运算方式, 与 uart_crc_val.c 一致)
 *  初始值: 0xFFFF, 多项式: 0xA001 (反射形式)
 *  输出: 低字节在前 (小端模式)
 * ============================================================ */
uint16_t crc16_modbus(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i];
        for (int j = 8; j != 0; j--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/* ============================================================
 *  串口初始化与配置
 *  参数: 9600bps, 8N1, 原始模式
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

    /* 波特率 9600 (mobus2.md 规范) */
    cfsetispeed(&options, UART_BAUD_RATE);
    cfsetospeed(&options, UART_BAUD_RATE);

    /* 8N1: 8数据位, 无校验, 1停止位 */
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag |= CLOCAL | CREAD;

    /* 关闭流控 */
    options.c_cflag &= ~CRTSCTS;
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP |
                          INLCR | IGNCR | ICRNL);

    /* 原始输出/输入 */
    options.c_oflag &= ~OPOST;
    options.c_oflag &= ~ONLCR;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_lflag &= ~(IEXTEN | ECHOK | ECHOCTL | ECHOKE);

    /* 超时设置 */
    options.c_cc[VMIN]  = 0;
    options.c_cc[VTIME] = 10;

    tcflush(fd, TCIFLUSH);
    if (tcsetattr(fd, TCSANOW, &options) != 0) {
        perror("[错误] 串口配置失败");
        close(fd);
        return -1;
    }

    printf("[信息] 串口已打开: %s @ 9600bps 8N1\n", device);
    return fd;
}

void uart_close(int fd) {
    if (fd >= 0) close(fd);
}

/* ============================================================
 *  工具函数
 * ============================================================ */
void print_hex(const char *label, const uint8_t *data, int len) {
    printf("%s[%d]: ", label, len);
    for (int i = 0; i < len; i++) {
        printf("%02X ", data[i]);
    }
    printf("\n");
}

static const char *exception_code_desc(uint8_t code) {
    switch (code) {
        case ERR_ILLEGAL_FUNC:  return "非法功能码";
        case ERR_ILLEGAL_ADDR:  return "非法寄存器地址";
        case ERR_ILLEGAL_DATA:  return "数据值超出范围";
        default:                return "未知异常";
    }
}

/* ============================================================
 *  Modbus RTU 帧构建
 *
 *  帧格式 (大端字节序):
 *  [地址码(1)] [功能码(1)] [数据(n)] [CRC_LO(1)] [CRC_HI(1)]
 * ============================================================ */

int build_read_request(uint8_t slave_addr, uint16_t reg_addr,
                       uint16_t reg_count, uint8_t *out_frame, int out_size) {
    if (out_size < 8) return -1;

    int idx = 0;
    out_frame[idx++] = slave_addr;
    out_frame[idx++] = FUNC_READ_HOLDING;
    out_frame[idx++] = (reg_addr >> 8) & 0xFF;
    out_frame[idx++] = reg_addr & 0xFF;
    out_frame[idx++] = (reg_count >> 8) & 0xFF;
    out_frame[idx++] = reg_count & 0xFF;

    uint16_t crc = crc16_modbus(out_frame, idx);
    out_frame[idx++] = crc & 0xFF;
    out_frame[idx++] = (crc >> 8) & 0xFF;

    return idx;
}

int build_write_request(uint8_t slave_addr, uint16_t reg_addr,
                        uint16_t value, uint8_t *out_frame, int out_size) {
    if (out_size < 8) return -1;

    int idx = 0;
    out_frame[idx++] = slave_addr;
    out_frame[idx++] = FUNC_WRITE_SINGLE;
    out_frame[idx++] = (reg_addr >> 8) & 0xFF;
    out_frame[idx++] = reg_addr & 0xFF;
    out_frame[idx++] = (value >> 8) & 0xFF;
    out_frame[idx++] = value & 0xFF;

    uint16_t crc = crc16_modbus(out_frame, idx);
    out_frame[idx++] = crc & 0xFF;
    out_frame[idx++] = (crc >> 8) & 0xFF;

    return idx;
}

/* ============================================================
 *  Modbus RTU 响应解析
 * ============================================================ */

int parse_read_response(const uint8_t *resp, int resp_len,
                        uint16_t *out_values, int max_values) {
    if (resp_len < 5) {
        printf("[解析] 响应帧过短 (%d 字节)\n", resp_len);
        return -3;
    }

    if (resp[1] & FUNC_ERROR_MASK) {
        uint8_t err_code = resp[2];
        printf("[异常] 功能码 0x%02X 异常响应, 错误码: 0x%02X - %s\n",
               resp[1] & 0x7F, err_code, exception_code_desc(err_code));
        return -1;
    }

    uint16_t calc_crc = crc16_modbus(resp, resp_len - 2);
    uint16_t recv_crc = resp[resp_len - 2] | (resp[resp_len - 1] << 8);
    if (calc_crc != recv_crc) {
        printf("[CRC错误] 计算值: %04X, 收到值: %04X\n", calc_crc, recv_crc);
        return -2;
    }

    uint8_t byte_count = resp[2];
    if (resp_len < (int)(3 + byte_count + 2)) {
        printf("[解析] 数据长度不匹配\n");
        return -3;
    }

    int reg_count = byte_count / 2;
    if (reg_count > max_values) reg_count = max_values;

    for (int i = 0; i < reg_count; i++) {
        out_values[i] = ((uint16_t)resp[3 + i * 2] << 8) | resp[3 + i * 2 + 1];
    }

    return reg_count;
}

int parse_write_response(const uint8_t *resp, int resp_len,
                         uint16_t exp_addr, uint16_t exp_value) {
    if (resp_len < 8) {
        printf("[解析] 写响应帧过短\n");
        return -3;
    }

    if (resp[1] & FUNC_ERROR_MASK) {
        uint8_t err_code = resp[2];
        printf("[异常] 功能码 0x%02X 异常响应, 错误码: 0x%02X - %s\n",
               resp[1] & 0x7F, err_code, exception_code_desc(err_code));
        return -1;
    }

    uint16_t calc_crc = crc16_modbus(resp, resp_len - 2);
    uint16_t recv_crc = resp[resp_len - 2] | (resp[resp_len - 1] << 8);
    if (calc_crc != recv_crc) {
        printf("[CRC错误] 计算值: %04X, 收到值: %04X\n", calc_crc, recv_crc);
        return -2;
    }

    uint16_t ack_addr  = ((uint16_t)resp[2] << 8) | resp[3];
    uint16_t ack_value = ((uint16_t)resp[4] << 8) | resp[5];

    if (ack_addr != exp_addr || ack_value != exp_value) {
        printf("[回显不匹配] 地址: 期望%04X 实际%04X, 值: 期望%04X 实际%04X\n",
               exp_addr, ack_addr, exp_value, ack_value);
        return -4;
    }

    return 0;
}

/* ============================================================
 *  底层通信函数 (带重试机制)
 * ============================================================ */
int modbus_transaction(int fd, const uint8_t *tx_frame, int tx_len,
                       uint8_t *rx_buffer, int rx_buffer_size) {
    for (int retry = 0; retry < MAX_RETRIES; retry++) {
        if (retry > 0) {
            printf("  [重试] 第 %d/%d 次...\n", retry + 1, MAX_RETRIES);
            usleep(50000);
        }

        tcflush(fd, TCIFLUSH);

        print_hex("TX", tx_frame, tx_len);
        ssize_t written = write(fd, tx_frame, tx_len);
        if (written != tx_len) {
            printf("[发送错误] 期望写%d字节, 实际写%zd字节\n", tx_len, written);
            continue;
        }

        usleep(TX_RX_DELAY_US);

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
                if (total_read >= 5) break;
            }
            usleep(10000);
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
 *  高级 API 封装
 * ============================================================ */

/**
 * 读保持寄存器
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

    return parse_read_response(rx_buf, rx_len, result_values, reg_count);
}

/**
 * 写单个寄存器
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
 *  业务逻辑辅助函数
 * ============================================================ */

/** 解析并打印状态字 (mobus2.md 第5节) */
void print_status_word(uint16_t status_word) {
    printf("  状态字: 0x%04X\n", status_word);
    printf("  ├─ Bit0 (稳定/Stability):  %s\n",
           (status_word & STATUS_BIT_STABLE) ? "1=稳定" : "0=波动中");
    printf("  ├─ Bit1 (过载/Overload):   %s\n",
           (status_word & STATUS_BIT_OVERLOAD) ? "1=过载!" : "0=正常");
    printf("  ├─ Bit2 (负重/Negative):   %s\n",
           (status_word & STATUS_BIT_NEGATIVE) ? "1=负重量" : "0=正常/零");
    printf("  └─ Bit3 (去皮/Tared):      %s\n",
           (status_word & STATUS_BIT_TARED) ? "1=已去皮" : "0=未去皮");
}

/** 解析 32 位净重值 (Net_Weight_H 为 Int16, Net_Weight_L 为 Uint16) */
int32_t parse_net_weight(uint16_t high, uint16_t low) {
    int32_t h = (int16_t)high;  // 有符号高位
    return (h << 16) | low;      // 组合为 32 位有符号
}

/** 解析 32 位 ADC 原始值 */
int32_t parse_adc_raw(uint16_t high, uint16_t low) {
    int32_t h = (int16_t)high;
    return (h << 16) | low;
}

/** 解析 32 位校准系数 */
int32_t parse_cal_factor(uint16_t high, uint16_t low) {
    int32_t h = (int16_t)high;
    return (h << 16) | low;
}

/* ============================================================
 *  测试框架
 * ============================================================ */
void test_result(const char *test_name, int passed) {
    g_total_tests++;
    if (passed) {
        g_passed_tests++;
        printf("  [PASS] %s\n", test_name);
    } else {
        g_failed_tests++;
        printf("  [FAIL] %s\n", test_name);
    }
}

void print_separator(void) {
    printf("──────────────────────────────────────────\n");
}

/* ============================================================
 *  测试用例集合 (对应 mobus2.md 协议规范)
 * ============================================================ */

/**
 * 测试1: 读取净重值 (寄存器 0x0000~0x0001)
 * 对应协议 6.1 示例: 读取重量及状态
 */
void test_read_net_weight(int fd) {
    printf("\n[测试1] 读取净重值 (寄存器 0x0000~0x0001)\n");
    print_separator();

    uint16_t values[2] = {0};
    int ret = modbus_read_holding(fd, MODBUS_SLAVE_ADDR,
                                   REG_NET_WEIGHT_H, 2, values);

    if (ret == 2) {
        int32_t weight_g = parse_net_weight(values[0], values[1]);
        printf("  Net_Weight_H: 0x%04X (%d)\n", values[0], (int16_t)values[0]);
        printf("  Net_Weight_L: 0x%04X (%d)\n", values[1], values[1]);
        printf("  净重 (32位): %ld g\n", (long)weight_g);
        test_result("净重值读取成功", 1);
    } else {
        printf("  读取失败, 返回码: %d\n", ret);
        test_result("净重值读取成功", 0);
    }
}

/**
 * 测试2: 读取状态字 (寄存器 0x0002)
 * 对应协议第5节: 状态字 Bit 详解
 */
void test_read_status_word(int fd) {
    printf("\n[测试2] 读取状态字 (寄存器 0x0002)\n");
    print_separator();

    uint16_t values[1] = {0};
    int ret = modbus_read_holding(fd, MODBUS_SLAVE_ADDR,
                                   REG_STATUS_WORD, 1, values);

    if (ret == 1) {
        print_status_word(values[0]);
        test_result("状态字读取成功", 1);
    } else {
        printf("  读取失败, 返回码: %d\n", ret);
        test_result("状态字读取成功", 0);
    }
}

/**
 * 测试3: 读取 ADC 原始采样值 (寄存器 0x0003~0x0004)
 */
void test_read_adc_raw(int fd) {
    printf("\n[测试3] 读取 ADC 原始值 (寄存器 0x0003~0x0004)\n");
    print_separator();

    uint16_t values[2] = {0};
    int ret = modbus_read_holding(fd, MODBUS_SLAVE_ADDR,
                                   REG_ADC_RAW_H, 2, values);

    if (ret == 2) {
        int32_t adc = parse_adc_raw(values[0], values[1]);
        printf("  ADC_Raw_H: 0x%04X (%d)\n", values[0], (int16_t)values[0]);
        printf("  ADC_Raw_L: 0x%04X (%u)\n", values[1], values[1]);
        printf("  ADC 原始值 (32位): 0x%08lX (%ld)\n",
               (unsigned long)adc, (long)adc);
        test_result("ADC原始值读取成功", 1);
        test_result("ADC值非零检查", (adc != 0) ? 1 : 0);
    } else {
        printf("  读取失败, 返回码: %d\n", ret);
        test_result("ADC原始值读取成功", 0);
    }
}

/**
 * 测试4: 批量读取净重+状态 (寄存器 0x0000~0x0002, 共3个寄存器)
 * 完整复现协议 6.1 报文示例:
 *   查询: 01 03 00 00 00 03 05 CB
 *   响应: 01 03 06 [Net_H] [Net_L] [Status] [CRC_L] [CRC_H]
 */
void test_read_weight_and_status(int fd) {
    printf("\n[测试4] 读取重量+状态 (协议6.1示例, 0x0000~0x0002)\n");
    print_separator();
    printf("  查询帧应为: 01 03 00 00 00 03 XX XX\n");

    uint16_t values[3] = {0};
    int ret = modbus_read_holding(fd, MODBUS_SLAVE_ADDR,
                                   REG_NET_WEIGHT_H, 3, values);

    if (ret == 3) {
        int32_t weight_g = parse_net_weight(values[0], values[1]);
        uint16_t status  = values[2];
        printf("  ┌─ 净重: %ld g\n", (long)weight_g);
        printf("  └─ 状态字:\n");
        print_status_word(status);
        test_result("批量读(净重+状态)成功", 1);
    } else {
        printf("  读取失败, 返回码: %d\n", ret);
        test_result("批量读(净重+状态)成功", 0);
    }
}

/**
 * 测试5: 连续读取净重 (稳定性测试)
 */
void test_weight_stability(int fd) {
    printf("\n[测试5] 净重稳定性测试 (连续 %d 次)\n", STABLE_CHECK_COUNT);
    print_separator();
    printf("  请确保秤盘处于稳定状态...\n");
    sleep(1);

    int32_t weights[STABLE_CHECK_COUNT];
    int success_count = 0;

    for (int i = 0; i < STABLE_CHECK_COUNT; i++) {
        uint16_t values[3] = {0};
        int ret = modbus_read_holding(fd, MODBUS_SLAVE_ADDR,
                                       REG_NET_WEIGHT_H, 3, values);
        if (ret == 3) {
            weights[i]  = parse_net_weight(values[0], values[1]);
            success_count++;
            const char *stable = (values[2] & STATUS_BIT_STABLE) ? "STABLE" : "unstable";
            printf("  [%2d] 净重: %8ld g  状态: 0x%04X (%s)\n",
                   i + 1, (long)weights[i], values[2], stable);
        } else {
            weights[i] = 0;
            printf("  [%2d] 读取失败\n", i + 1);
        }
        usleep(POLL_INTERVAL_MS * 1000);
    }

    if (success_count >= STABLE_CHECK_COUNT - 2) {
        long min_w = weights[0], max_w = weights[0];
        for (int i = 1; i < success_count; i++) {
            if (weights[i] < min_w) min_w = weights[i];
            if (weights[i] > max_w) max_w = weights[i];
        }
        long range = max_w - min_w;
        printf("  波动范围: %ld g (Min=%ld, Max=%ld)\n", range, min_w, max_w);
        test_result("连续读取成功率", 1);
        test_result("波动范围 <= 5g", (range <= 5) ? 1 : 0);
    } else {
        test_result("连续读取成功率", 0);
    }
}

/**
 * 测试6: 执行去皮操作 (功能码 0x06)
 * 对应协议 6.2 示例:
 *   主机下发: 01 06 00 10 00 01 88 09
 *   从机确认: 01 06 00 10 00 01 88 09
 */
void test_tare_operation(int fd) {
    printf("\n[测试6] 执行去皮操作 (功能码 0x06, 写 System_Cmd=1)\n");
    print_separator();
    printf("  下发帧应为: 01 06 00 10 00 01 XX XX\n");
    printf("  请确保秤盘上无物品或已放置容器...\n");
    sleep(1);

    int ret = modbus_write_single(fd, MODBUS_SLAVE_ADDR,
                                   REG_SYSTEM_CMD, CMD_TARE);
    if (ret == 0) {
        test_result("去皮指令发送成功 (回显匹配)", 1);
    } else {
        printf("  发送/确认失败, 返回码: %d\n", ret);
        test_result("去皮指令发送成功", 0);
        return;
    }

    /* 等待 MCU 处理 */
    usleep(300000);

    /* 验证: 重新读取净重和状态, 确认去皮生效 */
    printf("\n  验证: 读取去皮后的净重和状态...\n");
    uint16_t values[3] = {0};
    ret = modbus_read_holding(fd, MODBUS_SLAVE_ADDR,
                               REG_NET_WEIGHT_H, 3, values);
    if (ret == 3) {
        int32_t weight = parse_net_weight(values[0], values[1]);
        uint16_t status = values[2];
        printf("  去皮后净重: %ld g\n", (long)weight);
        printf("  状态字 Bit3 (去皮): %s\n",
               (status & STATUS_BIT_TARED) ? "1=已去皮" : "0=未去皮");
        test_result("去皮后状态验证", (status & STATUS_BIT_TARED) ? 1 : 0);
    } else {
        test_result("去皮后状态验证", 0);
    }
}

/**
 * 测试7: 读取校准系数 (寄存器 0x0020~0x0021)
 */
void test_read_calibration_factor(int fd) {
    printf("\n[测试7] 读取校准系数 Cal_Factor (寄存器 0x0020~0x0021)\n");
    print_separator();

    uint16_t values[2] = {0};
    int ret = modbus_read_holding(fd, MODBUS_SLAVE_ADDR,
                                   REG_CAL_FACTOR_H, 2, values);

    if (ret == 2) {
        int32_t cal_k = parse_cal_factor(values[0], values[1]);
        printf("  Cal_Factor_H: 0x%04X (%d)\n", values[0], (int16_t)values[0]);
        printf("  Cal_Factor_L: 0x%04X (%u)\n", values[1], values[1]);
        printf("  校准系数 K (32位): %ld (0x%08lX)\n",
               (long)cal_k, (unsigned long)cal_k);
        test_result("校准系数读取成功", 1);
    } else {
        printf("  读取失败, 返回码: %d\n", ret);
        test_result("校准系数读取成功", 0);
    }
}

/**
 * 测试8: 全寄存器扫描 (一次性读取所有已知寄存器区域)
 * 从 0x0000 ~ 0x0022 覆盖所有定义的寄存器
 */
void test_full_register_scan(int fd) {
    printf("\n[测试8] 全寄存器扫描 (0x0000 ~ 0x0021)\n");
    print_separator();

    /*
     * 注意: 寄存器不连续 (0x0004 -> 0x0010 有间隙),
     * 这里分两段读取:
     *   段A: 0x0000~0x0004 (净重H/L + 状态 + ADC H/L) = 5个寄存器
     *   段B: 0x0010~0x0010 (系统命令) = 1个寄存器
     *   段C: 0x0020~0x0021 (校准系数) = 2个寄存器
     */
    printf("  ┌─ 区域A: 0x0000~0x0004 (5个寄存器)\n");
    uint16_t area_a[5] = {0};
    int ret_a = modbus_read_holding(fd, MODBUS_SLAVE_ADDR, 0x0000, 5, area_a);

    if (ret_a == 5) {
        int32_t net_w = parse_net_weight(area_a[0], area_a[1]);
        printf("  │  Net_Weight: %ld g\n", (long)net_w);
        print_status_word(area_a[2]);
        int32_t adc = parse_adc_raw(area_a[3], area_a[4]);
        printf("  │  ADC_Raw:    0x%08lX\n", (unsigned long)adc);
        test_result("区域A 读取成功", 1);
    } else {
        printf("  │  区域A 读取失败 (ret=%d), 尝试逐个读取...\n", ret_a);
        test_result("区域A 读取成功", 0);
    }

    printf("  ├─ 区域B: 0x0010 (系统命令)\n");
    uint16_t area_b[1] = {0};
    int ret_b = modbus_read_holding(fd, MODBUS_SLAVE_ADDR, REG_SYSTEM_CMD, 1, area_b);
    if (ret_b == 1) {
        printf("  │  System_Cmd: 0x%04X\n", area_b[0]);
        test_result("区域B 读取成功", 1);
    } else {
        test_result("区域B 读取成功", 0);
    }

    printf("  └─ 区域C: 0x0020~0x0021 (校准系数)\n");
    uint16_t area_c[2] = {0};
    int ret_c = modbus_read_holding(fd, MODBUS_SLAVE_ADDR, REG_CAL_FACTOR_H, 2, area_c);
    if (ret_c == 2) {
        int32_t cal = parse_cal_factor(area_c[0], area_c[1]);
        printf("     Cal_Factor: %ld (0x%08lX)\n", (long)cal, (unsigned long)cal);
        test_result("区域C 读取成功", 1);
    } else {
        test_result("区域C 读取成功", 0);
    }
}

/**
 * 测试9: 异常测试 - 非法功能码
 */
void test_exception_illegal_func(int fd) {
    printf("\n[测试9] 异常测试 - 非法功能码 (0x42)\n");
    print_separator();

    uint8_t tx_buf[] = {MODBUS_SLAVE_ADDR, 0x42, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00};
    uint16_t crc = crc16_modbus(tx_buf, 6);
    tx_buf[6] = crc & 0xFF;
    tx_buf[7] = (crc >> 8) & 0xFF;

    uint8_t rx_buf[MAX_RESPONSE_SIZE];
    int rx_len = modbus_transaction(fd, tx_buf, 8, rx_buf, sizeof(rx_buf));

    if (rx_len > 0 && (rx_buf[1] & FUNC_ERROR_MASK)) {
        printf("  收到异常响应: 错误码 0x%02X (%s)\n",
               rx_buf[2], exception_code_desc(rx_buf[2]));
        test_result("非法功能码检测", (rx_buf[2] == ERR_ILLEGAL_FUNC) ? 1 : 0);
    } else if (rx_len == 0) {
        printf("  从机无响应 (静默忽略)\n");
        test_result("非法功能码检测", 0);
    } else {
        printf("  意外响应\n");
        print_hex("RX", rx_buf, rx_len);
        test_result("非法功能码检测", 0);
    }
}

/**
 * 测试10: 异常测试 - 非法寄存器地址
 */
void test_exception_illegal_addr(int fd) {
    printf("\n[测试10] 异常测试 - 非法地址 (0x00FF)\n");
    print_separator();

    uint16_t values[1] = {0};
    int ret = modbus_read_holding(fd, MODBUS_SLAVE_ADDR, 0x00FF, 1, values);

    if (ret == -1) {
        test_result("非法地址异常检测", 1);
    } else if (ret == -11) {
        printf("  从机无响应\n");
        test_result("非法地址异常检测", 0);
    } else {
        printf("  意外读到数据\n");
        test_result("非法地址异常检测", 0);
    }
}

/**
 * 测试11: CRC 错误处理测试
 */
void test_crc_error_handling(int fd) {
    printf("\n[测试11] CRC 错误处理测试\n");
    print_separator();

    uint8_t tx_buf[8];
    build_read_request(MODBUS_SLAVE_ADDR, REG_STATUS_WORD, 1, tx_buf, sizeof(tx_buf));

    printf("  正常帧: ");
    for (int i = 0; i < 8; i++) printf("%02X ", tx_buf[i]);
    printf("\n");

    /* 破坏 CRC */
    tx_buf[6] ^= 0xFF;
    printf("  破坏后: ");
    for (int i = 0; i < 8; i++) printf("%02X ", tx_buf[i]);
    printf("\n");

    tcflush(fd, TCIFLUSH);
    write(fd, tx_buf, 8);
    usleep(TX_RX_DELAY_US);

    uint8_t rx_buf[MAX_RESPONSE_SIZE];
    int rx_len = read(fd, rx_buf, sizeof(rx_buf));
    if (rx_len > 0) {
        printf("  从机有响应: ");
        for (int i = 0; i < rx_len; i++) printf("%02X ", rx_buf[i]);
        printf("\n");
        test_result("CRC错误时从机应静默", 0);
    } else {
        printf("  从机无响应 (正确行为)\n");
        test_result("CRC错误时从机应静默", 1);
    }
}

/**
 * 测试12: 综合工作流 — 去皮 → 放物 → 等待稳定 → 读重量
 */
void test_complete_workflow(int fd) {
    printf("\n[测试12] 综合工作流测试\n");
    print_separator();
    printf("  步骤:\n");
    printf("    1. 读取初始状态\n");
    printf("    2. 执行去皮\n");
    printf("    3. 放置物品后等待稳定\n");
    printf("    4. 读取最终净重\n\n");
    printf("  请在秤盘上放置待称物品, 5秒后开始...\n");

    for (int i = 5; i > 0; i--) {
        printf("  %d...\n", i);
        sleep(1);
    }

    int pass_all = 1;

    /* Step 1: 初始状态快照 */
    printf("\n  >>> Step 1: 读取初始状态 <<<\n");
    uint16_t init_vals[3] = {0};
    int ret = modbus_read_holding(fd, MODBUS_SLAVE_ADDR,
                                   REG_NET_WEIGHT_H, 3, init_vals);
    if (ret == 3) {
        int32_t init_w = parse_net_weight(init_vals[0], init_vals[1]);
        printf("  初始净重: %ld g\n", (long)init_w);
        print_status_word(init_vals[2]);
    } else {
        pass_all = 0;
    }

    /* Step 2: 去皮 */
    printf("\n  >>> Step 2: 执行去皮 <<<\n");
    ret = modbus_write_single(fd, MODBUS_SLAVE_ADDR,
                               REG_SYSTEM_CMD, CMD_TARE);
    if (ret != 0) {
        printf("  去皮失败!\n");
        pass_all = 0;
    } else {
        test_result("去皮指令OK", 1);
    }
    usleep(300000);

    /* Step 3: 等待稳定 */
    printf("\n  >>> Step 3: 等待稳定 <<<\n");
    int stable = 0;
    for (int i = 0; i < 20; i++) {
        uint16_t sv[1] = {0};
        modbus_read_holding(fd, MODBUS_SLAVE_ADDR, REG_STATUS_WORD, 1, sv);
        if (sv[0] & STATUS_BIT_STABLE) {
            stable = 1;
            printf("  已稳定! (第%d次检查)\n", i + 1);
            break;
        }
        usleep(POLL_INTERVAL_MS * 1000);
    }
    test_result("稳定状态检测", stable);

    /* Step 4: 读取最终重量 */
    printf("\n  >>> Step 4: 最终净重 <<<\n");
    uint16_t final_vals[3] = {0};
    ret = modbus_read_holding(fd, MODBUS_SLAVE_ADDR,
                               REG_NET_WEIGHT_H, 3, final_vals);
    if (ret == 3) {
        int32_t final_w = parse_net_weight(final_vals[0], final_vals[1]);
        printf("  最终净重: %ld g\n", (long)final_w);
        print_status_word(final_vals[2]);
        test_result("最终重量读取", 1);
    } else {
        test_result("最终重量读取", 0);
        pass_all = 0;
    }

    test_result("综合工作流", pass_all);
}

/**
 * 测试13: 实时监控模式 (持续循环读取净重和状态, 按 Ctrl+C 退出)
 */
void test_realtime_monitor(int fd) {
    printf("\n[测试13] 实时监控模式 (按 Ctrl+C 退出)\n");
    print_separator();
    printf("  每隔 %d ms 刷新一次净重和状态\n\n", POLL_INTERVAL_MS);
    printf("  %-8s %-14s %-12s %s\n", "#", "净重(g)", "状态字", "状态摘要");
    printf("  %-8s %-14s %-12s %s\n", "---", "--------", "--------", "------------------");

    int count = 0;
    while (1) {
        uint16_t values[3] = {0};
        int ret = modbus_read_holding(fd, MODBUS_SLAVE_ADDR,
                                       REG_NET_WEIGHT_H, 3, values);
        if (ret == 3) {
            int32_t w = parse_net_weight(values[0], values[1]);
            uint16_t s = values[2];

            const char *stab = (s & STATUS_BIT_STABLE) ? "STABLE" : "unstable";
            const char *ovr  = (s & STATUS_BIT_OVERLOAD) ? "OVLD" : "";
            const char *neg  = (s & STATUS_BIT_NEGATIVE) ? "NEG" : "";
            const char *tared = (s & STATUS_BIT_TARED) ? "TARED" : "";

            printf("  %-8ld %-14ld 0x%04X       %s %s %s %s\n",
                   (long)(++count), (long)w, s, stab, ovr, neg, tared);
        } else {
            printf("  %-8ld [读取失败]\n", (long)(++count));
        }
        usleep(POLL_INTERVAL_MS * 1000);
    }
}

/* ============================================================
 *  主程序入口
 * ============================================================ */
void print_banner(void) {
    printf("\n");
    printf("======================================================\n");
    printf("  电子秤 Modbus RTU 测试联调程序\n");
    printf("  协议版本: mobus2.md V1.1\n");
    printf("  接口: UART 9600bps 8-N-1  (TTL)\n");
    printf("======================================================\n");
}

void print_usage(const char *prog) {
    printf("用法: %s [选项]\n", prog);
    printf("选项:\n");
    printf("  -d <设备>   串口设备路径 (默认: %s)\n", UART_DEVICE_DEFAULT);
    printf("  -a <地址>   从机地址 hex (默认: 0x%02X)\n", MODBUS_SLAVE_ADDR);
    printf("  -t <编号>   运行指定测试 (1-13), 不指定则运行全部\n");
    printf("  -h          显示帮助信息\n");
    printf("\n可用测试 (基于 mobus2.md V1.1):\n");
    printf("  1  - 读取净重值 (0x0000~0x0001)\n");
    printf("  2  - 读取状态字 (0x0002)\n");
    printf("  3  - 读取ADC原始值 (0x0003~0x0004)\n");
    printf("  4  - 批量读净重+状态 (协议6.1示例)\n");
    printf("  5  - 净重稳定性测试\n");
    printf("  6  - 去皮操作 (功能码0x06, 协议6.2示例)\n");
    printf("  7  - 读取校准系数 (0x0020~0x0021)\n");
    printf("  8  - 全寄存器扫描\n");
    printf("  9  - 异常: 非法功能码\n");
    printf("  10 - 异常: 非法地址\n");
    printf("  11 - CRC错误处理\n");
    printf("  12 - 综合工作流\n");
    printf("  13 - 实时监控模式 (循环)\n");
}

typedef void (*test_func_t)(int);

int main(int argc, char *argv[]) {
    const char *uart_device = UART_DEVICE_DEFAULT;
    uint8_t slave_addr = MODBUS_SLAVE_ADDR;
    int specific_test = 0;

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

    int fd = uart_open(uart_device);
    if (fd < 0) {
        fprintf(stderr, "\n致命错误: 无法初始化串口, 退出。\n");
        return 1;
    }

    test_func_t tests[] = {
        test_read_net_weight,         // 1
        test_read_status_word,        // 2
        test_read_adc_raw,            // 3
        test_read_weight_and_status,  // 4
        test_weight_stability,        // 5
        test_tare_operation,          // 6
        test_read_calibration_factor, // 7
        test_full_register_scan,      // 8
        test_exception_illegal_func,  // 9
        test_exception_illegal_addr,  // 10
        test_crc_error_handling,      // 11
        test_complete_workflow,       // 12
        test_realtime_monitor         // 13
    };
    int num_tests = sizeof(tests) / sizeof(tests[0]);

    if (specific_test > 0 && specific_test <= num_tests) {
        printf("\n▶ 运行测试 %d...\n", specific_test);
        tests[specific_test - 1](fd);
    } else {
        printf("\n▶ 运行全部 %d 个测试 (跳过#13实时监控, 需单独调用)...\n", num_tests - 1);
        for (int i = 0; i < num_tests - 1; i++) {
            tests[i](fd);
        }
    }

    /* 打印总结 */
    printf("\n");
    printf("======================================================\n");
    printf("  测试汇总: 总计=%d  通过=%d  失败=%d\n",
           g_total_tests, g_passed_tests, g_failed_tests);
    printf("======================================================\n");

    if (g_failed_tests > 0) {
        printf("\n[WARN] %d 个测试未通过, 请检查硬件连接和模块配置。\n", g_failed_tests);
    } else {
        printf("\n[OK] 所有测试通过! Modbus RTU 通信正常。\n");
    }

    uart_close(fd);
    return (g_failed_tests > 0) ? 1 : 0;
}
