#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdint.h>

// CRC-16/MODBUS 计算函数 (查表法效率高)
uint16_t crc16_modbus(uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < len; i++) {
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

// 串口初始化函数
int uart_init(const char *device) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        perror("无法打开串口");
        return -1;
    }

    struct termios options;
    tcgetattr(fd, &options);
    
    // 设置波特率 9600
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);

    // 8N1 (8位数据, 无校验, 1位停止位)
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    
    // 原始模式 (不做特殊处理)
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;

    tcsetattr(fd, TCSANOW, &options);
    return fd;
}

int main() {
    // 树莓派 5 默认串口
    const char *dev = "/dev/ttyAMA0";
    int fd = uart_init(dev);
    if (fd < 0) return 1;

    // 1. 准备原始数据包 (假设：站号 0x01, 功能码 0x03, 地址 0x0001)
    uint8_t payload[] = {0x01, 0x03, 0x00, 0x01};
    uint16_t payload_len = sizeof(payload);
    
    // 2. 计算并附加 CRC (Modbus 习惯低位在前)
    uint16_t crc = crc16_modbus(payload, payload_len);
    uint8_t tx_buf[6]; 
    memcpy(tx_buf, payload, payload_len);
    tx_buf[payload_len] = crc & 0xFF;         // CRC 低位
    tx_buf[payload_len + 1] = (crc >> 8) & 0xFF; // CRC 高位

    printf("--- [发送启动] ---\n数据内容: ");
    for(int i=0; i<6; i++) printf("%02X ", tx_buf[i]);
    printf("\nCRC 校验值: %04X\n", crc);

    // 3. 发送数据
    write(fd, tx_buf, 6);

    // 4. 延迟等待回环读取
    usleep(200000); 

    // 5. 接收并处理
    uint8_t rx_buf[255];
    int rx_len = read(fd, rx_buf, 255);

    if (rx_len > 0) {
        printf("\n--- [接收成功] ---\n长度: %d 字节\n数据内容: ", rx_len);
        for(int i=0; i<rx_len; i++) printf("%02X ", rx_buf[i]);
        printf("\n");

        // 6. 验证 CRC
        if (rx_len >= 3) { // 至少需要 数据 + 2字节CRC
            uint16_t calc_crc = crc16_modbus(rx_buf, rx_len - 2);
            uint16_t recv_crc = rx_buf[rx_len - 2] | (rx_buf[rx_len - 1] << 8);

            printf("收到校验值: %04X, 重新计算校验值: %04X\n", recv_crc, calc_crc);

            if (calc_crc == recv_crc) {
                printf("结果: [PASS] CRC 校验匹配，数据完整。\n");
            } else {
                printf("结果: [FAIL] CRC 校验不匹配，数据损坏！\n");
            }
        }
    } else {
        printf("\n结果: [FAIL] 未接收到数据，请检查引脚 8 和 10 短接情况。\n");
    }

    close(fd);
    return 0;
}