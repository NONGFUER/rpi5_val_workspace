#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

int main() {
    int uart0_filestream = -1;
    // 树莓派 5 默认串口
    uart0_filestream = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NDELAY);
    
    if (uart0_filestream == -1) {
        perror("错误：无法打开串口");
        return -1;
    }

    struct termios options;
    tcgetattr(uart0_filestream, &options);
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);
    options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush(uart0_filestream, TCIFLUSH);
    tcsetattr(uart0_filestream, TCSANOW, &options);

    char tx_buffer[] = "Hello RPi5 UART!";
    printf("发送: %s\n", tx_buffer);
    
    int count = write(uart0_filestream, &tx_buffer[0], strlen(tx_buffer));
    if (count < 0) perror("发送失败");

    usleep(100000); // 等待数据回传

    char rx_buffer[255];
    int rx_length = read(uart0_filestream, (void*)rx_buffer, 255);
    
    if (rx_length < 0) {
        printf("接收错误。\n");
    } else if (rx_length == 0) {
        printf("未接收到数据，请检查引脚 8 和 10 是否短接。\n");
    } else {
        rx_buffer[rx_length] = '\0';
        printf("接收到 %d 字节: %s\n", rx_length, rx_buffer);
    }

    close(uart0_filestream);
    return 0;
}