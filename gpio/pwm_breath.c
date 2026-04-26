#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <signal.h>

// 修改为你的系统识别到的 pwmchip0
#define PWM_BASE_PATH "/sys/class/pwm/pwmchip0/pwm0"
#define EXPORT_PATH   "/sys/class/pwm/pwmchip0/export"

// 周期设定：1,000,000 纳秒 = 1 毫秒 (1kHz)
#define PERIOD 1000000
#define STEP   10000    // 呼吸步长
#define DELAY_US 15000  // 呼吸速度控制

int fd_duty;

void sysfs_write(const char *path, const char *value) {
    int fd = open(path, O_WRONLY);
    if (fd < 0) {
        perror("写入 sysfs 失败");
        exit(1);
    }
    write(fd, value, strlen(value));
    close(fd);
}

void handle_sigint(int sig) {
    printf("\n停止 PWM 验证，正在清理并熄灭 LED...\n");
    sysfs_write(PWM_BASE_PATH "/duty_cycle", "0");
    sysfs_write(PWM_BASE_PATH "/enable", "0");
    if (fd_duty > 0) close(fd_duty);
    exit(0);
}

int main() {
    signal(SIGINT, handle_sigint);

    // 1. 导出 PWM0 通道
    if (access(PWM_BASE_PATH, F_OK) == -1) {
        printf("正在导出 PWM0 通道...\n");
        sysfs_write(EXPORT_PATH, "0");
        usleep(200000); // 给系统一点时间生成节点
    }

    // 2. 设置周期 (1ms = 1kHz)
    // 注意：如果之前设置过不匹配的值，可能需要先关闭 enable
    sysfs_write(PWM_BASE_PATH "/enable", "0"); 
    sysfs_write(PWM_BASE_PATH "/period", "1000000");

    // 3. 初始占空比
    sysfs_write(PWM_BASE_PATH "/duty_cycle", "0");

    // 4. 使能 PWM
    sysfs_write(PWM_BASE_PATH "/enable", "1");

    // 5. 打开占空比文件句柄
    fd_duty = open(PWM_BASE_PATH "/duty_cycle", O_RDWR);
    if (fd_duty < 0) {
        perror("打开 duty_cycle 失败");
        return 1;
    }

    printf("PWM 呼吸灯已启动 (pwmchip0/pwm0)... \n使用物理引脚 32 (GPIO 12)\n按下 Ctrl+C 退出\n");

    char buffer[20];
    while (1) {
        // 渐亮
        for (int i = 0; i <= PERIOD; i += STEP) {
            int len = snprintf(buffer, sizeof(buffer), "%d", i);
            pwrite(fd_duty, buffer, len, 0);
            usleep(DELAY_US);
        }
        // 渐灭
        for (int i = PERIOD; i >= 0; i -= STEP) {
            int len = snprintf(buffer, sizeof(buffer), "%d", i);
            pwrite(fd_duty, buffer, len, 0);
            usleep(DELAY_US);
        }
    }

    return 0;
}