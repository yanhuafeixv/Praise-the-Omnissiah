#include <stdio.h>
#include <unistd.h>
#include "serial.h"

int main(void)
{
    /* 初始化 UART1（设备节点请用 dmesg | grep tty 确认） */
    if (Serial_Init("/dev/ttyS1", 9600) != 0) {
        fprintf(stderr, "Failed to init serial port!\n");
        return 1;
    }

    /* 启动接收线程 */
    Serial_StartRx();

    /* 发送一条 AT 测试指令 */
    Serial_SendString("AT\r\n");
    sleep(1);

    /* 主循环：等价于 STM32 中 while(1) 的使用方式 */
    while (1) {
        if (Serial_RxFlag) {
            /* 收到一帧 [xxx] 格式的数据 */
            printf("Received packet: %s\n", Serial_RxPacket);

            /* 回显 */
            Serial_Printf("[%s]\r\n", Serial_RxPacket);

            /* 清除标志，准备接收下一帧 */
            Serial_RxFlag = 0;
        }

        /* 可以在这里添加其他业务逻辑 */
        usleep(10000);  /* 10ms，降低 CPU 占用 */
        Serial_Printf("123\r\n");

    }

    Serial_Close();
    return 0;
}
