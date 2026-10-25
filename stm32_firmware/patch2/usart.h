/* PC 端 usart.h 桩：固件通过 LOG 宏输出调试信息，PC 上重定向到 printf */
#ifndef __USART_H_PC__
#define __USART_H_PC__
#include <stdio.h>
#define LOG(fmt, ...) printf(fmt, ##__VA_ARGS__)
#endif
