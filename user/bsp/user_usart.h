#ifndef __USER_USART_H
#define __USER_USART_H

#include "main.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;

void ZDT_UART_RxStart(void);
void Vision_UART_Poll(void);
void Vision_UART_Start(void);

#endif
