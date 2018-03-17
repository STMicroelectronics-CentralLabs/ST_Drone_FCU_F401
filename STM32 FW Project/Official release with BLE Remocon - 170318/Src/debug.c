#include <stdio.h>
#include <stdarg.h>
#include "debug.h"

#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef huart1;

#ifdef DEBUG
int myprintf(const char *format, ...)
{
    va_list arg;
    va_start(arg, format);
    char temp[255];
    int len;
    // Limit the length of string to 254
    len = vsnprintf(temp, 254, format, arg);
    usart_puts(temp, len);
    return len;
}
    
int usart_puts(const char *str, int len) 
{
    //putc(*str ++);
    //while (huart1.Lock == HAL_LOCKED);
    HAL_UART_Transmit(&huart1, (uint8_t *)str, len, 1000);
    return 0;
}

#endif
