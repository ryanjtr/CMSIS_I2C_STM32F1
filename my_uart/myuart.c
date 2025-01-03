/*
 * myuart.c
 *
 *  Created on: Nov 16, 2024
 *      Author: dell
 */

#include "myuart.h"

static void uart_print(const char *str)
{
  uint32_t length = strlen(str);
  for (uint32_t i = 0; i < length; i++)
  {
    while (!LL_USART_IsActiveFlag_TXE(USART2))
      ;
    LL_USART_TransmitData8(USART2, (uint8_t)str[i]);
  }
  while (!LL_USART_IsActiveFlag_TC(USART2))
    ;
}

void uart_printf(const char *format, ...)
{
  char buffer[128];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  uart_print(buffer);
}