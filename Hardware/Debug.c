#include "main.h"
#include "Debug.h"
#include "stdio.h"

extern UART_HandleTypeDef DEBUG_UART;
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&DEBUG_UART, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}
int fgetc(FILE *f)
{
  uint8_t ch = 0;
  HAL_UART_Receive(&DEBUG_UART, &ch, 1, 0xffff);
  return ch;
}
void print_binary(uint8_t value) {
    for (int i = 7; i >= 0; i--) {  // 从高位到低位输出
        printf("%d", (value >> i) & 1);
        //if (i == 8) printf(" ");  // 可选：每4位加空格
    }
    printf("\r\n");
}