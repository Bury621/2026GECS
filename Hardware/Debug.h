#ifndef __DEBUG_H
#define __DEBUG_H
#include "stdio.h"
#include "stdint.h"

#define DEBUG_UART huart1 //设置重定向的串口

int fputc(int ch, FILE *f);
int fgetc(FILE *f);
void print_binary(uint8_t value);
#endif