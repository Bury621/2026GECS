#include "Led.h"
#include "main.h"

void Led_on(void)
{
    HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,0);
}
void Led_off(void)
{
    HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,1);
}