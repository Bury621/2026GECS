#include "Ray.h"

void Ray_on(void)
{
    HAL_GPIO_WritePin(GPIOD,GPIO_PIN_11,1);
}
void Ray_off(void)
{
    HAL_GPIO_WritePin(GPIOD,GPIO_PIN_11,0);
}