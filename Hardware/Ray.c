#include "Ray.h"

void Ray_on(void)
{
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,1);
}
void Ray_off(void)
{
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,0);
}