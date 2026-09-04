#include "App.h"
#include "string.h"
uint8_t usart1_rx;
vofamanager_csx vofamanager_t;

float Kp=0;
void App_Init(void)
{
    printf("Booting\r\n");
    Led_off();

    ssd1306_Init(&hi2c2);
    ssd1306_WriteString("Hello world",Font_7x10,White);
    ssd1306_UpdateScreen(&hi2c2);

    VOFA_Init(&vofamanager_t);//初始化一下vofamanager
    HAL_UART_Receive_IT(&huart1,&usart1_rx,1);
}
void App_Run(void)
{
    static uint32_t last_update = 0;
    char buffer[128];
    sprintf(buffer,"%.2f",Kp);
    ssd1306_Fill(Black);
    ssd1306_SetCursor(0,0);
    ssd1306_WriteString(buffer,Font_16x26,White);
    ssd1306_UpdateScreen(&hi2c2);
    if(HAL_GetTick() - last_update > 100)  // 100ms刷新一次
    {
        // 更新显示
        last_update = HAL_GetTick();
    }
}
void VOFA_Get_Package_Callback(vofamanager_csx *csx,char *sign,float data)
{
    Kp=data;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance==USART1)
    {
        VOFA_Recevice_Callback(&vofamanager_t,usart1_rx); //回调一下
        HAL_UART_Receive_IT(&huart1,&usart1_rx,1);
    }
}