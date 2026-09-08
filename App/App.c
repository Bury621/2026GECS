#include "App.h"

uint8_t usart1_rx;
vofamanager_csx vofamanager_t;

int a;
void App_Init(void)
{
    printf("Booting\r\n");

    HAL_Delay(500); //等待整个系统稳定
    
    Led_off();

    ssd1306_Init(&hi2c2);
    ssd1306_WriteString("Hello world",Font_7x10,White);
    ssd1306_UpdateScreen(&hi2c2);

    VOFA_Init(&vofamanager_t);//初始化一下vofamanager


    HAL_UART_Receive_IT(&huart1,&usart1_rx,1); //开启串口中断接收

    //Emm_Speed_Mode(0x01,100,0,EMM_SYNC_DISABLE);
}
void App_Run(void)
{
    static uint32_t last_update = 0;
    char buffer[128];

    if(HAL_GetTick() - last_update > 100)  // 100ms刷新一次
    {
        // 更新显示
        last_update = HAL_GetTick();

        sprintf(buffer,"%d",a);
        ssd1306_Fill(Black);
        ssd1306_SetCursor(0,0);
        ssd1306_WriteString(buffer,Font_16x26,White);
        ssd1306_UpdateScreen(&hi2c2);
    }
}
void VOFA_Get_Package_Callback(vofamanager_csx *csx,char *sign,float data)
{
    if(strcmp(sign,"motor")==0)
    {
        a=(int)data;
        Emm_Speed_Mode(0x01,a,0,EMM_SYNC_DISABLE);
    }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance==USART1)
    {
        VOFA_Recevice_Callback(&vofamanager_t,usart1_rx); //回调一下
        HAL_UART_Receive_IT(&huart1,&usart1_rx,1);
    }
}