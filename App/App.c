#include "App.h"

uint8_t usart1_rx;
vofamanager_csx vofamanager_t;
Maxi_csx maxi_connect_t;
volatile Maxi_posion_package Maxi_posion_data={.state=1}; //从maxicam得到的坐标数据
int a;

void App_Init(void)
{
    printf("Booting\r\n");
    HAL_Delay(500); //等待整个系统稳定

    //中间层初始化
    VOFA_Init(&vofamanager_t);//初始化一下vofamanager
    Maxi_Init(&maxi_connect_t);

    //硬件初始化
    ssd1306_Init(&hi2c2);
    ssd1306_Fill(Black);

    Emm_Speed_Mode(0x01,0,0,EMM_SYNC_DISABLE);

    HAL_UART_Receive_IT(&huart1,&usart1_rx,1); //开启串口中断接收
}
void App_Run(void)
{
    static uint32_t last_update = 0;
    char buffer[128];

    if(HAL_GetTick() - last_update > 100)
    {
        last_update = HAL_GetTick();
        if(Maxi_posion_data.state==0)
        {
            sprintf(buffer,"%d %d",Maxi_posion_data.x,Maxi_posion_data.y);
            ssd1306_Fill(Black);
            ssd1306_SetCursor(0,0);
            ssd1306_WriteString(buffer,Font_7x10,White);
            ssd1306_UpdateScreen(&hi2c2);
        }
    }
}
//收到调试包
void VOFA_Get_Package_Callback(vofamanager_csx *csx,char *sign,float data)
{
    Led_on();
    Led_off();
    if(csx==&vofamanager_t)
    {
        if(strcmp(sign,"motor")==0)
        {
            a=(int)data;
            Emm_Speed_Mode(0x01,a,0,EMM_SYNC_DISABLE);
        }
        if(strcmp(sign,"Home")==0)
        {
            Emm_Home(0x01,EMM_SYNC_DISABLE);
        }
    }
}
//喂调试数据
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance==USART1)
    {
        VOFA_Recevice_Callback(&vofamanager_t,usart1_rx); //回调一下
        HAL_UART_Receive_IT(&huart1,&usart1_rx,1);
    }
}
//DMA中断
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    Led_on();
    if (huart->Instance == USART6 && HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_IDLE)
    {
        //处理中断
        Maxi_posion_data=Maxi_Feed(&maxi_connect_t,Size); //喂数据
        HAL_UARTEx_ReceiveToIdle_DMA(&MAXI_UART, maxi_connect_t.rx_buf, MAXI_CONNECT_RX_BUFFER_SIZE); //别忘了重启DMA接收
    }
}