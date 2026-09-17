#include "App.h"

//上下文
vofamanager_csx vofamanager_t;
Maxi_csx maxi_connect_t;
Openmv_csx openmv_connect_t;
DMG6220 dmg6220_t;
hwt101_csx hwt101_t;

//一些接收数据的变量
volatile Maxi_posion_package Maxi_posion_data={.state=1}; //从maxicam得到的坐标数据
volatile Openmv_package openmv_data={.state=1}; //从openmv得到的巡线数据
uint8_t usart1_rx;
int a;
volatile uint8_t update=0;

void App_Init(void)
{
    DMG6220_Init(&dmg6220_t, &hcan1, 0x01);
    DMG6220_Enable(&dmg6220_t);
    DMG6220_MoveAngle(&dmg6220_t, 90.0f, 0.0f);

    printf("Booting\r\n");
    
    HAL_Delay(1000); //等待整个系统稳定

    //中间层初始化
    VOFA_Init(&vofamanager_t);
    Maxi_Init(&maxi_connect_t);
    Openmv_Init(&openmv_connect_t);

    //硬件初始化
    // ssd1306_Init(&hi2c2);
    // ssd1306_Fill(Black);
    // ssd1306_WriteString("Bury",Font_16x26,White);
    // ssd1306_UpdateScreen(&hi2c2);

    hwt101_Init(&hwt101_t,&huart2);

    //Gyro_integrate_Init(&mpu6050_t);

    Servo_Init();
    Servo_Set(90);

    //Emm_Home(0x01,EMM_SYNC_DISABLE);

    //Emm_Speed_Mode(0x01,3000,100,EMM_SYNC_DISABLE);

    //HAL_TIM_Base_Start_IT(&htim7);
    HAL_UART_Receive_IT(&huart1,&usart1_rx,1); //开启串口中断接收

    Robot_set_move(100,100,50);
    Led_on();

}
void App_Run(void)
{
    static uint32_t last_update = 0;
    char buffer[128];
    
    if(HAL_GetTick() - last_update > 100)
    {
        last_update = HAL_GetTick();

        //处理视觉给来的数据包
        /*
        if(Maxi_posion_data.state==0)
        {
            sprintf(buffer,"%d %d",Maxi_posion_data.x,Maxi_posion_data.y);
            ssd1306_Fill(Black);
            ssd1306_SetCursor(0,0);
            ssd1306_WriteString(buffer,Font_7x10,White);
            ssd1306_UpdateScreen(&hi2c2);
        }
        if(openmv_data.state==0)
        {
            sprintf(buffer,"%d %d",openmv_data.line_error,openmv_data.turning_state);
            ssd1306_Fill(Black);
            ssd1306_SetCursor(0,0);
            ssd1306_WriteString(buffer,Font_7x10,White);
            ssd1306_UpdateScreen(&hi2c2);
        }
        */
    }

}
//收到调试包
void VOFA_Get_Package_Callback(vofamanager_csx *csx,char *sign,float data)
{
    // Led_on();
    // Led_off();
    if(csx==&vofamanager_t)
    {
        if(strcmp(sign,"motor")==0)
        {
            a=(int)data;
            //Emm_Speed_Mode(0x01,a,0,EMM_SYNC_DISABLE);
            Servo_Set(a);
        }
        if(strcmp(sign,"Home")==0)
        {
            //Emm_Home(0x01,EMM_SYNC_DISABLE);
            float data;
            if(hwt101_read_yaw(&hwt101_t,&data)!=0) return;
            printf("%.2f\r\n",data);
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
//DMA空闲中断
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    HAL_UART_RxEventTypeTypeDef event = HAL_UARTEx_GetRxEventType(huart);

    if (huart->Instance == USART6 && event == HAL_UART_RXEVENT_IDLE)
    {
        Led_on();
        Maxi_posion_data=Maxi_Feed(&maxi_connect_t,Size); //喂数据
        HAL_UARTEx_ReceiveToIdle_DMA(&MAXI_UART, maxi_connect_t.rx_buf, MAXI_CONNECT_RX_BUFFER_SIZE); //重启DMA接收
    }
    if(huart->Instance == UART5)
    {
        
        openmv_data=Openmv_Feed(&openmv_connect_t,Size); //喂数据
        HAL_UARTEx_ReceiveToIdle_DMA(&OPENMV_UART, openmv_connect_t.rx_buf, OPENMV_CONNECT_RX_BUFFER_SIZE); //重启DMA接收
    }
    /* HWT101连续输出时不一定产生IDLE事件。缓冲区收满11字节时，
       使用TC事件结束本帧，并立即重启下一帧接收。 */
    if (huart->Instance == USART2 &&
        (event == HAL_UART_RXEVENT_IDLE || event == HAL_UART_RXEVENT_TC))
    {
        hwt101_Feed(&hwt101_t,Size);
        Led_on();
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
}
