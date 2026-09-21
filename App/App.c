#include "App.h"

//上下文
vofamanager_csx vofamanager_t;
Maxi_csx maxi_connect_t;
Openmv_csx openmv_connect_t;
DMG6220 dmg6220_t;
hwt101_csx hwt101_t;
PIDController robot_angle_pid_t; //角度环

//接收数据的变量
volatile Maxi_posion_package Maxi_posion_data={.state=1}; //从maxicam得到的坐标数据
volatile Openmv_package openmv_data={.state=1}; //从openmv得到的巡线数据
uint8_t usart1_rx;
int a;

//标志位
extern Robot_remote_sign remote_sign; //小车遥控标志位
volatile uint8_t robot_angle_update_sign=0; //角度环更新标志位

//一些变量
volatile uint16_t robot_angle=0; //角度环的目标角度

void App_Init(void)
{
    printf("Booting\r\n");
    HAL_Delay(1000); //等待整个系统稳定

    //中间层初始化
    VOFA_Init(&vofamanager_t);
    Maxi_Init(&maxi_connect_t);
    Openmv_Init(&openmv_connect_t);
    PIDController_Init(&robot_angle_pid_t);
    robot_angle_pid_t.Kp=6.81f;
    robot_angle_pid_t.Ki=0.0f;
    robot_angle_pid_t.Kd=3.91f;
    robot_angle_pid_t.tau=0.05f;
    robot_angle_pid_t.deadband=1.5f;
    robot_angle_pid_t.limMin=-1000.0f;
    robot_angle_pid_t.limMax=1000.0f;
    robot_angle_pid_t.limMinInt=-20.0f;
    robot_angle_pid_t.limMaxInt=20.0f;
    robot_angle_pid_t.T=0.01f;

    //硬件初始化
    ssd1306_Init(&hi2c2);
    ssd1306_Fill(Black);
    ssd1306_WriteString("Bury666",Font_16x26,White);
    ssd1306_UpdateScreen(&hi2c2);

    hwt101_Init(&hwt101_t,&huart2);
    HAL_Delay(100);
    hwt101_reset_yaw(&hwt101_t);

    Servo_Init();

    DMG6220_Init(&dmg6220_t, &hcan1, 0x01);
    DMG6220_Enable(&dmg6220_t);

    Robot_set_move(0,0,0); //停车

    //片上外设初始化
    HAL_TIM_Base_Start_IT(&htim7); //开启tim7
    HAL_UART_Receive_IT(&huart1,&usart1_rx,1); //开启串口中断接收

}
void App_Run(void)
{
    Robot_remote_control(); //处理遥控
    if(robot_angle_update_sign==1) //动态更新角度
    {
        robot_angle_update_sign=0;
        Robot_Angle_Update(&hwt101_t,&robot_angle_pid_t,robot_angle);

        // float data;
        // hwt101_read_yaw(&hwt101_zt,&data);
        // printf("%.2f\r\n",data);
    }
}

//收到调试包
void VOFA_Get_Package_Callback(vofamanager_csx *csx,char *sign,float data)
{
    if(csx==&vofamanager_t)
    {
        if(strcmp(sign,"motor")==0)
        {
            a=(int)data;
            DMG6220_MoveAngle(&dmg6220_t, a, 0.0f);
        }
        if(strcmp(sign,"servo")==0)
        {
            a=(int)data;
            Servo_Set(a);
        }
        if(strcmp(sign,"Get_yaw")==0)
        {
            float data;
            hwt101_read_yaw(&hwt101_t,&data);
            printf("%.2f\r\n",data);
        }
        if(strcmp(sign,"Reset_mcu")==0)
        {
            HAL_NVIC_SystemReset();
        }
        if(strcmp(sign,"forward")==0)
        {
            if(((int)data)==1)
            {
                remote_sign=1;
            }
            else
            {
                remote_sign=255;
            }
        }
        if(strcmp(sign,"backward")==0)
        {
            if(((int)data)==1)
            {
                remote_sign=2;
            }
            else
            {
                remote_sign=255;
            }
        }
        if(strcmp(sign,"left")==0)
        {
            if(((int)data)==1)
            {
                remote_sign=3;
            }
            else
            {
                remote_sign=255;
            }
        }
        if(strcmp(sign,"right")==0)
        {
            if(((int)data)==1)
            {
                remote_sign=4;
            }
            else
            {
                remote_sign=255;
            }
        }
        if(strcmp(sign,"Stop")==0)
        {
            remote_sign=255;
        }
        if(strcmp(sign,"Kp")==0)
        {
            robot_angle_pid_t.Kp=data;
        }
        if(strcmp(sign,"Ki")==0)
        {
            robot_angle_pid_t.Ki=data;
            
        }
        if(strcmp(sign,"Kd")==0)
        {
            robot_angle_pid_t.Kd=data; 
        }
        if(strcmp(sign,"angle")==0)
        {
            robot_angle=data;
        }
        if(strcmp(sign,"turn")==0)
        {
            robot_angle=(robot_angle+90) % 360;
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
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance==TIM7)
    {
        robot_angle_update_sign=1; //置标志位，更新角度环
    }
}