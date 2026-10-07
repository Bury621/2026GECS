#include "App.h"

//上下文
vofamanager_csx vofamanager_t;
Maxi_csx maxi_connect_t;
Openmv_csx openmv_connect_t;
DMG6220 dmg6220_t;
hwt101_csx hwt101_t;
PIDController robot_angle_pid_t; //角度环
PIDController robot_y_pid_t; //水平位置环
PIDController robot_task1_ball_pid_t; //任务一的控制环
PIDController robot_task2_target_pid_t; //任务二的控制环
PIDController robot_task3_hostage_pid_t; //任务三的控制环

//接收数据的变量
volatile Maxi_posion_package Maxi_posion_data={.state=1}; //从maxicam得到的坐标数据
volatile Openmv_package openmv_data={.state=1}; //从openmv得到的巡线数据
uint8_t usart1_rx;
int a;
static volatile float robot_diff_turn_radius_cm = 50.0f;
static volatile float robot_diff_turn_target_angle_deg = 0.0f;
static volatile uint8_t robot_diff_turn_start_request = 0;
static volatile uint8_t robot_diff_turn_stop_request = 0;

//标志位
extern Robot_remote_sign remote_sign; //小车遥控标志位
volatile uint8_t tim7_100ms_trigged_sign=0; //角度环更新标志位
volatile uint8_t tim10_10ms_trigged_sign=0; //小车状态更新标志位
volatile uint8_t debug_start_flag=0; //上位机发送start:1后才启动

//一些变量
volatile uint16_t robot_angle=0; //角度环的目标角度
volatile uint16_t robot_y=26; //水平位置环的目标

volatile uint8_t robot_y_pid_is_open=0; //是否开启水平位置环
void App_Init(void)
{
    printf("Booting...\n");
    for(int x=0;x<20;x++)//等待整个系统稳定
    {
        HAL_Delay(50);
        Led_toggle();
        
    }

    //中间层初始化
    VOFA_Init(&vofamanager_t);
    Maxi_Init(&maxi_connect_t);
    Openmv_Init(&openmv_connect_t);
    PIDController_Init(&robot_angle_pid_t);
    robot_angle_pid_t.Kp=1.0f;
    robot_angle_pid_t.Ki=0.019f;
    robot_angle_pid_t.Kd=0.101f;
    robot_angle_pid_t.tau=0.05f;
    robot_angle_pid_t.deadband=0.3f;
    robot_angle_pid_t.limMin=-1000.0f;
    robot_angle_pid_t.limMax=1000.0f;
    robot_angle_pid_t.limMinInt=-20.0f;
    robot_angle_pid_t.limMaxInt=20.0f;
    robot_angle_pid_t.T=0.1f;

    PIDController_Init(&robot_y_pid_t);
    robot_y_pid_t.Kp=1.2f;
    robot_y_pid_t.Ki=0.0f;
    robot_y_pid_t.Kd=0.0f;
    robot_y_pid_t.tau=0.05f;
    robot_y_pid_t.deadband=2;
    robot_y_pid_t.limMin=-1000.0f;
    robot_y_pid_t.limMax=1000.0f;
    robot_y_pid_t.limMinInt=-20.0f;
    robot_y_pid_t.limMaxInt=20.0f;
    robot_y_pid_t.T=0.1f;

    PIDController_Init(&robot_task1_ball_pid_t);
    robot_task1_ball_pid_t.Kp=0.3f;
    robot_task1_ball_pid_t.Ki=0.0012f;
    robot_task1_ball_pid_t.Kd=0.0f;
    robot_task1_ball_pid_t.tau=0.05f;
    robot_task1_ball_pid_t.deadband=2;
    robot_task1_ball_pid_t.limMin=-50.0f;
    robot_task1_ball_pid_t.limMax=50.0f;
    robot_task1_ball_pid_t.limMinInt=-20.0f;
    robot_task1_ball_pid_t.limMaxInt=20.0f;
    robot_task1_ball_pid_t.T=0.1f;

    PIDController_Init(&robot_task2_target_pid_t);
    robot_task2_target_pid_t.Kp=0.3f;
    robot_task2_target_pid_t.Ki=0.015f;
    robot_task2_target_pid_t.Kd=0.001f;
    robot_task2_target_pid_t.tau=0.05f;
    robot_task2_target_pid_t.deadband=2;
    robot_task2_target_pid_t.limMin=-50.0f;
    robot_task2_target_pid_t.limMax=50.0f;
    robot_task2_target_pid_t.limMinInt=-20.0f;
    robot_task2_target_pid_t.limMaxInt=20.0f;
    robot_task2_target_pid_t.T=0.1f;

    PIDController_Init(&robot_task3_hostage_pid_t);
    robot_task3_hostage_pid_t.Kp=0.3f;
    robot_task3_hostage_pid_t.Ki=0.015f;
    robot_task3_hostage_pid_t.Kd=0.001f;
    robot_task3_hostage_pid_t.tau=0.05f;
    robot_task3_hostage_pid_t.deadband=2;
    robot_task3_hostage_pid_t.limMin=-50.0f;
    robot_task3_hostage_pid_t.limMax=50.0f;
    robot_task3_hostage_pid_t.limMinInt=-20.0f;
    robot_task3_hostage_pid_t.limMaxInt=20.0f;
    robot_task3_hostage_pid_t.T=0.1f;
    
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
    HAL_Delay(100);
    //DMG6220_MoveAngle(&dmg6220_t, 280, 120.0f);

    Robot_set_move(0,0,0); //停车

    Ray_off();
    Robot_Claw_close();

    //片上外设初始化
    HAL_TIM_Base_Start_IT(&htim7); //开启tim7
    HAL_TIM_Base_Start_IT(&htim10); //开启tim7
    HAL_UART_Receive_IT(&huart1,&usart1_rx,1); //开启串口中断接收

    printf("Ready\n");

    //Robot_Tower_down();
    
}
void App_Run(void)
{
    DMG6220_Update(&dmg6220_t);
    DMG6220_Service(&dmg6220_t);

    if (robot_diff_turn_stop_request)
    {
        robot_diff_turn_stop_request = 0;
        Robot_stop_differential_turn();
    }
    if (robot_diff_turn_start_request)
    {
        robot_diff_turn_start_request = 0;
        Robot_start_differential_turn(robot_diff_turn_radius_cm,
                                      robot_diff_turn_target_angle_deg);
    }

    Robot_remote_control(); //处理遥控
    if(tim7_100ms_trigged_sign==1) //动态更新角度
    {
        tim7_100ms_trigged_sign=0;
        Robot_Angle_Update(&hwt101_t,&robot_angle_pid_t,robot_angle);

        float sensor_data;
        hwt101_read_yaw(&hwt101_t,&sensor_data);

        //printf("%.2f,%d\n",sensor_data,robot_angle);
        Openmv_package now_y_data=openmv_data;
        //y闭环
        if(robot_y_pid_is_open && now_y_data.line_error != 0)
        {
            if(now_y_data.turning_state)
            {
                Robot_set_vy(0);

            }
            else
            {
                Robot_set_vy(PIDController_Update(&robot_y_pid_t,robot_y,now_y_data.line_error));
                printf("%d\n",now_y_data.line_error);
            }
        }
            
        printf("%d,%d,%d\n",Maxi_posion_data.x,Maxi_posion_data.y,Maxi_posion_data.position);
        //printf("%d\n",Emm_Read_Pulse_Count(0x01));
    }
    if(tim10_10ms_trigged_sign==1)
    {
        tim10_10ms_trigged_sign=0;
        if(debug_start_flag==1)
        {
            Robot_tick();
        }
        
    }
    
}

//收到调试包
void VOFA_Get_Package_Callback(vofamanager_csx *csx,char *sign,float data)
{
    if(csx==&vofamanager_t)
    {
        if(strcmp(sign,"motor")==0)
        {
            DMG6220_MoveAngle(&dmg6220_t, (int)data, 145.0f);
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
        if(strcmp(sign,"start")==0)
        {
            debug_start_flag=1;
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
                robot_y_pid_is_open=0;
            }
            else
            {
                remote_sign=255;
                //robot_y_pid_is_open=1;
            }
        }
        if(strcmp(sign,"right")==0)
        {
            if(((int)data)==1)
            {
                remote_sign=4;
                robot_y_pid_is_open=0;
            }
            else
            {
                remote_sign=255;
                //robot_y_pid_is_open=1;
            }
        }
        if(strcmp(sign,"Stop")==0)
        {
            remote_sign=255;
            robot_diff_turn_stop_request=1;
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
        if(strcmp(sign,"diff_radius")==0)
        {
            robot_diff_turn_radius_cm=data;
        }
        if(strcmp(sign,"diff_turn")==0)
        {
            robot_diff_turn_target_angle_deg=data;
            robot_diff_turn_start_request=1;
        }
        if(strcmp(sign,"high")==0)
        {
            Emm_position_Mode(0x06,100,0,data,EMM_MODE_RELATIVE_TO_PREVIOUS,EMM_SYNC_DISABLE);
        }
        if(strcmp(sign, "tower_down") == 0)
        {
            Robot_Tower_down();
        }
        if(strcmp(sign, "tower_up") == 0)
        {
            Robot_Tower_up();
        }
        if(strcmp(sign, "claw_out") == 0)
        {
            Robot_Claw_out1();
        }
        if(strcmp(sign, "claw_in") == 0)
        {
            Robot_Claw_in();
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
        tim7_100ms_trigged_sign=1; //置标志位
    }
    if(htim->Instance==TIM10)
    {
        tim10_10ms_trigged_sign=1;
    }
}
