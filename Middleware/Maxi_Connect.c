#include "Maxi_Connect.h"

/**
 * @brief 初始化和MaxiCam的通信通道
 * 
 * @param csx 上下文
 */
void Maxi_Init(Maxi_csx *csx)
{
    //数据会自动转移到上下文的rx_buf里面
    HAL_UARTEx_ReceiveToIdle_DMA(&MAXI_UART, csx->rx_buf, MAXI_CONNECT_RX_BUFFER_SIZE);
}

/** 
 * @brief 在中断里面调用，用来解析数据帧
 * 
 * @param csx 上下文
 * @param size 返回的数据长度，这里实际上没有用，只是预留的
 * @return Maxi_posion_package 返回的结构体，里面可以拿到目标坐标
 */
Maxi_posion_package Maxi_Feed(Maxi_csx *csx,uint16_t size)
{
    (void)size;
    Maxi_posion_package r={.state=1,.x=UINT16_MAX,.y=UINT16_MAX,.position=UINT8_MAX}; //初始化

    if(csx->rx_buf[0] == 0xAA && csx->rx_buf[6] == 0xD0) //检验包头包尾
    {
        r.x=(csx->rx_buf[1] << 8) | csx->rx_buf[2]; //拼接得到x坐标
        r.y=(csx->rx_buf[3] << 8) | csx->rx_buf[4]; //拼接得到y坐标
        r.position=csx->rx_buf[5]; //保存目标位置
        r.state=0; //状态正常
        return r;
    }
    else
    {
        return r; //返回异常
    }    
}
/**
 * @brief 向MaxiCam发送命令码
 * 
 * @param cmd 命令
 */
void Maxi_Send_cmd(uint8_t cmd)
{
    HAL_UART_Transmit(&MAXI_UART,&cmd,1,HAL_MAX_DELAY);
}
