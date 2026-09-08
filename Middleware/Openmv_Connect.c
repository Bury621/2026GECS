#include "Openmv_Connect.h"

/**
 * @brief 初始化和Openmv的通信通道
 * 
 * @param csx 上下文
 */
void Openmv_Init(Openmv_csx *csx)
{
    //数据会自动转移到上下文的rx_buf里面
    HAL_UARTEx_ReceiveToIdle_DMA(&OPENMV_UART, csx->rx_buf, OPENMV_CONNECT_RX_BUFFER_SIZE);
}

/** 
 * @brief 在中断里面调用，用来解析数据帧
 * 
 * @param csx 上下文
 * @param size 返回的数据长度，这里实际上没有用，只是预留的
 * @return OPENMV_package 返回的结构体，里面可以拿到目标数据
 */
Openmv_package Openmv_Feed(Openmv_csx *csx,uint16_t size)
{
    (void)size;
    Openmv_package r={.state=1,.line_error=UINT8_MAX,.turning_state=UINT8_MAX}; //初始化

    if(csx->rx_buf[0] == 0xAA && csx->rx_buf[3] == 0xD0) //检验包头包尾
    {
        r.line_error=csx->rx_buf[1]; //保存到边线的距离
        r.turning_state=csx->rx_buf[2]; //保存是否在转弯状态
        r.state=0; //状态正常
        return r;
    }
    else
    {
        return r; //返回异常
    }    
}