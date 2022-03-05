/*******************************************************************************
*                                 Apollo
*                       ---------------------------
*                       innovating embedded systems
*
* Copyright (c) 2001-2015 Guangzhou ZHIYUAN Electronics Stock Co., Ltd.
* All rights reserved.
*
* Contact information:
* web site:    http://www.zlg.cn/
* e-mail:      apollo.support@zlg.cn
*******************************************************************************/
/**
 * \file
 * \brief UART的中断模式例程，通过标准接口实现
 *
 * - 操作步骤：
 *   1. PIO0_0 引脚连接PC串口的TXD;
 *   2. PIO0_1 引脚连接PC串口的RXD。
 *
 * - 实验现象：
 *   1. 串口输出"UART interrupt mode test:"；
 *   2. 串口输出接收到的字符串。
 *
 * \note 例程没有使用传输缓冲区或则接收缓冲区（像环形缓冲区），只使用了为接收和
 *       发送而设置的用户数据缓冲区，因此每次发送或接收数据要等上次发送或接收数
 *       据完成后才能进行。
 *
 * \par 源代码
 * \snippet demo_std_uart_int.c src_std_uart_int
 *
 * \internal
 * \par Modification History
 * - 1.00 15-07-14  lielinhua, first implementation.
 * \endinternal
 */


/**
 * \addtogroup demo_if_std_uart_int
 * \copydoc demo_std_uart_int.c
 */

 /** [src_std_uart_int] */
#include "ametal.h"
#include "am_board.h"
#include "am_prj_config.h"

#include "..\XMODEM1K\xmodem1k.h"


/**
 * \brief 调试UART使用，用于未实现SWM，
 */
 
#define    USART_CH           AMHW_USART0

/** \brief 串口时钟分频数 */
#define AM_CFG_UARTCLKIDV            1

/**\brief 波特率 */
#define    USART_BAUDRATE     (115200)

 /* 定义串口（中断模式）设备结构体，用于分配相关空间 */
am_uart_int_dev_t  g_uart0_int_dev;

uint32_t g_systick_time_count = 0;

/**
 * \brief SYSTICK 中断处理函数
 * \return 无
 * \note SYSTICK中断处理属于内核中断，故无需调用
 *       am_int_connect() 和 am_int_enable() 函数。
 */
void SysTick_Handler(void)
{
    g_systick_time_count++;
}

int PktHandle(uint8_t *puData, uint16_t n)
{
    
    return 0;
}


uint32_t sysTimerGet(void)
{
    return g_systick_time_count;
}

void sysTimerClr(uint32_t i)
{
    /* 设置当前的值为0 */
    g_systick_time_count = 0;
}


/**
 * \brief 初始化USART
 */
void usart_init (void)
{
    /* 配置PIO0_0作为UART0_RXD */
    am_gpio_pin_cfg(PIO0_0, PIO0_0_PULLUP | PIO_FUNC_U0_RXD);

    /* 配置PIO0_1作为UART0_TXD */
    am_gpio_pin_cfg(PIO0_4, PIO0_4_PULLUP | PIO_FUNC_U0_TXD);
    
    /* UART时钟分频值为 1           */
    amhw_syscon_uartclkdiv_set(AM_CFG_UARTCLKIDV);
    /* 使能USART0时钟，并复位USART0 */
    amhw_clock_periph_enable(AMHW_CLOCK_UART0);
    amhw_syscon_periph_reset(AMHW_RESET_UART0);
    /* 配置串口*/         
    amhw_usart_config(USART_CH, AMHW_USART_CFG_8BIT       |              /* 8位数据位                   */
                                AMHW_USART_CFG_PARITY_NO  |              /* 无校验                      */
                                AMHW_USART_CFG_STOP_1     );             /* 1位停止位                   */
    /* 设置UART波特率 */
    amhw_usart_baudrate_set(USART_CH,
                            amhw_clock_periph_freq_get(USART_CH),
                            USART_BAUDRATE);
    /* 使能UART*/
    amhw_usart_enable(USART_CH);
    
}

uint8_t uartreadnoblock (uint8_t *p_ch)
{
    if ((AMHW_USART0->stat & AMHW_USART_STAT_RXRDY) != 0){
        *p_ch = amhw_usart_rxdata_read(AMHW_USART0);
        
        return 1;        
    }else{
        return 0;
    }
  
}

uint8_t uartsendnoblock (uint8_t *p_ch)
{
    if ((AMHW_USART0->stat & AMHW_USART_STAT_TXRDY) != 0){
        amhw_usart_txdata_write(AMHW_USART0, (uint32_t)(*p_ch));
        return 1;
    } else {
        return 0;
    }
}

xmodem_fun_handle_t gp_xmodem_fun ={
    uartreadnoblock,
    uartsendnoblock,
    sysTimerClr,
    sysTimerGet,
    NULL,
    
};

pfn_data_handle_t p_fun = NULL;

/**
 * \brief 主函数入口
 */
int main (void)
{

    /* 板级初始化  */
    am_board_init();
    
    usart_init();
    
    /* 使用系统时钟作为SYSTICK时钟源 */
    am_systick_config(AMHW_SYSTICK, AMHW_SYSTICK_CONFIG_CLKSRC_SYSTEM);
    
    /* 设置重载值，最大值为0xFFFFFF */
    am_systick_reload_val_set(AMHW_SYSTICK, g_system_clkfreq / 100);    
    
    /* 设置当前的值为0 */
    am_systick_val_set(AMHW_SYSTICK, g_system_clkfreq);
    
    /* 使能SYSTICK中断 */
    am_systick_int_enable(AMHW_SYSTICK);
    
    /* 使能SYSTICK，开始向下计数 */
    am_systick_enable(AMHW_SYSTICK);

//    /* 平台初始化 */
//    amhw_plfm_usart0_init();

//    /* USART初始化 */
//    uart_handle = amdr_usart_init(&g_usart0_dev, &g_usart0_devinfo);

//    /* USART初始化为中断模式 */
//    handle      = am_uart_int_init(&g_uart0_int_dev, uart_handle);

//    am_uart_int_send(handle,
//                     "UART interrupt mode test:\r\n",
//                     sizeof("UART interrupt mode test:\r\n"));

    while (1) {
        
        xmodem1k_client(p_fun,&gp_xmodem_fun,100,1000);

//        /* 接收一个字符 */
//        am_uart_int_receive(handle, uart0_buf, 3);

//        /* 发送刚刚接收的字符 */
//        am_uart_int_send(handle, uart0_buf, 1);
//        am_delay(1000);
        
    }
}

 /** [src_std_uart_int] */

/* end of file */
