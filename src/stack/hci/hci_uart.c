/**
 ****************************************************************************************
 * @file    hci_uart.c
 * @brief

 * @version 0.01
 * @date    2018/10/18
 * @history
 * @note
 * detailed description

 ****************************************************************************************
 * @attention
 *
 * Copyright (C) 2016 Shanghai Panchip Microelectronics Co., Ltd. All rights reserved.
 ****************************************************************************************
 */

#include "panip_config.h"

#include "hci_uart.h"
#include "uart.h"
#include "stack_svc_api.h"

#if DBG_HCI_UART_ENABLE
#define DBG_HCI_UART(x)     printf x
#else
#define DBG_HCI_UART(x)
#endif

extern void UART0_SendBuf ( const uint8_t* buf, uint32_t len );
extern void UART1_SendBuf ( const uint8_t* buf, uint32_t len );

#define APP_FIFO_BUF_MASK   0xFF
#define APP_FIFO_BUF_SIZE   256

typedef struct
{
    uint8_t  buf[APP_FIFO_BUF_SIZE];
    uint16_t pos_in;
    uint16_t pos_out;
} app_fifo_t;

static app_fifo_t m_hci_uart_fifo;

void app_fifo_init ( app_fifo_t* p_fifo )
{
    p_fifo->pos_in  = 0;
    p_fifo->pos_out = 0;
}

uint8_t app_fifo_is_empty ( app_fifo_t* const p_fifo )
{
    return ( p_fifo->pos_in == p_fifo->pos_out );
}

uint8_t app_fifo_is_full ( app_fifo_t* const p_fifo )
{
    return ( ( ( p_fifo->pos_in + 1 ) & APP_FIFO_BUF_MASK ) == p_fifo->pos_out );
}

uint16_t app_fifo_get_data_size ( app_fifo_t* const p_fifo )
{
    uint16_t pos_in    = p_fifo->pos_in;
    uint16_t pos_out   = p_fifo->pos_out;
    uint16_t data_size = 0;

    if ( pos_in >= pos_out )
    {
        data_size = pos_in - pos_out;
    }
    else
    {
        data_size = pos_in + APP_FIFO_BUF_SIZE - pos_out;
    }

    return data_size;
}


uint16_t app_fifo_get_free_size ( app_fifo_t* const p_fifo )
{
    return APP_FIFO_BUF_SIZE - app_fifo_get_data_size ( p_fifo );
}

uint32_t app_fifo_write ( app_fifo_t* p_fifo, const void* p_data, uint16_t length )
{
    uint16_t i     = 0;
    uint8_t* p_src = ( uint8_t* ) p_data;

    if ( app_fifo_get_free_size ( p_fifo ) < length )
    {
        return SDK_ERROR_NO_MEMORY;
    }

    for ( i = 0; i < length; i++ )
    {
        p_fifo->buf[p_fifo->pos_in] = p_src[i];

        p_fifo->pos_in++;
        p_fifo->pos_in &= APP_FIFO_BUF_MASK;
    }

    return SDK_SUCCESS;
}

uint32_t app_fifo_read ( app_fifo_t* p_fifo, const void* p_buffer, uint16_t length )
{
    uint16_t i     = 0;
    uint8_t* p_src = ( uint8_t* ) p_buffer;

    if ( app_fifo_get_data_size ( p_fifo ) < length )
    {
        return SDK_ERROR_NO_MEMORY;
    }

    for ( i = 0; i < length; i++ )
    {
        p_src[i] = p_fifo->buf[p_fifo->pos_out];

        p_fifo->pos_out++;
        p_fifo->pos_out &= APP_FIFO_BUF_MASK;
    }

    return SDK_SUCCESS;
}

/**
 * @brief HCI Uart Init
 *
 */
void hci_uart_init ( void )
{
    app_fifo_init ( &m_hci_uart_fifo );

    UART_InitTypeDef Init_Struct;

    /* Set P2 multi-function pins for UART1 RXD, TXD */
    SYS->P2_MFP |= SYS_MFP_P24_UART1_RXD | SYS_MFP_P25_UART1_TXD; 
	GPIO_ENABLE_DIGITAL_PATH ( P2, (1<<4) );
	GPIO_ENABLE_DIGITAL_PATH ( P2, (1<<5) );
	CLK_EnableModuleClock ( UART1_MODULE ); 
    
    /* Init UART to 115200-8n1 for print message */
    Init_Struct.UART_BaudRate = 115200;
    Init_Struct.UART_LineCtrl = Uart_line_8n1;

    UART_Init ( UART1, &Init_Struct );

    UART_ResetRxFifo ( UART1 );
    UART_EnableFifo ( UART1 );
    
	
	UART_EnableIrq ( UART1, Uart_irq_erbfi );
    
	NVIC_ClearPendingIRQ ( UART1_IRQn );
	NVIC_EnableIRQ ( UART1_IRQn );
	((interrupt_register_handler)SVC_interrupt_register)(UART1_IRQ,UART1_Handler);		//register uart0 interrupt callback function
    
    DBG_HCI_UART ( ( "uart1 init sucess \r\n" ) );
}

uint32_t hci_uart_rx_fifo_write ( const void* p_data, uint16_t length )
{
    return app_fifo_write ( &m_hci_uart_fifo, p_data, length );
}

uint32_t hci_uart_rx_fifo_read ( void* p_buffer, uint16_t length )
{
    return app_fifo_read ( &m_hci_uart_fifo, p_buffer, length );
}

uint32_t hci_uart_send ( const void *p_buffer, uint16_t length )
{
    UART1_SendBuf ( p_buffer, length );
    return 0;
}

void hci_uart_fifo_dbg( void )
{
    DBG_HCI_UART ( ( "fifo.pos_in %d\n", m_hci_uart_fifo.pos_in ) );
    DBG_HCI_UART ( ( "fifo.pos_out %d\n", m_hci_uart_fifo.pos_in ) );
    
    for (int i = 0; i < APP_FIFO_BUF_SIZE; i++)
    {
        DBG_HCI_UART ( ( "%02X ", m_hci_uart_fifo.buf[i] ) );
    }
    
    DBG_HCI_UART ( ( "\n" ) );
}

#include "hci_uart_task.h"
void UART1_Handler ( void )
{
    uint32_t int_status = UART_GetActiveEvent ( UART1 );

    if ( int_status & Uart_event_data )
    {
        while ( !UART_IsRxFifoEmpty ( UART1 ) )
        {
            uint8_t ch = UART_ReceiveData ( UART1 );
            hci_uart_rx_fifo_write ( &ch, 1 );
        }
        
        //hci_uart_fifo_dbg();
        
        hci_uart_data_recv ( 0, 0, 0, 0 );
        
      //  struct hci_uart_task *msg = KE_MSG_ALLOC( HCI_UART_DATA_RECV, TASK_HCI_UART, TASK_NONE, hci_uart_task );
        //msg->length = 0;
		//ke_msg_send(msg);
    }
    
}

