/**
 ****************************************************************************************
 * @file    hci_uart.h
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
/** @defgroup Module Description
* @{
*/
#ifndef __HCI_UART_H__
#define __HCI_UART_H__


#include "sdk_errors.h"

#ifdef __cplusplus
#if __cplusplus
extern "C"{
#endif
#endif /* __cplusplus */

    
extern void     hci_uart_init ( void );

extern uint32_t hci_uart_rx_fifo_write ( const void *p_buffer, uint16_t length );

extern uint32_t hci_uart_rx_fifo_read ( void* p_buffer, uint16_t length );

extern uint32_t hci_uart_send ( const void *p_buffer, uint16_t length );

extern void     hci_uart_fifo_dbg( void );
	
extern void UART1_Handler ( void );

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */


#endif /* __HCI_UART_H__ */
/** @} */
