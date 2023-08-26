/**
 ****************************************************************************************
 * @file    hci_uart_task.h
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
#ifndef __HCI_UART_TASK_H__
#define __HCI_UART_TASK_H__

#include "panip_config.h"     // SW configuration
#include "panip_task.h"     // SW configuration
#include "ke_task.h"
#include "sdk_errors.h"


/// HCI Uart Task messages
enum hci_uart_msg_id
{
    /// Data Received
    HCI_UART_DATA_RECV = TASK_FIRST_MSG(TASK_ID_GATTC),
    /// Data 
    HCI_UART_DATA_RECV_CMP,
    /// Data Send
    HCI_UART_DATA_SEND,
    /// Data Send Complete
    HCI_UART_DATA_SEND_CMP,
    
    HCI_UART_STATE_MAX,
	
};




#define HCI_UART_IDX_MAX 1

#ifdef __cplusplus
#if __cplusplus
extern "C"{
#endif
#endif /* __cplusplus */

typedef void (*hci_dtm_evt_handler_t) ( uint16_t opcode, const void *param, uint16 length );
typedef void (*rx_data_report_evt_handler_t) ( const uint8_t* data, uint16 length );
    
extern void hci_uart_task_init ( void );
    
extern void hci_uart_data_recv ( ke_msg_id_t const msgid,
                                 void const* param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id );


extern void le_test_end(void);

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */


#endif /* __HCI_UART_TASK_H__ */
/** @} */
