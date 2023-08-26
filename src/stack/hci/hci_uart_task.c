/**
 ****************************************************************************************
 * @file    hci_uart_task.c
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

#include "hci_uart_task.h"
#include "co_utils.h"
#include "co_endian.h"
#include "hci_uart.h"

#include "hci.h"
#include "hci_cmd.h"
#include "whitening.h"
#include "stack_svc_api.h"
#include "string.h" // memset / memcpy


#define DBG_HCI_UART_TASK_ENABLE   0
#if DBG_HCI_UART_TASK_ENABLE
#define DBG_HCI_UART_TASK(x)        printf x
#else
#define DBG_HCI_UART_TASK(x)        
#endif


#define HCI_CMD_HEAD_LEN                    (HCI_TRANSPORT_HDR_LEN + HCI_CMD_OPCODE_LEN + HCI_CMDEVT_PARLEN_LEN)

#define HCI_CMD_HEAD_OFFSET                 (0)
#define HCI_CMD_OPCODE_OFFSET               (1)
#define HCI_CMD_LENGTH_OFFSET               (3)
#define HCI_CMD_PARAM_OFFSET                (4)


#define HCI_EXT_RX_DATA_REPORT_SUBCODE     0x01


enum HCI_STATE
{
    HCI_STATE_RX_TYPE = 0,
    HCI_STATE_RX_COMMAND_OPCODE1,
    HCI_STATE_RX_COMMAND_OPCODE2,
    HCI_STATE_RX_COMMAND_LENGTH,
    HCI_STATE_RX_ACL_HEAD1,
    HCI_STATE_RX_ACL_HEAD2,
    HCI_STATE_RX_ACL_LENGTH1,
    HCI_STATE_RX_ACL_LENGTH2,
    HCI_STATE_RX_DATA_START,
    HCI_STATE_RX_DATA_CONT
};

typedef struct
{
    uint8 status;
}  common_cmplt_evt_t;

typedef struct
{
    uint8 status;
    uint8 num_of_pktslo8;
    uint8 num_of_pktshi8;
} le_test_end_evt_t;

typedef union
{
    common_cmplt_evt_t common_cmplt;
    le_test_end_evt_t  le_test_end;
    uint8 bytes[24];
} event_parameters_t;

typedef struct
{
    uint8 num_hci_cmd_packet;
    uint8 cmd_opcode_low_8;
    uint8 cmd_opcode_high_8;

	
}  cmd_cmplt_evt_t;

typedef struct
{
    uint8_t subcode;
    uint8_t data[31];
} hci_ext_rx_data_report_evt_t;

typedef struct
{
    ///Packet type
    uint8_t  type;
    
    ///Event code
    uint8_t  code;
    
    ///Event parameters length
    uint8_t  parlen;
} hci_evt_hdr_t;



enum HCI_RX_MODE_STATE
{
    HCI_RX_MODE_OPEN_PRINTF  = 0,
    HCI_RX_MODE_CLEAR_PACKAGE,
    HCI_STATE_RX_GET_PACKAGE
  
};






uint8_t hci_rx_dispatch_state = HCI_STATE_RX_TYPE;
uint8_t hci_cmd_buf[200] = { 0 };
uint8_t hci_rx_mode = HCI_RX_MODE_OPEN_PRINTF;
static struct hci_cmd_hdr m_hci_cmd_hdr;

hci_evt_hdr_t hci_evt_hdr;

cmd_cmplt_evt_t cmd_cmplt_evt;


static void hci_cmd_cmp_evt_handler ( uint16_t opcode, const void *param, uint16 length );
static void hci_ext_rx_data_report_evt_handler ( const uint8_t* data, uint16 length );


/**
* @brief       Set Test Mode
* @param[in]   : mode, 0 normal DTM mode; 1 raw data mode, for PAN297
* @return      : N/A
*/


void hci_uart_task_init ( void )
{
    hci_rx_dispatch_state = HCI_STATE_RX_TYPE;
    memset ( hci_cmd_buf, 0, sizeof ( hci_cmd_buf ) );
    
    ((hci_dtm_init_evt_handler)SVC_hci_dtm_init_evt_handler) ( hci_cmd_cmp_evt_handler );
    ((rx_data_report_evt_handler_init)SVC_rx_data_report_evt_handler_init) ( hci_ext_rx_data_report_evt_handler );
}

#if 1
//#define XN297_PAY_LOAD_SIZE 1
//#define ADDRESS_SIZE 5
//#define BLE_ADV2_SIZE	3 + ADDRESS_SIZE + XN297_PAY_LOAD_SIZE + 2
//#define XN297_PREAMBLE_SIZE 3
//#define XN297_CRC_SIZE  2
//#define TYPE_SIZE 1
//#define MANUFACTURE_ID_SIZE 2
//#define EXCEPT_PAYLOAD_PUDE_SIZE   (ADDRESS_SIZE+XN297_PREAMBLE_SIZE+XN297_CRC_SIZE)


//void ble_set_xn297_payload(struct ext_tx_pdu_cmd *param)
//{
//    #define PRE_PAYLOAD_SIZE 13
//    char pre_payload[PRE_PAYLOAD_SIZE] = {0x12,0xf1,0xfe,0xca,0xca,0xbb,0x02,0x01,0x06,0x0e,0xff,0xf0,0xff};
//	char output_ble_payload[BLE_PAYLOAD_SIZE]={0,};

//	if((param->payload_len)> 27)	
//	{
//		DBG_HCI_UART_TASK ( ( "param->payload_lenh=%d\n", param->payload_len ) );
//	}
//	else
//	{
//		whitenintg_payload( param->frc_ch,param->address, ADDRESS_SIZE, param->payload, param->payload_len,	output_ble_payload+PRE_PAYLOAD_SIZE) ;
//		memcpy(output_ble_payload,pre_payload,PRE_PAYLOAD_SIZE);
//        
//        ((llm_set_test_mode_tx_payload)SVC_llm_set_test_mode_tx_payload)((uint8_t *)output_ble_payload,param->payload_len+EXCEPT_PAYLOAD_PUDE_SIZE+PRE_PAYLOAD_SIZE);
//		
//		for(int i=0;i<param->payload_len+EXCEPT_PAYLOAD_PUDE_SIZE+PRE_PAYLOAD_SIZE;i++)
//		{
//			//printf(" 0 m_ble_adv_data[%d]=[%x] \n",i,m_ble_adv_data[i]);
//         //   DBG_HCI_UART_TASK ( ( " 0 output_ble_payload[%d]=[%x]\n", i, output_ble_payload[i] ) );
//		}
//	}
//}


uint8_t ble_set_xn297_payload(struct ext_tx_pdu_cmd *param)
{
	uint8_t output_ble_payload[BLE_PDU_PAYLOAD_SIZE] = { 0 };
    
	if ( param->payload_len > BLE_PDU_PAYLOAD_SIZE - XN297_PREMABLE_SIZE - XN297_ADDRESS_SIZE - XN297_CRC_SIZE )	
	{
		DBG_HCI_UART_TASK ( ( "param->payload_lenh=%d\n", param->payload_len ) );
        return 1;
	}
	else
	{
        uint8_t length = xn297_whitening_payload_generate ( param->frc_ch, 
                                                            (const uint8_t *)param->address, 
                                                            XN297_ADDRESS_SIZE, 
                                                            (const uint8_t *)param->payload,
                                                            param->payload_len,
                                                            output_ble_payload );
        
				((llm_set_test_mode_tx_payload)SVC_llm_set_test_mode_tx_payload)((uint8_t *)output_ble_payload, length);																										

		
		for ( int i=0; i<length; i++ )
		{
			//printf(" 0 m_ble_adv_data[%d]=[%x] \n",i,m_ble_adv_data[i]);
            DBG_HCI_UART_TASK ( ( " 0 output_ble_payload[%d]=[%x]\n", i, output_ble_payload[i] ) );
		}
	}
    
    return 0;
}






#endif

uint32_t hci_event_generate ( uint8_t event_code, const void* param, uint8_t param_len, const void*param_ext, uint8_t param_ext_len )
{
    hci_evt_hdr_t evt_hdr;
    
    evt_hdr.type   = 0x04;
    evt_hdr.code   = event_code;
    evt_hdr.parlen = param_len + param_ext_len;
	
    if ( ( event_code != 0x3f ) || ( hci_rx_mode == HCI_RX_MODE_OPEN_PRINTF ) )
    {
        hci_uart_send ( &evt_hdr, 3 );
        hci_uart_send ( param, param_len );
        hci_uart_send ( param_ext, param_ext_len );
    }
    
    return SDK_SUCCESS;
}

uint32_t hci_event_cmd_cmplt ( uint16_t cmd_opcode, const void* param, uint8_t length )
{
    cmd_cmplt_evt_t cmplt_evt;
    
    cmplt_evt.num_hci_cmd_packet = 1;
    cmplt_evt.cmd_opcode_low_8   = cmd_opcode;
    cmplt_evt.cmd_opcode_high_8  = cmd_opcode >> 8;
	
    hci_event_generate ( 0x0E, &cmplt_evt, 3, param, length );
	
    return SDK_SUCCESS;
}

void hci_cmd_cmp_evt_handler ( uint16_t opcode, const void* param, uint16 length )
{
    if ( NULL == param )
    {
        DBG_HCI_UART_TASK ( ( "Event param empty\n" ) );
    }
    
    switch ( opcode )
    {
        case HCI_RESET_CMD_OPCODE:
        {
#if DBG_HCI_UART_TASK_ENABLE
            common_cmplt_evt_t* p_evt = ( common_cmplt_evt_t* ) param;

            DBG_HCI_UART_TASK ( ( "\nLE Reset Event\n" ) );
            DBG_HCI_UART_TASK ( ( "Status 0x%02X (%s)\n", p_evt->status, ( p_evt->status == 0 ) ? "Success" : "Error" ) );
#endif /* DBG_HCI_UART_TASK_ENABLE */
            
            hci_event_cmd_cmplt ( HCI_RESET_CMD_OPCODE, param, 1 );
        }
        break;
        
        case HCI_LE_RX_TEST_CMD_OPCODE:
        {
#if DBG_HCI_UART_TASK_ENABLE
            common_cmplt_evt_t* p_evt = ( common_cmplt_evt_t* ) param;

            DBG_HCI_UART_TASK ( ( "\nLE Receiver Event\n" ) );
            DBG_HCI_UART_TASK ( ( "Status 0x%02X (%s)\n", p_evt->status, ( p_evt->status == 0 ) ? "Success" : "Error" ) );
#endif /* DBG_HCI_UART_TASK_ENABLE */
            
            hci_event_cmd_cmplt ( HCI_LE_RX_TEST_CMD_OPCODE, param, 1 );
        }
        break;
        
        
        case HCI_LE_TX_TEST_CMD_OPCODE:
        {
#if DBG_HCI_UART_TASK_ENABLE
            common_cmplt_evt_t* p_evt = ( common_cmplt_evt_t* ) param;

            DBG_HCI_UART_TASK ( ( "\nLE Transmitter Event\n" ) );
            DBG_HCI_UART_TASK ( ( "Status 0x%02X (%s)\n", p_evt->status, ( p_evt->status == 0 ) ? "Success" : "Error" ) );
#endif /* DBG_HCI_UART_TASK_ENABLE */
            
            hci_event_cmd_cmplt ( HCI_LE_TX_TEST_CMD_OPCODE, param, 1 );
        }
        break;
        
        case HCI_LE_TEST_END_CMD_OPCODE:
        {
            struct hci_test_end_cmd_cmp_evt const* p_evt = ( struct hci_test_end_cmd_cmp_evt const* ) param;

            DBG_HCI_UART_TASK ( ( "\nLE Test End Event\n" ) );
            DBG_HCI_UART_TASK ( ( "Status 0x%02X (%s)\n", p_evt->status, ( p_evt->status == 0 ) ? "Success" : "Error" ) );
            DBG_HCI_UART_TASK ( ( "Packet Received %d\n", p_evt->nb_packet_received ) );

            uint8_t p_buf[3];
            p_buf[0] = p_evt->status;
            p_buf[1] = p_evt->nb_packet_received;
            p_buf[2] = p_evt->nb_packet_received >> 8;

            hci_event_cmd_cmplt ( HCI_LE_TEST_END_CMD_OPCODE, p_buf, 3 );
        }
        break;
        
        default: /* Nothing to do */
            break;
    } /* end of switch */
}


void hci_ext_rx_data_report_evt_handler ( const uint8_t* data, uint16 length )
{
    int i = 0;

	if ( HCI_RX_MODE_OPEN_PRINTF == hci_rx_mode )
    {
	    for ( i = 0; i < length; i++ )
	    {
	        DBG_HCI_UART_TASK ( ( "%02X ", data[i] ) );
	    }
	    
	    DBG_HCI_UART_TASK ( ( "\n" ) );
	}
    
    uint8_t subcode = HCI_EXT_RX_DATA_REPORT_SUBCODE;

    hci_event_generate ( 0x3F, &subcode, 1, data, length );
}


uint32_t hci_cmd_dispatch ( uint8_t* payl )
{
    uint32_t error_code = SDK_SUCCESS;

    switch ( m_hci_cmd_hdr.opcode )
    {
        case HCI_RESET_CMD_OPCODE :
        {
            DBG_HCI_UART_TASK ( ( "\nLE Reset Command\n" ) );

            //app_reset_app();
            hci_basic_cmd_send_2_controller ( HCI_RESET_CMD_OPCODE );
        }
        break;
        
        case HCI_LE_RX_TEST_CMD_OPCODE :
        {
            struct lld_le_rx_test_cmd *param = (struct lld_le_rx_test_cmd *)&hci_cmd_buf[HCI_CMD_HEAD_OFFSET];
            
            DBG_HCI_UART_TASK ( ( "\nLE Receiver Test Command\n" ) );
            DBG_HCI_UART_TASK ( ( "Frequency       %d MHz\n", 2402 + param->rx_freq * 2 ) );
            
            struct lld_le_rx_test_cmd *cmd = KE_MSG_ALLOC ( HCI_COMMAND, 0, HCI_LE_RX_TEST_CMD_OPCODE, lld_le_rx_test_cmd );
            cmd->rx_freq = param->rx_freq;
            
            hci_send_2_controller ( cmd );
        }
        break;
        
        case HCI_LE_TX_TEST_CMD_OPCODE :
        {
            struct lld_le_tx_test_cmd *param = (struct lld_le_tx_test_cmd *)&hci_cmd_buf[HCI_CMD_HEAD_OFFSET];
            
            DBG_HCI_UART_TASK ( ( "\nLE Transmitter Test Command\n" ) );
            DBG_HCI_UART_TASK ( ( "Frequency       %d MHz\n",   2402 + param->tx_freq * 2 ) );
            DBG_HCI_UART_TASK ( ( "Payload Pattern %d\n",       param->pk_payload_type ) );
            DBG_HCI_UART_TASK ( ( "Payload Length  %d Bytes\n", param->test_data_len ) );

            ((llm_set_test_mode)SVC_llm_set_test_mode) (0);
            
            struct lld_le_tx_test_cmd *cmd = KE_MSG_ALLOC ( HCI_COMMAND, 0, HCI_LE_TX_TEST_CMD_OPCODE, lld_le_tx_test_cmd );
            cmd->pk_payload_type = param->pk_payload_type;// PAYL_01010101;
            cmd->test_data_len = param->test_data_len;
            cmd->tx_freq = param->tx_freq;
                    
            hci_send_2_controller ( cmd );
        }
        break;
        
        case HCI_LE_TEST_END_CMD_OPCODE :
        {
            DBG_HCI_UART_TASK ( ( "\nLE Test End Command\n" ) );
            
            hci_basic_cmd_send_2_controller ( HCI_LE_TEST_END_CMD_OPCODE );
        }
        break;
        
//        .\Out\proj_hid_keyboard.axf: Error: L6200E: Symbol hci_ext_stop_trx multiply defined (by hci_cmd.o and hci.o).
        // Vendor Specific
        case HCI_EXTENSION_RESET_OPCODE :
        {
            DBG_HCI_UART_TASK ( ( "\nHCI Extension Reset System\n" ) );
            
            // Reset System
            hci_ext_reset();
            
            struct hci_basic_cmd_cmp_evt evt_param;
            evt_param.status = 0;
            hci_event_cmd_cmplt ( HCI_EXTENSION_RESET_OPCODE, &evt_param, 1 );
        }
        break;
        
        case HCI_EXTENSION_SET_TX_POWER_OPCODE :
        {
            DBG_HCI_UART_TASK ( ( "\nHCI Extension Set Tx Power\n" ) );
            
            struct hci_basic_cmd_cmp_evt evt_param;
            
            evt_param.status = hci_ext_set_tx_power ( (struct ext_set_tx_power_cmd *)&hci_cmd_buf[HCI_CMD_HEAD_OFFSET] );
            
            hci_event_cmd_cmplt ( HCI_EXTENSION_SET_TX_POWER_OPCODE, &evt_param, 1 );
        }
        break;
        
        case HCI_EXTENSION_SET_TX_COUNT_OPCODE :
        {
            DBG_HCI_UART_TASK ( ( "\n HCI_EXTENSION_SET_TX_COUNT_OPCODE\n" ) );
            
            struct hci_basic_cmd_cmp_evt evt_param;
            
            uint16_t tx_count = (hci_cmd_buf[HCI_CMD_HEAD_OFFSET]<<8)|hci_cmd_buf[HCI_CMD_HEAD_OFFSET+1];
            
            ((lld_test_tx_count_init)SVC_lld_test_tx_count_init) ( tx_count );
            
            DBG_HCI_UART_TASK ( ( "\n tx_count %d\n" ,tx_count ));
            evt_param.status = 0;
            hci_event_cmd_cmplt ( HCI_EXTENSION_SET_TX_COUNT_OPCODE, &evt_param, 1 );
        }
        break;
        
        case HCI_EXTENSION_SET_TX_DATA_OPCODE :
        {
            DBG_HCI_UART_TASK ( ( "\nHCI Extension Set Tx Data\n" ) );
            
            ((llm_set_test_mode_tx_payload)SVC_llm_set_test_mode_tx_payload) ( &hci_cmd_buf[HCI_CMD_HEAD_OFFSET], m_hci_cmd_hdr.parlen );
            
            struct hci_basic_cmd_cmp_evt evt_param;
            evt_param.status = 0;
            hci_event_cmd_cmplt ( HCI_EXTENSION_SET_TX_DATA_OPCODE, &evt_param, 1 );
        }
        break;
        
        
        case HCI_EXTENSION_SET_TX_PDU_OPCODE :
        {
            struct ext_tx_pdu_cmd *param = (struct ext_tx_pdu_cmd *)&hci_cmd_buf[HCI_CMD_HEAD_OFFSET];
            
            DBG_HCI_UART_TASK ( ( "\nHCI Extension Set Tx pdu\n" ) );
            
            ble_set_xn297_payload(param);

            struct hci_basic_cmd_cmp_evt evt_param; 
   
            evt_param.status = 0;     
            hci_event_cmd_cmplt ( HCI_EXTENSION_SET_TX_PDU_OPCODE, &evt_param, 1 );
        }
        break;
	    
        case HCI_EXTENSION_SET_RX_MODE_OPCODE :
	    {
	        DBG_HCI_UART_TASK ( ( "\n HCI_EXTENSION_SET_RX_MODE_OPCODE \n" ) );
            
	        struct ext_set_rx_mode_cmd *param = (struct ext_set_rx_mode_cmd *)&hci_cmd_buf[HCI_CMD_HEAD_OFFSET];             
            
			hci_rx_mode = param->status;
	        struct hci_basic_cmd_cmp_evt evt_param;
	        evt_param.status = 0;
	        hci_event_cmd_cmplt ( HCI_EXTENSION_SET_RX_MODE_OPCODE, &evt_param, 1 );
	    }
	    break;               
	    
        
        case HCI_EXTENSION_TX_TEST_OPCODE :
        {
            DBG_HCI_UART_TASK ( ( "\nHCI Extension TX Test\n" ) );
            
            hci_ext_start_tx ( (struct ext_start_tx_cmd *)&hci_cmd_buf[HCI_CMD_HEAD_OFFSET] );
            
            struct hci_basic_cmd_cmp_evt evt_param; 
            evt_param.status = 0;     
            hci_event_cmd_cmplt ( HCI_EXTENSION_TX_TEST_OPCODE, &evt_param, 1 );
        }
        break;
        
        case HCI_EXTENSION_RX_TEST_OPCODE :
        {
            DBG_HCI_UART_TASK ( ( "\nHCI Extension RX Test\n" ) );
            
            hci_ext_start_rx ( (struct ext_start_rx_cmd *)&hci_cmd_buf[HCI_CMD_HEAD_OFFSET] );
            
            struct hci_basic_cmd_cmp_evt evt_param; 
            evt_param.status = 0;
            hci_event_cmd_cmplt ( HCI_EXTENSION_RX_TEST_OPCODE, &evt_param, 1 );
        }
        break;

        case HCI_EXTENSION_TEST_END_OPCODE :
        {
            DBG_HCI_UART_TASK ( ( "\nHCI Extension Test End\n" ) );

            hci_ext_stop_trx();
            
            struct hci_basic_cmd_cmp_evt evt_param; 
            evt_param.status = 0;
            hci_event_cmd_cmplt ( HCI_EXTENSION_TEST_END_OPCODE, &evt_param, 1 );
        }
        break;
        
        default:
        {
            DBG_HCI_UART_TASK ( ( "OPCODE %04X\n", m_hci_cmd_hdr.opcode ) );
            
            error_code = SDK_ERROR_INVALID_PARAM;
        }
    }

    return error_code;
}


void hci_uart_data_send_cmp ( ke_msg_id_t const msgid,
                              void const* param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id )
{

}

void hci_uart_data_recv_cmp ( ke_msg_id_t const msgid,
                              void const* param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id )
{

}

void hci_uart_data_send ( ke_msg_id_t const msgid,
                          void const* param,
                          ke_task_id_t const dest_id,
                          ke_task_id_t const src_id )
{

}







void hci_uart_data_recv ( ke_msg_id_t const msgid,
                          void const* param,
                          ke_task_id_t const dest_id,
                          ke_task_id_t const src_id )
{
    uint32_t error_code;

    //DBG_HCI_UART_TASK (( "hci rx dispatch %d\n", hci_rx_dispatch_state ));
    //hci_uart_fifo_dbg();
    
    switch ( hci_rx_dispatch_state )
    {
        case HCI_STATE_RX_TYPE:
        {
            error_code = hci_uart_rx_fifo_read ( &hci_cmd_buf[HCI_CMD_HEAD_OFFSET], HCI_CMD_HEAD_LEN );

            if ( SDK_SUCCESS != error_code )
            {
                return;
            }

            if ( HCI_CMD_MSG_TYPE == hci_cmd_buf[HCI_CMD_HEAD_OFFSET] )
            {
                m_hci_cmd_hdr.opcode = co_read16p ( &hci_cmd_buf[HCI_CMD_OPCODE_OFFSET] );

                m_hci_cmd_hdr.parlen = hci_cmd_buf[HCI_CMD_LENGTH_OFFSET];
                
                if ( 0 == m_hci_cmd_hdr.parlen )
                {
                    hci_cmd_dispatch ( NULL );
                    
                    hci_rx_dispatch_state = HCI_STATE_RX_TYPE;
                }
                else
                {
                    hci_rx_dispatch_state = HCI_STATE_RX_DATA_START;
                }
            }
        }
        break;

        case HCI_STATE_RX_DATA_START:
        case HCI_STATE_RX_DATA_CONT:
        {
            error_code = hci_uart_rx_fifo_read ( &hci_cmd_buf[HCI_CMD_HEAD_OFFSET], m_hci_cmd_hdr.parlen );

            if ( SDK_SUCCESS != error_code )
            {
                return;
            }

            hci_cmd_dispatch ( &hci_cmd_buf[HCI_CMD_PARAM_OFFSET] );

            hci_rx_dispatch_state = HCI_STATE_RX_TYPE;
        }
        break;
    }
}


/* Default State handlers definition. */
//KE_MSG_HANDLER_TAB(hci_uart)
static const struct ke_msg_handler task_hci_uart_msg_handler_tab[] =
{
    // Note: first message is latest message checked by kernel so default is put on top.
    { HCI_UART_DATA_SEND_CMP,   ( ke_msg_func_t ) hci_uart_data_send_cmp},
    { HCI_UART_DATA_RECV_CMP,   ( ke_msg_func_t ) hci_uart_data_recv_cmp},
    { HCI_UART_DATA_SEND,       ( ke_msg_func_t ) hci_uart_data_send},
    { HCI_UART_DATA_RECV,       ( ke_msg_func_t ) hci_uart_data_recv},
		
};

/* Defines the place holder for the states of all the task instances. */
ke_state_t hci_uart_state[HCI_UART_IDX_MAX];

const struct ke_state_handler hci_uart_default_handler = KE_STATE_HANDLER ( task_hci_uart_msg_handler_tab );

const struct ke_task_desc TASK_DESC_HCI_UART =
{
    NULL,
    &hci_uart_default_handler,
    hci_uart_state,
    HCI_UART_STATE_MAX,
    HCI_UART_IDX_MAX,
    ARRAY_LEN ( task_hci_uart_msg_handler_tab )
};



