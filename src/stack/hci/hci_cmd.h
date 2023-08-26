/**
 ****************************************************************************************
 * @file    hci_cmd.h
 * @brief   
 
 * @version 0.01
 * @date    2018/12/6
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
#ifndef __HCI_CMD_H__
#define __HCI_CMD_H__

#include "stdint.h"

#ifdef __cplusplus
#if __cplusplus
extern "C"{
#endif
#endif /* __cplusplus */


///LLM LE Transmitter Test Command parameters structure
struct lld_le_tx_test_cmd
{
    ///TX frequency for Tx test
    uint8_t        tx_freq;
    ///TX test data length
    uint8_t        test_data_len;
    ///TX test payload type - see enum
    uint8_t        pk_payload_type;
};

///LLM LE Receiver Test Command parameters structure
struct lld_le_rx_test_cmd
{
    ///TX frequency for Rx test
    uint8_t        rx_freq;
};


// HCI_EXTENSION_SET_TX_POWER_OPCODE
struct ext_set_tx_power_cmd
{
    uint8_t        tx_power;
};

struct ext_set_tx_count_cmd
{
    uint16_t        tx_count;
};


// HCI_EXTENSION_SET_TX_DATA_OPCODE
struct ext_set_tx_data_cmd
{
    uint8_t        length;
    uint8_t        data[1];
};

// HCI_EXTENSION_START_TX_OPCODE
struct ext_start_tx_cmd
{
    uint8_t        freq;
    uint16_t       count;
    uint16_t       interval;
    uint8_t        length;
    uint8_t        data[1];
};

// HCI_EXTENSION_START_RX_OPCODE
struct ext_start_rx_cmd
{
    uint8_t        freq;
    uint16_t       count;
    uint16_t       interval;
    uint16_t       window;
};

//whitenintg_payload( param->frc_ch,param->address, ADDRESS_SIZE, param->payload, param->payload_len,	output_ble_payload) ;
struct ext_tx_pdu_cmd
{
    char       frc_ch;
    char       address[5];
    char       payload_len;
    char       payload[37];

};

struct ext_set_rx_mode_cmd
{
    uint8_t       status;

};


extern uint8_t hci_ext_reset ( void );

extern uint8_t hci_ext_set_tx_power ( struct ext_set_tx_power_cmd *param );

extern uint8_t hci_ext_set_tx_data ( struct ext_set_tx_data_cmd *param );

extern uint8_t hci_ext_start_tx ( struct ext_start_tx_cmd *param );

extern uint8_t hci_ext_start_rx ( struct ext_start_rx_cmd *param );

extern uint8_t hci_ext_stop_trx ( void );


#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */


#endif /* __HCI_CMD_H__ */
/** @} */
