
#include "hci_cmd.h"
#include "hci.h"
#include "ke_task.h"
#include "ke_msg.h"
#include "whitening.h"
#include "rf.h"
#include "stack_svc_api.h"


/**
* @brief       Set Test Mode
* @param[in]   : mode, 0 normal DTM mode; 1 raw data mode, for PAN297
* @return      : N/A
*/

uint8_t hci_ext_reset ( void )
{
    hci_basic_cmd_send_2_controller ( HCI_RESET_CMD_OPCODE );
    
    return 0;
}

uint8_t hci_ext_set_tx_power ( struct ext_set_tx_power_cmd *param )
{
    ((rf_power_map)SVC_rf_power_map)( param->tx_power );
    
    return 0;
}

uint8_t hci_ext_set_tx_data ( struct ext_set_tx_data_cmd *param )
{
    ((llm_set_test_mode_tx_payload)SVC_llm_set_test_mode_tx_payload) ( param->data, param->length );
    
    return 0;
}

uint8_t hci_ext_start_tx ( struct ext_start_tx_cmd *param )
{
    ((llm_set_test_mode)SVC_llm_set_test_mode) (1);
    
    struct lld_le_tx_test_cmd *cmd = KE_MSG_ALLOC ( HCI_COMMAND, 
                                                    0, 
                                                    HCI_LE_TX_TEST_CMD_OPCODE, 
                                                    lld_le_tx_test_cmd );
    
    cmd->pk_payload_type = PAYL_01010101;// PAYL_01010101;
    cmd->test_data_len = 37;
    cmd->tx_freq = param->freq;
    
    hci_send_2_controller ( cmd );
    
    return 0;
}

uint8_t hci_ext_start_rx ( struct ext_start_rx_cmd *param )
{
    ((llm_set_test_mode)SVC_llm_set_test_mode) (1);
    
    struct lld_le_rx_test_cmd *cmd = KE_MSG_ALLOC ( HCI_COMMAND, 
                                                    0, 
                                                    HCI_LE_RX_TEST_CMD_OPCODE, 
                                                    lld_le_rx_test_cmd );
    cmd->rx_freq = param->freq;
    
    hci_send_2_controller ( cmd );
    
    return 0;
}

uint8_t hci_ext_stop_trx ( void )
{
    //hci_basic_cmd_send_2_controller ( HCI_EXTENSION_TEST_END_OPCODE );
    hci_basic_cmd_send_2_controller ( HCI_LE_TEST_END_CMD_OPCODE );
    
    return 0;
}

