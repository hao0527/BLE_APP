
/**
 ****************************************************************************************
 * @addtogroup HCI
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "panip_config.h"       // SW configuration

#if (HCI_PRESENT)

#if (HCI_TL_SUPPORT)

#include <string.h>          // string manipulation
#include "co_error.h"        // error definition
#include "co_utils.h"        // common utility definition
#include "co_endian.h"       // common endianess definition
#include "co_list.h"         // list definition

#include "hci.h"             // hci definition
#include "hci_int.h"         // hci internal definition

#include "ke_msg.h"          // kernel message declaration
#include "ke_task.h"         // kernel task definition
#include "ke_mem.h"          // kernel memory definition
#include "ke_timer.h"        // kernel timer definition
#include "rwip.h"            // rw bt core interrupt

#if (H4TL_SUPPORT)
#include "h4tl.h"            // H4TL definitions
#endif // H4TL_SUPPORT

#if BLE_EMB_PRESENT
//#include "ble_util_buf.h"
#include "em_map.h"
#include "reg_access.h"
#endif //BLE_EMB_PRESENT



#if ((BLE_HOST_PRESENT) && (!BLE_EMB_PRESENT))
#include "gapc.h"
#include "gap.h"
#endif // ((BLE_HOST_PRESENT) && (!BLE_EMB_PRESENT))
#include "stack_svc_api.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/// Number of HCI commands the stack can handle simultaneously
#define HCI_NB_CMD_PKTS                    5

/// Offset of the Kernel message parameters compared to the event packet position
#define HCI_CMD_PARAM_OFFSET               3
#define HCI_EVT_CC_PARAM_OFFSET            5
#define HCI_EVT_CS_PARAM_OFFSET            5
#define HCI_EVT_PARAM_OFFSET               2
#define HCI_EVT_LE_PARAM_OFFSET            2
#define HCI_EVT_DBG_PARAM_OFFSET           2


/*
 * ENUMERATIONS DEFINITIONS
 ****************************************************************************************
 */

///HCI TX states
enum HCI_TX_STATE
{
    ///HCI TX Start State - when packet is ready to be sent
    HCI_STATE_TX_ONGOING,
    ///HCI TX Done State - TX ended with no error
    HCI_STATE_TX_IDLE
};


/*
 * STRUCTURES DEFINITIONS
 ****************************************************************************************
 */

///HCI Environment context structure
struct hci_tl_env_tag
{
    /// Queue of kernel messages corresponding to HCI TX packets
    struct co_list tx_queue;

    #if (BT_EMB_PRESENT || HCI_BLE_CON_SUPPORT)
    /// acl data messages
    struct co_list acl_queue;
    #endif //(BT_EMB_PRESENT || HCI_BLE_CON_SUPPORT)



    /// keep the link to message in transmission to a Host
    struct ke_msg *curr_tx_msg;



    ///Tx state - either transmitting, done or error.
    uint8_t  tx_state;

    /// Allowed number of commands from Host to controller (sent from controller to host)
    int8_t  nb_h2c_cmd_pkts;

    /// This flag indicates that the current ACL payload is received in a trash buffer
    bool acl_trash_payload;
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

///HCI environment structure external global variable declaration
static struct hci_tl_env_tag hci_tl_env;

/*
 * LOCAL FUNCTION DECLARATIONS
 ****************************************************************************************
 */

static void hci_tx_done(void);

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)

/**
 ****************************************************************************************
 * @brief Reject a HCI command
 *
 * This function creates a CS or CC event Kernel message to reject a command.
 *
 * @param[in]   cmd_desc    Command descriptor (NULL if command unknown)
 * @return      Pointer to the beginning of the event
 *****************************************************************************************
 */
static void hci_cmd_reject(const struct hci_cmd_desc_tag* cmd_desc, uint16_t opcode, uint8_t error)
{
    if(cmd_desc != NULL)
    {
        // Check if this command shall be replied with a CC or CS event
        if(cmd_desc->ret_par_fmt != NULL)
        {
            // Get size of the Command Complete Event message associated with the opcode
            uint16_t ret_par_len;
            uint8_t status = CO_UTIL_PACK_OK;

            // Check if the generic packer can be used (basic fixed-length format)
            if(!(cmd_desc->dest_field & HCI_CMD_DEST_SPEC_RET_PAR_PK_MSK))
            {
                // Use the generic unpacker to get unpacked parameters length
                status = co_util_unpack(NULL, NULL, &ret_par_len, 0xFFFF, cmd_desc->ret_par_fmt);
            }
            else
            {
                // Use the special unpacker to get unpacked parameters length
                status = ((hci_pkupk_func_t)cmd_desc->ret_par_fmt)(NULL, NULL, &ret_par_len, 0);
            }

            if(status == CO_UTIL_PACK_OK)
            {
                // Send a CC event with error code "Unknown connection identifier"
                struct hci_basic_cmd_cmp_evt* evt = (struct hci_basic_cmd_cmp_evt*) ((ke_msg_alloc_handler)SVC_ke_msg_alloc)(HCI_CMD_CMP_EVENT, 0, opcode, ret_par_len);
                evt->status = error;
                hci_send_2_host(evt);
            }
            else
            {
                ASSERT_INFO(0, status, opcode);
            }
        }
        else
        {
            // Send a CS event with error code "Unknown connection identifier"
            struct hci_cmd_stat_event* evt = KE_MSG_ALLOC(HCI_CMD_STAT_EVENT, 0, opcode, hci_cmd_stat_event);
            evt->status = error;
            hci_send_2_host(evt);
        }
    }
    else
    {
        // Send a CC event with error code "Unknown HCI command"
        struct hci_basic_cmd_cmp_evt * evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_basic_cmd_cmp_evt);
        evt->status = error;
        hci_send_2_host(evt);
    }
}

/**
 ****************************************************************************************
 * @brief Build a HCI Command Status event
 *
 * This function build a HCI CS event from the CS event Kernel message.
 *
 * @param[in]   msg    Kernel message containing the CS event
 * @return      Pointer to the beginning of the event
 *****************************************************************************************
 */
static uint8_t* hci_build_cs_evt(struct ke_msg * msg)
{
    uint8_t* param = ke_msg2param(msg);
    uint8_t* buf = param - HCI_EVT_CS_PARAM_OFFSET;
    uint8_t* pk = buf;
    uint16_t opcode = msg->src_id;

    //pack event code
    *pk++ = HCI_CMD_STATUS_EVT_CODE;

    //pack event parameter length
    *pk++ = HCI_CSEVT_PARLEN;

    //pack the status
    *pk++ = *param;

    //pack the number of h2c packets
    *pk++ = (hci_tl_env.nb_h2c_cmd_pkts > 0)? (uint8_t) hci_tl_env.nb_h2c_cmd_pkts : 0;

    //pack opcode
    co_write16p(pk, co_htobs(opcode));

    return buf;
}



/**
 ****************************************************************************************
 * @brief Build a HCI Command Complete event
 *
 * This function build a HCI CC event from the CC event Kernel message.
 *
 * @param[in]   msg    Kernel message containing the CC event
 * @return      Pointer to the beginning of the event
 *****************************************************************************************
 */
static uint8_t* hci_build_cc_evt(struct ke_msg * msg)
{
    uint8_t* param = ke_msg2param(msg);
    uint8_t* buf = param - HCI_EVT_CC_PARAM_OFFSET;
    uint8_t* pk = buf;
    uint16_t opcode = msg->src_id;
    uint16_t ret_par_len = msg->param_len;

    // Look for the command descriptor
    const struct hci_cmd_desc_tag* cmd_desc = hci_look_for_cmd_desc(opcode);

    if((cmd_desc != NULL) && (ret_par_len > 0))
    {
        uint8_t status = CO_UTIL_PACK_ERROR;

        if(cmd_desc->ret_par_fmt != NULL)
        {
            // Check if the generic packer can be used (basic fixed-length format)
            if(!(cmd_desc->dest_field & HCI_CMD_DEST_SPEC_RET_PAR_PK_MSK))
            {
                // Pack the returned parameters using the generic packer
                status = co_util_pack(param, param, &ret_par_len, ret_par_len, cmd_desc->ret_par_fmt);
            }
            else
            {
                // Pack the return parameters using the special packer
                status = ((hci_pkupk_func_t)cmd_desc->ret_par_fmt)(param, param, &ret_par_len, ret_par_len);
            }
        }

        if(status != CO_UTIL_PACK_OK)
        {
            ASSERT_INFO((status == CO_UTIL_PACK_OK), status, opcode);
        }
    }
    else if (opcode != HCI_NO_OPERATION_CMD_OPCODE)
    {
        // Set the status "Unknown HCI command" as unique returned parameter
        *param = CO_ERROR_UNKNOWN_HCI_COMMAND;
    }
    else
    {
        ASSERT_INFO(0, opcode, ret_par_len);
    }

    //pack event code
    *pk++ = HCI_CMD_CMP_EVT_CODE;

    //pack event parameter length
    *pk++ = HCI_CCEVT_HDR_PARLEN + ret_par_len;

    //pack the number of h2c packets
    *pk++ = (hci_tl_env.nb_h2c_cmd_pkts > 0)? (uint8_t) hci_tl_env.nb_h2c_cmd_pkts : 0;

    //pack opcode
    co_write16p(pk, co_htobs(opcode));

    return buf;
}

/**
 ****************************************************************************************
 * @brief Build a HCI event
 *
 * This function build a HCI event from the event Kernel message.
 *
 * @param[in]   msg    Kernel message containing the event
 * @return      Pointer to the beginning of the event
 *****************************************************************************************
 */
static uint8_t* hci_build_evt(struct ke_msg * msg)
{
    uint8_t* param = ke_msg2param(msg);
    uint8_t* buf = param - HCI_EVT_LE_PARAM_OFFSET;
    uint8_t* pk = buf;
    uint8_t evt_code = (uint8_t) msg->src_id;
    uint16_t par_len = msg->param_len;
    uint8_t status = CO_UTIL_PACK_OK;

    ASSERT_INFO(msg->src_id <= UINT8_MAX, msg->src_id, 0);

    // Look for the event descriptor
    const struct hci_evt_desc_tag* evt_desc = hci_look_for_evt_desc(evt_code);

    if(evt_desc != NULL)
    {
        if(evt_desc->par_fmt != NULL)
        {
            // Check if the generic packer can be used (basic fixed-length format)
            if(evt_desc->special_pack == PK_GEN)
            {
                // Pack the returned parameters using the generic packer
                status = co_util_pack(param, param, &par_len, par_len, evt_desc->par_fmt);
            }
            else
            {
                // Pack the parameters using the special packer
                status = ((hci_pkupk_func_t)evt_desc->par_fmt)(param, param, &par_len, par_len);
            }
        }
        else if(msg->param_len > 0)
        {
            status = CO_UTIL_PACK_ERROR;
        }

        if(status != CO_UTIL_PACK_OK)
        {
            ASSERT_INFO((status == CO_UTIL_PACK_OK), status, evt_code);
        }

        ASSERT_INFO(par_len <= msg->param_len, status, evt_code);

        //pack event code
        *pk++ = evt_code;

        //pack event parameter length
        *pk++ = par_len;
    }
    else
    {
        ASSERT_INFO(0, evt_code, 0);
    }

    return buf;
}

#if (RW_DEBUG || (BLE_EMB_PRESENT && BLE_TESTER))
/**
 ****************************************************************************************
 * @brief Build a HCI DBG event
 *
 * This function build a HCI LE event from the DBG event Kernel message.
 *
 * @param[in]   msg    Kernel message containing the DBG event
 * @return      Pointer to the beginning of the event
 *****************************************************************************************
 */
static uint8_t* hci_build_dbg_evt(struct ke_msg * msg)
{
    uint8_t* param = ke_msg2param(msg);
    uint8_t* buf = param - HCI_EVT_DBG_PARAM_OFFSET;
    uint8_t* pk = buf;
    uint8_t subcode = *param;
    uint16_t par_len = msg->param_len;

    // Look for the event descriptor
    const struct hci_evt_desc_tag* evt_desc = hci_look_for_dbg_evt_desc(subcode);

    if(evt_desc != NULL)
    {
        uint8_t status = CO_UTIL_PACK_ERROR;

        if(evt_desc->par_fmt != NULL)
        {
            // Check if the generic packer can be used (basic fixed-length format)
            if(evt_desc->special_pack == PK_GEN)
            {
                // Pack the returned parameters
                status = co_util_pack(param, param, &par_len, par_len, evt_desc->par_fmt);
            }
            else
            {
                // Pack the parameters using the special packer
                status = ((hci_pkupk_func_t)evt_desc->par_fmt)(param, param, &par_len, par_len);
            }
        }

        ASSERT_INFO((status == CO_UTIL_PACK_OK), status, subcode);
        ASSERT_INFO(par_len <= msg->param_len, par_len, subcode);
        if(status == CO_UTIL_PACK_OK)
        {
            //pack event code
            *pk++ = HCI_DBG_META_EVT_CODE;

            //pack event parameter length
            *pk++ = par_len;
        }
    }
    else
    {
        ASSERT_INFO(0, subcode, 0);
    }

    return buf;
}
#endif // (RW_DEBUG || (BLE_EMB_PRESENT && BLE_TESTER))


#if BLE_EMB_PRESENT
/**
 ****************************************************************************************
 * @brief Build a HCI LE event
 *
 * This function build a HCI LE event from the LE event Kernel message.
 *
 * @param[in]   msg    Kernel message containing the LE event
 * @return      Pointer to the beginning of the event
 *****************************************************************************************
 */
static uint8_t* hci_build_le_evt(struct ke_msg * msg)
{
    uint8_t* param = ke_msg2param(msg);
    uint8_t* buf = param - HCI_EVT_LE_PARAM_OFFSET;
    uint8_t* pk = buf;
    uint8_t subcode = *param;
    uint16_t par_len = msg->param_len;

    // Look for the event descriptor
    const struct hci_evt_desc_tag* evt_desc = hci_look_for_le_evt_desc(subcode);

    if(evt_desc != NULL)
    {
        uint8_t status = CO_UTIL_PACK_ERROR;

        if(evt_desc->par_fmt != NULL)
        {
            // Check if the generic packer can be used (basic fixed-length format)
            if(evt_desc->special_pack == PK_GEN)
            {
                // Pack the returned parameters
                status = co_util_pack(param, param, &par_len, par_len, evt_desc->par_fmt);
            }
            else
            {
                // Pack the parameters using the special packer
                status = ((hci_pkupk_func_t)evt_desc->par_fmt)(param, param, &par_len, par_len);
            }
        }

        ASSERT_INFO((status == CO_UTIL_PACK_OK), status, subcode);
        ASSERT_INFO(par_len <= msg->param_len, par_len, subcode);
        if(status == CO_UTIL_PACK_OK)
        {
            //pack event code
            *pk++ = HCI_LE_META_EVT_CODE;

            //pack event parameter length
            *pk++ = par_len;
        }
    }
    else
    {
        ASSERT_INFO(0, subcode, 0);
    }

    return buf;
}

#if (HCI_BLE_CON_SUPPORT)
/**
 ****************************************************************************************
 * @brief Build a HCI ACL RX data packet
 *
 * This function build a HCI ACL RX data packet from the Kernel message.
 *
 * @param[in]   msg    Kernel message associated to the HCI ACL RX data
 *****************************************************************************************
 */
static uint8_t* hci_build_ble_acl_rx_data(struct ke_msg * msg)
{
    // Point to message parameters structure
    struct hci_ble_acl_data_rx *param = (struct hci_ble_acl_data_rx *) ke_msg2param(msg);
    uint16_t handle_flags             = param->conhdl_pb_bc_flag;
    uint32_t pk_start_addr            = param->buf_ptr - HCI_ACL_HDR_LEN;

    // Set Connection handle and Packet Boundary and Broadcast Flags
    SETF(handle_flags, HCI_ACL_HDR_HDL, (msg->dest_id + BLE_CONHDL_MIN));
    em_wr16p(pk_start_addr + HCI_ACL_HDR_HDL_FLAGS_POS, handle_flags);
    // Pack the data length
    em_wr16p(pk_start_addr + HCI_ACL_HDR_DATA_LEN_POS, param->length);

    return (uint8_t *)(EM_BASE_ADDR + pk_start_addr);
}
#endif // (HCI_BLE_CON_SUPPORT)
#endif //BLE_EMB_PRESENT


#endif //(BLE_EMB_PRESENT || BT_EMB_PRESENT)


#if ((BLE_HOST_PRESENT) && (!BLE_EMB_PRESENT))

#if (HCI_BLE_CON_SUPPORT)
/**
 ****************************************************************************************
 * @brief Extract connection handle from a given format message
 *
 * This function returns the handle contained in a descriptor
 *
 * @param[in]   code    Kernel message containing the HCI command
 * @param[in]   payload Data
 * @param[in]   format  String containning the format
 * @return      Connection Handle
 *****************************************************************************************
 */
static uint16_t hci_look_for_conhdl(uint8_t code, uint8_t *payload, uint8_t *format)
{
    uint8_t *params = payload;
    uint8_t index = 0;
    uint16_t conhdl = GAP_INVALID_CONHDL;

    // Different frame format for CC events
    if ((code == HCI_CMD_CMP_EVT_CODE) ||
            (code == HCI_CMD_STATUS_EVT_CODE))
    {
        params += HCI_CMD_OPCODE_LEN + HCI_CMDEVT_PARLEN_LEN;
    }

    while(*(format + index) != '\0')
    {
        // Assume all parameters before connection handle are single bytes
        if(*(format + index) == 'H')
        {
            // Read connection handle
            conhdl = co_read16p(params + index);
            break;
        }
        // Increment index
        index++;
    }

    return (conhdl);
}

/**
****************************************************************************************
* @brief Search if a CS is expected
*
* @param[in]  code      message code
*
* @return     Connection index
*****************************************************************************************
*/
static uint8_t hci_search_cs_index(uint8_t code)
{
    uint8_t idx = GAP_INVALID_CONIDX;

    for (uint8_t i=0; i<BLE_CONNECTION_MAX; i++)
    {
        if (code == hci_env.ble_acl_con_tab[i].code)
        {
            // Get index
            idx = i;
            // Clear data
            hci_env.ble_acl_con_tab[i].code = 0;
            break;
        }
    }

    return(idx);
}

#endif //(HCI_BLE_CON_SUPPORT)
/**
 ****************************************************************************************
 * @brief Build a HCI command
 *
 * This function builds a HCI command from the Kernel message.
 *
 * @param[in]   msg    Kernel message containing the HCI command
 * @return      Pointer to the beginning of the event
 *****************************************************************************************
 */
static uint8_t* hci_build_cmd(struct ke_msg * msg)
{
    uint8_t* param = ke_msg2param(msg);
    uint8_t* buf = param - HCI_CMD_PARAM_OFFSET;
    uint8_t* pk = buf;
    uint16_t opcode = msg->src_id;
    uint16_t par_len = msg->param_len;
    uint8_t status = CO_UTIL_PACK_ERROR;

    // Look for the command descriptor
    const struct hci_cmd_desc_tag* cmd_desc = hci_look_for_cmd_desc(opcode);

    if((cmd_desc != NULL) && (par_len > 0))
    {
        if(cmd_desc->par_fmt != NULL)
        {
            // Check if the generic packer can be used (basic fixed-length format)
            if(!(cmd_desc->dest_field & HCI_CMD_DEST_SPEC_PAR_PK_MSK))
            {
                // Pack the returned parameters using the generic packer
                status = co_util_pack(param, param, &par_len, par_len, cmd_desc->par_fmt);

                #if ((HCI_BLE_CON_SUPPORT) && (BLE_HOST_PRESENT) && (!BLE_EMB_PRESENT))
                // Check if CS is expected

                uint8_t hl_type = (cmd_desc->dest_field & HCI_CMD_DEST_HL_MASK) >> HCI_CMD_DEST_HL_POS;
                if ((hl_type == HL_CTRL) && (cmd_desc->ret_par_fmt == NULL))
                {
                    // Parse connection handle
                    uint16_t conhdl = hci_look_for_conhdl(opcode, param, (uint8_t *)cmd_desc->par_fmt);
                    // Save code in order to route CS correctly
                    if (conhdl != GAP_INVALID_CONHDL)
                    {
                        // dest_id actually contains the connection index
                        hci_env.ble_acl_con_tab[msg->dest_id].code = opcode;
                    }
                    else
                    {
                        ASSERT_INFO(0, opcode, conhdl);
                    }
                }
                #endif //((HCI_BLE_CON_SUPPORT) && (BLE_HOST_PRESENT) && (!BLE_EMB_PRESENT))
            }
            else
            {
                // Pack the return parameters using the special packer
                status = ((hci_pkupk_func_t)cmd_desc->par_fmt)(param, param, &par_len, par_len);
            }
        }

        ASSERT_INFO((status == CO_UTIL_PACK_OK), status, opcode);
    }

    //pack command opcode
    co_write16p(pk, co_htobs(opcode));
    pk += 2;

    //pack command parameter length
    *pk++ = par_len;

    return (status == CO_UTIL_PACK_OK) ? buf : NULL;
}


#if (HCI_BLE_CON_SUPPORT)
/**
 ****************************************************************************************
 * @brief Build a HCI ACL TX data packet
 *
 * This function build a HCI ACL TX data packet from the Kernel message.
 *
 * @param[in]   msg    Kernel message associated to the HCI ACL TX data
 *****************************************************************************************
 */
static uint8_t* hci_build_acl_tx_data(struct ke_msg * msg)
{
    // Point to message parameters structure
    struct hci_ble_acl_data_tx *param = (struct hci_ble_acl_data_tx *) ke_msg2param(msg);

    #if (BLE_EMB_PRESENT)
    // Get the TX descriptor corresponding to the TX handle
    struct em_buf_tx_desc *txdesc = em_buf_tx_desc_get(param->desc->idx);
    // Point to the HCI header position within the descriptor space
    uint8_t* buf = (uint8_t *)em_buf_tx_buff_addr_get(em_buf_tx_desc_addr_get(hci_tl_env.txtag->idx)) - HCI_ACL_HDR_LEN;
    #else// (BLE_HOST_PRESENT)
    // retrieve buffer pointer
    uint8_t* buf = ((uint8_t*)param->buf_ptr) - HCI_ACL_HDR_LEN;
    #endif // (BLE_EMB_PRESENT) / (BLE_HOST_PRESENT)
    uint8_t* pk = buf;

    // Pack connection handle and data flags
    uint16_t handle_flags = param->conhdl_pb_bc_flag;
    co_write16p(pk, co_htobs(handle_flags));
    pk +=2;

    // Pack the data length
    co_write16p(pk, co_htobs(param->length));
    pk +=2;

    return buf;
}
#endif // (HCI_BLE_CON_SUPPORT)
#endif // ((BLE_HOST_PRESENT) && (!BLE_EMB_PRESENT))

/**
 ****************************************************************************************
 * @brief Check if a transmission has to be done and start it.
 *****************************************************************************************
 */
static void hci_tx_start(void)
{
    uint8_t *buf = NULL;
    uint16_t len = 0;
    uint8_t type = 0;

    // check we have message in the event Q
    struct ke_msg *msg = (struct ke_msg *)co_list_pick(&hci_tl_env.tx_queue);
    if (msg == NULL)
    {


#if (BT_EMB_PRESENT || (HCI_BLE_CON_SUPPORT && BLE_EMB_PRESENT))
        if (msg == NULL)
        {
            //if acl flow control allow us to send?
            if (hci_fc_check_host_available_nb_acl_packets())
            {
                // BLE ACL trafic here
                //check if we have acl data to be sent to HOST
               msg = (struct ke_msg *)co_list_pick(&hci_tl_env.acl_queue);
               if (msg == NULL)
               {
                   return;
               }
           }
       }
#endif //(BT_EMB_PRESENT ||  (HCI_BLE_CON_SUPPORT && BLE_EMB_PRESENT))

#if (HCI_BLE_CON_SUPPORT && !BLE_EMB_PRESENT)
        return;
#endif //(HCI_BLE_CON_SUPPORT && !BLE_EMB_PRESENT)
    }
    
    // need also to know what type of data was sent to empty the right Q
    // keep pointer to the message for processing when message transmitting is done
    hci_tl_env.curr_tx_msg = msg;

    // Check what kind of TX packet
    switch(msg->id)
    {

        #if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
        case HCI_CMD_CMP_EVENT:
        {
            // Increase the number of commands available for reception
            if(hci_tl_env.nb_h2c_cmd_pkts < HCI_NB_CMD_PKTS)
            {
                hci_tl_env.nb_h2c_cmd_pkts++;
            }
            // Build the HCI event
            buf = hci_build_cc_evt(msg);

            // Extract information (buffer, length, type)
            len = HCI_EVT_HDR_LEN + *(buf + HCI_EVT_CODE_LEN);
            type = HCI_EVT_MSG_TYPE;
        }
        break;
        case HCI_CMD_STAT_EVENT:
        {
            // Increase the number of commands available for reception
            if(hci_tl_env.nb_h2c_cmd_pkts < HCI_NB_CMD_PKTS)
            {
                hci_tl_env.nb_h2c_cmd_pkts++;
            }

            // Build the HCI event
            buf = hci_build_cs_evt(msg);

            // Extract information (buffer, length, type)
            len = HCI_EVT_HDR_LEN + *(buf + HCI_EVT_CODE_LEN);
            type = HCI_EVT_MSG_TYPE;
        }
        break;
        case HCI_EVENT:
        {
            // Build the HCI event
            buf = hci_build_evt(msg);

            // Extract information (buffer, length, type)
            len = HCI_EVT_HDR_LEN + *(buf + HCI_EVT_CODE_LEN);
            type = HCI_EVT_MSG_TYPE;
        }
        break;
        #if BLE_EMB_PRESENT
        case HCI_LE_EVENT:
        {
            // Build the HCI event
            buf = hci_build_le_evt(msg);

            // Extract information (buffer, length, type)
            len = HCI_EVT_HDR_LEN + *(buf + HCI_EVT_CODE_LEN);
            type = HCI_EVT_MSG_TYPE;
        }
        break;
        #if (HCI_BLE_CON_SUPPORT)
        case HCI_BLE_ACL_DATA_RX:
        {
            // Build the HCI data packet from Kernel message
            buf = hci_build_ble_acl_rx_data(msg);

            // Extract information (buffer, length, type)
            len = HCI_ACL_HDR_LEN + co_read16p(buf + HCI_ACL_HDR_HDL_FLAGS_LEN);
            type = HCI_ACL_MSG_TYPE;
        }
        break;
        #endif // (HCI_BLE_CON_SUPPORT)
        #endif //BLE_EMB_PRESENT

        #endif //(BLE_EMB_PRESENT || BT_EMB_PRESENT)

        #if ((BLE_HOST_PRESENT) && (!BLE_EMB_PRESENT))
        case HCI_COMMAND:
        {
            // Build the HCI command
            buf = hci_build_cmd(msg);

            // Extract information (buffer, length, type)
            len = HCI_CMD_HDR_LEN + *(buf + HCI_CMD_OPCODE_LEN);
            type = HCI_CMD_MSG_TYPE;
        }
        break;

        #if (HCI_BLE_CON_SUPPORT)
        case HCI_BLE_ACL_DATA_TX:
        {
            // Build the HCI data packet from Kernel message
            buf = hci_build_acl_tx_data(msg);

            // Extract information (buffer, length, type)
            len = HCI_ACL_HDR_LEN + co_read16p(buf + HCI_ACL_HDR_HDL_FLAGS_LEN);
            type = HCI_ACL_MSG_TYPE;
        }
        break;
        #endif // (HCI_BLE_CON_SUPPORT)
        #endif // ((BLE_HOST_PRESENT) && (!BLE_EMB_PRESENT))

        #if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
        // only one debug event supported for the moment, so flag to be
        // removed as soon as more debug event are supported
        #if (RW_DEBUG || (BLE_EMB_PRESENT && BLE_TESTER))
        case HCI_DBG_EVT:
        {
            // Build the DBG event
            buf = hci_build_dbg_evt(msg);

            // Extract information (buffer, length, type)
            len = HCI_EVT_HDR_LEN + *(buf + HCI_EVT_CODE_LEN);
            type = HCI_EVT_MSG_TYPE;
        }
        break;
        #endif // (RW_DEBUG || (BLE_EMB_PRESENT && BLE_TESTER))
        #endif // (BLE_EMB_PRESENT || BT_EMB_PRESENT)

        default:
        {
            ASSERT_INFO(0, msg->id, 0);
        }
        break;
    }

    // Set TX state
    hci_tl_env.tx_state = HCI_STATE_TX_ONGOING;

    #if (H4TL_SUPPORT)
    // Forward the message to the H4TL for immediate transmission
    h4tl_write(type, buf, len, &hci_tx_done);
    #endif //(H4TL_SUPPORT)
}

/**
 ****************************************************************************************
 * @brief Function called after sending message through UART, to free ke_msg space and
 * push the next message for transmission if any.
 *
 * The message is popped from the tx queue kept in hci_tl_env and freed using ke_msg_free.
 *****************************************************************************************
 */
static void hci_tx_done(void)
{
    struct ke_msg *msg = hci_tl_env.curr_tx_msg;
    GLOBAL_INT_DISABLE();


    // Go back to IDLE state
    hci_tl_env.tx_state = HCI_STATE_TX_IDLE;

    // Free the message resources
    switch(msg->id)
    {
        case HCI_EVENT:
        #if (BLE_EMB_PRESENT)
        case HCI_LE_EVENT:
        #endif // (BLE_EMB_PRESENT)
        #if (RW_DEBUG || (BLE_EMB_PRESENT && BLE_TESTER))
        case HCI_DBG_EVT:
        #endif // (RW_DEBUG || (BLE_EMB_PRESENT && BLE_TESTER))
        #if (TCI_LMP_ENABLED)
        case HCI_TCI_LMP:
        #endif //(TCI_LMP_ENABLED)
        case HCI_CMD_CMP_EVENT:
        case HCI_CMD_STAT_EVENT:
        case HCI_COMMAND:
        {
            // Nothing to do
        }
        break;
        #if (HCI_BLE_CON_SUPPORT && BLE_EMB_PRESENT)
        case HCI_BLE_ACL_DATA_RX:
        {
            struct hci_ble_acl_data_rx *data_rx;
            // Pop event from the event queue
            msg = (struct ke_msg *)((co_list_pop_front_handler)SVC_co_list_pop_front)(&hci_tl_env.acl_queue);
            data_rx = (struct hci_ble_acl_data_rx *)(msg->param);
            // Free the RX buffer associated with the message
            ble_util_buf_rx_free((uint16_t) data_rx->buf_ptr);
		
            hci_tl_env.curr_tx_msg = NULL;
            //count packets for flow control
            hci_fc_acl_packet_sent();
        }
        break;
        #endif // (HCI_BLE_CON_SUPPORT && BLE_EMB_PRESENT)
        #if (HCI_BLE_CON_SUPPORT && BLE_HOST_PRESENT && !BLE_EMB_PRESENT)
        case HCI_BLE_ACL_DATA_TX:
        {
//            // L2CC_PDU_SEND_RSP has been already sent TODO
//            struct hci_acl_data_tx *data_tx = (struct hci_acl_data_tx *)(msg->param);
//
//            // free ACL TX buffer
//            ((ke_free_handler)SVC_ke_free)(data_tx->buffer - (HCI_ACL_HDR_LEN + HCI_TRANSPORT_HDR_LEN));
        }
        break;
        #endif // (BLE_HOST_PRESENT && HCI_BLE_CON_SUPPORT && !BLE_EMB_PRESENT)

        default:
        {
            ASSERT_INFO(0, msg->id, 0);
        }
        break;
    }

    if (hci_tl_env.curr_tx_msg != NULL)
    {
        // Pop event from the event queue
        msg = (struct ke_msg *)((co_list_pop_front_handler)SVC_co_list_pop_front)(&hci_tl_env.tx_queue);
    }

    // Free the kernel message space
    ((ke_msg_free_handler)SVC_ke_msg_free)(msg);

    GLOBAL_INT_RESTORE();

    // Check if there is a new message pending for transmission
    hci_tx_start();
}

/**
 ****************************************************************************************
 * @brief Trigger the transmission over HCI TL if possible
 *****************************************************************************************
 */
static void hci_tx_trigger(void)
{
    // Check if there is no transmission ongoing
    if (hci_tl_env.tx_state == HCI_STATE_TX_IDLE)
    {
        // Start the transmission
        hci_tx_start();
    }
}


/*
 * MODULES INTERNAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void hci_tl_send(struct ke_msg *msg)
{
    GLOBAL_INT_DISABLE();

    /// put the message into corresponding queue
    switch(msg->id)
    {
        #if (BT_EMB_PRESENT || BLE_EMB_PRESENT)
        #if (HCI_BLE_CON_SUPPORT)
        case HCI_BLE_ACL_DATA_RX:
        #endif //(HCI_BLE_CON_SUPPORT)

        #if (HCI_BLE_CON_SUPPORT || BT_EMB_PRESENT)
        {
            ((co_list_push_back_handler)SVC_co_list_push_back)(&hci_tl_env.acl_queue, &msg->hdr);
        }
        break;
        #endif // (HCI_BLE_CON_SUPPORT || BT_EMB_PRESENT)
        #endif //(BT_EMB_PRESENT || BLE_EMB_PRESENT)


        default:
        {
            // Push the message into the HCI queue
            ((co_list_push_back_handler)SVC_co_list_push_back)(&hci_tl_env.tx_queue, &msg->hdr);
        }
        break;
    }

    GLOBAL_INT_RESTORE();

    // Trigger the HCI transmission
    hci_tx_trigger();
}


/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void hci_tl_init(bool reset)
{
    if(!reset)
    {
        // Reset the HCI environment
        memset(&hci_tl_env, 0, sizeof(hci_tl_env));

        // Initialize the HCI event transmit queues
        ((co_list_init_handler)SVC_co_list_init)(&hci_tl_env.tx_queue);

    #if (BT_EMB_PRESENT || HCI_BLE_CON_SUPPORT)
        // Initialize the BT ACL transmit queues
        ((co_list_init_handler)SVC_co_list_init)(&hci_tl_env.acl_queue);
    #endif //(BT_EMB_PRESENT || HCI_BLE_CON_SUPPORT)



        // Initialize TX state machine
        hci_tl_env.tx_state = HCI_STATE_TX_IDLE;
    }
    // Initialize the number of HCI commands the stack can handle simultaneously
    hci_tl_env.nb_h2c_cmd_pkts = HCI_NB_CMD_PKTS;
}

#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
uint8_t hci_cmd_get_max_param_size(uint16_t opcode)
{
    uint16_t max_param_size = HCI_MAX_CMD_PARAM_SIZE;

    // Find a command descriptor associated to the command opcode
    const struct hci_cmd_desc_tag* cmd_desc = hci_look_for_cmd_desc(opcode);

    // Check if the command is supported
    if(cmd_desc != NULL)
    {
        max_param_size = cmd_desc->par_size_max;
    }

    return (uint8_t) max_param_size;
}

void hci_cmd_received(uint16_t opcode, uint8_t length, uint8_t *payload)
{
    // Find a command descriptor associated to the command opcode
    const struct hci_cmd_desc_tag* cmd_desc = hci_look_for_cmd_desc(opcode);

    // Decrease the number of commands available for reception
    if(hci_tl_env.nb_h2c_cmd_pkts > INT8_MIN)
        hci_tl_env.nb_h2c_cmd_pkts--;

    // Check if the command is supported
    if(cmd_desc != NULL)
    {
        // Check if a new command can be received
        if (hci_tl_env.nb_h2c_cmd_pkts >= 0)
        {
            uint16_t dest = TASK_NONE;
            uint8_t ll_type = (cmd_desc->dest_field & HCI_CMD_DEST_LL_MASK) >> HCI_CMD_DEST_LL_POS;

            // Find the lower layers destination task
            switch(ll_type)
            {

                #if BLE_EMB_PRESENT
                case BLE_MNG:
                #if !BT_EMB_PRESENT
                case MNG:
                #endif //BT_EMB_PRESENT
                {
                    dest = TASK_LLM;
                }
                break;
                #endif //BLE_EMB_PRESENT
                case DBG:
                {
                    dest = TASK_DBG;
                }
                break;
                #if (BT_EMB_PRESENT || (BLE_EMB_PRESENT && (HCI_BLE_CON_SUPPORT)))
                
                #if (BLE_EMB_PRESENT && (HCI_BLE_CON_SUPPORT))
                case BLE_CTRL:
                #endif //(BLE_EMB_PRESENT && (HCI_BLE_CON_SUPPORT))
                case CTRL:
                {

                    {
                        // Check if the parameters can contain a connection handle
                        if(length >= 2)
                        {
                            // Retrieve connection handle from command parameters (expecting at payload 1st 2 bytes)
                            uint16_t conhdl = co_btohs(co_read16p(payload));

                            #if (BLE_EMB_PRESENT && (HCI_BLE_CON_SUPPORT))
                            // Check if the connection handle corresponds to an active BLE link
                            if((conhdl < BLE_CONNECTION_MAX) && hci_env.ble_con_state[conhdl])
                            {
                                // Build the destination task ID
                                dest = KE_BUILD_ID(TASK_LLC, conhdl);
                            }
                            #endif //(BLE_EMB_PRESENT && (HCI_BLE_CON_SUPPORT))


                        }
                    }

                    // Reject command if not possible to match a valid connection
                    if(dest == TASK_NONE)
                    {
                        // Reject the command with error code "Unknown connection identifier"
                        hci_cmd_reject(cmd_desc, opcode, CO_ERROR_UNKNOWN_CONNECTION_ID);
                    }
                }
                break;
                #endif //(BT_EMB_PRESENT || (BLE_EMB_PRESENT && (HCI_BLE_CON_SUPPORT)))
                default:
                {
                    ASSERT_INFO(0, ll_type, opcode);
                }
                break;
            }

            // Check if the command can be handled by the controller
            if(dest != TASK_NONE)
            {
                uint16_t unpk_length = 0;
                uint8_t status = CO_UTIL_PACK_OK;

                // Check if there are parameters (to compute the unpacked parameters size for Kernel message allocation)
                if (length > 0)
                {
                    if(cmd_desc->par_fmt != NULL)
                    {
                        // Check if the generic packer can be used (basic fixed-length format)
                        if(!(cmd_desc->dest_field & HCI_CMD_DEST_SPEC_PAR_PK_MSK))
                        {
                            // Compute the space needed for unpacked parameters
                            status = co_util_unpack(NULL, NULL, &unpk_length, length, cmd_desc->par_fmt);
                        }
                        else
                        {
                            status = ((hci_pkupk_func_t)cmd_desc->par_fmt)(NULL, payload, &unpk_length, length);
                        }
                    }
                    else
                    {
                        status = CO_UTIL_PACK_ERROR;
                    }
                }

                // Check if there is input buffer overflow (received parameter size is less than expected for this command)
                if(status == CO_UTIL_PACK_IN_BUF_OVFLW)
                {
                    // Reject the command with error code "Invalid HCI parameters"
                    hci_cmd_reject(cmd_desc, opcode, CO_ERROR_INVALID_HCI_PARAM);
                }
                else
                {
                    // Allocate a Kernel message (with space for unpacked parameters)
                    void* cmd = ((ke_msg_alloc_handler)SVC_ke_msg_alloc)(HCI_COMMAND, dest, opcode, unpk_length);

                    ASSERT_INFO(status == CO_UTIL_PACK_OK, status, opcode);

                    if(cmd == NULL)
                    {
                        // Reject the command with error code "Memory Capacity Exceeded"
                        hci_cmd_reject(cmd_desc, opcode, CO_ERROR_MEMORY_CAPA_EXCEED);
                    }
                    else
                    {
                        // Check if there are parameters to unpack
                        if ((unpk_length > 0) && (cmd_desc->par_fmt != NULL))
                        {
                            // Check if the generic packer can be used (basic fixed-length format)
                            if(!(cmd_desc->dest_field & HCI_CMD_DEST_SPEC_PAR_PK_MSK))
                            {
                                // Unpack parameters
                                status = co_util_unpack((uint8_t*) cmd, payload, &unpk_length, length, cmd_desc->par_fmt);
                            }
                            else
                            {
                                status = ((hci_pkupk_func_t)cmd_desc->par_fmt)((uint8_t*) cmd, payload, &unpk_length, length);
                            }
                        }

                        ASSERT_INFO(status == CO_UTIL_PACK_OK, status, opcode);

                        // Send the command to the internal destination task associated to this command
                        ((ke_msg_send_handler)SVC_ke_msg_send)(cmd);
                    }
                }
            }
        }
        else
        {
            // Reject the command with error code "Memory Capacity Exceeded"
            hci_cmd_reject(cmd_desc, opcode, CO_ERROR_MEMORY_CAPA_EXCEED);
        }
    }
    else
    {
        // Reject the command with error code "Unknown HCI Command"
        hci_cmd_reject(NULL, opcode, CO_ERROR_UNKNOWN_HCI_COMMAND);
    }
}

uint8_t* hci_acl_tx_data_alloc(uint16_t hdl_flags, uint16_t len)
{
    uint8_t* buf = NULL;
    uint16_t dest = TASK_NONE;
    uint16_t max_len = 0;

    #if ((BLE_EMB_PRESENT && (HCI_BLE_CON_SUPPORT)) || BT_EMB_PRESENT)
    // Retrieve connection handle from command parameters (expecting at payload 1st 2 bytes)
    uint16_t conhdl  = GETF(hdl_flags, HCI_ACL_HDR_HDL);
    uint8_t  bc_flag = GETF(hdl_flags, HCI_ACL_HDR_BC_FLAG);
    #endif // ((BLE_EMB_PRESENT && (HCI_BLE_CON_SUPPORT)) || BT_EMB_PRESENT)

    #if (BLE_EMB_PRESENT && (HCI_BLE_CON_SUPPORT))
    // Check if the connection handle corresponds to an active BLE link
    if((conhdl < BLE_CONNECTION_MAX) && (bc_flag == BCF_P2P))
    {
        // Build the destination task ID
        dest = KE_BUILD_ID(TASK_LLC, conhdl);
        max_len = BLE_MAX_OCTETS;
    }
    #endif //(BLE_EMB_PRESENT && (HCI_BLE_CON_SUPPORT))



    // Check if the requested size fits within BT/BLE data max length
    if((dest != TASK_NONE) && (len <= max_len))
    {
        switch(KE_TYPE_GET(dest))
        {
            #if (BLE_EMB_PRESENT && (HCI_BLE_CON_SUPPORT))
            case TASK_LLC:
            {
                // Try allocating a buffer from BLE pool
               uint16_t buf_ptr = ble_util_buf_acl_tx_alloc();
				ASSERT_ERR(0);

                if(buf_ptr != 0)
                {
                    // Give the pointer to the data space
                    buf = (uint8_t*) (EM_BASE_ADDR + buf_ptr);
                }
            }
            break;
            #endif //(BLE_EMB_PRESENT && (HCI_BLE_CON_SUPPORT))



            default:
            {
                ASSERT_ERR(0);
            }
            break;
        }

        if(buf == NULL)
        {
            // Report buffer oveflow
            struct hci_data_buf_ovflw_evt * evt = KE_MSG_ALLOC(HCI_EVENT, 0, HCI_DATA_BUF_OVFLW_EVT_CODE, hci_data_buf_ovflw_evt);
            evt->link_type = ACL_TYPE;
            hci_send_2_host(evt);

            // Allocate a temporary buffer from heap
            buf = ((ke_malloc_handler)SVC_ke_malloc)(len, KE_MEM_KE_MSG);

            // Indicate the reception of ACL payload in a trash buffer
            hci_tl_env.acl_trash_payload = true;
        }
    }

    return buf;
}

void hci_acl_tx_data_received(uint16_t hdl_flags, uint16_t datalen, uint8_t * payload)
{
    do
    {
        #if ((BLE_EMB_PRESENT && (HCI_BLE_CON_SUPPORT)) || BT_EMB_PRESENT)
        uint16_t conhdl = GETF(hdl_flags, HCI_ACL_HDR_HDL);
        #endif // ((BLE_EMB_PRESENT && (HCI_BLE_CON_SUPPORT)) || BT_EMB_PRESENT)

        #if (HCI_BLE_CON_SUPPORT)
        // Check if the received packet was considered BLE one at header reception
        if((payload != NULL) && !hci_tl_env.acl_trash_payload)
        {
            ASSERT_ERR(conhdl < BLE_CONNECTION_MAX);

            if(conhdl < BLE_CONNECTION_MAX)
            {
                // Allocate a Kernel message
                struct hci_ble_acl_data_tx* data_tx = KE_MSG_ALLOC(HCI_BLE_ACL_DATA_TX, KE_BUILD_ID(TASK_LLC, conhdl), 0, hci_ble_acl_data_tx);
                data_tx->conhdl_pb_bc_flag          = hdl_flags;
                data_tx->length                     = datalen;
                data_tx->buf_ptr                    = ((uint32_t) payload) - EM_BASE_ADDR;
                ((ke_msg_send_handler)SVC_ke_msg_send)(data_tx);
            }
            else
            {
                // free unused buffer
                ble_util_buf_acl_tx_free(((uint32_t) payload) - EM_BASE_ADDR);
            }

            break;
        }
        #endif //(HCI_BLE_CON_SUPPORT)



        // Free the temporary buffer
        ((ke_free_handler)SVC_ke_free)(payload);

        // Clear the indication of ACL payload reception in a trash buffer
        hci_tl_env.acl_trash_payload = false;

    } while(0);
}


#endif //(BLE_EMB_PRESENT || BT_EMB_PRESENT)

#if (BLE_HOST_PRESENT && !BLE_EMB_PRESENT)
#if (HCI_BLE_CON_SUPPORT)
uint8_t* hci_acl_rx_data_alloc(uint16_t hdl_flags, uint16_t len)
{
    return ((ke_malloc_handler)SVC_ke_malloc)(len, KE_MEM_NON_RETENTION);
}

void hci_acl_rx_data_received(uint16_t hdl_flags, uint16_t datalen, uint8_t * payload)
{
    uint16_t conhdl = GETF(hdl_flags, HCI_ACL_HDR_HDL);

    // Get connection index
    uint8_t idx = gapc_get_conidx(conhdl);

    if ((conhdl != GAP_INVALID_CONHDL) &&
            (idx != GAP_INVALID_CONIDX))
    {
        // Allocate a Kernel message (with space for unpacked parameters)
        struct hci_ble_acl_data_rx* data_rx = KE_MSG_ALLOC(HCI_BLE_ACL_DATA_RX, KE_BUILD_ID(TASK_L2CC, idx), 0, hci_ble_acl_data_rx);
        data_rx->conhdl_pb_bc_flag = hdl_flags;

        data_rx->length = datalen;

        // retrieve data buffer
        data_rx->buf_ptr = (uint32_t) payload;

        // Send the kernel message
        ((ke_msg_send_handler)SVC_ke_msg_send)(data_rx);
    }
    else
    {
        ASSERT_INFO(0, conhdl, idx);
    }
}
#endif // (HCI_BLE_CON_SUPPORT)

uint8_t hci_evt_received(uint8_t code, uint8_t length, uint8_t *payload)
{
    uint8_t status = CO_UTIL_PACK_OK;

    switch(code)
    {
        case HCI_CMD_CMP_EVT_CODE:
        {
            if(length >= HCI_CCEVT_HDR_PARLEN)
            {
                // Retrieve opcode from parameters, expected at position 1 in payload (after nb cmp pkts)
                uint16_t opcode = co_btohs(co_read16p(payload + 1));
                // Look for the command descriptor
                const struct hci_cmd_desc_tag* cmd_desc = hci_look_for_cmd_desc(opcode);

                // Check if the command is supported
                if(cmd_desc != NULL)
                {
                    uint16_t dest = TASK_NONE;
                    uint8_t hl_type = (cmd_desc->dest_field & HCI_CMD_DEST_HL_MASK) >> HCI_CMD_DEST_HL_POS;

                    // Find the higher layers destination task
                    switch(hl_type)
                    {
                        case HL_MNG:
                        {
                            // Build the destination task ID
                            dest = TASK_GAPM;
                        }
                        break;
                        case HL_CTRL:
                        case HL_DATA:
                        {
#if (HCI_BLE_CON_SUPPORT)
                            // Parse connection handle
                            uint16_t conhdl = hci_look_for_conhdl(code, payload, (uint8_t *)cmd_desc->ret_par_fmt);
                            // Get connection index
                            uint8_t idx = gapc_get_conidx(conhdl);

                            if ((conhdl != GAP_INVALID_CONHDL) &&
                                    (idx != GAP_INVALID_CONIDX))
                            {
                                // Build the destination task ID if found
                                dest = (hl_type == HL_CTRL) ?
                                        KE_BUILD_ID(TASK_GAPC, idx) : KE_BUILD_ID(TASK_L2CC, idx);
                            }
#else
                            // Forward message to the first instance
                            dest = (hl_type == HL_CTRL) ?
                                    TASK_GAPC : TASK_L2CC;
#endif //(HCI_BLE_CON_SUPPORT)
                        }
                        break;

                        default:
                        {
                            ASSERT_INFO(0, hl_type, opcode);
                        }
                        break;
                    }

                    if(dest != TASK_NONE)
                    {
                        uint16_t unpk_length = length - HCI_CCEVT_HDR_PARLEN;
                        void* evt;

                        // Check if there are parameters
                        if (unpk_length > 0)
                        {
                            if(cmd_desc->ret_par_fmt != NULL)
                            {
                                // Check if the generic packer can be used (basic fixed-length format)
                                if(!(cmd_desc->dest_field & HCI_CMD_DEST_SPEC_RET_PAR_PK_MSK))
                                {
                                    // Compute the space needed for unpacked parameters
                                    status = co_util_unpack(NULL, NULL, &unpk_length, length - HCI_CCEVT_HDR_PARLEN, cmd_desc->ret_par_fmt);
                                }
                                else
                                {
                                    status = ((hci_pkupk_func_t)cmd_desc->ret_par_fmt)(NULL, NULL, &unpk_length, length - HCI_CCEVT_HDR_PARLEN);
                                }
                            }
                            else
                            {
                                status = CO_UTIL_PACK_ERROR;
                            }

                            ASSERT_INFO((status == CO_UTIL_PACK_OK), status, opcode);
                        }

                        // Allocate a Kernel message (with space for unpacked parameters)
                        evt = ((ke_msg_alloc_handler)SVC_ke_msg_alloc)(HCI_CMD_CMP_EVENT, dest, opcode, unpk_length);

                        // Check if there are parameters to unpack
                        if (unpk_length > 0)
                        {
                            if(cmd_desc->ret_par_fmt != NULL)
                            {
                                // Check if the generic unpacker can be used (basic fixed-length format)
                                if(!(cmd_desc->dest_field & HCI_CMD_DEST_SPEC_RET_PAR_PK_MSK))
                                {
                                    // Unpack parameters
                                    status = co_util_unpack((uint8_t*) evt, payload + HCI_CCEVT_HDR_PARLEN, &unpk_length, length - HCI_CCEVT_HDR_PARLEN, cmd_desc->ret_par_fmt);
                                }
                                else
                                {
                                    status = ((hci_pkupk_func_t)cmd_desc->ret_par_fmt)((uint8_t*) evt, payload + HCI_CCEVT_HDR_PARLEN, &unpk_length, length - HCI_CCEVT_HDR_PARLEN);
                                }
                            }
                        }

                        // Send the command to the internal destination task associated to this event
                        ((ke_msg_send_handler)SVC_ke_msg_send)(evt);
                    }
                    else
                    {
                        ASSERT_INFO(0, hl_type, opcode);
                    }
                }
                else
                {
                    ASSERT_INFO(0, opcode, 0);
                }
            }
            else
            {
                ASSERT_INFO(0, length, 0);
            }
        }
        break;
        case HCI_CMD_STATUS_EVT_CODE:
        {
            if(length == HCI_CSEVT_PARLEN)
            {
                // Retrieve opcode from parameters, expected at position 2 in payload (after status and nb cmp pkts)
                uint16_t opcode = co_btohs(co_read16p(payload + 2));
                // Look for the command descriptor
                const struct hci_cmd_desc_tag* cmd_desc = hci_look_for_cmd_desc(opcode);

                // Check if the command is supported
                if(cmd_desc != NULL)
                {
                    uint16_t dest = TASK_NONE;
                    uint8_t hl_type = (cmd_desc->dest_field & HCI_CMD_DEST_HL_MASK) >> HCI_CMD_DEST_HL_POS;

                    // Find the higher layers destination task
                    switch(hl_type)
                    {
                        case HL_MNG:
                        {
                            // Build the destination task ID
                            dest = TASK_GAPM;
                        }
                        break;

                        case HL_CTRL:
                        case HL_DATA:
                        {
#if (HCI_BLE_CON_SUPPORT)
                            // Get connection index
                            uint8_t idx = hci_search_cs_index(opcode);

                            if (idx != GAP_INVALID_CONIDX)
                            {
                                // Build the destination task ID if found
                                dest = (hl_type == HL_CTRL) ?
                                        KE_BUILD_ID(TASK_GAPC, idx) : KE_BUILD_ID(TASK_L2CC, idx);
                            }
#else
                            // Forward message to the first instance
                            dest = (hl_type == HL_CTRL) ?
                                    TASK_GAPC : TASK_L2CC;
#endif //(HCI_BLE_CON_SUPPORT)
                        }
                        break;

                        default:
                        {
                            ASSERT_INFO(0, hl_type, opcode);
                        }
                        break;
                    }

                    if(dest != TASK_NONE)
                    {
                        // Allocate a Kernel message (with space for unpacked parameters)
                        void* evt = ((ke_msg_alloc_handler)SVC_ke_msg_alloc)(HCI_CMD_STAT_EVENT, dest, opcode, length);

                        // Send the command to the internal destination task associated to this event
                        ((ke_msg_send_handler)SVC_ke_msg_send)(evt);
                    }
                    else
                    {
                        ASSERT_INFO(0, hl_type, opcode);
                    }
                }
                else
                {
                    ASSERT_INFO(0, opcode, 0);
                }
            }
            else
            {
                ASSERT_INFO(0, length, 0);
            }
        }
        break;
        case HCI_LE_META_EVT_CODE:
        default:
        {
            uint8_t code_or_subcode = code;
            // Find an event descriptor associated to the event code
            const struct hci_evt_desc_tag* evt_desc;
            if(code_or_subcode == HCI_LE_META_EVT_CODE)
            {
                code_or_subcode = *payload;
                evt_desc = hci_look_for_le_evt_desc(code_or_subcode);
            }
            else
            {
                evt_desc = hci_look_for_evt_desc(code_or_subcode);
            }

            // Check if the evt is supported
            if(evt_desc != NULL)
            {
                uint16_t dest = TASK_NONE;
                uint8_t hl_type = (evt_desc->dest_field & HCI_EVT_DEST_HL_MASK) >> HCI_EVT_DEST_HL_POS;

                // Find the higher layers destination task
                switch(hl_type)
                {
                    case HL_MNG:
                    {
                        // Build the destination task ID
                        dest = TASK_GAPM;
                    }
                    break;

                    case HL_CTRL:
                    case HL_DATA:
                    {
#if (HCI_BLE_CON_SUPPORT)
                        // Parse connection handle
                        uint16_t conhdl = hci_look_for_conhdl(code, payload, (uint8_t *)evt_desc->par_fmt);
                        // Get connection index
                        uint8_t idx = gapc_get_conidx(conhdl);

                        if ((conhdl != GAP_INVALID_CONHDL) &&
                                (idx != GAP_INVALID_CONIDX))
                        {
                            // Build the destination task ID if found
                            dest = (hl_type == HL_CTRL) ?
                                    KE_BUILD_ID(TASK_GAPC, idx) : KE_BUILD_ID(TASK_L2CC, idx);
                        }
#else
                        // Forward message to the first instance
                        dest = (hl_type == HL_CTRL) ?
                                TASK_GAPC : TASK_L2CC;
#endif //(HCI_BLE_CON_SUPPORT)
                    }
                    break;

                    default:
                    {
                        ASSERT_INFO(0, hl_type, code);
                    }
                    break;
                }

                if(dest != TASK_NONE)
                {
                    uint16_t unpk_length = 0;
                    void* evt;

                    // Check if there are parameters
                    if (length > 0)
                    {
                        if(evt_desc->par_fmt != NULL)
                        {
                            // Check if the generic packer can be used (basic fixed-length format)
                            if(evt_desc->special_pack == PK_GEN)
                            {
                                // Compute the space needed for unpacked parameters
                                status = co_util_unpack(NULL, NULL, &unpk_length, length, evt_desc->par_fmt);
                            }
                            else
                            {
                                status = ((hci_pkupk_func_t)evt_desc->par_fmt)(NULL, NULL, &unpk_length, length);
                            }

                            ASSERT_INFO((status == CO_UTIL_PACK_OK), status, code);
                        }
                        else
                        {
                            status = CO_UTIL_PACK_ERROR;
                        }
                    }

                    // Allocate a Kernel message (with space for unpacked parameters)
                    if(code == HCI_LE_META_EVT_CODE)
                    {
                        evt = ((ke_msg_alloc_handler)SVC_ke_msg_alloc)(HCI_LE_EVENT, dest, 0, unpk_length);
                    }
                    else
                    {
                        evt = ((ke_msg_alloc_handler)SVC_ke_msg_alloc)(HCI_EVENT, dest, code_or_subcode, unpk_length);
                    }

                    // Check if there are parameters to unpack
                    if (unpk_length > 0)
                    {
                        if(evt_desc->par_fmt != NULL)
                        {
                            // Check if the generic unpacker can be used (basic fixed-length format)
                            if(evt_desc->special_pack == PK_GEN)
                            {
                                // Unpack parameters
                                status = co_util_unpack((uint8_t*) evt, payload, &unpk_length, length, evt_desc->par_fmt);
                            }
                            else
                            {
                                status = ((hci_pkupk_func_t)evt_desc->par_fmt)((uint8_t*) evt, payload, &unpk_length, length);
                            }
                        }

                        ASSERT_INFO((status == CO_UTIL_PACK_OK), status, code);
                    }

                    // Send the command to the internal destination task associated to this event
                    ((ke_msg_send_handler)SVC_ke_msg_send)(evt);
                }
                else
                {
                    ASSERT_INFO(0, hl_type, code_or_subcode);
                }
            }
            else
            {
                ASSERT_INFO(0, code_or_subcode, 0);
            }
        }
        break;
    }

    return (status);
}
#endif //(BLE_HOST_PRESENT && !BLE_EMB_PRESENT)


#endif // (HCI_TL_SUPPORT)
#endif //(HCI_PRESENT)

/// @} HCI
