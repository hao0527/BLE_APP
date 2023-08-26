
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

#include "panip_config.h"     // SW configuration

#if (BLE_HCIC_ITF)

#include <string.h>          // string manipulation
#include "co_error.h"        // error definition
#include "co_utils.h"        // common utility definition

#include "hcic.h"
#include "hcic_task.h"
#include "ke_msg.h"          // kernel message defines

#if (BLE_CENTRAL || BLE_PERIPHERAL)
#include "llc_task.h"
#endif // #if (BLE_CENTRAL || BLE_PERIPHERAL)

#if (DEEP_SLEEP)
#include "rwip.h"
#endif // DEEP_SLEEP

/*
 * DEFINES
 ****************************************************************************************
 */


/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */



/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @brief Function called to send a kernel message through UART.
 *
 * @param[in]  msgid   U16 message id from ke_msg.
 * @param[in] *param   Pointer to parameters of the message in ke_msg.
 * @param[in]  dest_id Destination task id.
 * @param[in]  src_id  Source task ID.
 *
 * @return             Kernel message state, must be KE_MSG_NO_FREE.
 *****************************************************************************************
 */
static int hci_msg_send_handler (ke_msg_id_t const msgid,
                          void const *param,
                          ke_task_id_t const dest_id,
                          ke_task_id_t const src_id)
{
     //extract the ke_msg pointer from the param passed
    struct ke_msg * msg = ke_param2msg(param);

    #if (BLE_CENTRAL || BLE_PERIPHERAL)
    //check if it is data
    if (msgid == LLC_LE_DATA_IND)
    {
        msg->hci_len = hci_data_pk(msg);
        msg->hci_type = HCI_ACL_MSG_TYPE;
        msg->hci_off = -HCI_ACL_HDR_LEN;
    }

    //event or command
    else
    #endif // BLE_CENTRAL || BLE_PERIPHERAL
    {
        //call the appropriate packing handler
        int msg_idx;
        switch(MSG_T(msgid))
        {
            //when it is not data, in lower layers only events can be sent
            case TASK_LLM:
                msg_idx = LLM_EVTID2IDX(msgid);
                msg->hci_len = hci_llm_epk[msg_idx].func(msg);
                msg->hci_type = HCI_EVT_MSG_TYPE;
                msg->hci_off = hci_llm_epk[msg_idx].evt_type;
                break;

            #if (BLE_CENTRAL || BLE_PERIPHERAL)
            case TASK_LLC:
                msg_idx = LLC_EVTID2IDX(msgid);
                msg->hci_len = hci_llc_epk[msg_idx].func(msg);
                msg->hci_type = HCI_EVT_MSG_TYPE;
                msg->hci_off = hci_llc_epk[msg_idx].evt_type;
                break;
            #endif // BLE_CENTRAL || BLE_PERIPHERAL

            #if (BLE_DEBUG)
            case TASK_DBG:
                msg_idx = DBG_EVTID2IDX(msgid);
                msg->hci_len = hci_dbg_epk[msg_idx].func(msg);
                msg->hci_type = HCI_EVT_MSG_TYPE;
                msg->hci_off = hci_dbg_epk[msg_idx].evt_type;
                break;
            #endif //BLE_DEBUG

            default:
                // Sanity check
                ASSERT_ERR(0);
                break;
        }
    }

    // Push the message into the HCI queue
    hci_push(msg);

    //return NO_FREE always since hci_eif_write handles the freeing
    return KE_MSG_NO_FREE;
}


#if (DEEP_SLEEP)
/**
 ****************************************************************************************
 * @brief Handles the delay between 2 consecutive  HCI cmd.
 * The handler allow the reception of a new host channel classification command.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int hci_polling_to_handler(ke_msg_id_t const msgid,
                        void const *param,
                        ke_task_id_t const dest_id,
                        ke_task_id_t const src_id)
{
    // Clear the HCI timeout
    rwip_prevent_sleep_clear(RW_HCI_TIMEOUT);

    return (KE_MSG_CONSUMED);
}
#endif // DEEP_SLEEP

/*
 * TASK DESCRIPTOR DEFINITIONS
 ****************************************************************************************
 */

/// Specifies the message handlers that are common to all states.
const struct ke_msg_handler hci_default_state[] =
{

    /** Default handler for HCI TX message, this entry has to be put first as table is
        parsed from end to start by Kernel */
    {KE_MSG_DEFAULT_HANDLER, (ke_msg_func_t)hci_msg_send_handler},

    #if (DEEP_SLEEP)
    {HCI_POLLING_TO, (ke_msg_func_t)hci_polling_to_handler},
    #endif // DEEP_SLEEP
};

/// Specifies the message handlers that are common to all states.
const struct ke_state_handler hci_default_handler = KE_STATE_HANDLER(hci_default_state);

/// Defines the placeholder for the states of all the task instances.
ke_state_t hci_state[HCI_IDX_MAX];


#endif //BLE_HCIC_ITF

/// @} HCI
