
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

#include <string.h>          // string manipulation
#include "co_error.h"        // error definition
#include "co_utils.h"        // common utility definition
#include "co_list.h"         // list definition

#include "hci.h"             // hci definition
#include "hci_int.h"         // hci internal definition

#include "ke_msg.h"          // kernel message declaration
#include "ke_task.h"         // kernel task definition
#include "ke_event.h"        // kernel event definition
#include "ke_mem.h"          // kernel memory definition
#include "ke_timer.h"        // kernel timer definition

#if(BLE_HOST_PRESENT && BLE_EMB_PRESENT && HCI_TL_SUPPORT)
#include "gapm.h"            // use to check if embedded host is enabled or not
#endif // (BLE_HOST_PRESENT && BLE_EMB_PRESENT && HCI_TL_SUPPORT)

#include "dbg.h"
#include "stack_svc_api.h"

/*
 * DEFINES
 ****************************************************************************************
 */


/*
 * ENUMERATIONS DEFINITIONS
 ****************************************************************************************
 */

/*
 * STRUCTURES DEFINITIONS
 ****************************************************************************************
 */



/*
 * CONSTANTS DEFINITIONS
 ****************************************************************************************
 */

/// Default event mask
static const struct evt_mask hci_def_evt_msk =  {{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x1F, 0x00, 0x00}};

/// Reserved event mask
//static const struct evt_mask hci_rsvd_evt_msk = {{0x00, 0x60, 0x04, 0x00, 0xF8, 0x07, 0x40, 0x02}};

static const struct evt_mask hci_rsvd_evt_msk = {{0x00, 0x60, 0x04, 0x00, 0xF8, 0x07, 0x40, 0x22}}; 
	// mask[7].bit(5)  for le event, set to 1, not masked;    0 masked


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

///HCI environment context
struct hci_env_tag hci_env;


/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
/**
 ****************************************************************************************
 * @brief Check if the event to be sent to the host is masked or not
 *
 * @param[in] msg  Pointer to the message containing the event
 *
 * @return true id the message has to be filtered out, false otherwise
 *****************************************************************************************
 */
static bool hci_evt_mask_check(struct ke_msg *msg)
{
    bool masked = false;
    uint8_t evt_code;

    switch(msg->id)
    {
        case HCI_LE_EVENT:
        {
            // LE meta event
            evt_code = HCI_LE_META_EVT_CODE;
        }
        break;
        case HCI_EVENT:
        {
            // Get event code
            evt_code = msg->src_id;
        }
        break;

        default:
        {
            // Cannot be masked
            return false;
        }
    }

    // Check if this event is maskable
    if(evt_code < HCI_MAX_EVT_MSK_PAGE_1_CODE)
    {
        uint8_t index = evt_code - 1;

        //Checking if the event is masked or not
        masked = ((hci_env.evt_msk.mask[index/8] & (1<<(index - ((index/8)*8)))) == 0x00);
//proj_con_printf("index:%d,msk:%x\n",index,hci_env.evt_msk.mask[index/8]);

        #if (BLE_EMB_PRESENT)
        if (!masked && (evt_code == HCI_LE_META_EVT_CODE))
        {
            // Get Sub-code of the mevent
            uint8_t *subcode = (uint8_t*)ke_msg2param(msg);
            //Translate the subcode in index to parse the mask
            uint8_t index = *subcode - 1;
//proj_con_printf("index:%d\n",index);
            //Checking if the event is masked or not
            masked =((hci_env.le_evt_msk.mask[index/8] & (1<<(index - ((index/8)*8)))) == 0x00);
//proj_con_printf("masked:%d\n",masked);			
        }
        #endif // (BLE_EMB_PRESENT)
    }
    else if(evt_code < HCI_MAX_EVT_MSK_PAGE_2_CODE)
    {
        // In this area the evt code is in the range [EVT_MASK_CODE_MAX<evt_code<HCI_MAX_EVT_MSK_CODE]
        // The index should be < EVT_MASK_CODE_MAX to avoid evt_msk_page_2.mask array overflow
        uint8_t index = evt_code - EVT_MASK_CODE_MAX;

        //Checking if the event is masked or not
        masked = ((hci_env.evt_msk_page_2.mask[index/8] & (1<<(index - ((index/8)*8)))) == 0x00);
    }
    return masked;
}
#endif //(BLE_EMB_PRESENT || BT_EMB_PRESENT)




/*
 * MODULES INTERNAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */


/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void hci_init(bool reset)
{
    memset(&hci_env, 0, sizeof(hci_env));

    // Initialize event mask
    hci_evt_mask_set(&hci_def_evt_msk, HCI_PAGE_DFT);

    #if (BLE_EMB_PRESENT)
    // Initialize LE event mask
    hci_env.le_evt_msk.mask[0] = LE_EVT_MASK_DFT;
    memset(&hci_env.le_evt_msk.mask[1], 0, (EVT_MASK_LEN-1));
	//memset(&hci_env.le_evt_msk.mask[0], 0xFF, (EVT_MASK_LEN));
    #endif // (BLE_EMB_PRESENT)
	
//show_reg2(hci_env.le_evt_msk.mask,EVT_MASK_LEN);

    #if (HCI_TL_SUPPORT)
    // Reset the HCI
    hci_tl_init(reset);
    #endif //(HCI_TL_SUPPORT)
    #if (BT_EMB_PRESENT || BLE_EMB_PRESENT)
    hci_fc_init();
    #endif //(BT_EMB_PRESENT || BLE_EMB_PRESENT)
}

#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
void hci_send_2_host(void *param)
{
    struct ke_msg *msg = ke_param2msg(param);
	
hci_printf("chk_msg:%d\n",hci_evt_mask_check(msg));

    if(   hci_evt_mask_check(msg)
  
      )
    {
        // Free the kernel message space
        ((ke_msg_free_handler)SVC_ke_msg_free)(msg);
        return;
    }

    #if BLE_HOST_PRESENT
    #if(BLE_EMB_PRESENT && HCI_TL_SUPPORT)
    // check if communication is performed over embedded host
    if(gapm_is_embedded_host())
    #endif // (BLE_EMB_PRESENT && HCI_TL_SUPPORT)
    {
        ke_task_id_t dest = TASK_NONE;
        uint8_t hl_type = HL_UNDEF;
hci_printf("hl:msg->id:%04X\n",msg->id);	

        // The internal destination first depends on the message type (command, event, data)
        switch(msg->id)
        {
            case HCI_CMD_STAT_EVENT:
            case HCI_CMD_CMP_EVENT:
            {
                if(msg->src_id != HCI_NO_OPERATION_CMD_OPCODE)
                {
                    // Find a command descriptor associated to the command opcode
                    const struct hci_cmd_desc_tag* cmd_desc = hci_look_for_cmd_desc(msg->src_id);

                    // Check if the command is supported
                    if(cmd_desc != NULL)
                    {
                        hl_type = (cmd_desc->dest_field & HCI_CMD_DEST_HL_MASK) >> HCI_CMD_DEST_HL_POS;
hci_printf("hl_type1:%02X\n",hl_type);		
					}
                }
                else
                {
                    hl_type = HL_MNG;
                }
            }
            break;
            case HCI_EVENT:
            {
                // Find an event descriptor associated to the event code
                const struct hci_evt_desc_tag* evt_desc = hci_look_for_evt_desc(msg->src_id);

                // Check if the event is supported
                if(evt_desc != NULL)
                {
                    hl_type = (evt_desc->dest_field & HCI_EVT_DEST_HL_MASK) >> HCI_EVT_DEST_HL_POS;
                }
            }
            break;
            case HCI_LE_EVENT:
            {
                uint8_t subcode = *((uint8_t *)ke_msg2param(msg));

                // Find an LE event descriptor associated to the LE event subcode
                const struct hci_evt_desc_tag* evt_desc = hci_look_for_le_evt_desc(subcode);

                // Check if the event is supported
                if(evt_desc != NULL)
                {
                    hl_type = (evt_desc->dest_field & HCI_EVT_DEST_HL_MASK) >> HCI_EVT_DEST_HL_POS;
                }
hci_printf("hl_type2:%02X\n",hl_type);				
            }
            break;
            #if (HCI_BLE_CON_SUPPORT)
            case HCI_BLE_ACL_DATA_RX:
            {
                hl_type = HL_DATA;
            }
            break;
            #endif // (HCI_BLE_CON_SUPPORT)

            default:
            {
                // Nothing to do
            }
            break;
        }

        // Find the higher layers destination task
        switch(hl_type)
        {
            case HL_MNG:
            {
                // Build the destination task ID
                dest = TASK_GAPM;
            }
            break;

            #if (HCI_BLE_CON_SUPPORT)
            case HL_CTRL:
            {
                // Check if the link identifier in the dest_id field corresponds to an active BLE link
                if(msg->dest_id < BLE_CONNECTION_MAX)
                {
                    // Build the destination task ID
                    dest = KE_BUILD_ID(TASK_GAPC, msg->dest_id);
                }
                else
                {
                    ASSERT_INFO(0, msg->id, msg->dest_id);
                }
            }
            break;

            case HL_DATA:
            {
                // Check if the link identifier in the dest_id field corresponds to an active BLE link
                if(msg->dest_id < BLE_CONNECTION_MAX)
                {
                    // Build the destination task ID
                    dest = KE_BUILD_ID(TASK_L2CC, msg->dest_id);
                }
                else
                {
                    ASSERT_INFO(0, msg->id, msg->dest_id);
                }
            }
            break;
            #endif //(HCI_BLE_CON_SUPPORT)

            default:
            {
                ASSERT_INFO(0, hl_type, 0);
            }
            break;
        }
		
hci_printf("hl_dest:%02X\n",dest);

        // Check it the destination has been found
        if(dest != TASK_NONE)
        {
            // Send the command to the internal destination task associated to this command
            msg->dest_id = dest;

            #if (HCI_BLE_CON_SUPPORT)
            if (msg->id != HCI_BLE_ACL_DATA_RX)
            #endif /*(HCI_BLE_CON_SUPPORT)*/
            {
                //TRC_REQ_HCI_EVT(msg->src_id, msg->param_len, msg->param);
            }
hci_printf("hl:msg->id:%04X\n",msg->id);	

hci_printf("hl:msg->src_id:%04X\n",msg->src_id);	

            ((ke_msg_send_handler)SVC_ke_msg_send)(param);
        }
        else
        {
            ASSERT_INFO(0, msg->id, msg->src_id);

            // Free message to avoid memory leak
            ((ke_msg_free_handler)SVC_ke_msg_free)(msg);
        }
    }
    #if(BLE_EMB_PRESENT && HCI_TL_SUPPORT)
    else
    #endif // (BLE_EMB_PRESENT && HCI_TL_SUPPORT)
    #endif //BLE_HOST_PRESENT
    #if (HCI_TL_SUPPORT)
    {
        // Send the HCI message over TL
        hci_tl_send(msg);
    }
    #endif //(HCI_TL_SUPPORT)
}
#endif //(BLE_EMB_PRESENT || BT_EMB_PRESENT)

#if BLE_HOST_PRESENT
void hci_send_2_controller(void *param)
{
    struct ke_msg *msg = ke_param2msg(param);

    #if (HCI_TL_SUPPORT && !BLE_EMB_PRESENT)
    // Send the HCI message over TL
    hci_tl_send(msg);
    #else //(HCI_TL_SUPPORT)
	
    #if BLE_EMB_PRESENT
    #if(BLE_EMB_PRESENT && HCI_TL_SUPPORT)
    // check if communication is performed over embedded host
    if(gapm_is_embedded_host())
    #endif // (BLE_EMB_PRESENT && HCI_TL_SUPPORT)
    {
        ke_task_id_t dest = TASK_NONE;
        uint8_t ll_type = LL_UNDEF;

        // The internal destination first depends on the message type (command, event, data)
        switch(msg->id)
        {
            case HCI_COMMAND:
            {
               // TRC_REQ_HCI_CMD(msg->src_id, msg->param_len, msg->param);

                // Find a command descriptor associated to the command opcode
                const struct hci_cmd_desc_tag* cmd_desc = hci_look_for_cmd_desc(msg->src_id);
				//hci_printf("msg->src_id:%08X,cmd_desc:%p\n",msg->src_id,cmd_desc);


                // Check if the command is supported
                if(cmd_desc != NULL)
                {
                    ll_type = (cmd_desc->dest_field & HCI_CMD_DEST_LL_MASK) >> HCI_CMD_DEST_LL_POS;
                    //hci_printf("ll_type:%02X\n",ll_type);					
                }
            }
            break;
            #if (HCI_BLE_CON_SUPPORT)
            case HCI_BLE_ACL_DATA_TX:
            {
                ll_type = BLE_CTRL;
            }
            break;
            #endif // (HCI_BLE_CON_SUPPORT)

            default:
            {
                // Nothing to do
            }
            break;
        }

        switch(ll_type)
        {
            case MNG:
            case BLE_MNG:
            {
                // Build the destination task ID
                dest = TASK_LLM;
            }
            break;

            #if (HCI_BLE_CON_SUPPORT)
            case CTRL:
            case BLE_CTRL:
            {
                // Check if the link identifier in the dest_id field corresponds to an active BLE link
                if(msg->dest_id < BLE_CONNECTION_MAX)
                {
                    // Build the destination task ID
                    dest = KE_BUILD_ID(TASK_LLC, msg->dest_id);
                }
                else
                {
                    ASSERT_INFO(0, msg->id, msg->dest_id);
                }
            }
            break;
            #endif //(HCI_BLE_CON_SUPPORT)

            default:
            {
                // Nothing to do
            }
            break;
        }
		
hci_printf("dest:%02X\n",dest);

        // Check it the destination has been found
        if(dest != TASK_NONE)
        {
            // Send the command to the internal destination task associated to this command
            msg->dest_id = dest;

hci_printf("ll:msg->src_id:%04X\n",msg->src_id);	

            ((ke_msg_send_handler)SVC_ke_msg_send)(param);
        }
        else
        {
            ASSERT_INFO(0, msg->id, msg->src_id);

            // Free message to avoid memory leak
            ((ke_msg_free_handler)SVC_ke_msg_free)(msg);
        }
    }
    #if(BLE_EMB_PRESENT && HCI_TL_SUPPORT)
    else
    {
        // receiving a message from internal host is not expected at all
        ASSERT_ERR(0);
        // Free message to avoid memory leak
        ((ke_msg_free_handler)SVC_ke_msg_free)(msg);
    }
    #endif // (BLE_EMB_PRESENT && HCI_TL_SUPPORT)
    #endif //BLE_EMB_PRESENT
    #endif //(HCI_TL_SUPPORT)
}

void hci_basic_cmd_send_2_controller(uint16_t opcode)
{
    void *no_param = ((ke_msg_alloc_handler)SVC_ke_msg_alloc)(HCI_COMMAND, 0, opcode, 0);
    hci_send_2_controller(no_param);
}

#endif //BLE_HOST_PRESENT

#if (BLE_EMB_PRESENT && HCI_BLE_CON_SUPPORT)
void hci_ble_conhdl_register(uint8_t link_id)
{
    ASSERT_INFO(link_id < BLE_CONNECTION_MAX, link_id, BLE_CONNECTION_MAX);
    ASSERT_ERR(!hci_env.ble_con_state[link_id]);

    // Link ID associated with BD address AND connection handle
    hci_env.ble_con_state[link_id] = true;
}

void hci_ble_conhdl_unregister(uint8_t link_id)
{
    ASSERT_INFO(link_id < BLE_CONNECTION_MAX, link_id, BLE_CONNECTION_MAX);
    ASSERT_ERR(hci_env.ble_con_state[link_id]);

    // Link ID associated with BD address
    hci_env.ble_con_state[link_id] = false;
}
#endif //(BLE_EMB_PRESENT && !BLE_HOST_PRESENT && HCI_BLE_CON_SUPPORT)



uint8_t hci_evt_mask_set(struct evt_mask const *evt_msk, uint8_t page)
{
    uint8_t i;

    switch(page)
    {
        case HCI_PAGE_0:
        case HCI_PAGE_1:
        {
            ASSERT_INFO(page == HCI_PAGE_DFT,page,page);
        }break;
        case HCI_PAGE_2:
        {
            // Store event mask
            memcpy(&hci_env.evt_msk_page_2.mask[0], &evt_msk->mask[0], EVT_MASK_LEN);
        }break;
        case HCI_PAGE_DFT:
        {
            // Store event mask
            memcpy(&hci_env.evt_msk.mask[0], &evt_msk->mask[0], EVT_MASK_LEN);

            // ensure that reserved bit are set
            for (i = 0; i < EVT_MASK_LEN; i++)
            {
                hci_env.evt_msk.mask[i] |= hci_rsvd_evt_msk.mask[i];
            }
        }break;
        #if (BLE_EMB_PRESENT)
        case HCI_PAGE_LE:
        {
            // Store event mask
            memcpy(&hci_env.le_evt_msk.mask[0], &evt_msk->mask[0], EVT_MASK_LEN);
        }break;
        #endif // (BLE_EMB_PRESENT)
        default:
        {
        }break;
    }
    return CO_ERROR_NO_ERROR;
}



#endif //(HCI_PRESENT)

/// @} HCI
