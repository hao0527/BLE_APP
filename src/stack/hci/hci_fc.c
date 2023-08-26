
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


/*
 * DEFINES
 ****************************************************************************************
 */

#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)

/*
 * ENUMERATIONS DEFINITIONS
 ****************************************************************************************
 */

/*
 * STRUCTURES DEFINITIONS
 ****************************************************************************************
 */
/// Flow control structure
struct host_set_fc
{
    /// flow control enabled
    bool acl_flow_cntl_en;
    /// host packet size max
    uint16_t acl_pkt_len;
    /// host packet number max
    uint16_t acl_pkt_nb;
    /// current packet available
    uint16_t curr_pkt_nb;


};

struct counter_fc
{
    /// counter for number of ACL packets sent to Host
    uint16_t acl_pkt_sent;

};

struct hci_fc_tag 
{
    /// Flow Control
    struct host_set_fc host_set;
    struct counter_fc cntr;
};


/*
 * CONSTANTS DEFINITIONS
 ****************************************************************************************
 */


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

///HCI FC environment context
struct hci_fc_tag hci_fc_env;


/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */



/*
 * MODULES INTERNAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */


/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void hci_fc_init(void)
{
    memset(&hci_fc_env, 0, sizeof(hci_fc_env));
}

uint8_t hci_fc_acl_buf_size_set(uint16_t acl_pkt_len, uint16_t nb_acl_pkts)
{
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;

    // ACL packet size, Number of ACL packet
    if ((acl_pkt_len != 0) && (nb_acl_pkts != 0))
    {
    	if (acl_pkt_len >= DH5_3_PACKET_SIZE)
    	{
			status = CO_ERROR_NO_ERROR;

			hci_fc_env.host_set.acl_pkt_len = acl_pkt_len;
			hci_fc_env.host_set.acl_pkt_nb  = nb_acl_pkts;
    	}
    	else
    	{
    		status = CO_ERROR_UNSUPPORTED;
    	}
    }

    return(status);
}


 
uint8_t hci_fc_acl_en(bool flow_enable)
{
    uint8_t status = CO_ERROR_NO_ERROR;

    #if (BLE_EMB_PRESENT && (HCI_BLE_CON_SUPPORT))
    // Check that there is no BLE link
    for(uint8_t conhdl = 0 ; conhdl < BLE_CONNECTION_MAX ; conhdl++)
    {
//        if(((ke_state_get_handler)SVC_ke_state_get)(KE_BUILD_ID(TASK_LLC, conhdl)) != LLC_FREE)
//        {
//            status = CO_ERROR_COMMAND_DISALLOWED;
//            break;
//        }
    }
    #endif //BLE_EMB_PRESENT



    if(status == CO_ERROR_NO_ERROR)
    {
        hci_fc_env.host_set.acl_flow_cntl_en = flow_enable;
    }

    return status;
}



void hci_fc_acl_packet_sent(void)
{
    if (hci_fc_env.host_set.acl_flow_cntl_en == true)
    {
        hci_fc_env.cntr.acl_pkt_sent++;
    }
}



void hci_fc_host_nb_acl_pkts_complete(uint16_t acl_pkt_nb)
{
    if (hci_fc_env.cntr.acl_pkt_sent > acl_pkt_nb)
    {
        hci_fc_env.cntr.acl_pkt_sent -= acl_pkt_nb;
    }
    else
    {
        hci_fc_env.cntr.acl_pkt_sent = 0;
    }
}



uint16_t hci_fc_check_host_available_nb_acl_packets(void)
{
    uint16_t cnt = 0;
    // if flow control is not enabled we can send number of packets
    if (hci_fc_env.host_set.acl_flow_cntl_en != true)
    {
        cnt = 0xFFFF;// maximum packets
    }
    else
    if (hci_fc_env.host_set.acl_pkt_nb > hci_fc_env.cntr.acl_pkt_sent)
    {
        cnt = hci_fc_env.host_set.acl_pkt_nb - hci_fc_env.cntr.acl_pkt_sent;
    }
    return cnt;
}



#endif //(BLE_EMB_PRESENT || BT_EMB_PRESENT)

#endif //(HCI_PRESENT)

/// @} HCI
