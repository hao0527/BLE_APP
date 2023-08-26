
/**
 ****************************************************************************************
 * @addtogroup HCI_EIF
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "panip_config.h"      // SW configuration

#if (BLE_HCIC_ITF)
 
#include "co_buf.h"           // stack buffering
#include "co_endian.h"        // endian-ness definition
#include "co_utils.h"         // stack common utility definitions
#include "co_error.h"         // error definition
#include "ke_event.h"         // kernel event

#include "panip.h"             // stack definitions

#include "hcic.h"              // hci definition
#include "hcic_msg.h"          // hci messages
#include "hcic_eif.h"         // hci External Interface definition




/*
 * ENUMERATIONS
 ****************************************************************************************
 */


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
///HCI table for correspondence between External Interface message type and header length.
static const uint8_t hci_msgtype2hdrlen[]=
{
    [HCI_CMD_MSG_TYPE] = HCI_CMD_HDR_LEN,
    [HCI_ACL_MSG_TYPE] = HCI_ACL_HDR_LEN,
    [HCI_EVT_MSG_TYPE] = HCI_EVT_HDR_LEN
};


/*
 * LOCAL FUNCTION DECLARATION
 ****************************************************************************************
 */

static void hci_eif_rx_cmd_hdr_extract(void);
static void hci_eif_rx_acl_hdr_extract(void);
static void hci_eif_read_start(void);
static void hci_eif_read_hdr(uint8_t len);
static void hci_eif_read_payl(uint8_t len);
static void hci_eif_read_next_out_of_sync(void);
static void hci_eif_out_of_sync(void);
static void hci_eif_tx_done(uint8_t status);
static void hci_eif_rx_done(uint8_t status);


/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
****************************************************************************************
* @brief Local function : extracts command header components and places them in the
* HCI environment command header structure.
*****************************************************************************************
*/
static void hci_eif_rx_cmd_hdr_extract(void)
{
    uint16_t opcode = co_btohs(co_read16p(&hci_env.curr_hdr_buff[0]));

    //check opcode existence
    hci_env.chdr.known_opcode = 0;

    //check if opcode exists
    for(int i=0; i<HCI_CMD_OPCODE_NB_MAX;i++)
    {
        if (hci_cmd_opcodes[i] == opcode)
        {
            hci_env.chdr.known_opcode = 1;
            break;
        }
    }

    //extract command header:ocf, ogf, parameter length
    hci_env.chdr.ogf    = HCI_OP2OGF(opcode);
    hci_env.chdr.ocf    = HCI_OP2OCF(opcode);
    hci_env.chdr.parlen = hci_env.curr_hdr_buff[2];
}

/**
****************************************************************************************
* @brief Local function : extracts ACL header components and places them in the
* HCI environment ACL header structure.
*****************************************************************************************
*/
static void hci_eif_rx_acl_hdr_extract(void)
{
    //extract acl flags
    hci_env.ahdr.bcpb_flag = hci_env.curr_hdr_buff[1]>>4;

    //extract connection handle
    hci_env.ahdr.hdl = co_btohs(co_read16p(hci_env.curr_hdr_buff));

    //mask the flag bits
    hci_env.ahdr.hdl = hci_env.ahdr.hdl & 0x0FFF;

    //extract data length MSB to LSB
    hci_env.ahdr.datalen = co_btohs(co_read16p(hci_env.curr_hdr_buff+2));
}

/**
******************************************************************************************
* @brief Local function : places HCIH EIF in RX_START state and sets the External Interface environment.
******************************************************************************************
*/
static void hci_eif_read_start(void)
{
    //Initialize External Interface in reception mode state
    hci_env.rx_state = HCI_STATE_RX_START;

    //Set the External Interface environment to message type 1 byte reception
    hci_env.ext_if->read(&hci_env.curr_msg_type, 1, &hci_eif_rx_done);

    #if DEEP_SLEEP
    // No HCI reception is ongoing, so allow going to sleep
    rwip_prevent_sleep_clear(RW_HCI_RX_ONGOING);
    #endif
}


/**
****************************************************************************************
* @brief Local function : places HCIH EIF in RX header state and sets the External Interface env.
*
* @param[in] len Length of header to be received in the currently set buffer.
*****************************************************************************************
*/
static void hci_eif_read_hdr(uint8_t len)
{
    //change Rx state - wait for header next
    hci_env.rx_state = HCI_STATE_RX_HDR;

    //set External Interface environment to header reception of len bytes
    hci_env.ext_if->read(hci_env.curr_hdr_buff, len, &hci_eif_rx_done);
    
    #if DEEP_SLEEP
    // An HCI reception is ongoing
    rwip_prevent_sleep_set(RW_HCI_RX_ONGOING);
    #endif //DEEP_SLEEP
}


/**
******************************************************************************************
* @brief Local function : places HCIH EIF in RX payload state and sets the External Interface env.
*
* @param[in] len Length of payload to be received in the currently set buffer.
******************************************************************************************
*/
static void hci_eif_read_payl(uint8_t len)
{
    //change rx state to payload reception
    hci_env.rx_state = HCI_STATE_RX_PAYL;

    //set External Interface environment to payload reception of len bytes
    hci_env.ext_if->read(hci_env.curr_payl_buff, len, &hci_eif_rx_done);
}

/**
******************************************************************************************
* @brief Local function : places HCIH EIF in RX_START_OUT_OF_SYNC state.
******************************************************************************************
*/
static void hci_eif_read_next_out_of_sync(void)
{
    //Set External Interface reception state to HCI_STATE_RX_START_OUT_OF_SYNC
    hci_env.rx_state = HCI_STATE_RX_OUT_OF_SYNC;

    //Set the External Interface environment to 1 byte reception
    hci_env.ext_if->read(&hci_env.out_of_sync.byte, 1, &hci_eif_rx_done);
}

/**
 *****************************************************************************************
 *@brief Static function handling External Interface out of synchronization detection.
 *
 * At External Interface reception, when packet indicator opcode of a command is not
 * recognized.
 *
 *****************************************************************************************
 */
static void hci_eif_out_of_sync(void)
{
    struct llm_hw_error_evt *evt;

    // Allocate a message structure for hardware error event
    evt = (struct llm_hw_error_evt *)KE_MSG_ALLOC(LLM_HW_ERROR_EVT, HOST_GAP_TASK,
                                                        TASK_LLM, llm_hw_error_evt);

    // Fill the parameter structure
    evt->hw_code = CO_ERROR_HW_UART_OUT_OF_SYNC;

    // Send the message
    ke_msg_send(evt);

    // Initialize index
    hci_env.out_of_sync.index = 0;

    // Start reception of new packet ID
    hci_eif_read_next_out_of_sync();

    #if DEEP_SLEEP
    // No HCI reception is ongoing, so allow going to sleep
    rwip_prevent_sleep_clear(RW_HCI_RX_ONGOING);
    #endif // DEEP_SLEEP
}

/**
 ****************************************************************************************
 * @brief Actions after External Interface TX.
 *
 * Analyzes the status value and sets the hci environment state to TX_DONE/ERR
 * accordingly. This allows the higher function calling write to have feedback
 * and decide the following action (repeat/abort tx in case of error, continue otherwise).
 *
 * @param[in]  status External Interface Tx status: ok or error.
 *****************************************************************************************
 */
static void hci_eif_tx_done(uint8_t status)
{
    // Sanity check: Transmission should always work
    ASSERT_ERR(status == RWIP_EIF_STATUS_OK);

    // Defer the freeing of resources to ensure that it is done in background
    ke_event_set(KE_EVENT_HCI_TX_DONE);

    #if (DEEP_SLEEP)
    // The HCI transmission is finished, so allow going back to sleep
    rwip_prevent_sleep_clear(RW_HCI_TX_ONGOING);
    #endif // DEEP_SLEEP
}

/**
 ****************************************************************************************
 * @brief Function called at each RX interrupt.
 *
 * According to HCI RX state, the received data is treated differently: message type,
 * header or payload. Once payload is obtained (if existing) the appropriate hci unpacking
 * function is called thus generating a ke_msg for the appropriate task.
 *
 * @param[in]  status External Interface RX status: ok or error
 *****************************************************************************************
 */
static void hci_eif_rx_done(uint8_t status)
{
    //detect External Interface RX error and handle accordingly
    if (status == RWIP_EIF_STATUS_ERROR)
    {
        // External Interface RX error -> enter in out of sync
        hci_eif_out_of_sync();
    }
    else
    {
        //check HCI state to see what was received
        switch(hci_env.rx_state)
        {
            /* RECEIVE MESSAGE TYPE STATE*/
            case HCI_STATE_RX_START:
            {
                // Check received packet indicator
                if((hci_env.curr_msg_type == HCI_CMD_MSG_TYPE) ||
                        (hci_env.curr_msg_type == HCI_ACL_MSG_TYPE))
                {
                    //change state to header reception
                    hci_eif_read_hdr(hci_msgtype2hdrlen[hci_env.curr_msg_type]);
                }
                else
                {
                    // Incorrect packet indicator -> enter in out of sync
                    hci_eif_out_of_sync();
                }
            }
            break;
            /* RECEIVE MESSAGE TYPE STATE END*/

            /* RECEIVE HEADER STATE*/
            case HCI_STATE_RX_HDR:
            {
                switch (hci_env.curr_msg_type)
                {
                    //Command Header reception
                    case HCI_CMD_MSG_TYPE:
                    {
                        //Extract the command header components
                        hci_eif_rx_cmd_hdr_extract();

                        // Check received parameter size
                        if(hci_cmd_parameter_size_check())
                        {
                            //Check command payload size from HCI header
                            if(hci_env.chdr.parlen == 0)
                            {
                                //Send basic kernel message
                                hci_cmd_dispatch_basic();

                                //change hci rx state to message type reception
                                hci_eif_read_start();
                            }

                            //Command has parameters so go to payload reception
                            else
                            {
                                //Allocate enough space for expected struct
                                hci_env.curr_payl_buff = hci_cmd_alloc();

                                //change HCI rx state to payload reception
                                if (hci_env.curr_payl_buff != NULL)
                                {
                                    hci_eif_read_payl(hci_env.chdr.parlen);
                                }
                            }
                        }
                        else
                        {
                            // Incorrect payload size -> enter in out of sync
                            hci_eif_out_of_sync();
                        }
                    }
                    break;

                    case HCI_ACL_MSG_TYPE:
                    {
                        //Extract header fields into hci_env.ahdr
                        hci_eif_rx_acl_hdr_extract();

                        //allocate a descriptor for reception data (HL rx or LL tx)
                        hci_data_alloc();

                        //Check data length
                        if (hci_env.ahdr.datalen > BLE_DATA_LEN)
                        {
                            // Incorrect payload size -> enter in out of sync
                            hci_eif_out_of_sync();
                        }
                        else
                        {
                            //change HCI rx state to payload reception
                            hci_eif_read_payl(hci_env.ahdr.datalen);
                        }
                    }
                    break;
                    default:break;
                }//end switch msg type in header reception
            }
            break;
            /* RECEIVE HEADER STATE END*/

            /* RECEIVE PAYLOAD STATE */
            case HCI_STATE_RX_PAYL:
            {
                switch (hci_env.curr_msg_type)
                {
                    case HCI_CMD_MSG_TYPE:
                    {
                        //call the right unpack handler
                        hci_cmd_dispatch(hci_env.curr_payl_buff );

                        //change hci rx state to message type reception - common to all types
                        hci_eif_read_start();
                    }
                    break;

                    case HCI_ACL_MSG_TYPE:
                    {
                        // if we have no overflow dispatch the buffer
                        if(hci_env.curr_payl_buff  != hci_env.unk_cmd_buf)
                        {
                            //send it to appropriate task
                            hci_data_dispatch(hci_env.curr_payl_buff);
                        }

                        //change hci rx state to message type reception - common to all types
                        hci_eif_read_start();
                    }
                    break;
                    default:break;
                }
            }
            break;
            /* RECEIVE PAYLOAD STATE END*/

            /* RX OUT OF SYNC STATE */
            case HCI_STATE_RX_OUT_OF_SYNC:
            {
                // Check received byte
                hci_out_of_sync_check();

                // Check if hci_reset is fully received
                if(hci_env.out_of_sync.index == 4)
                {
                    //Set command header components
                    hci_env.chdr.known_opcode = 1;
                    hci_env.chdr.ocf    = HCI_OP2OCF(HCI_RESET_CMD_OPCODE);
                    hci_env.chdr.ogf    = HCI_OP2OGF(HCI_RESET_CMD_OPCODE);
                    //                hci_env.chdr.parlen = 0;

                    //Send basic kernel message
                    hci_cmd_dispatch_basic();

                    //change hci rx state to message type reception
                    hci_eif_read_start();
                }
                else
                {
                    // Start a new byte reception
                    hci_eif_read_next_out_of_sync();
                }
            }
            break;
            /* RX OUT OF SYNC STATE END*/

            /* DEFAULT STATE */
            default:
            {
                ASSERT_ERR(0);
            }
            break;
            /* DEFAULT END*/

        }
        /* STATE SWITCH END */
    }
}



/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void hci_eif_init(void)
{
    // Enable External Interface
    //hci_env.ext_if->flow_on();
	
    //initialize tx state
    hci_env.tx_state = HCI_STATE_TX_IDLE;

    //start External Interface reception
    hci_eif_read_start();
}


void hci_eif_write(struct ke_msg *msg)
{
    uint8_t *transport_buf;

    // Check for particular case of data packets
    #if (BLE_CENTRAL || BLE_PERIPHERAL)
    if (msg->hci_type == HCI_ACL_MSG_TYPE)
    {
        struct llc_data_packet_ind *data_ind = (struct llc_data_packet_ind *)(msg->param);
        struct co_buf_rx_desc *desc = co_buf_rx_get(data_ind->rx_hdl);
        transport_buf = ((uint8_t *)&desc->data[0]) + msg->hci_off;
    }
    else
    #endif // #if (BLE_CENTRAL || BLE_PERIPHERAL)
    {
        //get the pointer to the complete buffer to transport - offset in front for transport
        transport_buf = ((uint8_t *)msg->param) + msg->hci_off;
    }

    #if (DEEP_SLEEP)
    // An HCI transmission is ongoing - The bit has to be set prior to call to write
    // as this function may call hci_eif_tx_done immediately
    rwip_prevent_sleep_set(RW_HCI_TX_ONGOING);
    #endif // DEEP_SLEEP

    //pack event type message (External Interface header)
    transport_buf -= HCI_TRANSPORT_HDR_LEN;
    *transport_buf = msg->hci_type;

    //go to start tx state
    hci_env.tx_state = HCI_STATE_TX_ONGOING;
    hci_env.ext_if->write(transport_buf, msg->hci_len + HCI_TRANSPORT_HDR_LEN, &hci_eif_tx_done);
}

#if (DEEP_SLEEP)
void hci_eif_start(void)
{
    // Enable External Interface flow
    hci_env.ext_if->flow_on();
}

bool hci_eif_stop(void)
{
    return (hci_env.ext_if->flow_off());
}
#endif //DEEP_SLEEP

#endif //BLE_HCIC_ITF

/// @} HCI_EIF
