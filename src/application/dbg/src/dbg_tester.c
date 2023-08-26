
/**
 ****************************************************************************************
 * @addtogroup DBGTESTER
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"             // stack configuration

#if (RW_TESTER)

#include "co_error.h"                // common error definition
#include "co_bt.h"                   // common bt definition
#include "arch.h"                    // arch definition

#include "dbg_tester.h"              // tester definition
#include "dbg_task.h"                // debug task definition
#include "ke_task.h"                 // kernel task definition
#include "ld_util.h"                 // ld utility definition
#include "lc_lmppdu.h"               // lmp pdu definition

// For HV1 packet
#include "ld.h"                      // link driver definition
#include "reg_btcore.h"              // bt core registers
#include "reg_bt_em_cs.h"            // control structure definition
#include "reg_bt_em_et.h"            // exchange table definition
#include "reg_bt_em_rxsco.h"         // rx sco definition
#include "reg_bt_em_txsco.h"         // tx sco definition

/*
 * ENUMERATION DEFINITIONS
 *****************************************************************************************
 */
/// DBG Tester status
enum DBG_TESTER_STAT
{
    /// tester disabled
    DBG_DISABLE = 0,
    /// tester enabled
    DBG_ENABLE
};

/*
 * STRUCT DEFINITION
 ****************************************************************************************
 */

///Tester emulator environment parameters definition
struct dbg_tester_env_tag
{
    /// link identifier
    uint8_t    link_id;
    /// test scenario
    uint8_t    test_scenario;
    /// hopping mode
    uint8_t    hopping_mode;
    /// transmit frequency
    uint8_t    tx_freq;
    /// receive frequency
    uint8_t    rx_freq;
    /// power control
    uint8_t    power_control;
    /// polling period
    uint8_t    poll_period;
    /// packet type
    uint8_t    packet_type;
    /// pay load size
    uint16_t   payload_length;
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
///Tester emulator environment
static struct dbg_tester_env_tag dbg_tester_env;

///Tester emulator variable used to block SW in infinite loop
static volatile uint8_t dbg_tester_block = 1;

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
******************************************************************************************
* @brief Stop tester emulation.
*
* @param[in] status       Error status
*
******************************************************************************************
*/
static void dbg_tester_stop(uint8_t status)
{
    struct dbg_emul_finished_evt *evt = (struct dbg_emul_finished_evt *)
     KE_MSG_ALLOC(DBG_EMUL_FINISHED_EVT, TASK_HCI, TASK_DBG, dbg_emul_finished_evt);

    // Fill in the event structure
    evt->status = status;

    // Send event
    ke_msg_send(evt);

    // Clear flag in LD
    ld_set_tester_mode(dbg_tester_env.link_id, DBG_DISABLE);

    // Change state to idle
    ke_state_set(TASK_DBG, DBG_IDLE);
}

/**
******************************************************************************************
* @brief Send data to DUT
*
* This function implements the test scenario.

* @return Error status
******************************************************************************************
*/
static uint8_t dbg_tester_send_data(void)
{
    uint32_t clock;
    uint8_t status = CO_ERROR_NO_ERROR;
    struct cntl_struct *cs_ptr;
    struct cntl_struct *cs_ptr_data;
    uint8_t * ptr_acl;

    // Get control structure of ACL link
    cs_ptr = (struct cntl_struct *)(uint32_t)(REG_BT_EM_CS_BASE_ADDR + REG_BT_EM_CS_ADDR_GET(dbg_tester_env.link_id));
    cs_ptr_data = cs_ptr+1;

    memcpy((uint8_t*)cs_ptr_data,(uint8_t*)cs_ptr, sizeof(struct cntl_struct));

    // Check test scenario
    switch(dbg_tester_env.test_scenario)
    {
        case SCOLOOP_MODE:
        {
            if((dbg_tester_env.packet_type & 0xF) == HV1_TYPE)
            {
                /*************************************************************************
                * Send HV1 packet
                *************************************************************************/

                // Init the LT address with the ACL LT address in the eSCO control register
                bt_e_scoltcntl_set(0, BT_SYNTYPE_BIT |  (cs_ptr->LTADDR & BT_ACLLTADDR_MASK));

                // Set the eSCO interval in the ESCOPORTCNTL register
                bt_e_scoportcntl_set(0,2 << BT_TE_SCT_LSB);

                /*
                 * Set the eSCO TX/RX types and length in the ESCOTRCNTL register
                 * Set also the TXSEQn to 1 so that the first eSCO packet is sent with SEQn=0
                 */
                bt_e_scotrcntl_set(0, TXSEQN |
                        (HV1_TYPE << BT_TXTYPE_LSB) |
                        (HV1_TYPE << BT_RXTYPE_LSB) |
                        (HV1_PACKET_SIZE << BT_TXLEN_LSB)|
                        (HV1_PACKET_SIZE << BT_RXLEN_LSB)) ;

                // Fill data in buffers
                memset((uint8_t*)LD_TXSCO_BUF(0), 0x55, (SYNC_DATA_PACKET_SIZE*2));
                memset((uint8_t*)LD_RXSCO_BUF(0), 0x55, (SYNC_DATA_PACKET_SIZE*2));


                // Init the eSCO port TX/RX pointers
                bt_e_scoptrrx_set(0,(PTR_TO_EM_ADDR(LD_RXSCO_BUF(0))|
                        ((PTR_TO_EM_ADDR(LD_RXSCO_BUF(0))+SYNC_DATA_PACKET_SIZE) << 16)));
                bt_e_scoptrtx_set(0,(
                        PTR_TO_EM_ADDR(LD_TXSCO_BUF(0))|
                        ((PTR_TO_EM_ADDR(LD_TXSCO_BUF(0))+SYNC_DATA_PACKET_SIZE) << 16)));

                // Set reception buffer size
                bt_e_scomaxsz_set(HV1_PACKET_SIZE);

                // Get clock value
                clock = ld_util_read_clock();

                // Round clock value to upper even
                if(clock & 0x1)
                {
                    clock = clock + 1;
                }

                // Add delay
                clock += LD_DELAY;

                // Rise diag_port signal to trigger analyzer

                // Program HV1 packet
                bt_extab_set(clock & 0x0F, BT_E_SCO_BIT|EXC_VXCHAN0|BT_EXCSCORSVD_BIT| PTR_TO_EM_ADDR(cs_ptr));

                // Set poll type in control structure
                cs_ptr->LINKCNTL[0] = POLL_TYPE;
                cs_ptr->LINKCNTL[1] = POLL_TYPE;

                // Program POLL packet
                bt_extab_set((clock+2) & 0x0F, EXC_NO_VOICE| PTR_TO_EM_ADDR(cs_ptr));
            }

            else if((dbg_tester_env.packet_type & 0xF) == DV_TYPE)
            {
                /*************************************************************************
                * Send DV packet
                *************************************************************************/

                /*****************************
                 *  ACL part
                 *****************************/

                // Allocate TX buffer
                ptr_acl = co_alloc_acl_buf_tx();

                // Fill data in buffers
                memset(ptr_acl, 0x55, ACL_DATA_PACKET_SIZE);

                // Point to ACL data buffer
                cs_ptr_data->TXPTR[0] = PTR_TO_EM_ADDR(ptr_acl);
                cs_ptr_data->TXPTR[1] = PTR_TO_EM_ADDR(ptr_acl);

                // Set AUX1 type in control structure
                cs_ptr_data->LINKCNTL[0] = DV_TYPE;
                cs_ptr_data->LINKCNTL[1] = DV_TYPE;
                cs_ptr_data->TXPHDR[0] = (DV_ACL_PACKET_SIZE << 3);
                cs_ptr_data->TXPHDR[1] = (DV_ACL_PACKET_SIZE << 3);


                // Set RX buffer pointers
                WriteRXPTR( cs_ptr_data, 0,
                            PTR_TO_EM_ADDR(co_alloc_acl_buf_rx()));
                WriteRXPTR( cs_ptr_data, 1,
                            PTR_TO_EM_ADDR(co_alloc_acl_buf_rx()));


                /*****************************
                 *  SCO part
                 *****************************/

                // Init the LT address with the ACL LT address in the eSCO control register
                bt_e_scoltcntl_set(0, BT_SYNTYPE_BIT |  (cs_ptr->LTADDR & BT_ACLLTADDR_MASK));

                // Set the eSCO interval in the ESCOPORTCNTL register
                bt_e_scoportcntl_set(0,2 << BT_TE_SCT_LSB);

                /*
                 * Set the eSCO TX/RX types and length in the ESCOTRCNTL register
                 * Set also the TXSEQn to 1 so that the first eSCO packet is sent with SEQn=0
                 */
                bt_e_scotrcntl_set(0, TXSEQN |
                        (HV1_TYPE << BT_TXTYPE_LSB) |
                        (HV1_TYPE << BT_RXTYPE_LSB) |
                        (HV1_PACKET_SIZE << BT_TXLEN_LSB)|
                        (HV1_PACKET_SIZE << BT_RXLEN_LSB)) ;

                // Fill data in buffers
                memset((uint8_t*)LD_TXSCO_BUF(0), 0x55, (SYNC_DATA_PACKET_SIZE*2));
                memset((uint8_t*)LD_RXSCO_BUF(0), 0x55, (SYNC_DATA_PACKET_SIZE*2));


                // Init the eSCO port TX/RX pointers
                bt_e_scoptrrx_set(0,(PTR_TO_EM_ADDR(LD_RXSCO_BUF(0))|
                        ((PTR_TO_EM_ADDR(LD_RXSCO_BUF(0))+SYNC_DATA_PACKET_SIZE) << 16)));
                bt_e_scoptrtx_set(0,(
                        PTR_TO_EM_ADDR(LD_TXSCO_BUF(0))|
                        ((PTR_TO_EM_ADDR(LD_TXSCO_BUF(0))+SYNC_DATA_PACKET_SIZE) << 16)));

                // Set reception buffer size
                bt_e_scomaxsz_set(HV1_PACKET_SIZE);


                /*****************************
                 *  Schedule data packets
                 *****************************/

                // Get clock value
                clock = ld_util_read_clock();

                // Round clock value to upper even
                if(clock & 0x1)
                {
                    clock = clock + 1;
                }

                // Add delay
                clock += LD_DELAY;

                // Rise diag_port signal to trigger analyzer


                // Program DV packet
                bt_extab_set((clock) & 0x0F, BT_E_SCO_BIT | EXC_VXCHAN0 | PTR_TO_EM_ADDR(cs_ptr_data));

                // Program POLL packet
                bt_extab_set((clock+2) & 0x0F, EXC_NO_VOICE| PTR_TO_EM_ADDR(cs_ptr));

                // Program DV packet
                bt_extab_set((clock+4) & 0x0F, BT_E_SCO_BIT | EXC_VXCHAN0 | PTR_TO_EM_ADDR(cs_ptr_data));

                // Program POLL packet
                bt_extab_set((clock+6) & 0x0F, EXC_NO_VOICE| PTR_TO_EM_ADDR(cs_ptr));

            }
        }
        break;

        case ACLLOOP_MODE:
        {
            /*************************************************************************
             * Send AUX1 packet
             *************************************************************************/


            // Allocate TX buffer
            ptr_acl = co_alloc_acl_buf_tx();

            // Fill data in buffers
            memset(ptr_acl, 0x55, ACL_DATA_PACKET_SIZE);

            // Point to ACL data buffer
            cs_ptr_data->TXPTR[0] = PTR_TO_EM_ADDR(ptr_acl);
            cs_ptr_data->TXPTR[1] = PTR_TO_EM_ADDR(ptr_acl);

            // Set AUX1 type in control structure
            cs_ptr_data->LINKCNTL[0] = AUX1_TYPE;
            cs_ptr_data->LINKCNTL[1] = AUX1_TYPE;
            cs_ptr_data->TXPHDR[0] = (AUX1_PACKET_SIZE << 3);
            cs_ptr_data->TXPHDR[1] = (AUX1_PACKET_SIZE << 3);


            // Set RX buffer pointers
            WriteRXPTR( cs_ptr_data, 0,
                        PTR_TO_EM_ADDR(co_alloc_acl_buf_rx()));
            WriteRXPTR( cs_ptr_data, 1,
                        PTR_TO_EM_ADDR(co_alloc_acl_buf_rx()));

            // Get clock value
            clock = ld_util_read_clock();

            // Round clock value to upper even
            if(clock & 0x1)
            {
                clock = clock + 1;
            }

            // Add delay
            clock += LD_DELAY;

            // Rise diag_port signal to trigger analyzer

            // Program POLL packet
            bt_extab_set((clock) & 0x0F, EXC_NO_VOICE| PTR_TO_EM_ADDR(cs_ptr));

            // Program POLL packet
            bt_extab_set((clock+2) & 0x0F, EXC_NO_VOICE| PTR_TO_EM_ADDR(cs_ptr));

            // Program AUX1 packet
            bt_extab_set((clock+4) & 0x0F, EXC_NO_VOICE| PTR_TO_EM_ADDR(cs_ptr_data));

            // Program AUX1 packet
            bt_extab_set((clock+6) & 0x0F, EXC_NO_VOICE| PTR_TO_EM_ADDR(cs_ptr_data));
        }
        break;

        default:
        {
            // Test scenario not supported
            status = CO_ERROR_UNSUPPORTED;
        }
        break;
    }


    // Block SW execution to stop scheduling
    GLOBAL_INT_DISABLE();
    while(dbg_tester_block)
    GLOBAL_INT_RESTORE();

    return status;
}

/**
******************************************************************************************
* @brief Handle reception of LMP responses during tester emulation.
*
* @param[in] status       Error status
*
******************************************************************************************
*/
static void dbg_tester_lmp_acc_received(uint8_t status)
{
    uint8_t lmp_buffer[LMP_TEST_CNTL_LEN];

    // Fill lmp_test_control PDU
    lmp_buffer[0] = (LMP_TEST_CNTL_OPCODE<<1)|MASTER_ROLE;
    lmp_buffer[1] = (dbg_tester_env.test_scenario ^ 0x55);
    lmp_buffer[2] = (dbg_tester_env.hopping_mode ^ 0x55);
    lmp_buffer[3] = (dbg_tester_env.tx_freq ^ 0x55);
    lmp_buffer[4] = (dbg_tester_env.rx_freq ^ 0x55);
    lmp_buffer[5] = (dbg_tester_env.power_control ^ 0x55);
    lmp_buffer[6] = (dbg_tester_env.poll_period ^ 0x55);
    lmp_buffer[7] = (dbg_tester_env.packet_type ^ 0x55);
    lmp_buffer[8] = ((dbg_tester_env.payload_length & 0xFF) ^ 0x55);
    lmp_buffer[9] = ((dbg_tester_env.payload_length >> 8 & 0xFF) ^ 0x55);


    // Check DBG task state
    switch(ke_state_get(TASK_DBG))
    {
        // Waiting for LMP_test_activate response
        case DBG_WAIT_TEST_ACT_ACC:
        {
            if(status == CO_ERROR_NO_ERROR)
            {
                // Send lmp_test_activate
                status = ld_util_lmp_tx(dbg_tester_env.link_id, &lmp_buffer[0], LMP_TEST_CNTL_LEN, false);
            }

            if(status == CO_ERROR_NO_ERROR)
            {
                // Change state to wait lmp_accepted(lmp_test_activate)
                ke_state_set(TASK_DBG, DBG_WAIT_TEST_CNTL_ACC);
            }
            else
            {
                // Stop tester emulation
                dbg_tester_stop(status);
            }
        }
        break;

        // Waiting for LMP_test_control response
        case DBG_WAIT_TEST_CNTL_ACC:
        {
            if(status == CO_ERROR_NO_ERROR)
            {
                status = dbg_tester_send_data();
            }

            // Stop tester emulation
            dbg_tester_stop(status);
        }
        break;

        // Other state
        default:
        {
            // Should not happen
            ASSERT_ERR(0);
        }
        break;
    }
}

/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

uint8_t dbg_tester_start(uint8_t link_id, uint8_t test_scenario, uint8_t hopping_mode,
                 uint8_t tx_freq, uint8_t rx_freq, uint8_t power_control,
                 uint8_t poll_period, uint8_t packet_type, uint16_t payload_length)
{
    uint8_t status = CO_ERROR_NO_ERROR;
    uint8_t lmp_buffer[LMP_TEST_ACTIV_LEN];

    // Fill lmp_test_activate PDU
    lmp_buffer[0] = (LMP_TEST_ACTIV_OPCODE<<1)|MASTER_ROLE;

    // Set flag in LD
    ld_set_tester_mode(link_id, DBG_ENABLE);

    // Store test scenario parameters
    dbg_tester_env.link_id         = link_id;
    dbg_tester_env.test_scenario   = test_scenario;
    dbg_tester_env.hopping_mode    = hopping_mode;
    dbg_tester_env.tx_freq         = tx_freq;
    dbg_tester_env.rx_freq         = rx_freq;
    dbg_tester_env.power_control   = power_control;
    dbg_tester_env.poll_period     = poll_period;
    dbg_tester_env.packet_type     = packet_type;
    dbg_tester_env.payload_length  = payload_length;

    // Send lmp_test_activate
    status = ld_util_lmp_tx(link_id, &lmp_buffer[0], LMP_TEST_ACTIV_LEN, false);

    if(status == CO_ERROR_NO_ERROR)
    {
        // Change state to wait lmp_accepted(lmp_test_activate)
        ke_state_set(TASK_DBG, DBG_WAIT_TEST_ACT_ACC);
    }
    else
    {
        // Clear flag in LD
        ld_set_tester_mode(link_id, DBG_DISABLE);
    }

    return status;
}

void dbg_tester_forward_lmp(uint8_t link_id, uint8_t * pdu, uint8_t buflen)
{
    uint8_t opcode;
    uint8_t acc_opcode;
    uint8_t status = CO_ERROR_UNSPECIFIED_ERROR;

    //Protection from overflow
    if ( (buflen > DM1_PACKET_SIZE) || (buflen == 0) )
    {
        //critical section
        GLOBAL_INT_DISABLE();
        //Free the LMP buffer
        co_free_buf(pdu);
        //end critical section
        GLOBAL_INT_RESTORE();
        return;
    }

    //critical section
    GLOBAL_INT_DISABLE();

    // Extract opcode
    opcode = (*pdu)>>1;

    // Check link ID
    if(link_id == dbg_tester_env.link_id)
    {
        // Check LMP message opcode
        if (opcode == LMP_ACCEPTED_OPCODE)
        {
            status = CO_ERROR_NO_ERROR;
        }
        else if (opcode == LMP_NOT_ACCEPTED_OPCODE)
        {
            // Extract original opcode
            acc_opcode = *(pdu+1);

            // Check original opcode
            if(acc_opcode == LMP_TEST_ACTIV_OPCODE || acc_opcode == LMP_TEST_CNTL_OPCODE )
            {
                // Extract error status
                status = *(pdu+2);
            }
        }

        // Call tester state machine event handler
        dbg_tester_lmp_acc_received(status);
    }

    // Free the LMP buffer
    co_free_buf(pdu);

    //end critical section
    GLOBAL_INT_RESTORE();
}
#endif //(BT_TESTER == 1)

/// @} DBGTESTER
