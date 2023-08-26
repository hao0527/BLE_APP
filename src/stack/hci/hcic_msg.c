
/**
 ****************************************************************************************
 * @addtogroup HCI_MSG
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

#include "co_endian.h"         // endian-ness declaration
#include "co_utils.h"          // common utility declaration
#include "co_bt.h"             // common bt definitiondeclaration
#include "co_error.h"          // common error code definition
#include "co_buf.h"            // stack buffering definition

#include "hcic.h"              // hci declaration
#include "hcic_msg.h"          // hci messages definition

#include "llm_task.h"          // link layer manager task definition
#include "llc_task.h"          // link layer controller task definition
#include "dbg_task.h"

#include "ke_task.h"           // kernel task definition
#include "ke_msg.h"            // kernel message definition


/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

// not used elsewhere
#if (BLE_CENTRAL || BLE_PERIPHERAL)
/**
 ****************************************************************************************
 * @brief Static function for swapping (if necssary) a short value without address change.
 *
 * @param[in,out] ptr   Pointer to the position where value remains(after swap).
 * @param[in]     value Short value to swap if necessary.
 *
 ****************************************************************************************
 */
static void hci_swap16(uint8_t * ptr, uint16_t value)
{
#if (!CPU_LE)
    co_write16p(ptr, co_htobs(value));
#endif // CPU_LE
}
#endif // #if (BLE_CENTRAL || BLE_PERIPHERAL)


/**
 ****************************************************************************************
 * @brief Static function for swapping (if necessary) and packing a short value to a new
 *        location.
 *
 * @param[in, out] ptr    Pointer to the position where value will be packed.
 * @param[in]      value  Short value to swap if necessary.
 *
 ****************************************************************************************
 */
static void hci_pack16(uint8_t * ptr, uint16_t value)
{
    co_write16p(ptr, co_htobs(value));
}
#if ((BLE_STD_MODE)&&(BLE_PERIPHERAL || BLE_CENTRAL))
/**
 *****************************************************************************************
 *@brief Static function handling the transmission of a an unknown connection handle
 *       complete event.
 *
 *       Necessary at UART reception, when connection handle of a command is not recognized.
 *
 *****************************************************************************************
 */
static void hci_unkconhdl_ccevt_send(uint16_t conhdl)
{
    struct llc_unknwn_conhdl_cmp_evt *evt;

    // Allocate a message structure for unknown command complete event
    evt = (struct llc_unknwn_conhdl_cmp_evt *)KE_MSG_ALLOC(LLC_UNKNWN_CH_CMP_EVT, TASK_HCI,
                                                   TASK_LLC, llc_unknwn_conhdl_cmp_evt);

    // Fill the parameter structure
    evt->opcode = (((uint16_t)hci_env.chdr.ogf) << 10) | ((uint16_t)hci_env.chdr.ocf);
    evt->status = CO_ERROR_UNKNOWN_CONNECTION_ID;
    evt->conhdl = conhdl;

    // Send the message
    ke_msg_send(evt);
}
#endif // ((BLE_STD_MODE)&&(BLE_PERIPHERAL || BLE_CENTRAL))
/*
 * FUNCTION DEFINITIONS - Lower Layers: HCI Legacy Commands content unpacking functions
 ****************************************************************************************
 */
///HCI Command unpacking function for all commands with aligned structures
void hci_cmd_aligned_unpk(uint8_t * bufptr)
{
    //send kernel message
    ke_msg_send(bufptr);
}

#if (BLE_PERIPHERAL || BLE_CENTRAL)

///HCI Generic Command unpack function for LLC commands with only conhdl to align
void hci_llc_basic_cmd_unpk(uint8_t * bufptr)
{
    uint16_t index;
    
    /* The commands to which this generic function applies to correspond to LLC structs
     * with only con_hdl parameter - which needs to be BE formatted, and other parameters
     * which require no pad adding for a correct structure mapping.*/

    struct ke_msg * msg = ke_param2msg(bufptr);
    //format connection handle position to BE
    hci_swap16(bufptr, co_read16p(bufptr));

    // Get the LE index, based on LE connection handle
    index = co_read16p(bufptr);

    // Check if the index is valid and the connection handle active
    if ((index < BLE_CONNECTION_MAX) && (ke_state_get(KE_BUILD_ID(TASK_LLC, index)) != LLC_FREE))
    {
        // Set kernel message destination to the right LLC task instance
        msg->dest_id = KE_BUILD_ID(TASK_LLC, index);
    }
    else
    {
        ke_msg_free(msg);

        // Rejects with a CC event with error code 'Unknown Connection Identifier'
        hci_unkconhdl_ccevt_send(index);

        return;
    }

    //Send kernel message
    ke_msg_send(bufptr);
}
#endif //(BLE_PERIPHERAL || BLE_CENTRAL)

///HCI Host Buffer Size Command unpacking function for LLM.
void hci_host_buffer_size_cmd_unpk(uint8_t * bufptr)
{
    //map structure on received buffer
    struct llm_host_buff_size_cmd * s = (struct llm_host_buff_size_cmd * )bufptr;

    /* Parameters 1 and 2 are unpacked in order and then
     * 3 and 4 in reverse to avoid value corruption in the reception buffer.
     */

    //4th par: unpack the uint16_t host_tot_sync_pk parameter Big Endian Byte mode
    s->nb_sync_pkts = co_btohs(co_read16p(bufptr+5));

    //3rd par: upack the uint16_t host_tot_acl_pk parameter Big Endian Byte mode
    s->nb_acl_pkts  = co_btohs(co_read16p(bufptr+3));

    //2nd par is in the correct position

    //1st par:unpack the uint16_t host_acl_pk_len parameter Big Endian Byte mode
    s->acl_pkt_len  = co_btohs(s->acl_pkt_len);

    //send kernel message
    ke_msg_send(s);
}


///HCI Host Number of Completed Packets Command unpacking function for LLM.
void hci_host_nb_completed_pkts_cmd_unpk(uint8_t * bufptr)
{
    //map structure on received buffer
    struct llm_host_nb_cmp_pkts_cmd * s = (struct llm_host_nb_cmp_pkts_cmd *)bufptr;

    //1st par:u8 number of handles parameter, is normally followed by a pad in dest struct
    s->nb_of_hdl = *bufptr;

    //unpack the struct pkts_per_hdl parameter in Big Endian Mode, params in reverse order
    for (int i = s->nb_of_hdl; i>0;i--)
    {
        //(2k+2)th par: the number of completed packets for ith handle.
        s->nb_comp_pkt[i-1] = co_btohs(co_read16p(bufptr + 4*(i-1)+3));

        //(2k+1)th par: connection handle value for index i
        s->con_hdl[i-1]     = co_btohs(co_read16p(bufptr + 4*(i-1)+1));
    }

    //send kernel message
    ke_msg_send(s);
}


/*
 * FUNCTION DEFINITIONS - Lower Layers: HCI LE Commands content unpacking functions
 ****************************************************************************************
 */
#if (BLE_PERIPHERAL || BLE_BROADCASTER)
///HCI LE Set Advertising Parameters Command unpacking function for LLM
void hci_le_set_adv_param_cmd_unpk( uint8_t * bufptr)
{
    //map structure on received buffer
    struct llm_le_set_adv_param_cmd * s = (struct llm_le_set_adv_param_cmd *)bufptr;

    /* Params aligned, but 1st two (may) need swapping because they are u16.
     */

    //1st par: u16 adv_intv_min - may swap
    s->adv_intv_min = co_btohs(s->adv_intv_min);

    //2nd par: u16 adv_intv_max - may swap
    s->adv_intv_max = co_btohs(s->adv_intv_max);

    //send kernel message
    ke_msg_send(s);
}
#endif // BLE_PERIPHERAL || BLE_BROADCASTER

#if (BLE_CENTRAL || BLE_OBSERVER)
///HCI LE Set Scan Parameters Command unpacking function for LLM.
void hci_le_set_scan_param_cmd_unpk(uint8_t * bufptr)
{
    //map structure on received buffer
    struct llm_le_set_scan_param_cmd * s = (struct llm_le_set_scan_param_cmd *)bufptr;

    /* Assume 1 byte pad after the first parameter of the structure =>
     * unpack 2nd to last parameters in reverse order to avoid corruption.
     */

    //5th par: u8 scan filter policy
    s->scan_filt_policy = *(bufptr+6);

    //4th par: u8 own address type
    s->own_addr_type = *(bufptr+5);

    //3rd par: u16 scan window  - (swap and) move
    s->scan_window = co_btohs(co_read16p(bufptr+3));

    //2nd par: u16 scan interval - (swap and) move
    s->scan_intv = co_btohs(co_read16p(bufptr+1));

    //1st par is on the correct position

    //send kernel message
    ke_msg_send(s);
}
#endif // BLE_CENTRAL || BLE_OBSERVER

#if (BLE_CENTRAL || BLE_PERIPHERAL)
///HCI LE Set Create Connection Command unpacking function for LLM.
void hci_le_create_con_cmd_unpk(uint8_t * bufptr)
{
    //map structure on received buffer
    struct llm_le_create_con_cmd * s = (struct llm_le_create_con_cmd *)bufptr;

    /* There should be one pad byte after own_addr_type=> unpack in order up to it
     * then work back from the end.
     * Since there are unint16_t values until 6th param, put them in BE format.
     */

    //1st par: scan_intv u16
    s->scan_intv = co_btohs(s->scan_intv);

    //2nd par: scan_window u16
    s->scan_window = co_btohs(s->scan_window);

    //the middle  params are aligned and do not need swap

    //start revrs unpacking to shift one position
    //12th par: ce_len_max u16
    s->ce_len_max   = co_btohs(co_read16p(bufptr+23));

    //11th par: ce_len_min u16
    s->ce_len_min   = co_btohs(co_read16p(bufptr+21));

    //10th par: superv_to u16
    s->superv_to    = co_btohs(co_read16p(bufptr+19));

    //9th par: con_latency u16
    s->con_latency  = co_btohs(co_read16p(bufptr+17));

    //8th par: con_intv_max u16
    s->con_intv_max = co_btohs(co_read16p(bufptr+15));

    //7th par: con_intv_min u16
    s->con_intv_min = co_btohs(co_read16p(bufptr+13));

    //send kernel message
    ke_msg_send(s);
}


///HCI LE Connection Update Command unpacking function for LLC.
void hci_le_con_update_cmd_unpk(uint8_t * bufptr)
{
    struct ke_msg * msg = ke_param2msg(bufptr);
    //map llc structure on received buffer - aligned, all u16, no need to move them
    struct llc_le_con_update_cmd * s  = (struct llc_le_con_update_cmd * )(bufptr);

    //Extract connection handle u16
    s->conhdl       = co_btohs(s->conhdl);

    // Check if the index is valid and the connection handle active
    if ((s->conhdl < BLE_CONNECTION_MAX) && (ke_state_get(KE_BUILD_ID(TASK_LLC, s->conhdl)) != LLC_FREE))
    {
        // Set kernel message destination to the right LLC task instance
        msg->dest_id = KE_BUILD_ID(TASK_LLC, s->conhdl);
    }
    else
    {
        ke_msg_free(msg);

        // Rejects with a CC event with error code 'Unknown Connection Identifier'
        hci_unkconhdl_ccevt_send(s->conhdl);

        return;
    }

    //2nd par: con_intv_min u16
    s->con_intv_min = co_btohs(s->con_intv_min);

    //3rd par: con_intv_max u16
    s->con_intv_max = co_btohs(s->con_intv_max);

    //4th par: con_latency u16
    s->con_latency  = co_btohs(s->con_latency);

    //5th par: superv_to u16
    s->superv_to    = co_btohs(s->superv_to);

    //6th par: ce_len_min u16
    s->ce_len_min   = co_btohs(s->ce_len_min);

    //7th par: ce_len_max u16
    s->ce_len_max   = co_btohs(s->ce_len_max);

    //send kernel message
    ke_msg_send(s);
}


///HCI LE Start Encryption Command unpacking function for LLC.
void hci_le_start_enc_cmd_unpk(uint8_t * bufptr)
{
    struct ke_msg * msg = ke_param2msg(bufptr);
    //map llc structure on received buffer - aligned, only u16 need swap
    struct llc_le_start_enc_cmd * s  = (struct llc_le_start_enc_cmd * )(bufptr);

    //Extract connection handle u16
    s->conhdl  = co_btohs(s->conhdl);

    // Check if the index is valid and the connection handle active
    if ((s->conhdl < BLE_CONNECTION_MAX) && (ke_state_get(KE_BUILD_ID(TASK_LLC, s->conhdl)) != LLC_FREE))
    {
        // Set kernel message destination to the right LLC task instance
        msg->dest_id = KE_BUILD_ID(TASK_LLC, s->conhdl);
    }
    else
    {
        ke_msg_free(msg);

        // Rejects with a CC event with error code 'Unknown Connection Identifier'
        hci_unkconhdl_ccevt_send(s->conhdl);

        return;
    }

    //3rd par: enc_div u16
    s->enc_div = co_btohs(s->enc_div);

    //send kernel message
    ke_msg_send(s);
}
#endif // BLE_CENTRAL || BLE_PERIPHERAL

#if (BLE_DEBUG)
void hci_dbg_rd_mem_cmd_unpk( uint8_t * bufptr)
{
    //map  structure on received buffer
    struct dbg_rd_mem_cmd * s  = (struct dbg_rd_mem_cmd * )(bufptr);

    //unpack in reverse
    s->length     = *(bufptr+5);
    s->type       = *(bufptr+4);
    s->start_addr = co_btohl(co_read32p(bufptr));

    //send kernel message
    ke_msg_send(s);
}

void hci_dbg_wr_mem_cmd_unpk( uint8_t * bufptr)
{
    //map  structure on received buffer
    struct dbg_wr_mem_cmd * s  = (struct dbg_wr_mem_cmd * )(bufptr);

    //unpack in reverse
    memcpy(&s->buf.data[0], bufptr+6, *(bufptr+5));
    s->buf.length = *(bufptr+5);
    s->type       = *(bufptr+4);
    s->start_addr = co_btohl(co_read32p(bufptr));

    //send kernel message
    ke_msg_send(s);
}

void hci_dbg_del_param_cmd_unpk( uint8_t * bufptr)
{
    //map  structure on received buffer
    struct dbg_del_param_cmd * s  = (struct dbg_del_param_cmd * )(bufptr);

    //unpack u16
    s->param_tag = co_btohs(co_read16p(bufptr));

    //send kernel message
    ke_msg_send(s);
}

///HCI Debug Flash Erase Command unpack function
void hci_dbg_flash_erase_cmd_unpk( uint8_t * bufptr)
{
    //map  structure on received buffer
    struct dbg_flash_erase_cmd * s  = (struct dbg_flash_erase_cmd * )(bufptr);

    //unpack in reverse
    s->size         = co_btohl(co_read32p(bufptr+5));
    s->startoffset  = co_btohl(co_read32p(bufptr+1));

    //send kernel message
    ke_msg_send(s);
}

///HCI Debug Flash Write Command unpack function
void hci_dbg_flash_write_cmd_unpk( uint8_t * bufptr)
{
    //map  structure on received buffer
    struct dbg_flash_write_cmd * s  = (struct dbg_flash_write_cmd * )(bufptr);
    uint8_t  tmp_tab[128];  //unpack in reverse
    uint8_t *tmp_buf = (bufptr+6);
    uint8_t tmp_size = *(bufptr+5);
    memcpy(tmp_tab, tmp_buf, tmp_size);
    s->buf.length  = *(bufptr+5);
    s->startoffset = co_btohl(co_read32p(bufptr+1));
    memcpy(&s->buf.data[0], tmp_tab, tmp_size);

    //send kernel message
    ke_msg_send(s);
}

///HCI Debug Flash Read Command unpack function
void hci_dbg_flash_read_cmd_unpk( uint8_t * bufptr)
{
    //map  structure on received buffer
    struct dbg_flash_read_cmd * s  = (struct dbg_flash_read_cmd * )(bufptr);

    s->size = *(bufptr+5);
    s->startoffset = co_btohl(co_read32p(bufptr+1));

    //send kernel message
    ke_msg_send(s);
}

///HCI Debug Read Param Command unpack function
void hci_dbg_rd_param_cmd_unpk( uint8_t * bufptr)
{
    //map  structure on received buffer
    struct dbg_rd_param_cmd * s  = (struct dbg_rd_param_cmd * )(bufptr);

    //unpack u16
    s->param_tag = co_btohs(co_read16p(bufptr));

    //send kernel message
    ke_msg_send(s);
}

///HCI Debug Write Param Command unpack function
void hci_dbg_wr_param_cmd_unpk( uint8_t * bufptr)
{
    //map  structure on received buffer
    struct dbg_wr_param_cmd * s  = (struct dbg_wr_param_cmd * )(bufptr);
    uint8_t  tmp_tab[128];
    uint8_t *tmp_buf = (bufptr+3);
    uint8_t tmp_size = *(bufptr+2);
    //unpack u16
    memcpy(tmp_tab, tmp_buf, tmp_size);
    s->buf.length = tmp_size;
    s->param_tag  = co_btohs(co_read16p(bufptr));
    memcpy(&s->buf.data[0], tmp_tab, tmp_size);
    //send kernel message
    ke_msg_send(s);
}

///HCI Debug HW Register Read Command unpacking function for DBG.
void hci_dbg_hw_reg_rd_cmd_unpk( uint8_t * bufptr)
{
    //map llc structure on received buffer - aligned, only u16 need swap
    struct dbg_hw_reg_rd_cmd * s  = (struct dbg_hw_reg_rd_cmd * )(bufptr);

    //Just swap reg_addr param if ncessary
    s->reg_addr  = co_btohs(s->reg_addr);

    //send kernel message
    ke_msg_send(s);
}


///HCI Debug HW Register Write Command unpacking function for DBG.
void hci_dbg_hw_reg_wr_cmd_unpk( uint8_t * bufptr)
{
    //map llc structure on received buffer - aligned, only u16 need swap
    struct dbg_hw_reg_wr_cmd * s  = (struct dbg_hw_reg_wr_cmd * )(bufptr);

    //The 2 params are aligned, but may need swap.
    s->reg_addr   = co_btohs(s->reg_addr);
    s->reg_value  = co_btohl(s->reg_value);

    //send kernel message
    ke_msg_send(s);
}


///HCI Debug Set CRC Command unpacking function for DBG.
void hci_dbg_set_crc_cmd_unpk( uint8_t * bufptr)
{
    //map llc structure on received buffer - aligned, only u16 need swap
    struct dbg_set_crc_cmd * s  = (struct dbg_set_crc_cmd * )(bufptr);

    //Just swap conhdl param if ncessary
    s->conhdl  = co_btohs(s->conhdl);

    //send kernel message
    ke_msg_send(s);
}


///HCI Debug Set CRC Command unpacking function for DBG.
void hci_dbg_llcp_discard_cmd_unpk( uint8_t * bufptr)
{
    //map llc structure on received buffer - aligned, only u16 need swap
    struct dbg_le_llcp_discard_req * s  = (struct dbg_le_llcp_discard_req * )(bufptr);

    //Just swap conhdl param if necessary
    s->conhdl  = co_btohs(s->conhdl);

    //send kernel message
    ke_msg_send(s);
}

///HCI Debug Set CRC Command unpacking function for DBG.
void hci_dbg_reset_cnt_cmd_unpk( uint8_t * bufptr)
{
    //map llc structure on received buffer - aligned, only u16 need swap
    struct dbg_reset_cnt_req * s  = (struct dbg_reset_cnt_req * )(bufptr);

    //Just swap conhdl param if ncessary
    s->conhdl  = co_btohs(s->conhdl);

    //send kernel message
    ke_msg_send(s);
}

///HCI Debug rf Register Read Command unpacking function for DBG.
void hci_dbg_rf_reg_rd_cmd_unpk( uint8_t * bufptr)
{
    //map llc structure on received buffer - aligned, only u16 need swap
    struct dbg_rf_reg_rd_cmd * s  = (struct dbg_rf_reg_rd_cmd * )(bufptr);

    //Just swap reg_addr param if necessary
    s->addr  = co_btohs(s->addr);

    //send kernel message
    ke_msg_send(s);
}


///HCI Debug rf Register Write Command unpacking function for DBG.
void hci_dbg_rf_reg_wr_cmd_unpk( uint8_t * bufptr)
{
    struct dbg_rf_reg_wr_cmd * s  = (struct dbg_rf_reg_wr_cmd * )(bufptr);

    s->value  = co_btohl(co_read32p(bufptr+2));
    s->addr   = co_btohs(co_read16p(bufptr));

    //send kernel message
    ke_msg_send(s);
}

///HCI Debug Set TX Power Level Command unpacking function for DBG
void hci_dbg_set_tx_pw_cmd_unpk( uint8_t * bufptr)
{
    //map llc structure on received buffer - aligned, only u16 need swap
    struct dbg_set_tx_pw_cmd * s  = (struct dbg_set_tx_pw_cmd * )(bufptr);

    //swap only conhdl
    s->conhdl   = co_btohs(s->conhdl);

    //send kernel message
    ke_msg_send(s);
}
#endif //BLE_DEBUG

/*
 * FUNCTION DEFINITIONS - Lower Layers: command complete events packing functions
 ****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief Static function for Command Complete Events header packing.
 *
 * The structures in the kernel messages handled contain CCEVT parameters from status on,
 * so this header actually represents everything from Event Code field up to Command
 * opcode.
 *
 * CCEVT fields are packed byte by byte into the kernel message space from
 * (dest_id field + 1) on. Opcodes and parameter lengths (considering as parameters the
 * fields from status on) are kept in tables with information about events correpsonding
 * to LLM or LLC.
 *
 * @param[in,out] msg  Pointer to Kernel message with information to send to Host.
 *
 * @return size of the packed HCI CCEVT packet (independent of transport header).
 *****************************************************************************************
 */
static int hci_ccevt_hdr_pk(struct ke_msg * msg)
{
    /* Use the ke msg space to pack the message to send starting dest id position
     * so that the parameter fields start at the same boundary in the HCI packet as in
     * the KE message */
     uint8_t * pk = (uint8_t *)(&msg->dest_id)+ 1;

     // Increase the number of commands available for reception
     hci_env.nb_h2c_cmd_pkts++;

     //extract opcode and length from table (only CCEVTS have relevant length, so no pb)
     uint16_t opcode = 0;
     uint8_t parlen = 0;
     int msg_idx;
     switch(KE_TYPE_GET(msg->src_id))
     {
         case TASK_LLM:
             msg_idx = LLM_EVTID2IDX(msg->id);
             opcode = hci_llm_epk[msg_idx].opcode;
             parlen = hci_llm_epk[msg_idx].parlen;
             break;

         #if (BLE_CENTRAL || BLE_PERIPHERAL)
         case TASK_LLC:
             msg_idx = LLC_EVTID2IDX(msg->id);
             opcode = hci_llc_epk[msg_idx].opcode;
             parlen = hci_llc_epk[msg_idx].parlen;
             break;
         #endif // BLE_CENTRAL || BLE_PERIPHERAL

         #if BLE_DEBUG
         case TASK_DBG:
             msg_idx = DBG_EVTID2IDX(msg->id);
             opcode = hci_dbg_epk[msg_idx].opcode;
             parlen = hci_dbg_epk[msg_idx].parlen;
             break;
         #endif //BLE_DEBUG

         default:
             ASSERT_ERR(0);
             break;
     }

    //pack event code
    *pk++ = (uint8_t)HCI_CMD_CMPL_EVT_CODE;

    //pack command complete event parameter length  - extracted from event pack table
    *pk++ = HCI_CCEVT_HDR_PARLEN + parlen;

    //pack the number of h2c packets
    *pk++ = hci_env.nb_h2c_cmd_pkts;

    //pack opcode - always swap because it is defined MSB to LSB
    hci_pack16(pk, opcode);

    return HCI_CCEVT_HDR_LEN + parlen;
}
#if (BLE_DEBUG)
/**
 *****************************************************************************************
 * @brief Static function for Command Complete Events header packing with variable size.
 *
 * The structures in the kernel messages handled contain CCEVT parameters from status on,
 * so this header actually represents everything from Event Code field up to Command
 * opcode.
 *
 * CCEVT fields are packed byte by byte into the kernel message space from
 * (dest_id field + 1) on. Opcodes and parameter lengths (considering as parameters the
 * fields from status on) are kept in tables with information about events corresponding
 * to LM or LC.
 *
 * @param[in,out] msg     Pointer to Kernel message with information to send to Host.
 * @param[in]     opcode  Command OPCODE.
 * @param[in]     len     Length of parameter.
 *
 * @return size of the packed HCI CCEVT packet (independent of transport header).
 *****************************************************************************************
 */
static int hci_ccevt_var_size_hdr_pk(struct ke_msg * msg, uint16_t opcode, uint8_t len)
{
    /* Use the ke msg space to pack the message to send starting dest id position
     * so that the parameter fields start at the same boundary in the HCI packet as in
     * the KE message */
     uint8_t * pk = (uint8_t *)(&msg->dest_id)+ 1;

     // Increase the number of commands available for reception
     hci_env.nb_h2c_cmd_pkts++;

    //pack event code
    *pk++ = (uint8_t)HCI_CMD_CMPL_EVT_CODE;

    //pack command complete event parameter length  - extracted from event pack table
    *pk++ = HCI_CCEVT_HDR_PARLEN + len;

    //pack the number of h2c packets
    *pk++ = hci_env.nb_h2c_cmd_pkts;

    //pack opcode - always swap because it is defined MSB to LSB
    hci_pack16(pk, opcode);

    return HCI_CCEVT_HDR_LEN + len;
}
#endif
///HCI Command Complete Event basic packing handler: for unpadded parameter structures.
int hci_ccevt_aligned_pk(struct ke_msg * msg)
{
    //fill ccevt header and return size of buffer for transport (params need no treatment)
    return hci_ccevt_hdr_pk(msg);
}


/*
 * FUNCTION DEFINITIONS - Lower Layers: Legacy command complete events packing functions
 ****************************************************************************************
 */
#if (BLE_CENTRAL || BLE_PERIPHERAL)
///HCI Flush Command Complete Event packing function for LLC.
int hci_flush_ccevt_pk(struct ke_msg * msg)
{
    //use the ke msg space to pack the message to send; jump header length
    uint8_t * pk = (uint8_t *)(&msg->dest_id) + 1 + HCI_CCEVT_HDR_LEN;

    //map structure for command return parameters on the ke msg param field - not aligned
    struct llc_flush_cmd_complete * s =
                                   (struct llc_flush_cmd_complete * )(msg->param);

    //pack the u8
    *pk++ = s->status;

    //pack connection handle u16
    hci_pack16(pk, s->conhdl);

    //fill ccevt header and return size of buffer for transport
    return hci_ccevt_hdr_pk(msg);
}
#endif // #if (BLE_CENTRAL || BLE_PERIPHERAL)

///HCI Read Local Version Information Command Complete Event packing function for LLM.
int hci_rd_local_ver_info_ccevt_pk(struct ke_msg * msg)
{
    //use the ke msg space to pack the message to send; jump header length
    uint8_t * pk = (uint8_t *)(&msg->dest_id) + 1 + HCI_CCEVT_HDR_LEN;

    //map structure for command return parameters on the ke msg param field - not aligned
    struct llm_rd_local_ver_info_cmd_complete * s =
                               (struct llm_rd_local_ver_info_cmd_complete * )(msg->param);

    //first two params are u8 - no need recopy
    pk +=2;

    // manuf name u16
    hci_pack16(pk, s->hci_rev);
    pk += 2;

    //skip lmp ver too, u8 aligned ot last, followed by pad
    pk += 1;


    // manuf name u16
    hci_pack16(pk, s->manuf_name);
    pk += 2;

    // subversion u16
    hci_pack16(pk, s->lmp_subver);

    //fill ccevt header and return size of buffer for transport
    return hci_ccevt_hdr_pk(msg);
}

#if (BLE_CENTRAL || BLE_PERIPHERAL)
///HCI Read Transmit Power Level Command Complete Event packing function for LLC.
int hci_rd_tx_pw_lvl_ccevt_pk(struct ke_msg * msg)
{
    //use the ke msg space to pack the message to send; jump header length
    uint8_t * pk = (uint8_t *)(&msg->dest_id) + 1 + HCI_CCEVT_HDR_LEN;

    //map structure for command return parameters on the ke msg param field - not aligned
    struct llc_rd_tx_pw_lvl_cmd_complete * s =
                                   (struct llc_rd_tx_pw_lvl_cmd_complete * )(msg->param);

    //pack the u8
    *pk++ = s->status;

    //pack connection handle u16
    hci_pack16(pk, s->conhdl);
    pk+=2;

    //pack the level u8
    *pk = s->tx_pow_lvl;

    //fill ccevt header and return size of buffer for transport
    return hci_ccevt_hdr_pk(msg);
}
#endif // BLE_CENTRAL || BLE_PERIPHERAL

///HCI Read Buffer Size Command Complete Event packing function for LLM.
int hci_rd_buff_size_ccevt_pk(struct ke_msg * msg)
{
    //use the ke msg space to pack the message to send; jump header length
    uint8_t * pk = (uint8_t *)(&msg->dest_id) + 1 + HCI_CCEVT_HDR_LEN;

    //map structure for command return parameters on the ke msg param field - not aligned
    struct llm_rd_buff_size_cmd_complete * s =
                                   (struct llm_rd_buff_size_cmd_complete * )(msg->param);

    //pack the u8
    *pk++ = s->status;

    //pack connection handle u16
    hci_pack16(pk, s->hc_data_pk_len);
    pk+=2;

    *pk++ = s->hc_sync_pk_len;

    hci_pack16(pk, s->hc_tot_nb_data_pkts);
    pk+=2;

    hci_pack16(pk, s->hc_tot_nb_sync_pkts);

    //fill ccevt header and return size of buffer for transport
    return hci_ccevt_hdr_pk(msg);
}

#if (BLE_CENTRAL || BLE_PERIPHERAL)
///HCI Read RSSI Command Complete Event packing function for LLC.
int hci_rd_rssi_ccevt_pk(struct ke_msg * msg)
{
    //use the ke msg space to pack the message to send; jump header length
    uint8_t * pk = (uint8_t *)(&msg->dest_id) + 1 + HCI_CCEVT_HDR_LEN;

    //map structure for command return parameters on the ke msg param field - not aligned
    struct llc_rd_rssi_cmd_complete * s =
                                   (struct llc_rd_rssi_cmd_complete * )(msg->param);

    //pack the u8
    *pk++ = s->status;

    //pack connection handle u16
    hci_pack16(pk, s->conhdl);
    pk+=2;

    //pack the rssi u8
    *pk = s->rssi;

    //fill ccevt header and return size of buffer for transport
    return hci_ccevt_hdr_pk(msg);
}
#endif // BLE_CENTRAL || BLE_PERIPHERAL

/*
 * FUNCTION DEFINITIONS - Lower Layers: LE command complete events packing functions
 ****************************************************************************************
 */
///HCI LE Read Buffer Size Command Complete Event pack function from LLM.
int hci_le_rd_buff_size_ccevt_pk(struct ke_msg * msg)
{
    //use the ke msg space to pack the message to send; jump header length
    uint8_t * pk = (uint8_t *)(&msg->dest_id) + 1 + HCI_CCEVT_HDR_LEN;

    //map structure for command return parameters on the ke msg param field - not aligned
    struct llm_le_rd_buff_size_cmd_complete * s =
                                  (struct llm_le_rd_buff_size_cmd_complete *)(msg->param);

    //pack the u8
    *pk++ = s->status;

    //pack connection handle u16
    hci_pack16(pk, s->hc_data_pk_len);
    pk +=2;

    *pk = s->hc_tot_nb_data_pkts;

    //fill ccevt header and return size of buffer for transport
    return hci_ccevt_hdr_pk(msg);
}

#if (BLE_CENTRAL || BLE_PERIPHERAL)
///HCI LE Read Channel Map Command Complete Event pack function from LLC.
int hci_le_rd_chnl_map_ccevt_pk(struct ke_msg * msg)
{
    //use the ke msg space to pack the message to send; jump header length
    uint8_t * pk = (uint8_t *)(&msg->dest_id) + 1 + HCI_CCEVT_HDR_LEN;

    //map structure for command return parameters on the ke msg param field - not aligned
    struct llc_le_rd_chnl_map_cmd_complete * s =
                                  (struct llc_le_rd_chnl_map_cmd_complete *)(msg->param);

    //pack the u8
    *pk++ = s->status;

    //pack connection handle u16
    hci_pack16(pk, s->conhdl);
    pk +=2;

    //copy the array in order
    for (int i=0; i< LE_CHNL_MAP_LEN; i++)
    {
        *pk++ = s->ch_map.map[i];
    }

    //fill ccevt header and return size of buffer for transport
    return hci_ccevt_hdr_pk(msg);
}


///HCI LE Long Term Key Request Reply Command Complete Event pack function from LLC.
int hci_le_ltk_req_rply_ccevt_pk(struct ke_msg * msg)
{
    //use the ke msg space to pack the message to send; jump header length
    uint8_t * pk = (uint8_t *)(&msg->dest_id) + 1 + HCI_CCEVT_HDR_LEN;

    //map structure for command return parameters on the ke msg param field - not aligned
    struct llc_event_common_cmd_complete * s =
                                   (struct llc_event_common_cmd_complete * )(msg->param);

    //pack the u8
    *pk++ = s->status;

    //pack connection handle u16
    hci_pack16(pk, s->conhdl);

    //fill ccevt header and return size of buffer for transport
    return hci_ccevt_hdr_pk(msg);
}


///HCI LE Long Term Key Request Negative Reply Command Complete Event pack function from LLC.
int hci_le_ltk_req_neg_rply_ccevt_pk(struct ke_msg * msg)
{
    //use the ke msg space to pack the message to send; jump header length
    uint8_t * pk = (uint8_t *)(&msg->dest_id) + 1 + HCI_CCEVT_HDR_LEN;

    //map structure for command return parameters on the ke msg param field - not aligned
    struct llc_event_common_cmd_complete * s =
                                   (struct llc_event_common_cmd_complete * )(msg->param);

    //pack the u8
    *pk++ = s->status;

    //pack connection handle u16
    hci_pack16(pk, s->conhdl);

    //fill ccevt header and return size of buffer for transport
    return hci_ccevt_hdr_pk(msg);
}
#endif // BLE_CENTRAL || BLE_PERIPHERAL

///HCI LE Test End Command Complete Event pack function from LLM.
int hci_le_test_end_ccevt_pk(struct ke_msg * msg)
{
    //use the ke msg space to pack the message to send; jump header length
    uint8_t * pk = (uint8_t *)(&msg->dest_id) + 1 + HCI_CCEVT_HDR_LEN;

    //map structure for command return parameters on the ke msg param field - not aligned
    struct llm_test_end_cmd_complete * s =
                                    (struct llm_test_end_cmd_complete * )(msg->param);

    //pack the u8
    *pk++ = s->status;

    //pack number of rx packets u16
    hci_pack16(pk, s->nb_packet_received);

    //fill ccevt header and return size of buffer for transport
    return hci_ccevt_hdr_pk(msg);
}


///HCI generic Command Status Event pack function.
int hci_csevt_pk(struct ke_msg * msg)
{
    //recover the opcode of the command of which status is returned to source.
     uint16_t opcode = 0;

     // Increase the number of commands available for reception
     hci_env.nb_h2c_cmd_pkts++;

     switch(KE_TYPE_GET(msg->src_id))
     {
         case TASK_LLM:
             opcode = hci_llm_epk[LLM_EVTID2IDX(msg->id)].opcode;
             break;

         #if (BLE_CENTRAL || BLE_PERIPHERAL)
         case TASK_LLC:
             opcode = hci_llc_epk[LLC_EVTID2IDX(msg->id)].opcode;
             break;
         #endif // BLE_CENTRAL || BLE_PERIPHERAL

        #if (BLE_DEBUG)
        case TASK_DBG:
            opcode = hci_dbg_epk[DBG_EVTID2IDX(msg->id)].opcode;
            break;
        #endif //(BLE_DEBUG)

         default:
             ASSERT_ERR(0);
             break;
     }

    //use the ke msg space to pack the message to send
     uint8_t * pk = ((uint8_t *)msg->param) + TYPE_CSEVT;

    //pack event code
    *pk++= (uint8_t)HCI_CMD_STATUS_EVT_CODE;

    //pack parameter length:4
    *pk++ = (uint8_t)HCI_CSEVT_PARLEN;

    //pack the parameters in the structure
    //status is in place
    *pk++ = msg->param[0];

    //hdr
    *pk++ = hci_env.nb_h2c_cmd_pkts;

    //opcode
    hci_pack16(pk, opcode);

    return HCI_CSEVT_LEN;
}

#if ((BLE_STD_MODE)&&(BLE_PERIPHERAL || BLE_CENTRAL))
/// Unknown Connection Handle Complete Event packing function for LC.
int hci_unknwn_conhdl_ccevt_pk(struct ke_msg * msg)
{
    //use the ke msg space to pack the message to send; jump header length
    uint8_t * pk = (uint8_t *)(&msg->dest_id) + 1;

    //map structure for command return parameters on the ke msg param field - not aligned
    struct llc_unknwn_conhdl_cmp_evt * s = (struct llc_unknwn_conhdl_cmp_evt * )(msg->param);

    // Increase the number of commands available for reception
    hci_env.nb_h2c_cmd_pkts++;

    //pack event code
    *pk++ = (uint8_t)HCI_CMD_CMPL_EVT_CODE;

    //pack command complete event parameter length  - extracted from event pack table
    *pk++ = HCI_CCEVT_HDR_PARLEN + 3;

    //pack the number of h2c packets
    *pk++ = hci_env.nb_h2c_cmd_pkts;

    //pack opcode - always swap because it is defined MSB to LSB
    hci_pack16(pk, s->opcode);
    pk += 2;
    *pk++ = s->status;
    //pack connection handle u16
    hci_pack16(pk, s->conhdl);

   return HCI_CCEVT_HDR_LEN + 3;
}
#endif // (BLE_STD_MODE)

/*
 * FUNCTION DEFINITIONS - Lower Layers: Legacy special events packing functions
 ****************************************************************************************
 */
#if (BLE_CENTRAL || BLE_PERIPHERAL)
///HCI Disconnection Complete Event packing function for LLC.
int hci_disconnection_cmp_evt_pk(struct ke_msg * msg)
{
    //recover the structure for HCI, LLC has the extra subevt code u8 at beginning
    struct llc_disc_evt_complete * s = (struct llc_disc_evt_complete * )((msg->param));

    //use the ke msg space to pack the message to send
    uint8_t * pk = ((uint8_t *)&msg->param_len);

    //pack event code
    *pk++ = (uint8_t)HCI_DISCONNECTION_CMPL_EVT_CODE;

    //pack parameter length
    *pk++ = HCI_DISCCMP_EVTPAR_LEN;

    //pack number of reports
    *pk++ = s->status;

    //pack rscnx hdlsi
    hci_pack16(pk, s->conhdl);
    pk+=2;

    //pack number of reports
    *pk = s->reason;

    return HCI_EVT_HDR_LEN + HCI_DISCCMP_EVTPAR_LEN;
}

///HCI Read Remote Version Information Complete Event packing function for LLC.
int hci_rd_rem_ver_info_cmp_evt_pk(struct ke_msg * msg)
{
    //recover the structure for HCI, LLC has the extra subevt code u8 at beginning
    struct llc_rd_rem_info_ver_cmd_complete * s =
                               (struct llc_rd_rem_info_ver_cmd_complete * )((msg->param));

    //use the ke msg space to pack the message to send
    uint8_t * pk = ((uint8_t *)&msg->param_len);

    //pack event code
    *pk++ = (uint8_t)HCI_RD_REM_VER_INFO_CMPL_EVT_CODE;

    //pack parameter length
    *pk++ = HCI_RDREMVI_EVTPAR_LEN;

    //pack status u8
    *pk++ = s->status;

    //pack connection handle
    hci_pack16(pk, s->conhdl);
    pk +=2;

    //pack lmp version u8
    *pk++ = s->vers;

    //pack manufacturer name u16
    hci_pack16(pk, s->compid);
    pk +=2;

    //pack lmp subversion u16
    hci_pack16(pk, s->subvers);

    return HCI_EVT_HDR_LEN + HCI_RDREMVI_EVTPAR_LEN;
}


///HCI Flush Occurred Event packing function for LLC.
int hci_flush_occurred_evt_pk(struct ke_msg * msg)
{
    //recover the structure for HCI, LLC has the extra subevt code u8 at beginning
    struct llc_flush_occurred_evt * s = (struct llc_flush_occurred_evt * )((msg->param));

    //use the ke msg space to pack the message to send
    uint8_t * pk = ((uint8_t *)&msg->param_len);

    //pack event code
    *pk++ = (uint8_t)HCI_FLUSH_OCCURRED_EVT_CODE;

    //pack parameter length
    *pk++ = HCI_FLSHOCC_EVTPAR_LEN;

    //pack connection handle
    hci_pack16(pk, s->conhdl);

    return HCI_EVT_HDR_LEN + HCI_FLSHOCC_EVTPAR_LEN;
}


///HCI Encryption Change Event packing function for LLC.
int hci_enc_change_evt_pk(struct ke_msg * msg)
{
    //recover the structure for HCI, LLC has the extra subevt code u8 at beginning
    struct llc_enc_change_evt * s = (struct llc_enc_change_evt * )((msg->param));

    //use the ke msg space to pack the message to send
    uint8_t * pk = ((uint8_t *)&msg->param_len);

    //pack event code
    *pk++ = (uint8_t)HCI_ENC_CHG_EVT_CODE;

    //pack parameter length
    *pk++ = HCI_ENCCHG_EVTPAR_LEN;

    *pk++ = s->status;

    //pack connection handle
    hci_pack16(pk, s->conhdl);
    pk += 2;

    *pk = s->enc_stat;

    return HCI_EVT_HDR_LEN + HCI_ENCCHG_EVTPAR_LEN;
}


///HCI Encryption Key Refresh Event packing function for LLC.
int hci_enc_key_refresh_evt_pk(struct ke_msg * msg)
{
    //recover the structure for HCI, LLC has the extra subevt code u8 at beginning
    struct llc_enc_key_refresh_evt * s = (struct llc_enc_key_refresh_evt *)((msg->param));

    //use the ke msg space to pack the message to send
    uint8_t * pk = ((uint8_t *)&msg->param_len);

    //pack event code
    *pk++ = (uint8_t)HCI_ENC_KEY_REFRESH_CMPL_EVT_CODE;

    //pack parameter length
    *pk++ = HCI_ENCREFSH_EVTPAR_LEN;

    *pk++ = s->status;

    //pack connection handle
    hci_pack16(pk, s->conhdl);

    return HCI_EVT_HDR_LEN + HCI_ENCREFSH_EVTPAR_LEN;
}
#endif // BLE_CENTRAL || BLE_PERIPHERAL

///HCI Hardware Error Event packing function for LLM
int hci_hw_error_evt_pk(struct ke_msg * msg)
{
    //use the ke msg space to pack the message to send
    uint8_t * pk = ((uint8_t *)&msg->param_len);

    //pack event code
    *pk++ = (uint8_t)HCI_HW_ERR_EVT_CODE;

    //pack parameter length
    *pk++ = HCI_HWERR_EVTPAR_LEN;

    //params aligned, no need to remap

    return HCI_EVT_HDR_LEN + HCI_HWERR_EVTPAR_LEN;
}


///HCI Data Buffer Overflow Event packing function for LLC.
int hci_data_buf_ovflw_evt_pk(struct ke_msg * msg)
{
    //use the ke msg space to pack the message to send
    uint8_t * pk = ((uint8_t *)&msg->param_len);

    //pack event code
    *pk++ = (uint8_t)HCI_DATA_BUFF_OVFLW_EVT_CODE;

    //pack parameter length
    *pk++ = HCI_DBOVFLW_EVTPAR_LEN;

    //params aligned, no need to remap

    return HCI_EVT_HDR_LEN + HCI_DBOVFLW_EVTPAR_LEN;
}

#if (BLE_CENTRAL || BLE_PERIPHERAL)
///HCI Number of Completed Packets Event packing function for LLC
int hci_nb_cmp_pkts_evt_pk(struct ke_msg * msg)
{
    //use the ke msg space to pack the message to send; jump header length
    uint8_t * pk = (uint8_t *)(&msg->param_len);

    struct llc_nb_of_pkt_evt_complete * s =
                                        (struct llc_nb_of_pkt_evt_complete *)(msg->param);

    //save number of handles
    uint8_t nbhdl = s->nb_of_hdl;

    //pack event code
    *pk++ = (uint8_t)HCI_NB_CMPL_PKTS_EVT_CODE;

    //pack parameter length
    *pk++ = 1 + nbhdl*4;

    //1st par: nb handles u8
    *pk++ = nbhdl;

    //loop for all handles+nb packets
    for (int i=0; i<nbhdl; i++)
    {
        //(2k+1)th par: connection handle u16
        hci_pack16(pk, s->conhdl[i]);
        pk +=2;

        //(2k+2)th par: number of packets u16
        hci_pack16(pk, s->nb_comp_pkt[i]);
        pk +=2;
    }

    return HCI_EVT_HDR_LEN + 1 + nbhdl*4;
}

/*
 * FUNCTION DEFINITIONS - Lower Layers: LE special events packing functions
 ****************************************************************************************
 */
///HCI LE Connection Complete Event pack function for LLC.
int hci_le_con_cmp_evt_pk(struct ke_msg * msg)
{
    //use the ke msg space to pack the message to send
    uint8_t * pk = ((uint8_t *)&msg->param_len);

    //pack event code
    *pk++ = (uint8_t)HCI_LE_META_EVT_CODE;

    //fixed param len
    *pk++ = HCI_LECONCMP_EVTPAR_LEN;

    //recover the structure for HCI
    struct llc_create_con_cmd_complete * s =
                                      (struct llc_create_con_cmd_complete * )(msg->param);

    //pack sub event code
    *pk++ = s->subevent_code;

    //pack command status u8
    *pk++ = s->status;

    //pack connection handle u16
    hci_swap16(pk, s->conhdl);
    pk += 2;

    //pack device role u8
    *pk++ = s->role;

    //pack peer address type  u8
    *pk++ = s->peer_addr_type;

    //no need to move the address
    pk += BD_ADDR_LEN;

    //pack connection interval u16
    hci_swap16(pk, s->con_interval);
    pk += 2;

    //pac connection latency u16
    hci_swap16(pk, s->con_latency);
    pk += 2;

    //apck link supervision timeout u16
    hci_swap16(pk, s->sup_to);
    pk += 2;

    //pack master clock accuracy u8
    *pk = s->clk_accuracy;

    return HCI_EVT_HDR_LEN + HCI_LECONCMP_EVTPAR_LEN;
}


///HCI LE Connection Update Complete Event packing function for LLC.
int hci_le_con_update_cmp_evt_pk(struct ke_msg * msg)
{
    //use the ke msg space to pack the message to send
    uint8_t * pk = ((uint8_t *)&msg->param_len);

    //pack event code
    *pk++ = (uint8_t)HCI_LE_META_EVT_CODE;

    //fixed param len
    *pk++ = HCI_LECONUPDP_EVTPAR_LEN;

    struct llc_le_con_update_cmd_complete * s =
                                   (struct llc_le_con_update_cmd_complete * )(msg->param);

    //pack sub event code
    *pk++ = s->subevent_code;

    //pack command status u8
    *pk++ = s->status;

    //pack connection interval u16
    hci_swap16(pk, s->conhdl);
    pk += 2;

    //pac connection latency u16
    hci_swap16(pk, s->con_interval);
    pk += 2;

    //pac connection latency u16
    hci_swap16(pk, s->con_latency);
    pk += 2;

    //apck link supervision timeout u16
    hci_swap16(pk, s->sup_to);

    return HCI_EVT_HDR_LEN + HCI_LECONUPDP_EVTPAR_LEN;
}


///HCI LE Read Remote Used Features Complete Event packing function for LLC.
int hci_le_rd_rem_used_feats_cmp_evt_pk(struct ke_msg * msg)
{
    //use the ke msg space to pack the message to send
    uint8_t * pk = ((uint8_t *)&msg->param_len);

    //pack event code
    *pk++ = (uint8_t)HCI_LE_META_EVT_CODE;

    //fixed param len
    *pk++ = HCI_LERDREMFEATS_EVTPAR_LEN;

    struct llc_le_rd_rem_used_feats_cmd_complete * s =
                            (struct llc_le_rd_rem_used_feats_cmd_complete * )(msg->param);

    //1st two aligned and u8, no need to copy
    pk += 2;

    //pack connection interval u16
    hci_swap16(pk, s->conhdl);

    //features array is aligned too, so need ot recopy

    return HCI_EVT_HDR_LEN + HCI_LERDREMFEATS_EVTPAR_LEN;
}
#endif // BLE_CENTRAL || BLE_PERIPHERAL

///HCI LE Advertising Report Event packing function for LLM(?)
int hci_le_adv_report_evt_pk(struct ke_msg * msg)
{
    //recover the structure for HCI, LLC has the extra subevt code u8 at beginning
    struct llm_le_adv_report_evt * s =
                                    (struct llm_le_adv_report_evt * )(msg->param);

    //local variable for data length
    uint8_t dlen;

    //local variable for rssi save, because it may be overwritten by data packing
    uint8_t rssi;

    //local variable for report in array
    struct adv_report rep;

    //use the ke msg space to pack the message to send
    uint8_t * pk =  ((uint8_t *)&msg->param_len);

    //keep pointer to length field to update it
    uint8_t * ptr_len = pk + HCI_EVT_CODE_LEN;

    //Extract number of reports
    uint8_t nbrep = s->nb_reports;

    //pack event code
    *pk++ = (uint8_t)HCI_LE_META_EVT_CODE;

    //pack parameter length
    *pk++ = 0;*ptr_len = 0;

    //subevent code and numbe rof reports are two u8 at right position
    pk +=2; (*ptr_len)+= 2;

    for (int i=0; i< nbrep; i++)
    {
        int j;

        rep = s->adv_rep[i];

        //pack the report's event type
        *pk++ = rep.evt_type;
        (*ptr_len)++;

        //pack the report's advertising address type
        *pk++ = rep.adv_addr_type;
        (*ptr_len)++;

        //pack the array
        for (j = 0; j < BD_ADDR_LEN; j++)
        {
            *pk++ = rep.adv_addr.addr[j];
        }
        (*ptr_len)+= BD_ADDR_LEN;

        //get rssi value, which needs to go at end
        rssi = rep.rssi;

        //pack data length
        *pk++ = rep.data_len;
        dlen = rep.data_len;
        (*ptr_len)++;

        //pack data in the same order (adv data is not handled by hci)
        for (j = 0; j < dlen; j++)
        {
            *pk++ = rep.data[j];
        }
        (*ptr_len) += dlen;

        //pack rssi
        *pk++ = rssi;
        (*ptr_len) ++;
    }

    return HCI_EVT_HDR_LEN + (*ptr_len);
}

#if (BLE_CENTRAL || BLE_PERIPHERAL)
///HCI LE Long Term Key Request Event packing function for LLC
int hci_le_ltk_req_evt_pk(struct ke_msg * msg)
{
    //use the ke msg space to pack the message to send
    uint8_t * pk = ((uint8_t *)&msg->param_len);

    //pack event code
    *pk++ = (uint8_t)HCI_LE_META_EVT_CODE;

    //fixed param len
    *pk++ = HCI_LELTKREQ_EVTPAR_LEN;

    //recover the structure for HCI, LLC has the extra subevt code u8 at beginning
    struct llc_le_ltk_req * s = (struct llc_le_ltk_req * )(msg->param);

    //subevent code is on right position, followed by a pad, thus all must be recopied
    pk++;

    //pack connection interval u16
    hci_pack16(pk, s->conhdl);
    pk += 2;

    for (int i = 0; i< RAND_NB_LEN; i++)
    {
        *pk++ = s->rand.nb[i];
    }

    //pack connection interval u16
    hci_pack16(pk, s->ediv);

    return HCI_EVT_HDR_LEN + HCI_LELTKREQ_EVTPAR_LEN;
}
#endif // #if (BLE_CENTRAL || BLE_PERIPHERAL)

#if (BLE_DEBUG)
///HCI Debug flash read Command Complete Event packing function.
int hci_dbg_flash_rd_evt_pk(struct ke_msg * msg)
{
    uint8_t len = 0;

    //map structure for command return parameters on the ke msg param field - not aligned
    struct dbg_flash_read_cmp_evt * s = (struct dbg_flash_read_cmp_evt * )(msg->param);

    // Get data buffer length from structure
    len = s->buf.length;

    // Fill ccevt header and return size of buffer for transport
    return hci_ccevt_var_size_hdr_pk(msg, HCI_DBG_FLASH_RD_CMD_OPCODE, 2 + len);
}

///HCI Debug Read Param Command Complete Event packing function.
int hci_dbg_rd_par_ccevt_pk(struct ke_msg * msg)
{
    uint8_t len = 0;

    struct dbg_rd_param_cmp_evt * s = (struct dbg_rd_param_cmp_evt * )(msg->param);

    // Get data buffer length from structure
    len = s->buf.length;

    // Fill ccevt header and return size of buffer for transport
    return hci_ccevt_var_size_hdr_pk(msg, HCI_DBG_RD_PAR_CMD_OPCODE, 2 + len);
}

///HCI Debug Read Memory Command Complete Event packing function.
int hci_dbg_rd_mem_ccevt_pk(struct ke_msg * msg)
{
    uint8_t len = 0;

    struct dbg_rd_mem_cmp_evt * s = (struct dbg_rd_mem_cmp_evt * )(msg->param);

    // Get data buffer length from structure
    len = s->buf.length;

    // Fill ccevt header and return size of buffer for transport
    return hci_ccevt_var_size_hdr_pk(msg, HCI_DBG_RD_MEM_CMD_OPCODE, 2 + len);
}

///HCI Debug HW Register Read Complete Event packing function for DEBUG
int hci_dbg_hw_reg_rd_ccevt_pk(struct ke_msg * msg)
{
    //use the ke msg space to pack the message to send; jump header length
    uint8_t * pk = (uint8_t *)(&msg->dest_id) + 1 + HCI_CCEVT_HDR_LEN;

    //map structure for command return parameters on the ke msg param field - not aligned
    struct dbg_hw_reg_rd_cmd_cmp_evt * s = (struct dbg_hw_reg_rd_cmd_cmp_evt * )(msg->param);

    //pack register address
    hci_pack16(pk+1, s->reg_addr);

    //pack register value
    co_write32p(pk+3, s->reg_value);

    //fill ccevt header and return size of buffer for transport
    return hci_ccevt_hdr_pk(msg);
}

///HCI Debug HW Register Read Complete Event packing function for DEBUG
int hci_dbg_rd_mem_info_ccevt_pk(struct ke_msg * msg)
{
    //use the ke msg space to pack the message to send; jump header length
    uint8_t * pk = (uint8_t *)(&msg->dest_id) + 1 + HCI_CCEVT_HDR_LEN +1;
    uint8_t i;

    //map structure for command return parameters on the ke msg param field - not aligned
    struct dbg_rd_mem_info_cmp_evt * s = (struct dbg_rd_mem_info_cmp_evt * )(msg->param);

    for (i = 0 ; i < KE_MEM_BLOCK_MAX; i++)
    {
        //pack memory used
        hci_pack16(pk, s->mem_used[i]);
        pk += 2;
    }

    //pack register value
    co_write32p(pk, s->max_mem_used);

    //fill ccevt header and return size of buffer for transport
    return hci_ccevt_hdr_pk(msg);
}

#if (DSM_SUPPORT)
///HCI Debug Data Storage Indication event packing function
int hci_dbg_data_storage_ind_evt_pk(struct ke_msg * msg)
{
    uint8_t length;

    //map structure for event parameters on the ke msg param field - not aligned
    struct dbg_data_storage_ind_evt * s = (struct dbg_data_storage_ind_evt * )(msg->param);

    //use the ke msg space to pack the message to send; jump header length
    uint8_t * pk = (uint8_t *)(&msg->param_len);

    length = s->length;

    *pk++ = HCI_DBG_EVT_CODE;
    *pk++ = 4 + length;
    *pk++ = HCI_DBG_DATA_STORAGE_IND_EVT_SUBCODE;
    *pk++ = s->status;
    *pk++ = s->operation;
    *pk++ = length;

    if(length > 0)
    {
        memcpy(pk, &s->sample[0], length);
    }

    // Compute parameters size
    return HCI_EVT_HDR_LEN + 4 + length;
}
#endif //DSM_SUPPORT



///HCI Debug HW Register Write Complete Event packing function for DEBUG
int hci_dbg_hw_reg_wr_ccevt_pk(struct ke_msg * msg)
{
    //use the ke msg space to pack the message to send; jump header length
    uint8_t * pk = (uint8_t *)(&msg->dest_id) + 1 + HCI_CCEVT_HDR_LEN;

    //map structure for command return parameters on the ke msg param field - not aligned
    struct dbg_hw_reg_wr_cmd_cmp_evt * s = (struct dbg_hw_reg_wr_cmd_cmp_evt * )(msg->param);

    //pack register address
    hci_pack16(pk+1, s->reg_addr);

    //fill ccevt header and return size of buffer for transport
    return hci_ccevt_hdr_pk(msg);
}

///HCI Debug rf Register Read Complete Event packing function for DEBUG
int hci_dbg_rf_reg_rd_ccevt_pk(struct ke_msg * msg)
{
    //use the ke msg space to pack the message to send; jump header length
    uint8_t * pk = (uint8_t *)(&msg->dest_id) + 1 + HCI_CCEVT_HDR_LEN;

    //map structure for command return parameters on the ke msg param field - not aligned
    struct dbg_rf_reg_rd_cmp_evt * s = (struct dbg_rf_reg_rd_cmp_evt * )(msg->param);

    //pack register address
    hci_pack16(pk+1, s->addr);

    //pack register value
    co_write32p(pk+3, s->value);

    //fill ccevt header and return size of buffer for transport
    return hci_ccevt_hdr_pk(msg);
}

///HCI Debug rf Register Write Complete Event packing function for DEBUG
int hci_dbg_rf_reg_wr_ccevt_pk(struct ke_msg * msg)
{
    //use the ke msg space to pack the message to send; jump header length
    uint8_t * pk = (uint8_t *)(&msg->dest_id) + 1 + HCI_CCEVT_HDR_LEN;

    //map structure for command return parameters on the ke msg param field - not aligned
    struct dbg_rf_reg_wr_cmp_evt * s = (struct dbg_rf_reg_wr_cmp_evt * )(msg->param);

    //pack register address
    hci_pack16(pk+1, s->addr);

    //fill ccevt header and return size of buffer for transport
    return hci_ccevt_hdr_pk(msg);
}
#endif //BLE_DEBUG


#if (BLE_CENTRAL || BLE_PERIPHERAL)
///HCI Data packing handler (for REQ or IND)
int hci_data_pk(struct ke_msg * msg)
{
     uint16_t dlen;

     // Get the RX descriptor corresponding to the handle
    struct llc_data_packet_ind *s = (struct llc_data_packet_ind *)(msg->param);
    struct co_buf_rx_desc *rxdesc = co_buf_rx_get(s->rx_hdl);
    //use the descriptor space to pack the message to send
     uint8_t *pk = (uint8_t *)(&rxdesc->data[0]) - HCI_ACL_HDR_LEN;

    dlen   = s->length;

    hci_pack16(pk, ((uint16_t)(s->pb_bc_flag)<<12)| (s->conhdl & 0x0FFF));
    pk +=2;

    //pack connection handle u16
    hci_pack16(pk, s->length);
    pk +=2;

    return HCI_ACL_HDR_LEN + dlen;
}
#endif // BLE_CENTRAL || BLE_PERIPHERAL

#endif //BLE_HCIC_ITF

/// @} HCI_MSG
