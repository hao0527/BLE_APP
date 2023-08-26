
#ifndef LLC_TASK_H_
#define LLC_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup LLCTASK LLCTASK
 * @ingroup LLC
 * @brief Link Layer Controller Task
 *
 * The LLC task is responsible for managing link layer actions related to a
 * specific connection with a peer (e.g. Encryption setup, Supervision timeout, etc.). It
 * implements the state machine related to these actions.
 *
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stdint.h>
#include "co_bt.h"
#include "ke_task.h"

#if (BLE_PERIPHERAL || BLE_CENTRAL)
/*
 * INSTANCES
 ****************************************************************************************
 */
/// Maximum number of instances of the LLC task
#define LLC_IDX_MAX  BLE_CONNECTION_MAX

/// LE events subcodes
#define LLC_LE_EVT_CON_CMP          0x01
#define LLC_LE_EVT_UPD_CMP          0x03
#define LLC_LE_EVT_USE_FEAT_CMP     0x04
#define LLC_LE_EVT_LGT_KEY_REQ      0x05

/*
 * STATES
 ****************************************************************************************
 */
/// Possible states of the LLC task
enum llc_state_id
{
    /// IDLE state
    LLC_FREE,
    /// CONNECTED state
    LLC_CONNECTED,
    /// Wait for feature response
    LLC_FEAT_WAIT_RSP,
    /// Wait for peer version indication (when we initiated the procedure)
    LLC_VERS_WAIT_IND,
    /// Wait for completion of connection parameter update procedure
    LLC_CON_UPD_WAIT_INSTANT,
    /// Wait for completion of channel map update procedure
    LLC_MAP_UPD_WAIT_INSTANT,			//  5
    /// Wait for acknowledgment of terminate indication
    LLC_TERM_WAIT_ACK,
    /// Wait for completion of data traffic before encryption procedure
    LLC_ENC_PAUSING_TRAFFIC,
    /// Data traffic is now stopped, encryption procedure can start
    LLC_ENC_TRAFFIC_PAUSED,
    /// Wait for encryption response
    LLC_ENC_WAIT_RSP,
    /// Wait for start encryption request
    LLC_ENC_WAIT_START_REQ,				//  10
    /// Wait for start encryption response
    LLC_ENC_WAIT_START_RSP,
    /// Wait for Session Key generated by HW AES engine
    LLC_ENC_WAIT_SK,
    /// Wait for LTK from host
    LLC_ENC_WAIT_LTK,				//13
    /// Encryption pause procedure is ongoing
    LLC_ENC_WAIT_PAUSE_RSP,
    /// Encryption pause procedure is complete
    LLC_ENC_PAUSED,
    /// Wait for encryption request (when a encryption pause has been completed)
    LLC_ENC_WAIT_REQ,
    /// PAUSED state
    LLC_PAUSED,
    ///WAIT Session key
    LLC_WAIT_SK,
    /// DISCONNECTING state
    LLC_DISC,
    /// STOPPING state
    LLC_STOPPING,
    /// WAIT_ACK state
    LLCP_WAIT_ACK,

    /// Number of states.
    LLC_STATE_MAX
};


/*
 * MESSAGES
 ****************************************************************************************
 */
/// Message API of the LLC task
enum LLC_MSG
{
    /*
     * ************** Msg HCI->LLC****************
     */
    LLC_MSG_ID_CMD_FIRST = KE_FIRST_MSG(TASK_LLC),
    /// Legacy commands
    LLC_DISCONNECT_CMD = LLC_MSG_ID_CMD_FIRST,
    LLC_RD_REM_VER_INFO_CMD,
    LLC_FLUSH_CMD,
    LLC_RD_TX_PW_LVL_CMD,
    LLC_RD_RSSI_CMD,

    ///LE Commands
    LLC_LE_UPDATE_CON_REQ,
    LLC_LE_RD_CHNL_MAP_CMD,
    LLC_LE_RD_REM_USED_FEATS_CMD,
    LLC_LE_START_ENC_CMD,
    LLC_LE_LTK_REQ_RPLY_CMD,
    LLC_LE_LTK_REQ_NEG_RPLY_CMD,


    ///Data packet from HL
    LLC_LE_DATA_REQ,

    LLC_MSG_ID_EVT_FIRST,

    /*
     * ************** Msg LLC->HL****************
     */
    ///legacy event identifier
    LLC_DISCON_STAT_EVT = LLC_MSG_ID_EVT_FIRST,
    LLC_RD_REM_VER_INFO_STAT_EVT,
    LLC_FLUSH_CMP_EVT,
    LLC_RD_TX_PW_LVL_CMP_EVT,
    LLC_RD_RSSI_CMP_EVT,

    ///Special Legacy Event
    LLC_DISCON_CMP_EVT,
    LLC_RD_REM_VER_INFO_CMP_EVT,
    LLC_FLUSH_OCCUR_EVT,
    LLC_NB_PKT_CMP_EVT,
    LLC_DATA_BUF_OVER_FLOW_EVT,
    LLC_ENC_CHANGE_EVT,
    LLC_ENC_KEY_REFRESH_CMP_EVT,

    ///LE Events
    LLC_LE_UPDATE_CON_STAT_EVT,
    LLC_LE_RD_CHNL_MAP_CMP_EVT,
    LLC_LE_RD_REM_USED_FEATS_STAT_EVT,
    LLC_LE_START_ENC_STAT_EVT,
    LLC_LE_LTK_REQ_RPLY_CMP_EVT,
    LLC_LE_LTK_REQ_NEG_RPLY_CMP_EVT,

    ///low energy META event identifiers
    LLC_LE_UPDATE_CON_CMP_EVT,
    LLC_LE_RD_REM_USED_FEATS_CMP_EVT,
    LLC_LE_LTK_REQ_EVT,

    #if BLE_STD_MODE
    LLC_UNKNWN_CH_CMP_EVT,
    #endif //BLE_STD_MODE

    ///Data packet to HL
    LLC_LE_DATA_IND,

    LLC_MSG_ID_EVT_LAST,

    /*
     * ************** Msg LLC->LLC****************
     */
    LLC_LE_LINK_SUP_TO,
    LLC_LLCP_RSP_TO,
    LLC_LLCP_UNKNOWN_IND,
    LLC_LLCP_TX_CFM,
    LLC_VERSION_IND_SEND,
    LLC_UNKNOWN_RSP_SEND,

    /*
     * ************** LLCP messages **************
     */
    /// Connection update request
    LLCP_CONNECTION_UPDATE_REQ,
    /// Channel map request
    LLCP_CHANNEL_MAP_REQ,
    /// Termination indication
    LLCP_TERMINATE_IND,
    /// Encryption request
    LLCP_ENC_REQ,
    /// Encryption response
    LLCP_ENC_RSP,
    /// Start encryption request
    LLCP_START_ENC_REQ,
    /// Start encryption response
    LLCP_START_ENC_RSP,
    /// Unknown response
    LLCP_UNKNOWN_RSP,
    /// Feature request
    LLCP_FEATURE_REQ,
    /// Feature response
    LLCP_FEATURE_RSP,
    /// Pause encryption request
    LLCP_PAUSE_ENC_REQ,
    /// Pause encryption response
    LLCP_PAUSE_ENC_RSP,
    /// Version indication
    LLCP_VERSION_IND,
    /// Reject indication
    LLCP_REJECT_IND,
};

/*
 * ************** Local Defines****************
 */
/// type of tx power level
enum
{
    TX_LVL_CURRENT,
    TX_LVL_MAX,
    TX_LVL_LEN
};

/// different control state for the LLC
enum
{
    LLC_CNTL_STATE_IDLE,
    LLC_ENC_PAUSE_RESUME,
    LLC_ENC_START,
    LLC_UPDATE_CNX,
    LLC_CNTL_STATE_LEN

};
/*
 * ************** API low energy ****************
 */
///LLC LE Connection Update Command parameters structure
struct llc_le_con_update_cmd
{
    ///Connection Handle
    uint16_t       conhdl;
    ///Minimum of connection interval
    uint16_t       con_intv_min;
    ///Maximum of connection interval
    uint16_t       con_intv_max;
    ///Connection latency
    uint16_t       con_latency;
    ///Link supervision timeout
    uint16_t       superv_to;
    ///Minimum of CE length
    uint16_t       ce_len_min;
    ///Maximum of CE length
    uint16_t       ce_len_max;
};

///LLC LE Start Encryption Command parameters structure
struct llc_le_start_enc_cmd
{
    ///Connection handle
    uint16_t        conhdl;
    ///Random number - 8B
    struct rand_nb  nb;
    ///Encryption Diversifier
    uint16_t       enc_div;
    ///Long term key
    struct ltk     ltk;
};

///llc long term key request reply command parameters structure
struct llc_le_ltk_req_rply_cmd
{
    ///Connection handle
    uint16_t        conhdl;
    ///Long term key
    struct ltk      ltk;
};

///llc long term key request negative reply command parameters structure
struct llc_le_ltk_req_neg_rply_cmd
{
    uint16_t        conhdl;
};

///llc read channel map command parameters structure
struct llc_le_rd_ch_map_cmd
{
    ///Connection handle
    uint16_t        conhdl;
};

///llc command complete event structure for Read Channel Map Command
struct llc_le_rd_chnl_map_cmd_complete
{
    ///Status of command reception
    uint8_t            status;
    ///Connection handle
    uint16_t           conhdl;
    ///Channel map
    struct le_chnl_map ch_map;
};

///llc command complete event structure for Long Term Key Request Reply Command
struct llc_le_ltk_req_rply_cmd_complete
{
    ///Status of command reception
    uint8_t        status;
    ///Connection handle
    uint16_t       conhdl;
};

///llc command complete event structure for Long Term Key Request Negative Reply Command
struct llc_le_ltk_req_neg_rply_cmd_complete
{
    ///Status of command reception
    uint8_t        status;
    ///Connection handle
    uint16_t       conhdl;
};

/// llc le create connection request parameters structure description.
struct llc_create_con_req
{
    ///Connection handle
    uint16_t       conhdl;
    ///Scan interval
    uint16_t       scan_intv;
    ///Scan window size
    uint16_t       scan_window;
    ///Initiator filter policy
    uint8_t        init_filt_policy;
    ///Peer address type - 0=public/1=random
    uint8_t        peer_addr_type;
    ///Peer BD address
    struct bd_addr peer_addr;
    ///Own address type - 0=public/1=random
    uint8_t        own_addr_type;
    ///Minimum of connection interval
    uint16_t       con_intv_min;
    ///Maximum of connection interval
    uint16_t       con_intv_max;
    ///Connection latency
    uint16_t       con_latency;
    ///Link supervision timeout
    uint16_t       superv_to;
    ///Minimum CE length
    uint16_t       ce_len_min;
    ///Maximum CE length
    uint16_t       ce_len_max;
};

/// llc le create connection confirmation parameters structure description.
struct  llc_create_con_cfm
{
    /// status
    uint8_t         status;
    /// connection handle
    uint16_t        conhdl;
};

/// llc le create connection indication and cfm structure description.
struct  llc_create_con_req_ind
{
    /// connection interval, unit: 1.25ms
    uint16_t        con_int;
    /// connection latency, unit: 1.25ms
    uint16_t        con_lat;
    /// supervision time out, unit: 10ms
    uint16_t        sup_to;
    /// peer bd address
    struct bd_addr  peer_addr;
    /// peer bd address type
    uint8_t         peer_addr_type;
    /// hopping
    uint8_t         hop_inc;
    /// sleep accuracy
    uint8_t         sleep_clk_acc;
};

/// llc status event structure.
struct llc_event_common_cmd_status
{
    /// status
    uint8_t     status;
    /// connection handle
    uint16_t    conhdl;
};

/// llc complete event structure.
struct llc_event_common_cmd_complete
{
    /// status
    uint8_t     status;
    /// connection handle
    uint16_t    conhdl;
};

/// llc data rx packet structure
struct llc_data_packet_ind
{
    /// connection handle
    uint16_t    conhdl;
    /// broadcast and packet boundary flag
    uint8_t     pb_bc_flag;
    /// length of the data
    uint16_t    length;
    /// Handle of the descriptor containing RX Data
    uint8_t     rx_hdl;
};

/// llc data tx packet structure
struct llc_data_packet_req
{
    /// connection handle
    uint16_t    conhdl;
    /// broadcast and packet boundary flag
    uint8_t     pb_bc_flag;
    /// length of the data
    uint16_t    length;
    /// Pointer to the first descriptor containing RX Data
    struct co_buf_tx_node *desc;
};

/// llc disconnect command event  structure
struct llc_disc_req
{
    /// connection handle
    uint16_t    conhdl;
    /// reason
    uint8_t     reason;
};

/// llc disconnect complete event structure
struct llc_disc_evt_complete
{
    ///Status of received command
    uint8_t     status;
    ///Connection Handle
    uint16_t    conhdl;
    ///Reason for disconnection
    uint8_t     reason;
};

/// llc number of packet complete event structure
struct llc_nb_of_pkt_evt_complete
{
    /// number of handles
    uint8_t     nb_of_hdl;
    /// connection handle
    uint16_t    conhdl[1];
    /// number of completed packets
    uint16_t    nb_comp_pkt[1];
};

///llc data buffer overflow event structure
struct llc_data_buf_ovflw_evt
{
    ///Link type
    uint8_t link_type;
};

/*
 * ************** API META events ****************
 */
///llc command complete event structure for LLC LE Connection Update Command
struct llc_le_con_update_cmd_complete
{
    ///LE Subevent code
    uint8_t             subevent_code;
    ///Status of received command
    uint8_t             status;
    ///Connection handle
    uint16_t            conhdl;
    ///Connection interval value
    uint16_t            con_interval;
    ///Connection latency value
    uint16_t            con_latency;
    ///Supervision timeout
    uint16_t            sup_to;
};
///llc command complete event structure for create connection
struct llc_create_con_cmd_complete
{
    ///LE Subevent code
    uint8_t             subevent_code;
    ///Status of received command
    uint8_t             status;
    ///Connection handle
    uint16_t            conhdl;
    ///Device role - 0=Master/ 1=Slave
    uint8_t             role;
    ///Peer address type - 0=public/1=random
    uint8_t             peer_addr_type;
    ///Peer address
    struct bd_addr      peer_addr;
    ///Connection interval
    uint16_t            con_interval;
    ///Connection latency
    uint16_t            con_latency;
    ///Link supervision timeout
    uint16_t            sup_to;
    ///Master clock accuracy
    uint8_t             clk_accuracy;
};

///llc LE read remote used feature command parameters structure
struct llc_le_rd_rem_used_feats_cmd
{
    ///Connection handle
    uint16_t            conhdl;
};

///llc command complete event structure for LLC LE read remote used feature Command
struct llc_le_rd_rem_used_feats_cmd_complete
{
    ///LE Subevent code
    uint8_t             subevent_code;
    ///Status of received command
    uint8_t             status;
    ///Connection handle
    uint16_t            conhdl;
    ///Le Features used
    struct le_features  feats_used;
};

///llc command complete event structure for LLC LE read remote used feature Command
struct llc_le_ltk_req
{
    ///LE Subevent code
    uint8_t             subevent_code;
    ///Connection handle
    uint16_t            conhdl;
    ///Random number
    struct rand_nb      rand;
    ///Encryption diversifier
    uint16_t            ediv;
};
/*
 * ************** API legacy ****************
 */
///llc read rssi command parameters structure
struct llc_rd_rssi_cmd
{
    ///Connection handle
    uint16_t conhdl;
};

///llc command complete event structure for read rssi
struct llc_rd_rssi_cmd_complete
{
    ///Status for command reception
    uint8_t status;
    ///Connection handle
    uint16_t conhdl;
    ///RSSI value
    uint8_t rssi;
};

///llc flush command parameters structure
struct llc_flush_cmd
{
    ///Connection handle
    uint16_t conhdl;
};

///llc event structure for the flush occurred event
struct llc_flush_occurred_evt
{
    ///Connection handle
    uint16_t conhdl;
};

///llc command completed event structure for the flush command
struct llc_flush_cmd_complete
{
    ///Status for command reception
    uint8_t status;
    ///Connection handle
    uint16_t conhdl;
};
///llc command structure for the read transmit power level command
struct llc_rd_tx_pw_lvl_cmd
{
    ///Connection handle
    uint16_t    conhdl;
    ///Power Level type: current or maximum
    uint8_t     type;
};
///llc command complete event structure for the read transmit power level command
struct llc_rd_tx_pw_lvl_cmd_complete
{
    ///Status for command reception
    uint8_t status;
    ///Connection handle
    uint16_t conhdl;
    ///Value of TX power level
    uint8_t     tx_pow_lvl;
};

///llc read remote information version command parameters structure
struct llc_rd_rem_info_ver_cmd
{
    ///Connection handle
    uint16_t    conhdl;
};

///llc command complete event structure for the read remote information version command
struct llc_rd_rem_info_ver_cmd_complete
{
    ///Status for command reception
    uint8_t status;
    ///Connection handle
    uint16_t conhdl;
    ///LMP version
    uint8_t     vers;
    ///Manufacturer name
    uint16_t    compid;
    ///LMP subversion
    uint16_t    subvers;
};

///llc encryption change event structure
struct llc_enc_change_evt
{
    ///Status for command reception
    uint8_t status;
    ///Connection handle
    uint16_t conhdl;
    ///Encryption enabled information
    uint8_t     enc_stat;
};

///llc encryption key refresh complete event structure
struct llc_enc_key_ref_cmp_evt
{
    ///Status for command reception
    uint8_t status;
    ///Connection handle
    uint16_t conhdl;
};

///llc encryption key refresh event structure
struct llc_enc_key_refresh_evt
{
    ///Status for command reception
    uint8_t status;
    ///Connection handle
    uint16_t conhdl;
};

#if BLE_STD_MODE
/// Unknown connection handle command complete event with status and connection handle only.
struct llc_unknwn_conhdl_cmp_evt
{
    /// Opcode of the command
    uint16_t opcode;
    /// Status of the command completion
    uint8_t status;
    /// Connection handle
    uint16_t conhdl;
};
#endif //BLE_STD_MODE


///llc read rssi command parameters structure
struct llc_llcp_unknown_ind
{
    /// Unknown opcode
    uint8_t opcode;
};

///llc read rssi command parameters structure
struct llc_unknown_rsp_send
{
    /// Unknown opcode
    uint8_t opcode;
};

/*
 * ************** API LLCP ****************
 */

/// LL_VERS_IND structure
struct llcp_vers_ind
{
    /// opcode
    uint8_t     opcode;
    /// version
    uint8_t     vers;
    /// company id
    uint16_t    compid;
    /// sub version
    uint16_t    subvers;
};

/// LL_CONNECTION_UPDATE_REQ structure.
struct  llcp_con_up_req
{
    /// opcode
    uint8_t         opcode;
    /// window size
    uint8_t         win_size;
    /// window offset
    uint16_t        win_off;
    /// interval
    uint16_t        interv;
    /// latency
    uint16_t        latency;
    /// timeout
    uint16_t        timeout;
    /// instant
    uint16_t        instant;
};

/// LL_CHANNEL_MAP_REQ structure.
struct  llcp_channel_map_req
{
    /// opcode
    uint8_t            opcode;
    /// channel mapping
    struct le_chnl_map ch_map;
    /// instant
    uint16_t           instant;
};

/// LL_TERMINATE_IND structure.
struct  llcp_terminate_ind
{
    /// opcode
    uint8_t         opcode;
    /// termination code
    uint8_t         err_code;
};

/// LL_REJECT_IND structure.
struct  llcp_reject_ind
{
    /// opcode
    uint8_t         opcode;
    /// reject reason
    uint8_t         err_code;
};

/// LL_UNKNOWN_RSP structure.
struct  llcp_unknown_rsp
{
    /// opcode
    uint8_t         opcode;
    /// unknown type
    uint8_t         unk_type;
};
/// LL_PAUSE_ENC_REQ structure.
struct  llcp_pause_enc_req
{
    /// opcode
    uint8_t             opcode;
};

/// LL_PAUSE_ENC_RSP structure.
struct  llcp_pause_enc_rsp
{
    /// opcode
    uint8_t             opcode;
};

/// LL_START_ENC_REQ structure.
struct  llcp_start_enc_req
{
    /// opcode
    uint8_t             opcode;
};

/// LL_START_ENC_RSP structure.
struct  llcp_start_enc_rsp
{
    /// opcode
    uint8_t             opcode;
};
/// LL_ENC_REQ structure.
struct  llcp_enc_req
{
    /// opcode
    uint8_t             opcode;
    /// random value
    struct rand_nb      rand;
    /// ediv
    uint8_t             ediv[2];
    /// skdm
    struct sess_k_div_x   skdm;
    /// ivm
    struct init_vect    ivm;
};

/// LL_ENC_RSP structure.
struct  llcp_enc_rsp
{
    /// opcode
    uint8_t             opcode;
    /// skds
    struct sess_k_div_x   skds;
    /// ivs
    struct init_vect    ivs;
};

/// LL_FEATURE_REQ structure.
struct  llcp_feats_req
{
    /// opcode
    uint8_t             opcode;
    /// le features
    struct le_features  feats;
};

/// LL_FEATURE_RSP structure.
struct  llcp_feats_rsp
{
    /// opcode
    uint8_t             opcode;
    /// le features
    struct le_features  feats;
};

/*
 * TASK DESCRIPTOR DECLARATIONS
 ****************************************************************************************
 */
extern const struct ke_state_handler llc_state_handler[LLC_STATE_MAX];
extern const struct ke_state_handler llc_default_handler;
extern ke_state_t llc_state[LLC_IDX_MAX];

#endif // #if (BLE_PERIPHERAL || BLE_CENTRAL)
/// @} LLCTASK

#endif // LLC_TASK_H_
