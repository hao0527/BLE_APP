
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

#include "co_error.h"        // error definition
#include "co_utils.h"        // common utility definition
#include "co_buf.h"          // stack buffering
#include "co_bt.h"           // BT standard definitions

#include "hcic.h"             // hci definition
#include "hcic_eif.h"        // hci uart
#include "hcic_msg.h"         // hci message
#include "hcic_task.h"        // hci message
#include "llm.h"             // link layer manager definition
#include "llm_task.h"        // link layer manager task definition
#include "llc_task.h"        // link layer controller task definition
#include "dbg_task.h"        // debug task definition
#include "ke_msg.h"          // kernel message declaration
#include "ke_task.h"         // kernel task definition
#include "ke_event.h"        // kernel event definition
#include "ke_timer.h"        // kernel timer definition
#include "rwip.h"            // rw

/*
 * MACROS
 ****************************************************************************************
 */

#if (DEEP_SLEEP)
/// Period of sleep preventing when an HCI event is emitted
#define HCI_SLEEP_TIMEOUT                  4   // 4 * 10ms = 40ms (+/-10ms)
/// Period for polling the sleep status of HCI interface
#define HCI_SLEEP_POLLING_INTERVAL         2   // 2 * 10ms = 20ms (+/-10ms)
#endif //DEEPSLEEP

/// Macro to get OCF of a known command
#define OCF(cmd)        (HCI_OP2OCF(HCI_##cmd##_CMD_OPCODE))


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

///HCI environment context
struct hci_env_tag hci_env;

/// HCI task descriptor
static const struct ke_task_desc TASK_DESC_HCI = {NULL, &hci_default_handler, hci_state, HCI_STATE_MAX, HCI_IDX_MAX};

///HCI Table with handled command opcodes for check
const uint16_t hci_cmd_opcodes[HCI_CMD_OPCODE_NB_MAX]=
{
    //legacy commands
    HCI_DISCONNECT_CMD_OPCODE,
    HCI_RD_REM_VER_INFO_CMD_OPCODE,
    HCI_SET_EVT_MASK_CMD_OPCODE,
    HCI_RESET_CMD_OPCODE,
    HCI_FLUSH_CMD_OPCODE,
    HCI_RD_TX_PW_LVL_CMD_OPCODE,
    HCI_SET_CNTLR_TO_HOST_FLOW_CNTL_CMD_OPCODE,
    HCI_HOST_BUFFER_SIZE_CMD_OPCODE,
    HCI_HOST_NB_COMPLETED_PKTS_CMD_OPCODE,
    HCI_RD_LE_HOST_SUPP_CMD_OPCODE,
    HCI_WR_LE_HOST_SUPP_CMD_OPCODE,
    HCI_RD_LOCAL_VER_INFO_CMD_OPCODE,
    HCI_RD_LOCAL_SUPP_CMDS_CMD_OPCODE,
    HCI_RD_LOCAL_SUPP_FEATS_CMD_OPCODE,
    HCI_RD_BUFF_SIZE_CMD_OPCODE,
    HCI_RD_BD_ADDR_CMD_OPCODE,
    HCI_RD_RSSI_CMD_OPCODE,

    //LE commands
    HCI_LE_SET_EVT_MASK_CMD_OPCODE,
    HCI_LE_RD_BUFF_SIZE_CMD_OPCODE,
    HCI_LE_RD_LOCAL_SUPP_FEATS_CMD_OPCODE,
    HCI_LE_SET_RAND_ADDR_CMD_OPCODE,
    HCI_LE_SET_ADV_PARAM_CMD_OPCODE,
    HCI_LE_RD_ADV_CHNL_TX_PW_CMD_OPCODE,
    HCI_LE_SET_ADV_DATA_CMD_OPCODE,
    HCI_LE_SET_SCAN_RSP_DATA_CMD_OPCODE,
    HCI_LE_SET_ADV_EN_CMD_OPCODE,
    HCI_LE_SET_SCAN_PARAM_CMD_OPCODE,
    HCI_LE_SET_SCAN_EN_CMD_OPCODE,
    HCI_LE_CREATE_CON_CMD_OPCODE,
    HCI_LE_CREATE_CON_CANCEL_CMD_OPCODE,
    HCI_LE_RD_WLST_SIZE_CMD_OPCODE,
    HCI_LE_CLEAR_WLST_CMD_OPCODE,
    HCI_LE_ADD_DEV_TO_WLST_CMD_OPCODE,
    HCI_LE_RMV_DEV_FROM_WLST_CMD_OPCODE,
    HCI_LE_CON_UPDATE_CMD_OPCODE,
    HCI_LE_SET_HOST_CHNL_CLASSIF_CMD_OPCODE,
    HCI_LE_RD_CHNL_MAP_CMD_OPCODE,
    HCI_LE_RD_REM_USED_FEATS_CMD_OPCODE,
    HCI_LE_ENC_CMD_OPCODE,
    HCI_LE_RAND_CMD_OPCODE,
    HCI_LE_START_ENC_CMD_OPCODE,
    HCI_LE_LTK_REQ_RPLY_CMD_OPCODE,
    HCI_LE_LTK_REQ_NEG_RPLY_CMD_OPCODE,
    HCI_LE_RD_SUPP_STATES_CMD_OPCODE,
    HCI_LE_RX_TEST_CMD_OPCODE,
    HCI_LE_TX_TEST_CMD_OPCODE,
    HCI_LE_TEST_END_CMD_OPCODE,

    #if (BLE_DEBUG)
    //DBG commands
    HCI_DBG_RD_MEM_CMD_OPCODE,
    HCI_DBG_WR_MEM_CMD_OPCODE,
    HCI_DBG_DEL_PAR_CMD_OPCODE,
    HCI_DBG_FLASH_ID_CMD_OPCODE,
    HCI_DBG_FLASH_ER_CMD_OPCODE,
    HCI_DBG_FLASH_WR_CMD_OPCODE,
    HCI_DBG_FLASH_RD_CMD_OPCODE,
    HCI_DBG_RD_PAR_CMD_OPCODE,
    HCI_DBG_WR_PAR_CMD_OPCODE,
    #endif //BLE_DEBUG
    HCI_DBG_PLF_RESET_CMD_OPCODE,
    #if (BLE_DEBUG)
    HCI_DBG_RD_KE_STATS_CMD_OPCODE,
    HCI_DBG_HW_REG_RD_CMD_OPCODE,
    HCI_DBG_HW_REG_WR_CMD_OPCODE,
    HCI_DBG_SET_BD_ADDR_CMD_OPCODE,
    HCI_DBG_SET_TYPE_PUB_CMD_OPCODE,
    HCI_DBG_SET_TYPE_RAND_CMD_OPCODE,
    HCI_DBG_SET_CRC_CMD_OPCODE,
    HCI_DBG_LLCP_DISCARD_CMD_OPCODE,
    HCI_DBG_RESET_RX_CNT_CMD_OPCODE,
    HCI_DBG_RESET_TX_CNT_CMD_OPCODE,
    HCI_DBG_RF_REG_RD_CMD_OPCODE,
    HCI_DBG_RF_REG_WR_CMD_OPCODE,
    HCI_DBG_SET_TX_PW_CMD_OPCODE,
    #endif //BLE_DEBUG
    #if (DSM_SUPPORT)
    HCI_DBG_DATA_STORAGE_OP_CMD_OPCODE,
    #endif //DSM_SUPPORT
    #if (RW_BLE_WLAN_COEX)
    HCI_DBG_WLAN_COEX_CMD_OPCODE,
    #endif //RW_BT_WLAN_COEX
    #if (RW_BLE_WLAN_COEX_TEST)
    HCI_DBG_WLAN_COEXTST_SCEN_CMD_OPCODE,
    #endif //RW_BT_WLAN_COEX_TEST
    HCI_DBG_RD_MEM_INFO_CMD_OPCODE,
};


///HCI Table with event characteristics for pack - LLM Group
const struct hci_evt_pk_util hci_llm_epk[LLM_EVTID_MAX]=
{
    //Legacy events
    [LLM_EVTID2IDX(LLM_NO_OPERATIONS)] =
                                     {TYPE_CCEVT,HCI_CCEVT_BASIC_RETPAR_LEN,
                                      HCI_NO_OPERATION_CMD_OPCODE,
                                     (hci_evt_pk_func_t)hci_ccevt_aligned_pk},

    [LLM_EVTID2IDX(LLM_SET_EVT_MASK_CMP_EVT)] =
                                     {TYPE_CCEVT,HCI_CCEVT_BASIC_RETPAR_LEN,
                                      HCI_SET_EVT_MASK_CMD_OPCODE,
                                     (hci_evt_pk_func_t)hci_ccevt_aligned_pk},

    [LLM_EVTID2IDX(LLM_RESET_CMP_EVT)] =  {TYPE_CCEVT,HCI_CCEVT_BASIC_RETPAR_LEN,
                                      HCI_RESET_CMD_OPCODE,
                                     (hci_evt_pk_func_t)hci_ccevt_aligned_pk},

    [LLM_EVTID2IDX(LLM_SET_CNTLR_TO_HOST_FLOW_CNTL_CMP_EVT)] =
                                     {TYPE_CCEVT,HCI_CCEVT_BASIC_RETPAR_LEN,
                                      HCI_SET_CNTLR_TO_HOST_FLOW_CNTL_CMD_OPCODE,
                                     (hci_evt_pk_func_t)hci_ccevt_aligned_pk},

    [LLM_EVTID2IDX(LLM_HOST_BUFFER_SIZE_CMP_EVT)] =
                                     {TYPE_CCEVT,HCI_CCEVT_BASIC_RETPAR_LEN,
                                      HCI_HOST_BUFFER_SIZE_CMD_OPCODE,
                                     (hci_evt_pk_func_t)hci_ccevt_aligned_pk},

    [LLM_EVTID2IDX(LLM_HOST_NB_COMPLETED_PKTS_CMP_EVT)] =
                                     {TYPE_CCEVT,HCI_CCEVT_BASIC_RETPAR_LEN,
                                      HCI_HOST_NB_COMPLETED_PKTS_CMD_OPCODE,
                                     (hci_evt_pk_func_t)hci_ccevt_aligned_pk},

    [LLM_EVTID2IDX(LLM_RD_LOCAL_VER_INFO_CMP_EVT)] =
                                     {TYPE_CCEVT,HCI_CCEVT_RDLOCVER_RETPAR_LEN,
                                      HCI_RD_LOCAL_VER_INFO_CMD_OPCODE,
                                     (hci_evt_pk_func_t)hci_rd_local_ver_info_ccevt_pk},

    [LLM_EVTID2IDX(LLM_RD_LOCAL_SUPP_CMDS_CMP_EVT)] =
                                     {TYPE_CCEVT,HCI_CCEVT_RDLOCCMD_RETPAR_LEN,
                                      HCI_RD_LOCAL_SUPP_CMDS_CMD_OPCODE,
                                     (hci_evt_pk_func_t)hci_ccevt_aligned_pk},

    [LLM_EVTID2IDX(LLM_RD_LOCAL_SUPP_FEATS_CMP_EVT)] =
                                     {TYPE_CCEVT,HCI_CCEVT_RDLOCFEAT_RETPAR_LEN,
                                      HCI_RD_LOCAL_SUPP_FEATS_CMD_OPCODE,
                                     (hci_evt_pk_func_t)hci_ccevt_aligned_pk},
    [LLM_EVTID2IDX(LLM_RD_BUFF_SIZE_CMP_EVT)] =
                                     {TYPE_CCEVT,HCI_CCEVT_RDBUFSZ_RETPAR_LEN,
                                      HCI_RD_BUFF_SIZE_CMD_OPCODE,
                                     (hci_evt_pk_func_t)hci_rd_buff_size_ccevt_pk},

    [LLM_EVTID2IDX(LLM_RD_BD_ADDR_CMP_EVT)] =
                                     {TYPE_CCEVT,HCI_CCEVT_RDBDADDR_RETPAR_LEN,
                                      HCI_RD_BD_ADDR_CMD_OPCODE,
                                     (hci_evt_pk_func_t)hci_ccevt_aligned_pk},

    [LLM_EVTID2IDX(LLM_HW_ERROR_EVT)] =   {TYPE_OTHEREVT, HCI_HWERR_EVTPAR_LEN,
                                      0,
                                     (hci_evt_pk_func_t)hci_hw_error_evt_pk},


    //LE command complete LLM events
    [LLM_EVTID2IDX(LLM_LE_SET_EVT_MASK_CMP_EVT)]= {TYPE_CCEVT,HCI_CCEVT_BASIC_RETPAR_LEN,
                                              HCI_LE_SET_EVT_MASK_CMD_OPCODE,
                                             (hci_evt_pk_func_t)hci_ccevt_aligned_pk},
    [LLM_EVTID2IDX(LLM_LE_RD_BUFF_SIZE_CMP_EVT)] =
                                             {TYPE_CCEVT,HCI_CCEVT_LERDBFSZ_RETPAR_LEN,
                                              HCI_LE_RD_BUFF_SIZE_CMD_OPCODE,
                                         (hci_evt_pk_func_t)hci_le_rd_buff_size_ccevt_pk},

    [LLM_EVTID2IDX(LLM_LE_RD_LOCAL_SUPP_FEATS_CMP_EVT)] =
                                             {TYPE_CCEVT,HCI_CCEVT_LERDLOCFEAT_RETPAR_LEN,
                                              HCI_LE_RD_LOCAL_SUPP_FEATS_CMD_OPCODE,
                                             (hci_evt_pk_func_t)hci_ccevt_aligned_pk},

    [LLM_EVTID2IDX(LLM_LE_SET_RAND_ADDR_CMP_EVT)]={TYPE_CCEVT,HCI_CCEVT_BASIC_RETPAR_LEN,
                                              HCI_LE_SET_RAND_ADDR_CMD_OPCODE,
                                             (hci_evt_pk_func_t)hci_ccevt_aligned_pk},

    [LLM_EVTID2IDX(LLM_LE_SET_ADV_PARAM_CMP_EVT)]={TYPE_CCEVT,HCI_CCEVT_BASIC_RETPAR_LEN,
                                              HCI_LE_SET_ADV_PARAM_CMD_OPCODE,
                                             (hci_evt_pk_func_t)hci_ccevt_aligned_pk},

    [LLM_EVTID2IDX(LLM_LE_RD_ADV_CHNL_TX_PW_CMP_EVT)] =
                                             {TYPE_CCEVT,HCI_CCEVT_LERDADVTXPW_RETPAR_LEN,
                                              HCI_LE_RD_ADV_CHNL_TX_PW_CMD_OPCODE,
                                             (hci_evt_pk_func_t)hci_ccevt_aligned_pk},

    [LLM_EVTID2IDX(LLM_LE_SET_ADV_DATA_CMP_EVT)] ={TYPE_CCEVT,HCI_CCEVT_BASIC_RETPAR_LEN,
                                               HCI_LE_SET_ADV_DATA_CMD_OPCODE,
                                             (hci_evt_pk_func_t)hci_ccevt_aligned_pk},

    [LLM_EVTID2IDX(LLM_LE_SET_SCAN_RSP_DATA_CMP_EVT)]=
                                             {TYPE_CCEVT,HCI_CCEVT_BASIC_RETPAR_LEN,
                                              HCI_LE_SET_SCAN_RSP_DATA_CMD_OPCODE,
                                             (hci_evt_pk_func_t)hci_ccevt_aligned_pk},

    [LLM_EVTID2IDX(LLM_LE_SET_ADV_EN_CMP_EVT)] =  {TYPE_CCEVT,HCI_CCEVT_BASIC_RETPAR_LEN,
                                              HCI_LE_SET_ADV_EN_CMD_OPCODE,
                                             (hci_evt_pk_func_t)hci_ccevt_aligned_pk},

    [LLM_EVTID2IDX(LLM_LE_SET_SCAN_PARAM_CMP_EVT)] =
                                             {TYPE_CCEVT,HCI_CCEVT_BASIC_RETPAR_LEN,
                                              HCI_LE_SET_SCAN_PARAM_CMD_OPCODE,
                                             (hci_evt_pk_func_t)hci_ccevt_aligned_pk},

    [LLM_EVTID2IDX(LLM_LE_SET_SCAN_EN_CMP_EVT)] = {TYPE_CCEVT,HCI_CCEVT_BASIC_RETPAR_LEN,
                                              HCI_LE_SET_SCAN_EN_CMD_OPCODE,
                                             (hci_evt_pk_func_t)hci_ccevt_aligned_pk},

    [LLM_EVTID2IDX(LLM_LE_CREATE_CON_CANCEL_CMP_EVT)] =
                                             {TYPE_CCEVT,HCI_CCEVT_BASIC_RETPAR_LEN,
                                              HCI_LE_CREATE_CON_CANCEL_CMD_OPCODE,
                                             (hci_evt_pk_func_t)hci_ccevt_aligned_pk},

    [LLM_EVTID2IDX(LLM_LE_RD_WLST_SIZE_CMP_EVT)] ={TYPE_CCEVT,HCI_CCEVT_LERDWLSZ_RETPAR_LEN,
                                              HCI_LE_RD_WLST_SIZE_CMD_OPCODE,
                                             (hci_evt_pk_func_t)hci_ccevt_aligned_pk},

    [LLM_EVTID2IDX(LLM_LE_CLEAR_WLST_CMP_EVT)] =  {TYPE_CCEVT,HCI_CCEVT_BASIC_RETPAR_LEN,
                                              HCI_LE_CLEAR_WLST_CMD_OPCODE,
                                             (hci_evt_pk_func_t)hci_ccevt_aligned_pk},

    [LLM_EVTID2IDX(LLM_LE_ADD_DEV_TO_WLST_CMP_EVT)] =
                                             {TYPE_CCEVT,HCI_CCEVT_BASIC_RETPAR_LEN,
                                              HCI_LE_ADD_DEV_TO_WLST_CMD_OPCODE,
                                             (hci_evt_pk_func_t)hci_ccevt_aligned_pk},

    [LLM_EVTID2IDX(LLM_LE_RMV_DEV_FROM_WLST_CMP_EVT)]=
                                             {TYPE_CCEVT,HCI_CCEVT_BASIC_RETPAR_LEN,
                                              HCI_LE_RMV_DEV_FROM_WLST_CMD_OPCODE,
                                             (hci_evt_pk_func_t)hci_ccevt_aligned_pk},

    [LLM_EVTID2IDX(LLM_LE_SET_HOST_CHNL_CLASSIF_CMP_EVT)] =
                                             {TYPE_CCEVT,HCI_CCEVT_BASIC_RETPAR_LEN,
                                              HCI_LE_SET_HOST_CHNL_CLASSIF_CMD_OPCODE,
                                             (hci_evt_pk_func_t)hci_ccevt_aligned_pk},

    [LLM_EVTID2IDX(LLM_LE_ENC_CMP_EVT)] =         {TYPE_CCEVT,HCI_CCEVT_LEENC_RETPAR_LEN,
                                              HCI_LE_ENC_CMD_OPCODE,
                                             (hci_evt_pk_func_t)hci_ccevt_aligned_pk},

    [LLM_EVTID2IDX(LLM_LE_RAND_CMP_EVT)] =        {TYPE_CCEVT,HCI_CCEVT_LERAND_RETPAR_LEN,
                                              HCI_LE_RAND_CMD_OPCODE,
                                             (hci_evt_pk_func_t)hci_ccevt_aligned_pk},

    [LLM_EVTID2IDX(LLM_LE_RD_SUPP_STATES_CMP_EVT)] =
                                             {TYPE_CCEVT,HCI_CCEVT_LERDSUPST_RETPAR_LEN,
                                              HCI_LE_RD_SUPP_STATES_CMD_OPCODE,
                                             (hci_evt_pk_func_t)hci_ccevt_aligned_pk},

    [LLM_EVTID2IDX(LLM_LE_TEST_RX_CMP_EVT)] =     {TYPE_CCEVT,HCI_CCEVT_BASIC_RETPAR_LEN,
                                              HCI_LE_RX_TEST_CMD_OPCODE,
                                             (hci_evt_pk_func_t)hci_ccevt_aligned_pk},

    [LLM_EVTID2IDX(LLM_LE_TEST_TX_CMP_EVT)] =     {TYPE_CCEVT,HCI_CCEVT_BASIC_RETPAR_LEN,
                                              HCI_LE_TX_TEST_CMD_OPCODE,
                                             (hci_evt_pk_func_t)hci_ccevt_aligned_pk},

    [LLM_EVTID2IDX(LLM_LE_TEST_END_CMP_EVT)] =    {TYPE_CCEVT,HCI_CCEVT_LETSTEND_RETPAR_LEN,
                                              HCI_LE_TEST_END_CMD_OPCODE,
                                             (hci_evt_pk_func_t)hci_le_test_end_ccevt_pk},


    //LE Command status events
    [LLM_EVTID2IDX(LLM_LE_CREATE_CON_STAT_EVT)] = {TYPE_CSEVT,0,
                                              HCI_LE_CREATE_CON_CMD_OPCODE,
                                             (hci_evt_pk_func_t)hci_csevt_pk},

    // Special events
    #if (BLE_CENTRAL || BLE_PERIPHERAL)
    [LLM_EVTID2IDX(LLM_LE_CREATE_CON_CMP_EVT)] =  {TYPE_OTHEREVT,0,
                                              0,(hci_evt_pk_func_t)hci_le_con_cmp_evt_pk},
    #endif // #if (BLE_CENTRAL || BLE_PERIPHERAL)
    [LLM_EVTID2IDX(LLM_LE_ADV_REPORT_EVT)] =   {TYPE_OTHEREVT,0,
                                           0,(hci_evt_pk_func_t)hci_le_adv_report_evt_pk},

};

#if (BLE_CENTRAL || BLE_PERIPHERAL)
///HCI Table with event characteristics for pack - LLC Group
const struct hci_evt_pk_util hci_llc_epk[LLC_EVTID_MAX]=
{
    //events to legacy commands or just legacy events
    [LLC_EVTID2IDX(LLC_DISCON_STAT_EVT)] =
                                    {TYPE_CSEVT,0,
                                     HCI_DISCONNECT_CMD_OPCODE,
                                    (hci_evt_pk_func_t)hci_csevt_pk},

    [LLC_EVTID2IDX(LLC_DISCON_CMP_EVT)] =
                                  {TYPE_OTHEREVT,0,
                                  0,(hci_evt_pk_func_t)hci_disconnection_cmp_evt_pk},

    [LLC_EVTID2IDX(LLC_RD_REM_VER_INFO_STAT_EVT)] =
                                  {TYPE_CSEVT,0,
                                   HCI_RD_REM_VER_INFO_CMD_OPCODE,
                                  (hci_evt_pk_func_t)hci_csevt_pk},

    [LLC_EVTID2IDX(LLC_RD_REM_VER_INFO_CMP_EVT)] =
                                  {TYPE_OTHEREVT,0,
                                  HCI_RD_REM_VER_INFO_CMD_OPCODE,
                                  (hci_evt_pk_func_t)hci_rd_rem_ver_info_cmp_evt_pk},

    [LLC_EVTID2IDX(LLC_FLUSH_OCCUR_EVT)] =
                                  {TYPE_OTHEREVT,0,
                                   0,(hci_evt_pk_func_t)hci_flush_occurred_evt_pk},

    [LLC_EVTID2IDX(LLC_FLUSH_CMP_EVT)] =
                                  {TYPE_CCEVT,HCI_CCEVT_FLUSH_RETPAR_LEN,
                                   HCI_FLUSH_CMD_OPCODE,
                                  (hci_evt_pk_func_t)hci_flush_ccevt_pk},

    [LLC_EVTID2IDX(LLC_RD_TX_PW_LVL_CMP_EVT)] =
                                  {TYPE_CCEVT,HCI_CCEVT_RDTXPW_RETPAR_LEN,
                                   HCI_RD_TX_PW_LVL_CMD_OPCODE,
                                  (hci_evt_pk_func_t)hci_rd_tx_pw_lvl_ccevt_pk},
    [LLC_EVTID2IDX(LLC_RD_RSSI_CMP_EVT)] =
                                  {TYPE_CCEVT,HCI_CCEVT_RDRSSI_RETPAR_LEN,
                                   HCI_RD_RSSI_CMD_OPCODE,
                                  (hci_evt_pk_func_t)hci_rd_rssi_ccevt_pk},

    [LLC_EVTID2IDX(LLC_DATA_BUF_OVER_FLOW_EVT)] =
                                  {TYPE_OTHEREVT,HCI_DBOVFLW_EVTPAR_LEN,
                                   0,
                                  (hci_evt_pk_func_t)hci_data_buf_ovflw_evt_pk},

    [LLC_EVTID2IDX(LLC_NB_PKT_CMP_EVT)] =
                                  {TYPE_OTHEREVT,0,
                                   0,
                                  (hci_evt_pk_func_t)hci_nb_cmp_pkts_evt_pk},

    //events to le commands or just le events
    [LLC_EVTID2IDX(LLC_LE_UPDATE_CON_STAT_EVT)] =
                                  {TYPE_CSEVT,0,
                                   HCI_LE_CON_UPDATE_CMD_OPCODE,
                                  (hci_evt_pk_func_t)hci_csevt_pk},

    [LLC_EVTID2IDX(LLC_LE_UPDATE_CON_CMP_EVT)] =
                                  {TYPE_OTHEREVT,0,
                                   HCI_LE_CON_UPDATE_CMD_OPCODE,
                                  (hci_evt_pk_func_t)hci_le_con_update_cmp_evt_pk},

    [LLC_EVTID2IDX(LLC_LE_RD_CHNL_MAP_CMP_EVT)]  =
                                  {TYPE_CCEVT,HCI_CCEVT_LERDCHMAP_RETPAR_LEN,
                                   HCI_LE_RD_CHNL_MAP_CMD_OPCODE,
                                  (hci_evt_pk_func_t)hci_le_rd_chnl_map_ccevt_pk},

    [LLC_EVTID2IDX(LLC_LE_RD_REM_USED_FEATS_STAT_EVT)] =
                                  {TYPE_CSEVT,0,
                                   HCI_LE_RD_REM_USED_FEATS_CMD_OPCODE,
                                  (hci_evt_pk_func_t)hci_csevt_pk},

    [LLC_EVTID2IDX(LLC_LE_RD_REM_USED_FEATS_CMP_EVT)] =
                                  {TYPE_OTHEREVT,HCI_CCEVT_LERDREMFEAT_RETPAR_LEN,
                                   HCI_LE_RD_REM_USED_FEATS_CMD_OPCODE,
                                  (hci_evt_pk_func_t)hci_le_rd_rem_used_feats_cmp_evt_pk},

    [LLC_EVTID2IDX(LLC_LE_START_ENC_STAT_EVT)]         =
                                  {TYPE_CSEVT,0,
                                   HCI_LE_START_ENC_CMD_OPCODE,
                                  (hci_evt_pk_func_t)hci_csevt_pk},

    [LLC_EVTID2IDX(LLC_ENC_CHANGE_EVT)]         =
                                  {TYPE_OTHEREVT,HCI_ENCCHG_EVTPAR_LEN,
                                   HCI_LE_START_ENC_CMD_OPCODE,
                                  (hci_evt_pk_func_t)hci_enc_change_evt_pk},

    [LLC_EVTID2IDX(LLC_ENC_KEY_REFRESH_CMP_EVT)]         =
                                  {TYPE_OTHEREVT,0,
                                   HCI_LE_START_ENC_CMD_OPCODE,
                                  (hci_evt_pk_func_t)hci_enc_key_refresh_evt_pk},

    [LLC_EVTID2IDX(LLC_LE_LTK_REQ_RPLY_CMP_EVT)] =
                                  {TYPE_CCEVT,HCI_CCEVT_LELTKRR_RETPAR_LEN,
                                   HCI_LE_LTK_REQ_RPLY_CMD_OPCODE,
                                  (hci_evt_pk_func_t)hci_le_ltk_req_rply_ccevt_pk},

    [LLC_EVTID2IDX(LLC_LE_LTK_REQ_NEG_RPLY_CMP_EVT)] =
                                  {TYPE_CCEVT,HCI_CCEVT_LELTKRNR_RETPAR_LEN,
                                   HCI_LE_LTK_REQ_NEG_RPLY_CMD_OPCODE,
                                  (hci_evt_pk_func_t)hci_le_ltk_req_neg_rply_ccevt_pk},

    [LLC_EVTID2IDX(LLC_LE_LTK_REQ_EVT)] =
                                  {TYPE_OTHEREVT,HCI_LELTKREQ_EVTPAR_LEN,
                                   0,
                                  (hci_evt_pk_func_t)hci_le_ltk_req_evt_pk},

    #if BLE_STD_MODE
    // Generic unknown connection handle complete event
    [LLC_EVTID2IDX(LLC_UNKNWN_CH_CMP_EVT)] = {TYPE_CCEVT, HCI_CCEVT_BASIC_RETPAR_LEN,
                                   0, (hci_evt_pk_func_t)hci_unknwn_conhdl_ccevt_pk},
    #endif //BLE_STD_MODE
};
#endif // BLE_CENTRAL || BLE_PERIPHERAL

#if (BLE_DEBUG)
const struct hci_evt_pk_util hci_dbg_epk[DBG_EVTID_MAX]=
{
    [DBG_EVTID2IDX(DBG_RD_MEM_CMP_EVT)] =
                                {TYPE_CCEVT, 0, 0,
                                (hci_evt_pk_func_t)hci_dbg_rd_mem_ccevt_pk},

    [DBG_EVTID2IDX(DBG_WR_MEM_CMP_EVT)] =
                                {TYPE_CCEVT, HCI_CCEVT_BASIC_RETPAR_LEN,
                                        HCI_DBG_WR_MEM_CMD_OPCODE,
                                (hci_evt_pk_func_t)hci_ccevt_aligned_pk},

    [DBG_EVTID2IDX(DBG_DEL_PARAM_CMP_EVT)] =
                                {TYPE_CCEVT, HCI_CCEVT_BASIC_RETPAR_LEN,
                                 HCI_DBG_DEL_PAR_CMD_OPCODE,
                                (hci_evt_pk_func_t)hci_ccevt_aligned_pk},

    [DBG_EVTID2IDX(DBG_FLASH_IDENT_CMP_EVT)] =
                                {TYPE_CCEVT, HCI_CCEVT_DBGFLASHID_RETPAR_LEN,
                                HCI_DBG_FLASH_ID_CMD_OPCODE,
                                (hci_evt_pk_func_t)hci_ccevt_aligned_pk},

    [DBG_EVTID2IDX(DBG_FLASH_ERASE_CMP_EVT)] =
                                {TYPE_CCEVT, HCI_CCEVT_BASIC_RETPAR_LEN,
                                HCI_DBG_FLASH_ER_CMD_OPCODE,
                                (hci_evt_pk_func_t)hci_ccevt_aligned_pk},

    [DBG_EVTID2IDX(DBG_FLASH_WRITE_CMP_EVT)] =
                                {TYPE_CCEVT, HCI_CCEVT_BASIC_RETPAR_LEN,
                                HCI_DBG_FLASH_WR_CMD_OPCODE,
                                (hci_evt_pk_func_t)hci_ccevt_aligned_pk},

    [DBG_EVTID2IDX(DBG_FLASH_READ_CMP_EVT)] =
                                {TYPE_CCEVT, 0, 0,
                                (hci_evt_pk_func_t)hci_dbg_flash_rd_evt_pk},

    [DBG_EVTID2IDX(DBG_RD_PARAM_CMP_EVT)] =
                                {TYPE_CCEVT, 0, 0,
                                (hci_evt_pk_func_t)hci_dbg_rd_par_ccevt_pk},

    [DBG_EVTID2IDX(DBG_WR_PARAM_CMP_EVT)] =
                                {TYPE_CCEVT, HCI_CCEVT_BASIC_RETPAR_LEN,
                                HCI_DBG_WR_PAR_CMD_OPCODE,
                                (hci_evt_pk_func_t)hci_ccevt_aligned_pk},

    [DBG_EVTID2IDX(DBG_RD_KE_STATS_CMP_EVT)] =
                                {TYPE_CCEVT, HCI_CCEVT_DBGRDKESTATS_RETPAR_LEN,
                                    HCI_DBG_RD_KE_STATS_CMD_OPCODE,
                                (hci_evt_pk_func_t)hci_ccevt_aligned_pk},
    [DBG_EVTID2IDX(DBG_RD_MEM_INFO_CMP_EVT)] =
                                {TYPE_CCEVT, HCI_CCEVT_DBGRDMEMINFO_RETPAR_LEN,
                                    HCI_DBG_RD_MEM_INFO_CMD_OPCODE,
                                (hci_evt_pk_func_t)hci_dbg_rd_mem_info_ccevt_pk},
    #if (DSM_SUPPORT)
    [DBG_EVTID2IDX(DBG_DATA_STORAGE_OP_CS_EVT)] =
                                {TYPE_CSEVT, 0, HCI_DBG_DATA_STORAGE_OP_CMD_OPCODE, &hci_csevt_pk},
    [DBG_EVTID2IDX(DBG_DATA_STORAGE_IND_EVT)] =
                                {TYPE_OTHEREVT, 0, 0, &hci_dbg_data_storage_ind_evt_pk},
    #endif //DSM_SUPPORT
    [DBG_EVTID2IDX(DBG_HW_REG_RD_CMP_EVT)] =
                                  {TYPE_CCEVT,HCI_CCEVT_DBGHWREGRD_RETPAR_LEN,
                                  HCI_DBG_HW_REG_RD_CMD_OPCODE,
                                  (hci_evt_pk_func_t)hci_dbg_hw_reg_rd_ccevt_pk},

    [DBG_EVTID2IDX(DBG_HW_REG_WR_CMP_EVT)] =
                                  {TYPE_CCEVT,HCI_CCEVT_DBGHWREGWR_RETPAR_LEN,
                                  HCI_DBG_HW_REG_WR_CMD_OPCODE,
                                  (hci_evt_pk_func_t)hci_dbg_hw_reg_wr_ccevt_pk},

    [DBG_EVTID2IDX(DBG_LE_SET_BD_ADDR_CMP_EVT)] =
                                  {TYPE_CCEVT,HCI_CCEVT_BASIC_RETPAR_LEN,
                                   HCI_DBG_SET_BD_ADDR_CMD_OPCODE,
                                  (hci_evt_pk_func_t)hci_ccevt_aligned_pk},

    [DBG_EVTID2IDX(DBG_LE_SET_TYPE_PUB_CMP_EVT)] =
                                  {TYPE_CCEVT, HCI_CCEVT_BASIC_RETPAR_LEN,
                                   HCI_DBG_SET_TYPE_PUB_CMD_OPCODE,
                                  (hci_evt_pk_func_t)hci_ccevt_aligned_pk},

    [DBG_EVTID2IDX(DBG_LE_SET_TYPE_RAND_CMP_EVT)] =
                                  {TYPE_CCEVT, HCI_CCEVT_BASIC_RETPAR_LEN,
                                   HCI_DBG_SET_TYPE_RAND_CMD_OPCODE,
                                  (hci_evt_pk_func_t)hci_ccevt_aligned_pk},

    [DBG_EVTID2IDX(DBG_LE_SET_CRC_CMP_EVT)] =
                                  {TYPE_CCEVT, HCI_CCEVT_BASIC_RETPAR_LEN,
                                   HCI_DBG_SET_CRC_CMD_OPCODE,
                                  (hci_evt_pk_func_t)hci_ccevt_aligned_pk},

    [DBG_EVTID2IDX(DBG_LE_LLCP_DISCARD_CMP_EVT)] =
                                  {TYPE_CCEVT, HCI_CCEVT_BASIC_RETPAR_LEN,
                                  HCI_DBG_LLCP_DISCARD_CMD_OPCODE,
                                  (hci_evt_pk_func_t)hci_ccevt_aligned_pk},

    [DBG_EVTID2IDX(DBG_LE_RESET_RX_CNT_CMP_EVT)] =
                                  {TYPE_CCEVT, HCI_CCEVT_BASIC_RETPAR_LEN,
                                  HCI_DBG_RESET_RX_CNT_CMD_OPCODE,
                                  (hci_evt_pk_func_t)hci_ccevt_aligned_pk},

    [DBG_EVTID2IDX(DBG_LE_RESET_TX_CNT_CMP_EVT)] =
                                  {TYPE_CCEVT, HCI_CCEVT_BASIC_RETPAR_LEN,
                                  HCI_DBG_RESET_TX_CNT_CMD_OPCODE,
                                  (hci_evt_pk_func_t)hci_ccevt_aligned_pk},

    [DBG_EVTID2IDX(DBG_RF_REG_RD_CMP_EVT)] =
                                {TYPE_CCEVT,HCI_CCEVT_DBGRFREGRD_RETPAR_LEN,
                                HCI_DBG_RF_REG_RD_CMD_OPCODE,
                                (hci_evt_pk_func_t)hci_dbg_rf_reg_rd_ccevt_pk},

    [DBG_EVTID2IDX(DBG_RF_REG_WR_CMP_EVT)] =
                                {TYPE_CCEVT,HCI_CCEVT_DBGRFREGWR_RETPAR_LEN,
                                HCI_DBG_RF_REG_WR_CMD_OPCODE,
                                (hci_evt_pk_func_t)hci_dbg_rf_reg_wr_ccevt_pk},
    [DBG_EVTID2IDX(DBG_SET_TX_PW_CMP_EVT)] =
                                {TYPE_CCEVT,HCI_CCEVT_BASIC_RETPAR_LEN,
                                HCI_DBG_SET_TX_PW_CMD_OPCODE,
                                (hci_evt_pk_func_t)hci_ccevt_aligned_pk},
    [DBG_EVTID2IDX(DBG_PLF_RESET_CMP_EVT)] =
                                {TYPE_CCEVT, HCI_CCEVT_BASIC_RETPAR_LEN,
                                HCI_DBG_PLF_RESET_CMD_OPCODE,
                                (hci_evt_pk_func_t)hci_ccevt_aligned_pk},
    #if (RW_BLE_WLAN_COEX)
    [DBG_EVTID2IDX(DBG_WLAN_COEX_CMP_EVT)] =
                                {TYPE_CCEVT, HCI_CCEVT_BASIC_RETPAR_LEN,
                                HCI_DBG_WLAN_COEX_CMD_OPCODE,
                                (hci_evt_pk_func_t)hci_ccevt_aligned_pk},
    #endif // RW_BLE_WLAN_COEX
};
#endif //BLE_DEBUG


/// Table storing parameter size for testing commands
static const uint8_t hci_le_cmd_par_size[HCI_OP2OCF(HCI_LE_OPCODE_MAX)] =
{
    [OCF(LE_SET_EVT_MASK)-1]          = HCI_LESETEMSK_CMD_PARLEN       ,
    [OCF(LE_RD_BUFF_SIZE)-1]          = HCI_LERDBSZ_CMD_PARLEN         ,
    [OCF(LE_RD_LOCAL_SUPP_FEATS)-1]   = HCI_LERDLOCFEAT_CMD_PARLEN     ,
    [OCF(LE_SET_RAND_ADDR)-1]         = HCI_LESETRANDA_CMD_PARLEN      ,
    [OCF(LE_SET_ADV_PARAM)-1]         = HCI_LESETADVP_CMD_PARLEN       ,
    [OCF(LE_RD_ADV_CHNL_TX_PW)-1]     = HCI_LERDADVPW_CMD_PARLEN       ,
    [OCF(LE_SET_ADV_DATA)-1]          = HCI_LESETADVDATA_CMD_PARLEN    ,
    [OCF(LE_SET_SCAN_RSP_DATA)-1]     = HCI_LESETSCANRSPDATA_CMD_PARLEN,
    [OCF(LE_SET_ADV_EN)-1]            = HCI_LESETADVEN_CMD_PARLEN      ,
    [OCF(LE_SET_SCAN_PARAM)-1]        = HCI_LESETSCANP_CMD_PARLEN      ,
    [OCF(LE_SET_SCAN_EN)-1]           = HCI_LESETSCANEN_CMD_PARLEN     ,
    [OCF(LE_CREATE_CON)-1]            = HCI_LECRCON_CMD_PARLEN         ,
    [OCF(LE_CREATE_CON_CANCEL)-1]     = HCI_LECRCONCL_CMD_PARLEN       ,
    [OCF(LE_RD_WLST_SIZE)-1]          = HCI_LERDWLSZ_CMD_PARLEN        ,
    [OCF(LE_CLEAR_WLST)-1]            = HCI_LECLWL_CMD_PARLEN          ,
    [OCF(LE_ADD_DEV_TO_WLST)-1]       = HCI_LEADDWL_CMD_PARLEN         ,
    [OCF(LE_RMV_DEV_FROM_WLST)-1]     = HCI_LEREMWL_CMD_PARLEN         ,
    [OCF(LE_CON_UPDATE)-1]            = HCI_LECONUPD_CMD_PARLEN        ,
    [OCF(LE_SET_HOST_CHNL_CLASSIF)-1] = HCI_LESETHCHCLS_CMD_PARLEN     ,
    [OCF(LE_RD_CHNL_MAP)-1]           = HCI_LERDCHMP_CMD_PARLEN        ,
    [OCF(LE_RD_REM_USED_FEATS)-1]     = HCI_LERDREMUFEAT_CMD_PARLEN    ,
    [OCF(LE_ENC)-1]                   = HCI_LEENC_CMD_PARLEN           ,
    [OCF(LE_RAND)-1]                  = HCI_LERAND_CMD_PARLEN          ,
    [OCF(LE_START_ENC)-1]             = HCI_LESTENC_CMD_PARLEN         ,
    [OCF(LE_LTK_REQ_RPLY)-1]          = HCI_LELTKRR_CMD_PARLEN         ,
    [OCF(LE_LTK_REQ_NEG_RPLY)-1]      = HCI_LELTKRNR_CMD_PARLEN        ,
    [OCF(LE_RD_SUPP_STATES)-1]        = HCI_LERDSUPST_CMD_PARLEN       ,
    [OCF(LE_RX_TEST)-1]               = HCI_LERXTST_CMD_PARLEN         ,
    [OCF(LE_TX_TEST)-1]               = HCI_LETXTST_CMD_PARLEN         ,
    [OCF(LE_TEST_END)-1]              = HCI_LETSTEND_CMD_PARLEN
};

/// Table storing parameter size for debug commands (Note: dbg ocf start at 0)
static const uint8_t hci_dbg_cmd_par_size[HCI_OP2OCF(HCI_DBG_OPCODE_MAX)] =
{
    #if (BLE_DEBUG)
    [OCF(DBG_RD_MEM)-1]        = HCI_DBGRDMEM_CMD_PARLEN,
    [OCF(DBG_WR_MEM)-1]        = HCI_DBGWRMEM_CMD_PARLEN,
    [OCF(DBG_DEL_PAR)-1]       = HCI_DBGDELPAR_CMD_PARLEN,
    [OCF(DBG_FLASH_ID)-1]      = HCI_DBGFLASHID_CMD_PARLEN,
    [OCF(DBG_FLASH_ER)-1]      = HCI_DBGFLASHER_CMD_PARLEN,
    [OCF(DBG_FLASH_WR)-1]      = HCI_DBGFLASHWR_CMD_PARLEN,
    [OCF(DBG_FLASH_RD)-1]      = HCI_DBGFLASHRD_CMD_PARLEN,
    [OCF(DBG_RD_PAR)-1]        = HCI_DBGRDPAR_CMD_PARLEN,
    [OCF(DBG_WR_PAR)-1]        = HCI_DBGWRPAR_CMD_PARLEN,
    #endif //BLE_DEBUG
    [OCF(DBG_PLF_RESET)-1]     = HCI_DBGPLFRESET_CMD_PARLEN,
    #if (BLE_DEBUG)
    [OCF(DBG_RD_KE_STATS)-1]   = HCI_DBGRDKESTATS_CMD_PARLEN,
    [OCF(DBG_HW_REG_RD)-1]     = HCI_DBGHWREGRD_CMD_PARLEN,
    [OCF(DBG_HW_REG_WR)-1]     = HCI_DBGHWREGWR_CMD_PARLEN,
    [OCF(DBG_SET_BD_ADDR)-1]   = HCI_DBGSETBD_CMD_PARLEN,
    [OCF(DBG_SET_TYPE_PUB)-1]  = HCI_DBGSETPUB_CMD_PARLEN,
    [OCF(DBG_SET_TYPE_RAND)-1] = HCI_DBGSETRAND_CMD_PARLEN,
    [OCF(DBG_SET_CRC)-1]       = HCI_DBGSETCRC_CMD_PARLEN,
    [OCF(DBG_LLCP_DISCARD)-1]  = HCI_DBGLLCPDISC_CMD_PARLEN,
    [OCF(DBG_RESET_RX_CNT)-1]  = HCI_DBGRSTRXCNT_CMD_PARLEN,
    [OCF(DBG_RESET_TX_CNT)-1]  = HCI_DBGRSTTXCNT_CMD_PARLEN,
    [OCF(DBG_RF_REG_RD)-1]     = HCI_DBGRFREGRD_CMD_PARLEN,
    [OCF(DBG_RF_REG_WR)-1]     = HCI_DBGRFREGWR_CMD_PARLEN,
    [OCF(DBG_SET_TX_PW)-1]     = HCI_DBGSETTXPW_CMD_PARLEN,
    #endif //BLE_DEBUG
    #if (DSM_SUPPORT)
    [OCF(DBG_DATA_STORAGE_OP)-1] = HCI_DBGDATASTOROP_CMD_PARLEN,
    #endif //DSM_SUPPORT
    [OCF(DBG_RD_MEM_INFO)-1]   = HCI_DBGRDMEMINFO_CMD_PARLEN,
};



/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @brief Function called after sending message through UART, to free ke_msg space and
 * push the next message for transmission if any.
 *
 * The message is popped from the tx queue kept in hci_env and freed using ke_msg_free.
 *****************************************************************************************
 */
static void hci_tx_done(void)
{
    struct ke_msg * msg;

    // Clear the event
    ke_event_clear(KE_EVENT_HCI_TX_DONE);

    // Go back to IDLE state
    hci_env.tx_state = HCI_STATE_TX_IDLE;

    //release current message (which was just sent)
    msg = (struct ke_msg *)co_list_pop_front(&hci_env.queue_tx);

    // Free the message resources
    if (msg->hci_type == HCI_ACL_MSG_TYPE)
    {
        #if (BLE_CENTRAL || BLE_PERIPHERAL)
        ///@todo Implement host case
        struct llc_data_packet_ind *data_ind = (struct llc_data_packet_ind *)(msg->param);

        // Free the RX buffer associated with the message
        co_buf_rx_free(data_ind->rx_hdl);
        #endif // BLE_CENTRAL || BLE_PERIPHERAL
    }
    #if (DEEP_SLEEP)
    else
    {
        // Prevent from going to deep sleep, in case Host has other commands to send
        if(rwip_sleep_enable() && !rwip_ext_wakeup_enable())
        {
            // Prevent from going to sleep until next polling time
            rwip_prevent_sleep_set(RW_HCI_TIMEOUT);

            // Program the unblocking of HCI interface
            ke_timer_set(HCI_POLLING_TO, TASK_HCI, HCI_SLEEP_TIMEOUT);
        }
    }
    #endif //DEEP_SLEEP

    // Free the kernel message space
    ke_msg_free(msg);

    // Check if there is a new message pending for transmission
    if ((msg = (struct ke_msg *)co_list_pick(&hci_env.queue_tx)) != NULL)
    {
        // Forward the message to the HCIH EIF for immediate transmission
        hci_eif_write(msg);
    }
}


/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void hci_init(const struct rwip_eif_api* eif)
{
    // register interface.
    hci_env.ext_if = eif;

    // Create HCI Task
    ke_task_create(TASK_HCI, &TASK_DESC_HCI);

    // Initialize HCI task to idle state
    ke_state_set(TASK_HCI, HCI_IDLE);

    //init uart hci
    hci_eif_init();

    //init TX queue
    co_list_init(&hci_env.queue_tx);

    //init the hci environment context
    hci_env.curr_payl_buff = NULL;

    hci_env.nb_h2c_cmd_pkts = HCI_NB_CMD_PKTS;

    // Register HCI TX kernel event
    ke_event_callback_set(KE_EVENT_HCI_TX_DONE, &hci_tx_done);
}


/*
 * FUNCTION DEFINITIONS - command reception in HCI
 ****************************************************************************************
 */
/**
 *****************************************************************************************
 *@brief Static function handling immediate CCEVT sending back to host, with status
 *       of unknonwn command received.
 *
 *       Necessary at UART reception, when opcode of a command is not recognized.
 *
 *****************************************************************************************
 */
static void hci_unkcmd_ccevt_send(void)
{
    struct ke_msg *msg = ke_param2msg(ke_msg_alloc(0, TASK_NONE, TASK_NONE, 1));
    uint8_t * pk = ((uint8_t *)(&msg->param)) + TYPE_CCEVT;

    // Increase the number of commands available for reception
    hci_env.nb_h2c_cmd_pkts++;

    //pack event code
    *pk++ = (uint8_t)HCI_CMD_CMPL_EVT_CODE;

    //pack command complete event parameter length
    *pk++ = HCI_CCEVT_HDR_PARLEN + 1;

    //pack the number of h2c packets
    *pk++ = hci_env.nb_h2c_cmd_pkts;

    //pack opcode
    co_write16p(pk, co_htobs((((uint16_t)(hci_env.chdr.ogf) )<<10) | ((uint16_t)(hci_env.chdr.ocf))));
    pk+=2;

    //pack status
    *pk = CO_ERROR_UNKNOWN_HCI_COMMAND;

    //set event type for Tx
    msg->hci_type = HCI_EVT_MSG_TYPE;
    msg->hci_off = TYPE_CCEVT;
    msg->hci_len = HCI_CCEVT_BASIC_LEN;

    // Push the message into the HCI
    hci_push(msg);
}


bool hci_cmd_parameter_size_check(void)
{
    bool size_ok = true;

    if (hci_env.chdr.known_opcode)
    {
        switch(hci_env.chdr.ogf)
        {
            case LK_CNTL_OGF:
            {
                switch(hci_env.chdr.ocf)
                {
                    case OCF(DISCONNECT)     : size_ok = (hci_env.chdr.parlen <= HCI_DISC_CMD_PARLEN   ); break;
                    case OCF(RD_REM_VER_INFO): size_ok = (hci_env.chdr.parlen <= HCI_RDREMVI_CMD_PARLEN); break;
                    default: ASSERT_ERR(0); break;
                }
            }
            break;

            case CNTLR_BB_OGF:
            {
                switch(hci_env.chdr.ocf)
                {
                    case OCF(SET_EVT_MASK)               : size_ok = (hci_env.chdr.parlen <= HCI_SETEMSK_CMD_PARLEN  ); break;
                    case OCF(RESET)                      : size_ok = (hci_env.chdr.parlen <= HCI_RST_CMD_PARLEN      ); break;
                    case OCF(FLUSH)                      : size_ok = (hci_env.chdr.parlen <= HCI_FLSH_CMD_PARLEN     ); break;
                    case OCF(RD_TX_PW_LVL)               : size_ok = (hci_env.chdr.parlen <= HCI_RDTXPW_CMD_PARLEN   ); break;
                    case OCF(SET_CNTLR_TO_HOST_FLOW_CNTL): size_ok = (hci_env.chdr.parlen <= HCI_SETFLOW_CMD_PARLEN  ); break;
                    case OCF(HOST_BUFFER_SIZE)           : size_ok = (hci_env.chdr.parlen <= HCI_HBSZ_CMD_PARLEN     ); break;
                    case OCF(HOST_NB_COMPLETED_PKTS)     : size_ok = (hci_env.chdr.parlen <= HCI_HNBCMPPKT_CMD_PARLEN); break;
                    case OCF(RD_LE_HOST_SUPP)            : size_ok = (hci_env.chdr.parlen <= HCI_RDLEHS_CMD_PARLEN   ); break;
                    case OCF(WR_LE_HOST_SUPP)            : size_ok = (hci_env.chdr.parlen <= HCI_WRLEHS_CMD_PARLEN   ); break;
                    default: ASSERT_ERR(0); break;
                }
            }
            break;

            case INFO_PAR_OGF:
            {
                switch(hci_env.chdr.ocf)
                {
                    case OCF(RD_LOCAL_VER_INFO)  : size_ok = (hci_env.chdr.parlen <= HCI_RDLOCVI_CMD_PARLEN  ); break;
                    case OCF(RD_LOCAL_SUPP_CMDS) : size_ok = (hci_env.chdr.parlen <= HCI_RDLOCCMD_CMD_PARLEN ); break;
                    case OCF(RD_LOCAL_SUPP_FEATS): size_ok = (hci_env.chdr.parlen <= HCI_RDLOCFEAT_CMD_PARLEN); break;
                    case OCF(RD_BUFF_SIZE)       : size_ok = (hci_env.chdr.parlen <= HCI_RDBSZ_CMD_PARLEN    ); break;
                    case OCF(RD_BD_ADDR)         : size_ok = (hci_env.chdr.parlen <= HCI_RDBDA_CMD_PARLEN    ); break;
                    default: ASSERT_ERR(0); break;
                }
            }
            break;

            case STAT_PAR_OGF:
            {
                switch(hci_env.chdr.ocf)
                {
                    case OCF(RD_RSSI) : size_ok = (hci_env.chdr.parlen <= HCI_RDRSSI_CMD_PARLEN); break;
                    default: ASSERT_ERR(0); break;
                }
            }
            break;

            case LE_CNTLR_OGF:
            {
                size_ok = (hci_env.chdr.parlen <= hci_le_cmd_par_size[hci_env.chdr.ocf-1]);
            }
            break;

            case DBG_OGF:
            {
                size_ok = (hci_env.chdr.parlen <= hci_dbg_cmd_par_size[hci_env.chdr.ocf-1]);
            }
            break;

            default:
            {
                ASSERT_ERR(0);
            }
            break;
        }
    }

    return size_ok;
}


void hci_out_of_sync_check(void)
{
    // Check received byte according to current byte position
    switch(hci_env.out_of_sync.index) {
        case 0:
        {
            // Check if received packet ID is HCI command type
            if(hci_env.out_of_sync.byte == HCI_CMD_MSG_TYPE)
            {
                hci_env.out_of_sync.index = 1;
            }
        }
        break;

        case 1:
        {
            // Check if received second byte is HCI reset second byte
            if(hci_env.out_of_sync.byte == (HCI_RESET_CMD_OPCODE & 0xFF))
            {
                hci_env.out_of_sync.index = 2;
            }
            // Check if received second byte is HCI reset first byte
            else if(hci_env.out_of_sync.byte == HCI_CMD_MSG_TYPE)
            {
                hci_env.out_of_sync.index = 1;
            }
            else
            {
                hci_env.out_of_sync.index = 0;
            }
        }
        break;

        case 2:
        {
            // Check if received third byte is HCI reset third byte
            if(hci_env.out_of_sync.byte == ((HCI_RESET_CMD_OPCODE >> 8) & 0xFF))
            {
                hci_env.out_of_sync.index = 3;
            }
            // Check if received third byte is HCI reset first byte
            else if(hci_env.out_of_sync.byte == HCI_CMD_MSG_TYPE)
            {
                hci_env.out_of_sync.index = 1;
            }
            else
            {
                hci_env.out_of_sync.index = 0;
            }
        }
        break;

        case 3:
        {
            // Check if received fourth byte is HCI reset parameter size
            if(hci_env.out_of_sync.byte == HCI_RST_CMD_PARLEN)
            {
                hci_env.out_of_sync.index = 4;
            }
            // Check if received fourth byte is HCI reset first byte
            else if(hci_env.out_of_sync.byte == HCI_CMD_MSG_TYPE)
            {
                hci_env.out_of_sync.index = 1;
            }
            else
            {
                hci_env.out_of_sync.index = 0;
            }
        }
        break;

        default:
        {
            ASSERT_ERR(0);
        }
        break;
    }

    #if DEEP_SLEEP
    // Check if we are receiving something or not
    if (hci_env.out_of_sync.index == 0)
        // No HCI reception is ongoing
        rwip_prevent_sleep_clear(RW_HCI_RX_ONGOING);
    else
        // An HCI reception is ongoing
        rwip_prevent_sleep_set(RW_HCI_RX_ONGOING);
    #endif //DEEP_SLEEP
}

/**
 ****************************************************************************************
 * @brief Function called in kernel message allocation function for Link Control commands
 *        with parameters. All supported LK Cntl commands have parameters.
 *
 * @param ocf  OCF of command for identification.
 * @return Pointer to the allocated kernel message parameter space.
 *****************************************************************************************
 */
static uint8_t * hci_cmd_alloc_lk_cntl(uint8_t ocf)
{
    //Message ID of cmd
    uint16_t msgid = 0;
    //Destination of cmd
    uint16_t srcid = TASK_NONE;
    //Size of cmd parameters
    uint16_t size = 0;

    #if (BLE_CENTRAL || BLE_PERIPHERAL)
    switch(ocf)
    {
        case OCF(DISCONNECT):
            msgid  = LLC_DISCONNECT_CMD;
            srcid = TASK_HCI;
            size   = sizeof(struct  llc_disc_req);
            break;

        case OCF(RD_REM_VER_INFO):
            msgid  = LLC_RD_REM_VER_INFO_CMD;
            srcid = TASK_HCI;
            size   = sizeof (struct  llc_rd_rem_info_ver_cmd);
            break;
        default:
        	return (uint8_t*)(NULL);
        	break;
    }
    #endif // (BLE_CENTRAL || BLE_PERIPHERAL)
    //allocate kernel message space for command to be unpacked
    return (uint8_t *)ke_msg_alloc(msgid, MSG_T(msgid), srcid, co_max(hci_env.chdr.parlen, size));
}


/**
 ****************************************************************************************
 * @brief Function called in kernel message allocation function for Controller BB commands
 *        with parameters. Some supported Cntlr BB commands have no parameters, they
 *        should never end up in this function, assert if it happens.
 *
 * @param ocf  OCF of command for identification.
 * @return Pointer to the allocated kernel message parameter space.
 *****************************************************************************************
 */
static uint8_t * hci_cmd_alloc_cntlr_bb(uint8_t ocf)
{
    //Message ID of cmd
    uint16_t msgid = 0;
    //Destination of cmd
    uint16_t srcid = TASK_NONE;
    //Size of cmd parameters
    uint16_t size = 0;

    switch(ocf)
    {
        case OCF(SET_EVT_MASK):
            msgid = LLM_SET_EVT_MASK_CMD;
            srcid = TASK_HCI;
            size  = (sizeof (struct  llm_set_evt_msk_cmd));
            break;

        #if (BLE_CENTRAL || BLE_PERIPHERAL)
        case OCF(FLUSH):
            msgid  = LLC_FLUSH_CMD;
            srcid = TASK_HCI;
            size  = (sizeof (struct  llc_flush_cmd));
            break;

        case OCF(RD_TX_PW_LVL):
            msgid  = LLC_RD_TX_PW_LVL_CMD;
            srcid = TASK_HCI;
            size  = (sizeof (struct  llc_rd_tx_pw_lvl_cmd));
            break;
        #endif // #if (BLE_CENTRAL || BLE_PERIPHERAL)
        case OCF(SET_CNTLR_TO_HOST_FLOW_CNTL):
            msgid  = LLM_SET_CNTLR_TO_HOST_FLOW_CNTL_CMD;
            srcid = TASK_HCI;
            size  = (sizeof (struct  llm_set_cntl_host_flow_cmd));
            break;

        case OCF(HOST_BUFFER_SIZE):
            msgid  = LLM_HOST_BUFFER_SIZE_CMD;
            srcid = TASK_HCI;
            size  = (sizeof (struct  llm_host_buff_size_cmd));
            break;

        case OCF(HOST_NB_COMPLETED_PKTS):
            msgid  = LLM_HOST_NB_COMPLETED_PKTS_CMD;
            srcid = TASK_HCI;
            size  = (sizeof (struct  llm_host_nb_cmp_pkts_cmd));
            break;

        //param size 0, no alloc here
        default:
            ASSERT_ERR(0);
            break;
    }

    //allocate kernel message space for command to be unpacked
    return (uint8_t *)ke_msg_alloc(msgid, MSG_T(msgid), srcid, co_max(hci_env.chdr.parlen, size));
}

/**
 ****************************************************************************************
 * @brief Function called in kernel message allocation function for status and parameter
 *        commands with parameters.
 *
 * @param ocf  OCF of command for identification.
 * @return Pointer to the allocated kernel message parameter space.
 *****************************************************************************************
 */
static uint8_t * hci_cmd_alloc_stat_par(uint8_t ocf)
{
    //Message ID of cmd
    uint16_t msgid = 0;
    //Destination of cmd
    uint16_t srcid = TASK_NONE;
    //Size of cmd parameters
    uint16_t size = 0;

    switch(ocf)
    {
        #if (BLE_CENTRAL || BLE_PERIPHERAL)
        case OCF(RD_RSSI):
            msgid  = LLC_RD_RSSI_CMD;
            srcid = TASK_HCI;
            size   = sizeof(struct  llc_rd_rssi_cmd);
            break;
        #endif // #if (BLE_CENTRAL || BLE_PERIPHERAL)
        //param size 0, don't alloc here
        default:
            ASSERT_ERR(0);
            break;
    }

    //allocate kernel message space for command to be unpacked
    return (uint8_t *)ke_msg_alloc(msgid, MSG_T(msgid), srcid, co_max(hci_env.chdr.parlen, size));
}


/**
 ****************************************************************************************
 * @brief Function called in kernel message allocation function for LE commands
 *        with parameters. Some supported LE commands have no parameters, they
 *        should never end up in this function, assert if it happens.
 *
 * @param ocf  OCF of command for identification.
 * @return Pointer to the allocated kernel message parameter space.
 *****************************************************************************************
 */
static uint8_t * hci_cmd_alloc_le(uint8_t ocf)
{
    //Message ID of cmd
    uint16_t msgid = 0;
    //Destination of cmd
    uint16_t srcid = TASK_NONE;
    //Size of cmd parameters
    uint16_t size = 0;

    switch(ocf)
    {
        case OCF(LE_SET_EVT_MASK):
            msgid = LLM_LE_SET_EVT_MASK_CMD;
            srcid = TASK_HCI;
            size  = (sizeof (struct  llm_le_set_evt_mask_cmd));
            break;

        case OCF(LE_SET_RAND_ADDR):
            msgid = LLM_LE_SET_RAND_ADDR_CMD;
            srcid = TASK_HCI;
            size  = (sizeof (struct  llm_le_set_rand_addr_cmd));
            break;

        case OCF(LE_SET_ADV_PARAM):
            msgid = LLM_LE_SET_ADV_PARAM_CMD;
            srcid = TASK_HCI;
            size  = (sizeof (struct  llm_le_set_adv_param_cmd));
            break;

        case OCF(LE_SET_ADV_DATA):
            msgid = LLM_LE_SET_ADV_DATA_CMD;
            srcid = TASK_HCI;
            size  = (sizeof (struct  llm_le_set_adv_data_cmd));
            break;

        case OCF(LE_SET_SCAN_RSP_DATA):
            msgid = LLM_LE_SET_SCAN_RSP_DATA_CMD;
            srcid = TASK_HCI;
            size  = (sizeof (struct  llm_le_set_scan_rsp_data_cmd));
            break;

        case OCF(LE_SET_ADV_EN):
            msgid = LLM_LE_SET_ADV_EN_CMD;
            srcid = TASK_HCI;
            size  = (sizeof (struct  llm_le_set_adv_en_cmd));
            break;

        case OCF(LE_SET_SCAN_PARAM):
            msgid = LLM_LE_SET_SCAN_PARAM_CMD;
            srcid = TASK_HCI;
            size  = (sizeof (struct  llm_le_set_scan_param_cmd));
            break;

        case OCF(LE_SET_SCAN_EN):
            msgid = LLM_LE_SET_SCAN_EN_CMD;
            srcid = TASK_HCI;
            size  = (sizeof (struct  llm_le_set_scan_en_cmd));
            break;

        case OCF(LE_CREATE_CON):
            msgid = LLM_LE_CREATE_CON_CMD;
            srcid = TASK_HCI;
            size  = (sizeof (struct  llm_le_create_con_cmd));
            break;

        case OCF(LE_ADD_DEV_TO_WLST):
            msgid = LLM_LE_ADD_DEV_TO_WLST_CMD;
            srcid = TASK_HCI;
            size  = (sizeof (struct  llm_le_add_dev_to_wlst_cmd));
            break;

        case OCF(LE_RMV_DEV_FROM_WLST):
            msgid = LLM_LE_RMV_DEV_FROM_WLST_CMD;
            srcid = TASK_HCI;
            size  = (sizeof (struct  llm_le_add_dev_to_wlst_cmd));
            break;

        #if (BLE_CENTRAL || BLE_PERIPHERAL)
        case OCF(LE_CON_UPDATE):
            msgid = LLC_LE_UPDATE_CON_REQ;
            srcid = TASK_HCI;
            size  = (sizeof (struct  llc_le_con_update_cmd));
            break;
        #endif // #if (BLE_CENTRAL || BLE_PERIPHERAL)

        case OCF(LE_SET_HOST_CHNL_CLASSIF):
            msgid = LLM_LE_SET_HOST_CHNL_CLASSIF_CMD;
            srcid = TASK_HCI;
            size  = (sizeof (struct  llm_le_set_host_chnl_classif_cmd));
            break;

        #if (BLE_CENTRAL || BLE_PERIPHERAL)
        case OCF(LE_RD_CHNL_MAP):
            msgid = LLC_LE_RD_CHNL_MAP_CMD;
            srcid = TASK_HCI;
            size  = (sizeof (struct  llc_le_rd_ch_map_cmd));
            break;

        case OCF(LE_RD_REM_USED_FEATS):
            msgid = LLC_LE_RD_REM_USED_FEATS_CMD;
            srcid = TASK_HCI;
            size  = (sizeof (struct  llc_le_rd_rem_used_feats_cmd));
            break;
        #endif // #if (BLE_CENTRAL || BLE_PERIPHERAL)

        case OCF(LE_ENC):
            msgid = LLM_LE_ENC_CMD;
            srcid = TASK_HCI;
            size  = (sizeof (struct  llm_le_enc_cmd));
            break;

        #if (BLE_CENTRAL || BLE_PERIPHERAL)
        case OCF(LE_START_ENC):
            msgid = LLC_LE_START_ENC_CMD;
            srcid = TASK_HCI;
            size  = (sizeof (struct  llc_le_start_enc_cmd));
            break;

        case OCF(LE_LTK_REQ_RPLY):
            msgid = LLC_LE_LTK_REQ_RPLY_CMD;
            srcid = TASK_HCI;
            size  = (sizeof (struct  llc_le_ltk_req_rply_cmd));
            break;

        case OCF(LE_LTK_REQ_NEG_RPLY):
            msgid = LLC_LE_LTK_REQ_NEG_RPLY_CMD;
            srcid = TASK_HCI;
            size  = (sizeof (struct  llc_le_ltk_req_neg_rply_cmd));
            break;
        #endif // #if (BLE_CENTRAL || BLE_PERIPHERAL)

        case OCF(LE_RX_TEST):
            msgid = LLM_LE_TEST_RX_CMD;
            srcid = TASK_HCI;
            size  = (sizeof (struct  llm_le_rx_test_cmd));
            break;

        case OCF(LE_TX_TEST):
            msgid = LLM_LE_TEST_TX_CMD;
            srcid = TASK_HCI;
            size  = (sizeof (struct  llm_le_tx_test_cmd));
            break;

        //param size 0, don't alloc here
        default:
            ASSERT_ERR(0);
            break;
    }

    //allocate kernel message space for command to be unpacked
    return (uint8_t *)ke_msg_alloc(msgid, MSG_T(msgid), srcid, co_max(hci_env.chdr.parlen, size));
}


/**
 ****************************************************************************************
 * @brief Function called in kernel message allocation function for Debug commands
 *        with parameters. Some supported Debug commands have no parameters, they
 *        should never end up in this function, assert if it happens.
 *
 * @param ocf  OCF of command for identification.
 * @return Pointer to the allocated kernel message parameter space.
 *****************************************************************************************
 */
static uint8_t * hci_cmd_alloc_dbg(uint8_t ocf)
{
    //Message ID of cmd
    uint16_t msgid = 0;
    //Destination of cmd
    uint16_t srcid = TASK_NONE;
    //Size of cmd parameters
    uint16_t size = 0;

    switch(ocf)
    {
        #if (BLE_DEBUG)
        case OCF(DBG_RD_MEM):
            msgid = DBG_RD_MEM_REQ;
            srcid = TASK_HCI;
            size = (sizeof (struct dbg_rd_mem_cmd));break;

        case OCF(DBG_WR_MEM):
            msgid = DBG_WR_MEM_REQ;
            srcid = TASK_HCI;
            size = (sizeof (struct dbg_wr_mem_cmd));break;
            
        case OCF(DBG_DEL_PAR):
            msgid = DBG_DEL_PARAM_REQ;
            srcid = TASK_HCI;
            size = (sizeof (struct dbg_del_param_cmd));break;

        case OCF(DBG_FLASH_ER):
            msgid = DBG_FLASH_ERASE_REQ;
            srcid = TASK_HCI;
            size = (sizeof (struct dbg_flash_erase_cmd));break;

        case OCF(DBG_FLASH_WR):
            msgid = DBG_FLASH_WRITE_REQ;
            srcid = TASK_HCI;
            size = (sizeof (struct dbg_flash_write_cmd));break;

        case OCF(DBG_FLASH_RD):
            msgid = DBG_FLASH_READ_REQ;
            srcid = TASK_HCI;
            size = (sizeof (struct dbg_flash_read_cmd));break;

        case OCF(DBG_RD_PAR):
            msgid = DBG_RD_PARAM_REQ;
            srcid = TASK_HCI;
            size = (sizeof (struct dbg_rd_param_cmd));break;

        case OCF(DBG_WR_PAR):
            msgid = DBG_WR_PARAM_REQ;
            srcid = TASK_HCI;
            size = (sizeof (struct dbg_wr_param_cmd));break;

        case OCF(DBG_HW_REG_RD):
            msgid = DBG_HW_REG_RD_REQ;
            srcid = TASK_HCI;
            size  = (sizeof (struct dbg_hw_reg_rd_cmd));
            break;

        case OCF(DBG_HW_REG_WR):
            msgid = DBG_HW_REG_WR_REQ;
            srcid = TASK_HCI;
            size  = (sizeof (struct  dbg_hw_reg_wr_cmd));
            break;

        case OCF(DBG_SET_BD_ADDR):
            msgid = DBG_LE_SET_BD_ADDR_REQ;
            srcid = TASK_HCI;
            size  = (sizeof (struct dbg_set_bd_addr_cmd));
            break;

        case OCF(DBG_SET_CRC):
            msgid = DBG_LE_SET_CRC_REQ;
            srcid = TASK_HCI;
            size  = (sizeof (struct dbg_set_crc_cmd));
            break;

        case OCF(DBG_LLCP_DISCARD):
            msgid = DBG_LE_LLCP_DISCARD_REQ;
            srcid = TASK_HCI;
            size  = (sizeof (struct dbg_le_llcp_discard_req));
            break;

        case OCF(DBG_RESET_RX_CNT):
            msgid = DBG_LE_RESET_RX_CNT_REQ;
            srcid = TASK_HCI;
            size  = (sizeof (struct dbg_reset_cnt_req));
            break;

        case OCF(DBG_RESET_TX_CNT):
            msgid = DBG_LE_RESET_TX_CNT_REQ;
            srcid = TASK_HCI;
            size  = (sizeof (struct dbg_reset_cnt_req));
            break;

        case OCF(DBG_RF_REG_RD):
            msgid = DBG_RF_REG_RD_REQ;
            srcid = TASK_HCI;
            size  = (sizeof (struct dbg_rf_reg_rd_cmd));
            break;

        case OCF(DBG_RF_REG_WR):
            msgid = DBG_RF_REG_WR_REQ;
            srcid = TASK_HCI;
            size  = (sizeof (struct dbg_rf_reg_wr_cmd));
            break;

        case OCF(DBG_SET_TX_PW):
            msgid = DBG_SET_TX_PW_REQ;
            srcid = TASK_HCI;
            size  = (sizeof (struct dbg_set_tx_pw_cmd));
            break;
        #endif //BLE_DEBUG

        #if (DSM_SUPPORT)
        case OCF(DBG_DATA_STORAGE_OP):
            msgid = DBG_DATA_STORAGE_OP_REQ;
            srcid = TASK_HCI;
            size = (sizeof (struct dbg_data_storage_op_cmd));
            break;
        #endif //DSM_SUPPORT

        case OCF(DBG_PLF_RESET):
            msgid = DBG_PLF_RESET_REQ;
            srcid = TASK_HCI;
            size = (sizeof (struct dbg_plf_reset_cmd));
            break;
        #if (RW_BLE_WLAN_COEX)
        case OCF(DBG_WLAN_COEX):
            msgid = DBG_WLAN_COEX_REQ;
            srcid = TASK_HCI;
            size  = (sizeof (struct dbg_wlan_coex_cmd));
            break;
        #endif //RW_BLE_WLAN_COEX
            //cmds with no params must not reach here
        default:
            ASSERT_ERR(0);
            break;
    }
    //allocate kernel message space for command to be unpacked
    return (uint8_t *)ke_msg_alloc(msgid, MSG_T(msgid), srcid, co_max(hci_env.chdr.parlen, size));
}


uint8_t * hci_cmd_alloc(void)
{
    uint8_t ogf = hci_env.chdr.ogf;
    uint8_t ocf = hci_env.chdr.ocf;
    uint8_t *msg = NULL;

    if (hci_env.chdr.known_opcode)
    {

        switch(ogf)
        {
            case LK_CNTL_OGF:
                msg = hci_cmd_alloc_lk_cntl(ocf);
                break;

            case CNTLR_BB_OGF:
                msg = hci_cmd_alloc_cntlr_bb(ocf);
                break;

            case STAT_PAR_OGF:
                msg = hci_cmd_alloc_stat_par(ocf);
                break;

            case LE_CNTLR_OGF:
                msg = hci_cmd_alloc_le(ocf);
                break;

            case DBG_OGF:
                msg = hci_cmd_alloc_dbg(ocf);
                break;

            case INFO_PAR_OGF:
            default:
                ASSERT_ERR(0);
                break;
        }
    }
    //unknown opcode but params must be received
    else
    {
        //send back event: CCEVT with opcode and status CO_ERROR_UNKNOWN_HCI_COMMAND
        hci_unkcmd_ccevt_send();

        //set space for payload anyway, to finish transfer
        msg = hci_env.unk_cmd_buf;
    }
    return (msg);
}

/**
****************************************************************************************
* @brief Local function : allocates the appropriate descriptor for received data
* depending on the HCI location: EMB or Host in split mode.
*****************************************************************************************
*/
void hci_data_alloc(void)
{
    //allocate a descriptor
    hci_env.ahdr.txtag = co_buf_tx_alloc();

    if(hci_env.ahdr.txtag == NULL)
    {
        #if (BLE_CENTRAL || BLE_PERIPHERAL)
        //create the overflow event
        struct llc_data_buf_ovflw_evt * evt =
           (struct llc_data_buf_ovflw_evt *)ke_msg_alloc(LLC_DATA_BUF_OVER_FLOW_EVT,
                   TASK_HCI,
                                              KE_BUILD_ID(TASK_LLC, hci_env.ahdr.hdl),
                                              sizeof(struct llc_data_buf_ovflw_evt));
        //ACL buffer Overflow
        evt->link_type = 1;
        // Send the kernel message
        ke_msg_send(evt);
        #endif // #if (BLE_CENTRAL || BLE_PERIPHERAL)

        hci_env.ahdr.txtag = (struct co_buf_tx_node *)hci_env.unk_cmd_buf;
        hci_env.curr_payl_buff = hci_env.unk_cmd_buf;
    }
    else
        //Give the pointer to the data space
        hci_env.curr_payl_buff = co_buf_tx_desc_get(hci_env.ahdr.txtag->idx)->data;
}

/**
 ****************************************************************************************
 * @brief Function called in basic kernel message sending for Controller Baseband commands
 *        with no parameters.
 *
 * @param ocf  OCF of command for identification.
 * Uses the hci_env command header OGF and OCF components.
 *****************************************************************************************
 */
static void hci_cmd_dispatch_basic_cntlr_bb(uint8_t ocf)
{
    //Message ID of cmd
    uint16_t msgid = 0;
    //Destination of cmd
    uint16_t srcid = TASK_NONE;

    switch(ocf)
    {
        case OCF(RESET)          : msgid  = LLM_RESET_CMD;           srcid = TASK_HCI; break;
        // commands with params should not end up here
        default: ASSERT_ERR(0); break;
    }
    //send basic kernel message
    ke_msg_send_basic(msgid, MSG_T(msgid), srcid);
}


/**
 ****************************************************************************************
 * @brief Function called in basic kernel message sending for Information Parameters
 *        commands with no parameters.
 *
 * @param ocf  OCF of command for identification.
 * Uses the hci_env command header OGF and OCF components.
 *****************************************************************************************
 */
static void hci_cmd_dispatch_basic_info_par(uint8_t ocf)
{
    //Message ID of cmd
    uint16_t msgid = 0;
    //Destination of cmd
    uint16_t srcid = TASK_NONE;

    switch(ocf)
    {
        case OCF(RD_LOCAL_VER_INFO)  : msgid = LLM_RD_LOCAL_VER_INFO_CMD;   srcid = TASK_HCI; break;
        case OCF(RD_LOCAL_SUPP_CMDS) : msgid = LLM_RD_LOCAL_SUPP_CMDS_CMD; srcid = TASK_HCI; break;
        case OCF(RD_LOCAL_SUPP_FEATS): msgid = LLM_RD_LOCAL_SUPP_FEATS_CMD; srcid = TASK_HCI; break;
        case OCF(RD_BUFF_SIZE)       : msgid = LLM_RD_BUFF_SIZE_CMD;        srcid = TASK_HCI; break;
        case OCF(RD_BD_ADDR)         : msgid = LLM_RD_BD_ADDR_CMD;          srcid = TASK_HCI; break;
        // commands with params should not end up here
        default: ASSERT_ERR(0); break;
    }
    //send basic kernel message
    ke_msg_send_basic(msgid, MSG_T(msgid), srcid);
}


/**
 ****************************************************************************************
 * @brief Function called in basic kernel message sending for LE
 *        commands with no parameters.
 *
 * @param ocf  OCF of command for identification.
 * Uses the hci_env command header OGF and OCF components.
 *****************************************************************************************
 */
static void hci_cmd_dispatch_basic_le(uint8_t ocf)
{
    //Message ID of cmd
    uint16_t msgid = 0;
    //Destination of cmd
    uint16_t srcid = TASK_NONE;

    switch(ocf)
    {
        case OCF(LE_RD_BUFF_SIZE)       : msgid = LLM_LE_RD_BUFF_SIZE_CMD;        srcid = TASK_HCI;  break;
        case OCF(LE_RD_LOCAL_SUPP_FEATS): msgid = LLM_LE_RD_LOCAL_SUPP_FEATS_CMD; srcid = TASK_HCI;  break;
        case OCF(LE_RD_ADV_CHNL_TX_PW)  : msgid = LLM_LE_RD_ADV_CHNL_TX_PW_CMD;   srcid = TASK_HCI;  break;
        case OCF(LE_CREATE_CON_CANCEL)  : msgid = LLM_LE_CREATE_CON_CANCEL_CMD;   srcid = TASK_HCI;  break;
        case OCF(LE_RD_WLST_SIZE)       : msgid = LLM_LE_RD_WLST_SIZE_CMD;        srcid = TASK_HCI;  break;
        case OCF(LE_CLEAR_WLST)         : msgid = LLM_LE_CLEAR_WLST_CMD;          srcid = TASK_HCI;  break;
        case OCF(LE_RAND)               : msgid = LLM_LE_RAND_CMD;                srcid = TASK_HCI; break;
        case OCF(LE_RD_SUPP_STATES)     : msgid = LLM_LE_RD_SUPP_STATES_CMD;      srcid = TASK_HCI;  break;
        case OCF(LE_TEST_END)           : msgid = LLM_LE_TEST_END_CMD;            srcid = TASK_HCI;  break;

        default: ASSERT_ERR(0); break;
    }
    //send basic kernel message
    ke_msg_send_basic(msgid, MSG_T(msgid), srcid);
}


#if (BLE_DEBUG)
/**
 ****************************************************************************************
 * @brief Function called in basic kernel message sending for Debug
 *        commands with no parameters.
 *
 * @param ocf  OCF of command for identification.
 * Uses the hci_env command header OGF and OCF components.
 *****************************************************************************************
 */
static void hci_cmd_dispatch_basic_dbg(uint8_t ocf)
{
    //Message ID of cmd
    uint16_t msgid = 0;
    //Destination of cmd
    uint16_t srcid = TASK_NONE;

    switch(ocf)
    {
        case OCF(DBG_FLASH_ID)     : msgid = DBG_FLASH_IDENT_REQ      ; srcid = TASK_HCI; break;
        case OCF(DBG_RD_KE_STATS)  : msgid = DBG_RD_KE_STATS_REQ      ; srcid = TASK_HCI; break;
        case OCF(DBG_SET_TYPE_PUB) : msgid = DBG_LE_SET_TYPE_PUB_REQ  ; srcid = TASK_HCI; break;
        case OCF(DBG_SET_TYPE_RAND): msgid = DBG_LE_SET_TYPE_RAND_REQ ; srcid = TASK_HCI; break;
        case OCF(DBG_RD_MEM_INFO)  : msgid =  DBG_RD_MEM_INFO_REQ     ; srcid = TASK_HCI; break;

        default: ASSERT_ERR(0); break;
    }
    //send basic kernel message
    ke_msg_send_basic(msgid, MSG_T(msgid), srcid);
}
#endif //BLE_DEBUG


void hci_cmd_dispatch_basic(void)
{
    uint8_t ogf = hci_env.chdr.ogf;
    uint8_t ocf = hci_env.chdr.ocf;


    // Decrease the number of commands available for reception
    hci_env.nb_h2c_cmd_pkts--;

    if (hci_env.chdr.known_opcode)
    {
        switch(ogf)
        {
            case CNTLR_BB_OGF:
                hci_cmd_dispatch_basic_cntlr_bb(ocf);
                break;

            case INFO_PAR_OGF:
                hci_cmd_dispatch_basic_info_par(ocf);
                break;

            case LE_CNTLR_OGF:
                hci_cmd_dispatch_basic_le(ocf);
                break;

            #if (BLE_DEBUG)
            case DBG_OGF:
                hci_cmd_dispatch_basic_dbg(ocf);
                break;
            #endif //BLE_DEBUG

            case LK_CNTL_OGF:
            case STAT_PAR_OGF:
            default:
                ASSERT_ERR(0);
                break;
        }
    }

    //unknown opcode
    else
    {
        //send back event: CCEVT with opcode and status CO_ERROR_UNKNOWN_HCI_COMMAND
        hci_unkcmd_ccevt_send();
    }
}


/**
 ****************************************************************************************
 * @brief Function called in kernel message dispatch for Controller Baseband commands
 *        with parameters.
 * @param[in]  ocf  COmmand OCF field for identification.
 * @param[in]  payl Pointer to receiver buffer payload
 *****************************************************************************************
 */
static void hci_cmd_dispatch_cntlr_bb(uint8_t ocf, uint8_t * payl)
{
    switch(ocf)
    {
        case OCF(SET_EVT_MASK)               :
        case OCF(SET_CNTLR_TO_HOST_FLOW_CNTL):
        case OCF(WR_LE_HOST_SUPP)            : hci_cmd_aligned_unpk(payl); break;
        case OCF(FLUSH)                      :
        #if (BLE_PERIPHERAL || BLE_CENTRAL)
        case OCF(RD_TX_PW_LVL)               : hci_llc_basic_cmd_unpk(payl); break;
        #endif //(BLE_PERIPHERAL || BLE_CENTRAL)
        case OCF(HOST_BUFFER_SIZE)           : hci_host_buffer_size_cmd_unpk(payl); break;
        case OCF(HOST_NB_COMPLETED_PKTS)     : hci_host_nb_completed_pkts_cmd_unpk(payl); break;

        //cmds with no params do not end up here
        default: ASSERT_ERR(0); break;
    }
}


/**
 ****************************************************************************************
 * @brief Function called in kernel message dispatch for status parameter commands
 *        with parameters.
 * @param[in]  ocf  COmmand OCF field for identification.
 * @param[in]  payl Pointer to receiver buffer payload
 *****************************************************************************************
 */
static void hci_cmd_dispatch_stat_par(uint8_t ocf, uint8_t * payl)
{
    switch(ocf)
    {
        #if (BLE_PERIPHERAL || BLE_CENTRAL)
        case OCF(RD_RSSI): hci_llc_basic_cmd_unpk(payl); break;
        #endif //(BLE_PERIPHERAL || BLE_CENTRAL)
        //cmds with no params do not end up here
        default: ASSERT_ERR(0); break;
    }
}


/**
 ****************************************************************************************
 * @brief Function called in kernel message dispatch for LE commands with parameters.
 *
 * @param[in]  ocf  COmmand OCF field for identification.
 * @param[in]  payl Pointer to receiver buffer payload
 *****************************************************************************************
 */
static void hci_cmd_dispatch_le(uint8_t ocf, uint8_t * payl)
{
    switch(ocf)
    {
        case OCF(LE_SET_EVT_MASK)         :
        case OCF(LE_SET_RAND_ADDR)        :
        case OCF(LE_SET_ADV_DATA)         :
        case OCF(LE_SET_SCAN_RSP_DATA)    :
        case OCF(LE_SET_ADV_EN)           :
        case OCF(LE_SET_SCAN_EN)          :
        case OCF(LE_ADD_DEV_TO_WLST)      :
        case OCF(LE_RMV_DEV_FROM_WLST)    :
        case OCF(LE_SET_HOST_CHNL_CLASSIF):
        case OCF(LE_ENC)                  :
        case OCF(LE_RX_TEST)              :
        case OCF(LE_TX_TEST)              : hci_cmd_aligned_unpk(payl); break;
        #if (BLE_PERIPHERAL || BLE_CENTRAL)
        case OCF(LE_RD_CHNL_MAP)          :
        case OCF(LE_RD_REM_USED_FEATS)    :
        case OCF(LE_LTK_REQ_RPLY)         :
        case OCF(LE_LTK_REQ_NEG_RPLY)     : hci_llc_basic_cmd_unpk(payl); break;
        #endif // (BLE_PERIPHERAL || BLE_CENTRAL)
        #if (BLE_PERIPHERAL || BLE_BROADCASTER)
        case OCF(LE_SET_ADV_PARAM)        : hci_le_set_adv_param_cmd_unpk(payl); break;
        #endif // BLE_PERIPHERAL || BLE_BROADCASTER
        #if (BLE_CENTRAL || BLE_OBSERVER)
        case OCF(LE_SET_SCAN_PARAM)       : hci_le_set_scan_param_cmd_unpk(payl); break;
        #endif // BLE_CENTRAL || BLE_OBSERVER
        #if (BLE_CENTRAL)
        case OCF(LE_CREATE_CON)           : hci_le_create_con_cmd_unpk(payl); break;
        #endif // BLE_CENTRAL
        #if (BLE_CENTRAL || BLE_PERIPHERAL)
        case OCF(LE_CON_UPDATE)           : hci_le_con_update_cmd_unpk(payl); break;
        case OCF(LE_START_ENC)            : hci_le_start_enc_cmd_unpk(payl); break;
        #endif // BLE_CENTRAL || BLE_PERIPHERAL

        //cmds with no params end up here
        default: ASSERT_ERR(0); break;
    }
}


#if (BLE_DEBUG)
/**
 ****************************************************************************************
 * @brief Function called in kernel message dispatch for Debug commands with parameters.
 *
 * @param[in]  ocf  COmmand OCF field for identification.
 * @param[in]  payl Pointer to receiver buffer payload
 *****************************************************************************************
 */
static void hci_cmd_dispatch_dbg(uint8_t ocf, uint8_t * payl)
{
    switch(ocf)
    {
        #if (BLE_DEBUG)
        case OCF(DBG_RD_MEM)      : hci_dbg_rd_mem_cmd_unpk(payl);break;
        case OCF(DBG_WR_MEM)      : hci_dbg_wr_mem_cmd_unpk(payl);break;
        case OCF(DBG_DEL_PAR)     : hci_dbg_del_param_cmd_unpk(payl);break;
        case OCF(DBG_FLASH_ER)    : hci_dbg_flash_erase_cmd_unpk(payl);break;
        case OCF(DBG_FLASH_WR)    : hci_dbg_flash_write_cmd_unpk(payl);break;
        case OCF(DBG_FLASH_RD)    : hci_dbg_flash_read_cmd_unpk(payl);break;
        case OCF(DBG_RD_PAR)      : hci_dbg_rd_param_cmd_unpk(payl);break;
        case OCF(DBG_WR_PAR)      : hci_dbg_wr_param_cmd_unpk(payl);break;
        case OCF(DBG_HW_REG_RD)   : hci_dbg_hw_reg_rd_cmd_unpk(payl);break;
        case OCF(DBG_HW_REG_WR)   : hci_dbg_hw_reg_wr_cmd_unpk(payl);break;
        case OCF(DBG_SET_BD_ADDR) : hci_cmd_aligned_unpk(payl);break;
        case OCF(DBG_SET_CRC)     : hci_dbg_set_crc_cmd_unpk(payl);break;
        case OCF(DBG_LLCP_DISCARD): hci_dbg_llcp_discard_cmd_unpk(payl);break;
        case OCF(DBG_RESET_RX_CNT): hci_dbg_reset_cnt_cmd_unpk(payl);break;
        case OCF(DBG_RESET_TX_CNT): hci_dbg_reset_cnt_cmd_unpk(payl);break;
        case OCF(DBG_RF_REG_RD)   : hci_dbg_rf_reg_rd_cmd_unpk(payl);break;
        case OCF(DBG_RF_REG_WR)   : hci_dbg_rf_reg_wr_cmd_unpk(payl);break;
        case OCF(DBG_SET_TX_PW)   : hci_dbg_set_tx_pw_cmd_unpk(payl);break;
        #if (DSM_SUPPORT)
        case OCF(DBG_DATA_STORAGE_OP): hci_cmd_aligned_unpk(payl);break;
        #endif //DSM_SUPPORT
        #endif //BLE_DEBUG
        case OCF(DBG_PLF_RESET)   : hci_cmd_aligned_unpk(payl);break;
        #if (RW_BLE_WLAN_COEX)
        case OCF(DBG_WLAN_COEX)      : hci_cmd_aligned_unpk(payl);break;
        #endif // RW_BLE_WLAN_COEX
        //cmds with no params end up here
        default: ASSERT_ERR(0); break;
    }
}
#endif //BLE_DEBUG


void hci_cmd_dispatch(uint8_t * payl )
{
    uint8_t ogf = hci_env.chdr.ogf;
    uint8_t ocf = hci_env.chdr.ocf;

    // Decrease the number of commands available for reception
    hci_env.nb_h2c_cmd_pkts--;

    if (hci_env.chdr.known_opcode)
    {

        switch(ogf)
        {
            #if (BLE_PERIPHERAL || BLE_CENTRAL)
            case LK_CNTL_OGF:
                hci_llc_basic_cmd_unpk(payl);
                break;
            #endif // (BLE_PERIPHERAL || BLE_CENTRAL)
            case CNTLR_BB_OGF:
                hci_cmd_dispatch_cntlr_bb(ocf, payl);
                break;

            case STAT_PAR_OGF:
                hci_cmd_dispatch_stat_par(ocf, payl);
                break;

            case LE_CNTLR_OGF:
                hci_cmd_dispatch_le(ocf, payl);
                break;

            #if (BLE_DEBUG)
            case DBG_OGF:
                hci_cmd_dispatch_dbg(ocf, payl);
                break;
            #endif //BLE_DEBUG

            case INFO_PAR_OGF:
            default:
                ASSERT_ERR(0);
                break;
        }
    }

    //unknown opcode do nothing, data will be overwritten in the buffer at next unkn command
}

void hci_data_dispatch(uint8_t * payl)
{
    #if (BLE_CENTRAL || BLE_PERIPHERAL)
    //create the structure which is sent to the LLC
    struct llc_data_packet_req * s =
       (struct llc_data_packet_req *)ke_msg_alloc(LLC_LE_DATA_REQ,
                                                  KE_BUILD_ID(TASK_LLC, hci_env.ahdr.hdl),
                                                  TASK_HCI,
                                                  sizeof(struct llc_data_packet_req));

    //fill structure
    s->conhdl = hci_env.ahdr.hdl;
    s->pb_bc_flag = hci_env.ahdr.bcpb_flag;

    //dispatch is done before subtracting DL of a surpassing length if necessary
    if (hci_env.ahdr.datalen > BLE_DATA_LEN)
    {
        s->length = BLE_DATA_LEN;
    }
    else
    {
        s->length = hci_env.ahdr.datalen;
    }

    //give the descriptor to LLC
    s->desc = hci_env.ahdr.txtag;

    // Send the kernel message
    ke_msg_send(s);
    #endif // BLE_CENTRAL || BLE_PERIPHERAL
}


/*
 * FUNCTION DEFINITIONS - message sending at HCI
 ****************************************************************************************
 */

void hci_push(struct ke_msg *msg)
{
    // Push the message into the list of messages pending for transmission
    co_list_push_back(&hci_env.queue_tx, &msg->hdr);

    // Check if there is no transmission ongoing
    if (hci_env.tx_state == HCI_STATE_TX_IDLE)
    {
        // Forward the message to the HCIH EIF for immediate transmission
        hci_eif_write(msg);
    }
}

#if (DEEP_SLEEP)
bool hci_enter_sleep(void)
{
    uint8_t sleep_ok = false;

    do
    {
        // Check if there is activity on RX or TX
        if ((hci_env.tx_state != HCI_STATE_TX_IDLE) ||
            ((hci_env.rx_state != HCI_STATE_RX_START) && (hci_env.rx_state != HCI_STATE_RX_OUT_OF_SYNC)))
            break;

        // Now tries to stop the flow on the interface
        sleep_ok = hci_eif_stop();
    } while (false);

    if(rwip_ext_wakeup_enable())
    {
        if(sleep_ok)
        {
            // Clear the HCI polling timer
            ke_timer_clear(HCI_POLLING_TO, TASK_HCI);
        }
        else
        {
            // Program another checking of the HCI interface
            ke_timer_set(HCI_POLLING_TO, TASK_HCI, HCI_SLEEP_POLLING_INTERVAL);
        }
    }

    return sleep_ok;
}

void hci_exit_sleep(void)
{
    // Restart UART interface
    hci_eif_start();
}
#endif //DEEP_SLEEP

#endif //BLE_HCIC_ITF

/// @} HCI
