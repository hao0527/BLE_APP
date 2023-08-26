
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
 * LOCAL FUNCTIONS DECLARATIONS
 ****************************************************************************************
 */

/*
 * If Transport layer is present, some commands/events parameters with special format (variable content) needs
 * specific function for packing or unpacking. The specific function may support packing only, unpacking only, or both.
 *
 * Function types:
 *   - pkupk => functions used to pack or unpack parameters (depending on the Host<->Controller direction supported)
 *   - upk   => functions used to unpack parameters
 *   - pk    => functions used to pack parameters
 *
 * The support of packing or unpacking depends on the Host<->Controller direction supported by each commands/event:
 *  - for commands supported in LE:
 *      - Split-Host configuration          -> command parameters are packed / return parameters are unpacked
 *      - Split-Emb or full configuration   -> command parameters are unpacked / return parameters are packed
 *  - for events supported in LE:
 *      - Split-Host configuration          -> event parameters are unpacked
 *      - Split-Emb or full configuration   -> event parameters are packed
 *  - for commands supported in BT only:
 *                                          -> command parameters are unpacked / return parameters are packed
 *  - for events supported in BT only:
 *                                          -> event parameters are packed
 */

#if (HCI_TL_SUPPORT)
static uint8_t hci_host_nb_cmp_pkts_cmd_pkupk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len);

#if (BLE_EMB_PRESENT || BLE_HOST_PRESENT)
static uint8_t hci_le_set_ext_scan_param_cmd_upk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len);
static uint8_t hci_le_ext_create_con_cmd_upk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len);
static uint8_t hci_le_set_ext_adv_data_cmd_upk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len);
static uint8_t hci_le_set_ext_scan_rsp_data_cmd_upk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len);
static uint8_t hci_le_set_ext_adv_en_cmd_upk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len);
static uint8_t hci_le_set_per_adv_data_cmd_upk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len);
static uint8_t hci_le_adv_report_evt_pkupk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len);
static uint8_t hci_le_dir_adv_report_evt_pkupk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len);
static uint8_t hci_le_ext_adv_report_evt_pkupk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len);
static uint8_t hci_le_periodic_adv_report_evt_pkupk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len);
#endif //(BLE_EMB_PRESENT || BLE_HOST_PRESENT)
#if (RW_DEBUG &&  (BLE_EMB_PRESENT || BT_EMB_PRESENT))
static uint8_t hci_dbg_wr_mem_cmd_upk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len);
static uint8_t hci_dbg_wr_flash_cmd_upk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len);
static uint8_t hci_dbg_wr_par_cmd_upk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len);
static uint8_t hci_dbg_rd_data_cmd_cmp_evt_pk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len);
static uint8_t hci_dbg_assert_err_evt_pkupk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len);
#endif //(RW_DEBUG && (BLE_EMB_PRESENT || BT_EMB_PRESENT))
#if CRYPTO_UT
static uint8_t hci_dbg_test_crypto_func_cmd_upk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len);
#endif //CRYPTO_UT
#endif //(HCI_TL_SUPPORT)

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */


/// HCI command descriptors (OGF link control)
const struct hci_cmd_desc_tag hci_cmd_desc_tab_lk_ctrl[] =
{

    CMD(DISCONNECT                 , CTRL        , HL_CTRL, PK_GEN_GEN, 3 , "HB"      , NULL  ),
    CMD(RD_REM_VER_INFO            , CTRL        , HL_CTRL, PK_GEN_GEN, 2 , "H"       , NULL  ),
};

///// HCI command descriptors (OGF link policy)


/// HCI command descriptors (OGF controller and baseband)
const struct hci_cmd_desc_tag hci_cmd_desc_tab_ctrl_bb[] =
{
    CMD(SET_EVT_MASK                 , MNG      , HL_MNG , PK_GEN_GEN, 8  , "8B"   , "B"      ),
    CMD(RESET                        , MNG      , HL_MNG , PK_GEN_GEN, 0  , NULL   , "B"      ),
    CMD(RD_TX_PWR_LVL                , CTRL     , HL_CTRL, PK_GEN_GEN, 3  , "HB"   , "BHB"    ),
    CMD(SET_CTRL_TO_HOST_FLOW_CTRL   , MNG      , HL_MNG , PK_GEN_GEN, 1  , "B"    , "B"      ),
    CMD(HOST_BUF_SIZE                , MNG      , HL_MNG , PK_GEN_GEN, 7  , "HBHH" , "B"      ),
    CMD(HOST_NB_CMP_PKTS             , MNG      , HL_MNG , PK_SPE_GEN, 30 , &hci_host_nb_cmp_pkts_cmd_pkupk, "B"  ),
    CMD(RD_AUTH_PAYL_TO              , CTRL     , HL_CTRL, PK_GEN_GEN, 2  , "H"    , "BHH"    ),
    CMD(WR_AUTH_PAYL_TO              , CTRL     , HL_CTRL, PK_GEN_GEN, 4  , "HH"   , "BH"     ),
    CMD(SET_EVT_MASK_PAGE_2          , MNG      , HL_MNG , PK_GEN_GEN, 8  , "8B"   , "B"      ),

};

/// HCI command descriptors (OGF informational parameters)
const struct hci_cmd_desc_tag hci_cmd_desc_tab_info_par[] =
{
    CMD(RD_LOCAL_VER_INFO   , MNG   , HL_MNG , PK_GEN_GEN, 0, NULL, "BBHBHH"),
    CMD(RD_LOCAL_SUPP_CMDS  , MNG   , HL_MNG , PK_GEN_GEN, 0, NULL, "B64B"  ),
    CMD(RD_LOCAL_SUPP_FEATS , MNG   , HL_MNG , PK_GEN_GEN, 0, NULL, "B8B"   ),
    CMD(RD_BUFF_SIZE        , MNG   , HL_MNG , PK_GEN_GEN, 0, NULL, "BHBHH" ),
    CMD(RD_BD_ADDR          , MNG   , HL_MNG , PK_GEN_GEN, 0, NULL, "B6B"   ),

};

/// HCI command descriptors (OGF status parameters)
const struct hci_cmd_desc_tag hci_cmd_desc_tab_stat_par[] =
{
    CMD(RD_RSSI             , CTRL          , HL_CTRL, PK_GEN_GEN, 2, "H" , "BHB"   ),

};

///// HCI command descriptors (OGF testing)


#if (BLE_EMB_PRESENT || BLE_HOST_PRESENT)
/// HCI command descriptors (OGF LE controller)
const struct hci_cmd_desc_tag hci_cmd_desc_tab_le[] =
{
    CMD(LE_SET_EVT_MASK               , BLE_MNG , HL_MNG , PK_GEN_GEN, 8 , "8B"           , "B"     ),
    CMD(LE_RD_BUFF_SIZE               , BLE_MNG , HL_MNG , PK_GEN_GEN, 0 , NULL           , "BHB"   ),
    CMD(LE_RD_LOCAL_SUPP_FEATS        , BLE_MNG , HL_MNG , PK_GEN_GEN, 0 , NULL           , "B8B"   ),
    CMD(LE_SET_RAND_ADDR              , BLE_MNG , HL_MNG , PK_GEN_GEN, 6 , "6B"           , "B"     ),
    CMD(LE_SET_ADV_PARAM              , BLE_MNG , HL_MNG , PK_GEN_GEN, 15, "HHBBB6BBB"    , "B"     ),
    CMD(LE_RD_ADV_CHNL_TX_PW          , BLE_MNG , HL_MNG , PK_GEN_GEN, 0 , NULL           , "BB"    ),
    CMD(LE_SET_ADV_DATA               , BLE_MNG , HL_MNG , PK_GEN_GEN, 32, "B31B"         , "B"     ),
    CMD(LE_SET_SCAN_RSP_DATA          , BLE_MNG , HL_MNG , PK_GEN_GEN, 32, "B31B"         , "B"     ),
    CMD(LE_SET_ADV_EN                 , BLE_MNG , HL_MNG , PK_GEN_GEN, 1 , "B"            , "B"     ),
    CMD(LE_SET_SCAN_PARAM             , BLE_MNG , HL_MNG , PK_GEN_GEN, 7 , "BHHBB"        , "B"     ),
    CMD(LE_SET_SCAN_EN                , BLE_MNG , HL_MNG , PK_GEN_GEN, 2 , "BB"           , "B"     ),
    CMD(LE_CREATE_CON                 , BLE_MNG , HL_MNG , PK_GEN_GEN, 25, "HHBB6BBHHHHHH", NULL    ),
    CMD(LE_CREATE_CON_CANCEL          , BLE_MNG , HL_MNG , PK_GEN_GEN, 0 , NULL           , "B"     ),
    CMD(LE_RD_WLST_SIZE               , BLE_MNG , HL_MNG , PK_GEN_GEN, 0 , NULL           , "BB"    ),
    CMD(LE_CLEAR_WLST                 , BLE_MNG , HL_MNG , PK_GEN_GEN, 0 , NULL           , "B"     ),
    CMD(LE_ADD_DEV_TO_WLST            , BLE_MNG , HL_MNG , PK_GEN_GEN, 7 , "B6B"          , "B"     ),
    CMD(LE_RMV_DEV_FROM_WLST          , BLE_MNG , HL_MNG , PK_GEN_GEN, 7 , "B6B"          , "B"     ),
    CMD(LE_CON_UPDATE                 , BLE_CTRL, HL_CTRL, PK_GEN_GEN, 14, "HHHHHHH"      , NULL    ),
    CMD(LE_SET_HOST_CH_CLASS          , BLE_MNG , HL_MNG , PK_GEN_GEN, 5 , "5B"           , "B"     ),
    CMD(LE_RD_CHNL_MAP                , BLE_CTRL, HL_CTRL, PK_GEN_GEN, 2 , "H"            , "BH5B"  ),
    CMD(LE_RD_REM_FEATS               , BLE_CTRL, HL_CTRL, PK_GEN_GEN, 2 , "H"            , NULL    ),
    CMD(LE_ENC                        , BLE_MNG , HL_MNG , PK_GEN_GEN, 32, "16B16B"       , "B16B"  ),
    CMD(LE_RAND                       , BLE_MNG , HL_MNG , PK_GEN_GEN, 0 , NULL           , "B8B"   ),
    CMD(LE_START_ENC                  , BLE_CTRL, HL_CTRL, PK_GEN_GEN, 28, "H8BH16B"      , NULL    ),
    CMD(LE_LTK_REQ_REPLY              , BLE_CTRL, HL_CTRL, PK_GEN_GEN, 18, "H16B"         , "BH"    ),
    CMD(LE_LTK_REQ_NEG_REPLY          , BLE_CTRL, HL_CTRL, PK_GEN_GEN, 2 , "H"            , "BH"    ),
    CMD(LE_RD_SUPP_STATES             , BLE_MNG , HL_MNG , PK_GEN_GEN, 0 , NULL           , "B8B"   ),
    CMD(LE_RX_TEST                    , BLE_MNG , HL_MNG , PK_GEN_GEN, 1 , "B"            , "B"     ),
    CMD(LE_TX_TEST                    , BLE_MNG , HL_MNG , PK_GEN_GEN, 3 , "BBB"          , "B"     ),
    CMD(LE_TEST_END                   , BLE_MNG , HL_MNG , PK_GEN_GEN, 0 , NULL           , "BH"    ),
    CMD(LE_REM_CON_PARAM_REQ_REPLY    , BLE_CTRL, HL_CTRL, PK_GEN_GEN, 14, "HHHHHHH"      , "BH"    ),
    CMD(LE_REM_CON_PARAM_REQ_NEG_REPLY, BLE_CTRL, HL_CTRL, PK_GEN_GEN, 3 , "HB"           , "BH"    ),
    CMD(LE_SET_DATA_LEN               , BLE_CTRL, HL_CTRL, PK_GEN_GEN, 6 , "HHH"          , "BH"    ),
    CMD(LE_RD_SUGGTED_DFT_DATA_LEN    , BLE_MNG , HL_MNG , PK_GEN_GEN, 0 , NULL           , "BHH"   ),
    CMD(LE_WR_SUGGTED_DFT_DATA_LEN    , BLE_MNG , HL_MNG , PK_GEN_GEN, 4 , "HH"           , "B"     ),
    CMD(LE_RD_LOC_P256_PUB_KEY        , BLE_MNG , HL_MNG , PK_GEN_GEN, 0 , NULL           , NULL    ),
    CMD(LE_GEN_DHKEY                  , BLE_MNG , HL_MNG , PK_GEN_GEN, 64, "64B"          , NULL    ),
    CMD(LE_ADD_DEV_TO_RSLV_LIST       , BLE_MNG , HL_MNG , PK_GEN_GEN, 39, "B6B16B16B"    , "B"     ),
    CMD(LE_RMV_DEV_FROM_RSLV_LIST     , BLE_MNG , HL_MNG , PK_GEN_GEN, 7 , "B6B"          , "B"     ),
    CMD(LE_CLEAR_RSLV_LIST            , BLE_MNG , HL_MNG , PK_GEN_GEN, 0 , NULL           , "B"     ),
    CMD(LE_RD_RSLV_LIST_SIZE          , BLE_MNG , HL_MNG , PK_GEN_GEN, 0 , NULL           , "BB"    ),
    CMD(LE_RD_PEER_RSLV_ADDR          , BLE_MNG , HL_MNG , PK_GEN_GEN, 7 , "B6B"          , "B6B"   ),
    CMD(LE_RD_LOC_RSLV_ADDR           , BLE_MNG , HL_MNG , PK_GEN_GEN, 7 , "B6B"          , "B6B"   ),
    CMD(LE_SET_ADDR_RESOL_EN          , BLE_MNG , HL_MNG , PK_GEN_GEN, 1 , "B"            , "B"     ),
    CMD(LE_SET_RSLV_PRIV_ADDR_TO      , BLE_MNG , HL_MNG , PK_GEN_GEN, 2 , "H"            , "B"     ),
    CMD(LE_RD_MAX_DATA_LEN            , BLE_MNG , HL_MNG , PK_GEN_GEN, 0 , NULL           , "BHHHH" ),
    CMD(LE_RD_PHY                     , BLE_CTRL, HL_CTRL, PK_GEN_GEN, 2 , "H"            , "BHBB"  ),
    CMD(LE_SET_DFT_PHY                , BLE_MNG , HL_MNG , PK_GEN_GEN, 3 , "BBB"          , "B"     ),
    CMD(LE_SET_PHY                    , BLE_CTRL, HL_CTRL, PK_GEN_GEN, 7 , "HBBBH"        , NULL    ),
    CMD(LE_ENH_RX_TEST                , BLE_MNG , HL_MNG , PK_GEN_GEN, 3 , "BBB"          , "B"     ),
    CMD(LE_ENH_TX_TEST                , BLE_MNG , HL_MNG , PK_GEN_GEN, 4 , "BBBB"         , "B"     ),
    CMD(LE_SET_ADV_SET_RAND_ADDR      , BLE_MNG , HL_MNG , PK_GEN_GEN, 7 , "B6B"          , "B"     ),
    CMD(LE_SET_EXT_ADV_PARAM          , BLE_MNG , HL_MNG , PK_GEN_GEN, 25, "BH3B3BBBB6BBBBBBBB"    , "BB"     ),
    CMD(LE_SET_EXT_ADV_DATA           , BLE_MNG , HL_MNG , PK_SPE_GEN, 255, &hci_le_set_ext_adv_data_cmd_upk, "B"     ),
    CMD(LE_SET_EXT_SCAN_RSP_DATA      , BLE_MNG , HL_MNG , PK_SPE_GEN, 255, &hci_le_set_ext_scan_rsp_data_cmd_upk, "B"     ),
    CMD(LE_SET_EXT_ADV_EN             , BLE_MNG , HL_MNG , PK_SPE_GEN, 42, &hci_le_set_ext_adv_en_cmd_upk, "B"     ),
    CMD(LE_RD_MAX_ADV_DATA_LEN        , BLE_MNG , HL_MNG , PK_GEN_GEN, 0 , NULL           , "BH"    ),
    CMD(LE_RD_NB_SUPP_ADV_SETS        , BLE_MNG , HL_MNG , PK_GEN_GEN, 0 , NULL           , "BB"    ),
    CMD(LE_RMV_ADV_SET                , BLE_MNG , HL_MNG , PK_GEN_GEN, 1 , "B"            , "B"     ),
    CMD(LE_CLEAR_ADV_SETS             , BLE_MNG , HL_MNG , PK_GEN_GEN, 0 , NULL           , "B"     ),
    CMD(LE_SET_PER_ADV_PARAM          , BLE_MNG , HL_MNG , PK_GEN_GEN, 7 , "BHHH"         , "B"     ),
    CMD(LE_SET_PER_ADV_DATA           , BLE_MNG , HL_MNG , PK_SPE_GEN, 255, &hci_le_set_per_adv_data_cmd_upk, "B"     ),
    CMD(LE_SET_PER_ADV_EN             , BLE_MNG , HL_MNG , PK_GEN_GEN, 2 , "BB"           , "B"     ),
    CMD(LE_SET_EXT_SCAN_PARAM         , BLE_MNG , HL_MNG , PK_SPE_GEN, 13, &hci_le_set_ext_scan_param_cmd_upk, "B"    ),
    CMD(LE_SET_EXT_SCAN_EN            , BLE_MNG , HL_MNG , PK_GEN_GEN, 6, "BBHH"          , "B"     ),
    CMD(LE_EXT_CREATE_CON             , BLE_MNG , HL_MNG , PK_SPE_GEN, 30, &hci_le_ext_create_con_cmd_upk, NULL ),
    CMD(LE_PER_ADV_CREATE_SYNC        , BLE_MNG , HL_MNG , PK_GEN_GEN, 13, "BBB6BHH"      , NULL    ),
    CMD(LE_PER_ADV_CREATE_SYNC_CANCEL , BLE_MNG , HL_MNG , PK_GEN_GEN, 0 , NULL           , "B"     ),
    CMD(LE_PER_ADV_TERM_SYNC          , BLE_MNG , HL_MNG , PK_GEN_GEN, 2 , "H"            , "B"     ),
    CMD(LE_ADD_DEV_TO_PER_ADV_LIST    , BLE_MNG , HL_MNG , PK_GEN_GEN, 8 , "B6BB"         , "B"     ),
    CMD(LE_RMV_DEV_FROM_PER_ADV_LIST  , BLE_MNG , HL_MNG , PK_GEN_GEN, 8 , "B6BB"         , "B"     ),
    CMD(LE_CLEAR_PER_ADV_LIST         , BLE_MNG , HL_MNG , PK_GEN_GEN, 0 , NULL           , "B"     ),
    CMD(LE_RD_PER_ADV_LIST_SIZE       , BLE_MNG , HL_MNG , PK_GEN_GEN, 0 , NULL           , "B"     ),
    CMD(LE_RD_TX_PWR                  , BLE_MNG , HL_MNG , PK_GEN_GEN, 0 , NULL           , "BBB"   ),
    CMD(LE_SET_PRIV_MODE              , BLE_MNG , HL_MNG , PK_GEN_GEN, 8 , "B6BB"         , "B"     ),
};
#endif //(BLE_EMB_PRESENT || BLE_HOST_PRESENT)

#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
///// HCI command descriptors (OGF Vendor Specific)
const struct hci_cmd_desc_tag hci_cmd_desc_tab_vs[] =
{
    #if (RW_DEBUG && (!BLE_HOST_PRESENT || HCI_TL_SUPPORT))
    CMD(DBG_RD_MEM           , DBG, 0, PK_GEN_SPE, 6  , "LBB"                      , &hci_dbg_rd_data_cmd_cmp_evt_pk),
    CMD(DBG_WR_MEM           , DBG, 0, PK_SPE_GEN, 136, &hci_dbg_wr_mem_cmd_upk    , "B"                            ),
    CMD(DBG_DEL_PAR          , DBG, 0, PK_GEN_GEN, 2  , "H"                        , "B"                            ),
    CMD(DBG_ID_FLASH         , DBG, 0, PK_GEN_GEN, 0  , NULL                       , "BB"                           ),
    CMD(DBG_ER_FLASH         , DBG, 0, PK_GEN_GEN, 9  , "BLL"                      , "B"                            ),
    CMD(DBG_WR_FLASH         , DBG, 0, PK_SPE_GEN, 140, &hci_dbg_wr_flash_cmd_upk  , "B"                            ),
    CMD(DBG_RD_FLASH         , DBG, 0, PK_GEN_SPE, 6  , "BLB"                      , &hci_dbg_rd_data_cmd_cmp_evt_pk),
    CMD(DBG_RD_PAR           , DBG, 0, PK_GEN_SPE, 2  , "H"                        , &hci_dbg_rd_data_cmd_cmp_evt_pk),
    CMD(DBG_WR_PAR           , DBG, 0, PK_SPE_GEN, 132, &hci_dbg_wr_par_cmd_upk    , "B"                            ),
    #endif //(RW_DEBUG && (!BLE_HOST_PRESENT || HCI_TL_SUPPORT))
    #if (RW_DEBUG)
    CMD(DBG_RD_KE_STATS      , DBG, 0, PK_GEN_GEN, 0  , NULL                       , "BBBBHH"                       ),
    #if (RW_WLAN_COEX)
    CMD(DBG_WLAN_COEX        , DBG, 0, PK_GEN_GEN, 1  , "B"                        , "B"                            ),
    #if (RW_WLAN_COEX_TEST)
    CMD(DBG_WLAN_COEXTST_SCEN, DBG, 0, PK_GEN_GEN, 4  , "L"                        , "B"                            ),
    #endif //RW_BT_WLAN_COEX_TEST
    #endif //RW_WLAN_COEX
    #if (RW_MWS_COEX)
    CMD(DBG_MWS_COEX        , DBG, 0, PK_GEN_GEN, 1  , "B"                        , "B"                            ),
    #if (RW_MWS_COEX_TEST)
    CMD(DBG_MWS_COEXTST_SCEN, DBG, 0, PK_GEN_GEN, 4  , "L"                        , "B"                            ),
    #endif //RW_BT_MWS_COEX_TEST
    #endif //RW_MWS_COEX
    #endif //(RW_DEBUG)
    CMD(DBG_PLF_RESET        , DBG, 0, PK_GEN_GEN, 1  , "B"                        , "B"                            ),
    CMD(DBG_RD_MEM_INFO      , DBG, 0, PK_GEN_GEN, 0  , NULL                       , "BHHHL"                        ),
    #if BLE_EMB_PRESENT
    #if (RW_DEBUG)
    CMD(DBG_HW_REG_RD        , DBG, 0, PK_GEN_GEN, 2  , "H"                        , "BHL"                          ),
    CMD(DBG_HW_REG_WR        , DBG, 0, PK_GEN_GEN, 6  , "HL"                       , "BH"                           ),
    CMD(DBG_SET_BD_ADDR      , DBG, 0, PK_GEN_GEN, 6  , "6B"                       , "B"                            ),
    CMD(DBG_SET_CRC          , DBG, 0, PK_GEN_GEN, 5  , "H3B"                      , "B"                            ),
    CMD(DBG_LLCP_DISCARD     , BLE_CTRL, 0, PK_GEN_GEN, 3  , "HB"                       , "B"                            ),
    CMD(DBG_RESET_RX_CNT     , DBG, 0, PK_GEN_GEN, 2  , "H"                        , "B"                            ),
    CMD(DBG_RESET_TX_CNT     , DBG, 0, PK_GEN_GEN, 2  , "H"                        , "B"                            ),
    CMD(DBG_RF_REG_RD        , DBG, 0, PK_GEN_GEN, 2  , "H"                        , "BHL"                          ),
    CMD(DBG_RF_REG_WR        , DBG, 0, PK_GEN_GEN, 6  , "HL"                       , "BHE"                          ),
    #endif //(RW_DEBUG)
    #if (BLE_TESTER)
    CMD(TESTER_SET_LE_PARAMS,   BLE_CTRL, 0,PK_GEN_GEN, 0,  "HBBHHHHHH"               , NULL  ),
    CMD(DBG_BLE_TST_LLCP_PT_EN, BLE_CTRL, 0,PK_GEN_GEN, 3,  "HB"                      , NULL  ),
    CMD(DBG_BLE_TST_SEND_LLCP,  BLE_CTRL, 0,PK_GEN_GEN, 29, "HB26B"                   , NULL  ),
    #endif// (BLE_TESTER)
    CMD(DBG_SET_TX_PW        , DBG, 0, PK_GEN_GEN, 3  , "HB"                        , "B"                            ),
    #if (BLE_AUDIO)
    CMD(DBG_AUDIO_CONFIGURE  , DBG, 0,PK_GEN_GEN, 14,  "BBBBBBBBBB"                 , "B"                            ),
    CMD(DBG_AUDIO_SET_MODE   , DBG, 0,PK_GEN_GEN,  2,  "BB"                         , "B"                            ),
    CMD(DBG_AUDIO_RESET      , DBG, 0,PK_GEN_GEN,  1,  "B"                          , "B"                            ),
    CMD(DBG_AUDIO_SET_POINTER, DBG, 0,PK_GEN_GEN,  5,  "BBBH"                       , "B"                            ),
    CMD(DBG_AUDIO_ALLOCATE   , DBG, 0,PK_GEN_GEN,  3,  "HB"                         , "B"                            ),
    CMD(DBG_AUDIO_GET_VX_CH  , DBG, 0,PK_GEN_GEN,  2,  "H"                          , "B"                            ),
    #endif
    #endif //BLE_EMB_PRESENT

    #if CRYPTO_UT
    CMD(DBG_TEST_CRYPTO_FUNC , BT_MNG, 0, PK_SPE_GEN, 136, &hci_dbg_test_crypto_func_cmd_upk, "B"                   ),
    #endif //CRYPTO_UT
};
#endif //(BLE_EMB_PRESENT || BT_EMB_PRESENT)

/// HCI command descriptors root table (classified by OGF)
const struct hci_cmd_desc_tab_ref hci_cmd_desc_root_tab[] =
{
    {LK_CNTL_OGF,  ARRAY_LEN(hci_cmd_desc_tab_lk_ctrl) , hci_cmd_desc_tab_lk_ctrl },
    {CNTLR_BB_OGF, ARRAY_LEN(hci_cmd_desc_tab_ctrl_bb) , hci_cmd_desc_tab_ctrl_bb },
    {INFO_PAR_OGF, ARRAY_LEN(hci_cmd_desc_tab_info_par), hci_cmd_desc_tab_info_par},
    {STAT_PAR_OGF, ARRAY_LEN(hci_cmd_desc_tab_stat_par), hci_cmd_desc_tab_stat_par},

    #if (BLE_EMB_PRESENT || BLE_HOST_PRESENT)
    {LE_CNTLR_OGF, ARRAY_LEN(hci_cmd_desc_tab_le)      , hci_cmd_desc_tab_le      },
    #endif //(BLE_EMB_PRESENT || BLE_HOST_PRESENT)
    #if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
    {VS_OGF,       ARRAY_LEN(hci_cmd_desc_tab_vs)      , hci_cmd_desc_tab_vs      },
    #endif //(BLE_EMB_PRESENT || BT_EMB_PRESENT)
};

/// HCI event descriptors
const struct hci_evt_desc_tag hci_evt_desc_tab[] =
{
    EVT(DISC_CMP                 , HL_CTRL, PK_GEN, "BHB"          ),
    EVT(ENC_CHG                  , HL_CTRL, PK_GEN, "BHB"          ),
    EVT(RD_REM_VER_INFO_CMP      , HL_CTRL, PK_GEN, "BHBHH"        ),
    EVT(HW_ERR                   , HL_MNG , PK_GEN, "B"            ),
    EVT(FLUSH_OCCURRED           , HL_CTRL, PK_GEN, "H"            ),
    EVT(NB_CMP_PKTS              , HL_DATA, PK_GEN, "BHH"          ),
    EVT(DATA_BUF_OVFLW           , HL_MNG , PK_GEN, "B"            ),
    EVT(ENC_KEY_REFRESH_CMP      , HL_CTRL, PK_GEN, "BH"           ),
    EVT(AUTH_PAYL_TO_EXP         , HL_CTRL, PK_GEN, "H"            ),
   
};


// Note: remove specific BLE Flag as soon as new debug event available on BT

#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
#if (RW_DEBUG || (BLE_EMB_PRESENT && BLE_TESTER))
/// HCI DBG event descriptors
const struct hci_evt_desc_tag hci_evt_dbg_desc_tab[] =
{
    #if (BLE_EMB_PRESENT)
    #if (BLE_TESTER)
    DBG_EVT(DBG_BLE_TST_LLCP_RECV, HL_CTRL, PK_GEN, "BHB26B"),
    #endif // (BLE_TESTER)
    #endif // (BLE_EMB_PRESENT)

    #if (RW_DEBUG)
    DBG_EVT(DBG_ASSERT_ERR       , 0      , PK_SPE, &hci_dbg_assert_err_evt_pkupk ),
    #endif //(RW_DEBUG)
};
#endif // (RW_DEBUG || (BLE_EMB_PRESENT && BLE_TESTER))
#endif // (BLE_EMB_PRESENT || BT_EMB_PRESENT)


#if (BLE_EMB_PRESENT || BLE_HOST_PRESENT)
/// HCI LE event descriptors
const struct hci_evt_desc_tag hci_evt_le_desc_tab[] =
{
    LE_EVT(LE_CON_CMP,                 HL_MNG , PK_GEN, "BBHBB6BHHHB"             ),
    LE_EVT(LE_ADV_REPORT,              HL_MNG , PK_SPE, &hci_le_adv_report_evt_pkupk ),
    LE_EVT(LE_CON_UPDATE_CMP,          HL_CTRL, PK_GEN, "BBHHHH"                  ),
    LE_EVT(LE_RD_REM_FEATS_CMP,        HL_CTRL, PK_GEN, "BBH8B"                   ),
    LE_EVT(LE_LTK_REQUEST,             HL_CTRL, PK_GEN, "BH8BH"                   ),
    LE_EVT(LE_REM_CON_PARAM_REQ,       HL_CTRL, PK_GEN, "BHHHHH"                  ),
    LE_EVT(LE_DATA_LEN_CHG,            HL_CTRL, PK_GEN, "BHHHHH"                  ),
    LE_EVT(LE_ENH_CON_CMP,             HL_MNG , PK_GEN, "BBHBB6B6B6BHHHB"             ),
    #if (SECURE_CONNECTIONS==1)
    LE_EVT(LE_RD_LOC_P256_PUB_KEY_CMP, HL_MNG , PK_GEN, "BB64B"                   ),
    LE_EVT(LE_GEN_DHKEY_CMP,           HL_MNG , PK_GEN, "BB32B"                   ),
    #endif //(SECURE_CONNECTIONS==1)
    LE_EVT(LE_DIR_ADV_REP,             HL_MNG , PK_SPE, &hci_le_dir_adv_report_evt_pkupk),
    LE_EVT(LE_PHY_UPD_CMP,             HL_CTRL, PK_GEN, "BBHBB"             ),
    LE_EVT(LE_EXT_ADV_REPORT,          HL_MNG,  PK_SPE, &hci_le_ext_adv_report_evt_pkupk),
    LE_EVT(LE_PERIODIC_ADV_SYNC_EST,   HL_MNG,  PK_GEN, "BBHBB6BBHB"              ),
    LE_EVT(LE_PERIODIC_ADV_REPORT,     HL_MNG,  PK_SPE, &hci_le_periodic_adv_report_evt_pkupk),
    LE_EVT(LE_PERIODIC_ADV_SYNC_LOST,  HL_MNG,  PK_GEN, "BH"                      ),
    LE_EVT(LE_SCAN_TIMEOUT,            HL_MNG,  PK_GEN, "B"                       ),
    LE_EVT(LE_ADV_SET_TERMINATED,      HL_MNG,  PK_GEN, "BBBHB"                   ),
    LE_EVT(LE_SCAN_REQ_RCVD,           HL_MNG,  PK_GEN, "BBB6B"                   ),
    LE_EVT(LE_CH_SEL_ALGO,             HL_CTRL, PK_GEN, "BHB"                     ),
};
#endif //(BLE_EMB_PRESENT || BLE_HOST_PRESENT)


/*
 * SPECIAL PACKER-UNPACKER DEFINITIONS
 ****************************************************************************************
 */

#if (HCI_TL_SUPPORT)
#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
/**
****************************************************************************************
* @brief Apply a basic pack operation
*
* @param[inout] pp_in      Current input buffer position
* @param[inout] pp_out     Current output buffer position
* @param[in]    p_in_end   Input buffer end
* @param[in]    p_out_end  Output buffer end
* @param[in]    len        Number of bytes to copy
*
* @return status
*****************************************************************************************
*/
static uint8_t hci_pack_bytes(uint8_t** pp_in, uint8_t** pp_out, uint8_t* p_in_end, uint8_t* p_out_end, uint8_t len)
{
    uint8_t status = HCI_PACK_OK;

    // Check if enough space in input buffer to read
    if((*pp_in + len) > p_in_end)
    {
        status = HCI_PACK_IN_BUF_OVFLW;
    }
    else
    {
        if(p_out_end != NULL)
        {
            // Check if enough space in out buffer to write
            if((*pp_out + len) > p_out_end)
            {
                status = HCI_PACK_OUT_BUF_OVFLW;
            }

            // Copy BD Address
            memcpy(*pp_out, *pp_in, len);
        }
        *pp_in = *pp_in + len;
        *pp_out = *pp_out + len;
    }

    return (status);
}
#endif //(BLE_EMB_PRESENT || BT_EMB_PRESENT)

/// Special packing/unpacking function for HCI Host Number Of Completed Packets Command
static uint8_t hci_host_nb_cmp_pkts_cmd_pkupk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len)
{
    #if (BLE_EMB_PRESENT || BT_EMB_PRESENT)

    /*
     * PACKING FUNCTION
     */
    struct hci_host_nb_cmp_pkts_cmd* cmd = (struct hci_host_nb_cmp_pkts_cmd*) out;
    uint8_t* p_in = in;
    uint8_t* p_out = out;
    uint8_t* p_in_end = in + in_len;
    uint8_t* p_out_end = out + *out_len;
    uint8_t status = HCI_PACK_OK;

    // Check if there is input data to parse
    if(in != NULL)
    {
        do
        {
            uint8_t i = 0;
            // Number of handles
            p_out = &cmd->nb_of_hdl;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            while((i < *in) && ((p_out - out) <= sizeof(struct hci_host_nb_cmp_pkts_cmd)))
            {
                // Connection handle
                p_out = (uint8_t*) &cmd->con_hdl[i];
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 2);
                if(status != HCI_PACK_OK)
                    break;

                // Host_Num_Of_Completed_Packets
                p_out = (uint8_t*) &cmd->nb_comp_pkt[i];
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 2);
                if(status != HCI_PACK_OK)
                    break;

                i++;
            }
        } while(0);

        *out_len =  (uint16_t)(p_out - out);
    }
    else
    {
        // If no input data, size max is returned
        *out_len =  sizeof(struct hci_host_nb_cmp_pkts_cmd);
    }

    return (status);

    #elif BLE_HOST_PRESENT

    /*
     * UNPACKING FUNCTION
     */
    return sizeof(struct hci_host_nb_cmp_pkts_cmd);

    #endif //(BLE_EMB_PRESENT || BT_EMB_PRESENT)
}



#if (BLE_EMB_PRESENT || BLE_HOST_PRESENT)
/// Special packing/unpacking function for HCI LE Set Extended Scan Parameters command
static uint8_t hci_le_set_ext_scan_param_cmd_upk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len)
{
    #if BLE_EMB_PRESENT

    /*
     * PACKING FUNCTION
     */
    struct hci_le_set_ext_scan_param_cmd temp_out;
    struct hci_le_set_ext_scan_param_cmd* cmd;
    uint8_t* p_in = in;
    uint8_t* p_out;
    uint8_t* p_in_end = in + in_len;
    uint8_t* p_out_start;
    uint8_t* p_out_end;
    bool copy_data;
    uint8_t status = HCI_PACK_OK;

    // Check if there is input data to parse
    if (in != NULL)
    {
        // Check if there is output buffer to write to, else use temp_out buffer
        if (out != NULL)
        {
            cmd = (struct hci_le_set_ext_scan_param_cmd*)(out);
            p_out_start = out;
            p_out_end = out + *out_len;
            copy_data = true;
        }
        else
        {
            cmd = (struct hci_le_set_ext_scan_param_cmd*)(&temp_out);
            p_out_start = (uint8_t*)&temp_out;
            p_out_end = p_out_start + sizeof(temp_out);
            copy_data = false;
        }

        do
        {
            uint8_t  num_scan_phys = 0;
            int i;

            // Own addr type
            p_out = (uint8_t*) &cmd->own_addr_type;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            // Scan filter policy
            p_out = (uint8_t*) &cmd->scan_filt_policy;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            //Scanning PHYs
            p_out = (uint8_t*)&cmd->scan_phys;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            for (uint8_t bit_flags = cmd->scan_phys; bit_flags; bit_flags >>= 1)
            { //count num scan_phys bits set
                 if (bit_flags & 1)
                 {
                     num_scan_phys++;
                 }
            }

            ASSERT_ERR(num_scan_phys <= MAX_SCAN_PHYS);

            if(!copy_data)
            {
                p_out_end = NULL;
            }

            for (i=0; i < num_scan_phys; i++)
            {
                // Scanning type
                p_out = (uint8_t*) &cmd->phy[i].scan_type;
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
                if(status != HCI_PACK_OK)
                    break;

                // Scan interval
                p_out = (uint8_t*) &cmd->phy[i].scan_intv;
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 2);
                if(status != HCI_PACK_OK)
                    break;

                // Scan window
                p_out = (uint8_t*) &cmd->phy[i].scan_window;
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 2);
                if(status != HCI_PACK_OK)
                    break;
            }

        } while(0);

        *out_len =  (uint16_t)(p_out - p_out_start);
    }
    else
    {
        // If no input data, size max is returned
        *out_len = sizeof(struct hci_le_set_ext_scan_param_cmd);
    }

    return (status);

    #elif BLE_HOST_PRESENT

    /*
     * UNPACKING FUNCTION
     */
    //TODO unpack message as per compiler
    *out_len = in_len;
    if((out != NULL) && (in != NULL))
    {
        // copy command
        memcpy(out, in, in_len);
    }
    
    return (HCI_PACK_OK);

    #endif //BLE_EMB_PRESENT
}

/// Special packing/unpacking function for HCI LE Extended Create Connection command
static uint8_t hci_le_ext_create_con_cmd_upk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len)
{
    #if BLE_EMB_PRESENT

    /*
     * PACKING FUNCTION
     */
    struct hci_le_ext_create_con_cmd temp_out;
    struct hci_le_ext_create_con_cmd* cmd;
    uint8_t* p_in = in;
    uint8_t* p_out;
    uint8_t* p_in_end = in + in_len;
    uint8_t* p_out_start;
    uint8_t* p_out_end;
    bool copy_data;
    uint8_t status = HCI_PACK_OK;

    // Check if there is input data to parse
    if (in != NULL)
    {
        // Check if there is output buffer to write to, else use temp_out buffer
        if (out != NULL)
        {
            cmd = (struct hci_le_ext_create_con_cmd*)(out);
            p_out_start = out;
            p_out_end = out + *out_len;
            copy_data = true;
        }
        else
        {
            cmd = (struct hci_le_ext_create_con_cmd*)(&temp_out);
            p_out_start = (uint8_t*)&temp_out;
            p_out_end = p_out_start + sizeof(temp_out);
            copy_data = false;
        }

        do
        {
            uint8_t  num_init_phys = 0;
            int i;

            // Initiator filter policy
            p_out = (uint8_t*) &cmd->init_filter_policy;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            // Own address type
            p_out = (uint8_t*) &cmd->own_addr_type;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            // Peer address type
            p_out = (uint8_t*) &cmd->peer_addr_type;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            // Peer address
            p_out = (uint8_t*) &cmd->peer_addr.addr[0];
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, BD_ADDR_LEN);
            if(status != HCI_PACK_OK)
                break;

            //Initiating PHYs
            p_out = (uint8_t*)&cmd->init_phys;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            for (uint8_t bit_flags = cmd->init_phys; bit_flags; bit_flags >>= 1)
            { //count num init_phys bits set
                 if (bit_flags & 1)
                 {
                     num_init_phys++;
                 }
            }

            ASSERT_ERR(num_init_phys <= MAX_INIT_PHYS);

            if(!copy_data)
            {
                p_out_end = NULL;
            }

            for (i=0; i < num_init_phys; i++)
            {
                // Scan interval
                p_out = (uint8_t*) &cmd->phy[i].scan_interval;
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 2);
                if(status != HCI_PACK_OK)
                    break;

                // Scan window
                p_out = (uint8_t*) &cmd->phy[i].scan_window;
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 2);
                if(status != HCI_PACK_OK)
                    break;

                // Min connection interval
                p_out = (uint8_t*)&cmd->phy[i].con_intv_min;
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 2);
                if (status != HCI_PACK_OK)
                    break;

                // Max connection interval
                p_out = (uint8_t*)&cmd->phy[i].con_intv_max;
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 2);
                if (status != HCI_PACK_OK)
                    break;

                // Connection latency
                p_out = (uint8_t*)&cmd->phy[i].con_latency;
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 2);
                if (status != HCI_PACK_OK)
                    break;

                // Link supervision timeout
                p_out = (uint8_t*)&cmd->phy[i].superv_to;
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 2);
                if (status != HCI_PACK_OK)
                    break;

                // Min CE length
                p_out = (uint8_t*)&cmd->phy[i].ce_len_min;
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 2);
                if (status != HCI_PACK_OK)
                    break;

                // Max CE length
                p_out = (uint8_t*)&cmd->phy[i].ce_len_max;
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 2);
                if (status != HCI_PACK_OK)
                    break;
            }
        } while(0);

        *out_len =  (uint16_t)(p_out - p_out_start);
    }
    else
    {
        // If no input data, size max is returned
        *out_len = sizeof(struct hci_le_ext_create_con_cmd);
    }

    return (status);

    #elif BLE_HOST_PRESENT

    /*
     * UNPACKING FUNCTION
     */
    //TODO unpack message as per compiler
    *out_len = in_len;
    if((out != NULL) && (in != NULL))
    {
        // copy command
        memcpy(out, in, in_len);
    }
    
    return (HCI_PACK_OK);

    #endif //BLE_EMB_PRESENT
}

/// Special packing/unpacking function for HCI LE Set Extended Advertising Data command
static uint8_t hci_le_set_ext_adv_data_cmd_upk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len)
{
    #if BLE_EMB_PRESENT

    /*
     * PACKING FUNCTION
     */
    struct hci_le_set_ext_adv_data_cmd temp_out;
    struct hci_le_set_ext_adv_data_cmd* cmd;
    uint8_t* p_in = in;
    uint8_t* p_out;
    uint8_t* p_in_end = in + in_len;
    uint8_t* p_out_start;
    uint8_t* p_out_end;
    bool copy_data;
    uint8_t status = HCI_PACK_OK;

    // Check if there is input data to parse
    if (in != NULL)
    {
        // Check if there is output buffer to write to, else use temp_out buffer
        if (out != NULL)
        {
            cmd = (struct hci_le_set_ext_adv_data_cmd*)(out);
            p_out_start = out;
            p_out_end = out + *out_len;
            copy_data = true;
        }
        else
        {
            cmd = (struct hci_le_set_ext_adv_data_cmd*)(&temp_out);
            p_out_start = (uint8_t*)&temp_out;
            p_out_end = p_out_start + sizeof(temp_out);
            copy_data = false;
        }

        do
        {
            // Advertising handle
            p_out = (uint8_t*) &cmd->adv_hdl;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            // Operation
            p_out = (uint8_t*) &cmd->operation;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            // Fragment preference
            p_out = (uint8_t*)&cmd->frag_pref;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            // Advertising data length
            p_out = (uint8_t*)&cmd->data_len;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            ASSERT_ERR(cmd->data_len <= 251);

            if(!copy_data)
            {
                p_out_end = NULL;
            }

            // Advertising data
            p_out = (uint8_t*)&cmd->data;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, cmd->data_len);
            if(status != HCI_PACK_OK)
                break;

        } while(0);

        *out_len =  (uint16_t)(p_out - p_out_start);
    }
    else
    {
        // If no input data, size max is returned
        *out_len = sizeof(struct hci_le_set_ext_adv_data_cmd);
    }

    return (status);

    #elif BLE_HOST_PRESENT

    /*
     * UNPACKING FUNCTION
     */
    //TODO unpack message as per compiler
    *out_len = in_len;
    if((out != NULL) && (in != NULL))
    {
        // copy command
        memcpy(out, in, in_len);
    }

    return (HCI_PACK_OK);

    #endif //BLE_EMB_PRESENT
}

/// Special packing/unpacking function for HCI LE Set Extended Scan Response Data command
static uint8_t hci_le_set_ext_scan_rsp_data_cmd_upk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len)
{
    #if BLE_EMB_PRESENT

    /*
     * PACKING FUNCTION
     */
    struct hci_le_set_ext_scan_rsp_data_cmd temp_out;
    struct hci_le_set_ext_scan_rsp_data_cmd* cmd;
    uint8_t* p_in = in;
    uint8_t* p_out;
    uint8_t* p_in_end = in + in_len;
    uint8_t* p_out_start;
    uint8_t* p_out_end;
    bool copy_data;
    uint8_t status = HCI_PACK_OK;

    // Check if there is input data to parse
    if (in != NULL)
    {
        // Check if there is output buffer to write to, else use temp_out buffer
        if (out != NULL)
        {
            cmd = (struct hci_le_set_ext_scan_rsp_data_cmd*)(out);
            p_out_start = out;
            p_out_end = out + *out_len;
            copy_data = true;
        }
        else
        {
            cmd = (struct hci_le_set_ext_scan_rsp_data_cmd*)(&temp_out);
            p_out_start = (uint8_t*)&temp_out;
            p_out_end = p_out_start + sizeof(temp_out);
            copy_data = false;
        }

        do
        {
            // Advertising handle
            p_out = (uint8_t*) &cmd->adv_hdl;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            // Operation
            p_out = (uint8_t*) &cmd->operation;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            // Fragment preference
            p_out = (uint8_t*)&cmd->frag_pref;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            // Advertising data length
            p_out = (uint8_t*)&cmd->data_len;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            ASSERT_ERR(cmd->data_len <= 251);

            if(!copy_data)
            {
                p_out_end = NULL;
            }

            // Advertising data
            p_out = (uint8_t*)&cmd->data;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, cmd->data_len);
            if(status != HCI_PACK_OK)
                break;

        } while(0);

        *out_len =  (uint16_t)(p_out - p_out_start);
    }
    else
    {
        // If no input data, size max is returned
        *out_len = sizeof(struct hci_le_set_ext_scan_rsp_data_cmd);
    }

    return (status);

    #elif BLE_HOST_PRESENT

    /*
     * UNPACKING FUNCTION
     */
    //TODO unpack message as per compiler
    *out_len = in_len;
    if((out != NULL) && (in != NULL))
    {
        // copy command
        memcpy(out, in, in_len);
    }

    return (HCI_PACK_OK);

    #endif //BLE_EMB_PRESENT
}

/// Special packing/unpacking function for HCI LE Set Extended Advertising Enable command
static uint8_t hci_le_set_ext_adv_en_cmd_upk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len)
{
    #if BLE_EMB_PRESENT

    /*
     * PACKING FUNCTION
     */
    struct hci_le_set_ext_adv_en_cmd temp_out;
    struct hci_le_set_ext_adv_en_cmd* cmd;
    uint8_t* p_in = in;
    uint8_t* p_out;
    uint8_t* p_in_end = in + in_len;
    uint8_t* p_out_start;
    uint8_t* p_out_end;
    bool copy_data;
    uint8_t status = HCI_PACK_OK;

    // Check if there is input data to parse
    if (in != NULL)
    {
        // Check if there is output buffer to write to, else use temp_out buffer
        if (out != NULL)
        {
            cmd = (struct hci_le_set_ext_adv_en_cmd*)(out);
            p_out_start = out;
            p_out_end = out + *out_len;
            copy_data = true;
        }
        else
        {
            cmd = (struct hci_le_set_ext_adv_en_cmd*)(&temp_out);
            p_out_start = (uint8_t*)&temp_out;
            p_out_end = p_out_start + sizeof(temp_out);
            copy_data = false;
        }

        do
        {
            // Enable
            p_out = (uint8_t*) &cmd->enable;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            // Number of sets
            p_out = (uint8_t*) &cmd->nb_sets;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            ASSERT_ERR(cmd->nb_sets <= 63);

            if(!copy_data)
            {
                p_out_end = NULL;
            }


            for (uint8_t i = 0; i < cmd->nb_sets; i++)
            {
                // Advertising handle
                p_out = (uint8_t*)&cmd->adv_hdl[i];
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
                if(status != HCI_PACK_OK)
                    break;

                // Duration
                p_out = (uint8_t*)&cmd->duration[i];
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 2);
                if(status != HCI_PACK_OK)
                    break;

                // Maximum number of extended advertising events
                p_out = (uint8_t*)&cmd->max_ext_adv_evt[i];
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
                if(status != HCI_PACK_OK)
                    break;
            }

        } while(0);

        *out_len =  (uint16_t)(p_out - p_out_start);
    }
    else
    {
        // If no input data, size max is returned
        *out_len = sizeof(struct hci_le_set_ext_adv_en_cmd);
    }

    return (status);

    #elif BLE_HOST_PRESENT

    /*
     * UNPACKING FUNCTION
     */
    //TODO unpack message as per compiler
    *out_len = in_len;
    if((out != NULL) && (in != NULL))
    {
        // copy command
        memcpy(out, in, in_len);
    }

    return (HCI_PACK_OK);

    #endif //BLE_EMB_PRESENT
}

/// Special packing/unpacking function for HCI LE Set Periodic Advertising Data command
static uint8_t hci_le_set_per_adv_data_cmd_upk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len)
{
    #if BLE_EMB_PRESENT

    /*
     * PACKING FUNCTION
     */
    struct hci_le_set_per_adv_data_cmd temp_out;
    struct hci_le_set_per_adv_data_cmd* cmd;
    uint8_t* p_in = in;
    uint8_t* p_out;
    uint8_t* p_in_end = in + in_len;
    uint8_t* p_out_start;
    uint8_t* p_out_end;
    bool copy_data;
    uint8_t status = HCI_PACK_OK;

    // Check if there is input data to parse
    if (in != NULL)
    {
        // Check if there is output buffer to write to, else use temp_out buffer
        if (out != NULL)
        {
            cmd = (struct hci_le_set_per_adv_data_cmd*)(out);
            p_out_start = out;
            p_out_end = out + *out_len;
            copy_data = true;
        }
        else
        {
            cmd = (struct hci_le_set_per_adv_data_cmd*)(&temp_out);
            p_out_start = (uint8_t*)&temp_out;
            p_out_end = p_out_start + sizeof(temp_out);
            copy_data = false;
        }

        do
        {
            // Advertising handle
            p_out = (uint8_t*) &cmd->adv_hdl;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            // Operation
            p_out = (uint8_t*) &cmd->operation;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            // Advertising data length
            p_out = (uint8_t*)&cmd->data_len;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            ASSERT_ERR(cmd->data_len <= 252);

            if(!copy_data)
            {
                p_out_end = NULL;
            }

            // Advertising data
            p_out = (uint8_t*)&cmd->data;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, cmd->data_len);
            if(status != HCI_PACK_OK)
                break;

        } while(0);

        *out_len =  (uint16_t)(p_out - p_out_start);
    }
    else
    {
        // If no input data, size max is returned
        *out_len = sizeof(struct hci_le_set_per_adv_data_cmd);
    }

    return (status);

    #elif BLE_HOST_PRESENT

    /*
     * UNPACKING FUNCTION
     */
    //TODO unpack message as per compiler
    *out_len = in_len;
    if((out != NULL) && (in != NULL))
    {
        // copy command
        memcpy(out, in, in_len);
    }

    return (HCI_PACK_OK);

    #endif //BLE_EMB_PRESENT
}

/// Special packing/unpacking function for HCI LE Advertising Report Event
static uint8_t hci_le_adv_report_evt_pkupk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len)
{
    #if BLE_EMB_PRESENT

    /*
     * PACKING FUNCTION
     */
    struct hci_le_adv_report_evt temp_out;
    struct hci_le_adv_report_evt* s;
    uint8_t* p_in = in;
    uint8_t* p_out;
    uint8_t* p_in_end = in + in_len;
    uint8_t* p_out_start;
    uint8_t* p_out_end;
    uint8_t status = HCI_PACK_OK;

    // Check if there is input data to parse
    if(in != NULL)
    {
        // Check if there is output buffer to write to, else use temp_out buffer
        if (out != NULL)
        {
            s = (struct hci_le_adv_report_evt*)(out);
            p_out_start = out;
            p_out_end = out + *out_len;
        }
        else
        {
            s = (struct hci_le_adv_report_evt*)(&temp_out);
            p_out_start = (uint8_t*)&temp_out;
            p_out_end = p_out_start + sizeof(temp_out);
        }

        p_out = p_out_start;

        do
        {
            // Sub-code
            p_in = &s->subcode;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            // Number of reports
            p_in = &s->nb_reports;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            for (int i=0; i< s->nb_reports; i++)
            {
                uint8_t data_len;

                // Event type
                p_in = &s->adv_rep[i].evt_type;
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
                if(status != HCI_PACK_OK)
                    break;

                // Address type
                p_in = &s->adv_rep[i].adv_addr_type;
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
                if(status != HCI_PACK_OK)
                    break;

                // BD Address
                p_in = &s->adv_rep[i].adv_addr.addr[0];
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, BD_ADDR_LEN);
                if(status != HCI_PACK_OK)
                    break;

                // Data Length
                p_in = &s->adv_rep[i].data_len;
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
                if(status != HCI_PACK_OK)
                    break;

                data_len = s->adv_rep[i].data_len;

                // ADV data
                p_in = &s->adv_rep[i].data[0];
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, data_len);
                if(status != HCI_PACK_OK)
                    break;

                // RSSI
                p_in = &s->adv_rep[i].rssi;
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
                if(status != HCI_PACK_OK)
                    break;
            }

        } while(0);

        *out_len =  (uint16_t)(p_out - p_out_start);
    }
    else
    {
        *out_len = 0;
    }

    return (status);

    #elif BLE_HOST_PRESENT

    /*
     * UNPACKING FUNCTION
     */

    // TODO unpack message as per compiler
    *out_len = in_len;
    if((out != NULL) && (in != NULL))
    {
        // copy adv report
        memcpy(out, in, in_len);
    }

    return (HCI_PACK_OK);

    #endif //BLE_EMB_PRESENT
}

/// Special packing/unpacking function for HCI LE Direct Advertising Report Event
static uint8_t hci_le_dir_adv_report_evt_pkupk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len)
{
    #if BLE_EMB_PRESENT

    /*
     * PACKING FUNCTION
     */
    struct hci_le_dir_adv_rep_evt temp_out;
    struct hci_le_dir_adv_rep_evt* s;
    uint8_t* p_in = in;
    uint8_t* p_out;
    uint8_t* p_in_end = in + in_len;
    uint8_t* p_out_start;
    uint8_t* p_out_end;
    uint8_t status = HCI_PACK_OK;

    // Check if there is input data to parse
    if(in != NULL)
    {
        // Check if there is output buffer to write to, else use temp_out buffer
        if (out != NULL)
        {
            s = (struct hci_le_dir_adv_rep_evt*)(out);
            p_out_start = out;
            p_out_end = out + *out_len;
        }
        else
        {
            s = (struct hci_le_dir_adv_rep_evt*)(&temp_out);
            p_out_start = (uint8_t*)&temp_out;
            p_out_end = p_out_start + sizeof(temp_out);
        }

        p_out = p_out_start;

        do
        {
            // Sub-code
            p_in = &s->subcode;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            // Number of reports
            p_in = &s->nb_reports;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            for (int i=0; i< s->nb_reports; i++)
            {
                // Event type
                p_in = &s->adv_rep[i].evt_type;
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
                if(status != HCI_PACK_OK)
                    break;


                // Address type
                p_in = &s->adv_rep[i].addr_type;
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
                if(status != HCI_PACK_OK)
                    break;

                // BD Address
                p_in = &s->adv_rep[i].addr.addr[0];
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, BD_ADDR_LEN);
                if(status != HCI_PACK_OK)
                    break;

                // Direct Address type
                p_in = &s->adv_rep[i].dir_addr_type;
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
                if(status != HCI_PACK_OK)
                    break;

                // Direct BD Address
                p_in = &s->adv_rep[i].dir_addr.addr[0];
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, BD_ADDR_LEN);
                if(status != HCI_PACK_OK)
                    break;

                // RSSI
                p_in = &s->adv_rep[i].rssi;
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
                if(status != HCI_PACK_OK)
                    break;
            }

        } while(0);

        *out_len =  (uint16_t)(p_out - p_out_start);
    }
    else
    {
        *out_len = 0;
    }

    return (status);

    #elif BLE_HOST_PRESENT

    /*
     * UNPACKING FUNCTION
     */

    // TODO unpack message as per compiler
    *out_len = in_len;
    if((out != NULL) && (in != NULL))
    {
        // copy adv report
        memcpy(out, in, in_len);
    }

    return (HCI_PACK_OK);

    #endif //BLE_EMB_PRESENT
}

/// Special packing/unpacking function for HCI LE Extended Advertising Report Event
static uint8_t hci_le_ext_adv_report_evt_pkupk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len)
{
    #if BLE_EMB_PRESENT

    /*
     * PACKING FUNCTION
     */
    struct hci_le_ext_adv_report_evt temp_out;
    struct hci_le_ext_adv_report_evt* s;
    uint8_t* p_in = in;
    uint8_t* p_out;
    uint8_t* p_in_end = in + in_len;
    uint8_t* p_out_start;
    uint8_t* p_out_end;
    uint8_t status = HCI_PACK_OK;

    // Check if there is input data to parse
    if(in != NULL)
    {
        // Check if there is output buffer to write to, else use temp_out buffer
        if (out != NULL)
        {
            s = (struct hci_le_ext_adv_report_evt*)(out);
            p_out_start = out;
            p_out_end = out + *out_len;
        }
        else
        {
            s = (struct hci_le_ext_adv_report_evt*)(&temp_out);
            p_out_start = (uint8_t*)&temp_out;
            p_out_end = p_out_start + sizeof(temp_out);
        }

        p_out = p_out_start;

        do
        {
            uint8_t data_len;

            // Sub-code
            p_in = &s->subcode;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            // Number of reports
            p_in = &s->nb_reports;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            for (int i=0; i< s->nb_reports; i++)
            {
                // Event type
                p_in = (uint8_t*) &s->adv_rep[i].evt_type;
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 2);
                if(status != HCI_PACK_OK)
                    break;

                // Adv Address type
                p_in = &s->adv_rep[i].adv_addr_type;
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
                if(status != HCI_PACK_OK)
                    break;

                // Adv Address
                p_in = &s->adv_rep[i].adv_addr.addr[0];
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, BD_ADDR_LEN);
                if(status != HCI_PACK_OK)
                    break;

                // Primary PHY
                p_in = &s->adv_rep[i].phy;
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
                if(status != HCI_PACK_OK)
                    break;

                // Secondary PHY
                p_in = &s->adv_rep[i].phy2;
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
                if(status != HCI_PACK_OK)
                    break;

                // Advertising SID
                p_in = &s->adv_rep[i].adv_sid;
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
                if(status != HCI_PACK_OK)
                    break;

                // Tx Power
                p_in = &s->adv_rep[i].tx_power;
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
                if(status != HCI_PACK_OK)
                    break;

                // RSSI
                p_in = &s->adv_rep[i].rssi;
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
                if(status != HCI_PACK_OK)
                    break;

                // Periodic Advertising Interval
                p_in = (uint8_t*) &s->adv_rep[i].interval;
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 2);
                if(status != HCI_PACK_OK)
                    break;

                // Direct address type
                p_in = (uint8_t*) &s->adv_rep[i].dir_addr_type;
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
                if(status != HCI_PACK_OK)
                    break;

                // Direct BD Address
                p_in = &s->adv_rep[i].dir_addr.addr[0];
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, BD_ADDR_LEN);
                if(status != HCI_PACK_OK)
                    break;

                // Data Length
                p_in = &s->adv_rep[i].data_len;
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
                if(status != HCI_PACK_OK)
                    break;

                data_len = s->adv_rep[i].data_len;

                // ADV data
                p_in = &s->adv_rep[i].data[0];
                status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, data_len);
                if(status != HCI_PACK_OK)
                    break;
            }

        } while(0);

        *out_len =  (uint16_t)(p_out - p_out_start);
    }
    else
    {
        *out_len = 0;
    }

    return (status);

    #elif BLE_HOST_PRESENT

    /*
     * UNPACKING FUNCTION
     */
    //TODO unpack message as per compiler
    *out_len = in_len;
    if((out != NULL) && (in != NULL))
    {
        // copy adv report
        memcpy(out, in, in_len);
    }
    
    return (HCI_PACK_OK);

    #endif //BLE_EMB_PRESENT
}

/// Special packing/unpacking function for HCI LE Periodic Advertising Report Event
static uint8_t hci_le_periodic_adv_report_evt_pkupk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len)
{
    #if BLE_EMB_PRESENT

    struct hci_le_per_adv_report_evt temp_out;
    struct hci_le_per_adv_report_evt* s;
    uint8_t* p_in = in;
    uint8_t* p_out;
    uint8_t* p_in_end = in + in_len;
    uint8_t* p_out_start;
    uint8_t* p_out_end;
    uint8_t status = HCI_PACK_OK;

    // Check if there is input data to parse
    if(in != NULL)
    {
        // Check if there is output buffer to write to, else use temp_out buffer
        if (out != NULL)
        {
            s = (struct hci_le_per_adv_report_evt*)(out);
            p_out_start = out;
            p_out_end = out + *out_len;
        }
        else
        {
            s = (struct hci_le_per_adv_report_evt*)(&temp_out);
            p_out_start = (uint8_t*)&temp_out;
            p_out_end = p_out_start + sizeof(temp_out);
        }

        p_out = p_out_start;

        do
        {
            uint8_t data_len;

            // Sub-code
            p_in = &s->subcode;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            // Sync handle
            p_in = (uint8_t*) &s->sync_handle;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 2);
            if(status != HCI_PACK_OK)
                break;

            // Tx Power
            p_in = &s->tx_power;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            // RSSI
            p_in = &s->rssi;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            // Data Status
            p_in = &s->status;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            // Data Length
            p_in = &s->data_len;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            data_len = s->data_len;

            // ADV data
            p_in = &s->data[0];
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, data_len);
            if(status != HCI_PACK_OK)
                break;

            // Unused
            p_in = &s->unused; // PER_ADV_REPORT_TRAIL_BYTE
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

        } while(0);

        *out_len =  (uint16_t)(p_out - p_out_start);
    }
    else
    {
        *out_len = 0;
    }

    return (status);

    #elif BLE_HOST_PRESENT

    /*
     * UNPACKING FUNCTION
     */
    //TODO unpack message as per compiler
    *out_len = in_len;
    if((out != NULL) && (in != NULL))
    {
        // copy adv report
        memcpy(out, in, in_len);
    }

    return (HCI_PACK_OK);

    #endif //BLE_EMB_PRESENT
}

#endif //(BLE_EMB_PRESENT || BLE_HOST_PRESENT)

#if (RW_DEBUG && (BLE_EMB_PRESENT || BT_EMB_PRESENT))
/// Special packing/unpacking function for HCI Debug Write Memory Command
static uint8_t hci_dbg_wr_mem_cmd_upk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len)
{
    struct hci_dbg_wr_mem_cmd* cmd = (struct hci_dbg_wr_mem_cmd*) out;
    uint8_t* p_in = in;
    uint8_t* p_out = out;
    uint8_t* p_in_end = in + in_len;
    uint8_t* p_out_end = out + *out_len;
    uint8_t status = HCI_PACK_OK;

    // Check if there is input data to parse
    if(in != NULL)
    {
        do
        {
            uint8_t data_len;

            // Start address
            p_out = (uint8_t*) &cmd->start_addr;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 4);
            if(status != HCI_PACK_OK)
                break;

            // Memory access type
            p_out = &cmd->type;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            // Data length
            data_len = *p_in;
            p_out = &cmd->buf.length;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            // Data
            p_out = &cmd->buf.data[0];
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, data_len);
            if(status != HCI_PACK_OK)
                break;

        } while(0);

        *out_len =  (uint16_t)(p_out - out);
    }
    else
    {
        // If no input data, size max is returned
        *out_len = sizeof(struct hci_dbg_wr_mem_cmd);
    }

    return (status);
}

/// Special packing/unpacking function for HCI Debug Write Flash Command
static uint8_t hci_dbg_wr_flash_cmd_upk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len)
{
    struct hci_dbg_wr_flash_cmd* cmd = (struct hci_dbg_wr_flash_cmd*) out;
    uint8_t* p_in = in;
    uint8_t* p_out = out;
    uint8_t* p_in_end = in + in_len;
    uint8_t* p_out_end = out + *out_len;
    uint8_t status = HCI_PACK_OK;

    // Check if there is input data to parse
    if(in != NULL)
    {
        do
        {
            uint8_t data_len;

            // Flash type
            p_out = &cmd->flashtype;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            // Start offset
            p_out = (uint8_t*) &cmd->startoffset;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 4);
            if(status != HCI_PACK_OK)
                break;

            // Data length
            data_len = *p_in;
            p_out = &cmd->buf.length;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            // Data
            p_out = &cmd->buf.data[0];
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, data_len);
            if(status != HCI_PACK_OK)
                break;

        } while(0);

        *out_len =  (uint16_t)(p_out - out);
    }
    else
    {
        // If no input data, size max is returned
        *out_len = sizeof(struct hci_dbg_wr_flash_cmd);
    }

    return (status);
}

#if CRYPTO_UT
static uint8_t hci_dbg_test_crypto_func_cmd_upk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len)
{
	struct hci_dbg_test_crypto_func_cmd* cmd = (struct hci_dbg_test_crypto_func_cmd*) out;
    uint8_t* p_in = in;
    uint8_t* p_out = out;
    uint8_t* p_in_end = in + in_len;
    uint8_t* p_out_end = out + *out_len;
    uint8_t status = HCI_PACK_OK;

    // Check if there is input data to parse
    if(in != NULL)
    {
        do
        {
            uint8_t data_len;

            // Function type
              p_out = &cmd->function;
              status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
              if(status != HCI_PACK_OK)
                  break;

            // Data length
            data_len = *p_in;
            p_out = &cmd->buf.length;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            // Data
            p_out = &cmd->buf.data[0];
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, data_len);
            if(status != HCI_PACK_OK)
                break;

        } while(0);

        *out_len =  (uint16_t)(p_out - out);
    }
    else
    {
        // If no input data, size max is returned
        *out_len = sizeof(struct hci_dbg_test_crypto_func_cmd);
    }

    return (status);
}
#endif //CRYPTO_UT

/// Special packing/unpacking function for HCI Debug Write Param Command
static uint8_t hci_dbg_wr_par_cmd_upk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len)
{
    struct hci_dbg_wr_par_cmd* cmd = (struct hci_dbg_wr_par_cmd*) out;
    uint8_t* p_in = in;
    uint8_t* p_out = out;
    uint8_t* p_in_end = in + in_len;
    uint8_t* p_out_end = out + *out_len;
    uint8_t status = HCI_PACK_OK;

    // Check if there is input data to parse
    if(in != NULL)
    {
        do
        {
            uint8_t data_len;

            // Tag
            p_out = (uint8_t*) &cmd->param_tag;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 2);
            if(status != HCI_PACK_OK)
                break;

            // Data length
            data_len = *p_in;
            p_out = &cmd->buf.length;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            // Data
            p_out = &cmd->buf.data[0];
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, data_len);
            if(status != HCI_PACK_OK)
                break;

        } while(0);

        *out_len =  (uint16_t)(p_out - out);
    }
    else
    {
        // If no input data, size max is returned
        *out_len = sizeof(struct hci_dbg_wr_par_cmd);
    }

    return (status);
}

/// Special packing/unpacking function for Command Complete Event of HCI Debug Read Memory/Flash/Param Command
static uint8_t hci_dbg_rd_data_cmd_cmp_evt_pk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len)
{
    struct hci_dbg_basic_rd_data_cmd_cmp_evt* evt = (struct hci_dbg_basic_rd_data_cmd_cmp_evt*)(in);
    uint8_t* p_in = in;
    uint8_t* p_out = out;
    uint8_t* p_in_end = in + in_len;
    uint8_t* p_out_end = out + *out_len;
    uint8_t status = HCI_PACK_OK;

    // Check if there is input data to parse
    if(in != NULL)
    {
        do
        {
            uint8_t data_len;

            // Status
            p_in = &evt->status;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            // Data Length
            p_in = &evt->buf.length;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            data_len = evt->buf.length;

            // Data
            p_in = &evt->buf.data[0];
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, data_len);
            if(status != HCI_PACK_OK)
                break;

        } while(0);

        *out_len =  (uint16_t)(p_out - out);
    }
    else
    {
        *out_len = 0;
    }

    return (status);
}

/// Special packing/unpacking function for HCI DBG assert error Event
static uint8_t hci_dbg_assert_err_evt_pkupk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len)
{
    struct hci_dbg_assert_err_evt* evt = (struct hci_dbg_assert_err_evt*)(in);
    uint8_t* p_in = in;
    uint8_t* p_out = out;
    uint8_t* p_in_end = in + in_len;
    uint8_t* p_out_end = out + *out_len;
    uint8_t status = HCI_PACK_OK;

    // Check if there is input data to parse
    if(in != NULL)
    {
        do
        {
            // Subcode
            p_in = &evt->subcode;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            // Line
            p_in = (uint8_t*) &evt->line;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 4);
            if(status != HCI_PACK_OK)
                break;

            // Param0
            p_in = (uint8_t*) &evt->param0;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 4);
            if(status != HCI_PACK_OK)
                break;

            // Param1
            p_in = (uint8_t*) &evt->param1;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 4);
            if(status != HCI_PACK_OK)
                break;

            // File
            p_in = (uint8_t*) &evt->file;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, strlen((char*) evt->file));
            if(status != HCI_PACK_OK)
                break;
        } while(0);

        *out_len =  (uint16_t)(p_out - out);
    }
    else
    {
        *out_len = 0;
    }


    return (status);
}
#endif //(RW_DEBUG && (BLE_EMB_PRESENT || BT_EMB_PRESENT))
#endif //(HCI_TL_SUPPORT)


/*
 * MODULES INTERNAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

const struct hci_cmd_desc_tag* hci_look_for_cmd_desc(uint16_t opcode)
{
    const struct hci_cmd_desc_tag* tab = NULL;
    const struct hci_cmd_desc_tag* desc = NULL;
    uint16_t nb_cmds = 0;
    uint16_t index = 0;
    uint16_t ocf = HCI_OP2OCF(opcode);
    uint16_t ogf = HCI_OP2OGF(opcode);

    // Find table corresponding to this OGF
    for(index = 0; index < ARRAY_LEN(hci_cmd_desc_root_tab); index++)
    {
        // Compare the command opcodes
        if(hci_cmd_desc_root_tab[index].ogf == ogf)
        {
            // Get the command descriptors table information (size and pointer)
            tab = hci_cmd_desc_root_tab[index].cmd_desc_tab;
            nb_cmds = hci_cmd_desc_root_tab[index].nb_cmds;
            break;
        }
    }

    // Check if a table has been found for this OGF
    if(tab != NULL)
    {
        // Find the command descriptor associated to this OCF
        for(index = 0; index < nb_cmds; index++)
        {
            // Compare the command opcodes
            if(HCI_OP2OCF(tab->opcode) == ocf)
            {
                // Get the command descriptor pointer
                desc = tab;
                break;
            }

            // Jump to next command descriptor
            tab++;
        }
    }

    return (desc);
}

/**
****************************************************************************************
* @brief Look for an event descriptor that could match with the specified event code
*
* @param[in]  code   event code
*
* @return     Pointer the event descriptor (NULL if not found)
*****************************************************************************************
*/
const struct hci_evt_desc_tag* hci_look_for_evt_desc(uint8_t code)
{
    const struct hci_evt_desc_tag* desc = NULL;
    uint16_t index = 0;

    while(index < (sizeof(hci_evt_desc_tab)/sizeof(hci_evt_desc_tab[0])))
    {
        // Compare the command opcodes
        if(hci_evt_desc_tab[index].code == code)
        {
            // Get the event descriptor pointer
            desc = &hci_evt_desc_tab[index];
            break;
        }

        // Increment index
        index++;
    }

    return (desc);
}

#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
#if (RW_DEBUG || (BLE_EMB_PRESENT && BLE_TESTER))
const struct hci_evt_desc_tag* hci_look_for_dbg_evt_desc(uint8_t subcode)
{
    const struct hci_evt_desc_tag* desc = NULL;
    uint16_t index = 0;

    while(index < (sizeof(hci_evt_dbg_desc_tab)/sizeof(hci_evt_dbg_desc_tab[0])))
    {
        // Compare the command opcodes
        if(hci_evt_dbg_desc_tab[index].code == subcode)
        {
            // Get the event descriptor pointer
            desc = &hci_evt_dbg_desc_tab[index];
            break;
        }

        // Increment index
        index++;
    }

    return (desc);
}
#endif // (RW_DEBUG || (BLE_EMB_PRESENT && BLE_TESTER))
#endif // (BLE_EMB_PRESENT || BT_EMB_PRESENT)


#if (BLE_EMB_PRESENT || BLE_HOST_PRESENT)
const struct hci_evt_desc_tag* hci_look_for_le_evt_desc(uint8_t subcode)
{
    const struct hci_evt_desc_tag* desc = NULL;
    uint16_t index = 0;

    while(index < (sizeof(hci_evt_le_desc_tab)/sizeof(hci_evt_le_desc_tab[0])))
    {
        // Compare the command opcodes
        if(hci_evt_le_desc_tab[index].code == subcode)
        {
            // Get the event descriptor pointer
            desc = &hci_evt_le_desc_tab[index];
            break;
        }

        // Increment index
        index++;
    }

    return (desc);
}
#endif //(BLE_EMB_PRESENT || BLE_HOST_PRESENT)
#endif //(HCI_PRESENT)


/// @} HCI
