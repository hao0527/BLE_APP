/**
 ****************************************************************************************
 * @file    app_dfu.c
 * @brief

 * @version 0.01
 * @date    2018/11/27
 * @history
 * @note
 * detailed description

 ****************************************************************************************
 * @attention
 *
 * Copyright (C) 2016 Shanghai Panchip Microelectronics Co., Ltd. All rights reserved.
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup OTA APP
 * @{
 ****************************************************************************************
 */
#include "panip_config.h"     // SW configuration

#include "app_ota.h"
#include "ota_server.h"
#include "ota_server_task.h"

#include "gapm_task.h"
#include "att.h"
#include "co_utils.h"
#include "gattc_task.h"
#include "prf_types.h"
#include "string.h"
#include "stack_svc_api.h"

#if(PROJ_OTA)
static struct app_ota_env_tag app_ota_env;

void app_ota_init ( void )
{
    memset ( &app_ota_env, 0, sizeof ( app_ota_env ) );
}


void app_ota_add_server ( void )
{
    struct ota_server_db_cfg* db_cfg;

    // Allocate the OTA_CREATE_DB_REQ
    struct gapm_profile_task_add_cmd* req = KE_MSG_ALLOC_DYN ( GAPM_PROFILE_TASK_ADD_CMD,
                                                               TASK_GAPM, TASK_APP,
                                                               gapm_profile_task_add_cmd, sizeof ( struct ota_server_db_cfg ) );
    // Fill message
    req->operation	 = GAPM_PROFILE_TASK_ADD;
    req->sec_lvl	 = ( PERM ( SVC_AUTH, DISABLE ) | PERM ( SVC_UUID_LEN, UUID_16 ) | PERM ( SVC_EKS, DISABLE ) | PERM ( SVC_DIS, DISABLE ) );

    req->prf_task_id = TASK_ID_OTA;
    req->app_task	 = TASK_APP;
    req->start_hdl	 = 0;

    // Set parameters
    db_cfg = ( struct ota_server_db_cfg* ) req->param;
    db_cfg->features = 0x1fff;

    // Send the message
    ((ke_msg_send_handler)SVC_ke_msg_send) ( req );
}


void app_ota_enable_server_prf ( uint8_t conidx )
{
    app_ota_env.conidx = conidx;

    // Allocate the message
    struct ota_server_enable_req* req = KE_MSG_ALLOC ( OTA_SERVER_ENABLE_REQ,
                                                       prf_get_task_from_id ( TASK_ID_OTA ),
                                                       TASK_APP,
                                                       ota_server_enable_req );
    // Fill in the parameter structure
    req->conidx             = conidx;

    // NTF initial status - Disabled
    req->ntf_cfg           = PRF_CLI_STOP_NTFIND;

    // Send the message
    ((ke_msg_send_handler)SVC_ke_msg_send) ( req );
}

static int ota_server_peer_write_data_ind_handler ( ke_msg_id_t const msgid,
                                                    struct ota_server_peer_write_data_ind const* param,
                                                    ke_task_id_t const dest_id,
                                                    ke_task_id_t const src_id )
{
#if 1
    printf ( "recv %d\n", param->packet_size );
#else
    uint8_t Sendata[PROJ_TEMPLATE_SERVER_PACKET_SIZE] = {0};

    show_reg3 ( param->packet, param->packet_size );
    memcpy ( Sendata, param->packet, param->packet_size );
    app_proj_template_send_value ( PROJ_TEMPLATE_IDX_15, Sendata, param->packet_size );
#endif
    return ( KE_MSG_CONSUMED );
}

static int ota_server_enable_rsp_handler ( ke_msg_id_t const msgid,
                                           struct ota_server_enable_rsp const* param,
                                           ke_task_id_t const dest_id,
                                           ke_task_id_t const src_id )
{
    return ( KE_MSG_CONSUMED );
}

static int app_ota_msg_dflt_handler ( ke_msg_id_t const msgid,
                                      void const* param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id )
{
    // Drop the message
    return ( KE_MSG_CONSUMED );
}

/// Default State handlers definition
const struct ke_msg_handler app_ota_msg_handler_list[] =
{
    // Note: first message is latest message checked by kernel so default is put on top.
    {KE_MSG_DEFAULT_HANDLER,            ( ke_msg_func_t ) app_ota_msg_dflt_handler},
    {OTA_SERVER_PEER_WRITE_DATA_IND,    ( ke_msg_func_t ) ota_server_peer_write_data_ind_handler},
    {OTA_SERVER_ENABLE_RSP,             ( ke_msg_func_t ) ota_server_enable_rsp_handler},
};

const struct ke_state_handler app_ota_table_handler =
{
    app_ota_msg_handler_list, ( sizeof ( app_ota_msg_handler_list ) / sizeof ( struct ke_msg_handler ) )
};
#endif
