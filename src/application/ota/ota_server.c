/**
 ****************************************************************************************
 * @file    ota_server.c
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


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "prf.h"
#include "prf_utils.h"
#include "attm.h"
#include "ke_mem.h"

#include "ota_server_task.h"
#include "ota_server.h"
#include "stack_svc_api.h"

#if(PROJ_OTA)
const struct attm_desc ota_server_att_db[OTA_IDX_NB] =
{
    [OTA_IDX_SVC]               = { ATT_DECL_PRIMARY_SERVICE, PERM ( RD, ENABLE ), 0, 0 },

    [OTA_IDX_DATA_CHAR]         = { ATT_DECL_CHARACTERISTIC, PERM ( RD, ENABLE ), 0, 0 },
    [OTA_IDX_DATA_VAL]          = { OTA_CHAR_DATA_UUID, PERM ( WRITE_COMMAND, ENABLE ), PERM ( RI, ENABLE ), OTA_SERVER_PACKET_SIZE },

    [OTA_IDX_CTRL_CHAR]         = { ATT_DECL_CHARACTERISTIC, PERM ( RD, ENABLE ), 0, 0 },
    [OTA_IDX_CTRL_VAL]          = { OTA_CHAR_CTRL_UUID, PERM ( WRITE_REQ, ENABLE ) | PERM ( NTF, ENABLE ), PERM ( RI, ENABLE ), OTA_SERVER_PACKET_SIZE }, // PERM ( WRITE_COMMAND, ENABLE ) | 
    [OTA_IDX_CTRL_CLIENT_CFG]   = { ATT_DESC_CLIENT_CHAR_CFG, PERM ( RD, ENABLE ) | PERM ( WRITE_REQ, ENABLE ), 0, 0 },
};


/**
 ****************************************************************************************
 * @brief       Init of the SPS device
 * @param[in] Void
 * @return      None
 ****************************************************************************************
 */
static uint8_t ota_server_init ( struct prf_task_env* env, uint16_t* start_hdl, uint16_t app_task, uint8_t sec_lvl, struct ota_server_db_cfg* params )
{
    uint16_t shdl;
    struct ota_server_env_tag* p_env = NULL;
    uint8_t status = GAP_ERR_NO_ERROR;

    // Service content flag
    uint32_t cfg_flag = params->features;

    shdl = *start_hdl;
    status = attm_svc_create_db ( &shdl,
                                  OTA_SVC_UUID,
                                  ( uint8_t* ) &cfg_flag,
                                  OTA_IDX_NB,
                                  NULL,
                                  env->task,
                                  ota_server_att_db,
                                  ( sec_lvl & ( PERM_MASK_SVC_DIS | PERM_MASK_SVC_AUTH | PERM_MASK_SVC_EKS ) ) );

    if ( status == ATT_ERR_NO_ERROR )
    {
        p_env = ( struct ota_server_env_tag* ) ((ke_malloc_handler)SVC_ke_malloc) ( sizeof ( struct ota_server_env_tag ), KE_MEM_ATT_DB );
        memset ( p_env, 0, sizeof ( struct ota_server_env_tag ) );

        //set prf_env.prf[i]
        env->env = ( prf_env_t* ) p_env;
        env->id = TASK_ID_OTA;

        *start_hdl = shdl;
        p_env->start_hdl = *start_hdl;
        p_env->prf_env.app_task = app_task;
        //| (PERM_GET(sec_lvl, SVC_MI) ? PERM(PRF_MI, ENABLE) : PERM(PRF_MI, DISABLE));
        p_env->prf_env.prf_task = env->task;
        //| PERM(PRF_MI, DISABLE);

        ota_server_task_init ( & ( env->desc ) );
        // service is ready, go into an Idle state
        ((ke_state_set_handler)SVC_ke_state_set) ( env->task, OTA_SVS_IDLE );
    }

    return status;
}

static void ota_server_destroy ( struct prf_task_env* env )
{
    struct ota_server_env_tag* p_env = ( struct ota_server_env_tag* ) env->env;

    // clear on-going operation
    if ( p_env->operation != NULL )
    {
        ((ke_free_handler)SVC_ke_free) ( p_env->operation );
    }

    // free profile environment variables
    env->env = NULL;
    ((ke_free_handler)SVC_ke_free) ( p_env );
}

/**
 ****************************************************************************************
 * @brief Handles Connection creation
 *
 * @param[in|out]    env        Collector or Service allocated environment data.
 * @param[in]        conidx     Connection index
 ****************************************************************************************
 */
static void ota_server_create ( struct prf_task_env* env, uint8_t conidx )
{
    struct ota_server_env_tag* proj_template_server_env = ( struct ota_server_env_tag* ) env->env;

    ASSERT_ERR ( conidx < BLE_CONNECTION_MAX );

    proj_template_server_env->client_cfg_ctrl[conidx] = PRF_CLI_STOP_NTFIND;
}

/**
 ****************************************************************************************
 * @brief Handles Disconnection
 *
 * @param[in|out]    env        Collector or Service allocated environment data.
 * @param[in]        conidx     Connection index
 * @param[in]        reason     Detach reason
 ****************************************************************************************
 */
static void ota_server_cleanup ( struct prf_task_env* env, uint8_t conidx, uint8_t reason )
{
    struct ota_server_env_tag* p_env = ( struct ota_server_env_tag* ) env->env;

    ASSERT_ERR ( conidx < BLE_CONNECTION_MAX );

    // force notification config to zero when peer device is disconnected
    p_env->client_cfg_ctrl[conidx] = PRF_CLI_STOP_NTFIND;
}

/// OTA Task interface required by profile manager
const struct prf_task_cbs ota_srv_itf =
{
    ( prf_init_fnct ) ota_server_init,
    ota_server_destroy,
    ota_server_create,
    ota_server_cleanup,
};

const struct prf_task_cbs* ota_server_prf_itf_get ( void )
{
    return &ota_srv_itf;
}

uint16_t ota_server_get_att_handle ( uint8_t svc_idx, uint8_t att_idx )
{
    struct ota_server_env_tag* p_env = ( struct ota_server_env_tag* ) prf_env_get ( TASK_ID_OTA );
    
    uint16_t handle = p_env->start_hdl;

    if ( att_idx < OTA_IDX_NB )
    {
        handle += att_idx;
    }
    else
    {
        handle = ATT_INVALID_HDL;
    }

    return handle;
}

uint8_t ota_server_get_att_idx ( uint16_t handle, uint8_t* svc_idx, uint8_t* att_idx )
{
    struct ota_server_env_tag* p_env = ( struct ota_server_env_tag* ) prf_env_get ( TASK_ID_OTA );

    uint16_t hdl_cursor = p_env->start_hdl;
    uint8_t  status     = PRF_APP_ERROR;

    if ( handle < ( hdl_cursor + OTA_IDX_NB ) )
    {
        *att_idx = handle - hdl_cursor;
        status = GAP_ERR_NO_ERROR;
    }

    return ( status );
}
#endif
