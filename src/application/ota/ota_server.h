/**
 ****************************************************************************************
 * @file    ota_server.h
 * @brief   
 
 * @version 0.01
 * @date    2018/11/26
 * @history 
 * @note	   
 * detailed description 
 
 ****************************************************************************************
 * @attention
 *
 * Copyright (C) 2016 Shanghai Panchip Microelectronics Co., Ltd. All rights reserved.
 ****************************************************************************************
 */


/** @defgroup OTA Server
* @{
*/

#ifndef __OTA_SERVER_H__
#define __OTA_SERVER_H__



#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* __cplusplus */


#include "prf_types.h"
#include "prf.h"

#define OTA_SERVER_IDX_MAX     0x01


enum ota_server_state
{
    /// Idle state
    OTA_SVS_IDLE,
    /// busy state
    OTA_SVS_BUSY,
    /// Number of defined states.
    OTA_SVS_STATE_MAX
};

enum
{
    OTA_SVC_UUID        = 0xFD00,
    OTA_CHAR_DATA_UUID  = 0xFD01,
    OTA_CHAR_CTRL_UUID  = 0xFD02,
};

//attibute idex
enum
{
    OTA_IDX_SVC,
    OTA_IDX_DATA_CHAR,
    OTA_IDX_DATA_VAL,
    OTA_IDX_CTRL_CHAR,
    OTA_IDX_CTRL_VAL,
    OTA_IDX_CTRL_CLIENT_CFG,
    OTA_IDX_NB,
};


#define OTA_CTRL_CLIENT_CFG_LEN         sizeof(uint16_t)

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

struct ota_server_env_tag
{
    /// profile environment
    prf_env_t prf_env;
    /// On-going operation
    struct ke_msg* operation;
    /// Services Start Handle
    uint16_t   start_hdl;
    ke_state_t state[OTA_SERVER_IDX_MAX];
    uint16_t   client_cfg_ctrl[BLE_CONNECTION_MAX];
};

extern struct ota_server_env_tag ota_server_env;
extern const struct prf_task_cbs* ota_server_prf_itf_get ( void );

uint16_t ota_server_get_att_handle ( uint8_t svc_idx, uint8_t att_idx );
uint8_t  ota_server_get_att_idx ( uint16_t handle, uint8_t* svc_idx, uint8_t* att_idx );


#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */


#endif /* __OTA_SERVER_H__ */
/** @} */

