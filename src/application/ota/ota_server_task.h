/**
 ****************************************************************************************
 * @file    ota_server_task.h
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


/** @defgroup OTA Server Task
* @{
*/

#ifndef __OTA_SERVER_TASK_H__
#define __OTA_SERVER_TASK_H__

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* __cplusplus */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stdint.h>
#include "ke_task.h"
#include "panip_task.h"

#include "l2cc_pdu.h"

/*
 * INSTANCES
 ****************************************************************************************
 */
#define OTA_SERVER_PACKET_SIZE (L2C_MIN_LE_MTUSIG - 3)

extern	void ota_server_task_init ( struct ke_task_desc* task_desc );
extern void bootloader_start ( void );
/*
 * MESSAGES
 ****************************************************************************************
 */
/// Message API of the OTA_SERVER task
enum
{
    /// Enables the OTA Service Device profile. The profile has to be enabled only
    /// once a connection has been established by the application
    OTA_SERVER_ENABLE_REQ = KE_FIRST_MSG ( TASK_ID_OTA ),

    //Enable condirmation message
    OTA_SERVER_ENABLE_RSP,

    //peer write data to att idx
    OTA_SERVER_PEER_WRITE_DATA_IND,

    //NTF / Indication data to peer
    OTA_SERVER_SEND_TO_PEER,
};

enum ntf_type
{
		NOTIFICATION,
		INDICATION,
};

struct ota_server_db_cfg
{
    uint32_t features;
};

/// @ref OTA_SERVER_ENABLE_REQ parameters structure description.
struct ota_server_enable_req
{
    /// connection index
    uint8_t  conidx;
    /// Notification Configuration
    uint8_t  ntf_cfg;
};

/// Parameters of the @ref OTA_ENABLE_RSP message
struct ota_server_enable_rsp
{
    /// connection index
    uint8_t conidx;
    ///status
    uint8_t status;
};

struct ota_server_peer_write_data_ind
{
    uint8_t 	conidx;
    uint8_t		att_idx;
    uint8_t 	packet[OTA_SERVER_PACKET_SIZE];
    uint8_t 	packet_size;
};

struct ota_server_send_to_peer
{
    uint8_t 	conidx;
    uint8_t		att_idx;
    uint8_t 	write_type;
    uint8_t 	length;
    uint8_t 	value[];
};


#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */


#endif /* __OTA_SERVER_TASK_H__ */
/** @} */

