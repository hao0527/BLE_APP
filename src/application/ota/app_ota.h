/**
 ****************************************************************************************
 * @file    app_ota.h
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


/** @defgroup OTA App
* @{
*/

#ifndef __APP_OTA_H__
#define __APP_OTA_H__


#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* __cplusplus */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "panip_config.h"     // SW configuration
#include <stdint.h>          // Standard Integer Definition
#include "ke_task.h"         // Kernel Task Definition


struct app_ota_env_tag
{
    /// Connection handle
    uint8_t conidx;
    /// Mouse timeout value
    uint16_t timeout;
    /// Internal state of the module
    uint8_t state;
    /// Timer enabled
    bool timer_enabled;
    /// Number of report that can be sent
    uint8_t nb_report;
};

extern const struct ke_state_handler app_ota_table_handler;
extern void app_ota_init(void);
extern void app_ota_enable_server_prf(uint8_t conidx);
extern void app_ota_add_server(void);
extern void app_ota_send_value(uint8_t att_idx,uint8_t *buf,uint8_t len);
extern void app_ota_check_status ( void );

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */


#endif /* __APP_OTA_H__ */
/** @} */

