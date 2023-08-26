
#ifndef APP_FINDME_H_
#define APP_FINDME_H_

/**
 ****************************************************************************************
 * @addtogroup APP
 * @ingroup RICOW
 *
 * @brief Battery Application Module entry point
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"     // SW configuration

#if (BLE_APP_FINDME)

#include <stdint.h>          // Standard Integer Definition
#include "ke_task.h"         // Kernel Task Definition

/*
 * STRUCTURES DEFINITION
 ****************************************************************************************
 */

/// Battery Application Module Environment Structure
struct app_findme_env_tag
{
    /// Connection handle
    uint8_t conidx;
    /// Current Battery Level
    uint8_t batt_lvl;
};

/*
 * GLOBAL VARIABLES DECLARATIONS
 ****************************************************************************************
 */

/// Battery Application environment
extern struct app_findme_env_tag app_findme_env;

/// Table of message handlers
extern const struct ke_state_handler app_findme_table_handler;

/*
 * FUNCTIONS DECLARATION
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 *
 * Health Thermometer Application Functions
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialize Battery Application Module
 ****************************************************************************************
 */
void app_findme_init(void);

/**
 ****************************************************************************************
 * @brief Add a Battery Service instance in the DB
 ****************************************************************************************
 */
void app_findme_add_bas(void);

/**
 ****************************************************************************************
 * @brief Enable the Battery Service
 ****************************************************************************************
 */
//void app_batt_enable_prf(uint8_t conidx);

/**
 ****************************************************************************************
 * @brief Send a Battery level value
 ****************************************************************************************
 */
//void app_batt_send_lvl(uint8_t batt_lvl);

#endif //(BLE_APP_FINDME)

/// @} APP

#endif // APP_FINDME_H_
