
#ifndef APP_HID_SELFIE_H_
#define APP_HID_SELFIE_H_

/**
 ****************************************************************************************
 * @addtogroup APP
 * @ingroup RICOW
 *
 * @brief HID Application Module entry point
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "panip_config.h"     // SW configuration

#if (BLE_APP_HID)

#include <stdint.h>          // Standard Integer Definition
#include "ke_task.h"         // Kernel Task Definition

#if (PS2_SUPPORT)
#include "ps2.h"             // PS2 Mouse Driver
#endif //(PS2_SUPPORT)


#define APP_HID_SELFIE_TIMER_CTRL      0


/*
 * STRUCTURES DEFINITION
 ****************************************************************************************
 */

/// HID Application Module Environment Structure
//struct app_hid_env_tag
//{
//    /// Connection handle
//    uint8_t conidx;
//    /// Mouse timeout value
//    uint16_t timeout;
//    /// Internal state of the module
//    uint8_t state;
//    /// Timer enabled
//    bool timer_enabled;
//    /// Number of report that can be sent
//    uint8_t nb_report;
//};

/*
 * GLOBAL VARIABLES DECLARATIONS
 ****************************************************************************************
 */

/// Table of message handlers
extern const struct ke_state_handler app_hid_table_handler;

/*
 * GLOBAL FUNCTIONS DECLARATION
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
 * @brief Initialize HID Application Module
 ****************************************************************************************
 */
void app_hid_init(void);

/**
 ****************************************************************************************
 * @brief Initialize the PS2 mouse driver so that it can be used by the application
 ****************************************************************************************
 */
void app_hid_start_mouse(void);

/**
 ****************************************************************************************
 * @brief Add a HID Service instance in the DB
 ****************************************************************************************
 */
void app_hid_add_hids(void);

/**
 ****************************************************************************************
 * @brief Enable the HID Over GATT Profile device role
 *
 * @param[in]:  conhdl - Connection handle for the connection
 ****************************************************************************************
 */
void app_hid_enable_prf(uint8_t conidx);

/**
 ****************************************************************************************
 * @brief Send a mouse report to the peer device
 *
 * @param[in]:  report - Mouse report sent by the PS2 driver
 ****************************************************************************************
 */
//void app_hid_send_mouse_report(struct ps2_mouse_msg report);

typedef enum
{
    HID_SELFIE_KEY_VOL_UP       = 1,
    HID_SELFIE_KEY_VOL_DOWN     = 2,
    HID_SELFIE_KEY_POWER        = 3,
    HID_SELFIE_KEY_MUTE         = 4,
} key_value_t;

typedef enum
{
    HID_SELFIE_KEY_PRESSED      = 1,
    HID_SELFIE_KEY_RELEASED     = 2,
} key_state_t;


extern void app_hid_selfie_send_key(key_value_t key, key_state_t state);

//void app_hid_send_value_report(uint8_t rpt_idx, uint8_t *buf, uint8_t len);
extern  int app_conn_update_handler(ke_msg_id_t const msgid,
									void *param,
									ke_task_id_t const dest_id,
									ke_task_id_t const src_id);

#endif //(BLE_APP_HID)

/// @} APP

#endif // APP_HID_SELFIE_H_
