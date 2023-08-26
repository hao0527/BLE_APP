
#ifndef APP_TEMPLATE_PROJ_H_
#define APP_TEMPLATE_PROJ_H_

/**
 ****************************************************************************************
 * @addtogroup APP
 * @ingroup RICOW
 *
 * @brief 
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "panble_config.h"
#include "app_task.h"                  // application task
#include "gapc_task.h"                 // gap functions and messages
#include "gapm_task.h"                 // gap functions and messages
#include "app.h"                       // application definitions
#include "co_error.h"                  // error code definitions
//#include "smpc_task.h"                 // error code definitions
 

/****************************************************************************
Add here supported profiles' application header files.
i.e.
#if (BLE_DIS_SERVER)
#include "app_dis.h"
#include "app_dis_task.h"
#endif
*****************************************************************************/

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/*
 * DEFINES
 ****************************************************************************************
 */

/****************************************************************************
Define device name. Used in Advertise string
*****************************************************************************/

extern struct gap_bdaddr connect_bdaddr;


/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */
extern void app_init_ind_func(void);
extern void app_reset_complete_ind_func(void);
extern void app_connection_req_ind_func ( uint8_t conidx, struct gapc_connection_req_ind const* param );
extern void app_db_init_complete_ind_func(void);
extern void app_connection_encrypted_ind_func(uint8_t conidx);
extern void app_disconnect_ind_func(ke_task_id_t task_id, struct gapc_disconnect_ind const *param);
extern void app_update_params_rejected_ind_func(uint8_t status);
extern void app_update_params_complete_ind_func(void);
extern void app_adv_undirect_complete_ind_func(uint8_t status);
extern void app_adv_direct_complete_ind_func(uint8_t status);
extern void app_scanning_completed_ind_func(uint8_t status);
extern void app_adv_report_ind_func(struct gapm_adv_report_ind *param);
extern void app_get_info_ind_func(void *param,uint8_t operation);
extern void app_connection_success_ind_func(uint8_t conidx);
extern void app_connect_failed_ind_func(void);
extern void app_connection_encrypt_completed_ind_func(uint8_t conidx);
extern void app_set_dev_config_complete_ind_func(void);

/// @} APP

#endif //APP_TEMPLATE_PROJ_H_
