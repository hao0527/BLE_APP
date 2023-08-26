
#include "rwip_config.h"     // SW configuration

#if (BLE_APP_FINDME)

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "app_findme.h"                // Battery Application Module Definitions
#include "app.h"                     // Application Definitions
#include "app_task.h"                // application task definitions
#include "findt_task.h"               // health thermometer functions
#include "co_bt.h"
#include "prf_types.h"               // Profile common types definition
#include "arch.h"                    // Platform Definitions
#include "prf.h"
#include <string.h>

struct app_findme_env_tag app_findme_env;

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void app_findme_init(void)
{
    // Reset the environment
//    memset(&app_batt_env, 0, sizeof(struct app_batt_env_tag));

    // Initial battery level: 100
 //   app_batt_env.batt_lvl = 100;
}

void app_findme_add_bas(void)
{

    struct findt_db_cfg* db_cfg;
    // Allocate the BASS_CREATE_DB_REQ
    struct gapm_profile_task_add_cmd *req = KE_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD,
                                                  TASK_GAPM, TASK_APP,
                                                  gapm_profile_task_add_cmd, sizeof(struct findt_db_cfg));
    // Fill message
    req->operation   = GAPM_PROFILE_TASK_ADD;
    req->sec_lvl     = PERM(SVC_AUTH, ENABLE);
    req->prf_task_id = TASK_ID_FINDT;
    req->app_task    = TASK_APP;	//TASK_APP;
    req->start_hdl   = 0;

    // Set parameters
    db_cfg = (struct findt_db_cfg* ) req->param;
 
    ke_msg_send(req);

}

void app_findme_enable_prf(uint8_t conidx)
{
#if 0
    app_batt_env.conidx = conidx;

    // Allocate the message
    struct bass_enable_req * req = KE_MSG_ALLOC(BASS_ENABLE_REQ,
                                                prf_get_task_from_id(TASK_ID_BASS),
                                                TASK_APP,
                                                bass_enable_req);

    // Fill in the parameter structure
    req->conidx             = conidx;

    // NTF initial status - Disabled
    req->ntf_cfg           = PRF_CLI_STOP_NTFIND;
    req->old_batt_lvl[0]   = 50;

    // Send the message

    ke_msg_send(req);
#endif
}


static int app_findme_msg_dflt_handler(ke_msg_id_t const msgid,
                                               void const *param,
                                               ke_task_id_t const dest_id,
                                               ke_task_id_t const src_id)
{
    return (KE_MSG_CONSUMED);
}

static int find_me_alert_ind_handler(ke_msg_id_t const msgid,
                                      struct findt_alert_ind const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
printf("alert_lvl:%d\n",param->alert_lvl);
    return (KE_MSG_CONSUMED);
}


/// Default State handlers definition
const struct ke_msg_handler app_findme_msg_handler_list[] =
{
    // Note: first message is latest message checked by kernel so default is put on top.
    {KE_MSG_DEFAULT_HANDLER,        (ke_msg_func_t)app_findme_msg_dflt_handler},
    {FINDT_ALERT_IND,   (ke_msg_func_t)find_me_alert_ind_handler},
   // {BASS_BATT_LEVEL_UPD_RSP,       (ke_msg_func_t)batt_level_upd_handler},
};

const struct ke_state_handler app_findme_table_handler =
    {&app_findme_msg_handler_list[0], (sizeof(app_findme_msg_handler_list)/sizeof(struct ke_msg_handler))};




#endif //BLE_APP_FINDME

/// @} APP
