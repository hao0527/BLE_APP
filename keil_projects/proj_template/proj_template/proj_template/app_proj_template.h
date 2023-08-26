
#ifndef APP_PROJ_TEMPLATE_H_
#define APP_PROJ_TEMPLATE_H_


/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "panip_config.h"     // SW configuration
#include <stdint.h>          // Standard Integer Definition
#include "ke_task.h"         // Kernel Task Definition


struct app_proj_template_env_tag
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

extern const struct ke_state_handler app_proj_template_table_handler;
extern void app_proj_template_init(void);
extern void app_proj_template_enable_server_prf(uint8_t conidx);
extern void app_proj_template_add_server(void);
extern void app_proj_template_send_value(uint8_t att_idx,uint8_t *buf,uint8_t len);
extern int app_conn_update_handler(ke_msg_id_t const msgid,
									void *param,
									ke_task_id_t const dest_id,
									ke_task_id_t const src_id);
#endif
