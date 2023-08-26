
/**
 ****************************************************************************************
 * @addtogroup APP
 * @{
 ****************************************************************************************
 */

#include "panip_config.h"            // SW configuration

#include <stdio.h>
#include <string.h>

#if (BLE_APP_HID)

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "app.h"                    // Application Definitions
#include "app_sec.h"                // Application Security Module API
#include "app_task.h"               // Application task definitions
#include "hogpd_task.h"             // HID Over GATT Profile Device Role Functions
#include "prf_types.h"              // Profile common types Definition
#include "arch.h"                    // Platform Definitions
#include "prf.h"
#include "ke_timer.h"

#include "hogpd.h"
#include "app_hid_keyboard.h"
#include "proj_hid_keyboard.h"
#include "stack_svc_api.h"


#if (DISPLAY_SUPPORT)
#include "app_display.h"            // Application Display Module
#endif //(DISPLAY_SUPPORT)

#include "co_utils.h"               // Common functions

#if (KE_PROFILING)
#include "ke_mem.h"
#endif //(KE_PROFILING)

#define TEST_TIMER_DURATION 50

#define DBG_APP_HID_KBD(x)           //printf x
#define DBG_APP_HID_KBD_FUNC()       //printf("%s\n", __func__);
/*
 * DEFINES
 ****************************************************************************************
 */

/// Length of the HID Mouse Report
#define APP_HID_KEYBOARD_REPORT_LEN       (6)
/// Length of the Report Descriptor for an HID Mouse
#define APP_HID_KEYBOARD_REPORT_MAP_LEN   (sizeof(app_hid_keyboard_report_map))

/// Duration before connection update procedure if no report received (mouse is silent) - 20s
#define APP_HID_SILENCE_DURATION_1     (2000)
/// Duration before disconnection if no report is received after connection update - 60s
#define APP_HID_SILENCE_DURATION_2     (6000)

/// Number of reports that can be sent
#define APP_HID_NB_SEND_REPORT         (10)

/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/// States of the Application HID Module
enum app_hid_states
{
    /// Module is disabled (Service not added in DB)
    APP_HID_DISABLED,
    /// Module is idle (Service added but profile not enabled)
    APP_HID_IDLE,
    /// Module is enabled (Device is connected and the profile is enabled)
    APP_HID_ENABLED,
    /// The application can send reports
    APP_HID_READY,
    /// Waiting for a report
    APP_HID_WAIT_REP,

    APP_HID_STATE_MAX,
};

/*
 * LOCAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// HID Application Module Environment Structure
static struct app_hid_env_tag app_hid_env;

/// HID Keyboard Report Descriptor
static const uint8_t app_hid_keyboard_report_map[] =
{
    0x05, 0x01,       /// Usage Page(Generic Desktop)
	0x09, 0x06,       /// Usage(Keyboard)
	0xA1, 0x01,       /// Collection(Application)
	0x85, 0x01,       /// Report Id(1)
    /// Byte 0
	0x05, 0x07,       /// Usage Page(Key Codes)
	0x19, 0xE0,       /// Usage Minimum(224)
	0x29, 0xE7,       /// Usage Maximum(231)
	0x15, 0x00,       /// Logical Minimum (0)
	0x25, 0x01,       /// Logical Maximum (1)
	0x75, 0x01,       /// Report Size (1)
	0x95, 0x08,       /// Report Count (8)
	0x81, 0x02,       /// Input(Data,Variable,Absolute)
    /// Byte 1, Reserved
    0x75, 0x08,       /// Report Size (8)
	0x95, 0x01,       /// Report Count (1)
	0x81, 0x01,       /// Input(Constant)
    /// Byte 2~7
    0x05, 0x07,       /// Usage Page (Key Codes)
    0x19, 0x00,       /// Usage Minimum (0)
    0x29, 0x65,       /// Usage Maximum (101)
    0x15, 0x00,       /// Logical Minimum (0)
    0x25, 0x65,       /// Logical Maximum (101)
    0x75, 0x08,       /// Report Size (8)
    0x95, 0x06,       /// Report Count (6)
    0x81, 0x00,       /// Input(Data, Array)
    
    // Report LEDs
    // bit 0~4
    0x05, 0x08,       /// Usage Page (LEDs)
    0x19, 0x01,       /// Usage Minimum (1)
    0x29, 0x05,       /// Usage Maximum (5)
    0x75, 0x01,       /// Report Size (1)
    0x95, 0x05,       /// Report Count (5)
    0x91, 0x02,       /// Output: (Data, Variable, Absolute)
    // bit 5~7, Reserved
    0x75, 0x03,       /// Report Size (3)
    0x95, 0x01,       /// Report Count (1)
    0x91, 0x01,       /// Output: (Constant)
    0xC0,             /// End Collection
};


/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void app_hid_init(void)
{
    DBG_APP_HID_KBD_FUNC();

    // Reset the environment
    memset(&app_hid_env, 0, sizeof(app_hid_env));
    app_hid_env.nb_report = APP_HID_NB_SEND_REPORT;
}

void app_hid_add_hids(void)
{
    DBG_APP_HID_KBD_FUNC();
    
    struct hogpd_db_cfg *db_cfg;
    // Prepare the HOGPD_CREATE_DB_REQ message
    struct gapm_profile_task_add_cmd *req = KE_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD,
                                                   TASK_GAPM, TASK_APP,
                                                   gapm_profile_task_add_cmd, sizeof(struct hogpd_db_cfg));

    // Fill message
    req->operation   = GAPM_PROFILE_TASK_ADD;
    req->sec_lvl     = PERM(SVC_AUTH, ENABLE);
    req->prf_task_id = TASK_ID_HOGPD;
    req->app_task    = TASK_APP;
    req->start_hdl   = 0;

    // Set parameters
    db_cfg = (struct hogpd_db_cfg* ) req->param;

    // Only one HIDS instance is useful
    db_cfg->hids_nb = 1;
    // The device is a mouse
    db_cfg->cfg[0].svc_features = HOGPD_CFG_KEYBOARD;

    // Report Characteristic
    db_cfg->cfg[0].report_nb    = 3;
    
    // HID Report
    db_cfg->cfg[0].report_id[0] = 1;
    // The report is an input report
    db_cfg->cfg[0].report_char_cfg[0] = HOGPD_CFG_REPORT_IN;
    
    db_cfg->cfg[0].report_id[1] = 1;
    // The report is an input report
    db_cfg->cfg[0].report_char_cfg[1] = HOGPD_CFG_REPORT_OUT;
	
    db_cfg->cfg[0].report_id[2] = 2;
	db_cfg->cfg[0].report_char_cfg[2] = HOGPD_CFG_REPORT_IN;

    // HID Information
    db_cfg->cfg[0].hid_info.bcdHID       = 0x0111;         // USB HID Version 1.11
    db_cfg->cfg[0].hid_info.bCountryCode = 0x00;
    db_cfg->cfg[0].hid_info.flags        = HIDS_REMOTE_WAKE_CAPABLE | HIDS_NORM_CONNECTABLE;
    
    // Send the message
    ((ke_msg_send_handler)SVC_ke_msg_send)(req);
}


/*
 ****************************************************************************************
 * @brief Function called when get connection complete event from the GAP
 *
 ****************************************************************************************
 */
void app_hid_enable_prf(uint8_t conidx)
{
    DBG_APP_HID_KBD_FUNC();
    
    // Requested connection parameters
//    struct gapc_conn_param conn_param;

    uint16_t ntf_cfg;

    // Store the connection handle
    app_hid_env.conidx = conidx;

    // Allocate the message
    struct hogpd_enable_req * req = KE_MSG_ALLOC(HOGPD_ENABLE_REQ,
                                                 prf_get_task_from_id(TASK_ID_HOGPD),
                                                 TASK_APP,
                                                 hogpd_enable_req);
    // Fill in the parameter structure
    req->conidx     = conidx;
    // Notifications are disabled
    ntf_cfg         = 0;

    // Go to Enabled state
    app_hid_env.state = APP_HID_ENABLED;
	((ke_state_set_handler)SVC_ke_state_set)(prf_get_task_from_id(TASK_ID_HOGPD), HOGPD_IDLE);
	
	if (app_sec_get_bond_status())
	{
        //Restore the cfg and nb_report.
        ntf_cfg = *(uint16_t *)hid_kbd_get_store_info(HID_KBD_STORE_TYPE_NTF);
        
		// The device is ready to send reports to the peer device
		app_hid_env.state       = APP_HID_READY;
		app_hid_env.nb_report   = APP_HID_NB_SEND_REPORT;
	}
    
	req->ntf_cfg[conidx] = ntf_cfg;
    
    // Send the message
    ((ke_msg_send_handler)SVC_ke_msg_send)(req);
}

void app_hid_send_value_report(uint8_t rpt_idx, uint8_t *buf, uint8_t len)
{
    DBG_APP_HID_KBD_FUNC();

    switch (app_hid_env.state)
    {
        case (APP_HID_READY):
        {
            DBG_APP_HID_KBD(("APP_HID_READY, nb_report %d\n", app_hid_env.nb_report));
            
            // Check if the report can be sent
            if (app_hid_env.nb_report)
            {
                // Allocate the HOGPD_REPORT_UPD_REQ message
                struct hogpd_report_upd_req * req = KE_MSG_ALLOC_DYN(HOGPD_REPORT_UPD_REQ,
                                                                  prf_get_task_from_id(TASK_ID_HOGPD),
                                                                  TASK_APP,
                                                                  hogpd_report_upd_req,
                                                                  len);

                req->conidx  = app_hid_env.conidx;
                //now fill report
                req->report.hid_idx  = app_hid_env.conidx;
                req->report.type     = HOGPD_REPORT; //HOGPD_BOOT_MOUSE_INPUT_REPORT;//
                req->report.idx      = rpt_idx; //0 for boot reports and report map
                req->report.length   = len;
                memcpy(&req->report.value[0], buf, len);

                ((ke_msg_send_handler)SVC_ke_msg_send)(req);

                app_hid_env.nb_report--;

                // Restart the mouse timeout timer if needed
            }
        } break;

        case (APP_HID_WAIT_REP):
        {
            DBG_APP_HID_KBD(("APP_HID_WAIT_REP\n"));

            // Go back to the ready state
            app_hid_env.state = APP_HID_READY;
        } break;

        case (APP_HID_IDLE):
        {
            DBG_APP_HID_KBD(("APP_HID_IDLE\n"));
            
            // Try to restart advertising if needed
            appm_start_advertising(NULL);
        } break;

        // DISABLE and ENABLED states
        default:
        {
            DBG_APP_HID_KBD(("state %d (unknown)\n", app_hid_env.state));
            
            // Drop the message
        } break;
    }
}


#endif

/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */


static int hogpd_ctnl_pt_ind_handler( ke_msg_id_t const               msgid,
                                      struct hogpd_ctnl_pt_ind const *param,
                                      ke_task_id_t const              dest_id,
                                      ke_task_id_t const              src_id )
{
    DBG_APP_HID_KBD_FUNC();

    if (param->conidx == app_hid_env.conidx)
    {
        //make use of param->hid_ctnl_pt
        struct hogpd_report_cfm *req = KE_MSG_ALLOC_DYN( HOGPD_REPORT_CFM,
                                                         prf_get_task_from_id(TASK_ID_HOGPD),/* src_id */
                                                         TASK_APP,
                                                         hogpd_report_cfm,
                                                         0 );

        req->conidx = param->conidx; ///app_hid_env.conidx; ///???
        /// Operation requested (read/write @see enum hogpd_op)
        req->operation = HOGPD_OP_REPORT_WRITE;
        /// Status of the request
        req->status = GAP_ERR_NO_ERROR;  ///???
        /// Report Info
        //req->report;
        /// HIDS Instance
        req->report.hid_idx = app_hid_env.conidx; ///???
        /// type of report (@see enum hogpd_report_type)
        req->report.type = HOGPD_REPORT;//-1;//outside 
        /// Report Length (uint8_t)
        req->report.length = 0;
        /// Report Instance - 0 for boot reports and report map
        req->report.idx = 0;
        /// Report data
        

        // Send the message
        ((ke_msg_send_handler)SVC_ke_msg_send)(req);
    }
    return (KE_MSG_CONSUMED);
}




static int hogpd_ntf_cfg_ind_handler( ke_msg_id_t const               msgid,
                                      struct hogpd_ntf_cfg_ind const *param,
                                      ke_task_id_t const              dest_id,
                                      ke_task_id_t const              src_id )
{
    DBG_APP_HID_KBD_FUNC();
    
    if (app_hid_env.conidx == param->conidx)
    {
        if ((param->ntf_cfg[param->conidx] & HOGPD_CFG_REPORT_NTF_EN ) != 0)
        {
            // The device is ready to send reports to the peer device
            app_hid_env.state = APP_HID_READY;
        }
        else
        {
            // Come back to the Enabled state
            if (app_hid_env.state == APP_HID_READY)
            {
                app_hid_env.state = APP_HID_ENABLED;
            }
        }

        hid_kbd_store_info((void *)&param->ntf_cfg[param->conidx], HID_KBD_STORE_TYPE_NTF);
    }

    return (KE_MSG_CONSUMED);
}

static int hogpd_report_req_ind_handler(ke_msg_id_t const msgid,
                                    struct hogpd_report_req_ind const *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{

    if ((param->operation == HOGPD_OP_REPORT_READ) && (param->report.type == HOGPD_REPORT_MAP))
    {
        struct hogpd_report_cfm *req = KE_MSG_ALLOC_DYN(HOGPD_REPORT_CFM,
                                                        src_id, ///prf_get_task_from_id(TASK_ID_HOGPD),/* src_id */
                                                        dest_id, ///TASK_APP,
                                                        hogpd_report_cfm,
                                                        APP_HID_KEYBOARD_REPORT_MAP_LEN);

        req->conidx = app_hid_env.conidx; ///???
        /// Operation requested (read/write @see enum hogpd_op)
        req->operation = HOGPD_OP_REPORT_READ;
        /// Status of the request
        req->status = GAP_ERR_NO_ERROR;  ///???
        /// Report Info
        //req->report;
        /// HIDS Instance
        req->report.hid_idx = param->report.hid_idx;///   ???///app_hid_env.conidx; ///???
        /// type of report (@see enum hogpd_report_type)
        req->report.type = HOGPD_REPORT_MAP;
        /// Report Length (uint8_t)
       req->report.length = APP_HID_KEYBOARD_REPORT_MAP_LEN;
        /// Report Instance - 0 for boot reports and report map
        req->report.idx = 0;
        /// Report data
         memcpy(&req->report.value[0], &app_hid_keyboard_report_map[0], APP_HID_KEYBOARD_REPORT_MAP_LEN);

        // Send the message
        ((ke_msg_send_handler)SVC_ke_msg_send)(req);
    }
    else
    {
        if (param->report.type == HOGPD_BOOT_MOUSE_INPUT_REPORT)
        { //request of boot mouse report
            struct hogpd_report_cfm *req = KE_MSG_ALLOC_DYN(HOGPD_REPORT_CFM,
                                                            prf_get_task_from_id(TASK_ID_HOGPD),/* src_id */
                                                            TASK_APP,
                                                            hogpd_report_cfm,
                                                            0/*param->report.length*/);

            req->conidx = param->conidx; ///app_hid_env.conidx; ///???
            /// Operation requested (read/write @see enum hogpd_op)
            req->operation = HOGPD_OP_REPORT_READ;
            /// Status of the request
            req->status = GAP_ERR_NO_ERROR;  ///???
            /// HIDS Instance
            req->report.hid_idx = app_hid_env.conidx; ///???
            /// type of report (@see enum hogpd_report_type)
            req->report.type = param->report.type;//-1;//outside 
            /// Report Length (uint8_t)
            req->report.length = 0; //param->report.length;
            /// Report Instance - 0 for boot reports and report map
            req->report.idx = param->report.idx; //0;
            /// Report data

            // Send the message
            ((ke_msg_send_handler)SVC_ke_msg_send)(req);
        }
        else if (param->report.type == HOGPD_REPORT)
        {
            //request of mouse report
            struct hogpd_report_cfm *req = KE_MSG_ALLOC_DYN(HOGPD_REPORT_CFM,
                                                            prf_get_task_from_id(TASK_ID_HOGPD),/* src_id */
                                                            TASK_APP,
                                                            hogpd_report_cfm,
                                                            8/*param->report.length*/);

            req->conidx = param->conidx; ///app_hid_env.conidx; ///???
            /// Operation requested (read/write @see enum hogpd_op)
            req->operation = HOGPD_OP_REPORT_READ;
            /// Status of the request
            req->status = GAP_ERR_NO_ERROR;  ///???
            /// Report Info
            //req->report;
            /// HIDS Instance
            req->report.hid_idx = app_hid_env.conidx; ///???
            /// type of report (@see enum hogpd_report_type)
            req->report.type = param->report.type;//-1;//outside 
            /// Report Length (uint8_t)
            req->report.length = 8; //param->report.length;
            /// Report Instance - 0 for boot reports and report map
            req->report.idx = param->report.idx; //0;
            /// Report data
            memset(&req->report.value[0], 0, 8); //???
            req->report.value[0] = param->report.hid_idx;    /// HIDS Instance
            req->report.value[1] = param->report.type;    /// type of report (@see enum hogpd_report_type)
            req->report.value[2] = param->report.length;    /// Report Length (uint8_t)
            req->report.value[3] = param->report.idx;    /// Report Instance - 0 for boot reports and report map

            // Send the message
            ((ke_msg_send_handler)SVC_ke_msg_send)(req);
        }
        else
        {
            struct hogpd_report_cfm *req = KE_MSG_ALLOC_DYN( HOGPD_REPORT_CFM,
                                                             prf_get_task_from_id(TASK_ID_HOGPD),/* src_id */
                                                             TASK_APP,
                                                             hogpd_report_cfm,
                                                             8/*param->report.length*/);

            req->conidx = param->conidx; ///app_hid_env.conidx; ///???
            /// Operation requested (read/write @see enum hogpd_op)
            req->operation = HOGPD_OP_REPORT_READ;
            /// Status of the request
            req->status = GAP_ERR_NO_ERROR;  ///???
            /// Report Info
            //req->report;
            /// HIDS Instance
            req->report.hid_idx = app_hid_env.conidx; ///???
            /// type of report (@see enum hogpd_report_type)
            req->report.type = param->report.type;//-1;//outside 
            /// Report Length (uint8_t)
            req->report.length = 8; //param->report.length;
            /// Report Instance - 0 for boot reports and report map
            req->report.idx = param->report.idx; //0;
            /// Report data
            memset(&req->report.value[0], 0, 8); //???
            req->report.value[0] = param->report.hid_idx;    /// HIDS Instance
            req->report.value[1] = param->report.type;    /// type of report (@see enum hogpd_report_type)
            req->report.value[2] = param->report.length;    /// Report Length (uint8_t)
            req->report.value[3] = param->report.idx;    /// Report Instance - 0 for boot reports and report map

            // Send the message
            ((ke_msg_send_handler)SVC_ke_msg_send)(req);
        }
    }

    return (KE_MSG_CONSUMED);
}

static int hogpd_proto_mode_req_ind_handler( ke_msg_id_t const                      msgid,
                                             struct hogpd_proto_mode_req_ind const *param,
                                             ke_task_id_t const                     dest_id,
                                             ke_task_id_t const                     src_id )
{

    if ((param->conidx == app_hid_env.conidx) && (param->operation == HOGPD_OP_PROT_UPDATE))
    {

        //make use of param->proto_mode
        struct hogpd_proto_mode_cfm *req = KE_MSG_ALLOC_DYN(HOGPD_PROTO_MODE_CFM,
                                                        prf_get_task_from_id(TASK_ID_HOGPD),/* src_id */
                                                        TASK_APP,
                                                        hogpd_proto_mode_cfm,
                                                        0);
        /// Connection Index
        req->conidx = app_hid_env.conidx; 
        /// Status of the request
        req->status = GAP_ERR_NO_ERROR;
        /// HIDS Instance
        req->hid_idx = app_hid_env.conidx;
        /// New Protocol Mode Characteristic Value
        req->proto_mode = param->proto_mode;
        
        // Send the message
        ((ke_msg_send_handler)SVC_ke_msg_send)(req);
    }
    else
    {
        struct hogpd_proto_mode_cfm *req = KE_MSG_ALLOC_DYN( HOGPD_PROTO_MODE_CFM,
                                                             prf_get_task_from_id(TASK_ID_HOGPD),/* src_id */
                                                             TASK_APP,
                                                             hogpd_proto_mode_cfm,
                                                             0 );
        /// Status of the request
        req->status = ATT_ERR_APP_ERROR;

        /// Connection Index
        req->conidx = app_hid_env.conidx;
        /// HIDS Instance
        req->hid_idx = app_hid_env.conidx;
        /// New Protocol Mode Characteristic Value
        req->proto_mode = param->proto_mode;
        
        // Send the message
        ((ke_msg_send_handler)SVC_ke_msg_send)(req);
    }
    
    return (KE_MSG_CONSUMED);
}


static int hogpd_report_upd_handler( ke_msg_id_t const msgid,
                                     struct hogpd_report_upd_rsp const *param,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id )
{
    if (app_hid_env.conidx == param->conidx)
    {
        if (GAP_ERR_NO_ERROR == param->status)
        {
            if (app_hid_env.nb_report < APP_HID_NB_SEND_REPORT)
            {
                app_hid_env.nb_report++;
            }
        }
        else
        {
            DBG_APP_HID_KBD(("error status, 0x%X\n", param->status));
            // we get this message if error occur while sending report
            // most likely - disconnect
            // Go back to the ready state
            app_hid_env.state = APP_HID_IDLE;
            // change mode
            // restart adv
            // Try to restart advertising if needed
            appm_start_advertising(NULL);
            //report was not success - need to restart???
        }
    }
    return (KE_MSG_CONSUMED);
}

static int hogpd_enable_rsp_handler( ke_msg_id_t const msgid,
                                     struct hogpd_enable_rsp const *param,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id )
{

    return (KE_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int app_hid_msg_dflt_handler( ke_msg_id_t const  msgid,
                                     void const *       param,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id )
{
    /// Drop the message

    return (KE_MSG_CONSUMED);
}



/**
 ****************************************************************************************
 * @brief Set the value of the Report Map Characteristic in the database
 ****************************************************************************************
 */
void app_hid_set_report_map(void);

/*
 * LOCAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// Default State handlers definition
const struct ke_msg_handler app_hid_msg_handler_list[] =
{
    /// Note: first message is latest message checked by kernel so default is put on top.
    {KE_MSG_DEFAULT_HANDLER,        (ke_msg_func_t)app_hid_msg_dflt_handler},

    {HOGPD_ENABLE_RSP,              (ke_msg_func_t)hogpd_enable_rsp_handler},
    
    /// notification configuration changed
    {HOGPD_NTF_CFG_IND,             (ke_msg_func_t)hogpd_ntf_cfg_ind_handler},
    {HOGPD_REPORT_REQ_IND,          (ke_msg_func_t)hogpd_report_req_ind_handler},
    {HOGPD_PROTO_MODE_REQ_IND,      (ke_msg_func_t)hogpd_proto_mode_req_ind_handler},

    {HOGPD_CTNL_PT_IND,             (ke_msg_func_t)hogpd_ctnl_pt_ind_handler},

    {HOGPD_REPORT_UPD_RSP,          (ke_msg_func_t)hogpd_report_upd_handler},
    
};

const struct ke_state_handler app_hid_table_handler =
{
    &app_hid_msg_handler_list[0], (sizeof(app_hid_msg_handler_list)/sizeof(struct ke_msg_handler))
};


/// @} APP
