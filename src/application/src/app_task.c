
/**
 ****************************************************************************************
 * @addtogroup APPTASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "panip_config.h"          // SW configuration

#if (BLE_APP_PRESENT)

#include "app_task.h"             // Application Manager Task API
#include "app.h"                  // Application Manager Definition
#include "gapc_task.h"            // GAP Controller Task API
#include "gap.h"
#include "gapm_task.h"            // GAP Manager Task API
#include "arch.h"                 // Platform Definitions
#include <string.h>
#include "ke_timer.h"             // Kernel timer
#include "co_utils.h"
#include "app_sec.h"
#include "app_proj_event.h"
#include "app_hid.h"
#include "app_batt.h"
#include "app_dis.h"

#if (USER_PROJ_TEMPLATE || PROJ_THROUGHPUT || USER_PROJ_WECHAT || PROJ_MULTIROLE)
#include "app_proj_template.h"
#include "proj_template_server.h"
#endif

#if (PROJ_HID_SELFIE)
#include "app_hid_selfie.h"
#endif
#if (PROJ_HID_KEYBOARD)
#include "app_hid_keyboard.h"
#endif
#if (USER_RGB_LIGHT)
#include "app_rgb_light1.h"
#include "app_rgb_light2.h"
#endif
#if (USER_PROJ_CENTRAL)
#include "tmp_client_task.h"   
#include "app_tmp.h"
#endif 
#if (PROJ_OTA)
#include "app_ota.h"
#endif

#define DBG_APP_TASK(x)         //printf x
#define DBG_APP_TASK_FUNC()     //printf("%s\n", __func__)
#include "stack_svc_api.h"

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

static uint8_t appm_get_handler(const struct ke_state_handler *handler_list,
                                ke_msg_id_t msgid,
                                void *param,
                                ke_task_id_t src_id)
{
    // Counter
    uint8_t counter;

    // Get the message handler function by parsing the message table
    for (counter = handler_list->msg_cnt; 0 < counter; counter--)
    {
    
        struct ke_msg_handler *handler = (struct ke_msg_handler *)(handler_list->msg_table + counter - 1);	
        if ((handler->id == msgid) ||
            (handler->id == KE_MSG_DEFAULT_HANDLER))
        {
            // If handler is NULL, message should not have been received in this state
            ASSERT_ERR(handler->func);
            return (uint8_t)(handler->func(msgid, param, TASK_APP, src_id));
        }		
    }

    // If we are here no handler has been found, drop the message
    return (KE_MSG_CONSUMED);
}

/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */

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
static int app_adv_timeout_handler(ke_msg_id_t const msgid,
                                   void const *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{

    // Stop advertising
    appm_stop_advertising();

    return (KE_MSG_CONSUMED);
}


int app_conn_timerout_handler(ke_msg_id_t const msgid,
									void *param,
									ke_task_id_t const dest_id,
									ke_task_id_t const src_id)
{
	appm_stop_connecting();
	
    return (KE_MSG_CONSUMED);
}

int app_conn_update_handler(ke_msg_id_t const msgid,
									void *param,
									ke_task_id_t const dest_id,
									ke_task_id_t const src_id)
{
	/*
         * Interval Max * (Slave Latency + 1) = 2 seconds;
         * Interval Min = 20 ms;
         * Interval Min + 20 ms = Interval Max;
         * Slave Latency = 4;connSupervisionTimeout = 6 seconds;
         * Interval Max * (Slave Latency + 1) * 3 < connSupervisionTimeout
         */
		#if(PROJ_THROUGHPUT)
		struct gapc_conn_param conn_param;
		conn_param.intv_min = 10;		//throughput test
		conn_param.intv_max = 10;
		conn_param.latency	= 0;
		conn_param.time_out = 100;
		appm_update_param(&conn_param);
		#elif(USER_PROJ_TEMPLATE | PROJ_HID_KEYBOARD | PROJ_HID_SELFIE)
		struct gapc_conn_param conn_param;
		conn_param.intv_min = 80;
		conn_param.intv_max = 80;
		conn_param.latency	= 0;
		conn_param.time_out = 600;
        #if(PROJ_TEMPER)
        conn_param.intv_min = 96;
        conn_param.intv_max = 96;
        conn_param.latency	= 2;
        conn_param.time_out = 600;
        #endif  // PROJ_TEMPER
		appm_update_param(&conn_param);
		#elif(PROJ_MULTIROLE)
		struct gapc_conn_param conn_param;
		conn_param.intv_min = 80;
		conn_param.intv_max = 80;
		conn_param.latency	= 0;
		conn_param.time_out = 600;
		appm_update_param(&conn_param);
		#endif
	
	return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles ready indication from the GAP. - Reset the stack
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapm_device_ready_ind_handler(ke_msg_id_t const msgid,
                                         void const *param,
                                         ke_task_id_t const dest_id,
                                         ke_task_id_t const src_id)
{
    // Application has not been initialized
    ASSERT_ERR(((ke_state_get_handler)SVC_ke_state_get)(dest_id) == APPM_INIT);

    // Reset the stack
    struct gapm_reset_cmd* cmd = KE_MSG_ALLOC(GAPM_RESET_CMD,
                                              TASK_GAPM, TASK_APP,
                                              gapm_reset_cmd);

    cmd->operation = GAPM_RESET;

    ((ke_msg_send_handler)SVC_ke_msg_send)(cmd);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles GAP manager command complete events.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapm_cmp_evt_handler(ke_msg_id_t const msgid,
                                struct gapm_cmp_evt const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    switch(param->operation)
    {
        // Reset completed
        case (GAPM_RESET):
        {
            if(param->status == GAP_ERR_NO_ERROR)
            {
            
				app_reset_complete_ind_func(); 	
            }
            else
            {
                ASSERT_ERR(0);
            }
        }
        break;

		// Device Configuration updated
		case (GAPM_SET_DEV_CONFIG):
		{
			ASSERT_INFO(param->status == GAP_ERR_NO_ERROR, param->operation, param->status);
#if 0
			// Go to the create db state
			((ke_state_set_handler)SVC_ke_state_set)(TASK_APP, APPM_CREATE_DB);

			// Add the first required service in the database
			// and wait for the PROFILE_ADDED_IND
			appm_add_svc();
#endif
			app_set_dev_config_complete_ind_func(); 		
		}
		break;

        case (GAPM_PROFILE_TASK_ADD):
        {
           // ASSERT_INFO(param->status == GAP_ERR_NO_ERROR, param->operation, param->status);
            // Add the next requested service
            if (!appm_add_svc())		//no more svc need add. then start adv
            {
                // Go to the ready state
                ((ke_state_set_handler)SVC_ke_state_set)(TASK_APP, APPM_READY);

                // No more service to add, start advertising
                //appm_start_advertising(NULL);
				app_db_init_complete_ind_func();
            }
        }
        break;

        case (GAPM_ADV_NON_CONN):
	//		break;
        case (GAPM_ADV_UNDIRECT):
		{
			app_adv_undirect_complete_ind_func(param->status);
        }
			break;
//        #if !(BLE_APP_HID)
//        case (GAPM_ADV_DIRECT):
//		{
//            app_adv_direct_complete_ind_func(param->status);  
//        }
//        break;
//        #endif// !(BLE_APP_HID)

        case GAPM_SCAN_PASSIVE:
		case GAPM_SCAN_ACTIVE:
        {
			app_scanning_completed_ind_func(param->status);
			#if (USER_PROJ_CENTRAL)
			if (param->status == GAP_ERR_CANCELED)
			{
				app_start_connecting(NULL);
			}						
			else 
			{
				 app_start_scan();
			}
			#endif
        }				
        break;

        case GAPM_CANCEL:
        {
//            if(param->status != GAP_ERR_NO_ERROR)
//            {
//                ASSERT_ERR(0); // unexpected error
//            }
        }
		case GAPM_CONNECTION_DIRECT:
		if (param->status == GAP_ERR_CANCELED)
		{
					app_connect_failed_ind_func();
		}
		break;

		
        case (GAPM_ADV_DIRECT_LDC):
        case (GAPM_UPDATE_ADVERTISE_DATA):
        {
            ASSERT_INFO(param->status == GAP_ERR_NO_ERROR, param->operation, param->status);
        } 
		break;

        #if (BLE_APP_HID)
        case (GAPM_ADV_DIRECT):			//Direct adv is over, start undirect adv
        {
            if (param->status == GAP_ERR_TIMEOUT)
            {
                ((ke_state_set_handler)SVC_ke_state_set)(TASK_APP, APPM_READY);
				app_adv_direct_complete_ind_func(param->status);
            }
        } 
		break;
        #endif //(BLE_APP_HID)

        default:
        {
            
        }
        break;
    }

    return (KE_MSG_CONSUMED);
}

static int gapc_get_dev_info_req_ind_handler(ke_msg_id_t const msgid,
        struct gapc_get_dev_info_req_ind const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    switch(param->req)
    {
        case GAPC_DEV_NAME:
        {
            struct gapc_get_dev_info_cfm * cfm = KE_MSG_ALLOC_DYN(GAPC_GET_DEV_INFO_CFM,
                                                    src_id, dest_id,
                                                    gapc_get_dev_info_cfm, APP_DEVICE_NAME_MAX_LEN);
            cfm->req = param->req;
            cfm->info.name.length = appm_get_dev_name(cfm->info.name.value);
            // Send message
            ((ke_msg_send_handler)SVC_ke_msg_send)(cfm);
        } break;

        case GAPC_DEV_APPEARANCE:
        {
            // Allocate message
            struct gapc_get_dev_info_cfm *cfm = KE_MSG_ALLOC(GAPC_GET_DEV_INFO_CFM,
                                                             src_id, dest_id,
                                                             gapc_get_dev_info_cfm);
            cfm->req = param->req;
            // Set the device appearance
            #if (BLE_APP_HT)
            // Generic Thermometer - TODO: Use a flag
            cfm->info.appearance = 728;
            #elif (BLE_APP_HID)
            // HID Mouse
            cfm->info.appearance = GAP_APPEARE_GENERIC_HID;
            #else
            // No appearance
            cfm->info.appearance = 0;
            #endif

            // Send message
            ((ke_msg_send_handler)SVC_ke_msg_send)(cfm);
        } break;

        case GAPC_DEV_SLV_PREF_PARAMS:
        {
printf("READ CON PARAM\n");
            // Allocate message
            struct gapc_get_dev_info_cfm *cfm = KE_MSG_ALLOC(GAPC_GET_DEV_INFO_CFM,
                                                            src_id, 
                                                            dest_id,
                                                            gapc_get_dev_info_cfm);
            
            cfm->req = param->req;
            // Slave preferred Connection interval Min
            cfm->info.slv_params.con_intv_min = 8;
            // Slave preferred Connection interval Max
            cfm->info.slv_params.con_intv_max = 10;
            // Slave preferred Connection latency
            cfm->info.slv_params.slave_latency  = 0;
            // Slave preferred Link supervision timeout
            cfm->info.slv_params.conn_timeout    = 200;  // 2s (500*10ms)

            // Send message
            ((ke_msg_send_handler)SVC_ke_msg_send)(cfm);
        } break;

        default: /* Do Nothing */ break;
    }


    return (KE_MSG_CONSUMED);
}
/**
 ****************************************************************************************
 * @brief Handles GAPC_SET_DEV_INFO_REQ_IND message.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_set_dev_info_req_ind_handler(ke_msg_id_t const msgid,
        struct gapc_set_dev_info_req_ind const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    // Set Device configuration
    struct gapc_set_dev_info_cfm* cfm = KE_MSG_ALLOC(GAPC_SET_DEV_INFO_CFM, src_id, dest_id,
                                                     gapc_set_dev_info_cfm);
    // Reject to change parameters
    cfm->status = GAP_ERR_REJECTED;
    cfm->req = param->req;
    // Send message
    ((ke_msg_send_handler)SVC_ke_msg_send)(cfm);

    return (KE_MSG_CONSUMED);
}

static int gapc_peer_features_ind_handler(ke_msg_id_t const msgid,
                               struct gapc_peer_features_ind const *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id)
{
	app_get_info_ind_func((void*)param,GAPC_GET_PEER_FEATURES);
	return KE_MSG_CONSUMED;
}

static int gapc_param_update_req_ind_handler(ke_msg_id_t const msgid,
                                           struct gapc_param_update_req_ind const *param,
                                           ke_task_id_t const dest_id,
                                           ke_task_id_t const src_id)
{
//please check con param here.
	struct gapc_param_update_cfm *cfm = KE_MSG_ALLOC(GAPC_PARAM_UPDATE_CFM, src_id, dest_id,
                                                     gapc_param_update_cfm);
	cfm->accept = true;
	cfm->ce_len_max = 0xffff;
	cfm->ce_len_min = 0xffff;
    ((ke_msg_send_handler)SVC_ke_msg_send)(cfm);

    return (KE_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handles connection complete event from the GAP. Enable all required profiles
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_connection_req_ind_handler(ke_msg_id_t const msgid,
                                           struct gapc_connection_req_ind const *param,
                                           ke_task_id_t const dest_id,
                                           ke_task_id_t const src_id)
{
    app_env.conidx = KE_IDX_GET(src_id);

    // Check if the received Connection Handle was valid
    if (app_env.conidx != GAP_INVALID_CONIDX)
    {
        // Retrieve the connection info from the parameters
        app_env.conhdl = param->conhdl;

        // Clear the advertising timeout timer
        if (((ke_timer_active_handler)SVC_ke_timer_active)(APP_ADV_TIMEOUT_TIMER, TASK_APP))
        {
            ((ke_timer_clear_handler)SVC_ke_timer_clear)(APP_ADV_TIMEOUT_TIMER, TASK_APP);
        }
		// Clear active connection timeout timer
        if (((ke_timer_active_handler)SVC_ke_timer_active)(APP_CONN_TIMEOUT_TIMER, TASK_APP))
        {
            ((ke_timer_clear_handler)SVC_ke_timer_clear)(APP_CONN_TIMEOUT_TIMER, TASK_APP);
        }

		
        // Send connection confirmation
        struct gapc_connection_cfm *cfm = KE_MSG_ALLOC(GAPC_CONNECTION_CFM,
                KE_BUILD_ID(TASK_GAPC, app_env.conidx), TASK_APP,
                gapc_connection_cfm);

        #if(BLE_APP_SEC)
        cfm->auth      = app_sec_get_bond_status() ? GAP_AUTH_REQ_NO_MITM_BOND : GAP_AUTH_REQ_NO_MITM_NO_BOND; // TODO [FBE] restore valid data
        cfm->ltk_present = true;
		cfm->svc_changed_ind_enable = true;
		cfm->lsign_counter = 0;
		cfm->lsign_counter = 0;
		//cfm->lcsrk = ;
		//cfm->rcsrk = ;
        #else // !(BLE_APP_SEC)
        cfm->auth      = GAP_AUTH_REQ_NO_MITM_NO_BOND;
        #endif // (BLE_APP_SEC)
        // Send the message
        ((ke_msg_send_handler)SVC_ke_msg_send)(cfm);


        // We are now in connected State
        ((ke_state_set_handler)SVC_ke_state_set)(dest_id, APPM_CONNECTED);

		if(param->role == 1)   //slave then inform
			app_connection_req_ind_func(app_env.conidx, param);
		else if(param->role == 0)	//master then inform
			app_connection_success_ind_func(app_env.conidx);
		
    }
    else
    {
        // No connection has been establish, restart advertising
        appm_start_advertising(NULL);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles GAP controller command complete events.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_cmp_evt_handler(ke_msg_id_t const msgid,
                                struct gapc_cmp_evt const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    switch(param->operation)
    {
        case (GAPC_UPDATE_PARAMS):
        {
            if (param->status != GAP_ERR_NO_ERROR)
            {
//                appm_disconnect();
				// it's application specific what to do when the Param Upd request is rejected
				app_update_params_rejected_ind_func(param->status);

            }
			else
			{
				// Go to Connected State
				((ke_state_set_handler)SVC_ke_state_set)(dest_id, APPM_CONNECTED);
				// if state is APPM_CONNECTED then the request was accepted
				app_update_params_complete_ind_func();
			}
        } break;
		case GAPC_SECURITY_REQ:
		{
			printf("send sec req completed,status:%d\n",param->status);
		}
		break;
		
		case GAPC_ENCRYPT:
		{
			printf("send enc cmd completed,status:%d\n",param->status);
		}
		break;
		
        default:
        {
//			if(param->status != GAP_ERR_NO_ERROR)
//            {
//                ASSERT_ERR(0); // unexpected error
//            }
        } break;
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles disconnection complete event from the GAP.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_disconnect_ind_handler(ke_msg_id_t const msgid,
                                      struct gapc_disconnect_ind const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    // Go to the ready state
    ((ke_state_set_handler)SVC_ke_state_set)(TASK_APP, APPM_READY);

    #if (BLE_APP_HT)
    // Stop interval timer
    app_stop_timer();
    #endif //(BLE_APP_HT)

    #if (DISPLAY_SUPPORT)
    // Update Connection State screen
    app_display_set_con(false);
    #endif //(DISPLAY_SUPPORT)

	app_disconnect_ind_func(dest_id, param);

    return (KE_MSG_CONSUMED);
}


static int gapm_profile_added_ind_handler(ke_msg_id_t const msgid,
                                          struct gapm_profile_added_ind *param,
                                          ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id)
{
    // Current State
    uint8_t state            = ((ke_state_get_handler)SVC_ke_state_get)(dest_id);

    if (state == APPM_CREATE_DB)
    {
        switch (param->prf_task_id)
        {
			#if defined CFG_PRF_BASS
            case TASK_ID_BASS:
            {
                app_batt_enable_prf(param->prf_task_nb);
            }
            break;
            #endif
			
			#if defined CFG_PRF_HOGPD
            case TASK_ID_HOGPD:
            {
                app_hid_enable_prf(param->prf_task_nb);
            }
            break;
			#endif
            
            #if (PROJ_OTA_OTA)
            case TASK_ID_OTA:
            {
                app_ota_enable_server_prf(param->prf_task_nb);
            }
            #endif
            default: /* Nothing to do */ 
            {
            }break;
        }
    }
    else
    {
        ASSERT_INFO(0, state, src_id);
    }

    return KE_MSG_CONSUMED;
}



static int gapm_adv_report_ind_handler(ke_msg_id_t const msgid,
										  struct gapm_adv_report_ind *param,
										  ke_task_id_t const dest_id,
										  ke_task_id_t const src_id)
{
	app_adv_report_ind_func(param);
	return KE_MSG_CONSUMED;
}

/**
 ****************************************************************************************
 * @brief Handles reception of all messages sent from the lower layers to the application
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int appm_msg_handler(ke_msg_id_t const msgid,
                            void *param,
                            ke_task_id_t const dest_id,
                            ke_task_id_t const src_id)
{
    // Retrieve identifier of the task from received message
    ke_task_id_t src_task_id = MSG_T(msgid);
    // Message policy
    uint8_t msg_pol          = KE_MSG_CONSUMED;


    switch (src_task_id)
    {
        case (TASK_ID_GAPC):
        {
            #if (BLE_APP_SEC)
            if ((msgid >= GAPC_BOND_CMD) && (msgid <= GAPC_SECURITY_IND))
            {
                // Call the Security Module
                msg_pol = appm_get_handler(&app_sec_table_handler, msgid, param, src_id);
            }
            #endif //(BLE_APP_SEC)
            // else drop the message
        } break;

        case (TASK_ID_GATTC):
        {
            // Service Changed - Drop
        } break;
		
		#if (BLE_APP_PROJ_TEMPLATE)
		case TASK_ID_PROJ_TEMPLATE_SERVER:
		{
			msg_pol = appm_get_handler(&app_proj_template_table_handler, msgid, param, src_id);
		}break;
		#endif
		
		#if defined(CFG_PRF_HOGPD)
        case TASK_ID_HOGPD:
        {
            msg_pol = appm_get_handler(&app_hid_table_handler, msgid, param, src_id);
        }break;
		#endif
		
        #if defined(CFG_PRF_DISS)
        case TASK_ID_DISS:
        {
            msg_pol = appm_get_handler(&app_dis_table_handler, msgid, param, src_id);
        }
        break;
		#endif
		
        #if defined(CFG_PRF_BASS)
        case TASK_ID_BASS:
        {
            msg_pol = appm_get_handler(&app_batt_table_handler, msgid, param, src_id);
        }
        break;
		#endif
		
		#if (USER_RGB_LIGHT)
		case TASK_ID_RGB_SERVER1:
		{
			msg_pol = appm_get_handler(&app_rgb1_table_handler, msgid, param, src_id);
		}break;
		
		case TASK_ID_RGB_SERVER2:
		{
			msg_pol = appm_get_handler(&app_rgb2_table_handler, msgid, param, src_id);
		}break;
		#endif
		
		#if (USER_PROJ_CENTRAL)
		case (TASK_ID_TMP_CLIENT):
		{
			// Call the Battery Module
			msg_pol = appm_get_handler(&app_tmp_table_handler, msgid, param, src_id);
		} break;
		#endif //
		#if (PROJ_OTA)
        case (TASK_ID_OTA):
		{
			msg_pol = appm_get_handler(&app_ota_table_handler, msgid, param, src_id);
		} break;
        #endif
		
        default:
        {
        } break;
    }

    return (msg_pol);
}

/*
 * GLOBAL VARIABLES DEFINITION
 ****************************************************************************************
 */


/* Default State handlers definition. */
KE_MSG_HANDLER_TAB(appm)
{
    // Note: first message is latest message checked by kernel so default is put on top.
    {KE_MSG_DEFAULT_HANDLER,    (ke_msg_func_t)appm_msg_handler},

    {APP_ADV_TIMEOUT_TIMER,     (ke_msg_func_t)app_adv_timeout_handler},
    {APP_CONN_TIMEOUT_TIMER,    (ke_msg_func_t)app_conn_timerout_handler},
	#if(!USER_PROJ_CENTRAL)
	{APP_CONN_UPDATE_TIMER,     (ke_msg_func_t)app_conn_update_handler}, 
	#endif
    
    #if(PROJ_MIDI)
	{APP_MIDI_TIMER,            (ke_msg_func_t)app_midi_handler}, 
    #endif
    
	#if(PROJ_THROUGHPUT)
	{APP_TEST_TIMER,			(ke_msg_func_t)app_test_handler},	
	#endif

    {GAPM_DEVICE_READY_IND,     (ke_msg_func_t)gapm_device_ready_ind_handler},	//gapm ready,reset stack
    {GAPM_CMP_EVT,              (ke_msg_func_t)gapm_cmp_evt_handler},		//gapm cmp,then add svc
    {GAPC_GET_DEV_INFO_REQ_IND, (ke_msg_func_t)gapc_get_dev_info_req_ind_handler},
    {GAPC_SET_DEV_INFO_REQ_IND, (ke_msg_func_t)gapc_set_dev_info_req_ind_handler},
    //DYC added to rsp the slave update param req
    {GAPC_PARAM_UPDATE_REQ_IND,	(ke_msg_func_t)gapc_param_update_req_ind_handler},
    {GAPC_CONNECTION_REQ_IND,   (ke_msg_func_t)gapc_connection_req_ind_handler},
    {GAPC_CMP_EVT,              (ke_msg_func_t)gapc_cmp_evt_handler},			//nothing
    {GAPC_DISCONNECT_IND,       (ke_msg_func_t)gapc_disconnect_ind_handler},
    {GAPM_PROFILE_ADDED_IND,    (ke_msg_func_t)gapm_profile_added_ind_handler},	//prf added  nothing
    {GAPM_ADV_REPORT_IND,    	(ke_msg_func_t)gapm_adv_report_ind_handler},	//prf added  nothing

	{GAPC_PEER_FEATURES_IND,    (ke_msg_func_t)gapc_peer_features_ind_handler},	//prf added  nothing
    
};

/* Defines the place holder for the states of all the task instances. */
ke_state_t appm_state[APP_IDX_MAX];
const struct ke_state_handler appm_default_handler = KE_STATE_HANDLER(appm_msg_handler_tab);

const struct ke_task_desc TASK_DESC_APP = {NULL, &appm_default_handler,
                                                  appm_state, APPM_STATE_MAX, APP_IDX_MAX,ARRAY_LEN(appm_msg_handler_tab)};



#endif //(BLE_APP_PRESENT)

/// @} APPTASK
