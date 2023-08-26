
/**
 ****************************************************************************************
 * @addtogroup APP
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "panip_config.h"             // SW configuration

#if (BLE_APP_PRESENT)
#include <string.h>

#include "app_task.h"                // Application task Definition
#include "app.h"                     // Application Definition
#include "gap.h"                     // GAP Definition
#include "gapm_task.h"               // GAP Manager Task API
#include "gapc_task.h"               // GAP Controller Task API

#include "co_bt.h"                   // Common BT Definition
#include "co_math.h"                 // Common Maths Definition

#include "ke_timer.h"            			// kernel timer

#if (BLE_PROJ_TEMPLATE_SERVER)
#include "app_proj_template.h"
#endif

#if (PROJ_HID_KEYBOARD)
#include "app_hid_keyboard.h"
#include "proj_hid_keyboard.h"
#include "app_batt.h"
#include "app_dis.h"
#endif // (PROJ_HID_KEYBOARD)

#if (PROJ_HID_SELFIE)
#include "app_hid_selfie.h"
#include "proj_hid_selfie.h"
#include "app_batt.h"
#include "app_dis.h"
#endif // (PROJ_HID_SELFIE)

#if (USER_RGB_LIGHT)
#include "app_rgb_light1.h"
#include "app_rgb_light2.h"
#endif

#if (USER_PROJ_CENTRAL)
#include "app_tmp.h"               
#endif //(BLE_APP_TMP)

#if (PROJ_OTA)
#include "app_ota.h"
#endif

#include "app_proj_event.h"
#include "peripherals.h"
#include "stack_svc_api.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/// Default Device Name if no value can be found in NVDS
#define APP_DFLT_DEVICE_NAME            app_info.co_default_bdname
#define APP_DFLT_DEVICE_NAME_LEN        (strlen(app_info.co_default_bdname))


#define DEVICE_NAME_SIZE    sizeof(DEVICE_NAME)



/**
 * Advertising Parameters
 */
/// Default Advertising duration - 30s (in multiple of 10ms)
#define APP_DFLT_ADV_DURATION   (3000)

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

typedef void (*appm_add_svc_func_t)(void);

/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/// List of service to add in the database
enum appm_svc_list
{	
	#if BLE_PROJ_TEMPLATE_SERVER
	APPM_PROJ_TEMPLATE_SERVER,
	#endif
	
	#if (PROJ_HID_KEYBOARD | PROJ_HID_SELFIE)
	APPM_SVC_HID,
    APPM_SVC_DISS,
    APPM_SVC_BASS,
	#endif
	#if USER_RGB_LIGHT
	APPM_RGB_SERVER1,
	APPM_RGB_SERVER2,
	#endif
	#if (BLE_TMP_CLIENT)
    APPM_CLNT_TMP,
	#endif
	#if (PROJ_OTA)
    APPM_OTA_SERVER,
    #endif
	
    APPM_SVC_LIST_STOP,
};

/// List of functions used to create the database
static const appm_add_svc_func_t appm_add_svc_func_list[APPM_SVC_LIST_STOP+1] =
{
	#if (BLE_PROJ_TEMPLATE_SERVER)
	(appm_add_svc_func_t)app_proj_template_add_server,
	#endif
	
	#if (PROJ_HID_KEYBOARD | PROJ_HID_SELFIE)
    (appm_add_svc_func_t)app_hid_add_hids,
    (appm_add_svc_func_t)app_dis_add_dis,
    (appm_add_svc_func_t)app_batt_add_bas,
	#endif
	#if (USER_RGB_LIGHT)
	(appm_add_svc_func_t)app_rgb1_add_server,
	(appm_add_svc_func_t)app_rgb2_add_server,
	#endif
	#if(BLE_TMP_CLIENT)
    (appm_add_svc_func_t)app_tmp_add_client,
	#endif 
	#if (PROJ_OTA)
    (appm_add_svc_func_t)app_ota_add_server,
    #endif
	
	NULL,
};



/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// Application Environment Structure
struct app_env_tag app_env;

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void appm_init()
{
    #if (NVDS_SUPPORT)
    uint8_t key_len = KEY_LEN;
    #endif //(NVDS_SUPPORT)

    // Reset the application manager environment
    memset(&app_env, 0, sizeof(app_env));

    // Create APP task
    ((ke_task_create_handler)SVC_ke_task_create)(TASK_APP, &TASK_DESC_APP);

    // Initialize Task state
    ((ke_state_set_handler)SVC_ke_state_set)(TASK_APP, APPM_INIT);

    // Load the device name from NVDS
    // Get the Device Name to add in the Advertising Data (Default one or NVDS one)
    #if (NVDS_SUPPORT)
	
    app_env.dev_name_len = APP_DEVICE_NAME_MAX_LEN;
    if (nvds_get(NVDS_TAG_DEVICE_NAME, &(app_env.dev_name_len), app_env.dev_name) != NVDS_OK)
    #endif //(NVDS_SUPPORT)
    {
        // Get default Device Name (No name if not enough space)
        memcpy(app_env.dev_name, APP_DFLT_DEVICE_NAME, APP_DFLT_DEVICE_NAME_LEN);
        app_env.dev_name_len = APP_DFLT_DEVICE_NAME_LEN;

        // TODO update this value per profiles
    }

    #if (NVDS_SUPPORT)
    if (nvds_get(NVDS_TAG_LOC_IRK, &key_len, app_env.loc_irk) != NVDS_OK)
    #endif //(NVDS_SUPPORT)
    {
        uint8_t counter;

        // generate a new IRK
        for (counter = 0; counter < KEY_LEN; counter++)
        {
            app_env.loc_irk[counter]    = (uint8_t)co_rand_word();
        }

        // Store it in NVDS
        #if (NVDS_SUPPORT)
        // Store the generated value in NVDS
        if (nvds_put(NVDS_TAG_LOC_IRK, KEY_LEN, (uint8_t *)&app_env.loc_irk) != NVDS_OK)
        {
            ASSERT_INFO(0, NVDS_TAG_LOC_IRK, 0);
        }
        #endif // #if (NVDS_SUPPORT)
    }
		//For tmp project
		app_init_ind_func();
}

bool appm_add_svc(void)
{
    ((ke_state_set_handler)SVC_ke_state_set)(TASK_APP, APPM_CREATE_DB);

    // Indicate if more services need to be added in the database
    bool more_svc = false;
	
    // Check if another should be added in the database
    if (app_env.next_svc != APPM_SVC_LIST_STOP)
    {
        ASSERT_INFO(appm_add_svc_func_list[app_env.next_svc] != NULL, app_env.next_svc, 1);

        // Call the function used to add the required service
        appm_add_svc_func_list[app_env.next_svc]();
  
        // Select following service to add
        app_env.next_svc++;
        more_svc = true;
    }

    return more_svc;
}



void appm_set_dev_configuration(struct gapm_set_dev_config_cmd *msg)
{
	struct gapm_set_dev_config_cmd* cmd = KE_MSG_ALLOC(GAPM_SET_DEV_CONFIG_CMD,
	                                   TASK_GAPM, TASK_APP,
	                                   gapm_set_dev_config_cmd);
	if(msg == NULL)
	{
	    cmd->operation = GAPM_SET_DEV_CONFIG;
	    // Device Role
	    cmd->role = GAP_ROLE_ALL;
	    /// GAP service start handle
	    cmd->gap_start_hdl = 0;
	    /// GATT service start handle
	    cmd->gatt_start_hdl = 0;

	    // Set Data length parameters
	    cmd->sugg_max_tx_octets = BLE_MIN_OCTETS;
	    cmd->sugg_max_tx_time	= BLE_MIN_TIME;
	    // Do not support secure connections
	    cmd->pairing_mode = GAPM_PAIRING_LEGACY;
	    cmd->att_cfg = GAPM_MASK_ATT_SVC_CHG_EN | GAPM_MASK_ATT_SLV_PREF_CON_PAR_EN 
						| GAPM_MASK_ATT_APPEARENCE_PERM | GAPM_MASK_ATT_NAME_PERM;
	    cmd->api_settings = true;

	    cmd->gap_start_hdl = 0;
	    cmd->gatt_start_hdl = 12;

	    // load IRK
	    memcpy(cmd->irk.key, app_env.loc_irk, KEY_LEN);
	    //Defined maximum transmission unit
	    cmd->max_mtu = BLE_MIN_OCTETS;//160;
	    cmd->max_mps = BLE_MIN_TIME;
	}
	else
		memcpy(cmd,msg,sizeof(struct gapm_set_dev_config_cmd));

	((ke_msg_send_handler)SVC_ke_msg_send)(cmd);
}


/**
 ****************************************************************************************
 * Advertising Functions
 ****************************************************************************************
 */

void appm_start_scanning(struct gapm_start_scan_cmd *msg)
{
	// Check if the advertising procedure is already is progress
	//if (((ke_state_get_handler)SVC_ke_state_get)(TASK_APP) == APPM_READY)
	{
		// Prepare the GAPM_START_ADVERTISE_CMD message
		struct gapm_start_scan_cmd *cmd = KE_MSG_ALLOC(GAPM_START_SCAN_CMD,
														TASK_GAPM, TASK_APP,
														gapm_start_scan_cmd);
		if(msg == NULL)
		{
			cmd->mode = GAP_OBSERVER_MODE; //GAP_OBSERVER_MODE;//GAP_GEN_DISCOVERY;
			cmd->op.code = GAPM_SCAN_PASSIVE;
			cmd->op.addr_src = GAPM_STATIC_ADDR;
			cmd->filter_duplic = SCAN_FILT_DUPLIC_DIS;
			cmd->interval = 240;//160;	//150ms
			cmd->window = 40;	
		}
		else
			memcpy(cmd,msg,sizeof(struct gapm_start_scan_cmd));		

		((ke_state_set_handler)SVC_ke_state_set)(TASK_APP, APPM_READY);
		// Send the message
		((ke_msg_send_handler)SVC_ke_msg_send)(cmd);
	}
}

void appm_stop_scanning(void)
{
	//if (((ke_state_get_handler)SVC_ke_state_get)(TASK_APP) == APPM_READY)
	{
		struct gapm_cancel_cmd *cmd =(struct gapm_cancel_cmd *) KE_MSG_ALLOC(GAPM_CANCEL_CMD
			, TASK_GAPM, TASK_APP,gapm_cancel_cmd);
		cmd->operation = GAPM_CANCEL;
		((ke_msg_send_handler)SVC_ke_msg_send)(cmd);
	}
}


void app_adv_func_default(struct gapm_start_advertise_cmd *cmd)
{
     //  Device Name Length
    uint8_t device_name_length;
    int8_t device_name_avail_space;
    uint8_t device_name_temp_buf[64];
    
    cmd->op.code     = GAPM_ADV_UNDIRECT;//GAPM_ADV_UNDIRECT;//GAPM_ADV_NON_CONN;
    cmd->op.addr_src = GAPM_STATIC_ADDR;
	
//useless, must < intv_max
    cmd->intv_min    = 	0x20;			//APP_ADV_INT_MIN;				//0x40;

// decided evt->interval, 0x40*625us 
    cmd->intv_max    = 	1600;		//APP_ADV_INT_MAX;	1600		0x40
    cmd->channel_map = APP_ADV_CHMAP;
	
		cmd->info.host.adv_filt_policy = ADV_ALLOW_SCAN_ANY_CON_ANY;
    cmd->info.host.mode = GAP_GEN_DISCOVERABLE;

    // Get remaining space in the Advertising Data - 2 bytes are used for name length/flag
    device_name_avail_space = APP_ADV_DATA_MAX_SIZE - 2;

    // Check if data can be added to the Advertising data
    if (device_name_avail_space > 0)
    {
        // Get default Device Name (No name if not enough space)
        device_name_length = strlen("PanChip_BLE_TEM");
        memcpy(&device_name_temp_buf[0], "PanChip_BLE_TEM", device_name_length);

        if(device_name_length > 0)
        {
            // Check available space
            device_name_length = co_min(device_name_length, device_name_avail_space);
            // Fill Length
            cmd->info.host.adv_data[cmd->info.host.adv_data_len]     = device_name_length + 1;
            // Fill Device Name Flag
            cmd->info.host.adv_data[cmd->info.host.adv_data_len + 1] = '\x09';
            // Copy device name
            memcpy(&cmd->info.host.adv_data[cmd->info.host.adv_data_len + 2], device_name_temp_buf, device_name_length);
            // Update Advertising Data Length
            cmd->info.host.adv_data_len += (device_name_length + 2);
        }
    }

    return;
}


void appm_start_advertising(struct gapm_start_advertise_cmd *msg)
{
    // Check if the advertising procedure is already is progress
    //if (((ke_state_get_handler)SVC_ke_state_get)(TASK_APP) == APPM_READY)
    {
        // Prepare the GAPM_START_ADVERTISE_CMD message
        struct gapm_start_advertise_cmd *cmd = KE_MSG_ALLOC(GAPM_START_ADVERTISE_CMD,
                                                            TASK_GAPM, TASK_APP,
                                                            gapm_start_advertise_cmd);

        #if (BLE_APP_HID)
        /*
         * If the peripheral is already bonded with a central device, use the direct advertising
         * procedure (BD Address of the peer device is stored in NVDS.
         */
        if (app_sec_get_bond_status())
        {
            cmd->intv_min = 32;
            cmd->intv_max = 32;
        }
        else
        {
            cmd->intv_min = APP_ADV_INT_MIN;
            cmd->intv_max = APP_ADV_INT_MAX;
        }
        #else //(BLE_APP_HID)
        cmd->intv_min = APP_ADV_INT_MIN;
        cmd->intv_max = APP_ADV_INT_MAX;
        #endif //(BLE_APP_HID)
		
        if(msg == NULL)
					app_adv_func_default(cmd);
				else
					memcpy(cmd,msg,sizeof(struct gapm_start_advertise_cmd));
		if(cmd->info.host.mode == GAP_LIM_DISCOVERABLE)
		{
			((ke_timer_set_handler)SVC_ke_timer_set)(APP_ADV_TIMEOUT_TIMER, TASK_APP, (uint16_t)APP_DFLT_ADV_DURATION);
		}
        // Send the message
        ((ke_msg_send_handler)SVC_ke_msg_send)(cmd);
        // Set the state of the task to APPM_ADVERTISING
        ((ke_state_set_handler)SVC_ke_state_set)(TASK_APP, APPM_ADVERTISING);
    }
    // else ignore the request
}

void appm_start_beacon(const uint8_t *p_adv_data, uint8_t adv_data_len)
{
    // Check if the advertising procedure is already is progress
    //if (((ke_state_get_handler)SVC_ke_state_get)(TASK_APP) == APPM_READY)
    {
        // Prepare the GAPM_START_ADVERTISE_CMD message
        struct gapm_start_advertise_cmd *cmd = KE_MSG_ALLOC(GAPM_START_ADVERTISE_CMD,
                                                            TASK_GAPM, TASK_APP,
                                                            gapm_start_advertise_cmd);

        cmd->op.code	 = GAPM_ADV_NON_CONN;
        cmd->op.addr_src = GAPM_STATIC_ADDR;
        cmd->intv_min	 =	160;			//APP_ADV_INT_MIN;				
        cmd->intv_max	 =	160;			//100ms
        cmd->channel_map = APP_ADV_CHMAP;

        cmd->info.host.mode = GAP_BROADCASTER_MODE;
        cmd->info.host.adv_filt_policy = ADV_ALLOW_SCAN_ANY_CON_ANY;

        memcpy(cmd->info.host.adv_data, p_adv_data, adv_data_len);
        
        cmd->info.host.adv_data_len = adv_data_len;
        
        // Send the message
        ((ke_msg_send_handler)SVC_ke_msg_send)(cmd);
        // Set the state of the task to APPM_ADVERTISING
        ((ke_state_set_handler)SVC_ke_state_set)(TASK_APP, APPM_ADVERTISING);
    }
    // else ignore the request
}

void appm_stop_advertising(void)
{
    //if (((ke_state_get_handler)SVC_ke_state_get)(TASK_APP) == APPM_ADVERTISING)
    {
        // Go in ready state
        ((ke_state_set_handler)SVC_ke_state_set)(TASK_APP, APPM_READY);

        // Prepare the GAPM_CANCEL_CMD message
        struct gapm_cancel_cmd *cmd = KE_MSG_ALLOC(GAPM_CANCEL_CMD,
                                                   TASK_GAPM, TASK_APP,
                                                   gapm_cancel_cmd);

        cmd->operation = GAPM_CANCEL;

        // Send the message
        ((ke_msg_send_handler)SVC_ke_msg_send)(cmd);
    }
    // else ignore the request
}

void appm_add_white_list ( struct gap_bdaddr *addr )
{
    struct gapm_white_list_mgt_cmd *cmd = KE_MSG_ALLOC_DYN ( GAPM_WHITE_LIST_MGT_CMD, 
                                                             TASK_GAPM, 
                                                             TASK_APP, 
                                                             gapm_white_list_mgt_cmd, 
                                                             sizeof(struct gap_bdaddr) );
    cmd->operation = GAPM_ADD_DEV_IN_WLIST;
    cmd->nb = 1;
    addr->addr_type = 1; // 0,public;1,random
    memcpy ( &cmd->devices[0], addr, sizeof(struct gap_bdaddr) );
    
    ((ke_msg_send_handler)SVC_ke_msg_send)(cmd);
}

void appm_clear_white_list ( void )
{
    struct gapm_white_list_mgt_cmd *cmd = KE_MSG_ALLOC ( GAPM_WHITE_LIST_MGT_CMD, 
                                                         TASK_GAPM, 
                                                         TASK_APP,
                                                         gapm_white_list_mgt_cmd );
    cmd->operation = GAPM_CLEAR_WLIST;
    
    ((ke_msg_send_handler)SVC_ke_msg_send)(cmd);
}


void app_reset_app(void)
{
	struct gapm_reset_cmd* cmd = KE_MSG_ALLOC(GAPM_RESET_CMD, TASK_GAPM, TASK_APP,gapm_reset_cmd);		
	cmd->operation = GAPM_RESET;// Set GAPM_RESET
	((ke_msg_send_handler)SVC_ke_msg_send)(cmd);// Send the message
}



void appm_update_param(struct gapc_conn_param *conn_param)
{
    // Prepare the GAPC_PARAM_UPDATE_CMD message
    struct gapc_param_update_cmd *cmd = KE_MSG_ALLOC(GAPC_PARAM_UPDATE_CMD,
                                                     KE_BUILD_ID(TASK_GAPC, app_env.conidx), TASK_APP,
                                                     gapc_param_update_cmd);

    cmd->operation  = GAPC_UPDATE_PARAMS;
    cmd->intv_min   = conn_param->intv_min;
    cmd->intv_max   = conn_param->intv_max;
    cmd->latency    = conn_param->latency;
    cmd->time_out   = conn_param->time_out;

    // not used by a slave device
    cmd->ce_len_min = 0xFFFF;
    cmd->ce_len_max = 0xFFFF;

    // Send the message
    ((ke_msg_send_handler)SVC_ke_msg_send)(cmd);
}


void appm_get_peer_info(uint16_t conidx,uint8_t operation)
{
	// Send a GAPC_GET_INFO_CMD in order to read the device name characteristic value
	struct gapc_get_info_cmd *p_cmd = KE_MSG_ALLOC(GAPC_GET_INFO_CMD,
												   KE_BUILD_ID(TASK_GAPC, conidx), TASK_APP,
												   gapc_get_info_cmd);
	p_cmd->operation = operation;
	((ke_msg_send_handler)SVC_ke_msg_send)(p_cmd);
}

//For master only
void appm_start_bond_cmd(void)
{
	// generate a local bond command to update correctly internal state machine
	struct gapc_bond_cmd *cmd = KE_MSG_ALLOC(GAPC_BOND_CMD,
			KE_BUILD_ID(TASK_GAPC, app_env.conidx), TASK_APP,
			gapc_bond_cmd);
	cmd->operation = GAPC_BOND;
	cmd->pairing.auth= GAP_AUTH_REQ_MITM_BOND;
	cmd->pairing.ikey_dist= GAP_KDIST_ENCKEY | GAP_KDIST_IDKEY | GAP_KDIST_SIGNKEY;

	cmd->pairing.iocap=GAP_IO_CAP_NO_INPUT_NO_OUTPUT;

	cmd->pairing.key_size=KEY_LEN;
	cmd->pairing.oob=GAP_OOB_AUTH_DATA_NOT_PRESENT;
	cmd->pairing.rkey_dist= GAP_KDIST_ENCKEY | GAP_KDIST_IDKEY | GAP_KDIST_SIGNKEY;
	cmd->pairing.sec_req=GAP_NO_SEC;

	// Keep the packet for later use
	//memcpy(&(cmd->pairing), &(pdu->data.pairing_req.iocap), SMPC_CODE_PAIRING_REQ_RESP_LEN - 1);

	((ke_msg_send_handler)SVC_ke_msg_send)(cmd);
}

/**
 ****************************************************************************************
 * @brief Start the connection to the device with address connect_bdaddr
 * 
 * @return void
 ****************************************************************************************
 */
void app_start_connecting(struct gapm_start_connection_cmd *cmd)
{
    struct gapm_start_connection_cmd *msg = NULL;

	if(cmd == NULL)
	{
		    // create a kenrel message to start connecting  with an advertiser
	    msg = (struct gapm_start_connection_cmd *) KE_MSG_ALLOC_DYN(GAPM_START_CONNECTION_CMD 
					, TASK_GAPM, TASK_APP, gapm_start_connection_cmd, sizeof(struct gap_bdaddr));
			
		msg->op.code = GAPM_CONNECTION_DIRECT;
		msg->op.addr_src = GAPM_STATIC_ADDR;

		msg->scan_interval = 0xFFFF;	//no use,  must > scan_window, else wrong

		msg->scan_window = (msg->con_intv_min*2-6);

		// Maximal peer connection
		msg->nb_peers = 1;
		// Connect to a certain adress
		memcpy(&msg->peers[0].addr, &connect_bdaddr.addr, BD_ADDR_LEN);
		msg->peers[0].addr_type = connect_bdaddr.addr_type;

		msg->con_intv_min = 20;	// number * 1.25 = connection interval 48.75 ms	= 7
		msg->con_intv_max = 20;	// number * 1.25 = connection interval 48.75 ms = 7 
		msg->ce_len_min = 32; 	//0x20 * 625us
		msg->ce_len_max = 32;

//con_req pdu send it to peer.
		msg->con_latency = 0;	
//con_req pdu send it to peer.
		msg->superv_to = 100;	//1sec		 //0x7D0;	//	20 seconden
	}
	else
	{
			    // create a kenrel message to start connecting  with an advertiser
	    msg = (struct gapm_start_connection_cmd *) KE_MSG_ALLOC_DYN(GAPM_START_CONNECTION_CMD 
					, TASK_GAPM, TASK_APP, gapm_start_connection_cmd, cmd->nb_peers * sizeof(struct gap_bdaddr));
		memcpy(msg,cmd,sizeof(struct gapm_start_connection_cmd)+ cmd->nb_peers * sizeof(struct gap_bdaddr));
	}
	
	//set timer
   	((ke_timer_set_handler)SVC_ke_timer_set)(APP_CONN_TIMEOUT_TIMER, TASK_APP, 700);	//7 sec
   	
	if(msg != NULL)
    	((ke_msg_send_handler)SVC_ke_msg_send)(msg);    

	((ke_state_set_handler)SVC_ke_state_set)(TASK_APP, APPM_CONNECTING);

}


void appm_stop_connecting(void)
{
    if (((ke_state_get_handler)SVC_ke_state_get)(TASK_APP) == APPM_CONNECTING)
    {
        // Stop the advertising timer if needed
        if (((ke_timer_active_handler)SVC_ke_timer_active)(APP_CONN_TIMEOUT_TIMER, TASK_APP))
        {
            ((ke_timer_clear_handler)SVC_ke_timer_clear)(APP_CONN_TIMEOUT_TIMER, TASK_APP);
        }
		        // Go in ready state
        ((ke_state_set_handler)SVC_ke_state_set)(TASK_APP, APPM_READY);
        struct gapm_cancel_cmd *cmd = KE_MSG_ALLOC(GAPM_CANCEL_CMD,
                                                   TASK_GAPM, TASK_APP,
                                                   gapm_cancel_cmd);
        cmd->operation = GAPM_CANCEL;
        // Send the message
        ((ke_msg_send_handler)SVC_ke_msg_send)(cmd);
    }
}



//Only use for internal DBs,  user_app use
void appm_disconnect(void)
{
    struct gapc_disconnect_cmd *cmd = KE_MSG_ALLOC(GAPC_DISCONNECT_CMD,
                                                   KE_BUILD_ID(TASK_GAPC, app_env.conidx), TASK_APP,
                                                   gapc_disconnect_cmd);

    cmd->operation = GAPC_DISCONNECT;
    cmd->reason    = CO_ERROR_REMOTE_USER_TERM_CON;

    // Send the message
    ((ke_msg_send_handler)SVC_ke_msg_send)(cmd);
}


uint8_t appm_get_dev_name(uint8_t* name)
{
    // copy name to provided pointer
    memcpy(name, app_env.dev_name, app_env.dev_name_len);
    // return name length
    return app_env.dev_name_len;
}


bool appm_get_connect_status(void)
{
	return (((ke_state_get_handler)SVC_ke_state_get)(TASK_APP) == APPM_CONNECTED);
}


void appm_register_cb_func(uint8_t id	,	app_callback_func_t func)
{
	if(id < APP_CB_MAX)
		app_env.func[id] = func;
}


void user_code_start(void)
{
#if (BLE_APP_PRESENT)
	appm_init();
#endif //BLE_APP_PRESENT
	
#if (!HOST_TO_OLD)
	app_reset_app();
#endif
}
#endif //(BLE_APP_PRESENT)

/// @} APP
