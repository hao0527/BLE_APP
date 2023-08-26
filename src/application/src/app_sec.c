
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

#include "panip_config.h"

#if (BLE_APP_SEC)

#include <string.h>
#include "co_math.h"
#include "gapc_task.h"      // GAP Controller Task API Definition
#include "gap.h"            // GAP Definition
#include "gapc.h"           // GAPC Definition
#include "prf_types.h"

#include "app.h"            // Application API Definition
#include "app_sec.h"        // Application Security API Definition
#include "app_task.h"       // Application Manager API Definition
#include "gapc_int.h"
#include "app_proj_event.h"
#include "stack_svc_api.h"

#if (PROJ_HID_KEYBOARD)
#include "proj_hid_keyboard.h"
#endif // (PROJ_HID_KEYBOARD)

#if (PROJ_HID_SELFIE)
#include "proj_hid_selfie.h"
#endif // (PROJ_HID_SELFIE)

#if (USER_PROJ_TEMPLATE)
#include "proj_template.h"
#endif // (PROJ_TEMPLATE)

#define DBG_APP_SEC(x)              //printf x
#define DBG_APP_SEC_FUNC()          //printf("%s\n", __func__)
#define DBG_APP_SEC_HEX_GROUP(x)    //show_reg3 x
/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// Application Security Environment Structure
struct app_sec_env_tag app_sec_env;

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void app_sec_init()
{
    DBG_APP_SEC_FUNC();
    
#if (PROJ_HID_KEYBOARD)
    app_sec_env.bonded = *(bool *)hid_kbd_get_store_info(HID_KBD_STORE_TYPE_BONDED);
#endif // (PROJ_HID_KEYBOARD)

#if (PROJ_HID_SELFIE)
    app_sec_env.bonded = *(bool *)hid_selfie_get_store_info(HIDS_SELFIE_STORE_TYPE_BONDED);
#endif // (PROJ_HID_SELFIE)
	
#if (USER_PROJ_TEMPLATE)
    app_sec_env.bonded = *(bool *)proj_template_get_store_info(PROJ_TEMPLATE_STORE_TYPE_BONDED);
#endif // (PROJ_TEMPLATE)
}

bool app_sec_get_bond_status(void)
{
    return app_sec_env.bonded;
}


void app_sec_remove_bond(void)
{
    // Check if we are well bonded
    if (app_sec_env.bonded == true)
    {
        // Update the environment variable
        app_sec_env.bonded = false;

#if (PROJ_HID_KEYBOARD)
        hid_kbd_clr_store_info();
#endif // (PROJ_HID_KEYBOARD)

#if (PROJ_HID_SELFIE)
        hid_selfie_clr_store_info();
#endif // (PROJ_HID_SELFIE)
		
#if (USER_PROJ_TEMPLATE)
        proj_template_clr_store_info();
#endif // (USER_PROJ_TEMPLATE)
    }
}


//For slave role only
void app_sec_send_security_req(uint8_t conidx)
{
    // Send security request
    struct gapc_security_cmd *cmd = KE_MSG_ALLOC(GAPC_SECURITY_CMD,
                                                 KE_BUILD_ID(TASK_GAPC, conidx), TASK_APP,
                                                 gapc_security_cmd);

    cmd->operation = GAPC_SECURITY_REQ;
    cmd->auth      = GAP_AUTH_REQ_MITM_BOND;

    // Send the message
    ((ke_msg_send_handler)SVC_ke_msg_send)(cmd);
}


//For master role only
void app_sec_send_enc_req(uint8_t conidx,struct gapc_ltk *ltk)
{
	struct gapc_encrypt_cmd *cmd = KE_MSG_ALLOC(GAPC_ENCRYPT_CMD,
			KE_BUILD_ID(TASK_GAPC, conidx), TASK_APP, gapc_encrypt_cmd);
	cmd->operation = GAPC_ENCRYPT;
	memcpy(&cmd->ltk, ltk, sizeof(struct gapc_ltk));
	((ke_msg_send_handler)SVC_ke_msg_send)(cmd);
}

/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */

static int gapc_bond_req_ind_handler(ke_msg_id_t const msgid,
                                     struct gapc_bond_req_ind const *param,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id)
{
    // Prepare the GAPC_BOND_CFM message
    struct gapc_bond_cfm *cfm = KE_MSG_ALLOC(GAPC_BOND_CFM,
                                             src_id, TASK_APP,
                                             gapc_bond_cfm);
	
    DBG_APP_SEC(("bond_request_ind: %d\n", param->request));

    switch (param->request)
    {
        case (GAPC_PAIRING_REQ):
        {
            cfm->request = GAPC_PAIRING_RSP;
            {
                cfm->accept  = true;
                //feat auth      			
				cfm->data.pairing_feat.auth      = GAP_AUTH_REQ_MITM_BOND;

                //io capabilities
                cfm->data.pairing_feat.iocap     = GAP_IO_CAP_NO_INPUT_NO_OUTPUT;
                //max enc key size
                cfm->data.pairing_feat.key_size  = KEY_LEN;	//16;
                //oob data flag     
                cfm->data.pairing_feat.oob       = GAP_OOB_AUTH_DATA_NOT_PRESENT;
				
                cfm->data.pairing_feat.sec_req   = GAP_NO_SEC;
				
                //initiator key distribution
				cfm->data.pairing_feat.ikey_dist = (GAP_KDIST_ENCKEY | GAP_KDIST_IDKEY |GAP_KDIST_SIGNKEY);
                //responder key distribution
				cfm->data.pairing_feat.rkey_dist = (GAP_KDIST_ENCKEY | GAP_KDIST_IDKEY |GAP_KDIST_SIGNKEY);

            }
        } break;

        case (GAPC_LTK_EXCH):
        {
            // Counter
            uint8_t counter;

            cfm->accept  = true;
            cfm->request = GAPC_LTK_EXCH;

            // Generate all the values
            cfm->data.ltk.ediv = (uint16_t)co_rand_word();

            for (counter = 0; counter < RAND_NB_LEN; counter++)
            {
                cfm->data.ltk.ltk.key[counter]    = (uint8_t)co_rand_word();
                cfm->data.ltk.randnb.nb[counter] = (uint8_t)co_rand_word();
            }

            for (counter = RAND_NB_LEN; counter < KEY_LEN; counter++)
            {
                cfm->data.ltk.ltk.key[counter]    = (uint8_t)co_rand_word();
            }
			
			cfm->data.ltk.key_size = 16;
			
            //for slave role
			if(((gapc_get_role)SVC_gapc_get_role)(app_env.conidx) == 1)
			{
                DBG_APP_SEC(("store ltk for slave role\n"));
				//no use. 
				DBG_APP_SEC(("store slave peer ltk:\n"));
				DBG_APP_SEC_HEX_GROUP((cfm->data.ltk.ltk.key, 16));
				DBG_APP_SEC(("peer slave ediv:\n"));
				DBG_APP_SEC_HEX_GROUP(((uint8_t *)&cfm->data.ltk.ediv, 2));
				DBG_APP_SEC(("peer slave rand:\n"));
				DBG_APP_SEC_HEX_GROUP((cfm->data.ltk.randnb.nb, 8));
#if (PROJ_HID_KEYBOARD)
                #if (PAIR_FILTER_EN)
                if(m_hid_kbd_env.info.bonded == true)
                {
                    if((memcmp(m_hid_kbd_env.info.stored_peer_addr.addr.addr, cfm->data.irk.addr.addr.addr, 6) != 0))
                    {
                        appm_disconnect();
                        return (KE_MSG_CONSUMED);
                    }
                }
                #endif
                hid_kbd_store_info(&cfm->data.ltk, HID_KBD_STORE_TYPE_LTK);
#endif // (PROJ_HID_KEYBOARD)

#if (PROJ_HID_SELFIE)
                hid_selfie_store_info(&cfm->data.ltk, HIDS_SELFIE_STORE_TYPE_LTK);
#endif // (PROJ_HID_SELFIE)

#if (USER_PROJ_TEMPLATE)
                proj_template_store_info(&cfm->data.ltk, PROJ_TEMPLATE_STORE_TYPE_LTK);
#endif

			}

        } break;


        case (GAPC_IRK_EXCH):
        {
            cfm->accept  = true;
            cfm->request = GAPC_IRK_EXCH;

            // Load IRK
            memcpy(cfm->data.irk.irk.key, app_env.loc_irk, KEY_LEN);
            // load device address
            cfm->data.irk.addr.addr_type = ADDR_PUBLIC;

        } break;
		case (GAPC_CSRK_EXCH):
        {
            cfm->accept  = true;
            cfm->request = GAPC_CSRK_EXCH;

			uint8_t counter;
			for (counter = 0; counter < GAP_KEY_LEN; counter++)
			{
				cfm->data.csrk.key[counter]	= (uint8_t)co_rand_word();
			}

        } break;
		
        case (GAPC_TK_EXCH):
        {
            // Generate a PIN Code- (Between 100000 and 999999)
            //uint32_t pin_code = (100000 + (co_rand_word()%900000));
			uint32_t pin_code = 123456;
			
            cfm->accept  = true;
            cfm->request = GAPC_TK_EXCH;

            // Set the TK value
            memset(cfm->data.tk.key, 0, KEY_LEN);

            cfm->data.tk.key[0] = (uint8_t)((pin_code & 0x000000FF) >>  0);
            cfm->data.tk.key[1] = (uint8_t)((pin_code & 0x0000FF00) >>  8);
            cfm->data.tk.key[2] = (uint8_t)((pin_code & 0x00FF0000) >> 16);
            cfm->data.tk.key[3] = (uint8_t)((pin_code & 0xFF000000) >> 24);
        } break;

        default:
        {
        } break;
    }

    // Send the message
    ((ke_msg_send_handler)SVC_ke_msg_send)(cfm);

    return (KE_MSG_CONSUMED);
}

static int gapc_bond_ind_handler(ke_msg_id_t const msgid,
                                 struct gapc_bond_ind const *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id)
{
    DBG_APP_SEC(("bond_ind: %d\n", param->info));

    switch (param->info)
    {
        case (GAPC_PAIRING_SUCCEED):
        {
            // Update the bonding status in the environment
            app_sec_env.bonded = true;
			
            DBG_APP_SEC(("paring finished, store bonded & peer addr\n"));
            
#if (PROJ_HID_KEYBOARD)
			hid_kbd_store_info((void *)((gapc_get_bdaddr)SVC_gapc_get_bdaddr)(0, SMPC_INFO_PEER), HID_KBD_STORE_TYPE_PEER_BD_ADDRESS);
			hid_kbd_store_info((void *)app_sec_env.bonded, HID_KBD_STORE_TYPE_BONDED);
			hid_kbd_store_flash_data();
#endif // (PROJ_HID_KEYBOARD)

#if (PROJ_HID_SELFIE)
			hid_selfie_store_info((void *)((gapc_get_bdaddr)SVC_gapc_get_bdaddr)(0, SMPC_INFO_PEER), HIDS_SELFIE_STORE_TYPE_PEER_BD_ADDRESS);
			hid_selfie_store_info((void *)app_sec_env.bonded, HIDS_SELFIE_STORE_TYPE_BONDED);
			hid_selfie_store_flash_data();
#endif // (PROJ_HID_SELFIE)
            
#if (USER_PROJ_TEMPLATE)
			proj_template_store_info((void *)((gapc_get_bdaddr)SVC_gapc_get_bdaddr)(0, SMPC_INFO_PEER), PROJ_TEMPLATE_STORE_TYPE_PEER_BD_ADDRESS);
			proj_template_store_info((void *)app_sec_env.bonded, PROJ_TEMPLATE_STORE_TYPE_BONDED);
			proj_template_store_flash_data();
#endif // (USER_PROJ_TEMPLATE)
			
            //DYC added
			uint8_t role = ((gapc_get_role)SVC_gapc_get_role)(app_env.conidx);

			if(role == 1)	//slave then inform
				app_connection_encrypted_ind_func(app_env.conidx);
			else if(role == 0)	//master then inform
				app_connection_encrypt_completed_ind_func(app_env.conidx);

        } break;

        case (GAPC_REPEATED_ATTEMPT):
        {
            appm_disconnect();
        } break;

		case (GAPC_LTK_EXCH):
		{
			//Record slave's LTK info	for master role
			if(((gapc_get_role)SVC_gapc_get_role)(app_env.conidx) == 0)
			{
                DBG_APP_SEC(("store ltk for master role\n"));
#if (PROJ_HID_KEYBOARD)
            hid_kbd_store_info((void *)&param->data.ltk, HID_KBD_STORE_TYPE_LTK);
#endif // (PROJ_HID_KEYBOARD)

#if (PROJ_HID_SELFIE)
            hid_selfie_store_info((void *)&param->data.ltk, HIDS_SELFIE_STORE_TYPE_LTK);
#endif // (PROJ_HID_SELFIE)
				
#if (USER_PROJ_TEMPLATE)
            proj_template_store_info((void *)&param->data.ltk, PROJ_TEMPLATE_STORE_TYPE_LTK);
#endif // (USER_PROJ_TEMPLATE)
                
			}
		
		} break;

        case (GAPC_IRK_EXCH):
        {
            DBG_APP_SEC(("recvd slave peer irk:\n"));
            DBG_APP_SEC_HEX_GROUP((param->data.irk.irk.key, 16));
            DBG_APP_SEC(("peer slave addr:\n"));
            DBG_APP_SEC_HEX_GROUP((param->data.irk.addr.addr.addr, 6));
        } break;
		case (GAPC_CSRK_EXCH):
		{
            //no use. 
            DBG_APP_SEC(("recvd slave peer csrk:\n"));
            //show_reg3(param->data.csrk.key,16);

		} break;

        case (GAPC_PAIRING_FAILED):
        {
            app_sec_send_security_req(0);
        } break;

        default:
        {
            DBG_APP_SEC(("gapc_bond_ind_handler : %d\n", param->info));
        } break;
    }

    return (KE_MSG_CONSUMED);
}

static int gapc_encrypt_req_ind_handler(ke_msg_id_t const msgid,
                                        struct gapc_encrypt_req_ind const *param,
                                        ke_task_id_t const dest_id,
                                        ke_task_id_t const src_id)
{
    DBG_APP_SEC_FUNC();

    // Prepare the GAPC_ENCRYPT_CFM message
    struct gapc_encrypt_cfm *cfm = KE_MSG_ALLOC(GAPC_ENCRYPT_CFM,
                                                src_id, TASK_APP,
                                                gapc_encrypt_cfm);

    cfm->found    = false;

    if (app_sec_env.bonded == true)
    {
        
#if (PROJ_HID_KEYBOARD)
        struct gapc_ltk *ltk = (struct gapc_ltk *)hid_kbd_get_store_info(HID_KBD_STORE_TYPE_LTK);
#elif (PROJ_HID_SELFIE)
        struct gapc_ltk *ltk = (struct gapc_ltk *)hid_selfie_get_store_info(HIDS_SELFIE_STORE_TYPE_LTK);
#elif (USER_PROJ_TEMPLATE)
		struct gapc_ltk *ltk = (struct gapc_ltk *)proj_template_get_store_info(PROJ_TEMPLATE_STORE_TYPE_LTK);
#else
		struct gapc_ltk *ltk = NULL;
#endif
        
        if ((param->ediv == ltk->ediv) &&
			!memcmp(&param->rand_nb.nb[0], &ltk->randnb.nb[0], sizeof(struct rand_nb)))
		{
			cfm->found	  = true;
			cfm->key_size = 16;
			memcpy(&cfm->ltk, &ltk->ltk, sizeof(struct gap_sec_key));
		}
        
        DBG_APP_SEC(("STORE_TYPE_LTK:\n"));
        DBG_APP_SEC_HEX_GROUP((cfm->ltk.key, sizeof(struct gap_sec_key)));
        
        DBG_APP_SEC(("show local ediv:\n"));
        DBG_APP_SEC_HEX_GROUP(((uint8_t *)&(param->ediv), 2));
        DBG_APP_SEC(("show local rand:\n"));
        DBG_APP_SEC_HEX_GROUP((param->rand_nb.nb, 8));

        DBG_APP_SEC(("show peer ediv:\n"));
        DBG_APP_SEC_HEX_GROUP(((uint8_t *)&param->ediv, 2));
        DBG_APP_SEC(("show peer rand:\n"));
        DBG_APP_SEC_HEX_GROUP((param->rand_nb.nb, 8));
    }
    /*
     * else the peer device is not known, an error should trigger a new pairing procedure.
     */

    // Send the message
    ((ke_msg_send_handler)SVC_ke_msg_send)(cfm);

    return (KE_MSG_CONSUMED);
}


static int gapc_encrypt_ind_handler(ke_msg_id_t const msgid,
                                    struct gapc_encrypt_ind const *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    // encryption/ re-encryption succeeded
    DBG_APP_SEC(("encryption succeeded\n"));

	if (app_sec_get_bond_status())
	{
        uint8_t role = ((gapc_get_role)SVC_gapc_get_role)(param->conidx);
		if(role == 1)   //slave then inform
			app_connection_encrypted_ind_func(param->conidx);
		else if(role == 0)	//master then inform
			app_connection_encrypt_completed_ind_func(param->conidx);
	}

    return (KE_MSG_CONSUMED);
}

static int app_sec_msg_dflt_handler(ke_msg_id_t const msgid,
                                    void *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    // Drop the message

    return (KE_MSG_CONSUMED);
}

 /*
  * LOCAL VARIABLE DEFINITIONS
  ****************************************************************************************
  */

/// Default State handlers definition
const struct ke_msg_handler app_sec_msg_handler_list[] =
{
    // Note: first message is latest message checked by kernel so default is put on top.
    {KE_MSG_DEFAULT_HANDLER,  (ke_msg_func_t)app_sec_msg_dflt_handler},

    {GAPC_BOND_REQ_IND,       (ke_msg_func_t)gapc_bond_req_ind_handler},
    {GAPC_BOND_IND,           (ke_msg_func_t)gapc_bond_ind_handler},

    {GAPC_ENCRYPT_REQ_IND,    (ke_msg_func_t)gapc_encrypt_req_ind_handler},
    {GAPC_ENCRYPT_IND,        (ke_msg_func_t)gapc_encrypt_ind_handler},
};

const struct ke_state_handler app_sec_table_handler =
    {&app_sec_msg_handler_list[0], (sizeof(app_sec_msg_handler_list)/sizeof(struct ke_msg_handler))};

#endif //(BLE_APP_SEC)

/// @} APP
