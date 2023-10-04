
/**
 ****************************************************************************************
 * @addtogroup PROJ_TEMPLATE_SERVER_TASK
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "panble_config.h"
#include "co_utils.h"
#include "prf_utils.h"
#include "proj_template_server_task.h"
#include "proj_template_server.h"
#include "ota_server_task.h"

#include "app.h"
#include "gattc.h"
#include "gapm.h"
#include "stack_svc_api.h"

__STATIC int proj_template_server_enable_req_handler(ke_msg_id_t const msgid,
								   struct proj_template_server_enable_req const *param,
								   ke_task_id_t const dest_id,
								   ke_task_id_t const src_id)
{
	int msg_status = KE_MSG_CONSUMED;
	uint8_t state = ((ke_state_get_handler)SVC_ke_state_get)(dest_id);
	int status = 0;

	// check state of the task
	if(state == PROJ_TEMPLATE_SVS_IDLE)
	{
		struct proj_template_server_env_tag* proj_template_server_env = PRF_ENV_GET(PROJ_TEMPLATE_SERVER, proj_template_server);

		// Check provided values
		if((param->conidx > BLE_CONNECTION_MAX)
			|| (((gapc_get_conhdl_handler)SVC_gapc_get_conhdl)(param->conidx) == GAP_INVALID_CONHDL))
		{
			status = (param->conidx > BLE_CONNECTION_MAX) ? GAP_ERR_INVALID_PARAM : PRF_ERR_REQ_DISALLOWED;
		}
		else
		{
			status = GAP_ERR_NO_ERROR;
		}
		struct proj_template_server_enable_rsp* rsp = KE_MSG_ALLOC(PROJ_TEMPLATE_SERVER_ENABLE_RSP, src_id,
		dest_id, proj_template_server_enable_rsp);
		rsp->conidx = param->conidx;
		rsp->status = status;
		((ke_msg_send_handler)SVC_ke_msg_send)(rsp);
	}

	return msg_status;
}

/**
 ****************************************************************************************
 * @brief Handles reception of the attribute info request message.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int gattc_att_info_req_ind_handler(ke_msg_id_t const msgid,
		struct gattc_att_info_req_ind *param,
		ke_task_id_t const dest_id,
		ke_task_id_t const src_id)
{
	struct gattc_att_info_cfm * cfm;
	uint8_t svc_idx = 0, att_idx = 0;
	// retrieve handle information
	uint8_t status = proj_template_server_get_att_idx(param->handle, &svc_idx, &att_idx);

	//Send write response
	cfm = KE_MSG_ALLOC(GATTC_ATT_INFO_CFM, src_id, dest_id, gattc_att_info_cfm);
	cfm->handle = param->handle;

		//ASSERT_ERR(0);

	if(status == GAP_ERR_NO_ERROR)
	{
	   cfm->length = 2;
	}

	cfm->status = status;
	((ke_msg_send_handler)SVC_ke_msg_send)(cfm);

	return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GATTC_WRITE_REQ_IND message.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
//peer write data 
__STATIC int gattc_write_req_ind_handler(ke_msg_id_t const msgid, 
										 struct gattc_write_req_ind const *param,
										 ke_task_id_t const dest_id, 
										 ke_task_id_t const src_id)
{
	struct gattc_write_cfm * cfm;
	uint8_t svc_idx = 0, att_idx = 0;
	uint8_t conidx = KE_IDX_GET(src_id);
	// retrieve handle information
	uint8_t status = proj_template_server_get_att_idx(param->handle, &svc_idx, &att_idx);
	
	// If the attribute has been found, status is GAP_ERR_NO_ERROR
	if (status == GAP_ERR_NO_ERROR)
	{ 
		struct proj_template_server_env_tag* env = PRF_ENV_GET(PROJ_TEMPLATE_SERVER, proj_template_server);

		switch (att_idx)
		{
			case PROJ_TEMPLATE_IDX_S2C_CLIENT_CFG:
			{
				env->client_cfg_s2c[conidx] = co_read16p(&param->value[0]);
			}
			break;

			// case PROJ_TEMPLATE_IDX_CTRL_CLIENT_CFG:
			// {
			// 	env->client_cfg_ctrl[conidx] = co_read16p(&param->value[0]);
			// }
			// break;
			
			// case PROJ_TEMPLATE_IDX_CTRL_VAL:
			// {
			// 	if ((0x0002 == param->length) 
			// 		&& (0x01 == param->value[0]) && (0x01 == param->value[1]))
			// 	{
			// 		// disconnect
			// 		appm_disconnect();
			// 	}
			// 	else if ((0x0004 == param->length) 
			// 		&& (0x02 == param->value[0]) && (0x02 == param->value[1]))
			// 	{
			// 		// set connection interval
			// 		uint16_t intv = co_read16p(&param->value[2]);
			// 		uint16_t tout = intv * 10;
					
			// 		if ( intv  > 0x0C80 )
			// 		{
			// 			intv = 0x0C80;
			// 		}
					
			// 		if (tout > 3200)
			// 		{
			// 			tout = 3200;
			// 		}

			// 		struct gapc_conn_param conn_param;
			// 		conn_param.intv_min = intv;
			// 		conn_param.intv_max = intv;
			// 		conn_param.latency	= 0;
			// 		conn_param.time_out = tout;
			// 		appm_update_param(&conn_param);
			// 	}
			// 	else if ((0x0002 == param->length) 
			// 		&& (0x03 == param->value[0]) && (0x03 == param->value[1]))
			// 	{
			// 		// read connection interval
			// 		struct gattc_send_evt_cmd *req = KE_MSG_ALLOC_DYN(GATTC_SEND_EVT_CMD,
			// 														  KE_BUILD_ID(TASK_GATTC, conidx), 
			// 														  env->prf_env.prf_task,
			// 														  gattc_send_evt_cmd, 
			// 														  4);
			// 		// Fill in the parameter structure
			// 		req->operation = GATTC_NOTIFY;
			// 		req->seq_num = 0x3;
			// 		req->handle  = proj_template_server_get_att_handle(0, att_idx);
			// 		req->length  = 4;
			// 		req->value[0] = 0x03;
			// 		req->value[1] = 0x03;
			// 		req->value[2] = 20;
			// 		req->value[3] = 0;

			// 		((ke_msg_send_handler)SVC_ke_msg_send)(req);
			// 	}
			// 	else if ((0x0004 == param->length) 
			// 		&& (0x04 == param->value[0]) && (0x04 == param->value[1]))
			// 	{
			// 		//gattc_exc_mtu_cmd(TASK_GATTC, conidx);
			// 		struct gattc_exc_mtu_cmd *cmd =  KE_MSG_ALLOC(GATTC_EXC_MTU_CMD,
			// 		KE_BUILD_ID(TASK_GATTC,conidx ), TASK_APP,
			// 		gattc_exc_mtu_cmd);
			
			// 		cmd->operation = GATTC_MTU_EXCH;
			// 		cmd->seq_num = 0x1;

			// 		((ke_msg_send_handler)SVC_ke_msg_send)(cmd);
			// 	}
			// 	else if ((0x0002 == param->length) 
			// 		&& (0x05 == param->value[0]) && (0x05 == param->value[1]))
			// 	{
			// 		// read att mtu
			// 		uint16_t mtu = gattc_get_mtu(conidx);
			// 		printf("read mtu %d\n", mtu);

			// 		struct gattc_send_evt_cmd *req = KE_MSG_ALLOC_DYN(GATTC_SEND_EVT_CMD,
			// 														  KE_BUILD_ID(TASK_GATTC, conidx), 
			// 														  env->prf_env.prf_task,
			// 														  gattc_send_evt_cmd, 
			// 														  4);
			// 		// Fill in the parameter structure
			// 		req->operation = GATTC_NOTIFY;
			// 		req->seq_num = 0x3;
			// 		req->handle  = proj_template_server_get_att_handle(0, att_idx);
			// 		req->length  = 4;
			// 		req->value[0] = 0x05;
			// 		req->value[1] = 0x05;
			// 		req->value[2] = mtu;
			// 		req->value[3] = mtu >> 8;

			// 		((ke_msg_send_handler)SVC_ke_msg_send)(req);
			// 	}
			// }
			// break;
			
			case PROJ_TEMPLATE_IDX_C2S_VAL:
			{
				// Inform APP of configuration change
				struct proj_template_server_peer_write_data_ind * ind = KE_MSG_ALLOC(PROJ_TEMPLATE_SERVER_PEER_WRITE_DATA_IND,
																					 prf_dst_task_get(&(env->prf_env), conidx), dest_id,
																					 proj_template_server_peer_write_data_ind);
				ind->conidx = conidx;
				if( proj_template_server_get_att_idx(param->handle,&svc_idx,&(ind->att_idx)) != GAP_ERR_NO_ERROR)
				{
					ASSERT_ERR(0);
				}
				
				ind->packet_size = param->length;
				memcpy(ind->packet, (param->value + param->offset),param->length);

				((ke_msg_send_handler)SVC_ke_msg_send)(ind);
			}
			break;
			
			default:
			{
				status = ATT_ERR_APP_ERROR;
			}
			break;
		}
	}

	//Send write response
	cfm = KE_MSG_ALLOC(GATTC_WRITE_CFM, src_id, dest_id, gattc_write_cfm);
	cfm->handle = param->handle;
	cfm->status = status;
	((ke_msg_send_handler)SVC_ke_msg_send)(cfm);

	return (KE_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GATTC_READ_REQ_IND message.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int gattc_read_req_ind_handler(ke_msg_id_t const msgid, struct gattc_read_req_ind const *param,
									  ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
	struct gattc_read_cfm * cfm;
	uint8_t svc_idx = 0, att_idx = 0;
	uint8_t conidx = KE_IDX_GET(src_id);
	// retrieve handle information
	uint8_t status = proj_template_server_get_att_idx(param->handle, &svc_idx, &att_idx);
	uint16_t length = 0;
	struct proj_template_server_env_tag* env = PRF_ENV_GET(PROJ_TEMPLATE_SERVER, proj_template_server);
	
	// If the attribute has been found, status is GAP_ERR_NO_ERROR
	if (status == GAP_ERR_NO_ERROR)
	{
		switch (att_idx)
		{
			case PROJ_TEMPLATE_IDX_S2C_USER_DESC:
			{
				length = PROJ_TEMPLATE_S2C_USER_DESC_VAL_LEN;
			}
			break;

			case PROJ_TEMPLATE_IDX_S2C_CLIENT_CFG:
			{
				length = PROJ_TEMPLATE_S2C_CLIENT_CFG_LEN;
			}
			break;

			case PROJ_TEMPLATE_IDX_C2S_USER_DESC:
			{
				length = PROJ_TEMPLATE_C2S_USER_DESC_VAL_LEN;
			}
			break;

			// case PROJ_TEMPLATE_IDX_CTRL_USER_DESC:
			// {
			// 	length = PROJ_TEMPLATE_CTRL_USER_DESC_VAL_LEN;
			// }
			// break;
			// case PROJ_TEMPLATE_IDX_CTRL_CLIENT_CFG:
			// {
			// 	length = PROJ_TEMPLATE_CTRL_CLIENT_CFG_LEN;
			// }
			default:
			{
				status = PRF_APP_ERROR;
			}
		}
	}

	//Send read response
	cfm = KE_MSG_ALLOC_DYN(GATTC_READ_CFM, src_id, dest_id, gattc_read_cfm, length);
	cfm->handle = param->handle;
	cfm->status = status;
	cfm->length = length;
	
	// If the attribute has been found, status is GAP_ERR_NO_ERROR
	if (status == GAP_ERR_NO_ERROR)
	{
		switch (att_idx)
		{
			case PROJ_TEMPLATE_IDX_S2C_USER_DESC:
			{
				memcpy(cfm->value, PROJ_TEMPLATE_S2C_USER_DESC_VAL, PROJ_TEMPLATE_S2C_USER_DESC_VAL_LEN);
			}
			break;

			case PROJ_TEMPLATE_IDX_S2C_CLIENT_CFG:
			{
				memcpy(cfm->value, &env->client_cfg_s2c[conidx], PROJ_TEMPLATE_S2C_CLIENT_CFG_LEN);
			}
			break;

			case PROJ_TEMPLATE_IDX_C2S_USER_DESC:
			{
				memcpy(cfm->value, PROJ_TEMPLATE_C2S_USER_DESC_VAL, PROJ_TEMPLATE_C2S_USER_DESC_VAL_LEN);
			}
			break;

			// case PROJ_TEMPLATE_IDX_CTRL_USER_DESC:
			// {
			// 	memcpy(cfm->value, PROJ_TEMPLATE_CTRL_USER_DESC_VAL, PROJ_TEMPLATE_CTRL_USER_DESC_VAL_LEN);
			// }
			// break;

			// case PROJ_TEMPLATE_IDX_CTRL_CLIENT_CFG:
			// {
			// 	memcpy(cfm->value, &env->client_cfg_ctrl[conidx], PROJ_TEMPLATE_CTRL_CLIENT_CFG_LEN);
			// }
			// break;
			
			default:
			{
				cfm->status = ATT_ERR_APP_ERROR;
			}
			break;
		}
	}
	
	((ke_msg_send_handler)SVC_ke_msg_send)(cfm);

	return (KE_MSG_CONSUMED);
}

__STATIC int proj_template_server_write_cmd_handler(ke_msg_id_t const msgid,
								   struct proj_template_server_write_cmd const *param,
								   ke_task_id_t const dest_id,
								   ke_task_id_t const src_id)
 
{ 
	struct proj_template_server_env_tag* proj_template_server_env = PRF_ENV_GET(PROJ_TEMPLATE_SERVER, proj_template_server);
	struct gattc_send_evt_cmd *req = KE_MSG_ALLOC_DYN(GATTC_SEND_EVT_CMD,
													  KE_BUILD_ID(TASK_GATTC,param->conidx), 
													  proj_template_server_env->prf_env.prf_task,
													  gattc_send_evt_cmd, 
													  param->packet_size);
	
	// Fill in the parameter structure
	if(param->write_type == NOTIFICATION)
		req->operation = GATTC_NOTIFY;
	else if(param->write_type == INDICATION)
		req->operation = GATTC_INDICATE;
	
	req->seq_num = 0x3;
	req->handle  = proj_template_server_get_att_handle(0, param->att_idx);
	req->length  = param->packet_size;
	
	memcpy(req->value, param->packet, req->length);

	((ke_msg_send_handler)SVC_ke_msg_send)(req);

	return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles @ref GATTC_CMP_EVT for GATTC_NOTIFY message meaning that Measurement
 * notification has been correctly sent to peer device (but not confirmed by peer device).
 * *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC int gattc_cmp_evt_handler(ke_msg_id_t const msgid,  struct gattc_cmp_evt const *param,
								 ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
	if(param->operation == GATTC_NOTIFY)
	{
		
	}
	return (KE_MSG_CONSUMED);
}

/// Default State handlers definition
KE_MSG_HANDLER_TAB(proj_template_server)
{
	{PROJ_TEMPLATE_SERVER_ENABLE_REQ,       (ke_msg_func_t) proj_template_server_enable_req_handler},
	{GATTC_ATT_INFO_REQ_IND,                (ke_msg_func_t) gattc_att_info_req_ind_handler},
	{GATTC_WRITE_REQ_IND,                   (ke_msg_func_t) gattc_write_req_ind_handler},
	{GATTC_READ_REQ_IND,                    (ke_msg_func_t) gattc_read_req_ind_handler},
	{PROJ_TEMPLATE_SERVER_WRITE_CMD,        (ke_msg_func_t) proj_template_server_write_cmd_handler},
	{GATTC_CMP_EVT,                         (ke_msg_func_t) gattc_cmp_evt_handler},
};

const struct ke_state_handler task_proj_template_server_msg_default_handler = KE_STATE_HANDLER(proj_template_server_msg_handler_tab);


void proj_template_server_task_init(struct ke_task_desc *task_desc)
{
	// Get the address of the environment
	struct proj_template_server_env_tag* proj_template_server_env = PRF_ENV_GET(PROJ_TEMPLATE_SERVER, proj_template_server);

	task_desc->state_handler   = NULL;
	task_desc->default_handler = &task_proj_template_server_msg_default_handler;
	task_desc->msg_cnt         = ARRAY_LEN(proj_template_server_msg_handler_tab);
	task_desc->state           = proj_template_server_env->state;
	task_desc->idx_max         = PROJ_TEMPLATE_SERVER_IDX_MAX;
}
