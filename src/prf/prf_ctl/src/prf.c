

/**
 ****************************************************************************************
 * @addtogroup PRF
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "panip_config.h"

#if (BLE_PROFILES)
#include "prf.h"
#include "att.h"
#include "stack_svc_api.h"
#if (PROJ_HID_KEYBOARD | PROJ_HID_SELFIE)
#include "diss.h"
#include "diss_task.h"
#include "bass.h"
#include "bass_task.h"
#include "hogpd.h"
#include "hogpd_task.h"
#endif

#if (USER_RGB_LIGHT)
#include "rgb_server1.h"
#include "rgb_server2.h"
#endif

#if (BLE_PROJ_TEMPLATE_SERVER)
#include "proj_template_server.h"
#endif

#if (BLE_TMP_CLIENT)
#include "tmp_client.h"
#endif 

#if (PROJ_OTA)
#include "ota_server.h"
#include "ota_server_task.h"
#endif


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */


/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * MACROS
 ****************************************************************************************
 */


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
struct prf_env_tag prf_env;

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Retrieve profile interface
 ****************************************************************************************
 */
static const struct prf_task_cbs * prf_itf_get(uint16_t task_id)
{
    const struct prf_task_cbs* prf_cbs = NULL;

    switch(KE_TYPE_GET(task_id))
    {
		#if BLE_PROJ_TEMPLATE_SERVER
		case TASK_ID_PROJ_TEMPLATE_SERVER:
			prf_cbs = proj_template_server_prf_itf_get();
		break;
		#endif
		
		#if defined(CFG_PRF_DISS)
		case TASK_ID_DISS:
        {
            prf_cbs = diss_prf_itf_get();
        }
        break;
        #endif
		#if defined(CFG_PRF_BASS)
        case TASK_ID_BASS:
        {
            prf_cbs = bass_prf_itf_get();
        }
        break;
		#endif
		#if defined(CFG_PRF_HOGPD)
        case TASK_ID_HOGPD:
        {
            prf_cbs = hogpd_prf_itf_get();
        }
        break;
		#endif
		#if (USER_RGB_LIGHT)
		case TASK_ID_RGB_SERVER1:
		{
			prf_cbs = rgb_server1_prf_itf_get();
		}
		break;
		case TASK_ID_RGB_SERVER2:
		{
			prf_cbs = rgb_server2_prf_itf_get();
		}
		break;
		#endif
		#if (BLE_TMP_CLIENT)
        case TASK_ID_TMP_CLIENT:
            prf_cbs = tmp_client_prf_itf_get();
        break;
		#endif // (BLE_BATT_SERVER)
		#if (PROJ_OTA)
        case TASK_ID_OTA:
			prf_cbs = ota_server_prf_itf_get();
		break;
        #endif
		
        default: 
		break;
    }

    return prf_cbs;
}

void prf_init(bool reset)
{
    uint8_t i;
    if(!reset)
    {
        // FW boot profile initialization
        for(i = 0; i < BLE_NB_PROFILES ; i++)
        {
            prf_env.prf[i].env  = NULL;
            prf_env.prf[i].task = TASK_GAPC + i +1;
            prf_env.prf[i].id   = TASK_ID_INVALID;

            // Initialize Task Descriptor
            //prf_env.prf[i].desc.msg_handler_tab = NULL;
            prf_env.prf[i].desc.state_handler = NULL;
			prf_env.prf[i].desc.default_handler = NULL;
			
            prf_env.prf[i].desc.state           = NULL;
			prf_env.prf[i].desc.state_max       = 0;
            prf_env.prf[i].desc.idx_max         = 0;
            prf_env.prf[i].desc.msg_cnt         = 0;

            ((ke_task_create_handler)SVC_ke_task_create)(prf_env.prf[i].task, &(prf_env.prf[i].desc));
        }
    }
    else
    {
        // FW boot profile destruction
        for(i = 0; i < BLE_NB_PROFILES ; i++)
        {
            // Get Profile API
            const struct prf_task_cbs * cbs = prf_itf_get(prf_env.prf[i].id);
            if(cbs != NULL)
            {
                // request to destroy profile
                cbs->destroy(&(prf_env.prf[i]));
            }
            // unregister profile
            prf_env.prf[i].id   = TASK_ID_INVALID;
           // prf_env.prf[i].desc.msg_handler_tab = NULL;
		   	prf_env.prf[i].desc.state_handler = NULL;
			prf_env.prf[i].desc.default_handler = NULL;
			
            prf_env.prf[i].desc.state           = NULL;
			prf_env.prf[i].desc.state_max       = 0;
            prf_env.prf[i].desc.idx_max         = 0;
            prf_env.prf[i].desc.msg_cnt         = 0;

            // Request kernel to flush task messages
            ((ke_task_msg_flush_handler)SVC_ke_task_msg_flush)(KE_TYPE_GET(prf_env.prf[i].task));
        }
    }
}


uint8_t prf_add_profile(struct gapm_profile_task_add_cmd * params, ke_task_id_t* prf_task)
{
    uint8_t i;
    uint8_t status = GAP_ERR_NO_ERROR;

    // retrieve profile callback
    const struct prf_task_cbs * cbs = prf_itf_get(params->prf_task_id);
    if(cbs == NULL)
    {
        // profile API not available
        status = GAP_ERR_INVALID_PARAM;
    }

    // check if profile not already present in task list
    if(status == GAP_ERR_NO_ERROR)
    {
        for(i = 0; i < BLE_NB_PROFILES ; i++)
        {
            if(prf_env.prf[i].id == params->prf_task_id)
            {
                status = GAP_ERR_NOT_SUPPORTED;
                break;
            }
        }
    }

    if(status == GAP_ERR_NO_ERROR)
    {
        // find first available task
        for(i = 0; i < BLE_NB_PROFILES ; i++)
        {
            // available task found
            if(prf_env.prf[i].id == TASK_ID_INVALID)
            {
                // initialize profile
                status = cbs->init(&(prf_env.prf[i]), &(params->start_hdl), params->app_task, params->sec_lvl, params->param);
				//printf("cdb done,st:%d, params->prf_task_id:%d,prf_env.prf[i].task:%d\n",status,params->prf_task_id,prf_env.prf[i].task);

                // initialization succeed
                if(status == GAP_ERR_NO_ERROR)
                {
                    // register profile
                    prf_env.prf[i].id = params->prf_task_id;
                    *prf_task = prf_env.prf[i].task;
                }
                break;
            }
        }

        if(i == BLE_NB_PROFILES)
        {
            status = GAP_ERR_INSUFF_RESOURCES;
        }
    }

    return (status);
}



void prf_create(uint8_t conidx)
{
    uint8_t i;
    /* simple connection creation handler, nothing to do. */

    // execute create function of each profiles
    for(i = 0; i < BLE_NB_PROFILES ; i++)
    {
        // Get Profile API
        const struct prf_task_cbs * cbs = prf_itf_get(prf_env.prf[i].id);
        if(cbs != NULL)
        {
            // call create callback
            cbs->create(&(prf_env.prf[i]), conidx);
        }
    }
}


void prf_cleanup(uint8_t conidx, uint8_t reason)
{
    uint8_t i;
    /* simple connection creation handler, nothing to do. */

    // execute create function of each profiles
    for(i = 0; i < BLE_NB_PROFILES ; i++)
    {
        // Get Profile API
        const struct prf_task_cbs * cbs = prf_itf_get(prf_env.prf[i].id);
        if(cbs != NULL)
        {
            // call cleanup callback
            cbs->cleanup(&(prf_env.prf[i]), conidx, reason);
        }
    }
}


prf_env_t* prf_env_get(uint16_t prf_id)
{
    prf_env_t* env = NULL;
    uint8_t i;
    // find if profile present in profile tasks
    for(i = 0; i < BLE_NB_PROFILES ; i++)
    {
        // check if profile identifier is known
        if(prf_env.prf[i].id == prf_id)
        {
            env = prf_env.prf[i].env;
            break;
        }
    }

    return env;
}

ke_task_id_t prf_src_task_get(prf_env_t* env, uint8_t conidx)
{
    ke_task_id_t task = PERM_GET(env->prf_task, PRF_TASK);

    if(PERM_GET(env->prf_task, PRF_MI))
    {
        task = KE_BUILD_ID(task, conidx);
    }

    return task;
}

ke_task_id_t prf_dst_task_get(prf_env_t* env, uint8_t conidx)
{
    ke_task_id_t task = PERM_GET(env->app_task, PRF_TASK);

    if(PERM_GET(env->app_task, PRF_MI))
    {
        task = KE_BUILD_ID(task, conidx);
    }

    return task;
}


ke_task_id_t prf_get_id_from_task(ke_msg_id_t task)
{
    ke_task_id_t id = TASK_ID_INVALID;
    uint8_t idx = KE_IDX_GET(task);
    uint8_t i;
    task = KE_TYPE_GET(task);

    // find if profile present in profile tasks
    for(i = 0; i < BLE_NB_PROFILES ; i++)
    {
        // check if profile identifier is known
        if(prf_env.prf[i].task == task)
        {
            id = prf_env.prf[i].id;
            break;
        }
    }

    return KE_BUILD_ID(id, idx);
}

ke_task_id_t prf_get_task_from_id(ke_msg_id_t id)
{
    ke_task_id_t task = TASK_NONE;
    uint8_t idx = KE_IDX_GET(id);
    uint8_t i;
    id = KE_TYPE_GET(id);

    // find if profile present in profile tasks
    for(i = 0; i < BLE_NB_PROFILES ; i++)
    {
        // check if profile identifier is known
        if(prf_env.prf[i].id == id)
        {
            task = prf_env.prf[i].task;
            break;
        }
    }

    return KE_BUILD_ID(task, idx);
}


#endif // (BLE_PROFILES)

/// @} PRF
