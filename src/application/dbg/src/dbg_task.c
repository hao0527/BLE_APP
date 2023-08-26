
/**
 ****************************************************************************************
 * @addtogroup DBGTASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"         // stack configuration


#if (SDK_USELESS_CODE)


#include <string.h>               // for mem* functions
#include "co_version.h"
#include "co_endian.h"
#include "co_error.h"
#include "arch.h"                 // arch definition
#include "dbg_task.h"             // debug task definition
#include "dbg.h"                  // debug block definition
#include "rwip.h"                 // RF API definitions
#include "ke.h"                   // kernel definitions
#include "ke_mem.h"               // kernel memory management

#if (RW_DEBUG_FLASH)
#include "flash.h"                // flash functions and macros
#endif //RW_DEBUG_FLASH

#if (RW_DEBUG_NVDS)
#if (NVDS_SUPPORT)
#include "nvds.h"         // NVDS definitions
#endif // NVDS_SUPPORT
#endif //RW_DEBUG_NVDS

#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
//#include "reg_access.h"           // access registers
#endif // BLE_EMB_PRESENT || BT_EMB_PRESENT



#if (BLE_EMB_PRESENT)
//#include "reg_blecore.h"
//#include "reg_ble_em_cs.h"
//#include "lld.h"
//#include "lld_evt.h"
//#include "lld_util.h"
//#include "llm_util.h"
//#include "llc_util.h"
//#include "llc.h"
#if (RW_WLAN_COEX)
#include "lld_wlcoex.h"
#endif // RW_WLAN_COEX
#endif //BLE_EMB_PRESENT

#if (DSM_SUPPORT)
#include "dsm.h"            // data storage manager definitions
#endif //DSM_SUPPORT


/*
 * DEFINES
 ****************************************************************************************
 */

/// 8 bit access types
#define _8_Bit                              8
/// 16 bit access types
#define _16_Bit                             16
/// 32 bit access types
#define _32_Bit                             32

/// PLATFORM RESET REASON: Reset and load FW from flash
#define PLATFORM_RESET_TO_FW    (0)
/// PLATFORM RESET REASON: Reset and stay in ROM code
#define PLATFORM_RESET_TO_ROM   (1)


#if (DSM_SUPPORT)
/// Data Storage Operation
#define DS_OP_NOP              0
#define DS_OP_INIT             1
#define DS_OP_CLEAR            2
#define DS_OP_START_WRITE      3
#define DS_OP_PUSH_SAMPLE      4
#define DS_OP_STOP_WRITE       5
#define DS_OP_START_READ       6
#define DS_OP_POP_SAMPLE       7
#define DS_OP_STOP_READ        8
#define DS_OP_DELETE_BLOCK     9
#endif //DSM_SUPPORT


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

#if (DSM_SUPPORT)
/// Data Storage Current Operation
static uint8_t dbg_data_storage_op_curr;
#endif //DSM_SUPPORT


/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

#if (DSM_SUPPORT)
/**
 ****************************************************************************************
 * @brief   Data Storage operation callback
 *
 * param[in]   status    Operation status
 ****************************************************************************************
 */
static void data_storage_op_callback(uint8_t status)
{
    struct dbg_data_storage_ind_evt *evt = (struct dbg_data_storage_ind_evt *)
     KE_MSG_ALLOC(DBG_DATA_STORAGE_IND_EVT, TASK_HCI, TASK_DBG, dbg_data_storage_ind_evt);

    // Fill in the event structure
    evt->status = status;
    evt->operation = dbg_data_storage_op_curr;
    evt->length = 0;

    // Send event
    ke_msg_send(evt);
}

/**
 ****************************************************************************************
 * @brief   Data Storage operation callback
 *
 * param[in]   status    Operation status
 ****************************************************************************************
 */
static void data_storage_op_callback_block(uint8_t status, uint16_t block_id)
{
    struct dbg_data_storage_ind_evt *evt = (struct dbg_data_storage_ind_evt *)
     KE_MSG_ALLOC(DBG_DATA_STORAGE_IND_EVT, TASK_HCI, TASK_DBG, dbg_data_storage_ind_evt);

    // Fill in the event structure
    evt->status = status;
    evt->operation = dbg_data_storage_op_curr;
    evt->length = 2;
    co_write16p(&evt->sample[0], co_htobs(block_id));

    // Send event
    ke_msg_send(evt);
}

/**
 ****************************************************************************************
 * @brief   Data Storage operation callback
 *
 * param[in]   status    Operation status
 ****************************************************************************************
 */
static void data_storage_op_callback_sample(uint8_t status, uint8_t length, uint8_t *data)
{
    struct dbg_data_storage_ind_evt *evt = (struct dbg_data_storage_ind_evt *)
     KE_MSG_ALLOC(DBG_DATA_STORAGE_IND_EVT, TASK_HCI, TASK_DBG, dbg_data_storage_ind_evt);

    // Fill in the event structure
    evt->status = status;
    evt->operation = dbg_data_storage_op_curr;
    evt->length = length;
    memcpy(&evt->sample[0], data, length);

    // Send event
    ke_msg_send(evt);
}
#endif //DSM_SUPPORT


/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @brief Handle the reception of the vendor specific command read from memory.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int
dbg_rd_mem_req_handler(ke_msg_id_t const msgid, struct dbg_rd_mem_cmd const *param,
                       ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    #if (RW_DEBUG_MEM)
    uint8_t length = 0;
    uint32_t init_addr = 0;
    uint32_t value = 0;
    uint32_t i = 0;
    #endif //RW_DEBUG_MEM

    // structure type for the complete command event
    struct dbg_rd_mem_cmp_evt *evt = KE_MSG_ALLOC(DBG_RD_MEM_CMP_EVT,
            TASK_HCI, TASK_DBG, dbg_rd_mem_cmp_evt);

    #if (RW_DEBUG_MEM)
    init_addr = param->start_addr;
    length = param->length;


    /* Check that data length is not null or too big before reading */
    if ((length == 0)||
        (length > (sizeof(struct buffer_tag)- sizeof(uint8_t))))
    {
        evt->status = CO_ERROR_INVALID_HCI_PARAM;
    }
    else
    {
        /* Check type of data to be read */
        if (param->type == _8_Bit)
        {
            /* Read bytes */
            for (i = 0; i < length; i++)
            {
                /* Read value at @ set in Param1+i */
                evt->buf.data[i] = *(volatile uint8_t *)(init_addr+i);
            }
        }
        else if (param->type == _16_Bit)
        {
            for (i = 0; i < length; i += 2)
            {
                /* Read value at @ set in Param1+i */
                value = (*(volatile uint16_t *)(init_addr+i));

                /* store in the buffer */
                evt->buf.data[i]   = (uint8_t) value;
                value >>= 8;
                evt->buf.data[i+1] = (uint8_t) value;
             }
        }
        else if (param->type == _32_Bit)
        {
            /* Read 32 bit word */
            for (i = 0; i < length; i += 4)
            {
                value = (*(volatile uint32_t *)(init_addr+i));

                /* store in the buffer */
                evt->buf.data[i]   = (uint8_t) value;
                value >>= 8;
                evt->buf.data[i+1] = (uint8_t) value;
                value >>= 8;
                evt->buf.data[i+2] = (uint8_t) value;
                value >>= 8;
                evt->buf.data[i+3] = (uint8_t) value;
            }
        }
        evt->buf.length = length;
        evt->status = CO_ERROR_NO_ERROR;
    }
    #else //RW_DEBUG_MEM
    evt->status = CO_ERROR_UNSUPPORTED;
    #endif //RW_DEBUG_MEM

    ke_msg_send(evt);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handle the reception of the vendor specific command write to memory.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int
dbg_wr_mem_req_handler(ke_msg_id_t const msgid, struct dbg_wr_mem_cmd const *param,
                       ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    #if (RW_DEBUG_MEM)
    uint32_t value = 0;
    uint32_t length = 0;
    uint32_t init_addr = 0;
    uint8_t *data_buf;
    uint32_t i = 0;
    #endif //RW_DEBUG_MEM

    // structure type for the complete command event
    struct dbg_common_cmd_cmp_evt *evt = KE_MSG_ALLOC(DBG_WR_MEM_CMP_EVT,
            TASK_HCI, TASK_DBG, dbg_common_cmd_cmp_evt);

    #if (RW_DEBUG_MEM)
    length    = param->buf.length;
    data_buf  = (uint8_t*)&param->buf.data[0];
    init_addr = param->start_addr;

    /* Check that data length is not null or too big before reading */
    if ((length == 0)||
        (length > (sizeof(struct buffer_tag)- sizeof(uint8_t))))
    {
        evt->status = CO_ERROR_INVALID_HCI_PARAM;
    }
    else
    {
        /* Check type of data to be written */
        if (param->type == _8_Bit)
        {
            /* Write bytes */
            for (i = 0; i < length; i++)
            {
                /* Set value type at @ Param1 */
                *(volatile uint8_t *)(init_addr+i) = data_buf[i];
            }
        }
        else if (param->type == _16_Bit)
        {
            /* Write 16 bits word */
            for (i = 0; i < length; i += 2)
            {
                /* Set value type at @ Param1 */
                value = ((uint32_t)data_buf[i+1]);
                value <<= 8;
                value |= ((uint32_t)data_buf[i+0]);
                *(volatile uint16_t *)(init_addr+i) = value;
            }
        }
        else if(param->type == _32_Bit)
        {
            /* Write 32 bit word */
            for (i = 0; i < length; i += 4)
            {
                /* Set value at @ Param1 */
                value  = ((uint32_t)data_buf[i+3]);
                value <<= 8;
                value |= ((uint32_t)data_buf[i+2]);
                value <<= 8;
                value |= ((uint32_t)data_buf[i+1]);
                value <<= 8;
                value |= ((uint32_t)data_buf[i+0]);
                *(volatile uint32_t *)(init_addr+i) = value;
            }
        }
        evt->status = CO_ERROR_NO_ERROR;
    }
    #else //RW_DEBUG_MEM
    evt->status = CO_ERROR_UNSUPPORTED;
    #endif //RW_DEBUG_MEM

    ke_msg_send(evt);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handle the reception of the vendor specific command delete param.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int
dbg_del_param_req_handler(ke_msg_id_t const msgid, struct dbg_del_param_cmd const *param,
                         ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // structure type for the complete command event
    struct dbg_common_cmd_cmp_evt *evt = KE_MSG_ALLOC(DBG_DEL_PARAM_CMP_EVT,
               TASK_HCI, TASK_DBG, dbg_common_cmd_cmp_evt);

    #if (NVDS_SUPPORT && RW_DEBUG_NVDS && NVDS_READ_WRITE)
    nvds_del(param->param_tag);
    evt->status = CO_ERROR_NO_ERROR;
    #else
    evt->status = CO_ERROR_UNSUPPORTED;
    #endif // RW_DEBUG_NVDS && NVDS_READ_WRITE


    ke_msg_send(evt);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handle the reception of the vendor specific command id flash.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int
dbg_flash_identify_req_handler(ke_msg_id_t const msgid, void const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // structure type for the complete command event
    struct dbg_flash_identify_cmp_evt *evt = KE_MSG_ALLOC(DBG_FLASH_IDENT_CMP_EVT,
            TASK_HCI, TASK_DBG, dbg_flash_identify_cmp_evt);

    #if (RW_DEBUG_FLASH)
    flash_identify(&evt->flash_id, NULL);
    evt->status = (evt->flash_id != FLASH_TYPE_UNKNOWN)?CO_ERROR_NO_ERROR : CO_ERROR_HARDWARE_FAILURE;
    #else //RW_DEBUG_FLASH
    evt->status = CO_ERROR_UNSUPPORTED;
    #endif //RW_DEBUG_FLASH

    ke_msg_send(evt);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handle the reception of the vendor specific command flash erase.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int
dbg_flash_erase_req_handler(ke_msg_id_t const msgid, struct dbg_flash_erase_cmd const *param,
                            ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // structure type for the complete command event
    struct dbg_common_cmd_cmp_evt *evt = KE_MSG_ALLOC(DBG_FLASH_ERASE_CMP_EVT,
             TASK_HCI, TASK_DBG, dbg_common_cmd_cmp_evt);

    #if (RW_DEBUG_FLASH)
    evt->status = flash_erase(param->flashtype, param->startoffset,param->size, NULL);
    #else //RW_DEBUG_FLASH
    evt->status = CO_ERROR_UNSUPPORTED;
    #endif //RW_DEBUG_FLASH

    ke_msg_send(evt);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handle the reception of the vendor specific command write flash.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int
dbg_flash_write_req_handler(ke_msg_id_t const msgid, struct dbg_flash_write_cmd const *param,
                            ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // structure type for the complete command event
    struct dbg_common_cmd_cmp_evt *evt = KE_MSG_ALLOC(DBG_FLASH_WRITE_CMP_EVT,
             TASK_HCI, TASK_DBG, dbg_common_cmd_cmp_evt);

    #if (RW_DEBUG_FLASH)
    evt->status = flash_write(param->flashtype, param->startoffset, param->buf.length,
            (uint8_t*)&param->buf.data[0], NULL);
    #else //RW_DEBUG_FLASH
    evt->status = CO_ERROR_UNSUPPORTED;
    #endif //RW_DEBUG_FLASH

    ke_msg_send(evt);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handle the reception of the vendor specific command read flash.
 *
 * Read the requested number of byte from flash.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int
dbg_flash_read_req_handler(ke_msg_id_t const msgid, struct dbg_flash_read_cmd const *param,
                           ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // structure type for the complete command event
    struct dbg_flash_read_cmp_evt *evt = KE_MSG_ALLOC(DBG_FLASH_READ_CMP_EVT,
            TASK_HCI, TASK_DBG, dbg_flash_read_cmp_evt);

    #if (RW_DEBUG_FLASH)
    evt->buf.length = param->size;
    evt->status     = flash_read(param->flashtype, param->startoffset, param->size,
            (uint8_t*)&evt->buf.data[0], NULL);
    #else //RW_DEBUG_FLASH
    evt->status = CO_ERROR_UNSUPPORTED;
    #endif //RW_DEBUG_FLASH

    ke_msg_send(evt);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handle the reception of the vendor specific command read
 *        parameter request.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int
dbg_rd_param_req_handler(ke_msg_id_t const msgid, struct dbg_rd_param_cmd const *param,
                         ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // structure type for the complete command event
    struct dbg_rd_param_cmp_evt *evt = KE_MSG_ALLOC(DBG_RD_PARAM_CMP_EVT,
          TASK_HCI, TASK_DBG, dbg_rd_param_cmp_evt);


    #if (NVDS_SUPPORT && RW_DEBUG_NVDS)
    evt->buf.length = sizeof(evt->buf.data);
    nvds_get(param->param_tag, (uint8_t*)&evt->buf.length, (uint8_t*)&evt->buf.data[0]);
    evt->status = CO_ERROR_NO_ERROR;
    #else //RW_DEBUG_NVDS
    evt->buf.length = 0;
    evt->status = CO_ERROR_UNSUPPORTED;
    #endif //RW_DEBUG_NVDS

    ke_msg_send(evt);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handle the reception of the vendor specific command write
 *        parameter request.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int
dbg_wr_param_req_handler(ke_msg_id_t const msgid, struct dbg_wr_param_cmd const *param,
                         ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // structure type for the complete command event
    struct dbg_common_cmd_cmp_evt *evt = KE_MSG_ALLOC(DBG_WR_PARAM_CMP_EVT,
          TASK_HCI, TASK_DBG, dbg_common_cmd_cmp_evt);

    #if (NVDS_SUPPORT && RW_DEBUG_NVDS && NVDS_READ_WRITE)
    nvds_put(param->param_tag, param->buf.length, (uint8_t*)&param->buf.data[0]);
    evt->status = CO_ERROR_NO_ERROR;
    #else
    evt->status = CO_ERROR_UNSUPPORTED;
    #endif // RW_DEBUG_NVDS && NVDS_READ_WRITE

    ke_msg_send(evt);

    return (KE_MSG_CONSUMED);
}



/**
 ****************************************************************************************
 * @brief Handle the reception of the vendor specific command Read Kernel Stats
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int
dbg_rd_ke_stats_req_handler(ke_msg_id_t const msgid, void const *param,
                              ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // structure type for the complete command event
    struct dbg_rd_ke_stats_cmp_evt *evt = KE_MSG_ALLOC(DBG_RD_KE_STATS_CMP_EVT,
          TASK_HCI, TASK_DBG, dbg_rd_ke_stats_cmp_evt);

    #if (KE_PROFILING)
    if(ke_stats_get(&evt->max_msg_sent, &evt->max_msg_saved, &evt->max_timer_used, &evt->max_heap_used) == KE_SUCCESS)
    {
        evt->status = CO_ERROR_NO_ERROR;
    }
    else
    {
        evt->status = CO_ERROR_UNSUPPORTED;
        evt->max_msg_sent   = 0;
        evt->max_msg_saved  = 0;
        evt->max_timer_used = 0;
        evt->max_heap_used  = 0;
    }
    #if (RW_DEBUG_STACK_PROF)
    evt->max_stack_used = get_stack_usage();
    #else //RW_DEBUG_STACK_PROF
    evt->max_stack_used = 0;
    #endif //RW_DEBUG_STACK_PROF
    #else //KE_PROFILING
    evt->status         = CO_ERROR_UNSUPPORTED;
    evt->max_msg_sent   = 0;
    evt->max_msg_saved  = 0;
    evt->max_timer_used = 0;
    evt->max_heap_used  = 0;
    evt->max_stack_used = 0;
    #endif //KE_PROFILING

    ke_msg_send(evt);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handle the reception of the vendor specific command Read Memory info
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int dbg_rd_mem_info_req_handler(ke_msg_id_t const msgid, void const *param,
                              ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    struct dbg_rd_mem_info_cmp_evt * meminfo_msg;
    #if (KE_PROFILING)
    uint8_t cursor;
    struct dbg_rd_mem_info_cmp_evt meminfo;

    // First remove command message in order to be sure it's not taken in account.
    ke_msg_free(ke_param2msg(param));

    // Then retrieve memory information from kernel
    meminfo.max_mem_used = ke_get_max_mem_usage();

    for(cursor = 0; cursor < KE_MEM_BLOCK_MAX ; cursor++)
    {
        meminfo.mem_used[cursor] = ke_get_mem_usage(cursor);
    }
    #endif // (KE_PROFILING)
    // Finally send indication to application that request memory information
    meminfo_msg = KE_MSG_ALLOC(DBG_RD_MEM_INFO_CMP_EVT, TASK_HCI, TASK_DBG, dbg_rd_mem_info_cmp_evt);

    #if (KE_PROFILING)
    memcpy(meminfo_msg, &meminfo, sizeof(struct dbg_rd_mem_info_cmp_evt));
    meminfo_msg->status = CO_ERROR_NO_ERROR;
    #else
    meminfo_msg->status = CO_ERROR_UNSUPPORTED;
    #endif // (KE_PROFILING)

    ke_msg_send(meminfo_msg);

    // source message already free
    return (KE_MSG_NO_FREE);
}

#if (DSM_SUPPORT)
/**
 ****************************************************************************************
 * @brief Handle the reception of the vendor specific command Data Storage Operation
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int dbg_data_storage_op_req_handler(ke_msg_id_t const msgid, struct dbg_data_storage_op_cmd const *param,
                              ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    struct dbg_common_stat_evt * cs_evt;
    uint8_t const * ptr;

    // Send Command status event
    cs_evt = KE_MSG_ALLOC(DBG_DATA_STORAGE_OP_CS_EVT, TASK_HCI, TASK_DBG, dbg_common_stat_evt);

    GLOBAL_INT_DISABLE()

    cs_evt->status = CO_ERROR_NO_ERROR;

    dbg_data_storage_op_curr = param->operation;
    ptr = &param->params[0];

    switch(param->operation)
    {
        case DS_OP_INIT:
        {
            /*
             * Init operation
             *
             * Parameters:
             *  XXXX: base | XXXX: sector used | XXXX: sector size | XX: nb sector
             */
            uint32_t base = co_htobl(co_read32p(ptr));
            uint32_t used = co_htobl(co_read32p(ptr+4));
            uint32_t max  = co_htobl(co_read32p(ptr+8));
            uint8_t nb   = *(ptr+12);

            dsm_init(base, used, max, nb, &data_storage_op_callback);
        }
        break;

        case DS_OP_CLEAR:
        {
            // Clear operation
            dsm_clear(&data_storage_op_callback);
        }
        break;

        case DS_OP_START_WRITE:
        {
            // Start operation
            cs_evt->status = dsm_start_write(&data_storage_op_callback_block);
        }
        break;

        case DS_OP_PUSH_SAMPLE:
        {
            /*
             * Push operation
             *
             * Parameters:
             *  XX: size | X...X: sample
             */
            uint8_t size = *(ptr);
            cs_evt->status = dsm_push_sample(size , (uint8_t *) ptr+1, &data_storage_op_callback);
        }
        break;

        case DS_OP_STOP_WRITE:
        {
            // Start operation
            cs_evt->status = dsm_stop_write(&data_storage_op_callback);
        }
        break;

        case DS_OP_START_READ:
        {
            // Start read operation
            cs_evt->status = dsm_start_read(&data_storage_op_callback_block);
        }
        break;

        case DS_OP_POP_SAMPLE:
        {
            // Pop sample operation
            cs_evt->status = dsm_pop_sample(&data_storage_op_callback_sample);
        }
        break;

        case DS_OP_STOP_READ:
        {
            // Stop read operation
            cs_evt->status = dsm_stop_read();
        }
        break;

        case DS_OP_DELETE_BLOCK:
        {
            // Delete block operation
            cs_evt->status = dsm_delete_block(&data_storage_op_callback);
        }
        break;

        case DS_OP_NOP:
        default:
        {

        }
        break;
    }

    GLOBAL_INT_RESTORE()

    ke_msg_send(cs_evt);

    return (KE_MSG_CONSUMED);
}
#endif //DSM_SUPPORT

/**
 ****************************************************************************************
 * @brief Handle the reception of the vendor specific command Read RF Register value.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int
dbg_rf_reg_rd_req_handler(ke_msg_id_t const msgid, struct dbg_rf_reg_rd_cmd const *param,
                          ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // structure type for the complete command event
    struct dbg_rf_reg_rd_cmp_evt *event = KE_MSG_ALLOC(DBG_RF_REG_RD_CMP_EVT,
            TASK_HCI, TASK_DBG, dbg_rf_reg_rd_cmp_evt);

    #if (BLE_STD_MODE && BLE_EMB_PRESENT)
    // Unlock BLE<->RF SPI access
//    ble_spiswaccdbgen_setf(1);
    #endif //BLE_STD_MODE && BLE_EMB_PRESENT

    // Read RF register
    event->value   = co_htobl(rwip_rf.reg_rd(co_btohs(param->addr)));

    #if (BLE_STD_MODE && BLE_EMB_PRESENT)
    // Restore BLE<->RF SPI access lock
 //   ble_spiswaccdbgen_setf(0);
    #endif //BLE_STD_MODE && BLE_EMB_PRESENT

    event->addr    = param->addr;
    event->status  = CO_ERROR_NO_ERROR;

    // sends the message
    ke_msg_send(event);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handle the reception of the vendor specific command Write RF Register value.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int
dbg_rf_reg_wr_req_handler(ke_msg_id_t const msgid, struct dbg_rf_reg_wr_cmd const *param,
                          ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // structure type for the complete command event
    struct dbg_rf_reg_wr_cmp_evt *event = KE_MSG_ALLOC(DBG_RF_REG_WR_CMP_EVT,
              TASK_HCI, TASK_DBG, dbg_rf_reg_wr_cmp_evt);

    #if (BLE_STD_MODE && BLE_EMB_PRESENT)
    // Unlock BLE<->RF SPI access
//    ble_spiswaccdbgen_setf(1);
    #endif //BLE_STD_MODE && BLE_EMB_PRESENT

    // Write RF register
    rwip_rf.reg_wr(co_btohs(param->addr), co_btohl(param->value));

    #if (BLE_STD_MODE && BLE_EMB_PRESENT)
    // Restore BLE<->RF SPI access lock
  //  ble_spiswaccdbgen_setf(0);
    #endif //BLE_STD_MODE && BLE_EMB_PRESENT

    event->addr    = param->addr;
    event->status  = CO_ERROR_NO_ERROR;

    // sends the message
    ke_msg_send(event);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handle the reception of the vendor specific command platform reset.
 * This command reset FW and return to ROM code.
 *
 * @note command complete(success) is sent by ROM code.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int
dbg_plf_reset_req_handler(ke_msg_id_t const msgid, struct dbg_plf_reset_cmd const *param,
        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint32_t error_code = RESET_NO_ERROR;

    // structure type for the complete command event
    struct dbg_common_cmd_cmp_evt *evt = KE_MSG_ALLOC(DBG_PLF_RESET_CMP_EVT,
            src_id, dest_id, dbg_common_cmd_cmp_evt);

    switch(param->reason)
    {
        case PLATFORM_RESET_TO_FW: error_code = RESET_AND_LOAD_FW; break;
        case PLATFORM_RESET_TO_ROM: error_code = RESET_TO_ROM; break;
        default: break;
    }

    if(error_code != RESET_NO_ERROR)
    {
        // Perform platform Reset (return to ROM code)
        platform_reset(error_code);
    }

    // If reason is not valid, returns a CC event with bad status
    evt->status = CO_ERROR_INVALID_HCI_PARAM;
    ke_msg_send(evt);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handle the reception of the vendor specific command read hw register.
 * Return the value of the register requested in the command.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int
dbg_hw_reg_rd_req_handler(ke_msg_id_t const msgid,
                         struct dbg_hw_reg_rd_cmd const *param,
                         ke_task_id_t const dest_id,
                         ke_task_id_t const src_id)
{
    // structure type for the complete command event
    struct dbg_hw_reg_rd_cmd_cmp_evt *event;

    // allocate the status event message
    event =  KE_MSG_ALLOC(DBG_HW_REG_RD_CMP_EVT, src_id, dest_id,
            dbg_hw_reg_rd_cmd_cmp_evt);

    #if (BLE_EMB_PRESENT)
    //event->reg_value = co_htobl(REG_BLE_RD(REG_BLECORE_BASE_ADDR +
                                      //      co_btohs(param->reg_addr)));
 
    #endif //BLE_EMB_PRESENT / BT_EMB_PRESENT
    event->reg_addr = param->reg_addr;
    event->status = CO_ERROR_NO_ERROR;
    // sends the message
    ke_msg_send(event);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handle the reception of the vendor specific command write hw register.
 * Write the requested value in the register passed in the command parameter.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int
dbg_hw_reg_wr_req_handler(ke_msg_id_t const msgid,
                         struct dbg_hw_reg_wr_cmd const *param,
                         ke_task_id_t const dest_id,
                         ke_task_id_t const src_id)
{
    // structure type for the complete command event
    struct dbg_hw_reg_wr_cmd_cmp_evt *event;

    // allocate the status event message
    event =  KE_MSG_ALLOC(DBG_HW_REG_WR_CMP_EVT, src_id, dest_id,
            dbg_hw_reg_wr_cmd_cmp_evt);

    #if (BLE_EMB_PRESENT)
//    REG_BLE_WR(REG_BLECORE_BASE_ADDR + co_btohs(param->reg_addr),
            //   co_btohl(param->reg_value));
    #elif (BT_EMB_PRESENT)
    REG_BT_WR(REG_BTCORE_BASE_ADDR + co_btohs(param->reg_addr),
               co_btohl(param->reg_value));
    #endif //BLE_EMB_PRESENT / BT_EMB_PRESENT


    event->reg_addr = param->reg_addr;
    event->status = CO_ERROR_NO_ERROR;
    // sends the message
    ke_msg_send(event);

    return (KE_MSG_CONSUMED);
}


#if (BLE_EMB_PRESENT)
/**
 ****************************************************************************************
 * @brief Handle the reception of the vendor specific command set bd address.
 * Write the requested bd address in the dedicated registers.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int
dbg_set_bd_addr_req_handler(ke_msg_id_t const msgid,
                         struct dbg_set_bd_addr_cmd const *param,
                         ke_task_id_t const dest_id,
                         ke_task_id_t const src_id)
{
    // structure type for the complete command event
    struct dbg_common_cmd_cmp_evt *event;

    // Set public address
 //   llm_util_set_public_addr((struct bd_addr *) &param->addr);

    // Sends the CC event
    event = KE_MSG_ALLOC(DBG_LE_SET_BD_ADDR_CMP_EVT, src_id, dest_id, dbg_common_cmd_cmp_evt);
    event->status = CO_ERROR_NO_ERROR;
    ke_msg_send(event);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handle the reception of the vendor specific command set public address type..
 * Write the requested type in the dedicated bit field.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int
dbg_set_type_pub_req_handler(ke_msg_id_t const msgid,
                         void const *param,
                         ke_task_id_t const dest_id,
                         ke_task_id_t const src_id)
{
    // structure type for the complete command event
    struct dbg_common_cmd_cmp_evt *event;

    // allocate the status event message
    event =  KE_MSG_ALLOC(DBG_LE_SET_TYPE_PUB_CMP_EVT, src_id, dest_id, dbg_common_cmd_cmp_evt);
    event->status = CO_ERROR_NO_ERROR;
    // sends the message
    ke_msg_send(event);

    return (KE_MSG_CONSUMED);
}
/**
 ****************************************************************************************
 * @brief Handle the reception of the vendor specific command set random address type.
 * Write the requested type in the dedicated bit field.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int
dbg_set_type_rand_req_handler(ke_msg_id_t const msgid,
                         void const *param,
                         ke_task_id_t const dest_id,
                         ke_task_id_t const src_id)
{
    // structure type for the complete command event
    struct dbg_common_cmd_cmp_evt *event;

    // allocate the status event message
    event =  KE_MSG_ALLOC(DBG_LE_SET_TYPE_RAND_CMP_EVT, src_id, dest_id, dbg_common_cmd_cmp_evt);
    event->status = CO_ERROR_NO_ERROR;
    // sends the message
    ke_msg_send(event);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handle the reception of the vendor specific command set crc.
 * Write the requested crc in the dedicated registers.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int
dbg_set_crc_req_handler(ke_msg_id_t const msgid,
                         struct dbg_set_crc_cmd const *param,
                         ke_task_id_t const dest_id,
                         ke_task_id_t const src_id)
{
    // structure type for the complete command event
    struct dbg_common_cmd_cmp_evt *event;
    uint16_t conhdl;

    // Check if the command is for the advertising channel or a connection link
    if (param->conhdl >= BLE_CONNECTION_MAX)
        conhdl = 0xff;		//default, change later LLD_ADV_HDL;
    else
        conhdl = param->conhdl;

    // Set the random address in the environment variable
    // Initialize the crcinit in the CS
//    ble_crcinit0_set(conhdl, co_htobs(co_read16p(&param->crc.crc[0])));
//    ble_crcinit1_setf(conhdl,param->crc.crc[2]);
    // allocate the status event message
    event =  KE_MSG_ALLOC(DBG_LE_SET_CRC_CMP_EVT, src_id, dest_id,
            dbg_common_cmd_cmp_evt);
    event->status = CO_ERROR_NO_ERROR;
    // sends the message
    ke_msg_send(event);

    ke_state_set(dest_id, DBG_IDLE);
    return (KE_MSG_CONSUMED);
}
#if (BLE_PERIPHERAL || BLE_CENTRAL)
/**
 ****************************************************************************************
 * @brief Handle the reception of the vendor specific command llcp discard.
 * Write the requested crc in the dedicated registers.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int
dbg_le_llcp_discard_req_handler(ke_msg_id_t const msgid,
                                struct dbg_le_llcp_discard_req const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    // structure type for the complete command event
    struct dbg_common_cmd_cmp_evt *event;

    // Set the LLCP discard flag in the LLC environment
//    llc_util_set_llcp_discard_enable(param->conhdl, param->enable);

    // allocate the status event message
    event =  KE_MSG_ALLOC(DBG_LE_LLCP_DISCARD_CMP_EVT, src_id, dest_id, dbg_common_cmd_cmp_evt);
    event->status = CO_ERROR_NO_ERROR;
    // sends the message
    ke_msg_send(event);

    return (KE_MSG_CONSUMED);
}
#endif // #if (BLE_PERIPHERAL || BLE_CENTRAL)
/**
 ****************************************************************************************
 * @brief Handle the reception of the vendor specific command reset rx counter in the CS.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int
dbg_reset_rx_cnt_req_handler(ke_msg_id_t const msgid,
                         struct dbg_reset_cnt_req const *param,
                         ke_task_id_t const dest_id,
                         ke_task_id_t const src_id)
{
    // structure type for the complete command event
    struct dbg_common_cmd_cmp_evt *event;

    // allocate the status event message
    event =  KE_MSG_ALLOC(DBG_LE_RESET_RX_CNT_CMP_EVT, src_id, dest_id,
            dbg_common_cmd_cmp_evt);
//    ble_rxccmpktcnt0_set((uint8_t)param->conhdl,0);
//    ble_rxccmpktcnt1_set((uint8_t)param->conhdl,0);
//    ble_rxccmpktcnt2_set((uint8_t)param->conhdl,0);
    event->status = CO_ERROR_NO_ERROR;
    // sends the message
    ke_msg_send(event);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handle the reception of the vendor specific command reset tx counter in the CS.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int
dbg_reset_tx_cnt_req_handler(ke_msg_id_t const msgid,
                         struct dbg_reset_cnt_req const *param,
                         ke_task_id_t const dest_id,
                         ke_task_id_t const src_id)
{
    // structure type for the complete command event
    struct dbg_common_cmd_cmp_evt *event;

    // allocate the status event message
    event =  KE_MSG_ALLOC(DBG_LE_RESET_TX_CNT_CMP_EVT, src_id, dest_id,
            dbg_common_cmd_cmp_evt);
//    ble_txccmpktcnt0_set((uint8_t)param->conhdl,0);
//    ble_txccmpktcnt1_set((uint8_t)param->conhdl,0);
//    ble_txccmpktcnt2_set((uint8_t)param->conhdl,0);
    event->status = CO_ERROR_NO_ERROR;
    // sends the message
    ke_msg_send(event);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handle the reception of the vendor specific command Set Tx power Level in CS.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int
dbg_set_tx_pw_req_handler (ke_msg_id_t const msgid,
                            struct dbg_set_tx_pw_cmd const *param,
                            ke_task_id_t const dest_id,
                            ke_task_id_t const src_id)
{
    // structure type for the complete command event
    struct dbg_common_cmd_cmp_evt *event= KE_MSG_ALLOC(DBG_SET_TX_PW_CMP_EVT, src_id, dest_id,
                                                   dbg_common_cmd_cmp_evt);
    uint16_t conhdl;
    struct lld_evt_tag *evt;

    // Check if the command is for the advertising channel or a connection link
    if (param->conhdl >= BLE_CONNECTION_MAX)
        conhdl = 0xff;//LLD_ADV_HDL;
    else
        conhdl = param->conhdl;

    // Get the event associated with this connection handle
//    evt = lld_evt_conhdl2evt(conhdl);

    // Update the TX power for this event
    if (evt != NULL)
    {
        event->status = CO_ERROR_NO_ERROR;
//        lld_evt_txpwr_update(evt, param->pw_lvl);
    }
    else
    {
        event->status = CO_ERROR_INVALID_HCI_PARAM;
    }

    // send the message
    ke_msg_send(event);

    ke_state_set(dest_id, DBG_IDLE);
    return (KE_MSG_CONSUMED);
}
#endif //BLE_EMB_PRESENT

#if (RW_WLAN_COEX)
/**
 ****************************************************************************************
 * @brief Handle the reception of the vendor specific command wlan coexistence.
 * This command set the coexistence state.
 *
 * @note command complete(success) is sent by ROM code.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int
dbg_wlan_coex_req_handler(ke_msg_id_t const msgid, struct dbg_wlan_coex_cmd const *param,
        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{

    // structure type for the complete command event
    struct dbg_common_cmd_cmp_evt *evt = KE_MSG_ALLOC(DBG_WLAN_COEX_CMP_EVT,
            src_id, dest_id, dbg_common_cmd_cmp_evt);
    #if (BLE_EMB_PRESENT)
    lld_wlcoex_set(param->state);
    #endif //BLE_EMB_PRESENT

    // If reason is not valid, returns a CC event with bad status
    evt->status = CO_ERROR_NO_ERROR;
    ke_msg_send(evt);

    return (KE_MSG_CONSUMED);
}

#if (RW_WLAN_COEX_TEST)
/**
 ****************************************************************************************
 * @brief Handle the reception of the vendor specific command wlan coexistence.
 * This command set the coexistence state.
 *
 * @note command complete(success) is sent by ROM code.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int
dbg_wlan_coextst_scen_req_handler(ke_msg_id_t const msgid, struct dbg_wlan_coextst_scen_cmd const *param,
        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{

    // structure type for the complete command event
    struct dbg_common_cmd_cmp_evt *evt = KE_MSG_ALLOC(DBG_WLAN_COEXTST_SCEN_CMP_EVT,
            src_id, dest_id, dbg_common_cmd_cmp_evt);
    #if (BLE_EMB_PRESENT)
    lld_wlcoex_scen_set(param->scenario);
    #endif // BLE_EMB_PRESENT

    // If reason is not valid, returns a CC event with bad status
    evt->status = CO_ERROR_NO_ERROR;
    ke_msg_send(evt);

    return (KE_MSG_CONSUMED);
}
#endif //RW_WLAN_COEX_TEST
#endif //RW_WLAN_COEX


/*
 * TASK DESCRIPTOR DEFINITIONS
 ****************************************************************************************
 */

/// Specifies the message handlers that are common to all states.
static const struct ke_msg_handler dbg_default_state[] =
{
    {DBG_RD_MEM_REQ,         (ke_msg_func_t)dbg_rd_mem_req_handler},
    {DBG_WR_MEM_REQ,         (ke_msg_func_t)dbg_wr_mem_req_handler},
    {DBG_DEL_PARAM_REQ,      (ke_msg_func_t)dbg_del_param_req_handler},
    {DBG_FLASH_IDENT_REQ,    (ke_msg_func_t)dbg_flash_identify_req_handler},
    {DBG_FLASH_ERASE_REQ,    (ke_msg_func_t)dbg_flash_erase_req_handler},
    {DBG_FLASH_WRITE_REQ,    (ke_msg_func_t)dbg_flash_write_req_handler},
    {DBG_FLASH_READ_REQ,     (ke_msg_func_t)dbg_flash_read_req_handler},
    {DBG_RD_PARAM_REQ,       (ke_msg_func_t)dbg_rd_param_req_handler},
    {DBG_WR_PARAM_REQ,       (ke_msg_func_t)dbg_wr_param_req_handler},
    {DBG_RD_KE_STATS_REQ,    (ke_msg_func_t)dbg_rd_ke_stats_req_handler},
    {DBG_RF_REG_RD_REQ,      (ke_msg_func_t)dbg_rf_reg_rd_req_handler},
    {DBG_RF_REG_WR_REQ,      (ke_msg_func_t)dbg_rf_reg_wr_req_handler},
    {DBG_PLF_RESET_REQ,      (ke_msg_func_t)dbg_plf_reset_req_handler},
    {DBG_HW_REG_RD_REQ, (ke_msg_func_t)dbg_hw_reg_rd_req_handler},
    {DBG_HW_REG_WR_REQ, (ke_msg_func_t)dbg_hw_reg_wr_req_handler},

    #if (BLE_EMB_PRESENT)
    {DBG_LE_SET_BD_ADDR_REQ, (ke_msg_func_t)dbg_set_bd_addr_req_handler},
    {DBG_LE_SET_TYPE_PUB_REQ, (ke_msg_func_t)dbg_set_type_pub_req_handler},
    {DBG_LE_SET_TYPE_RAND_REQ, (ke_msg_func_t)dbg_set_type_rand_req_handler},
    {DBG_LE_SET_CRC_REQ, (ke_msg_func_t)dbg_set_crc_req_handler},
    #if (BLE_PERIPHERAL || BLE_CENTRAL)
    {DBG_LE_LLCP_DISCARD_REQ, (ke_msg_func_t)dbg_le_llcp_discard_req_handler},
    #endif // #if (BLE_PERIPHERAL || BLE_CENTRAL)
    {DBG_LE_RESET_RX_CNT_REQ, (ke_msg_func_t)dbg_reset_rx_cnt_req_handler},
    {DBG_LE_RESET_TX_CNT_REQ, (ke_msg_func_t)dbg_reset_tx_cnt_req_handler},
    {DBG_SET_TX_PW_REQ, (ke_msg_func_t)dbg_set_tx_pw_req_handler},
    #endif //BLE_EMB_PRESENT
    #if (RW_WLAN_COEX)
    {DBG_WLAN_COEX_REQ, (ke_msg_func_t)dbg_wlan_coex_req_handler},
    #if (RW_WLAN_COEX_TEST)
    {DBG_WLAN_COEXTST_SCEN_REQ, (ke_msg_func_t)dbg_wlan_coextst_scen_req_handler},
    #endif //RW_WLAN_COEX_TEST
    #endif //RW_WLAN_COEX
    {DBG_RD_MEM_INFO_REQ, (ke_msg_func_t)dbg_rd_mem_info_req_handler},
    #if (DSM_SUPPORT)
    {DBG_DATA_STORAGE_OP_REQ, (ke_msg_func_t)dbg_data_storage_op_req_handler},
    #endif //(DSM_SUPPORT)
};

/// Specifies the message handlers that are common to all states.
const struct ke_state_handler dbg_default_handler = KE_STATE_HANDLER(dbg_default_state);

/// Defines the placeholder for the states of all the task instances.
ke_state_t dbg_state[DBG_IDX_MAX];

#endif
/// @} DBGTASK
