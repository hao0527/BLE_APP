
/**
****************************************************************************************
* @addtogroup DBG
* @{
****************************************************************************************
*/

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"    // stack configuration

#if (SDK_USELESS_CODE)
#include "co_error.h"       // common error definition
#include "ke_task.h"        // kernel task definition
#include "dbg_task.h"       // debug task definition
#include "dbg.h"            // debug definition

#include <string.h>         // memcpy and strlen definition
#include <stdio.h>          // standard io definition
#include <stdarg.h>         // standard arg definition


/*
 * DEFINES
 ****************************************************************************************
 */

#define DBG_TRACE_WARNING_MAX_SIZE   128


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// DEBUG task descriptor
static const struct ke_task_desc TASK_DESC_DBG = {NULL, &dbg_default_handler, dbg_state, DBG_STATE_MAX, DBG_IDX_MAX,0};

/*
 * LOCAL FUNCTION DEFINITION
 ****************************************************************************************
 */
 


/**
 ****************************************************************************************
 * @brief Send a trace message via HCI.
 *
 * @param[in] message   Pointer on message string.
 * @param[in] msg_len   Length of the message string.
 ****************************************************************************************
 */

static void dbg_trace_warning(const char * message, uint8_t msg_len)
{
    uint8_t len = msg_len;

    struct dbg_trace_warning_evt *evt =
                  KE_MSG_ALLOC(DBG_TRACE_WARNING_EVT, TASK_HCI, TASK_DBG, dbg_trace_warning_evt);

    // Message must fit the buffer, with a place for a NULL character
    if(len >= sizeof(evt->buf.data))
    {
        len = sizeof(evt->buf.data) - 1;
    }

    // Fill event parameters
    evt->buf.length = len;
    memcpy(&evt->buf.data[0], message, len);
    evt->buf.data[len] = 0;

    ke_msg_send(evt);
}




/*
 * EXPORTED FUNCTION DEFINITION
 ****************************************************************************************
 */

void dbg_init(void)
{
    // Create DEBUG Task
    ke_task_create(TASK_DBG, &TASK_DESC_DBG);

    // Initialize DBG task to idle state
    ke_state_set(TASK_DBG,DBG_IDLE);

    #if (RW_SWDIAG)
    // Initialize SW profiling module
    dbg_swdiag_init();
    #endif //BT_SWDIAG

}


void dbg_warning(const char *format, ...)
{

    char buffer[DBG_TRACE_WARNING_MAX_SIZE];
    va_list Args;
    uint8_t msg_len;
    // Format text to display
    va_start(Args, format);
    msg_len = vsprintf(buffer, format, Args);
    va_end(Args);
    dbg_trace_warning(buffer, msg_len);
}


void dbg_platform_reset_complete(uint32_t error)
{
    // structure type for the complete command event
    struct dbg_common_cmd_cmp_evt *evt = KE_MSG_ALLOC(DBG_PLF_RESET_CMP_EVT,
            TASK_HCI, TASK_DBG, dbg_common_cmd_cmp_evt);

    if(error == RESET_TO_ROM)
    {
        evt->status = CO_ERROR_HARDWARE_FAILURE;
    }
    else if(error == RESET_AND_LOAD_FW)
    {
        evt->status = CO_ERROR_NO_ERROR;
    }

    ke_msg_send(evt);
}
#endif
///@} DBG
