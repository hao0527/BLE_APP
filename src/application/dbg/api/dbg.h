#ifndef DBG_H_
#define DBG_H_

/**
****************************************************************************************
* @addtogroup DBG
* @ingroup CONTROLLER
* @brief Debug
*
* @{
****************************************************************************************
*/


/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "panip_config.h"     // stack configuration

/*
 * FUNCTION DECLARATION
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialization of the BT Debug task
 *
 * This function initializes the the DBG task
 *
 ****************************************************************************************
 */
void dbg_init(void);

/**
 ****************************************************************************************
 * @brief Print debug messages.
 *
 * @param[in] format         data buffer for printing
 *
 * @return Status
 *
 ****************************************************************************************
 */
void dbg_warning(const char *format, ...);


/**
 ****************************************************************************************
 * @brief Send back to host status of platform reset request.
 *
 * @param status Reset error code
 *
 ****************************************************************************************
 */
void dbg_platform_reset_complete(uint32_t error);

///@} DBG

#endif // DBG_H_
