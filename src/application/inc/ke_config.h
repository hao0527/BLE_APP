
#ifndef _KE_CONFIG_H_
#define _KE_CONFIG_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "panip_config.h"       // stack configuration

/**
 ****************************************************************************************
 * @addtogroup KERNEL KERNEL
 * @ingroup ROOT
 * @brief The Kernel module.
 *
 * The Kernel is responsible for providing essential OS features like time management,
 * inter-task communication, task management and message handling and administration.
 *
 * @{
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup CFG Settings and Configuration
 * @ingroup KERNEL
 * @brief Kernel Configuration
 *
 * @{
 ****************************************************************************************
 */

/*
 * CONSTANT DEFINITIONS
 ****************************************************************************************
 */
#define KE_MEM_RW       1
#define KE_MEM_LINUX    0
#define KE_MEM_LIBC     0

#define KE_FULL         1
#define KE_SEND_ONLY    0

/// @} CFG

#endif // _KE_CONFIG_H_
