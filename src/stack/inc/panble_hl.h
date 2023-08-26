
#ifndef PANBLE_HL_H_
#define PANBLE_HL_H_

#include <stdint.h>

/**
 ****************************************************************************************
 * @addtogroup ROOT
 * @brief Entry points of the BLE Host stack
 *
 * This module contains the primitives that allow an application accessing and running the
 * BLE protocol stack
 *
 * @{
 ****************************************************************************************
 */


/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @brief Initialize the BLE Host stack.
 ****************************************************************************************
 */
void rwble_hl_init(void);

/**
 ****************************************************************************************
 * @brief Initialize the host (reset requested)
 *
 ****************************************************************************************
 */
void rwble_hl_reset(void);

/// @} PANBLE_HL

#endif // PANBLE_HL_H_
