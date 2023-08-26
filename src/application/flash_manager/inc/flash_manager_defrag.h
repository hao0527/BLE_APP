#ifndef FLASH_MANAGER_DEFRAG_H__
#define FLASH_MANAGER_DEFRAG_H__

#include "flash_manager.h"

/**
 * @internal
 * @defgroup   FLASH_MANAGER_DEFRAG Flash Manager Defrag Submodule
 * Defrag procedure handler for the flash manager module.
 * @warning    The @ref flash_manager_defrag_init and @ref flash_manager_defrag should not interrupt
 *             each other since these are non-reentrant functions and they share a common state.
 * @{
 */

/**
 * Initialize flash manager defrag handler.
 *
 * @return     Whether recovery was started.
 */
bool flash_manager_defrag_init(void);

/**
 * Check whether the given manager is being defragged.
 *
 * @param[in]  p_manager  Manager to check for.
 *
 * @return     Whether the given manager is being defragged.
 */
bool flash_manager_defragging(const flash_manager_t * p_manager);

/**
 * Check whether a defrag procedure is currently running.
 *
 * @returns Whether a defrag procedure is currently running.
 */
bool flash_manager_defrag_is_running(void);

/**
 * Get a pointer to the page currently being defragged.
 *
 * @return     A pointer to the page being defragged, or NULL if no defrag procedure is currently in
 *             progress.
 */
const flash_manager_page_t * flash_manager_defrag_page_get(void);

/**
 * Defrag the given flash manager.
 *
 * @warning    The p_manager must be complete with valid entries and in state @ref FM_STATE_READY
 *
 * @param[in]  p_manager  The flash manager instance to defrag.
 */
void flash_manager_defrag(const flash_manager_t * p_manager);

/**
 * Get a pointer to the flash page being used as a recovery area.
 *
 * @return     Pointer to the start of the recovery area. Always page aligned.
 */
const void * flash_manager_defrag_recovery_page_get(void);

void flash_manager_defrag_reset(void);

/** @} */

#endif /* FLASH_MANAGER_DEFRAG_H__ */

