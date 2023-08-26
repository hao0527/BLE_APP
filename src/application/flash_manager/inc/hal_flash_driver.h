#ifndef __HAL_FLASH_DRIVER__H
#define __HAL_FLASH_DRIVER__H






#include <stdint.h>


/**
 * Erase all pages in flash covered by the specified range.
 *
 * @param[in] p_page First address in the first page to be erased.
 * @param[in] size Number of bytes to erase. All pages that have any data in
 * the range will be erased.
 *
 * @retval PAN_SUCCESS The pages starting at @c p_page, covering all bytes in
 * @c size have been erased.
 * @retval PAN_ERROR_INVALID_ADDR The @c p_page parameter wasn't page aligned.
 * @retval PAN_ERROR_INVALID_LENGTH The @c size parameter was 0.
 */
uint32_t pan_flash_erase(uint32_t * p_page, uint32_t size);

/**
 * Write data array to flash.
 *
 * @param[in,out] p_dst Address of the first word in the page to be filled.
 * @param[in] p_src Array of data to flash.
 * @param[in] size Size of the p_src data in bytes.
 *
 * @retval PAN_SUCCESS All data in @c p_src was successfully written to flash.
 * @retval PAN_ERROR_INVALID_ADDR One or more of the given addresses weren't
 * page aligned.
 * @retval PAN_ERROR_INVALID_LENGTH The @c size parameter was either 0 or not
 * word aligned.
 */
uint32_t pan_flash_write(uint32_t * p_dst, const uint32_t * p_src, uint32_t size);




#endif

