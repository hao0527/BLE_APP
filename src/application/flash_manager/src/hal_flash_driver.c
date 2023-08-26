#include "hal_flash_driver.h"
#include "utils.h"
#include "mesh_error.h"
#include "fmc.h"
#include "flash_config.h"
//#define FLASH_BLANK_WORD    (0x102000)                 //8K flash

/*****************************************************************************
* Interface functions
*****************************************************************************/
uint32_t pan_flash_erase(uint32_t * p_page, uint32_t size)
{
    
    if (!IS_PAGE_ALIGNED(p_page))
    {
        return PAN_ERROR_INVALID_ADDR;
    }
    if (size == 0)
    {
        return PAN_ERROR_INVALID_LENGTH;
    }

    uint32_t num_pages = (size + PAGE_SIZE - 1) / PAGE_SIZE;

    for (uint32_t  i = 0; i < num_pages; i++)
    {
        /* Erase page: */
        FMC_Erase((uint32_t)p_page);
        p_page += PAGE_SIZE / sizeof(uint32_t*);
    }
    return PAN_SUCCESS;
}

uint32_t pan_flash_write(uint32_t * p_dst, const uint32_t* p_src, uint32_t size)
{
    
    if (!IS_WORD_ALIGNED(p_dst) || !IS_WORD_ALIGNED(p_src))
    {
        return PAN_ERROR_INVALID_ADDR;
    }
    if (size == 0 || !IS_WORD_ALIGNED(size))
    {
        return PAN_ERROR_INVALID_LENGTH;
    }

    size /= WORD_SIZE;

    while (size--)
    {
//        if (*p_src != FLASH_BLANK_WORD)
        {
            FMC_Write((uint32_t)p_dst,*p_src);  
        }
        p_dst++;
        p_src++;
    }

    return PAN_SUCCESS;
}
