#include "flash_manager_internal.h"

const fm_entry_t * entry_get(const fm_entry_t * p_start_entry,
                             const void * p_end,
                             fm_handle_t handle)
{
    if(p_start_entry <= (const fm_entry_t *) p_end)
    {
    }else
    {
        printf("entry_get\n");
    }

    const fm_entry_t * p_entry = p_start_entry;
    while (p_entry->header.handle != handle)
    {
        /* Don't jump to next page if p_end is the next page. It would cause reads after p_end. */
        if (p_end <= (const void *) (PAGE_START_ALIGN(p_entry) + PAGE_SIZE) &&
            (p_entry->header.handle == HANDLE_PADDING ||
             p_entry + p_entry->header.len_words >= (const fm_entry_t *) p_end))
        {
            return NULL;
        }

        p_entry = get_next_entry(p_entry);

        if (p_entry >= (const fm_entry_t *) p_end || (p_entry <= p_start_entry))
        {
            return NULL;
        }
    }
    return p_entry;
}

