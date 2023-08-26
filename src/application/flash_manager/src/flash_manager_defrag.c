#include "flash_manager_defrag.h"
#include <string.h>
#include "flash_manager_internal.h"
#include "flash_config.h"
#include "utils.h"
//#include "internal_event.h"

/*****************************************************************************
* Local defines
*****************************************************************************/
#define DEFRAG_RECOVER_STEP (erase_source)
#define MAX_PROCEDURE_STEP (sizeof(m_procedure_steps) / sizeof(m_procedure_steps[0]))

/*****************************************************************************
* Local typedefs
*****************************************************************************/
/** Global defrag states */
typedef enum
{
    DEFRAG_STATE_IDLE, /**< No defrag is currently in progress. */
    DEFRAG_STATE_PROCESSING, /**< Currently in the middle of a defrag procedure. */
    DEFRAG_STATE_STABILIZING /**< Stabilizing flash contents after a finished defrag procedure. */
} defrag_state_t;

/** Procedure step end action. Returned by each procedure step to indicate what the next step in
 *  the procedure should be. */
typedef enum
{
    PROCEDURE_STAY,     /**< Wait for a flash operation to finish, and repeat the current step. */
    PROCEDURE_CONTINUE, /**< Move on to the next step in the procedure. */
    PROCEDURE_END,      /**< End the procedure. */
    PROCEDURE_RESTART,  /**< Start the procedure from the beginning. */
} procedure_action_t;

typedef struct
{
    defrag_state_t state;
    uint32_t step; /**< Current step number in the backup procedure. */
    const flash_manager_t * p_manager; /**< Flash manager owning the page currently being defragged. */
    const flash_manager_page_t * p_storage_page; /**< Page being defragmented. */
    const fm_entry_t * p_src; /**< Next entry to copy */
    const fm_entry_t * p_dst; /**< Next destination in recovery page. */
    bool wait_for_idle;       /**< Flag, that when set makes the procedure wait for all flash operations to end before proceeding. */
    bool found_all_entries;   /**< Whether we've ran through all entries in the original area. */
} defrag_t;

/** Single chunk of entries. */
typedef struct
{
    const fm_entry_t * p_start; /**< First entry in the chunk. */
    uint32_t length; /**< Length of the chunk in bytes. */
} fm_entry_chunk_t;

typedef procedure_action_t (*defrag_procedure_step_t)(void);

/*****************************************************************************
* Static globals
*****************************************************************************/
flash_manager_recovery_area_t * mp_recovery_area; /**< Recovery area pointer into flash. */
defrag_t m_defrag; /**< Global defrag state. */
uint16_t m_flash_token; /**< Flash operation token returned from the mesh flash module. */

/* We're iterating through pages with the assumption that one flash_manager_page_t and
 * flash_manager_recovery_area_t are exactly one page long, and that fm_entry_t is exactly one word.
 * Ensure these assumptions are true: */
PAN_MESH_STATIC_ASSERT(sizeof(flash_manager_page_t) == PAGE_SIZE);
PAN_MESH_STATIC_ASSERT(sizeof(flash_manager_recovery_area_t) == PAGE_SIZE);
PAN_MESH_STATIC_ASSERT(sizeof(fm_entry_t) == WORD_SIZE);

/*****************************************************************************
* Local utility functions
*****************************************************************************/
/**
 * Get the largest possible continuous chunk of data entries starting at the first entry
 * representing data after the @p p_src pointer.
 *
 * @param[out] Chunk object to populate.
 * @param[in] p_src Entry to start looking for a chunk from.
 * @param[in] p_end The upper boundary of memory to look through.
 * @param[in] max_length The maximum length of the chunk. The resulting chunk will never exceed
 * this length.
 *
 * @returns The first entry after the chunk, or NULL if there were no more valid data entries
 * before meeting the boundary conditions.
 */
const fm_entry_t * get_chunk_of_data_entries(fm_entry_chunk_t * p_chunk,
                                                           const fm_entry_t * p_src,
                                                           const void *       p_end,
                                                           uint32_t           max_length)
{
    p_chunk->length = 0;
    /* find start of chunk */
    if (!handle_represents_data(p_src->header.handle))
    {
        p_src = get_next_data_entry(p_src, p_end);
        if (p_src == NULL)
        {
            p_chunk->p_start = NULL;
            return NULL;
        }
    }
    p_chunk->p_start = p_src;

    /* find end of chunk */
    while (((const void *) p_src < p_end) &&
           (p_chunk->length + p_src->header.len_words * WORD_SIZE < max_length) &&
           (PAGE_START_ALIGN(p_src) == PAGE_START_ALIGN(p_chunk->p_start)) &&
           handle_represents_data(p_src->header.handle))
    {
        /* include p_src in the chunk */
        p_chunk->length += p_src->header.len_words * WORD_SIZE;

        p_src = get_next_entry(p_src);
    }
    return p_src;
}

/*****************************************************************************
* Defrag procedure m_procedure_steps
*****************************************************************************/
 procedure_action_t check_for_invalid_entries(void)
{
    /* If we can't find any invalid entries in this page, we should skip it. */
    if (entry_get(get_first_entry(m_defrag.p_storage_page),
                  m_defrag.p_storage_page + 1,
                  FLASH_MANAGER_HANDLE_INVALID) == NULL)
    {
        if (m_defrag.p_storage_page == get_last_page(m_defrag.p_storage_page))
        {
            return PROCEDURE_END;
        }
        else
        {
            /* Jump to the next page */
            m_defrag.p_storage_page++;
            return PROCEDURE_RESTART;
        }
    }
    else
    {
        return PROCEDURE_CONTINUE;
    }
}

 procedure_action_t erase_recovery_area(void)
{
    if (erase(mp_recovery_area, PAGE_SIZE, &m_flash_token) == PAN_SUCCESS)
    {
        return PROCEDURE_CONTINUE;
    }
    else
    {
        /* retry later */
        return PROCEDURE_STAY;
    }
}

 procedure_action_t copy_metadata(void)
{
    uint32_t metadata_length = m_defrag.p_storage_page->metadata.metadata_len;
    if (flash(&mp_recovery_area->data[0], m_defrag.p_storage_page, metadata_length, &m_flash_token) == PAN_SUCCESS)
    {
        /* ready to start copying entries */
        m_defrag.p_dst = (const fm_entry_t *) &mp_recovery_area->data[metadata_length / sizeof(mp_recovery_area->data[0])];
        m_defrag.p_src = get_first_entry(m_defrag.p_storage_page);
        m_defrag.found_all_entries = false;
        return PROCEDURE_CONTINUE;
    }
    else
    {
        /* retry later */
        return PROCEDURE_STAY;
    }
}

/**
 * Copy valid handles from storage area to backup page.
 */
 procedure_action_t backup_entries(void)
{
    MESH_ASSERT(m_defrag.p_src != NULL);
    MESH_ASSERT(m_defrag.p_dst != NULL);

    bool recovery_page_is_full = false;
    /* Copy entries in chunks of sequential data-entries until we get to the end or are unable to
       fit any more in the recovery page. */
    while (!m_defrag.found_all_entries && !recovery_page_is_full)
    {
        fm_entry_chunk_t chunk;
        uint32_t remaining_space = (PAGE_START_ALIGN(m_defrag.p_dst) + PAGE_SIZE) - (uint32_t) m_defrag.p_dst;
        const fm_entry_t * p_next =
            get_chunk_of_data_entries(&chunk,
                                      m_defrag.p_src,
                                      get_area_end(m_defrag.p_storage_page),
                                      remaining_space);

        if (chunk.length > 0)
        {
            if (PAN_SUCCESS == flash(m_defrag.p_dst, chunk.p_start, chunk.length, &m_flash_token))
            {
                m_defrag.p_dst += (chunk.length / sizeof(fm_entry_t));
                MESH_ASSERT(p_next != NULL);
                m_defrag.p_src = p_next;
            }
            else
            {
                /* Early return */
                return PROCEDURE_STAY;
            }
        }
        else
        {
            recovery_page_is_full = true;
        }
        /* Getting a NULL return from the chunk getter means there were no more valid data chunks left. */
        if (p_next == NULL || p_next->header.handle == HANDLE_SEAL)
        {
            m_defrag.found_all_entries = true;
        }
    }

    return PROCEDURE_CONTINUE;
}


 procedure_action_t write_defrag_start_pointer(void)
{
    if (flash(&mp_recovery_area->p_storage_page, (void *) &m_defrag.p_storage_page, WORD_SIZE, &m_flash_token) == PAN_SUCCESS)
    {
        return PROCEDURE_CONTINUE;
    }
    else
    {
        /* retry later */
        return PROCEDURE_STAY;
    }
}

 procedure_action_t erase_source(void)
{
    if (erase(m_defrag.p_storage_page, PAGE_SIZE, &m_flash_token) == PAN_SUCCESS)
    {
        return PROCEDURE_CONTINUE;
    }
    else
    {
        /* retry later */
        return PROCEDURE_STAY;
    }
}

 procedure_action_t write_back(void)
{
    if (flash(m_defrag.p_storage_page, mp_recovery_area->data, sizeof(mp_recovery_area->data), &m_flash_token) == PAN_SUCCESS)
    {
        /* In the next step, we'll iterate through the memory we write here, so we need this entire
           flash operation to finish before moving on: */
        m_defrag.wait_for_idle = true;
        return PROCEDURE_CONTINUE;
    }
    else
    {
        return PROCEDURE_STAY;
    }
}

/**
 * Write a seal or padding entry at the end of the current storage page.
 */
 procedure_action_t seal_storage_page(void)
{
    /* Put seal at the first blank header: */
    const fm_entry_t * p_seal_location = entry_get(get_first_entry(m_defrag.p_storage_page),
                                                   m_defrag.p_storage_page + 1,
                                                   HANDLE_BLANK);
    if (p_seal_location == NULL || p_seal_location == get_first_entry(m_defrag.p_storage_page))
    {
        /* Don't seal if the target page is either completely full or completely empty. */
        return PROCEDURE_CONTINUE;
    }
    /* If there are more entries in the area, pad the page, else seal it. */
    const fm_header_t * p_header;
    if (m_defrag.found_all_entries)
    {
        p_header = &SEAL_HEADER;
    }
    else
    {
        p_header = &PADDING_HEADER;
    }

    if (flash(p_seal_location, p_header, sizeof(fm_header_t), &m_flash_token) == PAN_SUCCESS)
    {
        m_defrag.wait_for_idle = true;
        return PROCEDURE_CONTINUE;
    }
    else
    {
        return PROCEDURE_STAY;
    }
}

/**
 * Step through the entries on the page AFTER the one we're recovering, and
 * remove any entries that also occur in the backup, as these are duplicates.
 */
 procedure_action_t invalidate_duplicate_entries(void)
{
    if (m_defrag.p_storage_page->metadata.page_index ==
        m_defrag.p_storage_page->metadata.pages_in_area - 1)
    {
        /* The current page was the last page in the area, and there won't be any duplicates. */
        return PROCEDURE_CONTINUE;
    }

    const fm_entry_t * p_area_entry = get_first_entry(m_defrag.p_storage_page + 1);
    const fm_entry_t * p_recovery_entry =
        get_first_entry((const flash_manager_page_t *) mp_recovery_area->data);
    const fm_entry_t * p_end = (const fm_entry_t *) (get_area_end(m_defrag.p_storage_page));

    if (!handle_represents_data(p_area_entry->header.handle))
    {
        p_area_entry = get_next_data_entry(p_area_entry, p_end);
    }

    while (p_area_entry != NULL)
    {
        p_recovery_entry =
            entry_get(p_recovery_entry, mp_recovery_area + 1, p_area_entry->header.handle);
        if (p_recovery_entry == NULL)
        {
            /* Since the entries would have been added to the recovery area in-order, we can assume
               that once we find an entry that's not duplicated in the recovery area, we won't find
               any more duplicates. */
            break;
        }

        /* Wait for all invalidations to finish before moving on to the next step in the procedure,
           as they will alter the way we'll traverse the page on the next round: */
        m_defrag.wait_for_idle = true;

        if (flash(p_area_entry, &INVALID_HEADER, sizeof(INVALID_HEADER), &m_flash_token) !=
            PAN_SUCCESS)
        {
            return PROCEDURE_STAY;
        }

        p_area_entry = get_next_data_entry(p_area_entry, p_end);
    }
    return PROCEDURE_CONTINUE;
}

 procedure_action_t post_process(void)
{
    if (m_defrag.p_storage_page == get_last_page(m_defrag.p_storage_page))
    {
        /* Invalidate area pointer */
         const uint32_t * p_null_ptr = NULL;
        if (flash(&mp_recovery_area->p_storage_page, &p_null_ptr, sizeof(p_null_ptr), &m_flash_token) == PAN_SUCCESS)
        {
            m_defrag.wait_for_idle = true;
            return PROCEDURE_END;
        }
        else
        {
            return PROCEDURE_STAY;
        }
    }
    else
    {
        /** Start the procedure from the beginning, operating on the next page in the area. */
        m_defrag.p_storage_page++;
        return PROCEDURE_RESTART;
    }
}

 const defrag_procedure_step_t m_procedure_steps[] =
{
    check_for_invalid_entries,
    erase_recovery_area,
    copy_metadata,
    backup_entries,
    write_defrag_start_pointer,
    erase_source,
    write_back,
    seal_storage_page,
    invalidate_duplicate_entries,
    post_process
};
/*****************************************************************************
* Static functions
*****************************************************************************/
 void defrag_on_end(void)
{
    const flash_manager_t * p_manager = m_defrag.p_manager;
    memset(&m_defrag, 0, sizeof(m_defrag));
    flash_manager_on_defrag_end((flash_manager_t *) p_manager);
}

 void jump_to_step(defrag_procedure_step_t step)
{
    for (uint32_t i = 0; i < MAX_PROCEDURE_STEP; i++)
    {
        if (m_procedure_steps[i] == step)
        {
            m_defrag.step = i;
            return;
        }
    }
    /* Couldn't find step */
    MESH_ASSERT(false);
}

 void execute_procedure_step(void)
{
    /* Verify the validity of the state */
    MESH_ASSERT(m_defrag.p_storage_page != NULL);
    MESH_ASSERT(!m_defrag.wait_for_idle);
    MESH_ASSERT(m_defrag.step < MAX_PROCEDURE_STEP);
    procedure_action_t action;

    do
    {
        action = m_procedure_steps[m_defrag.step]();
        switch (action)
        {
            case PROCEDURE_STAY:
                break;
            case PROCEDURE_CONTINUE:
                m_defrag.step++;
                MESH_ASSERT(m_defrag.step < MAX_PROCEDURE_STEP);
                break;
            case PROCEDURE_RESTART:
                m_defrag.step = 0;
                break;
            case PROCEDURE_END:
                if (m_defrag.wait_for_idle)
                {
                    m_defrag.state = DEFRAG_STATE_STABILIZING;
                }
                else
                {
                    defrag_on_end();
                }
                break;
        }
    } while ((action == PROCEDURE_CONTINUE || action == PROCEDURE_RESTART) &&
             !m_defrag.wait_for_idle);
}

 void on_flash_op_end(mesh_flash_user_t user, const flash_operation_t * p_op, uint16_t token)
{
    /*lint -esym(715, user, token) Ignore unused tokens */

    if (p_op->type == FLASH_OP_TYPE_ALL)
    {
        m_defrag.wait_for_idle = false;
    }

    if (m_defrag.state == DEFRAG_STATE_PROCESSING)
    {
        if (!m_defrag.wait_for_idle)
        {
            execute_procedure_step();
        }
    }
    else if (m_defrag.state == DEFRAG_STATE_STABILIZING && p_op->type == FLASH_OP_TYPE_ALL)
    {
        defrag_on_end();
    }
}

/**
 * Checks if a defrag was interrupted by a power cycle, and continues where it left off.
 *
 * @returns Whether there's a defrag in progress.
 */
 bool recover_defrag_progress(void)
{
    if (mp_recovery_area->p_storage_page != NULL &&
        mp_recovery_area->p_storage_page != (void *) BLANK_FLASH_WORD &&
        IS_PAGE_ALIGNED(mp_recovery_area->p_storage_page))
    {
        m_defrag.p_storage_page = mp_recovery_area->p_storage_page;
        m_defrag.wait_for_idle = false;
        m_defrag.found_all_entries = false;
        m_defrag.state = DEFRAG_STATE_PROCESSING;
        m_defrag.p_manager = NULL; /* Can't know which manager this is. */
        jump_to_step(DEFRAG_RECOVER_STEP);
        mesh_flash_user_callback_set(FLASH_MANAGER_FLASH_USER, on_flash_op_end);
        execute_procedure_step();
        return true;
    }
    else
    {
        return false;
    }
}

/*****************************************************************************
* Interface functions
*****************************************************************************/

bool flash_manager_defrag_init(void)
{
    flash_manager_recovery_area_t * p_flash_end;
    
    p_flash_end = (flash_manager_recovery_area_t *)DEVICE_FLASH_END_GET() ;
    
    mp_recovery_area = p_flash_end - FLASH_MANAGER_RECOVERY_PAGE_OFFSET_PAGES - 1; /* pointer arithmetic */

	memset((uint8_t*)&m_defrag, 0, sizeof(m_defrag));
	m_flash_token = 0;
	
	printf("FLASH_MANAGER_DEFRAG:%08x,SIZE:%d\n",(uint32_t)mp_recovery_area, 1*PAGE_SIZE);

    return recover_defrag_progress();
}

bool flash_manager_defragging(const flash_manager_t * p_manager)
{
    MESH_ASSERT(p_manager != NULL);
    return (m_defrag.state != DEFRAG_STATE_IDLE &&
            m_defrag.p_storage_page >= get_first_page(p_manager->config.p_area) &&
            m_defrag.p_storage_page <= get_last_page(p_manager->config.p_area));
}

bool flash_manager_defrag_is_running(void)
{
    return (m_defrag.state != DEFRAG_STATE_IDLE);
}

void flash_manager_defrag(const flash_manager_t * p_manager)
{
    MESH_ASSERT(m_defrag.state == DEFRAG_STATE_IDLE);
    MESH_ASSERT(p_manager->internal.state == FM_STATE_DEFRAG);

    m_defrag.p_manager = p_manager;
    m_defrag.p_storage_page = p_manager->config.p_area;
    m_defrag.step = 0;
    m_defrag.wait_for_idle = false;
    m_defrag.state = DEFRAG_STATE_PROCESSING;
    m_defrag.found_all_entries = false;

    mesh_flash_user_callback_set(FLASH_MANAGER_FLASH_USER, on_flash_op_end);

    execute_procedure_step();
    __INTERNAL_EVENT_PUSH(INTERNAL_EVENT_FM_DEFRAG, 0, 0, NULL);
}

const void * flash_manager_defrag_recovery_page_get(void)
{
    return mp_recovery_area;
}

#ifdef UNIT_TEST
void flash_manager_defrag_reset(void)
{
    memset((uint8_t*)&m_defrag, 0, sizeof(m_defrag));
    mp_recovery_area = NULL;
    m_flash_token = 0;
}
#endif
