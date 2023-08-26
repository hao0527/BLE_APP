#include "app_flash.h"
#include "mesh_flash.h"
#include "flash_manager.h"
#include "flash_manager_defrag.h"

#define TEST_FLASH_PAGE_COUNT  (2)
#define TEST_FLASH_VAL_HANDLE  (0x0001)

/* The flash manager instance used by this module. */
flash_manager_t m_flash_manager;
fm_mem_listener_t flash_add_mem_available_struct;
fm_mem_listener_t mem_listener;

void flash_module_init(void)
{
    SYS_UnlockReg();
    FMC_ENABLE_ISP();
    FMC_ENABLE_AP_UPDATE();
}


/**callback functions for flash manager init**/
typedef void (*flash_op_func_t) (void);
 void flash_manager_mem_available(void * p_args)
{
    ((flash_op_func_t) p_args)(); /*lint !e611 Suspicious cast */
}

 void flash_remove_complete(const flash_manager_t * p_manager)
{
    add_flash_manager();
}

 void flash_write_complete(const flash_manager_t * p_manager,
                                 const fm_entry_t * p_entry,
                                 fm_result_t result)
{
    if (result == FM_RESULT_SUCCESS)
    {
        printf("flash_write_complete success\n");
    }
    else
    {
        printf("flash_write_complete failed\n");
    }
}

 void flash_invalidate_complete(const flash_manager_t * p_manager, fm_handle_t handle, fm_result_t result)
{
    /* Expect no invalidate complete calls. */
    MESH_ASSERT(false);
}

void add_flash_manager(void)
{
	memset(&m_flash_manager,0,sizeof(flash_manager_t));
	
	flash_add_mem_available_struct.callback = flash_manager_mem_available;
	flash_add_mem_available_struct.p_args = add_flash_manager;
	memset(&flash_add_mem_available_struct.queue_elem,0,sizeof(queue_elem_t));
	
	mem_listener.callback = flash_manager_mem_available;
	mem_listener.p_args = flash_write_uint32;
	memset(&mem_listener.queue_elem,0,sizeof(queue_elem_t));
	
    flash_manager_config_t manager_config;
    manager_config.write_complete_cb = flash_write_complete;
    manager_config.invalidate_complete_cb = flash_invalidate_complete;
    manager_config.remove_complete_cb = flash_remove_complete;
    manager_config.min_available_space = WORD_SIZE;
    manager_config.p_area = (const flash_manager_page_t *) ((const uint8_t *) flash_manager_defrag_recovery_page_get() - (TEST_FLASH_PAGE_COUNT * PAGE_SIZE));
    manager_config.page_count = TEST_FLASH_PAGE_COUNT;
    uint32_t status = flash_manager_add(&m_flash_manager, &manager_config);

    if (PAN_SUCCESS != status)
    {
        flash_manager_mem_listener_register(&flash_add_mem_available_struct);
    }
}

/**demo that write a uint32 value**/
void flash_write_uint32(uint32_t val)
{
    fm_entry_t * p_new_entry = flash_manager_entry_alloc(&m_flash_manager, TEST_FLASH_VAL_HANDLE, sizeof(uint32_t));
    if (p_new_entry == NULL)
    {
        flash_manager_mem_listener_register(&mem_listener);
    }
    else
    {
        uint32_t* p_val = (uint32_t *) p_new_entry->data;
        *p_val = val;
        flash_manager_entry_commit(p_new_entry);
    }
}

/**demo that read a uint32 value**/
void flash_read_uint32(void)
{
    const uint32_t * p_val = NULL;

    const fm_entry_t * p_entry = flash_manager_entry_next_get(&m_flash_manager, NULL, NULL);

    while (p_entry != NULL)
    {
        switch (p_entry->header.handle)
        {
            case TEST_FLASH_VAL_HANDLE:
                p_val = (const uint32_t *) p_entry->data;
                printf("flash_read_uint32:%d\n",*p_val);
                break;
            default:
                break;
        }
        p_entry = flash_manager_entry_next_get(&m_flash_manager, NULL, p_entry);
    }
}

void app_flash_init(void)
{
	flash_module_init();
    bearer_event_init(BEARER_EVENT_PRIORITY);
    mesh_flash_init();
    flash_manager_init();

    printf("Flash Manager Module Init\n");
    
    /***app flash manager init***/
    add_flash_manager();
    printf("App Flash Manager Init\n");
	
//	/**write a uint32 value in flash**/
//    flash_write_uint32(100);
//    /**read a uint32 value from flash**/
//    flash_read_uint32();
}

/**dev cal write value**/
void dev_cal_flash_write(uint8_t index,uint32_t val)
{
    fm_entry_t * p_new_entry = flash_manager_entry_alloc(&m_flash_manager, index, sizeof(uint32_t));
    if (p_new_entry == NULL)
    {
        flash_manager_mem_listener_register(&mem_listener);
    }
    else
    {
        uint32_t* p_val = (uint32_t *) p_new_entry->data;
        *p_val = val;
        flash_manager_entry_commit(p_new_entry);
    }
}

/**dev cal read value**/
uint8_t dev_cal_flash_read(uint8_t index,uint32_t *val)
{
    const fm_entry_t * p_entry = flash_manager_entry_next_get(&m_flash_manager, NULL, NULL);

    while (p_entry != NULL)
    {
		if(p_entry->header.handle == index)
		{
			*val = *(uint32_t *) p_entry->data;
            //printf("flash_read_uint32:%d\n",val);
			
			return PAN_SUCCESS;
		}
        p_entry = flash_manager_entry_next_get(&m_flash_manager, NULL, p_entry);
    }
	return PAN_ERROR_NOT_FOUND;
}

