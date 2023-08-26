#ifndef FLASH_CONFIG__C
#define FLASH_CONFIG__C


//-------- <<< Use Configuration Wizard in Context Menu >>> --------------------

//<h> flash physical config 
    //<o> flash store entry address
        #define PAN_FLASH_ENTRY_ADDRESS  (0x3C000)
        /** pan data RAM start address. */
        #define DATA_RAM_START   (0x20000000)
        /** First address outside the data RAM */
        #define DEVICE_DATA_RAM_END_GET() (DATA_RAM_START + 1024 * 16)
       
   //<o> flash store size
        #define PAN_FLASH_SIZE    (0x1000)
        #define DEVICE_FLASH_END_GET()    (PAN_FLASH_SIZE + PAN_FLASH_ENTRY_ADDRESS)
   //<o> flash page size
        #define PAGE_SIZE        (0x200)
//</h>



/** Check if an address is a valid RAM address. */
#define IS_VALID_RAM_ADDR(ADDR)     (((uint32_t)(ADDR) > DATA_RAM_START) && ((uint32_t)(ADDR) < DEVICE_DATA_RAM_END_GET())) 
       
       
     
/** Check whether the given pointer is page aligned. */
#define IS_PAGE_ALIGNED(p) (((uint32_t)(p) & (PAGE_SIZE - 1)) == 0)
/** Check whether the given pointer is word aligned. */
#define IS_WORD_ALIGNED(p) (((uint32_t)(p) & (WORD_SIZE - 1)) == 0)


//<h> flash function config 
/**
 * @defgroup MESH_CONFIG_FLASH_MANAGER Flash manager configuration defines
 * @{
 */

/** Maximum number of pages that can be owned by a single flash manager */
#ifndef FLASH_MANAGER_PAGE_COUNT_MAX
#define FLASH_MANAGER_PAGE_COUNT_MAX 255
#endif

/** Size of the flash manager data pool, storing pending writes. */
//<o> RAM hold flash data size 
#ifndef FLASH_MANAGER_POOL_SIZE
#define FLASH_MANAGER_POOL_SIZE 128
#endif

/** Maximum size of a single flash entry in bytes. */
#ifndef FLASH_MANAGER_ENTRY_MAX_SIZE
#define FLASH_MANAGER_ENTRY_MAX_SIZE 128
#endif

/** Number of flash pages to be reserved between the flash manager recovery page and the bootloader.
 *  @note This value will be ignored if FLASH_MANAGER_RECOVERY_PAGE is set.
 */
#ifndef FLASH_MANAGER_RECOVERY_PAGE_OFFSET_PAGES
#define FLASH_MANAGER_RECOVERY_PAGE_OFFSET_PAGES 0
#endif

#ifndef BEARER_EVENT_FLAG_COUNT
#define BEARER_EVENT_FLAG_COUNT     12
#endif

#define BEARER_EVENT_PRIORITY 1

/** @} end of MESH_CONFIG_FLASH_MANAGER */
//</h>
// <<< end of configuration section >>>
#endif
