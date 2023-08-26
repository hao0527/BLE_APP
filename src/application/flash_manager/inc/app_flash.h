#ifndef APP_FLASH_H__
#define APP_FLASH_H__

#include <stdint.h>
#include <stdbool.h>

extern void flash_module_init(void);
extern void add_flash_manager(void);
extern void flash_write_uint32(uint32_t val);
extern void flash_read_uint32(void);
extern void app_flash_init(void);

extern void dev_cal_flash_write(uint8_t index,uint32_t val);
extern uint8_t dev_cal_flash_read(uint8_t index,uint32_t *val);

#endif /* APP__FLASH_H__ */
