#include <stdint.h>
#include "gapm_int.h"
#include "gattm_int.h"
#include "l2cm_int.h"
#include "hci_int.h"
#include "lld_evt.h"
#include "llm.h"
#include "prf.h"
#include "app_task.h"
#include "gapc_int.h"
#include "gapm_int.h"
#include "gattc_int.h"
#include "l2cc_int.h"
#include "llm_task.h"
#include "lld_sleep.h"
#include "app_proj_template.h"
#include "proj_template_server.h"
#include "proj_template_server_task.h"

#ifndef PERIPHERAL_H_INCLUDED
#define PERIPHERAL_H_INCLUDED

#define USART_REC_LEN  			PROJ_TEMPLATE_SERVER_PACKET_SIZE

typedef	enum
{
	WDT_IRQ,           
	EINT0_IRQ,          
	EINT1_IRQ,         
	GPIO01_IRQ,         
	GPIO234_IRQ,        
	PWM_IRQ,            
	TMR0_IRQ,          
	TMR1_IRQ,          
	EINT2_IRQ,          
	UART0_IRQ,          
	UART1_IRQ,       
	SPI0_IRQ,        
	SPI1_IRQ,    
	I2C0_IRQ,         
	I2C1_IRQ,          
	ADC_IRQ,
	DMA_IRQ,
	SysTick_IRQ,
}tIRQ_TYPE;

#if(EXT_WAKEUP)
extern uint8_t wakeup_cnt;
#endif
extern uint8_t sys_power_flag;
extern uint8_t sys_sleep_flag;
extern uint8_t sys_ble_conn_flag;
extern uint16_t USART_RX_CNT;
extern uint8_t  USART_RX_BUF[USART_REC_LEN];

extern struct app_env_tag app_env;
extern struct gattm_env_tag gattm_env;
extern struct hci_env_tag hci_env;
extern struct prf_env_tag prf_env;    
extern struct gattc_env_tag* gattc_env[GATTC_IDX_MAX];

extern ke_state_t appm_state[APP_IDX_MAX];
extern ke_state_t gattc_state[GATTC_IDX_MAX];
extern ke_state_t gattm_state[GATTM_IDX_MAX];

extern void hci_uart_write(uint8_t *bufptr, uint32_t size, void (*callback) (uint8_t));
extern void hci_uart_read(uint8_t *bufptr, uint32_t size, void (*callback) (uint8_t));
extern void periph_init(void) ;
extern void sys_clear_global_var(void);
extern void app_fun_resgister(void);
extern void sleep_handler(void);
extern void rf_init_handler(void);
extern void ble_rx_handler(void);
extern void ble_event_handler(void);
extern void rf_dev_cal_init_handler(void);

#include "rf.h"
#include "co_bt.h"

#define APP_INFO_ADDR			0x00016800
#define APP_VAR_ADDR 			0x20003800

//app中的宏
typedef struct __app_info_t
{
	/// Default Device Name
	const char *co_default_bdname;

	uint8_t default_use_ext_32k;
	uint8_t default_hz32000;
	uint8_t default_en_calib_rc;
	uint8_t default_calib_slot;
	uint32_t default_calib_32k_int;
}app_info_t;
extern const app_info_t app_info __attribute__((at(APP_INFO_ADDR)));

//app中的变量
typedef struct __app_var_t
{
	uint8_t default_tx_power;
	uint8_t default_sleep_en:1;	
	uint8_t ext_wakeup_enable:1;
	uint8_t RF_mode_select:1;
	uint8_t Gpio_retain_en:1;
	
	uint8_t rf_close_en:1;
	uint16_t Wakeup_int;	//rf关闭后自定义唤醒间隔，unit 625us
	
	 /// Default BD address,device mac addr
	struct bd_addr co_default_bdaddr;
}app_var_t;
extern app_var_t app_var __attribute__((at(APP_VAR_ADDR)));
#endif

