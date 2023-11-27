#include <stdio.h>
#include <string.h>
#include "PN102Series.h"
#include "panip_config.h"
#include "peripherals.h"
#include "panip.h"
#include "arch.h"
#include "app_task.h"
#include "app.h"
#include "ke_env.h"
#include "rf.h"
#include "gattm.h"
#include "hci_uart.h"
#include "hci_uart_task.h"
#include "app_temp_adc.h"
#include "dbg_sys.h"
#include "stack_svc_api.h"

#include "mcu_hal.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
#if(EXT_WAKEUP)
uint8_t wakeup_cnt;				//���߻��ѵļ�����unit�����Ӽ����㲥�����
#endif
uint8_t sys_power_flag;			//Ϊ1,�ر�32k,ϵͳ��ȫ���ߡ�Ϊ���������ر�
uint8_t sys_sleep_flag;			//Ϊ1,ϵͳ����͹��ġ�Ϊ0��ϵͳ���ѡ�
uint8_t sys_ble_conn_flag;      //1,connect;0,disconnect
bool ble_has_been_connected;		// TRUE: �ѱ����ӹ� FALSE: δ�����ӹ�
uint16_t USART_RX_CNT;
uint8_t  USART_RX_BUF[USART_REC_LEN];

app_var_t app_var __attribute__((at(APP_VAR_ADDR)));
/*---------------------------------------------------------------------------------------------------------*/
/*
 * STRUCT DEFINITIONS
 *****************************************************************************************
 */
#if(SPROM_DEC)
typedef void (FUNC_PTR)(void);
#endif

 
void Set_Device_MacAddr(void)
{
	uint8_t addrvalue[6] = {0};
	
	SYS_UnlockReg();
	FMC_Open();
	FMC->ISPCMD = FMC_ISPCMD_READ_UID;
	FMC->ISPADDR = 0x50;
	FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
	while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) ;
	addrvalue[0] = FMC->ISPDAT & 0xff;
	addrvalue[1] = FMC->ISPDAT >> 8;
	addrvalue[2] = FMC->ISPDAT >> 16;
	addrvalue[3] = FMC->ISPDAT >> 24;
	
	FMC->ISPCMD = FMC_ISPCMD_READ_UID;
	FMC->ISPADDR = 0x54;
	FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
	while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) ;
	addrvalue[4] = FMC->ISPDAT & 0xff;
	addrvalue[5] = FMC->ISPDAT >> 8;
	
	FMC_Close();
	SYS_LockReg();
	memcpy(app_var.co_default_bdaddr.addr,addrvalue,BD_ADDR_LEN);
} 
 
void UART0_Handler(void)
{
	uint32_t u32IntSts=UART_GetActiveEvent(UART0);
	
	if(u32IntSts & Uart_event_data) 
	{
		/* Get all the input characters */
		while(!UART_IsRxFifoEmpty(UART0)) 
		{
			/* Get the character from UART Buffer */
			USART_RX_BUF[USART_RX_CNT++] = UART_ReceiveData(UART0);
			if((USART_RX_BUF[USART_RX_CNT - 1] == '\n') || 
			   (USART_RX_BUF[USART_RX_CNT - 2] == '\r') ||
			   (USART_RX_CNT >= USART_REC_LEN))
			{
				if((USART_RX_CNT >1) && (sys_ble_conn_flag == 1))
				{
					app_proj_template_send_value(PROJ_TEMPLATE_IDX_S2C_VAL,USART_RX_BUF,USART_RX_CNT);
				}
				USART_RX_CNT = 0;
			}
		}
	}
}

void sys_clear_global_var(void)
{
	memset((uint8_t *)&app_env,0,sizeof( struct app_env_tag));
	memset((uint8_t *)&gattm_env,0,sizeof( struct gattm_env_tag));
	memset((uint8_t *)&hci_env,0,sizeof( struct hci_env_tag));
	memset((uint8_t *)&prf_env,0,sizeof( struct prf_env_tag));
	
	memset(appm_state,0,sizeof(appm_state));
	memset(gattc_env,0,sizeof(gattc_env));
	memset(gattc_state,0,sizeof(gattc_state));
	memset(gattm_state,0,sizeof(gattm_state));
	((stack_clear_global_var_handler)SVC_stack_clear_global_var)();
	app_fun_resgister();
	#if(TEMP_CHANGE_CALIB)
	adc_temp_param_init();
	#endif
	
	#if(SET_CUSTOME_ADDR)
	uint8_t addrvalue[6] = {0x03, 0x00, 0x20, 0x11, 0xCA, 0xBB};
	memcpy(app_var.co_default_bdaddr.addr,addrvalue,BD_ADDR_LEN);
	#else
	Set_Device_MacAddr();
	#endif
	
	#if(EXT_WAKEUP)
	wakeup_cnt = 0;
	#endif
	memset(USART_RX_BUF,0,USART_REC_LEN);
	USART_RX_CNT = 0;
	sys_power_flag = 0;
	sys_sleep_flag = 0;
	sys_ble_conn_flag = 0;
	ble_has_been_connected = FALSE;
	app_var.Wakeup_int = 0;
	app_var.rf_close_en = RF_CLOSE_EN;	
	app_var.Gpio_retain_en = GPIO_RETAIN_EN;
	app_var.default_sleep_en = SLEEP_EN;
	app_var.default_tx_power = TX_POWER_0DBM;
	app_var.ext_wakeup_enable = EXT_WAKEUP;
	app_var.RF_mode_select = XN297_MODE_EN;
}

void app_fun_resgister(void)
{
	((appm_init_register_handler)SVC_appm_init_register)(appm_init);
	((hci_init_register_handler)SVC_hci_init_register)(hci_init);
	((hci_send_2_controller_register_handler)SVC_hci_send_2_controller_register)(hci_send_2_controller);
	((attm_svc_create_db_register_handler)SVC_attm_svc_create_db_register)(attm_svc_create_db);
	((gattc_con_enable_register_handler)SVC_gattc_con_enable_register)(gattc_con_enable);
	((gattm_cleanup_register_handler)SVC_gattm_cleanup_register)(gattm_cleanup);
	((gattm_create_register_handler)SVC_gattm_create_register)(gattm_create);
	((prf_cleanup_register_handler)SVC_prf_cleanup_register)(prf_cleanup);
	((prf_create_register_handler)SVC_prf_create_register)(prf_create);
	((prf_get_id_from_task_register_handler)SVC_prf_get_id_from_task_register)(prf_get_id_from_task);
	((prf_get_task_from_id_register_handler)SVC_prf_get_task_from_id_register)(prf_get_task_from_id);
	((prf_init_register_handler)SVC_prf_init_register)(prf_init);
	((attm_att_update_perm_register_handler)SVC_attm_att_update_perm_register)(attm_att_update_perm);
	((gattm_init_attr_register_handler)SVC_gattm_init_attr_register)(gattm_init_attr);
	((hci_basic_cmd_send_2_controller_register_handler)SVC_hci_basic_cmd_send_2_controller_register)(hci_basic_cmd_send_2_controller);
	((prf_add_profile_register_handler)SVC_prf_add_profile_register)(prf_add_profile);
	((gattc_get_mtu_register_handler)SVC_gattc_get_mtu_register)(gattc_get_mtu);
	((attm_init_register_handler)SVC_attm_init_register)(attm_init);
	((gattm_init_register_handler)SVC_gattm_init_register)(gattm_init);
	((hci_send_2_host_register_handler)SVC_hci_send_2_host_register)(hci_send_2_host);
	((attmdb_destroy_register_handler)SVC_attmdb_destroy_register)(attmdb_destroy);
	((sleep_handler_register)SVC_sleep_handler_register)(sleep_handler);
	((ble_event_handler_register)SVC_ble_event_handler_register)(ble_event_handler);
	((rf_init_handler_register)SVC_rf_init_handler_register)(rf_init_handler);
	((ble_rx_handler_register)SVC_ble_rx_handler_register)(ble_rx_handler);
	((rf_dev_cal_init_handler_register)SVC_rf_dev_cal_init_handler_register)(rf_dev_cal_init_handler);
}

//void SysTick_IntHandler(void)
//{
//	printf("ss\n");
//}

//void SysTick_init(void)
//{
//	CLK_SysTick_Config(100*1000);
//	
//	NVIC_SetPriority(SysTick_IRQn,3);
//	NVIC_ClearPendingIRQ ( SysTick_IRQn );
//	NVIC_EnableIRQ ( SysTick_IRQn );
//	
//	((interrupt_register_handler)SVC_interrupt_register)(SysTick_IRQ,SysTick_IntHandler);
//}

void periph_init(void)
{		
	/*---------------------------------------------------------------------------------------------------------*/
	/* Init System Clock                                                                                       */
	/*---------------------------------------------------------------------------------------------------------*/
	/* Unlock protected registers */
	SYS_UnlockReg();
	//  CLK_InitHXTPLL();
	CLK->WAKEUPCTL = 0x08100010;
	
	if(app_info.default_use_ext_32k)
		SYS->P0_MFP |= SYS_MFP_P05_32K|SYS_MFP_P06_32K;
	
	#if(EXT_WAKEUP)
	//P52���ó�GPIO;
	SYS->P5_MFP |= SYS_MFP_P52_GPIO;
	GPIO_SetMode(P5, BIT2, GPIO_MODE_INPUT);
	GPIO_PullUp(P5, BIT2,GPIO_PULLUP_DISABLE);
	#endif
	
	CLK_EnableModuleClock(TMR2_MODULE);
	CLK_EnableModuleClock(ANAC_MODULE);
	CLK_EnableModuleClock(MDM_MODULE);
	
	CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_DPLL_26M);
	
	#if(SPROM_DEC)
	((FUNC_PTR *)(FMC_SPROM_BASE+1))();				//SPROM�ĳ�����"03_MCU\PN1020-peripheral\SampleCode\StdDriver\FMC\SPROM_project\Keil"Ŀ¼��
	#endif
	
	/* Update System Core Clock */
	SystemCoreClockUpdate();

	/* Lock protected registers */
	SYS_LockReg();
	/*---------------------------------------------------------------------------------------------------------*/
	/* Init I/O Multi-function                                                                                 */
	/*---------------------------------------------------------------------------------------------------------*/
	/* Set P0 multi-function pins for UART0 RXD, TXD */
#if (ENABLE_UART0 && (XN297_MODE_EN == 0))
	UART_InitTypeDef Init_Struct;

	SYS->P1_MFP |= SYS_MFP_P12_UART0_RXD|SYS_MFP_P13_UART0_TXD;   	
	GPIO_ENABLE_DIGITAL_PATH(P1,(1<<2));
	CLK_EnableModuleClock(UART0_MODULE);
	
	#if(EXT_WAKEUP)
	Init_Struct.UART_BaudRate = 921600;
	#else
	Init_Struct.UART_BaudRate = 115200;
	#endif
	Init_Struct.UART_LineCtrl = Uart_line_8n1;

	/* Init UART0 for printf */
	UART_Init ( UART0, &Init_Struct );
	
	UART_ResetRxFifo ( UART0 );
	UART_EnableFifo ( UART0 );

	UART_EnableIrq ( UART0, Uart_irq_erbfi );
	
	NVIC_ClearPendingIRQ ( UART0_IRQn );
	NVIC_EnableIRQ ( UART0_IRQn );
	((interrupt_register_handler)SVC_interrupt_register)(UART0_IRQ,UART0_Handler);		//register uart0 interrupt callback function
#else	
	hci_uart_init();
	hci_uart_task_init();
#endif

//	SysTick_init(); 
}

//�ر�32k
void rc32k_disable(void)
{
	ANAC->ANAC_3VCTL |= (1 << 13);
		
	ANAC->ANAC_3VCTL |= 0x01;
	while(ANAC->ANAC_3VCTL & 0x01);
}

//��32k
void rc32k_enable(void)
{
	ANAC->ANAC_3VCTL &= ~(1 << 13);
		
	ANAC->ANAC_3VCTL |= 0x01;
	while(ANAC->ANAC_3VCTL & 0x01);
}

void sleep_ldo_change(void)
{
	uint8_t ldo_value[4];
	
	SYS_UnlockReg();
	FMC_Open();
	FMC->ISPCMD = FMC_ISPCMD_READ_UID;
	FMC->ISPADDR = 0x24;
	FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
	while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) ;
	ldo_value[0] = FMC->ISPDAT & 0xff;
	ldo_value[1] = FMC->ISPDAT >> 8;
	ldo_value[2] = FMC->ISPDAT >> 16;
	ldo_value[3] = FMC->ISPDAT >> 24;

	FMC_Close();
	SYS_LockReg();
	
	if(ldo_value[3] == ((ldo_value[0] + ldo_value[1] + ldo_value[2]) & 0xff))
	{
		#define LDO0P7_MASK (0x7<<24)
		ANAC->LDO_CTL &= ~LDO0P7_MASK;
		ANAC->LDO_CTL |= (ldo_value[0] & 0x7)<<24;
		
		ANAC->ANAC_3VCTL |= 0x01;
		while(ANAC->ANAC_3VCTL & 0x01);
	}
}

//����͹��Ļ���ȫ����ǰ�Ĵ���
void sleep_handler(void)
{
	if(sys_power_flag == 1)
	{
		//P52���ó��ⲿ����
		SYS->P5_MFP |= SYS_MFP_P52_EXT_WAKEUP;
		GPIO_SetMode(P5, BIT2, GPIO_MODE_INPUT);
		GPIO_PullUp(P5, BIT2,GPIO_PULLUP_DISABLE);
		//�ر�32k��ֻ��P52�ⲿ����
		rc32k_disable();				
		sys_power_flag = 0;
	}
	else
	{
		rc32k_enable();
	}
	
	if(app_var.ext_wakeup_enable == 0)
	{
		//��ʹ���ⲿ����
		ble_extwkupdsb_setf(1);			
	}
	else
	{
		//ʹ���ⲿ����
		ble_extwkupdsb_setf(0);			
	}
	sleep_ldo_change();
	sys_sleep_flag = 1;
}

void rf_init_handler(void)
{
	
}

void ble_rx_handler(void)
{
	
}

void ble_event_handler(void)
{
	
}

//����ʽУ���Ĵ���
void rf_dev_cal_init_handler(void)
{
	uint8_t r_da_gain,r_da_verf_lb,r_da_verf_mb,r_guass_scale;
	
	((rf_freq_dev_cal_result)SVC_rf_freq_dev_cal_result)(&r_da_gain,&r_da_verf_lb,&r_da_verf_mb,&r_guass_scale);		//����ʽУ����Ľ��

	//clear [14:20]
	ANAC->TX_CTL &= ~ 0x1fc000;

	//set DA_GAIN, DA_VREF_LB, DA_VREF_MB
	ANAC->TX_CTL |= (r_da_gain << 14) | (r_da_verf_lb << 15) | (r_da_verf_mb << 18);

	ANAC->TP_CTL &= ~ 0xf;
	ANAC->TP_CTL |= r_guass_scale;
	
	//disable 2 piont calibration
	ANAC->TP_CTL = ANAC->TP_CTL & 0xfffffE7f;
	
	set_ui32_reg(ANAC->MCU_RF, 0x00000100);
	set_ui32_reg(ANAC->AGC_CTL, 0x00000fff);
}
