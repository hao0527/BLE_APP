

#ifndef _ARCH_H_
#define _ARCH_H_

/**
 ****************************************************************************************
 * @defgroup REFIP
 * @brief Reference IP Platform
 *
 * This module contains reference platform components - REFIP.
 *
 *
 * @{
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @defgroup DRIVERS
 * @ingroup REFIP
 * @brief Reference IP Platform Drivers
 *
 * This module contains the necessary drivers to run the platform with the
 * RW BT SW protocol stack.
 *
 * This has the declaration of the platform architecture API.
 *
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stdio.h> 
#include <stdint.h>        // standard integer definition
#include "compiler.h"      // inline functions
#include "PN102Series.h"
#include "panip_config.h"

//#define	CFG_FPGA
//#define	JY_HCI_UART1




/*
 * CPU WORD SIZE
 ****************************************************************************************
 */
/// ARM is a 32-bit CPU
#define CPU_WORD_SIZE   4

/*
 * CPU Endianness
 ****************************************************************************************
 */
/// ARM is little endian
#define CPU_LE          1



/*
 * DEBUG configuration
 ****************************************************************************************
 */
#define DEVELOPMENT_DEBUG 0
#if defined(CFG_DBG)
#define PLF_DEBUG          1
#else //CFG_DBG
#define PLF_DEBUG          0
#endif //CFG_DBG


#if 0
#define ARCH_INT_FRAME_TIME         150
#define ARCH_SCAN_REQ_DUR           150
#define ARCH_SCAN_RSP_MIN_DUR       130
#define ARCH_ADV_MIN_DUR            150
#define ARCH_ADV_INT_DIRECT         1250
#define ARCH_ADV_INT_UNDIRECT       1500
#define ARCH_ADV_INT_NONCONNECTABLE 500

//#ifdef __DA14581__
//#define ARCH_ADV_INT_CONNECTABLE    1250
//#define ARCH_ADV_INT_NONCONNECTABLE 500
//#endif
#endif

typedef enum
{
	ARCH_SLEEP_OFF,
	ARCH_EXT_SLEEP_ON,
	ARCH_DEEP_SLEEP_ON,
} sleep_state_t;

/// Arch Sleep environment structure
struct arch_sleep_env_tag
{
 	sleep_state_t slp_state;
};

typedef enum
{
	mode_active = 0,
	mode_idle,
	mode_ext_sleep,
	mode_deep_sleep,
	mode_sleeping,
}sleep_mode_t;





/*
 * DEFINES
 ****************************************************************************************
 */
#if 0

#if defined (CFG_EXT_SLEEP)
	#define EXT_SLEEP_ENABLED							1
#else
	#define EXT_SLEEP_ENABLED 							0
#endif /* CFG_EXT_SLEEP */

#if defined (CFG_DEEP_SLEEP)
	#if defined (CFG_EXT_SLEEP)
		#error "CFG_DEEP_SLEEP defined with CFG_EXT_SLEEP! Select one of them"
	#endif
	#define DEEP_SLEEP_ENABLED						    1
#else
	#define DEEP_SLEEP_ENABLED						    0
#endif


#if defined (CFG_DISABLE_RTS_CTS)
	#define DISABLE_UART_RTS_CTS					    1
#else
	#define DISABLE_UART_RTS_CTS 					    0
#endif /* CFG_DISABLE_RTS_CTS */

#if defined (CFG_LUT_PATCH)
	#define LUT_PATCH_ENABLED   					    1
#else
	#define LUT_PATCH_ENABLED   					    0
#endif /* CFG_LUT_PATCH */

#if BLE_APP_PRESENT
# if !defined(CFG_PRINTF)
#  undef PROGRAM_ENABLE_UART
# else
#  define PROGRAM_ENABLE_UART
# endif
#endif // BLE_APP_PRESENT
    
// #if  LUT_PATCH_ENABLED
// #define PROGRAM_ALTERNATE_UART_PINS                     1 // ES4 cannot use P0_0, P0_1, P0_2, P0_3. Alternate ports must be used for UART iface.
// #endif 
    
#define MGCKMODA_PATCH_ENABLED                          1

// Control the operational frequency of the BLE core
#define BLE_CORE_CLK_AT_8MHz		                    0 //0: 16MHz, 1: 8MHz 

/*
 * Duration from Wakeup to XTAL trim finish. Measured in low power clock cycles.
 ****************************************************************************************
 */

#define XTAL_TRIMMING_TIME_USEC     (4609) // 151 * (1/32768)
#define XTAL_TRIMMING_TIME          (151) 
#define XTAL_TRIMMING_TIME_RCX      (55) 


/*
 * Minimum and maximum LP clock periods. Measured in usec.
 ****************************************************************************************
 */
#define XTAL32_PERIOD_MIN           (30)    // 30.5usec
#define XTAL32_PERIOD_MAX           (31)
#define RCX_PERIOD_MIN              (85)    // normal range: 91.5 - 93usec
#define RCX_PERIOD_MAX              (100)
#endif


/*
 * DEEP SLEEP: Power down configuration
 ****************************************************************************************
 */
#define TWIRQ_RESET_VALUE           (1)                     
//#define TWIRQ_SET_VALUE			    (XTAL_TRIMMING_TIME)

#define TWEXT_VALUE_RCX             (TWIRQ_RESET_VALUE + 2) // ~190usec (LP ISR processing time) / 85usec (LP clk period) = 2.23 => 3 LP cycles

#define TWEXT_VALUE_XTAL32		    (TWIRQ_RESET_VALUE + 6) // ~190usec (LP ISR processing time) / 30usec (LP clk period) = 6.3 => 7 LP cycles

// The times below have been measured for RCX. For XTAL32 will be less. But this won't make much difference...
#define SLP_PROC_TIME               (60)    // 60 usec for SLP ISR to request Clock compensation
#define SLEEP_PROC_TIME             (30)    // 30 usec for rwip_sleep() to program Sleep time and drive DEEP_SLEEP_ON

// Change this if the application needs to increase the sleep delay limit from 9375usec that is now.
#define APP_SLEEP_DELAY_OFFSET      (0)

// Used to eliminated additional delays in the sleep duration
#define SLEEP_DURATION_CORR         (4)


/*
 * External wake up declarations and includes
 ****************************************************************************************
 */
#if ((EXTERNAL_WAKEUP) && (!BLE_APP_PRESENT)) // external wake up, only in full embedded designs
//#include "wkupct_quadec.h"
//#include "gpio.h"
void ext_wakeup_enable(uint32_t port, uint32_t pin, uint8_t polarity);
void ext_wakeup_disable(void);
void ext_wakeup_cb(void);
#endif

#define		DEEP_SLEEP_THRESHOLD    800000
		
		
#if 0		
/*
 * Fab Calibration configuration
 ****************************************************************************************
 */
#ifdef CFG_CALIBRATED_AT_FAB
#define UNCALIBRATED_AT_FAB		0	//0: OTP header has calibration values, 1: OTP header is empty
#else
#define UNCALIBRATED_AT_FAB		1	//0: OTP header has calibration values, 1: OTP header is empty
#endif
/*
 * Stream Queue
 ****************************************************************************************
 */
#ifdef CFG_STREAMDATA_QUEUE
#define STREAMDATA_QUEUE		1	
#else
#define STREAMDATA_QUEUE		0	
#undef  METRICS
#endif

#endif


/// Possible errors detected by FW
#define    RESET_NO_ERROR         0x00000000
#define    RESET_MEM_ALLOC_FAIL   0xF2F2F2F2

/// Reset platform and stay in ROM
#define    RESET_TO_ROM           0xA5A5A5A5
/// Reset platform and reload FW
#define    RESET_AND_LOAD_FW      0xC3C3C3C3



/**
 ****************************************************************************************
 * @brief Compute size of SW stack used.
 *
 * This function is compute the maximum size stack used by SW.
 *
 * @return Size of stack used (in bytes)
 ****************************************************************************************
 */
uint16_t get_stack_usage(void);

/**
 ****************************************************************************************
 * @brief Re-boot FW.
 *
 * This function is used to re-boot the FW when error has been detected, it is the end of
 * the current FW execution.
 * After waiting transfers on UART to be finished, and storing the information that
 * FW has re-booted by itself in a non-loaded area, the FW restart by branching at FW
 * entry point.
 *
 * Note: when calling this function, the code after it will not be executed.
 *
 * @param[in] error      Error detected by FW
 ****************************************************************************************
 */
void platform_reset(uint32_t error);

#if PLF_DEBUG
/**
 ****************************************************************************************
 * @brief Print the assertion error reason and loop forever.
 *
 * @param condition C string containing the condition.
 * @param file C string containing file where the assertion is located.
 * @param line Line number in the file where the assertion is located.
 ****************************************************************************************
 */
void assert_err(const char *condition, const char * file, int line);

/**
 ****************************************************************************************
 * @brief Print the assertion error reason and loop forever.
 * The parameter value that is causing the assertion will also be disclosed.
 *
 * @param param parameter value that is caused the assertion.
 * @param file C string containing file where the assertion is located.
 * @param line Line number in the file where the assertion is located.
 ****************************************************************************************
 */
void assert_param(int param0, int param1, const char * file, int line);

/**
 ****************************************************************************************
 * @brief Print the assertion warning reason.
 *
 * @param condition C string containing the condition.
 * @param file C string containing file where the assertion is located.
 * @param line Line number in the file where the assertion is located.
 ****************************************************************************************
 */
void assert_warn(const char *condition, const char * file, int line);
#endif //PLF_DEBUG


/*
 * ASSERTION CHECK
 ****************************************************************************************
 */
#ifndef ASSERT_ERROR
#define ASSERT_ERROR(x)                                         \
    {                                                           \
        do {                                                    \
            if (DEVELOPMENT_DEBUG)                              \
            {                                                   \
                if (!(x))                                       \
                {                                               \
                    __asm("BKPT #0\n");                         \
                }                                               \
            }                                                   \
            else                                                \
            {                                                   \
                if (!(x))                                       \
                {                                               \
					outp16(WATCHDOG_REG, 0x1);				 \
					outp16(RESET_FREEZE_REG, FRZ_WDOG); 	 \
                    while(1);                                   \
                }                                               \
            }                                                   \
        } while (0);                                            \
    }

#define ASSERT_WARNING(x)                                       \
    {                                                           \
        do {                                                    \
            if (DEVELOPMENT_DEBUG)                              \
            {                                                   \
                if (!(x))                                       \
                {                                               \
                    __asm("BKPT #0\n");                         \
                }                                               \
            }                                                   \
        } while (0);                                            \
    }
#endif


#if PLF_DEBUG
/// Assertions showing a critical error that could require a full system reset

#if 0
#define ASSERT_ERR(cond)                              \
    do {                                              \
        if (!(cond)) {                                \
            assert_err(#cond, __MODULE__, __LINE__);  \
        }                                             \
    } while(0)

/// Assertions showing a critical error that could require a full system reset
#define ASSERT_INFO(cond, param0, param1)             \
    do {                                              \
        if (!(cond)) {                                \
            assert_param((int)param0, (int)param1, __MODULE__, __LINE__);  \
        }                                             \
    } while(0)

/// Assertions showing a non-critical problem that has to be fixed by the SW
#define ASSERT_WARN(cond)                             \
    do {                                              \
        if (!(cond)) {                                \
            assert_warn(#cond, __MODULE__, __LINE__); \
        }                                             \
    } while(0)
#else
//#if 0
//		DBG_SYS(DBG_SYSTEM_ASSERT_ERROR, __LINE__); \
//		dbg_sys_print();                            \
//		printf("ass[%s:%d]\n",__func__,__LINE__);	\
//		while(1);                                   \
//#endif
#define ASSERT_ERR(cond) 							    						\
   do{                                               	\
       if (!(cond))                                   \
        {                                             \
			printf("ass[%s:%d]\n",__func__,__LINE__);	\
		   while(1);  																		\
		}                                               	\
    } while (0)

#define ASSERT_INFO(cond, param0, param1)

#define ASSERT_WARN(cond)

	
#endif


#endif //PLF_DEBUG




#if 0
#ifndef __DA14581__
extern const uint32_t * const jump_table_base[88];
#else    
extern const uint32_t * const jump_table_base[92];
#endif    
#define jump_table_struct (uint32_t)jump_table_base
#endif


/// Object allocated in shared memory - check linker script
#define __SHARED __attribute__ ((section("shram")))





/** lower two bits indicate debug level
 * - 0 all
 * - 1 warning
 * - 2 serious
 * - 3 severe
 */
#define RWIP_DBG_LEVEL_ALL     0x00
#define RWIP_DBG_LEVEL_OFF     RWIP_DBG_LEVEL_ALL /* compatibility define only */
#define RWIP_DBG_LEVEL_WARNING 0x01 /* bad checksums, dropped packets, ... */
#define RWIP_DBG_LEVEL_SERIOUS 0x02 /* memory allocation failures, ... */
#define RWIP_DBG_LEVEL_SEVERE  0x03
#define RWIP_DBG_MASK_LEVEL    0x03

/** flag for LWIP_DEBUGF to enable that debug message */
#define RWIP_DBG_ON            0x80U
/** flag for LWIP_DEBUGF to disable that debug message */
#define RWIP_DBG_OFF           0x00U
/** flag for LWIP_DEBUGF indicating a tracing message (to follow program flow) */
#define RWIP_DBG_TRACE         0x40U
/** flag for LWIP_DEBUGF indicating a state debug message (to follow module states) */
#define RWIP_DBG_STATE         0x20U
/** flag for LWIP_DEBUGF indicating newly added code, not thoroughly tested yet */
#define RWIP_DBG_FRESH         0x10U
/** flag for LWIP_DEBUGF to halt after printing this debug message */
#define RWIP_DBG_HALT          0x08U


#define RWIP_PLATFORM_DIAG(x)	do {printf("[%d %s:%d]\t",NOW(),__func__,__LINE__); printf x;} while(0)
#ifdef CFG_DBG
/** print debug message only if debug message type is enabled...
 *  AND is of correct type AND is at least LWIP_DBG_LEVEL
 */
#define RWIP_DEBUGF(debug, message) do { \
                               if ( \
                                   ((debug) & RWIP_DBG_ON) && \
                                   ((short)((debug) & RWIP_DBG_MASK_LEVEL) >= RWIP_DBG_LEVEL_ALL)) { \
                                 		RWIP_PLATFORM_DIAG(message); \
                                 if ((debug) & RWIP_DBG_HALT) { \
                                   while(1); \
                                 } \
                               } \
                             } while(0)

#else  /* RWIP_DEBUG */
#define RWIP_DEBUGF(debug, message) 
#endif /* RWIP_DEBUG */

__INLINE void show_reg(uint8_t *data,uint32_t len)
{
	uint32_t i=0;
	if(len == 0) return;
	for(;i<len;i++)
	{
		printf("%d:[%02X] ",i,data[i]);
		if(i%10 == 0 && i != 0)
			printf("\r\n");
	}
	if(i%10 != 0)
		printf("\r\n");
}

__INLINE void show_reg2(uint8_t *data,uint32_t len)
{
	uint32_t i=0;
	if(len == 0) return;
	for(;i<len;i++)
	{
		printf("0x%02X ",data[i]);
	}
	printf("\n");
}
__INLINE void show_reg3(uint8_t const *data,uint32_t len)
{
	uint32_t i=0;
	if(len == 0) return;
	for(;i<len;i++)
	{
		printf("0x%02X ",data[i]);
	}
	printf("\n");
}


#define APP_DEBUG RWIP_DBG_OFF
#define MEM_DEBUG RWIP_DBG_OFF

#define MSG_DEBUG RWIP_DBG_OFF
#define TASK_DEBUG RWIP_DBG_OFF
#define TIM_DEBUG RWIP_DBG_OFF


#define LLM_DEBUG RWIP_DBG_OFF
#define LLD_DEBUG RWIP_DBG_OFF
#define LLC_DEBUG RWIP_DBG_OFF


#define ISR_DEBUG RWIP_DBG_OFF

#define TRACE_DBG() //printf("[%s#%u]\r\n",__FUNCTION__,__LINE__)

#define mem_printf	 //printf
#define lld_printf(...)	//do{printf/("[%s##%u]",__FUNCTION__,__LINE__);printf(__VA_ARGS__);}while(0)
#define llc_printf(...)	//do{printf("[%s##%u]",__FUNCTION__,__LINE__);printf(__VA_ARGS__);}while(0)
#define llm_printf(...)	//do{printf("[%s##%u]",__FUNCTION__,__LINE__);printf(__VA_ARGS__);}while(0)
#define sleep_printf 	//printf

#define isr_printf  //printf

#define proj_bqb_printf 	//printf	

#define proj_printf(...) 		 //do{printf("[%s##%u]",__FUNCTION__,__LINE__);printf(__VA_ARGS__);}while(0)
#define proj_adv_printf(...) 	 //do{printf("[%s##%u]",__FUNCTION__,__LINE__);printf(__VA_ARGS__);}while(0)
#define proj_scan_printf 	//printf
#define proj_cdb_printf 	//printf

#define proj_sm_printf 	//printf

#define proj_att_printf 	//printf
#define proj_disc_svc_printf 	//printf
#define proj_rd_by_type_printf 	//printf
#define proj_write_req_printf 	//printf

#define hci_printf(...)	//do{printf("[%s##%u]",__FUNCTION__,__LINE__);printf(__VA_ARGS__);}while(0)
#define proj_buf_printf 	//printf
#define proj_data_printf 	//printf
#define proj_pdu_printf 	//printf

#define gapm_printf(...) //do{printf("[%s##%u]",__FUNCTION__,__LINE__);printf(__VA_ARGS__);}while(0)

#define proj_tmr_printf 	//printf
#define proj_send_con_printf 	//printf
#define proj_con_printf 	//printf

#define proj_sps_printf 	//printf
#define proj_tmp_client_printf //printf

#define proj_hid_server_printf 	//printf		//hid server
#define proj_findme_server_printf 	//printf	//findme server
#define proj_diss_server_printf 	//printf	//dis server
#define proj_bass_server_printf 	//printf	//batt server

#define proj_skyworth_printf 	//printf	
#define proj_hbtv_printf 	//printf
#define proj_hqtank_printf 	//printf	
#define proj_rgb_printf   //printf
#define proj_gun_play_printf //printf

#define dbg_printf  //printf

#define crash_printf	 //printf

// required to define GLOBAL_INT_** macros as inline assembly. This file is included after
// definition of ASSERT macros as they are used inside ll.h
#include "ll.h"     // ll definitions

/// @} DRIVERS
#endif // _ARCH_H_
