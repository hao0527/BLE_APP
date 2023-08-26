/**************************************************************************//**
 * @file     system_PN102Series.h
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 16/02/26 10:19a $
 * @brief    PN020 series system clock definition file
 *
 * @note
 * Copyright (C) 2016 Panchip Technology Corp. All rights reserved.
 *****************************************************************************/


#ifndef __SYSTEM_PN102SERIES_H__
#define __SYSTEM_PN102SERIES_H__

#ifdef __cplusplus
extern "C" {
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* Macro Definition                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
  Define SYSCLK
 *----------------------------------------------------------------------------*/

#define __XTAL4M        	(4000000UL)
 #define __XTAL16M        	(16000000UL)
#define __XTAL32K        	(32768UL)
#define __IRC10K         	(10000UL)
#define __XTAL            	__XTAL4M

#define	__EXT				__XTAL4M
#define __HXT             	__XTAL16M
#define __HIRC           	(26000000ul)
#define	__LIRC				__IRC10K
#define	__PLL				(26000000ul)

#define __DPLL_13M          (13000000ul)
#define __DPLL_26M          (26000000ul)
#define __DPLL_52M          (52000000ul)

extern uint32_t __HSI;
extern uint32_t SystemCoreClock;        /*!< System Clock Frequency (Core Clock) */
extern uint32_t CyclesPerUs;            /*!< Cycles per micro second */
extern uint32_t PllClock;               /*!< PLL Output Clock Frequency          */
/**
 * Update SystemCoreClock variable
 *
 * @param  None
 * @return None
 *
 * @brief  Updates the SystemCoreClock with current core Clock
 *         retrieved from CPU registers.
 */

extern void SystemCoreClockUpdate (void);
extern void SystemInit (void);

#ifdef __cplusplus
}
#endif

#endif  //__SYSTEM_PN102Series_H__


/*** (C) COPYRIGHT 2016 Panchip Technology Corp. ***/
