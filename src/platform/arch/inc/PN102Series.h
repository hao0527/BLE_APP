/**************************************************************************//**
 * @file     PN020.h
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 16/03/02 15:34 $
 * @brief    PN020 series peripheral access layer header file.
 *           This file contains all the peripheral register's definitions,
 *           bits definitions and memory mapping for PN020 series MCU.
 *
 * @note
 * Copyright (C) 2016 Panchip Technology Corp. All rights reserved.
 *****************************************************************************/
/**
   \mainpage Panchip PN020 Driver Reference Guide
   *
   * <b>Introduction</b>
   *
   * This user manual describes the usage of pn020 MCU device driver
   *
   * <b>Disclaimer</b>
   *
   * The Software is furnished "AS IS", without warranty as to performance or results, and
   * the entire risk as to performance or results is assumed by YOU. Panchip disclaims all
   * warranties, express, implied or otherwise, with regard to the Software, its use, or
   * operation, including without limitation any and all warranties of merchantability, fitness
   * for a particular purpose, and non-infringement of intellectual property rights.
   *
   * <b>Important Notice</b>
   *
   * Panchip Products are neither intended nor warranted for usage in systems or equipment,
   * any malfunction or failure of which may cause loss of human life, bodily injury or severe
   * property damage. Such applications are deemed, "Insecure Usage".
   *
   * Insecure usage includes, but is not limited to: equipment for surgical implementation,
   * atomic energy control instruments, airplane or spaceship instruments, the control or
   * operation of dynamic, brake or safety systems designed for vehicular use, traffic signal
   * instruments, all types of safety devices, and other applications intended to support or
   * sustain life.
   *
   * All Insecure Usage shall be made at customer's risk, and in the event that third parties
   * lay claims to Panchip as a result of customer's Insecure Usage, customer shall indemnify
   * the damages and liabilities thus incurred by Panchip.
   *
   * Please note that all data and specifications are subject to change without notice. All the
   * trademarks of products and companies mentioned in this datasheet belong to their respective
   * owners.
   *
   * <b>Copyright Notice</b>
   *
   * Copyright (C) 2016 Panchip Technology Corp. All rights reserved.
   */
/**
  * \page pg1 PanchipMicro PN020 BSP Directory Structure
  * Please refer to Readme.pdf under BSP root directory for the BSP directory structure
  *
  * \page pg2 Revision History

*/


#ifndef __PN102SERIES_H__
#define __PN102SERIES_H__

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup PN020_Definitions PN020 Definitions
  This file defines all structures and symbols for PN020:
    - interrupt numbers
    - registers and bit fields
    - peripheral base address
    - peripheral ID
    - Peripheral definitions
  @{
*/

/******************************************************************************/
/*                Processor and Core Peripherals                              */
/******************************************************************************/
/** @addtogroup PN020_CMSIS Device CMSIS Definitions
  Configuration of the Cortex-M0 Processor and Core Peripherals
  @{
*/

/**
 * @details  Interrupt Number Definition. The maximum of 32 Specific Interrupts are possible.
 */

//for 32k
//32K Coarse Tunning
 //32K Fine Tunning


typedef enum IRQn {


 /******  Cortex-M0 Processor Exceptions Numbers *****************************************/

NonMaskableInt_IRQn     = -14,    /*!< 2 Non Maskable Interrupt                           */
HardFault_IRQn          = -13,    /*!< 3 Cortex-M0 Hard Fault Interrupt                   */
SVCall_IRQn             = -5,     /*!< 11 Cortex-M0 SV Call Interrupt                     */
PendSV_IRQn             = -2,     /*!< 14 Cortex-M0 Pend SV Interrupt                     */
SysTick_IRQn            = -1,     /*!< 15 Cortex-M0 System Tick Interrupt                 */

/******  PN020 specific Interrupt Numbers ***********************************************/
DMA_IRQn                = 0,      /*!< DMA interrupt            */
WDT_IRQn                = 1,      /*!< Watch Dog Timer interrupt                          */
EINT0_IRQn              = 2,      /*!< External signal interrupt from P3.2 pin            */
EINT1_IRQn              = 3,      /*!< External signal interrupt from P3.3 pin            */
GPIO01_IRQn             = 4,      /*!< External signal interrupt from P0/P1               */
GPIO234_IRQn            = 5,      /*!< External interrupt from P2/P3/P4                   */
PWM_IRQn                = 6,      /*!< PWM interrupt                                      */
SPI2_IRQn                = 7,      /*!< SPI2 interrupt                                      */
TMR0_IRQn               = 8,      /*!< Timer 0 interrupt                                  */
TMR1_IRQn               = 9,      /*!< Timer 1 interrupt                                  */
TMR2_IRQn               = 10,     /*!< Timer 2 interrupt                                  */
BOD_IRQn			    = 11,	  /*!< BOD interrupt                                  */
UART0_IRQn              = 12,     /*!< UART0 interrupt                                    */
UART1_IRQn              = 13,     /*!< UART1 interrupt                                    */
SPI0_IRQn               = 14,     /*!< SPI0 interrupt                                      */
SPI1_IRQn			    = 15,     /*!< SPI1 interrupt                                      */
GPIO5_IRQn              = 16,     /*!< External interrupt from P5                         */
I2C0_IRQn               = 18,     /*!< I2C0 interrupt                                     */
I2C1_IRQn               = 19,     /*!< I2C1 interrupt                                     */
BLE_RADIOCTL_IRQn	  	= 20,
BLE_FINETGTIM_IRQn    	= 21,
BLE_GROSSTGTIM_IRQn   	= 22,
BLE_ERROR_IRQn        	= 23,
BLE_CRYPT_IRQn        	= 24,
SPI3_IRQn               = 25,     /*!< SPI3 interrupt                                     */
BLE_EVENT_IRQn        	= 26,
BLE_SLP_IRQn          	= 27,
PDWU_IRQn               = 28,     /*!< Power Down Wake up interrupt                       */
ADC_IRQn                = 29,      /*!< ADC interrupt                                      */
BLE_RX_IRQn           	= 30,
BLE_CSCNT_IRQn        	= 31

} IRQn_Type;



/*
 * ==========================================================================
 * ----------- Processor and Core Peripheral Section ------------------------
 * ==========================================================================
 */


/* Configuration of the Cortex-M0 Processor and Core Peripherals */
#define __CM0_REV                0x0201    /*!< Core Revision r2p1                               */
#define __NVIC_PRIO_BITS         2         /*!< Number of Bits used for Priority Levels          */
#define __Vendor_SysTickConfig   0         /*!< Set to 1 if different SysTick Config is used     */
#define __MPU_PRESENT            0         /*!< MPU present or not                               */
#define __FPU_PRESENT            0         /*!< FPU present or not                               */

/*@}*/ /* end of group PN020_CMSIS */


#include "core_cm0.h"                       /* Cortex-M0 processor and core peripherals           */
#include "system_PN102Series.h"            /* PN020 Series System include file                  */
#include <stdint.h>
#include <stdbool.h>

/******************************************************************************/
/*                Device Specific Peripheral registers structures             */
/******************************************************************************/
/** @addtogroup PN020_Peripherals PN020 Peripherals
  PN020 Device Specific Peripheral registers structures
  @{
*/

#if defined ( __CC_ARM  )
#pragma anon_unions
#endif


/******************************************************************************/
/*                Device Specific Peripheral registers structures             */
/******************************************************************************/


/*---------------------- Analog Comparator Controller -------------------------*/
/**
    @addtogroup ACMP Analog Comparator Controller(ACMP)
    Memory Mapped Structure for ACMP Controller
@{ */

typedef struct {


    /**
     * CTL0, 1
     * ===================================================================================================
     * Offset: 0x00, 0x04  Analog Comparator 0, 1 Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ACMPEN    |Analog Comparator x Enable Bit
     * |        |          |0 = Analog Comparator x Disabled.
     * |        |          |1 = Analog Comparator x Enabled.
     * |        |          |Note: Analog comparator output needs to wait 2 us stable time after this bit is set.
     * |[1]     |ACMPIE    |Analog Comparator x Interrupt Enable Bit
     * |        |          |0 = Interrupt function Disabled.
     * |        |          |1 = Interrupt function Enabled.
     * |[2]     |HYSSEL    |Analog Comparator x Hysteresis Select Bit
     * |        |          |0 = Hysteresis function Disabled.
     * |        |          |1 = Hysteresis function Enabled.
     * |[4]     |NEGSEL    |Analog Comparator x Negative Input Select Bit
     * |        |          |0 = The source of the negative comparator input is from CPNx pin.
     * |        |          |1 = The source of the negative comparator input is from internal band-gap voltage or comparator reference voltage.
     * |[23:20] |FILTSEL   |Comparator Output Filter Count Selection
     * |        |          |0000 = Filter function is Disabled.
     * |        |          |0001 = ACMP0 output is sampled 1 consecutive PCLK.
     * |        |          |0010 = ACMP0 output is sampled 2 consecutive PCLKs.
     * |        |          |0011 = ACMP0 output is sampled 4 consecutive PCLKs.
     * |        |          |0100 = ACMP0 output is sampled 8 consecutive PCLKs.
     * |        |          |0101 = ACMP0 output is sampled 16 consecutive PCLKs.
     * |        |          |0110 = ACMP0 output is sampled 32 consecutive PCLKs.
     * |        |          |0111 = ACMP0 output is sampled 64 consecutive PCLKs.
     * |        |          |1000 = ACMP0 output is sampled 128 consecutive PCLKs.
     * |        |          |1001 = ACMP0 output is sampled 256 consecutive PCLKs.
     * |        |          |1010 = ACMP0 output is sampled 512 consecutive PCLKs.
     * |        |          |Others = Reserved.
     * |[30:29] |POSSEL    |Analog Comparator Positive Input Selection
     * |        |          |00 = ACMP0_Px is from ACMPx_P0 pin.
     * |        |          |01 = ACMP0_Px is from ACMPx_P1 pin.
     * |        |          |10 = ACMP0_Px is from ACMPx_P2 pin.
     * |        |          |11 = ACMP0_Px is from ACMPx_P3 pin.
    */
    __IO uint32_t CTL[2];

    /**
     * STATUS
     * ===================================================================================================
     * Offset: 0x08  Analog Comparator 0/1 Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ACMPIF0   |Analog Comparator 0 Flag
     * |        |          |This bit is set by hardware whenever the comparator 0 output changes state.
     * |        |          |This will generate an interrupt if ACMPIE (ACMP_CTL0[1]) = 1.
     * |        |          |0 = Analog comparator 0 output does not change.
     * |        |          |1 = Analog comparator 0 output changed.
     * |        |          |Note: This bit can be cleared to 0 by software writing 1.
     * |[1]     |ACMPIF1   |Analog Comparator 1 Flag
     * |        |          |This bit is set by hardware whenever the comparator 1 output changes state.
     * |        |          |This will generate an interrupt if ACMPIE (ACMP_CTL1[1]) = 1.
     * |        |          |0 = Analog comparator 1 output does not change.
     * |        |          |1 = Analog comparator 1 output changed.
     * |        |          |Note: This bit can be cleared to 0 by software writing 1.
     * |[2]     |ACMPO0    |Analog Comparator 0 Output
     * |        |          |Synchronized to the APB clock to allow reading by software.
     * |        |          |Cleared when the comparator 0 is disabled ACMPEN (ACMP_CTL0[0]) = 0.
     * |        |          |0 = Analog comparator 0 outputs 0.
     * |        |          |1 = Analog comparator 0 outputs 1.
     * |[3]     |ACMPO1    |Analog Comparator 1 Output
     * |        |          |Synchronized to the APB clock to allow reading by software.
     * |        |          |Cleared when the comparator 1 is disabled ACMPEN (ACMP_CTL1[0]) = 0.
     * |        |          |0 = Analog comparator 1 outputs 0.
     * |        |          |1 = Analog comparator 1 outputs 1.
     * |[8]     |ACMPF0    |Analog Comparator 0 Flag
     * |        |          |This bit is set by hardware whenever the comparator 0 output changes state.
     * |        |          |0 = Analog comparator 0 output does not change.
     * |        |          |1 = Analog comparator 0 output changed.
     * |        |          |Note: This bit can be cleared to 0 by software writing 1.
     * |[9]     |ACMPF1    |Analog Comparator 1 Flag
     * |        |          |This bit is set by hardware whenever the comparator 1 output changes state.
     * |        |          |0 = Analog comparator 1 output does not change.
     * |        |          |1 = Analog comparator 1 output changed.
     * |        |          |Note: This bit can be cleared to 0 by software writing 1.

    */
    __IO uint32_t STATUS;

    /**
     * VREF
     * ===================================================================================================
     * Offset: 0x0C  Analog Comparator Reference Voltage Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |CRVCTL    |Comparator Reference Voltage Control
     * |        |          |Comparator reference voltage = AVDD * (1 / 6 + CRVCTL(ACMP_VREF[3:0]) / 24).
     * |[7]     |IREFSEL   |CRV Internal Reference Selection
     * |        |          |0 = Band-gap voltage.
     * |        |          |1 = Internal comparator reference voltage.
    */
    __IO uint32_t VREF;

} ACMP_T;

/**
    @addtogroup ACMP_CONST ACMP Bit Field Definition
    Constant Definitions for ACMP Controller
@{ */

#define ACMP_CTL_ACMPEN_Pos             (0)                                                /*!< ACMP_T::CTL: ACMPEN Position              */
#define ACMP_CTL_ACMPEN_Msk             (0x1ul << ACMP_CTL_ACMPEN_Pos)                    /*!< ACMP_T::CTL: ACMPEN Mask                  */

#define ACMP_CTL_ACMPIE_Pos             (1)                                                /*!< ACMP_T::CTL: ACMPIE Position              */
#define ACMP_CTL_ACMPIE_Msk             (0x1ul << ACMP_CTL_ACMPIE_Pos)                     /*!< ACMP_T::CTL: ACMPIE Mask                  */

#define ACMP_CTL_HYSSEL_Pos             (2)                                                /*!< ACMP_T::CTL: HYSSEL Position              */
#define ACMP_CTL_HYSSEL_Msk             (0x1ul << ACMP_CTL_HYSSEL_Pos)                     /*!< ACMP_T::CTL: HYSSEL Mask                  */

#define ACMP_CTL_NEGSEL_Pos             (4)                                                /*!< ACMP_T::CTL: NEGSEL Position              */
#define ACMP_CTL_NEGSEL_Msk             (0x1ul << ACMP_CTL_NEGSEL_Pos)                     /*!< ACMP_T::CTL: NEGSEL Mask                  */

#define ACMP_CTL_FILTSEL_Pos            (20)                                               /*!< ACMP_T::CTL: FILTSEL Position             */
#define ACMP_CTL_FILTSEL_Msk            (0xful << ACMP_CTL_FILTSEL_Pos)                    /*!< ACMP_T::CTL: FILTSEL Mask                 */

#define ACMP_CTL_POSSEL_Pos             (29)                                               /*!< ACMP_T::CTL: POSSEL Position              */
#define ACMP_CTL_POSSEL_Msk             (0x3ul << ACMP_CTL_POSSEL_Pos)                     /*!< ACMP_T::CTL: POSSEL Mask                  */

#define ACMP_STATUS_ACMPIF0_Pos         (0)                                               /*!< ACMP_T::STATUS: ACMPIF0 Position          */
#define ACMP_STATUS_ACMPIF0_Msk         (0x1ul << ACMP_STATUS_ACMPIF0_Pos)                /*!< ACMP_T::STATUS: ACMPIF0 Mask              */

#define ACMP_STATUS_ACMPIF1_Pos         (1)                                               /*!< ACMP_T::STATUS: ACMPIF1 Position          */
#define ACMP_STATUS_ACMPIF1_Msk         (0x1ul << ACMP_STATUS_ACMPIF1_Pos)                /*!< ACMP_T::STATUS: ACMPIF1 Mask              */

#define ACMP_STATUS_ACMPO0_Pos          (2)                                               /*!< ACMP_T::STATUS: ACMPO0 Position           */
#define ACMP_STATUS_ACMPO0_Msk          (0x1ul << ACMP_STATUS_ACMPO0_Pos)                 /*!< ACMP_T::STATUS: ACMPO0 Mask               */

#define ACMP_STATUS_ACMPO1_Pos          (3)                                               /*!< ACMP_T::STATUS: ACMPO1 Position           */
#define ACMP_STATUS_ACMPO1_Msk          (0x1ul << ACMP_STATUS_ACMPO1_Pos)                 /*!< ACMP_T::STATUS: ACMPO1 Mask               */

#define ACMP_STATUS_ACMPF0_Pos          (8)                                               /*!< ACMP_T::STATUS: ACMPIF0 Position          */
#define ACMP_STATUS_ACMPF0_Msk          (0x1ul << ACMP_STATUS_ACMPF0_Pos)                 /*!< ACMP_T::STATUS: ACMPIF0 Mask              */

#define ACMP_STATUS_ACMPF1_Pos          (9)                                               /*!< ACMP_T::STATUS: ACMPIF1 Position          */
#define ACMP_STATUS_ACMPF1_Msk          (0x1ul << ACMP_STATUS_ACMPF1_Pos)                 /*!< ACMP_T::STATUS: ACMPIF1 Mask              */

#define ACMP_VREF_CRVCTL_Pos            (0)                                               /*!< ACMP_T::VREF: CRVCTL Position             */
#define ACMP_VREF_CRVCTL_Msk            (0xful << ACMP_VREF_CRVCTL_Pos)                   /*!< ACMP_T::VREF: CRVCTL Mask                 */

#define ACMP_VREF_IREFSEL_Pos           (7)                                               /*!< ACMP_T::VREF: IREFSEL Position            */
#define ACMP_VREF_IREFSEL_Msk           (0x1ul << ACMP_VREF_IREFSEL_Pos)                  /*!< ACMP_T::VREF: IREFSEL Mask                */

/**@}*/ /* ACMP_CONST */
/**@}*/ /* end of ACMP register group */


/*---------------------- Analog to Digital Converter -------------------------*/
/**
    @addtogroup ADC Analog to Digital Converter(ADC)
    Memory Mapped Structure for ADC Controller
@{ */

typedef struct {


    /**
     * DAT
     * ===================================================================================================
     * Offset: 0x00  A/D Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[11:0]   |RESULT    |A/D Conversion Result
     * |        |          |This field contains conversion result of ADC.
     * |[16]    |OV        |Over Run Flag
     * |        |          |If converted data in RESULT[9:0] has not been read before the new conversion result is loaded to this register, OV is set to 1.
     * |        |          |It is cleared by hardware after the ADC_DAT register is rea.
     * |        |          |0 = Data in RESULT[9:0] is recent conversion result.
     * |        |          |1 = Data in RESULT[9:0] overwrote.
     * |[17]    |VALID     |Valid Flag
     * |        |          |This bit is set to 1 when ADC conversion is completed and cleared by hardware after the ADC_DAT register is read.
     * |        |          |0 = Data in RESULT[9:0] bits not valid.
     * |        |          |1 = Data in RESULT[9:0] bits valid.
    */
    __I  uint32_t DAT;
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVED0[7];
    /// @endcond //HIDDEN_SYMBOLS


    /**
     * CTL
     * ===================================================================================================
     * Offset: 0x20  A/D Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ADCEN     |A/D Converter Enable Bit
     * |        |          |0 = A/D Converter Disabled.
     * |        |          |1 = A/D Converter Enabled.
     * |        |          |Note: Before starting A/D conversion function, this bit should be set to 1.
     * |        |          |Clear it to 0 to disable A/D converter analog circuit to save power consumption.
     * |[1]     |ADCIEN    |A/D Interrupt Enable Bit
     * |        |          |A/D conversion end interrupt request is generated if ADCIEN bit is set to 1.
     * |        |          |0 = A/D interrupt function Disabled.
     * |        |          |1 = A/D interrupt function Enabled.
     * |[5:4]   |HWTRGSEL  |Hardware Trigger Source Select Bit
     * |        |          |00 = A/D conversion is started by external STADC pin.
     * |        |          |11 = A/D conversion is started by PWM trigger.
     * |        |          |Others = Reserved.
     * |        |          |Note: Software should disable TRGEN and SWTRG before change TRGS.
     * |[6]     |HWTRGCOND |Hardware External Trigger Condition
     * |        |          |This bit decides whether the external pin STADC trigger event is falling or raising edge.
     * |        |          |The signal must be kept at stable state at least 4 PCLKs at high and low state for edge trigge.
     * |        |          |0 = Falling edge.
     * |        |          |1 = Raising edge.
     * |[8]     |HWTRGEN   |Hardware External Trigger Enable Bit
     * |        |          |Enable or disable triggering of A/D conversion by external STADC pin.
     * |        |          |If external trigger is enabled, the SWTRG bit can be set to 1 by the selected hardware trigger sourc.
     * |        |          |0= External trigger Disabled.
     * |        |          |1= External trigger Enabled.
     * |[11]    |SWTRG     |Software Trigger A/D Conversion Start
     * |        |          |SWTRG bit can be set to 1 from two sources: software and external pin STADC.
     * |        |          |SWTRG will be cleared to 0 by hardware automatically after conversion complet.
     * |        |          |0 = Conversion stopped and A/D converter entered idle state.
     * |        |          |1 = Conversion start.
    */
    __IO uint32_t CTL;

    /**
     * CHEN
     * ===================================================================================================
     * Offset: 0x24  A/D Channel Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CHEN0     |Analog Input Channel 0 Enable Bit
     * |        |          |0 = Channel 0 Disabled.
     * |        |          |1 = Channel 0 Enabled.
     * |        |          |Note: If software enables more than one channel, the channel with the smallest number will be selected and the other enabled channels will be ignored.
     * |[1]     |CHEN1     |Analog Input Channel 1 Enable Bit
     * |        |          |0 = Channel 1 Disabled.
     * |        |          |1 = Channel 1 Enabled.
     * |[2]     |CHEN2     |Analog Input Channel 2 Enable Bit
     * |        |          |0 = Channel 2 Disabled.
     * |        |          |1 = Channel 2 Enabled.
     * |[3]     |CHEN3     |Analog Input Channel 3 Enable Bit
     * |        |          |0 = Channel 3 Disabled.
     * |        |          |1 = Channel 3 Enabled.
     * |[4]     |CHEN4     |Analog Input Channel 4 Enable Bit
     * |        |          |0 = Channel 4 Disabled.
     * |        |          |1 = Channel 4 Enabled.
     * |[5]     |CHEN5     |Analog Input Channel 5 Enable Bit
     * |        |          |0 = Channel 5 Disabled.
     * |        |          |1 = Channel 5 Enabled.
     * |[6]     |CHEN6     |Analog Input Channel 6 Enable Bit
     * |        |          |0 = Channel 6 Disabled.
     * |        |          |1 = Channel 6 Enabled.
     * |[7]     |CHEN7     |Analog Input Channel 7 Enable Bit
     * |        |          |0 = Channel 7 Disabled.
     * |        |          |1 = Channel 7 Enabled.
     * |[8]     |CHEN8     |Analog Input Channel 8 Enable Bit
     * |        |          |0 = Channel 8 Disabled.
     * |        |          |1 = Channel 8 Enabled.		 
     * |[9]     |CH8SEL    |Analog Input Channel 8 Selection
     * |        |          |0 = External analog input.
     * |        |          |1 = Internal band-gap voltage (VBG).
     * |        |          |Note: When software selects the band-gap voltage as the analog input source of ADC channel 8, the ADC clock rate needs to be limited to lower than 300 kHz.
    */
    __IO uint32_t CHEN;

    /**
     * CMP0
     * ===================================================================================================
     * Offset: 0x28  A/D Compare Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ADCMPEN   |A/D Compare Enable Bit
     * |        |          |Set 1 to this bit to enable comparing CMPDAT (ADC_CMPx[25:16]) with specified channel conversion results when converted data is loaded into the ADC_DAT register.
     * |        |          |0 = Compare function Disabled.
     * |        |          |1 = Compare function Enabled.
     * |[1]     |ADCMPIE   |A/D Compare Interrupt Enable Bit
     * |        |          |If the compare function is enabled and the compare condition matches the setting of CMPCOND and CMPMCNT, ADCMPIE bit will be asserted, in the meanwhile, if ADCMPIE is set to 1, a compare interrupt request is generated.
     * |        |          |0 = Compare function interrupt Disabled.
     * |        |          |1 = Compare function interrupt Enabled.
     * |[2]     |CMPCOND   |Compare Condition
     * |        |          |0 = Set the compare condition as that when a 10-bit A/D conversion result is less than the 10-bit CMPDAT (ADC_CMPx[25:16]), the internal match counter will increase one.
     * |        |          |1 = Set the compare condition as that when a 10-bit A/D conversion result is greater or equal to the 10-bit CMPDAT (ADC_CMPx[25:16]), the internal match counter will increase one.
     * |        |          |Note: When the internal counter reaches the value to (CMPMCNT+1), the ADCMPFx bit will be set.
     * |[5:3]   |CMPCH     |Compare Channel Selection
     * |        |          |Set this field to select which channel's result to be compared.
     * |        |          |Note: Valid setting of this field is channel 0~7.
     * |[11:8]  |CMPMCNT   |Compare Match Count
     * |        |          |When the specified A/D channel analog conversion result matches the compare condition defined by CMPCOND[2], the internal match counter will increase 1.
     * |        |          |When the internal counter reaches the value to (CMPMCNT+1), the ADCMPFx bit will be set.
     * |[27:16] |CMPDAT    |Comparison Data
     * |        |          |The 12-bit data is used to compare with conversion result of specified channel.
    */
    __IO uint32_t CMP0;

    /**
     * CMP1
     * ===================================================================================================
     * Offset: 0x2C  A/D Compare Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ADCMPEN   |A/D Compare Enable Bit
     * |        |          |Set 1 to this bit to enable comparing CMPDAT (ADC_CMPx[25:16]) with specified channel conversion results when converted data is loaded into the ADC_DAT register.
     * |        |          |0 = Compare function Disabled.
     * |        |          |1 = Compare function Enabled.
     * |[1]     |ADCMPIE   |A/D Compare Interrupt Enable Bit
     * |        |          |If the compare function is enabled and the compare condition matches the setting of CMPCOND and CMPMCNT, ADCMPIE bit will be asserted, in the meanwhile, if ADCMPIE is set to 1, a compare interrupt request is generated.
     * |        |          |0 = Compare function interrupt Disabled.
     * |        |          |1 = Compare function interrupt Enabled.
     * |[2]     |CMPCOND   |Compare Condition
     * |        |          |0 = Set the compare condition as that when a 10-bit A/D conversion result is less than the 10-bit CMPDAT (ADC_CMPx[25:16]), the internal match counter will increase one.
     * |        |          |1 = Set the compare condition as that when a 10-bit A/D conversion result is greater or equal to the 10-bit CMPDAT (ADC_CMPx[25:16]), the internal match counter will increase one.
     * |        |          |Note: When the internal counter reaches the value to (CMPMCNT+1), the ADCMPFx bit will be set.
     * |[5:3]   |CMPCH     |Compare Channel Selection
     * |        |          |Set this field to select which channel's result to be compared.
     * |        |          |Note: Valid setting of this field is channel 0~7.
     * |[11:8]  |CMPMCNT   |Compare Match Count
     * |        |          |When the specified A/D channel analog conversion result matches the compare condition defined by CMPCOND[2], the internal match counter will increase 1.
     * |        |          |When the internal counter reaches the value to (CMPMCNT+1), the ADCMPFx bit will be set.
     * |[25:16] |CMPDAT    |Comparison Data
     * |        |          |The 10-bit data is used to compare with conversion result of specified channel.
    */
    __IO uint32_t CMP1;

    /**
     * STATUS
     * ===================================================================================================
     * Offset: 0x30  A/D Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ADIF      |A/D Conversion End Interrupt Flag
     * |        |          |A status flag that indicates the end of A/D conversion. ADIF is set to 1 When A/D conversion ends.
     * |        |          |Note: This bit can be cleared to 0 by software writing 1.
     * |[1]     |ADCMPIF0   |A/D Compare interrupt Flag 0
     * |        |          |When the selected channel A/D conversion result meets the setting condition in ADC_CMP0, this bit is set to 1.
     * |        |          |0 = Conversion result in ADC_DAT does not meet the ADC_CMP0 setting.
     * |        |          |1 = Conversion result in ADC_DAT meets the ADC_CMP0 setting.
     * |        |          |Note: This bit can be cleared to 0 by software writing 1.
     * |[2]     |ADCMPIF1   |A/D Compare interrupt Flag 1
     * |        |          |When the selected channel A/D conversion result meets the setting condition in ADC_CMP1, this bit is set to 1.
     * |        |          |0 = Conversion result in ADC_DAT does not meet the ADC_CMP1 setting.
     * |        |          |1 = Conversion result in ADC_DAT meets the ADC_CMP1 setting.
     * |        |          |Note: This bit can be cleared to 0 by software writing 1.
     * |[3]     |BUSY      |BUSY/IDLE (Read Only)
     * |        |          |This bit is mirror of as SWTRG bit in ADC_CTL
     * |        |          |0 = A/D converter is in idle state.
     * |        |          |1 = A/D converter is busy at conversion.
     * |[6:4]   |CHANNEL   |Current Conversion Channel (Read Only)
     * |        |          |This filed reflects the current conversion channel when BUSY=1.
     * |        |          |When BUSY=0, it shows the number of the next converted channel.
     * |[8]     |VALID     |Data Valid Flag (Read Only)
     * |        |          |It is a mirror of VALID bit in ADC_DAT register.
     * |[16]    |OV        |Overrun Flag (Read Only)
     * |        |          |It is a mirror to OV bit in ADC_DAT register.
     * |[24]    |ADCF      |A/D Conversion End Flag
     * |        |          |A status flag that indicates the end of A/D conversion. ADIF is set to 1 When A/D conversion ends.
     * |        |          |Note: This bit can be cleared to 0 by software writing 1.
     * |[25]     |ADCMPF0   |A/D Compare Flag 0
     * |        |          |When the selected channel A/D conversion result meets the setting condition in ADC_CMP0, this bit is set to 1.
     * |        |          |0 = Conversion result in ADC_DAT does not meet the ADC_CMP0 setting.
     * |        |          |1 = Conversion result in ADC_DAT meets the ADC_CMP0 setting.
     * |        |          |Note: This bit can be cleared to 0 by software writing 1.
     * |[26]     |ADCMPF1   |A/D Compare Flag 1
     * |        |          |When the selecte d channel A/D conversion result meets the setting condition in ADC_CMP1, this bit is set to 1.
     * |        |          |0 = Conversion result in ADC_DAT does not meet the ADC_CMP1 setting.
     * |        |          |1 = Conversion result in ADC_DAT meets the ADC_CMP1 setting.
     * |        |          |Note: This bit can be cleared to 0 by software writing 1.

    */
    __IO uint32_t STATUS;
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVED1[4];
    /// @endcond //HIDDEN_SYMBOLS


    /**
     * TRGDLY
     * ===================================================================================================
     * Offset: 0x44  A/D Trigger Delay Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |DELAY     |PWM Trigger Delay Timer
     * |        |          |Set this field will delay ADC start conversion time after PWM trigger.
     * |        |          |PWM trigger delay time is (4 * DELAY) * system clock.
    */
    __IO uint32_t TRGDLY;

    /**
     * EXTSMPT
     * ===================================================================================================
     * Offset: 0x48  A/D Sampling Time Counter Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |EXTSMPT   |Additional ADC Sample Clock
     * |        |          |If the ADC input is unstable, user can set this register to increase the sampling time to get a stable ADC input signal.
     * |        |          |The default sampling time is 2 ADC clocks.
     * |        |          |The additional clock number will be inserted to lengthen the sampling clock.
     * |        |          |0 = disable adc.
     * |        |          |1 = Number of additional clock cycles is 1.
     * |        |          |2 = Number of additional clock cycles is 2.
     * |        |          |3 = Number of additional clock cycles is 3.
     * |        |          |4 = Number of additional clock cycles is 4.
     * |        |          |5 = Number of additional clock cycles is 5.
     * |        |          |6 = Number of additional clock cycles is 6.
     * |        |          |7 = Number of additional clock cycles is 7.
     * |        |          |8 = Number of additional clock cycles is 8.
     * |        |          |9 = Number of additional clock cycles is 9.
     * |        |          |... = Number of additional clock cycles is... 
     * |        |          |1023 = Number of additional clock cycles is1023
    */
    __IO uint32_t EXTSMPT;

    /**
     * SEQCTL
     * ===================================================================================================
     * Offset: 0x4C  A/D PWM Sequential Mode Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]    |SEQEN      |ADC Sequential Mode Enable Bit
     * |        |               |When ADC sequential mode is enabled, two of three ADC channels from 0 to 2 will automatically convert analog data in the sequence of channel [0, 1] or channel[1, 2] or channel[0, 2] defined by MODESEL (ADC_SEQCTL[3:2]).
     * |        |               |0 = ADC sequential mode Disabled.
     * |        |               |1 = ADC sequential mode Enabled.
     * |[1]    |SEQTYPE   |ADC Sequential Mode Type
     * |        |               |0 = ADC delay time is only inserted before the first conversion.
     * |        |               |The second conversion starts immediately after the first conversion is completed.
     * |        |               |(for 2/3-shunt type).
     * |        |               |1 = ADC delay time is inserted before each conversion. (for 1-shunt type)
     * |[3:2]   |MODESEL |ADC Sequential Mode Selection
     * |        |               |00 = Issue ADC_INT after Channel 0 then Channel 1 conversion finishes when SEQEN =1.
     * |        |               |01 = Issue ADC_INT after Channel 1 then Channel 2 conversion finishes when SEQEN =1.
     * |        |               |10 = Issue ADC_INT after Channel 0 then Channel 2 conversion finishes when SEQEN =1.
     * |        |               |11 = Reserved.
     * |[4]    |DELAY_EN  |ADC delay time inserted before 2nd conversion
     * |        |               |0 = ADC delay time only inserted before the 1st conversion;
     * |        |               |1 = ADC delay time inserted before each conversion.
     * |[5]    |TRG_SEL   |TRG1CTL or TRG2CTL select for 1-shunt sequential mode.
     * |        |               |0 = using TRG1CTL to trigger sequential conversion;
     * |        |               |1 = using TRG2CTL to trigger sequential conversion.
     * |[11:8]  |TRG1CTL   |PWM Trigger Source Selection For TRG1CTL[3:2]
     * |        |          |00 = PWM Trigger source is PWM0.
     * |        |          |01 = PWM Trigger source is PWM2.
     * |        |          |10 = PWM Trigger source is PWM4.
     * |        |          |11 = PWM Trigger source is PWM6..
     * |        |          |PWM Trigger Type Selection for TRG1CTL[1:0]
     * |        |          |00 = Rising of the selected PWM.
     * |        |          |01 = Center of the selected PWM.
     * |        |          |10 = Falling of the selected PWM.
     * |        |          |11 = Period of the selected PWM.
     * |        |          |Note: PWM trigger source is valid for 1-shunt and 2/3-shunt type.
     * |[19:16] |TRG2CTL   |PWM Trigger Source Selection For TRG2CTL[3:2]
     * |        |          |00 = PWM Trigger source is PWM0.
     * |        |          |01 = PWM Trigger source is PWM2.
     * |        |          |10 = PWM Trigger source is PWM4.
     * |        |          |11 = PWM Trigger source is PWM6..
     * |        |          |PWM Trigger Type Selection for TRG2CTL[1:0]
     * |        |          |00 = Rising of the selected PWM.
     * |        |          |01 = Center of the selected PWM.
     * |        |          |10 = Falling of the selected PWM.
     * |        |          |11 = Period of the selected PWM.
     * |        |          |Note: PWM trigger source is valid for 1-shunt type and 2/3-shunt type.
    */
    __IO uint32_t SEQCTL;

    /**
     * SEQDAT1
     * ===================================================================================================
     * Offset: 0x50  A/D PWM Sequential Mode First Result Register1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[9:0]   |RESULT    |A/D PWM Sequential Mode Conversion Result
     * |        |          |This field contains conversion result of ADC.
     * |[16]    |OV        |Over Run Flag
     * |        |          |If converted data in RESULT[9:0] has not been read before the new conversion result is loaded to this register, OV is set to 1.
     * |        |          |It is cleared by hardware after the ADC_SEQDATx register is read.
     * |        |          |0 = Data in RESULT[9:0] is recent conversion result.
     * |        |          |1 = Data in RESULT[9:0] overwritten.
     * |[17]    |VALID     |Valid Flag
     * |        |          |This bit is set to 1 when ADC conversion is completed and cleared by hardware after the ADC_SEQDATx register is read.
     * |        |          |0 = Data in RESULT[9:0] bits not valid.
     * |        |          |1 = Data in RESULT[9:0] bits valid.
    */
    __I  uint32_t SEQDAT1;

    /**
     * SEQDAT2
     * ===================================================================================================
     * Offset: 0x54  A/D PWM Sequential Mode Second Result Register1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[9:0]   |RESULT    |A/D PWM Sequential Mode Conversion Result
     * |        |          |This field contains conversion result of ADC.
     * |[16]    |OV        |Over Run Flag
     * |        |          |If converted data in RESULT[9:0] has not been read before the new conversion result is loaded to this register, OV is set to 1.
     * |        |          |It is cleared by hardware after the ADC_SEQDATx register is read.
     * |        |          |0 = Data in RESULT[9:0] is recent conversion result.
     * |        |          |1 = Data in RESULT[9:0] overwritten.
     * |[17]    |VALID     |Valid Flag
     * |        |          |This bit is set to 1 when ADC conversion is completed and cleared by hardware after the  ADC_SEQDATx; register is read.
     * |        |          |0 = Data in RESULT[9:0] bits not valid.
     * |        |          |1 = Data in RESULT[9:0] bits valid.
    */
    __I  uint32_t SEQDAT2;
    
    /**
     * SEQDAT2
     * ===================================================================================================
     * Offset: 0x58  A/D analog parameters register1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TESTMODE |A/D adc test mode enable
     * |         |          |0= test mode
     * |         |          |1= normal mode
     * |[1]     |SH_SEL | Default : 1
     * |         |          |0= External Sample Hold (SH) width with HOLD2(Internal Signal)
     * |         |          |1= SH is SH. 
     * |[19:16] |CLKDIV    |ADC Clock Divider  Could not be set as 0.
     * |[25:24] |CMPCTL   |ADC comparator current control
     * |[27:26] |DRV_CTL  |ADC VCM buffer current control
    */
    __IO  uint32_t CTL2;

}ADC_T;

/**
    @addtogroup ADC_CONST ADC Bit Field Definition
    Constant Definitions for ADC Controller
@{ */

#define ADC_DAT_RESULT_Pos               (0)                                               /*!< ADC_T::DAT: RESULT Position               */
#define ADC_DAT_RESULT_Msk               (0xffful << ADC_DAT_RESULT_Pos)                   /*!< ADC_T::DAT: RESULT Mask                   */

#define ADC_DAT_OV_Pos                   (16)                                              /*!< ADC_T::DAT: OV Position                   */
#define ADC_DAT_OV_Msk                   (0x1ul << ADC_DAT_OV_Pos)                         /*!< ADC_T::DAT: OV Mask                       */

#define ADC_DAT_VALID_Pos                (17)                                              /*!< ADC_T::DAT: VALID Position                */
#define ADC_DAT_VALID_Msk                (0x1ul << ADC_DAT_VALID_Pos)                      /*!< ADC_T::DAT: VALID Mask                    */

#define ADC_CTL_ADCEN_Pos                (0)                                               /*!< ADC_T::CTL: ADCEN Position                */
#define ADC_CTL_ADCEN_Msk                (0x1ul << ADC_CTL_ADCEN_Pos)                      /*!< ADC_T::CTL: ADCEN Mask                    */

#define ADC_CTL_ADCIEN_Pos               (1)                                               /*!< ADC_T::CTL: ADCIEN Position               */
#define ADC_CTL_ADCIEN_Msk               (0x1ul << ADC_CTL_ADCIEN_Pos)                     /*!< ADC_T::CTL: ADCIEN Mask                   */

#define ADC_CTL_HWTRGSEL_Pos             (4)                                               /*!< ADC_T::CTL: HWTRGSEL Position             */
#define ADC_CTL_HWTRGSEL_Msk             (0x3ul << ADC_CTL_HWTRGSEL_Pos)                   /*!< ADC_T::CTL: HWTRGSEL Mask                 */

#define ADC_CTL_HWTRGCOND_Pos            (6)                                               /*!< ADC_T::CTL: HWTRGCOND Position            */
#define ADC_CTL_HWTRGCOND_Msk            (0x1ul << ADC_CTL_HWTRGCOND_Pos)                  /*!< ADC_T::CTL: HWTRGCOND Mask                */

#define ADC_CTL_HWTRGEN_Pos              (8)                                               /*!< ADC_T::CTL: HWTRGEN Position              */
#define ADC_CTL_HWTRGEN_Msk              (0x1ul << ADC_CTL_HWTRGEN_Pos)                    /*!< ADC_T::CTL: HWTRGEN Mask                  */

#define ADC_CTL_SWTRG_Pos                (11)                                              /*!< ADC_T::CTL: SWTRG Position                */
#define ADC_CTL_SWTRG_Msk                (0x1ul << ADC_CTL_SWTRG_Pos)                      /*!< ADC_T::CTL: SWTRG Mask                    */

#define ADC_CHEN_CHEN0_Pos               (0)                                               /*!< ADC_T::CHEN: CHEN0 Position               */
#define ADC_CHEN_CHEN0_Msk               (0x1ul << ADC_CHEN_CHEN0_Pos)                     /*!< ADC_T::CHEN: CHEN0 Mask                   */

#define ADC_CHEN_CHEN1_Pos               (1)                                               /*!< ADC_T::CHEN: CHEN1 Position               */
#define ADC_CHEN_CHEN1_Msk               (0x1ul << ADC_CHEN_CHEN1_Pos)                     /*!< ADC_T::CHEN: CHEN1 Mask                   */

#define ADC_CHEN_CHEN2_Pos               (2)                                               /*!< ADC_T::CHEN: CHEN2 Position               */
#define ADC_CHEN_CHEN2_Msk               (0x1ul << ADC_CHEN_CHEN2_Pos)                     /*!< ADC_T::CHEN: CHEN2 Mask                   */

#define ADC_CHEN_CHEN3_Pos               (3)                                               /*!< ADC_T::CHEN: CHEN3 Position               */
#define ADC_CHEN_CHEN3_Msk               (0x1ul << ADC_CHEN_CHEN3_Pos)                     /*!< ADC_T::CHEN: CHEN3 Mask                   */

#define ADC_CHEN_CHEN4_Pos               (4)                                               /*!< ADC_T::CHEN: CHEN4 Position               */
#define ADC_CHEN_CHEN4_Msk               (0x1ul << ADC_CHEN_CHEN4_Pos)                     /*!< ADC_T::CHEN: CHEN4 Mask                   */

#define ADC_CHEN_CHEN5_Pos               (5)                                               /*!< ADC_T::CHEN: CHEN5 Position               */
#define ADC_CHEN_CHEN5_Msk               (0x1ul << ADC_CHEN_CHEN5_Pos)                     /*!< ADC_T::CHEN: CHEN5 Mask                   */

#define ADC_CHEN_CHEN6_Pos               (6)                                               /*!< ADC_T::CHEN: CHEN6 Position               */
#define ADC_CHEN_CHEN6_Msk               (0x1ul << ADC_CHEN_CHEN6_Pos)                     /*!< ADC_T::CHEN: CHEN6 Mask                   */

#define ADC_CHEN_CHEN7_Pos               (7)                                               /*!< ADC_T::CHEN: CHEN7 Position               */
#define ADC_CHEN_CHEN7_Msk               (0x1ul << ADC_CHEN_CHEN7_Pos)                     /*!< ADC_T::CHEN: CHEN7 Mask                   */

#define ADC_CHEN_CHEN8_Pos               (8)                                               /*!< ADC_T::CHEN: CHEN8 Position               */
#define ADC_CHEN_CHEN8_Msk               (0x1ul << ADC_CHEN_CHEN8_Pos)                     /*!< ADC_T::CHEN: CHEN8 Mask                   */

#define ADC_CHEN_CH8SEL_Pos              (9)                                               /*!< ADC_T::CHEN: CH8SEL Position              */
#define ADC_CHEN_CH8SEL_Msk              (0x1ul << ADC_CHEN_CH8SEL_Pos)                    /*!< ADC_T::CHEN: CH8SEL Mask                  */

#define ADC_CMP0_ADCMPEN_Pos             (0)                                               /*!< ADC_T::CMP0: ADCMPEN Position             */
#define ADC_CMP0_ADCMPEN_Msk             (0x1ul << ADC_CMP0_ADCMPEN_Pos)                   /*!< ADC_T::CMP0: ADCMPEN Mask                 */

#define ADC_CMP0_ADCMPIE_Pos             (1)                                               /*!< ADC_T::CMP0: ADCMPIE Position             */
#define ADC_CMP0_ADCMPIE_Msk             (0x1ul << ADC_CMP0_ADCMPIE_Pos)                   /*!< ADC_T::CMP0: ADCMPIE Mask                 */

#define ADC_CMP0_CMPCOND_Pos             (2)                                               /*!< ADC_T::CMP0: CMPCOND Position             */
#define ADC_CMP0_CMPCOND_Msk             (0x1ul << ADC_CMP0_CMPCOND_Pos)                   /*!< ADC_T::CMP0: CMPCOND Mask                 */

#define ADC_CMP0_CMPCH_Pos               (3)                                               /*!< ADC_T::CMP0: CMPCH Position               */
#define ADC_CMP0_CMPCH_Msk               (0x7ul << ADC_CMP0_CMPCH_Pos)                     /*!< ADC_T::CMP0: CMPCH Mask                   */

#define ADC_CMP0_CMPMCNT_Pos             (8)                                               /*!< ADC_T::CMP0: CMPMCNT Position             */
#define ADC_CMP0_CMPMCNT_Msk             (0xful << ADC_CMP0_CMPMCNT_Pos)                   /*!< ADC_T::CMP0: CMPMCNT Mask                 */

#define ADC_CMP0_CMPDAT_Pos              (16)                                              /*!< ADC_T::CMP0: CMPDAT Position              */
#define ADC_CMP0_CMPDAT_Msk              (0xffful << ADC_CMP0_CMPDAT_Pos)                  /*!< ADC_T::CMP0: CMPDAT Mask                  */

#define ADC_CMP1_ADCMPEN_Pos             (0)                                               /*!< ADC_T::CMP1: ADCMPEN Position             */
#define ADC_CMP1_ADCMPEN_Msk             (0x1ul << ADC_CMP1_ADCMPEN_Pos)                   /*!< ADC_T::CMP1: ADCMPEN Mask                 */

#define ADC_CMP1_ADCMPIE_Pos             (1)                                               /*!< ADC_T::CMP1: ADCMPIE Position             */
#define ADC_CMP1_ADCMPIE_Msk             (0x1ul << ADC_CMP1_ADCMPIE_Pos)                   /*!< ADC_T::CMP1: ADCMPIE Mask                 */

#define ADC_CMP1_CMPCOND_Pos             (2)                                               /*!< ADC_T::CMP1: CMPCOND Position             */
#define ADC_CMP1_CMPCOND_Msk             (0x1ul << ADC_CMP1_CMPCOND_Pos)                   /*!< ADC_T::CMP1: CMPCOND Mask                 */

#define ADC_CMP1_CMPCH_Pos               (3)                                               /*!< ADC_T::CMP1: CMPCH Position               */
#define ADC_CMP1_CMPCH_Msk               (0x7ul << ADC_CMP1_CMPCH_Pos)                     /*!< ADC_T::CMP1: CMPCH Mask                   */

#define ADC_CMP1_CMPMCNT_Pos             (8)                                               /*!< ADC_T::CMP1: CMPMCNT Position             */
#define ADC_CMP1_CMPMCNT_Msk             (0xful << ADC_CMP1_CMPMCNT_Pos)                   /*!< ADC_T::CMP1: CMPMCNT Mask                 */

#define ADC_CMP1_CMPDAT_Pos              (16)                                              /*!< ADC_T::CMP1: CMPDAT Position              */
#define ADC_CMP1_CMPDAT_Msk              (0xffful << ADC_CMP1_CMPDAT_Pos)                  /*!< ADC_T::CMP1: CMPDAT Mask                  */

#define ADC_STATUS_ADIF_Pos              (0)                                               /*!< ADC_T::STATUS: ADIF Position              */
#define ADC_STATUS_ADIF_Msk              (0x1ul << ADC_STATUS_ADIF_Pos)                    /*!< ADC_T::STATUS: ADIF Mask                  */

#define ADC_STATUS_ADCMPIF0_Pos           (1)                                               /*!< ADC_T::STATUS: ADCMPF0 Position           */
#define ADC_STATUS_ADCMPIF0_Msk           (0x1ul << ADC_STATUS_ADCMPIF0_Pos)                 /*!< ADC_T::STATUS: ADCMPF0 Mask               */

#define ADC_STATUS_ADCMPIF1_Pos           (2)                                               /*!< ADC_T::STATUS: ADCMPF1 Position           */
#define ADC_STATUS_ADCMPIF1_Msk           (0x1ul << ADC_STATUS_ADCMPIF1_Pos)                 /*!< ADC_T::STATUS: ADCMPF1 Mask               */

#define ADC_STATUS_BUSY_Pos              (3)                                               /*!< ADC_T::STATUS: BUSY Position              */
#define ADC_STATUS_BUSY_Msk              (0x1ul << ADC_STATUS_BUSY_Pos)                    /*!< ADC_T::STATUS: BUSY Mask                  */

#define ADC_STATUS_CHANNEL_Pos           (4)                                               /*!< ADC_T::STATUS: CHANNEL Position           */
#define ADC_STATUS_CHANNEL_Msk           (0x7ul << ADC_STATUS_CHANNEL_Pos)                 /*!< ADC_T::STATUS: CHANNEL Mask               */

#define ADC_STATUS_VALID_Pos             (8)                                               /*!< ADC_T::STATUS: VALID Position             */
#define ADC_STATUS_VALID_Msk             (0x1ul << ADC_STATUS_VALID_Pos)                   /*!< ADC_T::STATUS: VALID Mask                 */

#define ADC_STATUS_OV_Pos                (16)                                              /*!< ADC_T::STATUS: OV Position                */
#define ADC_STATUS_OV_Msk                (0x1ul << ADC_STATUS_OV_Pos)                      /*!< ADC_T::STATUS: OV Mask                    */

#define ADC_STATUS_ADCF_Pos              (24)                                               /*!< ADC_T::STATUS: ADIF Position              */
#define ADC_STATUS_ADCF_Msk              (0x1ul << ADC_STATUS_ADCF_Pos)                    /*!< ADC_T::STATUS: ADIF Mask                  */

#define ADC_STATUS_ADCMPF0_Pos           (25)                                               /*!< ADC_T::STATUS: ADCMPF0 Position           */
#define ADC_STATUS_ADCMPF0_Msk           (0x1ul << ADC_STATUS_ADCMPF0_Pos)                 /*!< ADC_T::STATUS: ADCMPF0 Mask               */

#define ADC_STATUS_ADCMPF1_Pos           (26)                                               /*!< ADC_T::STATUS: ADCMPF1 Position           */
#define ADC_STATUS_ADCMPF1_Msk           (0x1ul << ADC_STATUS_ADCMPF1_Pos)                 /*!< ADC_T::STATUS: ADCMPF1 Mask               */

#define ADC_TRGDLY_DELAY_Pos             (0)                                               /*!< ADC_T::TRGDLY: DELAY Position             */
#define ADC_TRGDLY_DELAY_Msk             (0xfful << ADC_TRGDLY_DELAY_Pos)                  /*!< ADC_T::TRGDLY: DELAY Mask                 */

#define ADC_EXTSMPT_EXTSMPT_Pos          (0)                                               /*!< ADC_T::EXTSMPT: EXTSMPT Position          */
#define ADC_EXTSMPT_EXTSMPT_Msk          (0xful << ADC_EXTSMPT_EXTSMPT_Pos)                /*!< ADC_T::EXTSMPT: EXTSMPT Mask              */

#define ADC_SEL_VREF_Pos                 (21)                                              /*!< ADC_T::SEL_VREF Position          */
#define ADC_SEL_VREF_Msk                 (0x1ul << ADC_SEL_VREF_Pos)                       /*!< ADC_T::SEL_VREF Mask              */
#define ADC_SEQCTL_SEQEN_Pos             (0)                                               /*!< ADC_T::SEQCTL: SEQEN Position             */
#define ADC_SEQCTL_SEQEN_Msk             (0x1ul << ADC_SEQCTL_SEQEN_Pos)                   /*!< ADC_T::SEQCTL: SEQEN Mask                 */

#define ADC_SEQCTL_SEQTYPE_Pos           (1)                                               /*!< ADC_T::SEQCTL: SEQTYPE Position           */
#define ADC_SEQCTL_SEQTYPE_Msk           (0x1ul << ADC_SEQCTL_SEQTYPE_Pos)                 /*!< ADC_T::SEQCTL: SEQTYPE Mask               */

#define ADC_SEQCTL_MODESEL_Pos           (2)                                               /*!< ADC_T::SEQCTL: MODESEL Position           */
#define ADC_SEQCTL_MODESEL_Msk           (0x3ul << ADC_SEQCTL_MODESEL_Pos)                 /*!< ADC_T::SEQCTL: MODESEL Mask               */

#define ADC_SEQCTL_DELAY_EN_Pos          (4)                                               /*!< ADC_T::SEQCTL: DELAY_EN Position           */
#define ADC_SEQCTL_DELAY_EN_Msk          (0x1ul << ADC_SEQCTL_DELAY_EN_Pos)                 /*!< ADC_T::SEQCTL: DELAY_EN Mask               */

#define ADC_SEQCTL_TRG_SEL_Pos           (5)                                               /*!< ADC_T::SEQCTL: TRG_SEL Position           */
#define ADC_SEQCTL_TRG_SEL_Msk           (0x1ul << ADC_SEQCTL_TRG_SEL_Pos)                 /*!< ADC_T::SEQCTL: TRG_SEL Mask               */

#define ADC_SEQCTL_TRG1CTL_Pos           (8)                                               /*!< ADC_T::SEQCTL: TRG1CTL Position           */
#define ADC_SEQCTL_TRG1CTL_Msk           (0xful << ADC_SEQCTL_TRG1CTL_Pos)                 /*!< ADC_T::SEQCTL: TRG1CTL Mask               */

#define ADC_SEQCTL_TRG2CTL_Pos           (16)                                              /*!< ADC_T::SEQCTL: TRG2CTL Position           */
#define ADC_SEQCTL_TRG2CTL_Msk           (0xful << ADC_SEQCTL_TRG2CTL_Pos)                 /*!< ADC_T::SEQCTL: TRG2CTL Mask               */

#define ADC_SEQDAT1_RESULT_Pos           (0)                                               /*!< ADC_T::SEQDAT1: RESULT Position           */
#define ADC_SEQDAT1_RESULT_Msk           (0x3fful << ADC_SEQDAT1_RESULT_Pos)               /*!< ADC_T::SEQDAT1: RESULT Mask               */

#define ADC_SEQDAT1_OV_Pos               (16)                                              /*!< ADC_T::SEQDAT1: OV Position               */
#define ADC_SEQDAT1_OV_Msk               (0x1ul << ADC_SEQDAT1_OV_Pos)                     /*!< ADC_T::SEQDAT1: OV Mask                   */

#define ADC_SEQDAT1_VALID_Pos            (17)                                              /*!< ADC_T::SEQDAT1: VALID Position            */
#define ADC_SEQDAT1_VALID_Msk            (0x1ul << ADC_SEQDAT1_VALID_Pos)                  /*!< ADC_T::SEQDAT1: VALID Mask                */

#define ADC_SEQDAT2_RESULT_Pos           (0)                                               /*!< ADC_T::SEQDAT2: RESULT Position           */
#define ADC_SEQDAT2_RESULT_Msk           (0x3fful << ADC_SEQDAT2_RESULT_Pos)               /*!< ADC_T::SEQDAT2: RESULT Mask               */

#define ADC_SEQDAT2_OV_Pos               (16)                                              /*!< ADC_T::SEQDAT2: OV Position               */
#define ADC_SEQDAT2_OV_Msk               (0x1ul << ADC_SEQDAT2_OV_Pos)                     /*!< ADC_T::SEQDAT2: OV Mask                   */

#define ADC_SEQDAT2_VALID_Pos            (17)                                              /*!< ADC_T::SEQDAT2: VALID Position            */
#define ADC_SEQDAT2_VALID_Msk            (0x1ul << ADC_SEQDAT2_VALID_Pos)                  /*!< ADC_T::SEQDAT2: VALID Mask                */

#define ADC_CTL2_TESTMODE_Pos            (0)                                               /*!< ADC_T::DAT: RESULT Position               */
#define ADC_CTL2_TESTMODE_Msk            (0x1ul << ADC_CTL2_TESTMODE_Pos)                   /*!< ADC_T::DAT: RESULT Mask                   */

#define ADC_CTL2_SHSEL_Pos            	 (1)                                               /*!< ADC_T::DAT: RESULT Position               */
#define ADC_CTL2_SHSEL_Msk            	 (0x1ul << ADC_CTL2_SHSEL_Pos)                   /*!< ADC_T::DAT: RESULT Mask                   */

#define ADC_CTL2_CLKDIV_Pos              (16)                                               /*!< ADC_T::DAT: RESULT Position               */
#define ADC_CTL2_CLKDIV_Msk              (0xful << ADC_CTL2_CLKDIV_Pos)                    /*!< ADC_T::DAT: RESULT Mask                   */

#define ADC_CTL2_CMPCTL_Pos              (24)                                               /*!< ADC_T::DAT: RESULT Position               */
#define ADC_CTL2_CMPCTL_Msk              (0x3ul << ADC_CTL2_CMPCTL_Pos)                    /*!< ADC_T::DAT: RESULT Mask                   */

#define ADC_CTL2_DRVCTL_Pos              (26)                                               /*!< ADC_T::DAT: RESULT Position               */
#define ADC_CTL2_DRVCTL_Msk              (0x3ul << ADC_CTL2_DRVCTL_Pos)                    /*!< ADC_T::DAT: RESULT Mask                   */
/**@}*/ /* ADC_CONST */
/**@}*/ /* end of ADC register group */


/*---------------------- System Clock Controller -------------------------*/
/**
    @addtogroup CLK System Clock Controller(CLK)
    Memory Mapped Structure for CLK Controller
@{ */

typedef struct {

    /**
        * PWRCTL
        * ===================================================================================================
        * Offset: 0x00  System Power-down Control Register
        * ---------------------------------------------------------------------------------------------------
        * |Bits    |Field     |Descriptions
        * | :----: | :----:   | :---- |
        * |[0]     |PDXTL     |External HXT Crystal Oscillator Disable Bit (Write Protect)
        * |        |          |The default clock source is from HIRC.
        * |        |          |0 = HXT Power up
        * |        |          |1 = XT1_IN and XT1_OUT are GPIO, disable both HXT (default).
        * |        |          |Note: To enable external XTAL function, ALT[1:0] and MFP[1:0] bits must also be set in SYS_P5_MFP.
        * |[1]     |PDXTLBUF  |External HXT Crystal Oscillator Enable Bit (Write Protect)
        * |        |          |0 = Budder enable
        * |        |          |1 = Buffer disable (default)
        * |[2]     |PDHIRC    |HIRC Disable Bit (Write Protect)
        * |        |          |0 = 48 MHz internal high speed RC oscillator (HIRC) Enabled.
        * |        |          |1 = 48 MHz internal high speed RC oscillator (HIRC) Disabled.
        * |        |          |Note: The default of HIRCEN bit is 0.
        * |[3]     |PDLIRC    |LIRC Disable Bit (Write Protect)
        * |        |          |0 = 10 kHz internal low speed RC oscillator (LIRC) Enabled.
        * |        |          |1 = 10 kHz internal low speed RC oscillator (LIRC) Disabled.
        * |        |          |Note: The default of LIRCEN bit is 0.
        * |[5]     |PDWKIEN   |Power-down Mode Wake-up Interrupt Enable Bit (Write Protect)
        * |        |          |0 = Power-down mode wake-up interrupt Disabled.
        * |        |          |1 = Power-down mode wake-up interrupt Enabled.
        * |        |          |Note: The interrupt will occur when both PDWKIF and PDWKIEN are high.
        * |[6]     |PDWKIF    |Power-down Mode Wake-up Interrupt Status
        * |        |          |Set by "Power-down wake-up event", which indicates that resume from Power-down mode
        * |        |          |The flag is set if the GPIO, UART, WDT, ACMP, Timer or BOD wake-up occurred.
        * |        |          |Note: This bit works only if PDWKIEN (CLK_PWRCTL[5]) set to 1. Write 1 to clear the bit to 0.
        * |[7]     |PDEN      |System Power-down Enable Bit (Write Protect)
        * |        |          |When chip wakes up from Power-down mode, this bit is cleared by hardware.
        * |        |          |User needs to set this bit again for next Power-down.
        * |        |          |In Power-down mode, 16 MHz external high speed crystal oscillator (HXT), and the 48 MHz internal high speed oscillator (HIRC) will be disabled in this mode, and 10 kHz internal low speed RC oscillator (LIRC) are not controlled by Power-down mode.
        * |        |          |In Power-down mode, the PLL and system clock are disabled, and ignored the clock source selection. 
        * |        |          |The clocks of peripheral are not controlled by Power-down mode, if the peripheral clock source is from 10 kHz internal low speed oscillator.
        * |        |          |0 = Chip operating normally or chip in Idle mode because of WFI command.
        * |        |          |1 = Chip enters Power-down mode instantly or waits CPU sleep command WFI.
        * |[8]     |PDIBG     |IBG Disable Bit (Write Protect)
        * |        |          |0 = IBG Power Up
        * |        |          |1 = IBG Power down
        * |        |          |Note: The default of PDIBG bit is 0.
        * |[10     |PDVD18    |VD18 Disable Bit (Write Protect)
        * |        |          |0 = VD18 Power Up
        * |        |          |1 = VD18 Power down
        * |        |          |Note: The default of PDIBG bit is 0.
        * |[13:12] |LDO_SELLL| LDO voltage control (Write Protect) 
        * |        |          |        Default : 2��b11     
        * |[15:14] |LDO_SELH|LDO voltage control (Write Protect)
        * |        |          |        Default : 2��b00             
        * |[23:16] |FREQ50M_CTL| 48M internal high speed RC oscillator frequency control
        * |        |          |default value ?
        * |[27:24] |FREQ10K_CTL| 10K internal  low speed RC oscillator frequency control
        *  |        |          |default value 
        * |[29:28] |ICTL_OSC_OP1| 48M internal high speed RC oscillator current control1
        * |        |          |default value 01
    */

    __IO uint32_t PWRCTL;

    /**
     * AHBCLK
     * ===================================================================================================
     * Offset: 0x04  AHB Devices Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2]     |ISPCKEN   |Flash ISP Controller Clock Enable Bit (ReadOnly)
     * |        |          |0 = Flash ISP peripheral clock Disabled.
     * |        |          |1 = Flash ISP peripheral clock Enabled.
     */
    __IO uint32_t AHBCLK;

    /**
    * Offset: 0x08  APB Devices Clock Enable Control Register
    * APBCLK
    * ===================================================================================================
    * ---------------------------------------------------------------------------------------------------
    * |Bits    |Field             |Descriptions
    * | :---: | :----:          | :---- |
    * |[0]    |WDTCKEN      |Watchdog Timer Clock Enable Bit (Write Protect)
    * |        |                   |0 = Watchdog Timer clock Disabled.
    * |        |                     |1 = Watchdog Timer clock Enabled.
    * |        |          |Note: This bit is the protected bit, and programming it needs to write 0x59, 0x16, and 0x88 to address 0x5000_0100 to disable register protection.
    * |        |          |Refer to the register SYS_REGLCTL at address SYS_BA + 0x10.
    * |[2]     |TMR0CKEN  |Timer0 Clock Enable Bit
    * |        |          |0 = Timer0 clock Disabled.
    * |        |          |1 = Timer0 clock Enabled.
    * |[3]     |TMR1CKEN  |Timer1 Clock Enable Bit
    * |        |          |0 = Timer1 clock Disabled.
    * |        |          |1 = Timer1 clock Enabled.
    * |[4]     |TMR2CKEN  |Timer2 Clock Enable Bit
    * |        |          |0 = Timer2 clock Disabled.
    * |        |          |1 = Timer2 clock Enabled.
    * |[6]     |CLKOCKEN  |Frequency Divider Output Clock Enable Bit
    * |        |          |0 = CLKO clock Disabled.
    * |        |          |1 = CLKO clock Enabled.
    * |[8]     |I2C0CKEN  |I2C0 Clock Enable Bit
    * |        |          |0 = I2C0 clock Disabled.
    * |        |          |1 = I2C0 clock Enabled.
    * |[9]     |I2C1CKEN  |I2C1 Clock Enable Bit
    * |        |          |0 = I2C1 clock Disabled.
    * |        |          |1 = I2C1 clock Enabled.
    * |[12]    |SPI0CKEN  |SPI0 Clock Enable Bit
    * |        |          |0 = SPI0 peripheral clock Disabled.
    * |        |          |1 = SPI0 peripheral clock Enabled.
    * |[13]    |SPI1CKEN  |SPI1 Clock Enable Bit
    * |        |          |0 = SPI1 peripheral clock Disabled.
    * |        |          |1 = SPI1 peripheral clock Enabled.
    * |[16]    |UART0CKEN |UART0 Clock Enable Bit
     * |        |          |0 = UART0 clock Disabled.
     * |        |          |1 = UART0 clock Enabled.
     * |[17]    |UART1CKEN |UART1 Clock Enable Bit
     * |        |          |0 = UART1 clock Disabled.
     * |        |          |1 = UART1 clock Enabled.
     * |[20]    |PWMCH01CKEN|PWM_01 Clock Enable Bit
     * |        |          |0 = PWM01 clock Disabled.
     * |        |          |1 = PWM01 clock Enabled.
     * |[21]    |PWMCH23CKEN|PWM_23 Clock Enable Bit
     * |        |          |0 = PWM23 clock Disabled.
     * |        |          |1 = PWM23 clock Enabled.
     * |[22]    |PWMCH45CKEN|PWM_45 Clock Enable Bit
     * |        |          |0 = PWM45 clock Disabled.
     * |        |          |1 = PWM45 clock Enabled.
    * |[23]    |PWMCH67CKEN|PWM_67 Clock Enable Bit
    * |        |          |0 = PWM67 clock Disabled.
    * |        |          |1 = PWM67 clock Enabled.
     * |[28]    |ADCCKEN   |Analog-digital-converter (ADC) Clock Enable Bit
     * |        |          |0 = ADC peripheral clock Disabled.
     * |        |          |1 = ADC peripheral clock Enabled.
     * |[30]    |ACMPCKEN  |Analog Comparator Clock Enable Bit
     * |        |          |0 = Analog Comparator clock Disabled.
     * |        |          |1 = Analog Comparator clock Enabled.
    */
    __IO uint32_t APBCLK;

   /**
    * STATUS
    * ===================================================================================================
    * Offset: 0x0C  Clock Status Monitor Register
    * ---------------------------------------------------------------------------------------------------
    * |Bits    |Field     |Descriptions
    * | :----: | :----:   | :---- |
    * |[0]     |XTLSTB    |HXT Or LXT Clock Source Stable Flag
    * |        |          |0 = HXT or LXT clock is not stable or disabled.
    * |        |          |1 = HXT or LXT clock is stable and enabled.
    * |[1]     |IBGSTB    |IBG Stable Flag (Read Only)
    * |        |          |0 = IBG is not stable or disabled.
    * |        |          |1 = IBG is stable and enabled.  
    * |[2]     |PLLSTB    |Internal PLL Clock Source Stable Flag (Read Only)
    * |        |          |0 = Internal PLL clock is not stable or disabled.
    * |        |          |1 = Internal PLL clock is stable and enabled.
    * |[3]     |LIRCSTB   |LIRC Clock Source Stable Flag (Read Only)
    * |        |          |0 = LIRC clock is not stable or disabled.
    * |        |          |1 = LIRC clock is stable and enabled.
    * |[4]     |HIRCSTB   |HIRC Clock Source Stable Flag (Read Only)
    * |        |          |0 = HIRC clock is not stable or disabled.
    * |        |          |1 = HIRC clock is stable and enabled.
    */
    __IO uint32_t STATUS;

    /**
    * CLKSEL0
    * ===================================================================================================
    * Offset: 0x10  Clock Source Select Control Register 0
    * ---------------------------------------------------------------------------------------------------
    * |Bits    |Field     |Descriptions
    * | :----: | :----:   | :---- |
    * |[0:1]     |HCLKSEL   |HCLK Clock Source Selection (it is STCLKSEL too) (Write Protect)
    * |        |          |00 = Clock source is from HIRC..
    * |        |          |01 = Clock source is from PLL.L.
    * |        |          |10 = Clock source is from LIRC.
    * |        |          |11 = Clock source is from HXT.
    * |        |          |Note1: Before clock switching, the related clock sources (both pre-select and new-select) must be turn-on and stable.
    * |        |          |Note2: These bits are protected bit, and programming them needs to write 0x59, 0x16, and 0x88 to address 0x5000_0100 to disable register protection.
    * |        |          |Refer to the register SYS_REGLCTL at address SYS_BA + 0x10.
    * |        |          |Note3: To set CLK_PWRCTL[1:0] to select HXTT crystal clock.
    */
    __IO uint32_t CLKSEL0;

    /**
    * CLKSEL1
    * ===================================================================================================
    * Offset: 0x14  Clock Source Select Control Register 1
    * ---------------------------------------------------------------------------------------------------
    * |Bits    |Field     |Descriptions
    * | :----: | :----:   | :---- |
    * |[1:0]   |WDTSEL    |WDT CLK Clock Source Selection (Write Protect)
    * |        |          |00 = Clock source is from HXT or LXT.
    * |        |          |01 = Reserved.
    * |        |          |10 = Clock source is from HCLK/2048 clock.
    * |        |          |11 = Clock source is from LIRC.
    * |        |          |Note1: These bits are the protected bit, and programming them needs to write 0x59, 0x16, and 0x88 to address 0x5000_0100 to disable register protection.
    * |        |          |Refer to the register SYS_REGLCTL at address SYS_BA + 0x10.
    * |        |          |Note2: To set CLK_PWRCTL[1:0], select HXT or LXT crystal clock.
    * |[3:2]   |ADCSEL    |ADC Peripheral Clock Source Selection
    * |        |          |00 = Clock source is from HXT or LXT.
    * |        |          |01 = Clock source is from PLL.
    * |        |          |10 = Clock source is from HCLK.
    * |        |          |11 = Clock source is from HIRC.
    * |        |          |Note: To set CLK_PWRCTL[1:0], select HXT or LXT crystal clock.
    * |[5:4]   |SPI0SEL   |SPI0 Clock Source Selection
    * |        |          |00 = Clock source is from HXT or LXT.
    * |        |          |01 = Clock source is from HCLK.
    * |        |          |10 = Clock source is from PLL.
    * |        |          |11 = Reserved.
    * |        |          |Note: To set CLK_PWRCTL[1:0], select HXT or LXT crystal clock.
    * |[7:6]   |SPI1SEL   |SPI1 Clock Source Selection
    * |        |          |00 = Reserved.
    * |        |          |01 = Clock source is from HCLK.
    * |        |          |10 = Reserved.
    * |        |          |11 = Reserved.
    * |        |          |Note: To set CLK_PWRCTL[1:0], select HXT or LXT crystal clock.
    * |[10:8]  |TMR0SEL   |TIMER0 Clock Source Selection
    * |        |          |000 = Clock source is from HCLK.
    * |        |          |001 = Clock source is from LIRC.
    * |        |          |010 = Clock source is from HCLK.
    * |        |          |011 = Clock source is from external trigger.
    * |        |          |111 = Clock source is from HIRC.
    * |        |          |Others = Reserved.
    * |        |          |Note: To set CLK_PWRCTL[1:0], select HXT or LXT crystal clock.
    * |[14:12] |TMR1SEL   |TIMER1 Clock Source Selection
    * |        |          |000 = Clock source is from HCLK.
    * |        |          |001 = Clock source is from LIRC.
    * |        |          |010 = Clock source is from HCLK.
    * |        |          |011 = Clock source is from external trigger.
    * |        |          |111 = Clock source is from HIRC.
    * |        |          |Others = Reserved.
    *|        |          |Note: To set CLK_PWRCTL[1:0], select HXT or LXT crystal clock.
    * |[18:16] |TMR2SEL   |TIMER2 Clock Source Selection
    * |        |          |000 = Clock source is from HCLK.
    * |        |          |001 = Clock source is from LIRC.
    * |        |          |010 = Clock source is from HCLK.
    * |        |          |011 = Clock source is from external trigger.
    * |        |          |111 = Reserved.
    * |        |          |Others = Reserved.
    * |        |          |Note: To set CLK_PWRCTL[1:0], select HXT or LXT crystal clock.
    * |[25:24] |UARTSEL   |UART Clock Source Selection
    * |        |          |00 = Clock source is from HXT or LXT.
    * |        |          |01 = Clock source is from PLL.
    * |        |          |10 = Clock source is from HCLK.
    * |        |          |11 = Clock source is from HCLK. * |        |          |Note: To set CLK_PWRCTL[1:0], select HXT or LXT crystal clock.
    * |[29:28] |PWMCH01SEL|PWM0 And PWM1 Clock Source Selection
    * |        |          |PWM0 and PWM1 use the same peripheral clock source. Both of them use the same prescaler.
    * |        |          |00 = Reserved.
    * |        |          |01 = Reserved.
    * |        |          |10 = Clock source is from HCLK.
    * |        |          |11 = Reserved.
    * |[31:30] |PWMCH23SEL|PWM2 And PWM3 Clock Source Selection
    * |        |          |PWM2 and PWM3 use the same peripheral clock source; Both of them use the same prescaler.
    * |        |          |00 = Reserved.
    * |        |          |01 = Reserved.
    * |        |          |10 = Clock source is from HCLK.
    * |        |          |11 = Reserved.
   */
   __IO uint32_t CLKSEL1;

    /**
    * CLKDIV
    * ===================================================================================================
    * Offset: 0x18  Clock Divider Number Register
    * ---------------------------------------------------------------------------------------------------
    * |Bits    |Field     |Descriptions
    * | :----: | :----:   | :---- |
    * |[1:0]   |HCLKDIV   |HCLK Clock Divide Number From HCLK Clock Source
    * |        |          |00 = HCLK clock frequency = (HCLK clock source frequency).
    * |        |          |01 = HCLK clock frequency = (HCLK clock source frequency) / 2.
    * |        |          |10 = HCLK clock frequency = (HCLK clock source frequency) / 4. 
    * |        |          |11 = HCLK clock frequency = (HCLK clock source frequency) / 8.
    */
 __IO uint32_t CLKDIV;

    /**
    * CLKSEL2
    * ===================================================================================================
    * Offset: 0x1C  Clock Source Select Control Register 2
    * ---------------------------------------------------------------------------------------------------
    * |Bits    |Field     |Descriptions
    * | :----: | :----:   | :---- |
    * |[3:2]   |CLKOSEL   |Clock Divider Clock Source Selection
    * |        |          |00 = Clock source is from PLL or HIRC divide 8.
    * |        |          |01 = Clock source is from LIRC.
    * |        |          |10 = Clock source is from PLL.
    * |        |          |11 = Clock source is from HIRC.
    * |        |          |Note: To set CLK_PWRCTL[1:0], select HXT or LXT crystal clock.
    * |[5:4]   |PWMCH45SEL|PWM4 And PWM5 Clock Source Selection
    * |        |          |PWM4 and PWM5 use the same peripheral clock source; Both of them use the same prescaler.
    * |        |          |00 = Reserved.
    * |        |          |01 = Reserved.
    * |        |          |10 = Clock source is from HCLK.
    * |        |          |11 = Reserved.
    * |[7:6]   |PWMCH67SEL|PWM6 And PWM7 Clock Source Selection
    * |        |          |PWM6 and PWM7 use the same peripheral clock source; Both of them use the same prescaler.
    * |        |          |00 = Reserved.
    * |        |          |01 = Reserved.
    * |        |          |10 = Clock source is from HCLK.
    * |        |          |11 = Reserved.
    * |[17:16] |WWDTSEL   |Window Watchdog Timer Clock Source Selection
    * |        |          |00 = Reserved.
    * |        |          |01 = Reserved.
    * |        |          |10 = Clock source from HCLK/2048 clock.
    * |        |          |11 = Clock source from 10 kHz internal low speed RC oscillator (LIRC).
     */
    __IO uint32_t CLKSEL2;

    /**
    * PLLCTL
    * ===================================================================================================
    * Offset: 0x20  PLL Control Register
    * ---------------------------------------------------------------------------------------------------
    * |Bits    |Field     |Descriptions
    * | :----: | :----:   | :---- |
    * |[16]    |PD        |Power-down Mode
    * |        |          |If the PDEN bit is set to 1 in CLK_PWRCTL register, the PLL will enter Power-down mode too.
    * |        |          |0 = PLL is in Normal mode.
    * |        |          |1 = PLL is in Power-down mode (default).
    * |[19]    |PLLSRC    |PLL Source Clock Selection
    * |        |          |0 = PLL source clock from HXT.
    * |        |          |1 = PLL source clock from Extarnal Clock.
    * |[20]    |PLLINFSEL |PLL Source Clock frequeny select to match PLL
    * |        |          |0 = when PLL source is External and Freq is 4MHz, or PLL sorce is HXT
    * |        |          |1 = when PLL source is External and Freq is 8MHz
    * |[23:22] |ICLT_DPLL |PLL current control
    * |        |          |00 =
    * |        |          |01 =
    * |        |          |10 = default
    * |        |          |11 =
    */
    __IO uint32_t PLLCTL;


    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVED0[1];
    /// @endcond //HIDDEN_SYMBOLS


    /**
     * WAKEUPCTL
     * ===================================================================================================
     * Offset: 0x28  Wakeup control
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
	 * |[5:0]   |XTLWUSEL  |XTL stable time select
	 * |        |          |bit0: =1, 2^5
	 * |        |          |bit1: =1, 2^7
	 * |        |          |bit2: =1, 2^9
	 * |        |          |bit3: =1, 2^11
	 * |        |          |bit4: =1, 2^13
	 * |        |          |bit5: =1, 2^15
	 * |[13:8]  |IBGWUSEL  |IBG stable tim
	 * |        |          |bit8: =1, 2^8. source clock
	 * |        |          |bit9: =1, 2^9
	 * |        |          |bit10: =1, 2^10
	 * |        |          |bit11: =1, 2^11
	 * |        |          |bit12: =1, 2^13
	 * |        |          |bit13: =1, 2^15
	 * |[21:16] |PLLWUSEL  |PLL stable tim
	 * |        |          |bit16: =1, 2^5. pll clock
	 * |        |          |bit17: =1, 2^7
	 * |        |          |bit18: =1, 2^9
	 * |        |          |bit19: =1, 2^11
	 * |        |          |bit20: =1, 2^13
	 * |        |          |bit21: =1, 2^15
	 * |[29:24] |HIRCWUSEL |HIRC stable tim
	 * |        |          |bit24: =1, 2^5. HIRC clock
	 * |        |          |bit25: =1, 2^7
	 * |        |          |bit26: =1, 2^9
	 * |        |          |bit27: =1, 2^11
	 * |        |          |bit28: =1, 2^13
	 * |        |          |bit29: =1, 2^15

    */
    __IO uint32_t WAKEUPCTL;
	
	
    /**
     * LPCTL	Lowpower control
     * ===================================================================================================
     * Offset: 0x2C  Wakeup control
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
	 * |[0]   	|PDXTL	   |1: Power down XTL in power-down mode
	 * |        |          |0: Not Power down XTL in power-down mode
	 * |[1]   	|PDIBG	   |1: Power down BIG in power-down mode
	 * |        |          |0: Not Power down IBG in power-down mode
	 * |[2]   	|PDPLL	   |1: Power down PLL in power-down mode
	 * |        |          |0: Not Power down PLL in power-down mode
	 * |[3]   	|PDHIRC	   |1: Power down HIRC in power-down mode
	 * |        |          |0: Not Power down HIRC in power-down mode
	 * |[4]   	|PDLIRC	   |1: Power down LIRC in power-down mode
	 * |        |          |0: Not Power down LIRC in power-down mode
	 * |[5]   	|PDVD18	   |1: Power down VD18 in power-down mode
	 * |        |          |0: Not Power down VD18 in power-down mode
	 * |[6]   	|PDADC	   |1: Power down ADC in power-down mode
	 * |        |          |0: Not Power down ADC in power-down mode
	 * |[7]   	|PDACMP	   |1: Power down ACMP in power-down mode
	 * |        |          |0: Not Power down ACMP in power-down mode
	 * |[8]   	|PDBOD	   |1: Power down BOD in power-down mode
	 * |        |          |0: Not Power down BOD in power-down mode
	 * |[9]   	|PDLVR	   |1: Power down LVR in power-down mode
	 * |        |          |0: Not Power down LVR in power-down mode
	 */
	__IO uint32_t LPCTL;

} CLK_T;

/**
    @addtogroup CLK_CONST CLK Bit Field Definition
    Constant Definitions for CLK Controller
@{ */

#define CLK_PWRCTL_PDXTL_Pos             (0)                                               /*!< CLK_T::PWRCTL: XTLEN Position             */
#define CLK_PWRCTL_PDXTL_Msk             (0x1ul << CLK_PWRCTL_PDXTL_Pos)                   /*!< CLK_T::PWRCTL: XTLEN Mask                 */

#define CLK_PWRCTL_PDXTLBUF_Pos          (1)                                               /*!< CLK_T::PWRCTL: XTLEN Position             */
#define CLK_PWRCTL_PDXTLBUF_Msk          (0x1ul << CLK_PWRCTL_PDXTLBUF_Pos)                   /*!< CLK_T::PWRCTL: XTLEN Mask                 */

#define CLK_PWRCTL_PDHIRC_Pos            (2)                                               /*!< CLK_T::PWRCTL: HIRCEN Position            */
#define CLK_PWRCTL_PDHIRC_Msk            (0x1ul << CLK_PWRCTL_PDHIRC_Pos)                  /*!< CLK_T::PWRCTL: HIRCEN Mask                */

#define CLK_PWRCTL_PDLIRC_Pos            (3)                                               /*!< CLK_T::PWRCTL: LIRCEN Position            */
#define CLK_PWRCTL_PDLIRC_Msk            (0x1ul << CLK_PWRCTL_PDLIRC_Pos)                  /*!< CLK_T::PWRCTL: LIRCEN Mask                */
#define CLK_PWRCTL_PDWKIEN_Pos           (5)                                               /*!< CLK_T::PWRCTL: PDWKIEN Position           */
#define CLK_PWRCTL_PDWKIEN_Msk           (0x1ul << CLK_PWRCTL_PDWKIEN_Pos)                 /*!< CLK_T::PWRCTL: PDWKIEN Mask               */

#define CLK_PWRCTL_PDWKIF_Pos            (6)                                               /*!< CLK_T::PWRCTL: PDWKIF Position            */
#define CLK_PWRCTL_PDWKIF_Msk            (0x1ul << CLK_PWRCTL_PDWKIF_Pos)                  /*!< CLK_T::PWRCTL: PDWKIF Mask                */

#define CLK_PWRCTL_PDEN_Pos              (7)                                               /*!< CLK_T::PWRCTL: PDEN Position              */
#define CLK_PWRCTL_PDEN_Msk              (0x1ul << CLK_PWRCTL_PDEN_Pos)                    /*!< CLK_T::PWRCTL: PDEN Mask                  */

#define CLK_PWRCTL_PDIBG_Pos             (8)                                               /*!< CLK_T::PWRCTL: PDIBGPosition              */
#define CLK_PWRCTL_PDIBG_Msk             (0x1ul << CLK_PWRCTL_PDIBG_Pos)                    /*!< CLK_T::PWRCTL: PDIBG Mask                  */

#define CLK_PWRCTL_PDVD18_Pos            (10)                                               /*!< CLK_T::PWRCTL: PDEN Position              */
#define CLK_PWRCTL_PDVD18_Msk            (0x1ul <<CLK_PWRCTL_PDVD18_Pos)                    /*!< CLK_T::PWRCTL: PDEN Mask                  */

#define CLK_PWRCTL_LDO_SELL_Pos            (12)                                               /*!< CLK_T::PWRCTL:LDO_SELL Position              */
#define CLK_PWRCTL_LDO_SELL_Msk            (0x3ul <<CLK_PWRCTL_LDO_SELL_Pos)                    /*!< CLK_T::PWRCTL: LDO_SELL Mask                  */

#define CLK_PWRCTL_LDO_SELH_Pos            (14)                                               /*!< CLK_T::PWRCTL:LDO_SELH Position              */
#define CLK_PWRCTL_LDO_SELH_Msk            (0x3ul <<CLK_PWRCTL_LDO_SELH_Pos)                    /*!< CLK_T::PWRCTL: LDO_SELH Mask                  */

#define CLK_PWRCTL_FREQ48MCTL_Pos        (16)                                               /*!< CLK_T::PWRCTL: FREQ48MCTL Position              */
#define CLK_PWRCTL_FREQ48MCTL_Msk        (0xfful << CLK_PWRCTL_FREQ48MCTL_Pos)                    /*!< CLK_T::PWRCTL: FREQ48MCTL Mask                  */

#define CLK_PWRCTL_FREQ10KCTL_Pos        (24)                                               /*!< CLK_T::PWRCTL: PDEN Position              */
#define CLK_PWRCTL_FREQ10KCTL_Msk        (0xful << CLK_PWRCTL_FREQ10KCTL_Pos)                    /*!< CLK_T::PWRCTL: PDEN Mask                  */

#define CLK_PWRCTL_ICTLOSCOP1_Pos        (28)                                               /*!< CLK_T::PWRCTL: PDEN Position              */
#define CLK_PWRCTL_ICTLOSCOP1_Msk        (0x3ul <<CLK_PWRCTL_ICTLOSCOP1_Pos)                    /*!< CLK_T::PWRCTL: PDEN Mask                  */


#define CLK_AHBCLK_ISPCKEN_Pos           (2)                                               /*!< CLK_T::AHBCLK: ISPCKEN Position           */
#define CLK_AHBCLK_ISPCKEN_Msk           (0x1ul << CLK_AHBCLK_ISPCKEN_Pos)                 /*!< CLK_T::AHBCLK: ISPCKEN Mask               */

#define CLK_APBCLK_WDTCKEN_Pos           (0)                                               /*!< CLK_T::APBCLK: WDTCKEN /WWDTCKEN Position           */
#define CLK_APBCLK_WDTCKEN_Msk           (0x1ul << CLK_APBCLK_WDTCKEN_Pos)                 /*!< CLK_T::APBCLK: WDTCKEN /WWDTCKEN Mask               */

#define CLK_APBCLK_TMR0CKEN_Pos          (2)                                               /*!< CLK_T::APBCLK: TMR0CKEN Position          */
#define CLK_APBCLK_TMR0CKEN_Msk          (0x1ul << CLK_APBCLK_TMR0CKEN_Pos)                /*!< CLK_T::APBCLK: TMR0CKEN Mask              */

#define CLK_APBCLK_TMR1CKEN_Pos          (3)                                               /*!< CLK_T::APBCLK: TMR1CKEN Position          */
#define CLK_APBCLK_TMR1CKEN_Msk          (0x1ul << CLK_APBCLK_TMR1CKEN_Pos)                /*!< CLK_T::APBCLK: TMR1CKEN Mask              */

#define CLK_APBCLK_TMR2CKEN_Pos          (4)                                               /*!< CLK_T::APBCLK: TMR1CKEN Position          */
#define CLK_APBCLK_TMR2CKEN_Msk          (0x1ul << CLK_APBCLK_TMR2CKEN_Pos)                /*!< CLK_T::APBCLK: TMR1CKEN Mask              */

#define CLK_APBCLK_CLKOCKEN_Pos          (6)                                               /*!< CLK_T::APBCLK: CLKOCKEN Position          */
#define CLK_APBCLK_CLKOCKEN_Msk          (0x1ul << CLK_APBCLK_CLKOCKEN_Pos)                /*!< CLK_T::APBCLK: CLKOCKEN Mask              */

#define CLK_APBCLK_I2C0CKEN_Pos          (8)                                               /*!< CLK_T::APBCLK: I2C0CKEN Position          */
#define CLK_APBCLK_I2C0CKEN_Msk          (0x1ul << CLK_APBCLK_I2C0CKEN_Pos)                /*!< CLK_T::APBCLK: I2C0CKEN Mask              */

#define CLK_APBCLK_I2C1CKEN_Pos          (9)                                               /*!< CLK_T::APBCLK: I2C1CKEN Position          */
#define CLK_APBCLK_I2C1CKEN_Msk          (0x1ul << CLK_APBCLK_I2C1CKEN_Pos)                /*!< CLK_T::APBCLK: I2C1CKEN Mask              */

#define CLK_APBCLK_SPI0CKEN_Pos           (12)                                              /*!< CLK_T::APBCLK: SPI0CKEN Position           */
#define CLK_APBCLK_SPI0CKEN_Msk           (0x1ul << CLK_APBCLK_SPI0CKEN_Pos)                 /*!< CLK_T::APBCLK: SPI0CKEN Mask               */

#define CLK_APBCLK_SPI1CKEN_Pos           (13)                                              /*!< CLK_T::APBCLK: SPI1CKEN Position           */
#define CLK_APBCLK_SPI1CKEN_Msk           (0x1ul << CLK_APBCLK_SPI1CKEN_Pos)                 /*!< CLK_T::APBCLK: SPI1CKEN Mask               */

#define CLK_APBCLK_UART0CKEN_Pos         (16)                                              /*!< CLK_T::APBCLK: UART0CKEN Position         */
#define CLK_APBCLK_UART0CKEN_Msk         (0x1ul << CLK_APBCLK_UART0CKEN_Pos)               /*!< CLK_T::APBCLK: UART0CKEN Mask             */

#define CLK_APBCLK_UART1CKEN_Pos         (17)                                              /*!< CLK_T::APBCLK: UART1CKEN Position         */
#define CLK_APBCLK_UART1CKEN_Msk         (0x1ul << CLK_APBCLK_UART1CKEN_Pos)               /*!< CLK_T::APBCLK: UART1CKEN Mask             */

#define CLK_APBCLK_PWMCH01CKEN_Pos       (20)                                              /*!< CLK_T::APBCLK: PWMCH01CKEN Position       */
#define CLK_APBCLK_PWMCH01CKEN_Msk       (0x1ul << CLK_APBCLK_PWMCH01CKEN_Pos)             /*!< CLK_T::APBCLK: PWMCH01CKEN Mask           */

#define CLK_APBCLK_PWMCH23CKEN_Pos       (21)                                              /*!< CLK_T::APBCLK: PWMCH23CKEN Position       */
#define CLK_APBCLK_PWMCH23CKEN_Msk       (0x1ul << CLK_APBCLK_PWMCH23CKEN_Pos)             /*!< CLK_T::APBCLK: PWMCH23CKEN Mask           */

#define CLK_APBCLK_PWMCH45CKEN_Pos       (22)                                              /*!< CLK_T::APBCLK: PWMCH45CKEN Position       */
#define CLK_APBCLK_PWMCH45CKEN_Msk       (0x1ul << CLK_APBCLK_PWMCH45CKEN_Pos)             /*!< CLK_T::APBCLK: PWMCH45CKEN Mask           */

#define CLK_APBCLK_PWMCH67CKEN_Pos       (23)                                              /*!< CLK_T::APBCLK: PWMCH67CKEN Position       */
#define CLK_APBCLK_PWMCH67CKEN_Msk       (0x1ul << CLK_APBCLK_PWMCH67CKEN_Pos)             /*!< CLK_T::APBCLK: PWMCH67CKEN Mask           */

#define CLK_APBCLK_ADCCKEN_Pos           (28)                                              /*!< CLK_T::APBCLK: ADCCKEN Position           */
#define CLK_APBCLK_ADCCKEN_Msk           (0x1ul << CLK_APBCLK_ADCCKEN_Pos)                 /*!< CLK_T::APBCLK: ADCCKEN Mask               */

#define CLK_APBCLK_ACMPCKEN_Pos          (30)                                              /*!< CLK_T::APBCLK: ACMPCKEN Position          */
#define CLK_APBCLK_ACMPCKEN_Msk          (0x1ul << CLK_APBCLK_ACMPCKEN_Pos)                /*!< CLK_T::APBCLK: ACMPCKEN Mask              */

#define CLK_STATUS_XTLSTB_Pos            (0)                                               /*!< CLK_T::STATUS: XTLSTB Position            */
#define CLK_STATUS_XTLSTB_Msk            (0x1ul << CLK_STATUS_XTLSTB_Pos)                  /*!< CLK_T::STATUS: XTLSTB Mask                */

#define CLK_STATUS_IBGSTB_Pos            (1)                                               /*!< CLK_T::STATUS: IBGSTB Position            */
#define CLK_STATUS_IBGSTB_Msk            (0x1ul << CLK_STATUS_IBGSTB_Pos)                  /*!< CLK_T::STATUS: IBGSTB Mask                */

#define CLK_STATUS_PLLSTB_Pos            (2)                                               /*!< CLK_T::STATUS: PLLSTB Position            */
#define CLK_STATUS_PLLSTB_Msk            (0x1ul << CLK_STATUS_PLLSTB_Pos)                  /*!< CLK_T::STATUS: PLLSTB Mask                */

#define CLK_STATUS_LIRCSTB_Pos           (3)                                               /*!< CLK_T::STATUS: LIRCSTB Position           */
#define CLK_STATUS_LIRCSTB_Msk           (0x1ul << CLK_STATUS_LIRCSTB_Pos)                 /*!< CLK_T::STATUS: LIRCSTB Mask               */

#define CLK_STATUS_HIRCSTB_Pos           (4)                                               /*!< CLK_T::STATUS: HIRCSTB Position           */
#define CLK_STATUS_HIRCSTB_Msk           (0x1ul << CLK_STATUS_HIRCSTB_Pos)                 /*!< CLK_T::STATUS: HIRCSTB Mask               */

#define CLK_CLKSEL0_HCLKSEL_Pos          (0)                                               /*!< CLK_T::CLKSEL0: HCLKSEL Position          */
#define CLK_CLKSEL0_HCLKSEL_Msk          (0x3ul << CLK_CLKSEL0_HCLKSEL_Pos)                /*!< CLK_T::CLKSEL0: HCLKSEL Mask              */

#define CLK_CLKSEL1_WDTSEL_Pos           (0)                                               /*!< CLK_T::CLKSEL1: WDTSEL Position           */
#define CLK_CLKSEL1_WDTSEL_Msk           (0x3ul << CLK_CLKSEL1_WDTSEL_Pos)                 /*!< CLK_T::CLKSEL1: WDTSEL Mask               */

#define CLK_CLKSEL1_ADCSEL_Pos           (2)                                               /*!< CLK_T::CLKSEL1: ADCSEL Position           */
#define CLK_CLKSEL1_ADCSEL_Msk           (0x3ul << CLK_CLKSEL1_ADCSEL_Pos)                 /*!< CLK_T::CLKSEL1: ADCSEL Mask               */

#define CLK_CLKSEL1_SPI0SEL_Pos           (4)                                               /*!< CLK_T::CLKSEL1: SPI0SEL Position           */
#define CLK_CLKSEL1_SPI0SEL_Msk           (0x3ul << CLK_CLKSEL1_SPI0SEL_Pos)                 /*!< CLK_T::CLKSEL1: SPI0SEL Mask               */

#define CLK_CLKSEL1_SPI1SEL_Pos           (6)                                               /*!< CLK_T::CLKSEL1: SPI1SEL Position           */
#define CLK_CLKSEL1_SPI1SEL_Msk           (0x3ul << CLK_CLKSEL1_SPI1SEL_Pos)                 /*!< CLK_T::CLKSEL1: SPI1SEL Mask               */

#define CLK_CLKSEL1_TMR0SEL_Pos          (8)                                               /*!< CLK_T::CLKSEL1: TMR0SEL Position          */
#define CLK_CLKSEL1_TMR0SEL_Msk          (0x7ul << CLK_CLKSEL1_TMR0SEL_Pos)                /*!< CLK_T::CLKSEL1: TMR0SEL Mask              */

#define CLK_CLKSEL1_TMR1SEL_Pos          (12)                                              /*!< CLK_T::CLKSEL1: TMR1SEL Position          */
#define CLK_CLKSEL1_TMR1SEL_Msk          (0x7ul << CLK_CLKSEL1_TMR1SEL_Pos)                /*!< CLK_T::CLKSEL1: TMR1SEL Mask              */

#define CLK_CLKSEL1_TMR2SEL_Pos          (16)                                              /*!< CLK_T::CLKSEL1: TMR2SEL Position          */
#define CLK_CLKSEL1_TMR2SEL_Msk          (0x7ul << CLK_CLKSEL1_TMR2SEL_Pos)                /*!< CLK_T::CLKSEL1: TMR2SEL Mask              */

#define CLK_CLKSEL1_UARTSEL_Pos          (24)                                              /*!< CLK_T::CLKSEL1: UARTSEL Position          */
#define CLK_CLKSEL1_UARTSEL_Msk          (0x3ul << CLK_CLKSEL1_UARTSEL_Pos)                /*!< CLK_T::CLKSEL1: UARTSEL Mask              */

#define CLK_CLKSEL1_PWMCH01SEL_Pos       (28)                                              /*!< CLK_T::CLKSEL1: PWMCH01SEL Position       */
#define CLK_CLKSEL1_PWMCH01SEL_Msk       (0x3ul << CLK_CLKSEL1_PWMCH01SEL_Pos)             /*!< CLK_T::CLKSEL1: PWMCH01SEL Mask           */

#define CLK_CLKSEL1_PWMCH23SEL_Pos       (30)                                              /*!< CLK_T::CLKSEL1: PWMCH23SEL Position       */
#define CLK_CLKSEL1_PWMCH23SEL_Msk       (0x3ul << CLK_CLKSEL1_PWMCH23SEL_Pos)             /*!< CLK_T::CLKSEL1: PWMCH23SEL Mask           */

#define CLK_CLKDIV_HCLKDIV_Pos           (0)                                               /*!< CLK_T::CLKDIV: HCLKDIV Position           */
#define CLK_CLKDIV_HCLKDIV_Msk           (0x3ul << CLK_CLKDIV_HCLKDIV_Pos)                 /*!< CLK_T::CLKDIV: HCLKDIV Mask               */

#define CLK_CLKSEL2_CLKOSEL_Pos          (2)                                               /*!< CLK_T::CLKSEL2: CLKOSEL Position          */
#define CLK_CLKSEL2_CLKOSEL_Msk          (0x3ul << CLK_CLKSEL2_CLKOSEL_Pos)                /*!< CLK_T::CLKSEL2: CLKOSEL Mask              */

#define CLK_CLKSEL2_PWMCH45SEL_Pos       (4)                                               /*!< CLK_T::CLKSEL2: PWMCH45SEL Position       */
#define CLK_CLKSEL2_PWMCH45SEL_Msk       (0x3ul << CLK_CLKSEL2_PWMCH45SEL_Pos)             /*!< CLK_T::CLKSEL2: PWMCH45SEL Mask           */

#define CLK_CLKSEL2_PWMCH67SEL_Pos       (6)                                               /*!< CLK_T::CLKSEL2: PWMCH67SEL Position       */
#define CLK_CLKSEL2_PWMCH67SEL_Msk       (0x3ul << CLK_CLKSEL2_PWMCH67SEL_Pos)             /*!< CLK_T::CLKSEL2: PWMCH67SEL Mask           */

#define CLK_CLKSEL2_WWDTSEL_Pos          (16)                                              /*!< CLK_T::CLKSEL2: WWDTSEL Position          */
#define CLK_CLKSEL2_WWDTSEL_Msk          (0x3ul << CLK_CLKSEL2_WWDTSEL_Pos)                /*!< CLK_T::CLKSEL2: WWDTSEL Mask              */

#define CLK_PLLCTL_PD_Pos                (16)                                              /*!< CLK_T::PLLCTL: PD Position                */
#define CLK_PLLCTL_PD_Msk                (0x1ul << CLK_PLLCTL_PD_Pos)                      /*!< CLK_T::PLLCTL: PD Mask                    */

#define CLK_PLLCTL_PLLSRC_Pos            (19)                                              /*!< CLK_T::PLLCTL: PLLSRC Position            */
#define CLK_PLLCTL_PLLSRC_Msk            (0x1ul << CLK_PLLCTL_PLLSRC_Pos)                  /*!< CLK_T::PLLCTL: PLLSRC Mask                */

#define CLK_PLLCTL_PLLINFSEL_Pos         (20)                                              /*!< CLK_T::PLLCTL: _PLLINFSEL Position            */
#define CLK_PLLCTL_PLLINFSEL_Msk         (0x1ul << CLK_PLLCTL_PLLINFSEL_Pos)                  /*!< CLK_T::PLLCTL: _PLLINFSEL Mask                */

#define CLK_PLLCTL_ICTLDPLL_Pos          (22)                                              /*!< CLK_T::PLLCTL: ICTLDPLL Position            */
#define CLK_PLLCTL_ICTLDPLL_Msk          (0x3ul << CLK_PLLCTL_ICTLDPLL_Pos)                  /*!< CLK_T::PLLCTL: ICTLDPLL Mask                */

#define	CLK_WAKEUPCTL_XTLWUSEL_Pos			(0)                                           
#define	CLK_WAKEUPCTL_XTLWUSEL_Msk			(0x3ful << CLK_WAKEUPCTL_XTLWUSEL_Pos)

#define	CLK_WAKEUPCTL_IBGWUSEL_Pos			(8)
#define	CLK_WAKEUPCTL_IBGWUSEL_Msk			(0x3ful << CLK_WAKEUPCTL_IBGWUSEL_Pos)

#define	CLK_WAKEUPCTL_PLLWUSEL_Pos			(16)
#define	CLK_WAKEUPCTL_PLLWUSEL_Msk			(0x3ful << CLK_WAKEUPCTL_PLLWUSEL_Pos)

#define	CLK_WAKEUPCTL_HIRCWUSEL_Pos			(24)
#define	CLK_WAKEUPCTL_HIRCWUSEL_Msk			(0x3ful << CLK_WAKEUPCTL_HIRCWUSEL_Pos)


#define	CLK_LPCTL_PDXTL_Pos					(0)
#define	CLK_PDCTL_PDXTL_Msk					(0x01ul << CLK_LPCTL_PDXTL_Pos)

#define	CLK_LPCTL_PDIBG_Pos					(1)
#define	CLK_PDCTL_PDIBG_Msk					(0x01ul << CLK_LPCTL_PDIBG_Pos)

#define	CLK_LPCTL_PDPLL_Pos					(2)
#define	CLK_PDCTL_PDPLL_Msk					(0x01ul << CLK_LPCTL_PDPLL_Pos)

#define	CLK_LPCTL_PDHIRC_Pos				(3)
#define	CLK_PDCTL_PDHIRC_Msk				(0x01ul << CLK_LPCTL_PDHIRC_Pos)

#define	CLK_LPCTL_PDLIRC_Pos				(4)
#define	CLK_PDCTL_PDLIRC_Msk				(0x01ul << CLK_LPCTL_PDLIRC_Pos)

#define	CLK_LPCTL_PDVD18_Pos				(5)
#define	CLK_PDCTL_PDVD18_Msk				(0x01ul << CLK_LPCTL_PDVD18_Pos)

#define	CLK_LPCTL_PDADC_Pos					(6)
#define	CLK_PDCTL_PDADC_Msk					(0x01ul << CLK_LPCTL_PDADC_Pos)

#define	CLK_LPCTL_PDACMP_Pos				(7)
#define	CLK_PDCTL_PDACMP_Msk				(0x01ul << CLK_LPCTL_PDACMP_Pos)

#define	CLK_LPCTL_PDBOD_Pos					(8)
#define	CLK_PDCTL_PDBOD_Msk					(0x01ul << CLK_LPCTL_PDBOD_Pos)

#define	CLK_LPCTL_PDLVR_Pos					(9)
#define	CLK_PDCTL_PDLVR_Msk					(0x01ul << CLK_LPCTL_PDLVR_Pos)
/**@}*/ /* CLK_CONST */
/**@}*/ /* end of CLK register group */


/*---------------------- Flash Memory Controller -------------------------*/
/**
    @addtogroup FMC Flash Memory Controller(FMC)
    Memory Mapped Structure for FMC Controller
@{ */

typedef struct {


    /**
     * ISPCTL
     * ===================================================================================================
     * Offset: 0x00  ISP Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ISPEN     |ISP Enable Bit (Write Protect)
     * |        |          |Set this bit to enable the ISP function.
     * |        |          |0 = ISP function Disabled.
     * |        |          |1 = ISP function Enabled.
     * |[1]     |BS        |Boot Select (Write Protect)
     * |        |          |Set/clear this bit to select next booting from LDROM/APROM, respectively.
     * |        |          |This bit also functions as chip booting status flag, which can be used to check where chip booted from.
     * |        |          |This bit is initiated with the inversed value of CBS[1] (CONFIG0[7]) after any reset is happened except CPU reset (RSTS_CPU is 1) or system reset (RSTS_SYS) is happened.
     * |        |          |0 = Booting from APROM.
     * |        |          |1 = Booting from LDROM.
     * |[2]     |SPUEN     |SPROM Update Enable Bit (Write Protect)
     * |        |          |0 = SPROM cannot be updated.
     * |        |          |1 = SPROM can be updated.
     * |[3]     |APUEN     |APROM Update Enable Bit (Write Protect)
     * |        |          |0 = APROM cannot be updated when the chip runs in APROM.
     * |        |          |1 = APROM can be updated when the chip runs in APROM.
     * |[4]     |CFGUEN    |CONFIG Update Enable Bit (Write Protect)
     * |        |          |0 = CONFIG cannot be updated.
     * |        |          |1 = CONFIG can be updated.
     * |[5]     |LDUEN     |LDROM Update Enable (Write Protect)
     * |        |          |LDROM update enable bit.
     * |        |          |0 = LDROM cannot be updated.
     * |        |          |1 = LDROM can be updated.
     * |[6]     |ISPFF     |ISP Fail Flag (Write Protect)
     * |        |          |This bit is set by hardware when a triggered ISP meets any of the following conditions:
     * |        |          |This bit needs to be cleared by writing 1 to it.
     * |        |          |(1) APROM writes to itself if APUEN is set to 0.
     * |        |          |(2) LDROM writes to itself if LDUEN is set to 0.
     * |        |          |(3) CONFIG is erased/programmed if CFGUEN is set to 0.
     * |        |          |(4) SPROM is erased/programmed if SPUEN is set to 0.
     * |        |          |(5) SPROM is programmed at SPROM secured mode.
     * |        |          |(6) Destination address is illegal, such as over an available range.
     * |        |          |(7) Invalid ISP commands.
    */
    __IO uint32_t ISPCTL;

    /**
     * ISPADDR
     * ===================================================================================================
     * Offset: 0x04  ISP Address Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |ISPADDR   |ISP Address
     * |        |          |The PN020 series is equipped with embedded flash.
     * |        |          |ISPADDR[1:0] must be kept 00 for ISP 32-bit operation.
     * |        |          |and ISPADR[8:0] must be kept all 0 for Vector Page Re-map Command.
     * |        |          |For CRC32 Checksum Calculation command, this field is the flash starting address for checksum calculation, 512 bytes alignment is necessary for checksum calculation.
    */
    __IO uint32_t ISPADDR;

    /**
     * ISPDAT
     * ===================================================================================================
     * Offset: 0x08  ISP Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |ISPDAT    |ISP Data
     * |        |          |Write data to this register before ISP program operation.
     * |        |          |Read data from this register after ISP read operation.
     * |        |          |For Run CRC32 Checksum Calculation command, ISPDAT is the memory size (byte) and 512 bytes alignment.
     * |        |          |For ISP Read Checksum command, ISPDAT is the checksum result.
     * |        |          |If ISPDAT = 0x0000_0000, it means that (1) the checksum calculation is in progress, or (2) the memory range for checksum calculation is incorrect.
    */
    __IO uint32_t ISPDAT;

    /**
     * ISPCMD
     * ===================================================================================================
     * Offset: 0x0C  ISP Command Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[6:0]   |CMD       |ISP CMD
     * |        |          |ISP command table is shown below:
     * |        |          |0x00= FLASH Read.
     * |        |          |0x04= Read Unique ID.
     * |        |          |0x0B= Read Company ID.
     * |        |          |0x0C= Read Device ID.
     * |        |          |0x0D= Read CRC32 Checksum.
     * |        |          |0x21= FLASH 32-bit Program.
     * |        |          |0x22= FLASH Page Erase.
     * |        |          |0x2D= Run CRC32 Checksum Calculation.
     * |        |          |0x2E= Vector Remap.
     * |        |          |The other commands are invalid.
    */
    __IO uint32_t ISPCMD;

    /**
     * ISPTRG
     * ===================================================================================================
     * Offset: 0x10  ISP Trigger Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ISPGO     |ISP Start Trigger (Write Protect)
     * |        |          |Write 1 to start ISP operation and this bit will be cleared to 0 by hardware automatically when ISP operation is finished.
     * |        |          |0 = ISP operation is finished.
     * |        |          |1 = ISP is progressed.
    */
    __IO uint32_t ISPTRG;

    /**
     * DFBA
     * ===================================================================================================
     * Offset: 0x14  Data Flash Base Address
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |DFBA      |Data Flash Base Address
     * |        |          |This register indicates Data Flash start address. It is a read only register.
     * |        |          |The Data Flash is shared with APROM. the content of this register is loaded from CONFIG1
     * |        |          |This register is valid when DFEN (CONFIG0[0]) =0 .
    */
    __I  uint32_t DFBA;

    /**
     * FATCTL
     * ===================================================================================================
     * Offset: 0x18  Flash Access Time Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[6:4]   |FOM       |Frequency Optimization Mode (Write Protect)
     * |        |          |The PN020 series supports adjustable flash access timing to optimize the flash access cycles in different working frequency.
     * |        |          |0x1 = Frequency <= 24MHz.
     * |        |          |Others = Frequency <= 50MHz.
    */
    __IO uint32_t FATCTL;
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVED0[9];
    /// @endcond //HIDDEN_SYMBOLS


    /**
     * ISPSTS
     * ===================================================================================================
     * Offset: 0x40  ISP Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ISPBUSY   |ISP BUSY (Read Only)
     * |        |          |0 = ISP operation is finished.
     * |        |          |1 = ISP operation is busy.
     * |[2:1]   |CBS       |Boot Selection Of CONFIG (Read Only)
     * |        |          |This bit is initiated with the CBS (CONFIG0[7:6]) after any reset is happened except CPU reset (RSTS_CPU is 1) or system reset (RSTS_SYS) is happened.
     * |        |          |00 = LDROM with IAP mode.
     * |        |          |01 = LDROM without IAP mode.
     * |        |          |10 = APROM with IAP mode.
     * |        |          |11 = APROM without IAP mode.
     * |[6]     |ISPFF     |ISP Fail Flag (Write Protect)
     * |        |          |This bit is the mirror of ISPFF (FMC_ISPCTL[6]), it needs to be cleared by writing 1 to FMC_ISPCTL[6] or FMC_ISPSTS[6].
     * |        |          |This bit is set by hardware when a triggered ISP meets any of the following conditions:
     * |        |          |(1) APROM writes to itself if APUEN is set to 0.
     * |        |          |(2) LDROM writes to itself if LDUEN is set to 0.
     * |        |          |(3) CONFIG is erased/programmed if CFGUEN is set to 0.
     * |        |          |(4) SPROM is erased/programmed if SPUEN is set to 0.
     * |        |          |(5) SPROM is programmed at SPROM secured mode.
     * |        |          |(6) Page Erase command at LOCK mode with ICE connection.
     * |        |          |(7) Erase or Program command at brown-out detected.
     * |        |          |(8) Destination address is illegal, such as over an available range.
     * |        |          |(9) Invalid ISP commands.
     * |[20:9]  |VECMAP    |Vector Page Mapping Address (Read Only)
     * |        |          |All access to 0x0000_0000~0x0000_01FF is remapped to the flash memory address {VECMAP[11:0], 9'h000} ~ {VECMAP[11:0], 9'h1FF}
     * |[31]    |SCODE     |Security Code Active Flag
     * |        |          |This bit is set to 1 by hardware when detecting SPROM secured code is active at flash initialization, or software writes 1 to this bit to make secured code active; this bit is only cleared by SPROM page erase operation.
     * |        |          |0 = SPROM secured code is inactive.
     * |        |          |1 = SPROM secured code is active.
    */
    __I  uint32_t ISPSTS;

} FMC_T;

/**
    @addtogroup FMC_CONST FMC Bit Field Definition
    Constant Definitions for FMC Controller
@{ */

#define FMC_ISPCTL_ISPEN_Pos             (0)                                               /*!< FMC_T::ISPCTL: ISPEN Position             */
#define FMC_ISPCTL_ISPEN_Msk             (0x1ul << FMC_ISPCTL_ISPEN_Pos)                   /*!< FMC_T::ISPCTL: ISPEN Mask                 */

#define FMC_ISPCTL_BS_Pos                (1)                                               /*!< FMC_T::ISPCTL: BS Position                */
#define FMC_ISPCTL_BS_Msk                (0x1ul << FMC_ISPCTL_BS_Pos)                      /*!< FMC_T::ISPCTL: BS Mask                    */

#define FMC_ISPCTL_SPUEN_Pos             (2)                                               /*!< FMC_T::ISPCTL: SPUEN Position             */
#define FMC_ISPCTL_SPUEN_Msk             (0x1ul << FMC_ISPCTL_SPUEN_Pos)                   /*!< FMC_T::ISPCTL: SPUEN Mask                 */

#define FMC_ISPCTL_APUEN_Pos             (3)                                               /*!< FMC_T::ISPCTL: APUEN Position             */
#define FMC_ISPCTL_APUEN_Msk             (0x1ul << FMC_ISPCTL_APUEN_Pos)                   /*!< FMC_T::ISPCTL: APUEN Mask                 */

#define FMC_ISPCTL_CFGUEN_Pos            (4)                                               /*!< FMC_T::ISPCTL: CFGUEN Position            */
#define FMC_ISPCTL_CFGUEN_Msk            (0x1ul << FMC_ISPCTL_CFGUEN_Pos)                  /*!< FMC_T::ISPCTL: CFGUEN Mask                */

#define FMC_ISPCTL_LDUEN_Pos             (5)                                               /*!< FMC_T::ISPCTL: LDUEN Position             */
#define FMC_ISPCTL_LDUEN_Msk             (0x1ul << FMC_ISPCTL_LDUEN_Pos)                   /*!< FMC_T::ISPCTL: LDUEN Mask                 */

#define FMC_ISPCTL_ISPFF_Pos             (6)                                               /*!< FMC_T::ISPCTL: ISPFF Position             */
#define FMC_ISPCTL_ISPFF_Msk             (0x1ul << FMC_ISPCTL_ISPFF_Pos)                   /*!< FMC_T::ISPCTL: ISPFF Mask                 */

#define FMC_ISPADDR_ISPADDR_Pos          (0)                                               /*!< FMC_T::ISPADDR: ISPADDR Position          */
#define FMC_ISPADDR_ISPADDR_Msk          (0xfffffffful << FMC_ISPADDR_ISPADDR_Pos)         /*!< FMC_T::ISPADDR: ISPADDR Mask              */

#define FMC_ISPDAT_ISPDAT_Pos            (0)                                               /*!< FMC_T::ISPDAT: ISPDAT Position            */
#define FMC_ISPDAT_ISPDAT_Msk            (0xfffffffful << FMC_ISPDAT_ISPDAT_Pos)           /*!< FMC_T::ISPDAT: ISPDAT Mask                */

#define FMC_ISPCMD_CMD_Pos               (0)                                               /*!< FMC_T::ISPCMD: CMD Position               */
#define FMC_ISPCMD_CMD_Msk               (0x7ful << FMC_ISPCMD_CMD_Pos)                    /*!< FMC_T::ISPCMD: CMD Mask                   */

#define FMC_ISPTRG_ISPGO_Pos             (0)                                               /*!< FMC_T::ISPTRG: ISPGO Position             */
#define FMC_ISPTRG_ISPGO_Msk             (0x1ul << FMC_ISPTRG_ISPGO_Pos)                   /*!< FMC_T::ISPTRG: ISPGO Mask                 */

#define FMC_DFBA_DFBA_Pos                (0)                                               /*!< FMC_T::DFBA: DFBA Position                */
#define FMC_DFBA_DFBA_Msk                (0xfffffffful << FMC_DFBA_DFBA_Pos)               /*!< FMC_T::DFBA: DFBA Mask                    */

#define FMC_FATCTL_FOM_Pos               (4)                                               /*!< FMC_T::FATCTL: FOM Position               */
#define FMC_FATCTL_FOM_Msk               (0x7ul << FMC_FATCTL_FOM_Pos)                     /*!< FMC_T::FATCTL: FOM Mask                   */

#define FMC_ISPSTS_ISPBUSY_Pos           (0)                                               /*!< FMC_T::ISPSTS: ISPBUSY Position           */
#define FMC_ISPSTS_ISPBUSY_Msk           (0x1ul << FMC_ISPSTS_ISPBUSY_Pos)                 /*!< FMC_T::ISPSTS: ISPBUSY Mask               */

#define FMC_ISPSTS_CBS_Pos               (1)                                               /*!< FMC_T::ISPSTS: CBS Position               */
#define FMC_ISPSTS_CBS_Msk               (0x3ul << FMC_ISPSTS_CBS_Pos)                     /*!< FMC_T::ISPSTS: CBS Mask                   */

#define FMC_ISPSTS_ISPFF_Pos             (6)                                               /*!< FMC_T::ISPSTS: ISPFF Position             */
#define FMC_ISPSTS_ISPFF_Msk             (0x1ul << FMC_ISPSTS_ISPFF_Pos)                   /*!< FMC_T::ISPSTS: ISPFF Mask                 */

#define FMC_ISPSTS_VECMAP_Pos            (9)                                               /*!< FMC_T::ISPSTS: VECMAP Position            */
#define FMC_ISPSTS_VECMAP_Msk            (0xffful << FMC_ISPSTS_VECMAP_Pos)                /*!< FMC_T::ISPSTS: VECMAP Mask                */

#define FMC_ISPSTS_SCODE_Pos             (31)                                              /*!< FMC_T::ISPSTS: SCODE Position             */
#define FMC_ISPSTS_SCODE_Msk             (0x1ul << FMC_ISPSTS_SCODE_Pos)                   /*!< FMC_T::ISPSTS: SCODE Mask                 */

/**@}*/ /* FMC_CONST */
/**@}*/ /* end of FMC register group */


/*---------------------- General Purpose Input/Output Controller -------------------------*/
/**
    @addtogroup GP General Purpose Input/Output Controller(GP)
    Memory Mapped Structure for GP Controller
@{ */

typedef struct {


    /**
     * MODE
     * ===================================================================================================
     * Offset: 0x00  P0 I/O Mode Control
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |MODE0     |Port 0-5 I/O Pin[N] Mode Control
     * |        |          |Determine each I/O mode of Px.n pins.
     * |        |          |00 = Px.n is in Input mode.
     * |        |          |01 = Px.n is in Push-pull Output mode.
     * |        |          |10 = Px.n is in Open-drain Output mode.
     * |        |          |11 = Px.n is in Quasi-bidirectional mode.
     * |        |          |analog input mode after chip powered on.
     * |        |          |Note1:
     * |        |          |Max. n=7 for port 0
     * |        |          |Max. n=7 for port 1
     * |        |          |Max. n=7 for port 2
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[3:2]   |MODE1     |Port 0-5 I/O Pin[N] Mode Control
     * |        |          |Determine each I/O mode of Px.n pins.
     * |        |          |00 = Px.n is in Input mode.
     * |        |          |01 = Px.n is in Push-pull Output mode.
     * |        |          |10 = Px.n is in Open-drain Output mode.
     * |        |          |11 = Px.n is in Quasi-bidirectional mode.
     * |        |          |analog input mode after chip powered on.
     * |        |          |Note1:
     * |        |          |Max. n=7 for port 0
     * |        |          |Max. n=7 for port 1
     * |        |          |Max. n=7 for port 2
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.

     * |[5:4]   |MODE2     |Port 0-5 I/O Pin[N] Mode Control
     * |        |          |Determine each I/O mode of Px.n pins.
     * |        |          |00 = Px.n is in Input mode.
     * |        |          |01 = Px.n is in Push-pull Output mode.
     * |        |          |10 = Px.n is in Open-drain Output mode.
     * |        |          |11 = Px.n is in Quasi-bidirectional mode.
     * |        |          |analog input mode after chip powered on.
     * |        |          |Note1:
     * |        |          |Max. n=7 for port 0
     * |        |          |Max. n=7 for port 1
     * |        |          |Max. n=7 for port 2
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.

     * |[7:6]   |MODE3     |Port 0-5 I/O Pin[N] Mode Control
     * |        |          |Determine each I/O mode of Px.n pins.
     * |        |          |00 = Px.n is in Input mode.
     * |        |          |01 = Px.n is in Push-pull Output mode.
     * |        |          |10 = Px.n is in Open-drain Output mode.
     * |        |          |11 = Px.n is in Quasi-bidirectional mode.
     * |        |          |analog input mode after chip powered on.
     * |        |          |Note1:
     * |        |          |Max. n=7 for port 0
     * |        |          |Max. n=7 for port 1
     * |        |          |Max. n=7 for port 2
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.

     * |[9:8]   |MODE4     |Port 0-5 I/O Pin[N] Mode Control
     * |        |          |Determine each I/O mode of Px.n pins.
     * |        |          |00 = Px.n is in Input mode.
     * |        |          |01 = Px.n is in Push-pull Output mode.
     * |        |          |10 = Px.n is in Open-drain Output mode.
     * |        |          |11 = Px.n is in Quasi-bidirectional mode.
     * |        |          |analog input mode after chip powered on.
     * |        |          |Note1:
     * |        |          |Max. n=7 for port 0
     * |        |          |Max. n=7 for port 1
     * |        |          |Max. n=7 for port 2
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.

     * |[11:10] |MODE5     |Port 0-5 I/O Pin[N] Mode Control
     * |        |          |Determine each I/O mode of Px.n pins.
     * |        |          |00 = Px.n is in Input mode.
     * |        |          |01 = Px.n is in Push-pull Output mode.
     * |        |          |10 = Px.n is in Open-drain Output mode.
     * |        |          |11 = Px.n is in Quasi-bidirectional mode.
     * |        |          |analog input mode after chip powered on.
     * |        |          |Note1:
     * |        |          |Max. n=7 for port 0
     * |        |          |Max. n=7 for port 1
     * |        |          |Max. n=7 for port 2
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.

     * |[13:12] |MODE6     |Port 0-5 I/O Pin[N] Mode Control
     * |        |          |Determine each I/O mode of Px.n pins.
     * |        |          |00 = Px.n is in Input mode.
     * |        |          |01 = Px.n is in Push-pull Output mode.
     * |        |          |10 = Px.n is in Open-drain Output mode.
     * |        |          |11 = Px.n is in Quasi-bidirectional mode.
     * |        |          |analog input mode after chip powered on.
     * |        |          |Note1:
     * |        |          |Max. n=7 for port 0
     * |        |          |Max. n=7 for port 1
     * |        |          |Max. n=7 for port 2
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.

     * |[15:14] |MODE7     |Port 0-5 I/O Pin[N] Mode Control
     * |        |          |Determine each I/O mode of Px.n pins.
     * |        |          |00 = Px.n is in Input mode.
     * |        |          |01 = Px.n is in Push-pull Output mode.
     * |        |          |10 = Px.n is in Open-drain Output mode.
     * |        |          |analog input mode after chip powered on.
     * |        |          |Note1:
     * |        |          |Max. n=7 for port 0
     * |        |          |Max. n=7 for port 1
     * |        |          |Max. n=7 for port 2
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5..
    */
    __IO uint32_t MODE;

    /**
     * DINOFF
     * ===================================================================================================
     * Offset: 0x04  P0 Digital Input Path Disable Control
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[16]    |DINOFF0   |Port 0-5 Pin[N] Digital Input Path Disable Control
     * |        |          |Each of these bits is used to control if the digital input path of corresponding Px.n pin is disabled.
     * |        |          |If input is analog signal, users can disable Px.n digital input path to avoid input current leakage.
     * |        |          |0 = Px.n digital input path Enabled.
     * |        |          |1 = Px.n digital input path Disabled (digital input tied to low).
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[17]    |DINOFF1   |Port 0-5 Pin[N] Digital Input Path Disable Control
     * |        |          |Each of these bits is used to control if the digital input path of corresponding Px.n pin is disabled.
     * |        |          |If input is analog signal, users can disable Px.n digital input path to avoid input current leakage.
     * |        |          |0 = Px.n digital input path Enabled.
     * |        |          |1 = Px.n digital input path Disabled (digital input tied to low).
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[18]    |DINOFF2   |Port 0-5 Pin[N] Digital Input Path Disable Control
     * |        |          |Each of these bits is used to control if the digital input path of corresponding Px.n pin is disabled.
     * |        |          |If input is analog signal, users can disable Px.n digital input path to avoid input current leakage.
     * |        |          |0 = Px.n digital input path Enabled.
     * |        |          |1 = Px.n digital input path Disabled (digital input tied to low).
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[19]    |DINOFF3   |Port 0-5 Pin[N] Digital Input Path Disable Control
     * |        |          |Each of these bits is used to control if the digital input path of corresponding Px.n pin is disabled.
     * |        |          |If input is analog signal, users can disable Px.n digital input path to avoid input current leakage.
     * |        |          |0 = Px.n digital input path Enabled.
     * |        |          |1 = Px.n digital input path Disabled (digital input tied to low).
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[20]    |DINOFF4   |Port 0-5 Pin[N] Digital Input Path Disable Control
     * |        |          |Each of these bits is used to control if the digital input path of corresponding Px.n pin is disabled.
     * |        |          |If input is analog signal, users can disable Px.n digital input path to avoid input current leakage.
     * |        |          |0 = Px.n digital input path Enabled.
     * |        |          |1 = Px.n digital input path Disabled (digital input tied to low).
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[21]    |DINOFF5   |Port 0-5 Pin[N] Digital Input Path Disable Control
     * |        |          |Each of these bits is used to control if the digital input path of corresponding Px.n pin is disabled.
     * |        |          |If input is analog signal, users can disable Px.n digital input path to avoid input current leakage.
     * |        |          |0 = Px.n digital input path Enabled.
     * |        |          |1 = Px.n digital input path Disabled (digital input tied to low).
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[22]    |DINOFF6   |Port 0-5 Pin[N] Digital Input Path Disable Control
     * |        |          |Each of these bits is used to control if the digital input path of corresponding Px.n pin is disabled.
     * |        |          |If input is analog signal, users can disable Px.n digital input path to avoid input current leakage.
     * |        |          |0 = Px.n digital input path Enabled.
     * |        |          |1 = Px.n digital input path Disabled (digital input tied to low).
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[23]    |DINOFF7   |Port 0-5 Pin[N] Digital Input Path Disable Control
     * |        |          |Each of these bits is used to control if the digital input path of corresponding Px.n pin is disabled.
     * |        |          |If input is analog signal, users can disable Px.n digital input path to avoid input current leakage.
     * |        |          |0 = Px.n digital input path Enabled.
     * |        |          |1 = Px.n digital input path Disabled (digital input tied to low).
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
    */
    __IO uint32_t DINOFF;

    /**
     * DOUT
     * ===================================================================================================
     * Offset: 0x08  P0 Data Output Value
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |DOUT0     |Port 0-5 Pin[N] Output Value
     * |        |          |Each of these bits controls the status of a Px.n pin when the Px.n is configured as Push-pull output, Open-drain output or Quasi-bidirectional mode.
     * |        |          |0 = Px.n will drive Low if the Px.n pin is configured as Push-pull output, Open-drain output or Quasi-bidirectional mode.
     * |        |          |1 = Px.n will drive High if the Px.n pin is configured as Push-pull output or Quasi-bidirectional mode.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[1]     |DOUT1     |Port 0-5 Pin[N] Output Value
     * |        |          |Each of these bits controls the status of a Px.n pin when the Px.n is configured as Push-pull output, Open-drain output or Quasi-bidirectional mode.
     * |        |          |0 = Px.n will drive Low if the Px.n pin is configured as Push-pull output, Open-drain output or Quasi-bidirectional mode.
     * |        |          |1 = Px.n will drive High if the Px.n pin is configured as Push-pull output or Quasi-bidirectional mode.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[2]     |DOUT2     |Port 0-5 Pin[N] Output Value
     * |        |          |Each of these bits controls the status of a Px.n pin when the Px.n is configured as Push-pull output, Open-drain output or Quasi-bidirectional mode.
     * |        |          |0 = Px.n will drive Low if the Px.n pin is configured as Push-pull output, Open-drain output or Quasi-bidirectional mode.
     * |        |          |1 = Px.n will drive High if the Px.n pin is configured as Push-pull output or Quasi-bidirectional mode.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[3]     |DOUT3     |Port 0-5 Pin[N] Output Value
     * |        |          |Each of these bits controls the status of a Px.n pin when the Px.n is configured as Push-pull output, Open-drain output or Quasi-bidirectional mode.
     * |        |          |0 = Px.n will drive Low if the Px.n pin is configured as Push-pull output, Open-drain output or Quasi-bidirectional mode.
     * |        |          |1 = Px.n will drive High if the Px.n pin is configured as Push-pull output or Quasi-bidirectional mode.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[4]     |DOUT4     |Port 0-5 Pin[N] Output Value
     * |        |          |Each of these bits controls the status of a Px.n pin when the Px.n is configured as Push-pull output, Open-drain output or Quasi-bidirectional mode.
     * |        |          |0 = Px.n will drive Low if the Px.n pin is configured as Push-pull output, Open-drain output or Quasi-bidirectional mode.
     * |        |          |1 = Px.n will drive High if the Px.n pin is configured as Push-pull output or Quasi-bidirectional mode.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[5]     |DOUT5     |Port 0-5 Pin[N] Output Value
     * |        |          |Each of these bits controls the status of a Px.n pin when the Px.n is configured as Push-pull output, Open-drain output or Quasi-bidirectional mode.
     * |        |          |0 = Px.n will drive Low if the Px.n pin is configured as Push-pull output, Open-drain output or Quasi-bidirectional mode.
     * |        |          |1 = Px.n will drive High if the Px.n pin is configured as Push-pull output or Quasi-bidirectional mode.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[6]     |DOUT6     |Port 0-5 Pin[N] Output Value
     * |        |          |Each of these bits controls the status of a Px.n pin when the Px.n is configured as Push-pull output, Open-drain output or Quasi-bidirectional mode.
     * |        |          |0 = Px.n will drive Low if the Px.n pin is configured as Push-pull output, Open-drain output or Quasi-bidirectional mode.
     * |        |          |1 = Px.n will drive High if the Px.n pin is configured as Push-pull output or Quasi-bidirectional mode.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[7]     |DOUT7     |Port 0-5 Pin[N] Output Value
     * |        |          |Each of these bits controls the status of a Px.n pin when the Px.n is configured as Push-pull output, Open-drain output or Quasi-bidirectional mode.
     * |        |          |0 = Px.n will drive Low if the Px.n pin is configured as Push-pull output, Open-drain output or Quasi-bidirectional mode.
     * |        |          |1 = Px.n will drive High if the Px.n pin is configured as Push-pull output or Quasi-bidirectional mode.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
    */
    __IO uint32_t DOUT;

    /**
     * DATMSK
     * ===================================================================================================
     * Offset: 0x0C  P0 Data Output Write Mask
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |DATMSK0   |Port 0-5 Pin[N] Data Output Write Mask
     * |        |          |These bits are used to protect the corresponding DOUT (Px_DOUT[n]) bit.
     * |        |          |When the DATMSK (Px_DATMSK[n]) bit is set to 1, the corresponding DOUT (Px_DOUT[n]) bit is protected.
     * |        |          |If the write signal is masked, writing data to the protect bit is ignore.
     * |        |          |0 = Corresponding DOUT (Px_DOUT[n]) bit can be updated.
     * |        |          |1 = Corresponding DOUT (Px_DOUT[n]) bit protected.
     * |        |          |Note1: This function only protects the corresponding DOUT (Px_DOUT[n]) bit, and will not protect the corresponding PDIO (Pxn_PDIO[0]) bit.
     * |        |          |Note2:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[1]     |DATMSK1   |Port 0-5 Pin[N] Data Output Write Mask
     * |        |          |These bits are used to protect the corresponding DOUT (Px_DOUT[n]) bit.
     * |        |          |When the DATMSK (Px_DATMSK[n]) bit is set to 1, the corresponding DOUT (Px_DOUT[n]) bit is protected.
     * |        |          |If the write signal is masked, writing data to the protect bit is ignore.
     * |        |          |0 = Corresponding DOUT (Px_DOUT[n]) bit can be updated.
     * |        |          |1 = Corresponding DOUT (Px_DOUT[n]) bit protected.
     * |        |          |Note1: This function only protects the corresponding DOUT (Px_DOUT[n]) bit, and will not protect the corresponding PDIO (Pxn_PDIO[0]) bit.
     * |        |          |Note2:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[2]     |DATMSK2   |Port 0-5 Pin[N] Data Output Write Mask
     * |        |          |These bits are used to protect the corresponding DOUT (Px_DOUT[n]) bit.
     * |        |          |When the DATMSK (Px_DATMSK[n]) bit is set to 1, the corresponding DOUT (Px_DOUT[n]) bit is protected.
     * |        |          |If the write signal is masked, writing data to the protect bit is ignore.
     * |        |          |0 = Corresponding DOUT (Px_DOUT[n]) bit can be updated.
     * |        |          |1 = Corresponding DOUT (Px_DOUT[n]) bit protected.
     * |        |          |Note1: This function only protects the corresponding DOUT (Px_DOUT[n]) bit, and will not protect the corresponding PDIO (Pxn_PDIO[0]) bit.
     * |        |          |Note2:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[3]     |DATMSK3   |Port 0-5 Pin[N] Data Output Write Mask
     * |        |          |These bits are used to protect the corresponding DOUT (Px_DOUT[n]) bit.
     * |        |          |When the DATMSK (Px_DATMSK[n]) bit is set to 1, the corresponding DOUT (Px_DOUT[n]) bit is protected.
     * |        |          |If the write signal is masked, writing data to the protect bit is ignore.
     * |        |          |0 = Corresponding DOUT (Px_DOUT[n]) bit can be updated.
     * |        |          |1 = Corresponding DOUT (Px_DOUT[n]) bit protected.
     * |        |          |Note1: This function only protects the corresponding DOUT (Px_DOUT[n]) bit, and will not protect the corresponding PDIO (Pxn_PDIO[0]) bit.
     * |        |          |Note2:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[4]     |DATMSK4   |Port 0-5 Pin[N] Data Output Write Mask
     * |        |          |These bits are used to protect the corresponding DOUT (Px_DOUT[n]) bit.
     * |        |          |When the DATMSK (Px_DATMSK[n]) bit is set to 1, the corresponding DOUT (Px_DOUT[n]) bit is protected.
     * |        |          |If the write signal is masked, writing data to the protect bit is ignore.
     * |        |          |0 = Corresponding DOUT (Px_DOUT[n]) bit can be updated.
     * |        |          |1 = Corresponding DOUT (Px_DOUT[n]) bit protected.
     * |        |          |Note1: This function only protects the corresponding DOUT (Px_DOUT[n]) bit, and will not protect the corresponding PDIO (Pxn_PDIO[0]) bit.
     * |        |          |Note2:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[5]     |DATMSK5   |Port 0-5 Pin[N] Data Output Write Mask
     * |        |          |These bits are used to protect the corresponding DOUT (Px_DOUT[n]) bit.
     * |        |          |When the DATMSK (Px_DATMSK[n]) bit is set to 1, the corresponding DOUT (Px_DOUT[n]) bit is protected.
     * |        |          |If the write signal is masked, writing data to the protect bit is ignore.
     * |        |          |0 = Corresponding DOUT (Px_DOUT[n]) bit can be updated.
     * |        |          |1 = Corresponding DOUT (Px_DOUT[n]) bit protected.
     * |        |          |Note1: This function only protects the corresponding DOUT (Px_DOUT[n]) bit, and will not protect the corresponding PDIO (Pxn_PDIO[0]) bit.
     * |        |          |Note2:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[6]     |DATMSK6   |Port 0-5 Pin[N] Data Output Write Mask
     * |        |          |These bits are used to protect the corresponding DOUT (Px_DOUT[n]) bit.
     * |        |          |When the DATMSK (Px_DATMSK[n]) bit is set to 1, the corresponding DOUT (Px_DOUT[n]) bit is protected.
     * |        |          |If the write signal is masked, writing data to the protect bit is ignore.
     * |        |          |0 = Corresponding DOUT (Px_DOUT[n]) bit can be updated.
     * |        |          |1 = Corresponding DOUT (Px_DOUT[n]) bit protected.
     * |        |          |Note1: This function only protects the corresponding DOUT (Px_DOUT[n]) bit, and will not protect the corresponding PDIO (Pxn_PDIO[0]) bit.
     * |        |          |Note2:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[7]     |DATMSK7   |Port 0-5 Pin[N] Data Output Write Mask
     * |        |          |These bits are used to protect the corresponding DOUT (Px_DOUT[n]) bit.
     * |        |          |When the DATMSK (Px_DATMSK[n]) bit is set to 1, the corresponding DOUT (Px_DOUT[n]) bit is protected.
     * |        |          |If the write signal is masked, writing data to the protect bit is ignore.
     * |        |          |0 = Corresponding DOUT (Px_DOUT[n]) bit can be updated.
     * |        |          |1 = Corresponding DOUT (Px_DOUT[n]) bit protected.
     * |        |          |Note1: This function only protects the corresponding DOUT (Px_DOUT[n]) bit, and will not protect the corresponding PDIO (Pxn_PDIO[0]) bit.
     * |        |          |Note2:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
    */
    __IO uint32_t DATMSK;

    /**
     * PIN
     * ===================================================================================================
     * Offset: 0x10  P0 Pin Value
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PIN0      |Port 0-5 Pin[N] Pin Value
     * |        |          |Each bit of the register reflects the actual status of the respective Px.n pin.
     * |        |          |If the bit is 1, it indicates the corresponding pin status is high; else the pin status is low.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[1]     |PIN1      |Port 0-5 Pin[N] Pin Value
     * |        |          |Each bit of the register reflects the actual status of the respective Px.n pin.
     * |        |          |If the bit is 1, it indicates the corresponding pin status is high; else the pin status is low.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[2]     |PIN2      |Port 0-5 Pin[N] Pin Value
     * |        |          |Each bit of the register reflects the actual status of the respective Px.n pin.
     * |        |          |If the bit is 1, it indicates the corresponding pin status is high; else the pin status is low.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[3]     |PIN3      |Port 0-5 Pin[N] Pin Value
     * |        |          |Each bit of the register reflects the actual status of the respective Px.n pin.
     * |        |          |If the bit is 1, it indicates the corresponding pin status is high; else the pin status is low.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[4]     |PIN4      |Port 0-5 Pin[N] Pin Value
     * |        |          |Each bit of the register reflects the actual status of the respective Px.n pin.
     * |        |          |If the bit is 1, it indicates the corresponding pin status is high; else the pin status is low.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[5]     |PIN5      |Port 0-5 Pin[N] Pin Value
     * |        |          |Each bit of the register reflects the actual status of the respective Px.n pin.
     * |        |          |If the bit is 1, it indicates the corresponding pin status is high; else the pin status is low.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[6]     |PIN6      |Port 0-5 Pin[N] Pin Value
     * |        |          |Each bit of the register reflects the actual status of the respective Px.n pin.
     * |        |          |If the bit is 1, it indicates the corresponding pin status is high; else the pin status is low.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[7]     |PIN7      |Port 0-5 Pin[N] Pin Value
     * |        |          |Each bit of the register reflects the actual status of the respective Px.n pin.
     * |        |          |If the bit is 1, it indicates the corresponding pin status is high; else the pin status is low.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
    */
    __IO  uint32_t PIN;

    /**
     * DBEN
     * ===================================================================================================
     * Offset: 0x14  P0 De-bounce Enable Control
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |DBEN0     |Port 0-5 Pin[N] Input Signal De-bounce Enable Bit
     * |        |          |The DBEN[n] bit is used to enable the de-bounce function for each corresponding bit.
     * |        |          |If the input signal pulse width cannot be sampled by continuous two de-bounce sample cycle, the input signal transition is seen as the signal bounce and will not trigger the interrupt.
     * |        |          |The de-bounce clock source is controlled by DBCLKSRC (GPIO_DBCTL [4]), one de-bounce sample cycle period is controlled by DBCLKSEL (GPIO_DBCTL [3:0]).
     * |        |          |0 = Px.n de-bounce function Disabled.
     * |        |          |1 = Px.n de-bounce function Enabled.
     * |        |          |The de-bounce function is valid only for edge triggered interrupt.
     * |        |          |If the interrupt mode is level triggered, the de-bounce enable bit is ignore.
     * |        |          |Note1: If Px.n pin is chosen as Power-down wake-up source, user should be disable the de-bounce function before entering Power-down mode to avoid the second interrupt event occurred after system waken up which caused by Px.n de-bounce function.
     * |        |          |Note2:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[1]     |DBEN1     |Port 0-5 Pin[N] Input Signal De-bounce Enable Bit
     * |        |          |The DBEN[n] bit is used to enable the de-bounce function for each corresponding bit.
     * |        |          |If the input signal pulse width cannot be sampled by continuous two de-bounce sample cycle, the input signal transition is seen as the signal bounce and will not trigger the interrupt.
     * |        |          |The de-bounce clock source is controlled by DBCLKSRC (GPIO_DBCTL [4]), one de-bounce sample cycle period is controlled by DBCLKSEL (GPIO_DBCTL [3:0]).
     * |        |          |0 = Px.n de-bounce function Disabled.
     * |        |          |1 = Px.n de-bounce function Enabled.
     * |        |          |The de-bounce function is valid only for edge triggered interrupt.
     * |        |          |If the interrupt mode is level triggered, the de-bounce enable bit is ignore.
     * |        |          |Note1: If Px.n pin is chosen as Power-down wake-up source, user should be disable the de-bounce function before entering Power-down mode to avoid the second interrupt event occurred after system waken up which caused by Px.n de-bounce function.
     * |        |          |Note2:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[2]     |DBEN2     |Port 0-5 Pin[N] Input Signal De-bounce Enable Bit
     * |        |          |The DBEN[n] bit is used to enable the de-bounce function for each corresponding bit.
     * |        |          |If the input signal pulse width cannot be sampled by continuous two de-bounce sample cycle, the input signal transition is seen as the signal bounce and will not trigger the interrupt.
     * |        |          |The de-bounce clock source is controlled by DBCLKSRC (GPIO_DBCTL [4]), one de-bounce sample cycle period is controlled by DBCLKSEL (GPIO_DBCTL [3:0]).
     * |        |          |0 = Px.n de-bounce function Disabled.
     * |        |          |1 = Px.n de-bounce function Enabled.
     * |        |          |The de-bounce function is valid only for edge triggered interrupt.
     * |        |          |If the interrupt mode is level triggered, the de-bounce enable bit is ignore.
     * |        |          |Note1: If Px.n pin is chosen as Power-down wake-up source, user should be disable the de-bounce function before entering Power-down mode to avoid the second interrupt event occurred after system waken up which caused by Px.n de-bounce function.
     * |        |          |Note2:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[3]     |DBEN3     |Port 0-5 Pin[N] Input Signal De-bounce Enable Bit
     * |        |          |The DBEN[n] bit is used to enable the de-bounce function for each corresponding bit.
     * |        |          |If the input signal pulse width cannot be sampled by continuous two de-bounce sample cycle, the input signal transition is seen as the signal bounce and will not trigger the interrupt.
     * |        |          |The de-bounce clock source is controlled by DBCLKSRC (GPIO_DBCTL [4]), one de-bounce sample cycle period is controlled by DBCLKSEL (GPIO_DBCTL [3:0]).
     * |        |          |0 = Px.n de-bounce function Disabled.
     * |        |          |1 = Px.n de-bounce function Enabled.
     * |        |          |The de-bounce function is valid only for edge triggered interrupt.
     * |        |          |If the interrupt mode is level triggered, the de-bounce enable bit is ignore.
     * |        |          |Note1: If Px.n pin is chosen as Power-down wake-up source, user should be disable the de-bounce function before entering Power-down mode to avoid the second interrupt event occurred after system waken up which caused by Px.n de-bounce function.
     * |        |          |Note2:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[4]     |DBEN4     |Port 0-5 Pin[N] Input Signal De-bounce Enable Bit
     * |        |          |The DBEN[n] bit is used to enable the de-bounce function for each corresponding bit.
     * |        |          |If the input signal pulse width cannot be sampled by continuous two de-bounce sample cycle, the input signal transition is seen as the signal bounce and will not trigger the interrupt.
     * |        |          |The de-bounce clock source is controlled by DBCLKSRC (GPIO_DBCTL [4]), one de-bounce sample cycle period is controlled by DBCLKSEL (GPIO_DBCTL [3:0]).
     * |        |          |0 = Px.n de-bounce function Disabled.
     * |        |          |1 = Px.n de-bounce function Enabled.
     * |        |          |The de-bounce function is valid only for edge triggered interrupt.
     * |        |          |If the interrupt mode is level triggered, the de-bounce enable bit is ignore.
     * |        |          |Note1: If Px.n pin is chosen as Power-down wake-up source, user should be disable the de-bounce function before entering Power-down mode to avoid the second interrupt event occurred after system waken up which caused by Px.n de-bounce function.
     * |        |          |Note2:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[5]     |DBEN5     |Port 0-5 Pin[N] Input Signal De-bounce Enable Bit
     * |        |          |The DBEN[n] bit is used to enable the de-bounce function for each corresponding bit.
     * |        |          |If the input signal pulse width cannot be sampled by continuous two de-bounce sample cycle, the input signal transition is seen as the signal bounce and will not trigger the interrupt.
     * |        |          |The de-bounce clock source is controlled by DBCLKSRC (GPIO_DBCTL [4]), one de-bounce sample cycle period is controlled by DBCLKSEL (GPIO_DBCTL [3:0]).
     * |        |          |0 = Px.n de-bounce function Disabled.
     * |        |          |1 = Px.n de-bounce function Enabled.
     * |        |          |The de-bounce function is valid only for edge triggered interrupt.
     * |        |          |If the interrupt mode is level triggered, the de-bounce enable bit is ignore.
     * |        |          |Note1: If Px.n pin is chosen as Power-down wake-up source, user should be disable the de-bounce function before entering Power-down mode to avoid the second interrupt event occurred after system waken up which caused by Px.n de-bounce function.
     * |        |          |Note2:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[6]     |DBEN6     |Port 0-5 Pin[N] Input Signal De-bounce Enable Bit
     * |        |          |The DBEN[n] bit is used to enable the de-bounce function for each corresponding bit.
     * |        |          |If the input signal pulse width cannot be sampled by continuous two de-bounce sample cycle, the input signal transition is seen as the signal bounce and will not trigger the interrupt.
     * |        |          |The de-bounce clock source is controlled by DBCLKSRC (GPIO_DBCTL [4]), one de-bounce sample cycle period is controlled by DBCLKSEL (GPIO_DBCTL [3:0]).
     * |        |          |0 = Px.n de-bounce function Disabled.
     * |        |          |1 = Px.n de-bounce function Enabled.
     * |        |          |The de-bounce function is valid only for edge triggered interrupt.
     * |        |          |If the interrupt mode is level triggered, the de-bounce enable bit is ignore.
     * |        |          |Note1: If Px.n pin is chosen as Power-down wake-up source, user should be disable the de-bounce function before entering Power-down mode to avoid the second interrupt event occurred after system waken up which caused by Px.n de-bounce function.
     * |        |          |Note2:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[7]     |DBEN7     |Port 0-5 Pin[N] Input Signal De-bounce Enable Bit
     * |        |          |The DBEN[n] bit is used to enable the de-bounce function for each corresponding bit.
     * |        |          |If the input signal pulse width cannot be sampled by continuous two de-bounce sample cycle, the input signal transition is seen as the signal bounce and will not trigger the interrupt.
     * |        |          |The de-bounce clock source is controlled by DBCLKSRC (GPIO_DBCTL [4]), one de-bounce sample cycle period is controlled by DBCLKSEL (GPIO_DBCTL [3:0]).
     * |        |          |0 = Px.n de-bounce function Disabled.
     * |        |          |1 = Px.n de-bounce function Enabled.
     * |        |          |The de-bounce function is valid only for edge triggered interrupt.
     * |        |          |If the interrupt mode is level triggered, the de-bounce enable bit is ignore.
     * |        |          |Note1: If Px.n pin is chosen as Power-down wake-up source, user should be disable the de-bounce function before entering Power-down mode to avoid the second interrupt event occurred after system waken up which caused by Px.n de-bounce function.
     * |        |          |Note2:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
    */
    __IO uint32_t DBEN;

    /**
     * INTTYPE
     * ===================================================================================================
     * Offset: 0x18  P0 Interrupt Mode Control
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TYPE0     |Port 0-5 Pin[N] Edge Or Level Detection Interrupt Trigger Type Control
     * |        |          |TYPE (Px_INTTYPE[n]) bit is used to control the triggered interrupt is by level trigger or by edge trigger.
     * |        |          |If the interrupt is by edge trigger, the trigger source can be controlled by de-bounce.
     * |        |          |If the interrupt is by level trigger, the input source is sampled by one HCLK clock and generates the interrupt.
     * |        |          |0 = Edge trigger interrupt.
     * |        |          |1 = Level trigger interrupt.
     * |        |          |If the pin is set as the level trigger interrupt, only one level can be set on the registers RHIEN (Px_INTEN[n+16])/FLIEN (Px_INTEN[n]).
     * |        |          |If both levels to trigger interrupt are set, the setting is ignored and no interrupt will occur.
     * |        |          |The de-bounce function is valid only for edge triggered interrupt.
     * |        |          |If the interrupt mode is level triggered, the de-bounce enable bit is ignore.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[1]     |TYPE1     |Port 0-5 Pin[N] Edge Or Level Detection Interrupt Trigger Type Control
     * |        |          |TYPE (Px_INTTYPE[n]) bit is used to control the triggered interrupt is by level trigger or by edge trigger.
     * |        |          |If the interrupt is by edge trigger, the trigger source can be controlled by de-bounce.
     * |        |          |If the interrupt is by level trigger, the input source is sampled by one HCLK clock and generates the interrupt.
     * |        |          |0 = Edge trigger interrupt.
     * |        |          |1 = Level trigger interrupt.
     * |        |          |If the pin is set as the level trigger interrupt, only one level can be set on the registers RHIEN (Px_INTEN[n+16])/FLIEN (Px_INTEN[n]).
     * |        |          |If both levels to trigger interrupt are set, the setting is ignored and no interrupt will occur.
     * |        |          |The de-bounce function is valid only for edge triggered interrupt.
     * |        |          |If the interrupt mode is level triggered, the de-bounce enable bit is ignore.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[2]     |TYPE2     |Port 0-5 Pin[N] Edge Or Level Detection Interrupt Trigger Type Control
     * |        |          |TYPE (Px_INTTYPE[n]) bit is used to control the triggered interrupt is by level trigger or by edge trigger.
     * |        |          |If the interrupt is by edge trigger, the trigger source can be controlled by de-bounce.
     * |        |          |If the interrupt is by level trigger, the input source is sampled by one HCLK clock and generates the interrupt.
     * |        |          |0 = Edge trigger interrupt.
     * |        |          |1 = Level trigger interrupt.
     * |        |          |If the pin is set as the level trigger interrupt, only one level can be set on the registers RHIEN (Px_INTEN[n+16])/FLIEN (Px_INTEN[n]).
     * |        |          |If both levels to trigger interrupt are set, the setting is ignored and no interrupt will occur.
     * |        |          |The de-bounce function is valid only for edge triggered interrupt.
     * |        |          |If the interrupt mode is level triggered, the de-bounce enable bit is ignore.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[3]     |TYPE3     |Port 0-5 Pin[N] Edge Or Level Detection Interrupt Trigger Type Control
     * |        |          |TYPE (Px_INTTYPE[n]) bit is used to control the triggered interrupt is by level trigger or by edge trigger.
     * |        |          |If the interrupt is by edge trigger, the trigger source can be controlled by de-bounce.
     * |        |          |If the interrupt is by level trigger, the input source is sampled by one HCLK clock and generates the interrupt.
     * |        |          |0 = Edge trigger interrupt.
     * |        |          |1 = Level trigger interrupt.
     * |        |          |If the pin is set as the level trigger interrupt, only one level can be set on the registers RHIEN (Px_INTEN[n+16])/FLIEN (Px_INTEN[n]).
     * |        |          |If both levels to trigger interrupt are set, the setting is ignored and no interrupt will occur.
     * |        |          |The de-bounce function is valid only for edge triggered interrupt.
     * |        |          |If the interrupt mode is level triggered, the de-bounce enable bit is ignore.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[4]     |TYPE4     |Port 0-5 Pin[N] Edge Or Level Detection Interrupt Trigger Type Control
     * |        |          |TYPE (Px_INTTYPE[n]) bit is used to control the triggered interrupt is by level trigger or by edge trigger.
     * |        |          |If the interrupt is by edge trigger, the trigger source can be controlled by de-bounce.
     * |        |          |If the interrupt is by level trigger, the input source is sampled by one HCLK clock and generates the interrupt.
     * |        |          |0 = Edge trigger interrupt.
     * |        |          |1 = Level trigger interrupt.
     * |        |          |If the pin is set as the level trigger interrupt, only one level can be set on the registers RHIEN (Px_INTEN[n+16])/FLIEN (Px_INTEN[n]).
     * |        |          |If both levels to trigger interrupt are set, the setting is ignored and no interrupt will occur.
     * |        |          |The de-bounce function is valid only for edge triggered interrupt.
     * |        |          |If the interrupt mode is level triggered, the de-bounce enable bit is ignore.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[5]     |TYPE5     |Port 0-5 Pin[N] Edge Or Level Detection Interrupt Trigger Type Control
     * |        |          |TYPE (Px_INTTYPE[n]) bit is used to control the triggered interrupt is by level trigger or by edge trigger.
     * |        |          |If the interrupt is by edge trigger, the trigger source can be controlled by de-bounce.
     * |        |          |If the interrupt is by level trigger, the input source is sampled by one HCLK clock and generates the interrupt.
     * |        |          |0 = Edge trigger interrupt.
     * |        |          |1 = Level trigger interrupt.
     * |        |          |If the pin is set as the level trigger interrupt, only one level can be set on the registers RHIEN (Px_INTEN[n+16])/FLIEN (Px_INTEN[n]).
     * |        |          |If both levels to trigger interrupt are set, the setting is ignored and no interrupt will occur.
     * |        |          |The de-bounce function is valid only for edge triggered interrupt.
     * |        |          |If the interrupt mode is level triggered, the de-bounce enable bit is ignore.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[6]     |TYPE6     |Port 0-5 Pin[N] Edge Or Level Detection Interrupt Trigger Type Control
     * |        |          |TYPE (Px_INTTYPE[n]) bit is used to control the triggered interrupt is by level trigger or by edge trigger.
     * |        |          |If the interrupt is by edge trigger, the trigger source can be controlled by de-bounce.
     * |        |          |If the interrupt is by level trigger, the input source is sampled by one HCLK clock and generates the interrupt.
     * |        |          |0 = Edge trigger interrupt.
     * |        |          |1 = Level trigger interrupt.
     * |        |          |If the pin is set as the level trigger interrupt, only one level can be set on the registers RHIEN (Px_INTEN[n+16])/FLIEN (Px_INTEN[n]).
     * |        |          |If both levels to trigger interrupt are set, the setting is ignored and no interrupt will occur.
     * |        |          |The de-bounce function is valid only for edge triggered interrupt.
     * |        |          |If the interrupt mode is level triggered, the de-bounce enable bit is ignore.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[7]     |TYPE7     |Port 0-5 Pin[N] Edge Or Level Detection Interrupt Trigger Type Control
     * |        |          |TYPE (Px_INTTYPE[n]) bit is used to control the triggered interrupt is by level trigger or by edge trigger.
     * |        |          |If the interrupt is by edge trigger, the trigger source can be controlled by de-bounce.
     * |        |          |If the interrupt is by level trigger, the input source is sampled by one HCLK clock and generates the interrupt.
     * |        |          |0 = Edge trigger interrupt.
     * |        |          |1 = Level trigger interrupt.
     * |        |          |If the pin is set as the level trigger interrupt, only one level can be set on the registers RHIEN (Px_INTEN[n+16])/FLIEN (Px_INTEN[n]).
     * |        |          |If both levels to trigger interrupt are set, the setting is ignored and no interrupt will occur.
     * |        |          |The de-bounce function is valid only for edge triggered interrupt.
     * |        |          |If the interrupt mode is level triggered, the de-bounce enable bit is ignore.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
    */
    __IO uint32_t INTTYPE;

    /**
     * INTEN
     * ===================================================================================================
     * Offset: 0x1C  P0 Interrupt Enable Control
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |FLIEN0    |Port 0-5 Pin[N] Falling Edge Or Low Level Interrupt Trigger Type Enable Bit
     * |        |          |The FLIEN (Px_INTEN[n]) bit is used to enable the interrupt for each of the corresponding input Px.n pin.
     * |        |          |Set bit to 1 also enable the pin wake-up function.
     * |        |          |When setting the FLIEN (Px_INTEN[n]) bit to 1:
     * |        |          |If the interrupt is level trigger (TYPE (Px_INTTYPE[n]) bit is set to 1), the input Px.n pin will generate the interrupt while this pin state is at low level.
     * |        |          |If the interrupt is edge trigger (TYPE (Px_INTTYPE[n]) bit is set to 0), the input Px.n pin will generate the interrupt while this pin state changed from high to low.
     * |        |          |0 = Px.n level low or high to low interrupt Disabled.
     * |        |          |1 = Px.n level low or high to low interrupt Enabled.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[1]     |FLIEN1    |Port 0-5 Pin[N] Falling Edge Or Low Level Interrupt Trigger Type Enable Bit
     * |        |          |The FLIEN (Px_INTEN[n]) bit is used to enable the interrupt for each of the corresponding input Px.n pin.
     * |        |          |Set bit to 1 also enable the pin wake-up function.
     * |        |          |When setting the FLIEN (Px_INTEN[n]) bit to 1:
     * |        |          |If the interrupt is level trigger (TYPE (Px_INTTYPE[n]) bit is set to 1), the input Px.n pin will generate the interrupt while this pin state is at low level.
     * |        |          |If the interrupt is edge trigger (TYPE (Px_INTTYPE[n]) bit is set to 0), the input Px.n pin will generate the interrupt while this pin state changed from high to low.
     * |        |          |0 = Px.n level low or high to low interrupt Disabled.
     * |        |          |1 = Px.n level low or high to low interrupt Enabled.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[2]     |FLIEN2    |Port 0-5 Pin[N] Falling Edge Or Low Level Interrupt Trigger Type Enable Bit
     * |        |          |The FLIEN (Px_INTEN[n]) bit is used to enable the interrupt for each of the corresponding input Px.n pin.
     * |        |          |Set bit to 1 also enable the pin wake-up function.
     * |        |          |When setting the FLIEN (Px_INTEN[n]) bit to 1:
     * |        |          |If the interrupt is level trigger (TYPE (Px_INTTYPE[n]) bit is set to 1), the input Px.n pin will generate the interrupt while this pin state is at low level.
     * |        |          |If the interrupt is edge trigger (TYPE (Px_INTTYPE[n]) bit is set to 0), the input Px.n pin will generate the interrupt while this pin state changed from high to low.
     * |        |          |0 = Px.n level low or high to low interrupt Disabled.
     * |        |          |1 = Px.n level low or high to low interrupt Enabled.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[3]     |FLIEN3    |Port 0-5 Pin[N] Falling Edge Or Low Level Interrupt Trigger Type Enable Bit
     * |        |          |The FLIEN (Px_INTEN[n]) bit is used to enable the interrupt for each of the corresponding input Px.n pin.
     * |        |          |Set bit to 1 also enable the pin wake-up function.
     * |        |          |When setting the FLIEN (Px_INTEN[n]) bit to 1:
     * |        |          |If the interrupt is level trigger (TYPE (Px_INTTYPE[n]) bit is set to 1), the input Px.n pin will generate the interrupt while this pin state is at low level.
     * |        |          |If the interrupt is edge trigger (TYPE (Px_INTTYPE[n]) bit is set to 0), the input Px.n pin will generate the interrupt while this pin state changed from high to low.
     * |        |          |0 = Px.n level low or high to low interrupt Disabled.
     * |        |          |1 = Px.n level low or high to low interrupt Enabled.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[4]     |FLIEN4    |Port 0-5 Pin[N] Falling Edge Or Low Level Interrupt Trigger Type Enable Bit
     * |        |          |The FLIEN (Px_INTEN[n]) bit is used to enable the interrupt for each of the corresponding input Px.n pin.
     * |        |          |Set bit to 1 also enable the pin wake-up function.
     * |        |          |When setting the FLIEN (Px_INTEN[n]) bit to 1:
     * |        |          |If the interrupt is level trigger (TYPE (Px_INTTYPE[n]) bit is set to 1), the input Px.n pin will generate the interrupt while this pin state is at low level.
     * |        |          |If the interrupt is edge trigger (TYPE (Px_INTTYPE[n]) bit is set to 0), the input Px.n pin will generate the interrupt while this pin state changed from high to low.
     * |        |          |0 = Px.n level low or high to low interrupt Disabled.
     * |        |          |1 = Px.n level low or high to low interrupt Enabled.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[5]     |FLIEN5    |Port 0-5 Pin[N] Falling Edge Or Low Level Interrupt Trigger Type Enable Bit
     * |        |          |The FLIEN (Px_INTEN[n]) bit is used to enable the interrupt for each of the corresponding input Px.n pin.
     * |        |          |Set bit to 1 also enable the pin wake-up function.
     * |        |          |When setting the FLIEN (Px_INTEN[n]) bit to 1:
     * |        |          |If the interrupt is level trigger (TYPE (Px_INTTYPE[n]) bit is set to 1), the input Px.n pin will generate the interrupt while this pin state is at low level.
     * |        |          |If the interrupt is edge trigger (TYPE (Px_INTTYPE[n]) bit is set to 0), the input Px.n pin will generate the interrupt while this pin state changed from high to low.
     * |        |          |0 = Px.n level low or high to low interrupt Disabled.
     * |        |          |1 = Px.n level low or high to low interrupt Enabled.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[6]     |FLIEN6    |Port 0-5 Pin[N] Falling Edge Or Low Level Interrupt Trigger Type Enable Bit
     * |        |          |The FLIEN (Px_INTEN[n]) bit is used to enable the interrupt for each of the corresponding input Px.n pin.
     * |        |          |Set bit to 1 also enable the pin wake-up function.
     * |        |          |When setting the FLIEN (Px_INTEN[n]) bit to 1:
     * |        |          |If the interrupt is level trigger (TYPE (Px_INTTYPE[n]) bit is set to 1), the input Px.n pin will generate the interrupt while this pin state is at low level.
     * |        |          |If the interrupt is edge trigger (TYPE (Px_INTTYPE[n]) bit is set to 0), the input Px.n pin will generate the interrupt while this pin state changed from high to low.
     * |        |          |0 = Px.n level low or high to low interrupt Disabled.
     * |        |          |1 = Px.n level low or high to low interrupt Enabled.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[7]     |FLIEN7    |Port 0-5 Pin[N] Falling Edge Or Low Level Interrupt Trigger Type Enable Bit
     * |        |          |The FLIEN (Px_INTEN[n]) bit is used to enable the interrupt for each of the corresponding input Px.n pin.
     * |        |          |Set bit to 1 also enable the pin wake-up function.
     * |        |          |When setting the FLIEN (Px_INTEN[n]) bit to 1function
     * |        |          |If the interrupt is level trigger (TYPE (Px_INTTYPE[n]) bit is set to 1), the input Px.n pin will generate the interrupt while this pin state is at low level.
     * |        |          |If the interrupt is edge trigger (TYPE (Px_INTTYPE[n]) bit is set to 0), the input Px.n pin will generate the interrupt while this pin state changed from high to low.
     * |        |          |0 = Px.n level low or high to low interrupt Disabled.
     * |        |          |1 = Px.n level low or high to low interrupt Enabled.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[16]    |RHIEN0    |Port 0-5 Pin[N] Rising Edge Or High Level Interrupt Trigger Type Enable Bit
     * |        |          |The RHIEN (Px_INTEN[n+16]) bit is used to enable the interrupt for each of the corresponding input Px.n pin.
     * |        |          |Set bit to 1 also enable the pin wake-up function.
     * |        |          |When setting the RHIEN (Px_INTEN[n+16]) bit to 1:
     * |        |          |If the interrupt is level trigger (TYPE (Px_INTTYPE[n]) bit is set to 1), the input Px.n pin will generate the interrupt while this pin state is at high level.
     * |        |          |If the interrupt is edge trigger (TYPE (Px_INTTYPE[n]) bit is set to 0), the input Px.n pin will generate the interrupt while this pin state changed from low to high.
     * |        |          |0 = Px.n level high or low to high interrupt Disabled.
     * |        |          |1 = Px.n level high or low to high interrupt Enabled.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[17]    |RHIEN1    |Port 0-5 Pin[N] Rising Edge Or High Level Interrupt Trigger Type Enable Bit
     * |        |          |The RHIEN (Px_INTEN[n+16]) bit is used to enable the interrupt for each of the corresponding input Px.n pin.
     * |        |          |Set bit to 1 also enable the pin wake-up function.
     * |        |          |When setting the RHIEN (Px_INTEN[n+16]) bit to 1:
     * |        |          |If the interrupt is level trigger (TYPE (Px_INTTYPE[n]) bit is set to 1), the input Px.n pin will generate the interrupt while this pin state is at high level.
     * |        |          |If the interrupt is edge trigger (TYPE (Px_INTTYPE[n]) bit is set to 0), the input Px.n pin will generate the interrupt while this pin state changed from low to high.
     * |        |          |0 = Px.n level high or low to high interrupt Disabled.
     * |        |          |1 = Px.n level high or low to high interrupt Enabled.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[18]    |RHIEN2    |Port 0-5 Pin[N] Rising Edge Or High Level Interrupt Trigger Type Enable Bit
     * |        |          |The RHIEN (Px_INTEN[n+16]) bit is used to enable the interrupt for each of the corresponding input Px.n pin.
     * |        |          |Set bit to 1 also enable the pin wake-up function.
     * |        |          |When setting the RHIEN (Px_INTEN[n+16]) bit to 1:
     * |        |          |If the interrupt is level trigger (TYPE (Px_INTTYPE[n]) bit is set to 1), the input Px.n pin will generate the interrupt while this pin state is at high level.
     * |        |          |If the interrupt is edge trigger (TYPE (Px_INTTYPE[n]) bit is set to 0), the input Px.n pin will generate the interrupt while this pin state changed from low to high.
     * |        |          |0 = Px.n level high or low to high interrupt Disabled.
     * |        |          |1 = Px.n level high or low to high interrupt Enabled.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[19]    |RHIEN3    |Port 0-5 Pin[N] Rising Edge Or High Level Interrupt Trigger Type Enable Bit
     * |        |          |The RHIEN (Px_INTEN[n+16]) bit is used to enable the interrupt for each of the corresponding input Px.n pin.
     * |        |          |Set bit to 1 also enable the pin wake-up function.
     * |        |          |When setting the RHIEN (Px_INTEN[n+16]) bit to 1:
     * |        |          |If the interrupt is level trigger (TYPE (Px_INTTYPE[n]) bit is set to 1), the input Px.n pin will generate the interrupt while this pin state is at high level.
     * |        |          |If the interrupt is edge trigger (TYPE (Px_INTTYPE[n]) bit is set to 0), the input Px.n pin will generate the interrupt while this pin state changed from low to high.
     * |        |          |0 = Px.n level high or low to high interrupt Disabled.
     * |        |          |1 = Px.n level high or low to high interrupt Enabled.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[20]    |RHIEN4    |Port 0-5 Pin[N] Rising Edge Or High Level Interrupt Trigger Type Enable Bit
     * |        |          |The RHIEN (Px_INTEN[n+16]) bit is used to enable the interrupt for each of the corresponding input Px.n pin.
     * |        |          |Set bit to 1 also enable the pin wake-up function.
     * |        |          |When setting the RHIEN (Px_INTEN[n+16]) bit to 1:
     * |        |          |If the interrupt is level trigger (TYPE (Px_INTTYPE[n]) bit is set to 1), the input Px.n pin will generate the interrupt while this pin state is at high level.
     * |        |          |If the interrupt is edge trigger (TYPE (Px_INTTYPE[n]) bit is set to 0), the input Px.n pin will generate the interrupt while this pin state changed from low to high.
     * |        |          |0 = Px.n level high or low to high interrupt Disabled.
     * |        |          |1 = Px.n level high or low to high interrupt Enabled.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[21]    |RHIEN5    |Port 0-5 Pin[N] Rising Edge Or High Level Interrupt Trigger Type Enable Bit
     * |        |          |The RHIEN (Px_INTEN[n+16]) bit is used to enable the interrupt for each of the corresponding input Px.n pin.
     * |        |          |Set bit to 1 also enable the pin wake-up function.
     * |        |          |When setting the RHIEN (Px_INTEN[n+16]) bit to 1:
     * |        |          |If the interrupt is level trigger (TYPE (Px_INTTYPE[n]) bit is set to 1), the input Px.n pin will generate the interrupt while this pin state is at high level.
     * |        |          |If the interrupt is edge trigger (TYPE (Px_INTTYPE[n]) bit is set to 0), the input Px.n pin will generate the interrupt while this pin state changed from low to high.
     * |        |          |0 = Px.n level high or low to high interrupt Disabled.
     * |        |          |1 = Px.n level high or low to high interrupt Enabled.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[22]    |RHIEN6    |Port 0-5 Pin[N] Rising Edge Or High Level Interrupt Trigger Type Enable Bit
     * |        |          |The RHIEN (Px_INTEN[n+16]) bit is used to enable the interrupt for each of the corresponding input Px.n pin.
     * |        |          |Set bit to 1 also enable the pin wake-up function.
     * |        |          |When setting the RHIEN (Px_INTEN[n+16]) bit to 1:
     * |        |          |If the interrupt is level trigger (TYPE (Px_INTTYPE[n]) bit is set to 1), the input Px.n pin will generate the interrupt while this pin state is at high level.
     * |        |          |If the interrupt is edge trigger (TYPE (Px_INTTYPE[n]) bit is set to 0), the input Px.n pin will generate the interrupt while this pin state changed from low to high.
     * |        |          |0 = Px.n level high or low to high interrupt Disabled.
     * |        |          |1 = Px.n level high or low to high interrupt Enabled.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[23]    |RHIEN7    |Port 0-5 Pin[N] Rising Edge Or High Level Interrupt Trigger Type Enable Bit
     * |        |          |The RHIEN (Px_INTEN[n+16]) bit is used to enable the interrupt for each of the corresponding input Px.n pin.
     * |        |          |Set bit to 1 also enable the pin wake-up function.
     * |        |          |When setting the RHIEN (Px_INTEN[n+16]) bit to 1:
     * |        |          |If the interrupt is level trigger (TYPE (Px_INTTYPE[n]) bit is set to 1), the input Px.n pin will generate the interrupt while this pin state is at high level.
     * |        |          |If the interrupt is edge trigger (TYPE (Px_INTTYPE[n]) bit is set to 0), the input Px.n pin will generate the interrupt while this pin state changed from low to high.
     * |        |          |0 = Px.n level high or low to high interrupt Disabled.
     * |        |          |1 = Px.n level high or low to high interrupt Enabled.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
    */
    __IO uint32_t INTEN;

    /**
     * INTSRC
     * ===================================================================================================
     * Offset: 0x20  P0 Interrupt Source Flag
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |INTSRC0   |Port 0-5 Pin[N] Interrupt Source Flag
     * |        |          |Write Operation:
     * |        |          |0 = No action.
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |Read Operation:
     * |        |          |0 = No interrupt at Px.n.
     * |        |          |1 = Px.n generates an interrupt.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[1]     |INTSRC1   |Port 0-5 Pin[N] Interrupt Source Flag
     * |        |          |Write Operation:
     * |        |          |0 = No action.
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |Read Operation:
     * |        |          |0 = No interrupt at Px.n.
     * |        |          |1 = Px.n generates an interrupt.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[2]     |INTSRC2   |Port 0-5 Pin[N] Interrupt Source Flag
     * |        |          |Write Operation:
     * |        |          |0 = No action.
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |Read Operation:
     * |        |          |0 = No interrupt at Px.n.
     * |        |          |1 = Px.n generates an interrupt.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[3]     |INTSRC3   |Port 0-5 Pin[N] Interrupt Source Flag
     * |        |          |Write Operation:
     * |        |          |0 = No action.
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |Read Operation:
     * |        |          |0 = No interrupt at Px.n.
     * |        |          |1 = Px.n generates an interrupt.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[4]     |INTSRC4   |Port 0-5 Pin[N] Interrupt Source Flag
     * |        |          |Write Operation:
     * |        |          |0 = No action.
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |Read Operation:
     * |        |          |0 = No interrupt at Px.n.
     * |        |          |1 = Px.n generates an interrupt.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[5]     |INTSRC5   |Port 0-5 Pin[N] Interrupt Source Flag
     * |        |          |Write Operation:
     * |        |          |0 = No action.
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |Read Operation:
     * |        |          |0 = No interrupt at Px.n.
     * |        |          |1 = Px.n generates an interrupt.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[6]     |INTSRC6   |Port 0-5 Pin[N] Interrupt Source Flag
     * |        |          |Write Operation:
     * |        |          |0 = No action.
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |Read Operation:
     * |        |          |0 = No interrupt at Px.n.
     * |        |          |1 = Px.n generates an interrupt.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[7]     |INTSRC7   |Port 0-5 Pin[N] Interrupt Source Flag
     * |        |          |Write Operation:
     * |        |          |0 = No action.
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |Read Operation:
     * |        |          |0 = No interrupt at Px.n.
     * |        |          |1 = Px.n generates an interrupt.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[8]     |INTSRC8   |Port 0-5 Pin[N] Interrupt Source Flag
     * |        |          |Write Operation:
     * |        |          |0 = No action.
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |Read Operation:
     * |        |          |0 = No interrupt at Px.n.
     * |        |          |1 = Px.n generates an interrupt.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[9]     |INTSRC9   |Port 0-5 Pin[N] Interrupt Source Flag
     * |        |          |Write Operation:
     * |        |          |0 = No action.
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |Read Operation:
     * |        |          |0 = No interrupt at Px.n.
     * |        |          |1 = Px.n generates an interrupt.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[10]    |INTSRC10  |Port 0-5 Pin[N] Interrupt Source Flag
     * |        |          |Write Operation:
     * |        |          |0 = No action.
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |Read Operation:
     * |        |          |0 = No interrupt at Px.n.
     * |        |          |1 = Px.n generates an interrupt.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[11]    |INTSRC11  |Port 0-5 Pin[N] Interrupt Source Flag
     * |        |          |Write Operation:
     * |        |          |0 = No action.
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |Read Operation:
     * |        |          |0 = No interrupt at Px.n.
     * |        |          |1 = Px.n generates an interrupt.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[12]    |INTSRC12  |Port 0-5 Pin[N] Interrupt Source Flag
     * |        |          |Write Operation:
     * |        |          |0 = No action.
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |Read Operation:
     * |        |          |0 = No interrupt at Px.n.
     * |        |          |1 = Px.n generates an interrupt.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[13]    |INTSRC13  |Port 0-5 Pin[N] Interrupt Source Flag
     * |        |          |Write Operation:
     * |        |          |0 = No action.
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |Read Operation:
     * |        |          |0 = No interrupt at Px.n.
     * |        |          |1 = Px.n generates an interrupt.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[14]    |INTSRC14  |Port 0-5 Pin[N] Interrupt Source Flag
     * |        |          |Write Operation:
     * |        |          |0 = No action.
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |Read Operation:
     * |        |          |0 = No interrupt at Px.n.
     * |        |          |1 = Px.n generates an interrupt.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
     * |[15]    |INTSRC15  |Port 0-5 Pin[N] Interrupt Source Flag
     * |        |          |Write Operation:
     * |        |          |0 = No action.
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |Read Operation:
     * |        |          |0 = No interrupt at Px.n.
     * |        |          |1 = Px.n generates an interrupt.
     * |        |          |Note:
     * |        |          |Max. n=7 for port 0.
     * |        |          |Max. n=7 for port 1.
     * |        |          |Max. n=7 for port 2.
     * |        |          |Max. n=7 for port 3, n=3, n=7 are reserved.
     * |        |          |Max. n=7 for port 4, n=0,.5 are reserved.
     * |        |          |Max. n=7 for port 5.
    */
    __IO uint32_t INTSRC;

} GPIO_T;

typedef struct {

    /**
     * DBCTL
     * ===================================================================================================
     * Offset: 0x180  De-bounce Cycle Control
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |DBCLKSEL  |De-bounce Sampling Cycle Selection
     * |        |          |0000 = Sample interrupt input once per 1 clock.
     * |        |          |0001 = Sample interrupt input once per 2 clocks.
     * |        |          |0010 = Sample interrupt input once per 4 clocks.
     * |        |          |0011 = Sample interrupt input once per 8 clocks.
     * |        |          |0100 = Sample interrupt input once per 16 clocks.
     * |        |          |0101 = Sample interrupt input once per 32 clocks.
     * |        |          |0110 = Sample interrupt input once per 64 clocks.
     * |        |          |0111 = Sample interrupt input once per 128 clocks.
     * |        |          |1000 = Sample interrupt input once per 256 clocks.
     * |        |          |1001 = Sample interrupt input once per 2*256 clocks.
     * |        |          |1010 = Sample interrupt input once per 4*256 clocks.
     * |        |          |1011 = Sample interrupt input once per 8*256 clocks.
     * |        |          |1100 = Sample interrupt input once per 16*256 clocks.
     * |        |          |1101 = Sample interrupt input once per 32*256 clocks.
     * |        |          |1110 = Sample interrupt input once per 64*256 clocks.
     * |        |          |1111 = Sample interrupt input once per 128*256 clocks.
     * |[4]     |DBCLKSRC  |De-bounce Counter Clock Source Selection
     * |        |          |0 = De-bounce counter clock source is HCLK.
     * |        |          |1 = De-bounce counter clock source is 10 kHz internal low speed RC oscillator (LIRC).
    */
    __IO uint32_t DBCTL;

} GPIO_DB_T;


/**
    @addtogroup GP_CONST GP Bit Field Definition
    Constant Definitions for GP Controller
@{ */

#define GP_MODE_MODE0_Pos                (0)                                               /*!< GP_T::MODE: MODE0 Position                */
#define GP_MODE_MODE0_Msk                (0x3ul << GP_MODE_MODE0_Pos)                      /*!< GP_T::MODE: MODE0 Mask                    */

#define GP_MODE_MODE1_Pos                (2)                                               /*!< GP_T::MODE: MODE1 Position                */
#define GP_MODE_MODE1_Msk                (0x3ul << GP_MODE_MODE1_Pos)                      /*!< GP_T::MODE: MODE1 Mask                    */

#define GP_MODE_MODE2_Pos                (4)                                               /*!< GP_T::MODE: MODE2 Position                */
#define GP_MODE_MODE2_Msk                (0x3ul << GP_MODE_MODE2_Pos)                      /*!< GP_T::MODE: MODE2 Mask                    */

#define GP_MODE_MODE3_Pos                (6)                                               /*!< GP_T::MODE: MODE3 Position                */
#define GP_MODE_MODE3_Msk                (0x3ul << GP_MODE_MODE3_Pos)                      /*!< GP_T::MODE: MODE3 Mask                    */

#define GP_MODE_MODE4_Pos                (8)                                               /*!< GP_T::MODE: MODE4 Position                */
#define GP_MODE_MODE4_Msk                (0x3ul << GP_MODE_MODE4_Pos)                      /*!< GP_T::MODE: MODE4 Mask                    */

#define GP_MODE_MODE5_Pos                (10)                                              /*!< GP_T::MODE: MODE5 Position                */
#define GP_MODE_MODE5_Msk                (0x3ul << GP_MODE_MODE5_Pos)                      /*!< GP_T::MODE: MODE5 Mask                    */

#define GP_DINOFF_DINOFF0_Pos            (16)                                              /*!< GP_T::DINOFF: DINOFF0 Position            */
#define GP_DINOFF_DINOFF0_Msk            (0x1ul << GP_DINOFF_DINOFF0_Pos)                  /*!< GP_T::DINOFF: DINOFF0 Mask                */

#define GP_DINOFF_DINOFF1_Pos            (17)                                              /*!< GP_T::DINOFF: DINOFF1 Position            */
#define GP_DINOFF_DINOFF1_Msk            (0x1ul << GP_DINOFF_DINOFF1_Pos)                  /*!< GP_T::DINOFF: DINOFF1 Mask                */

#define GP_DINOFF_DINOFF2_Pos            (18)                                              /*!< GP_T::DINOFF: DINOFF2 Position            */
#define GP_DINOFF_DINOFF2_Msk            (0x1ul << GP_DINOFF_DINOFF2_Pos)                  /*!< GP_T::DINOFF: DINOFF2 Mask                */

#define GP_DINOFF_DINOFF3_Pos            (19)                                              /*!< GP_T::DINOFF: DINOFF3 Position            */
#define GP_DINOFF_DINOFF3_Msk            (0x1ul << GP_DINOFF_DINOFF3_Pos)                  /*!< GP_T::DINOFF: DINOFF3 Mask                */

#define GP_DINOFF_DINOFF4_Pos            (20)                                              /*!< GP_T::DINOFF: DINOFF4 Position            */
#define GP_DINOFF_DINOFF4_Msk            (0x1ul << GP_DINOFF_DINOFF4_Pos)                  /*!< GP_T::DINOFF: DINOFF4 Mask                */

#define GP_DINOFF_DINOFF5_Pos            (21)                                              /*!< GP_T::DINOFF: DINOFF5 Position            */
#define GP_DINOFF_DINOFF5_Msk            (0x1ul << GP_DINOFF_DINOFF5_Pos)                  /*!< GP_T::DINOFF: DINOFF5 Mask                */

#define GP_DINOFF_DINOFF6_Pos            (22)                                              /*!< GP_T::DINOFF: DINOFF6 Position            */
#define GP_DINOFF_DINOFF6_Msk            (0x1ul << GP_DINOFF_DINOFF6_Pos)                  /*!< GP_T::DINOFF: DINOFF6 Mask                */

#define GP_DINOFF_DINOFF7_Pos            (23)                                              /*!< GP_T::DINOFF: DINOFF7 Position            */
#define GP_DINOFF_DINOFF7_Msk            (0x1ul << GP_DINOFF_DINOFF7_Pos)                  /*!< GP_T::DINOFF: DINOFF7 Mask                */

#define GP_DOUT_DOUT0_Pos                (0)                                               /*!< GP_T::DOUT: DOUT0 Position                */
#define GP_DOUT_DOUT0_Msk                (0x1ul << GP_DOUT_DOUT0_Pos)                      /*!< GP_T::DOUT: DOUT0 Mask                    */

#define GP_DOUT_DOUT1_Pos                (1)                                               /*!< GP_T::DOUT: DOUT1 Position                */
#define GP_DOUT_DOUT1_Msk                (0x1ul << GP_DOUT_DOUT1_Pos)                      /*!< GP_T::DOUT: DOUT1 Mask                    */

#define GP_DOUT_DOUT2_Pos                (2)                                               /*!< GP_T::DOUT: DOUT2 Position                */
#define GP_DOUT_DOUT2_Msk                (0x1ul << GP_DOUT_DOUT2_Pos)                      /*!< GP_T::DOUT: DOUT2 Mask                    */

#define GP_DOUT_DOUT3_Pos                (3)                                               /*!< GP_T::DOUT: DOUT3 Position                */
#define GP_DOUT_DOUT3_Msk                (0x1ul << GP_DOUT_DOUT3_Pos)                      /*!< GP_T::DOUT: DOUT3 Mask                    */

#define GP_DOUT_DOUT4_Pos                (4)                                               /*!< GP_T::DOUT: DOUT4 Position                */
#define GP_DOUT_DOUT4_Msk                (0x1ul << GP_DOUT_DOUT4_Pos)                      /*!< GP_T::DOUT: DOUT4 Mask                    */

#define GP_DOUT_DOUT5_Pos                (5)                                               /*!< GP_T::DOUT: DOUT5 Position                */
#define GP_DOUT_DOUT5_Msk                (0x1ul << GP_DOUT_DOUT5_Pos)                      /*!< GP_T::DOUT: DOUT5 Mask                    */

#define GP_DOUT_DOUT6_Pos                (6)                                               /*!< GP_T::DOUT: DOUT6 Position                */
#define GP_DOUT_DOUT6_Msk                (0x1ul << GP_DOUT_DOUT6_Pos)                      /*!< GP_T::DOUT: DOUT6 Mask                    */

#define GP_DOUT_DOUT7_Pos                (7)                                               /*!< GP_T::DOUT: DOUT7 Position                */
#define GP_DOUT_DOUT7_Msk                (0x1ul << GP_DOUT_DOUT7_Pos)                      /*!< GP_T::DOUT: DOUT7 Mask                    */

#define GP_DATMSK_DATMSK0_Pos            (0)                                               /*!< GP_T::DATMSK: DATMSK0 Position            */
#define GP_DATMSK_DATMSK0_Msk            (0x1ul << GP_DATMSK_DATMSK0_Pos)                  /*!< GP_T::DATMSK: DATMSK0 Mask                */

#define GP_DATMSK_DATMSK1_Pos            (1)                                               /*!< GP_T::DATMSK: DATMSK1 Position            */
#define GP_DATMSK_DATMSK1_Msk            (0x1ul << GP_DATMSK_DATMSK1_Pos)                  /*!< GP_T::DATMSK: DATMSK1 Mask                */

#define GP_DATMSK_DATMSK2_Pos            (2)                                               /*!< GP_T::DATMSK: DATMSK2 Position            */
#define GP_DATMSK_DATMSK2_Msk            (0x1ul << GP_DATMSK_DATMSK2_Pos)                  /*!< GP_T::DATMSK: DATMSK2 Mask                */

#define GP_DATMSK_DATMSK3_Pos            (3)                                               /*!< GP_T::DATMSK: DATMSK3 Position            */
#define GP_DATMSK_DATMSK3_Msk            (0x1ul << GP_DATMSK_DATMSK3_Pos)                  /*!< GP_T::DATMSK: DATMSK3 Mask                */

#define GP_DATMSK_DATMSK4_Pos            (4)                                               /*!< GP_T::DATMSK: DATMSK4 Position            */
#define GP_DATMSK_DATMSK4_Msk            (0x1ul << GP_DATMSK_DATMSK4_Pos)                  /*!< GP_T::DATMSK: DATMSK4 Mask                */

#define GP_DATMSK_DATMSK5_Pos            (5)                                               /*!< GP_T::DATMSK: DATMSK5 Position            */
#define GP_DATMSK_DATMSK5_Msk            (0x1ul << GP_DATMSK_DATMSK5_Pos)                  /*!< GP_T::DATMSK: DATMSK5 Mask                */

#define GP_DATMSK_DATMSK6_Pos            (6)                                               /*!< GP_T::DATMSK: DATMSK6 Position            */
#define GP_DATMSK_DATMSK6_Msk            (0x1ul << GP_DATMSK_DATMSK6_Pos)                  /*!< GP_T::DATMSK: DATMSK6 Mask                */

#define GP_DATMSK_DATMSK7_Pos            (7)                                               /*!< GP_T::DATMSK: DATMSK7 Position            */
#define GP_DATMSK_DATMSK7_Msk            (0x1ul << GP_DATMSK_DATMSK7_Pos)                  /*!< GP_T::DATMSK: DATMSK7 Mask                */

#define GP_PIN_PIN0_Pos                  (0)                                               /*!< GP_T::PIN: PIN0 Position                  */
#define GP_PIN_PIN0_Msk                  (0x1ul << GP_PIN_PIN0_Pos)                        /*!< GP_T::PIN: PIN0 Mask                      */

#define GP_PIN_PIN1_Pos                  (1)                                               /*!< GP_T::PIN: PIN1 Position                  */
#define GP_PIN_PIN1_Msk                  (0x1ul << GP_PIN_PIN1_Pos)                        /*!< GP_T::PIN: PIN1 Mask                      */

#define GP_PIN_PIN2_Pos                  (2)                                               /*!< GP_T::PIN: PIN2 Position                  */
#define GP_PIN_PIN2_Msk                  (0x1ul << GP_PIN_PIN2_Pos)                        /*!< GP_T::PIN: PIN2 Mask                      */

#define GP_PIN_PIN3_Pos                  (3)                                               /*!< GP_T::PIN: PIN3 Position                  */
#define GP_PIN_PIN3_Msk                  (0x1ul << GP_PIN_PIN3_Pos)                        /*!< GP_T::PIN: PIN3 Mask                      */

#define GP_PIN_PIN4_Pos                  (4)                                               /*!< GP_T::PIN: PIN4 Position                  */
#define GP_PIN_PIN4_Msk                  (0x1ul << GP_PIN_PIN4_Pos)                        /*!< GP_T::PIN: PIN4 Mask                      */

#define GP_PIN_PIN5_Pos                  (5)                                               /*!< GP_T::PIN: PIN5 Position                  */
#define GP_PIN_PIN5_Msk                  (0x1ul << GP_PIN_PIN5_Pos)                        /*!< GP_T::PIN: PIN5 Mask                      */

#define GP_PIN_PIN6_Pos                  (6)                                               /*!< GP_T::PIN: PIN6 Position                  */
#define GP_PIN_PIN6_Msk                  (0x1ul << GP_PIN_PIN6_Pos)                        /*!< GP_T::PIN: PIN6 Mask                      */

#define GP_PIN_PIN7_Pos                  (7)                                               /*!< GP_T::PIN: PIN7 Position                  */
#define GP_PIN_PIN7_Msk                  (0x1ul << GP_PIN_PIN7_Pos)                        /*!< GP_T::PIN: PIN7 Mask                      */

#define GP_DBEN_DBEN0_Pos                (0)                                               /*!< GP_T::DBEN: DBEN0 Position                */
#define GP_DBEN_DBEN0_Msk                (0x1ul << GP_DBEN_DBEN0_Pos)                      /*!< GP_T::DBEN: DBEN0 Mask                    */

#define GP_DBEN_DBEN1_Pos                (1)                                               /*!< GP_T::DBEN: DBEN1 Position                */
#define GP_DBEN_DBEN1_Msk                (0x1ul << GP_DBEN_DBEN1_Pos)                      /*!< GP_T::DBEN: DBEN1 Mask                    */

#define GP_DBEN_DBEN2_Pos                (2)                                               /*!< GP_T::DBEN: DBEN2 Position                */
#define GP_DBEN_DBEN2_Msk                (0x1ul << GP_DBEN_DBEN2_Pos)                      /*!< GP_T::DBEN: DBEN2 Mask                    */

#define GP_DBEN_DBEN3_Pos                (3)                                               /*!< GP_T::DBEN: DBEN3 Position                */
#define GP_DBEN_DBEN3_Msk                (0x1ul << GP_DBEN_DBEN3_Pos)                      /*!< GP_T::DBEN: DBEN3 Mask                    */

#define GP_DBEN_DBEN4_Pos                (4)                                               /*!< GP_T::DBEN: DBEN4 Position                */
#define GP_DBEN_DBEN4_Msk                (0x1ul << GP_DBEN_DBEN4_Pos)                      /*!< GP_T::DBEN: DBEN4 Mask                    */

#define GP_DBEN_DBEN5_Pos                (5)                                               /*!< GP_T::DBEN: DBEN5 Position                */
#define GP_DBEN_DBEN5_Msk                (0x1ul << GP_DBEN_DBEN5_Pos)                      /*!< GP_T::DBEN: DBEN5 Mask                    */

#define GP_DBEN_DBEN6_Pos                (6)                                               /*!< GP_T::DBEN: DBEN6 Position                */
#define GP_DBEN_DBEN6_Msk                (0x1ul << GP_DBEN_DBEN6_Pos)                      /*!< GP_T::DBEN: DBEN6 Mask                    */

#define GP_DBEN_DBEN7_Pos                (7)                                               /*!< GP_T::DBEN: DBEN7 Position                */
#define GP_DBEN_DBEN7_Msk                (0x1ul << GP_DBEN_DBEN7_Pos)                      /*!< GP_T::DBEN: DBEN7 Mask                    */

#define GP_INTTYPE_TYPE0_Pos             (0)                                               /*!< GP_T::INTTYPE: TYPE0 Position             */
#define GP_INTTYPE_TYPE0_Msk             (0x1ul << GP_INTTYPE_TYPE0_Pos)                   /*!< GP_T::INTTYPE: TYPE0 Mask                 */

#define GP_INTTYPE_TYPE1_Pos             (1)                                               /*!< GP_T::INTTYPE: TYPE1 Position             */
#define GP_INTTYPE_TYPE1_Msk             (0x1ul << GP_INTTYPE_TYPE1_Pos)                   /*!< GP_T::INTTYPE: TYPE1 Mask                 */

#define GP_INTTYPE_TYPE2_Pos             (2)                                               /*!< GP_T::INTTYPE: TYPE2 Position             */
#define GP_INTTYPE_TYPE2_Msk             (0x1ul << GP_INTTYPE_TYPE2_Pos)                   /*!< GP_T::INTTYPE: TYPE2 Mask                 */

#define GP_INTTYPE_TYPE3_Pos             (3)                                               /*!< GP_T::INTTYPE: TYPE3 Position             */
#define GP_INTTYPE_TYPE3_Msk             (0x1ul << GP_INTTYPE_TYPE3_Pos)                   /*!< GP_T::INTTYPE: TYPE3 Mask                 */

#define GP_INTTYPE_TYPE4_Pos             (4)                                               /*!< GP_T::INTTYPE: TYPE4 Position             */
#define GP_INTTYPE_TYPE4_Msk             (0x1ul << GP_INTTYPE_TYPE4_Pos)                   /*!< GP_T::INTTYPE: TYPE4 Mask                 */

#define GP_INTTYPE_TYPE5_Pos             (5)                                               /*!< GP_T::INTTYPE: TYPE5 Position             */
#define GP_INTTYPE_TYPE5_Msk             (0x1ul << GP_INTTYPE_TYPE5_Pos)                   /*!< GP_T::INTTYPE: TYPE5 Mask                 */

#define GP_INTTYPE_TYPE6_Pos             (6)                                               /*!< GP_T::INTTYPE: TYPE6 Position             */
#define GP_INTTYPE_TYPE6_Msk             (0x1ul << GP_INTTYPE_TYPE6_Pos)                   /*!< GP_T::INTTYPE: TYPE6 Mask                 */

#define GP_INTTYPE_TYPE7_Pos             (7)                                               /*!< GP_T::INTTYPE: TYPE7 Position             */
#define GP_INTTYPE_TYPE7_Msk             (0x1ul << GP_INTTYPE_TYPE7_Pos)                   /*!< GP_T::INTTYPE: TYPE7 Mask                 */

#define GP_INTEN_FLIEN0_Pos              (0)                                               /*!< GP_T::INTEN: FLIEN0 Position              */
#define GP_INTEN_FLIEN0_Msk              (0x1ul << GP_INTEN_FLIEN0_Pos)                    /*!< GP_T::INTEN: FLIEN0 Mask                  */

#define GP_INTEN_FLIEN1_Pos              (1)                                               /*!< GP_T::INTEN: FLIEN1 Position              */
#define GP_INTEN_FLIEN1_Msk              (0x1ul << GP_INTEN_FLIEN1_Pos)                    /*!< GP_T::INTEN: FLIEN1 Mask                  */

#define GP_INTEN_FLIEN2_Pos              (2)                                               /*!< GP_T::INTEN: FLIEN2 Position              */
#define GP_INTEN_FLIEN2_Msk              (0x1ul << GP_INTEN_FLIEN2_Pos)                    /*!< GP_T::INTEN: FLIEN2 Mask                  */

#define GP_INTEN_FLIEN3_Pos              (3)                                               /*!< GP_T::INTEN: FLIEN3 Position              */
#define GP_INTEN_FLIEN3_Msk              (0x1ul << GP_INTEN_FLIEN3_Pos)                    /*!< GP_T::INTEN: FLIEN3 Mask                  */

#define GP_INTEN_FLIEN4_Pos              (4)                                               /*!< GP_T::INTEN: FLIEN4 Position              */
#define GP_INTEN_FLIEN4_Msk              (0x1ul << GP_INTEN_FLIEN4_Pos)                    /*!< GP_T::INTEN: FLIEN4 Mask                  */

#define GP_INTEN_FLIEN5_Pos              (5)                                               /*!< GP_T::INTEN: FLIEN5 Position              */
#define GP_INTEN_FLIEN5_Msk              (0x1ul << GP_INTEN_FLIEN5_Pos)                    /*!< GP_T::INTEN: FLIEN5 Mask                  */

#define GP_INTEN_FLIEN6_Pos              (6)                                               /*!< GP_T::INTEN: FLIEN6 Position              */
#define GP_INTEN_FLIEN6_Msk              (0x1ul << GP_INTEN_FLIEN6_Pos)                    /*!< GP_T::INTEN: FLIEN6 Mask                  */

#define GP_INTEN_FLIEN7_Pos              (7)                                               /*!< GP_T::INTEN: FLIEN7 Position              */
#define GP_INTEN_FLIEN7_Msk              (0x1ul << GP_INTEN_FLIEN7_Pos)                    /*!< GP_T::INTEN: FLIEN7 Mask                  */

#define GP_INTEN_RHIEN0_Pos              (16)                                              /*!< GP_T::INTEN: RHIEN0 Position              */
#define GP_INTEN_RHIEN0_Msk              (0x1ul << GP_INTEN_RHIEN0_Pos)                    /*!< GP_T::INTEN: RHIEN0 Mask                  */

#define GP_INTEN_RHIEN1_Pos              (17)                                              /*!< GP_T::INTEN: RHIEN1 Position              */
#define GP_INTEN_RHIEN1_Msk              (0x1ul << GP_INTEN_RHIEN1_Pos)                    /*!< GP_T::INTEN: RHIEN1 Mask                  */

#define GP_INTEN_RHIEN2_Pos              (18)                                              /*!< GP_T::INTEN: RHIEN2 Position              */
#define GP_INTEN_RHIEN2_Msk              (0x1ul << GP_INTEN_RHIEN2_Pos)                    /*!< GP_T::INTEN: RHIEN2 Mask                  */

#define GP_INTEN_RHIEN3_Pos              (19)                                              /*!< GP_T::INTEN: RHIEN3 Position              */
#define GP_INTEN_RHIEN3_Msk              (0x1ul << GP_INTEN_RHIEN3_Pos)                    /*!< GP_T::INTEN: RHIEN3 Mask                  */

#define GP_INTEN_RHIEN4_Pos              (20)                                              /*!< GP_T::INTEN: RHIEN4 Position              */
#define GP_INTEN_RHIEN4_Msk              (0x1ul << GP_INTEN_RHIEN4_Pos)                    /*!< GP_T::INTEN: RHIEN4 Mask                  */

#define GP_INTEN_RHIEN5_Pos              (21)                                              /*!< GP_T::INTEN: RHIEN5 Position              */
#define GP_INTEN_RHIEN5_Msk              (0x1ul << GP_INTEN_RHIEN5_Pos)                    /*!< GP_T::INTEN: RHIEN5 Mask                  */

#define GP_INTEN_RHIEN6_Pos              (22)                                              /*!< GP_T::INTEN: RHIEN6 Position              */
#define GP_INTEN_RHIEN6_Msk              (0x1ul << GP_INTEN_RHIEN6_Pos)                    /*!< GP_T::INTEN: RHIEN6 Mask                  */

#define GP_INTEN_RHIEN7_Pos              (23)                                              /*!< GP_T::INTEN: RHIEN7 Position              */
#define GP_INTEN_RHIEN7_Msk              (0x1ul << GP_INTEN_RHIEN7_Pos)                    /*!< GP_T::INTEN: RHIEN7 Mask                  */

#define GP_INTSRC_INTSRC0_Pos            (0)                                               /*!< GP_T::INTSRC: INTSRC0 Position            */
#define GP_INTSRC_INTSRC0_Msk            (0x1ul << GP_INTSRC_INTSRC0_Pos)                  /*!< GP_T::INTSRC: INTSRC0 Mask                */

#define GP_INTSRC_INTSRC1_Pos            (1)                                               /*!< GP_T::INTSRC: INTSRC1 Position            */
#define GP_INTSRC_INTSRC1_Msk            (0x1ul << GP_INTSRC_INTSRC1_Pos)                  /*!< GP_T::INTSRC: INTSRC1 Mask                */

#define GP_INTSRC_INTSRC2_Pos            (2)                                               /*!< GP_T::INTSRC: INTSRC2 Position            */
#define GP_INTSRC_INTSRC2_Msk            (0x1ul << GP_INTSRC_INTSRC2_Pos)                  /*!< GP_T::INTSRC: INTSRC2 Mask                */

#define GP_INTSRC_INTSRC3_Pos            (3)                                               /*!< GP_T::INTSRC: INTSRC3 Position            */
#define GP_INTSRC_INTSRC3_Msk            (0x1ul << GP_INTSRC_INTSRC3_Pos)                  /*!< GP_T::INTSRC: INTSRC3 Mask                */

#define GP_INTSRC_INTSRC4_Pos            (4)                                               /*!< GP_T::INTSRC: INTSRC4 Position            */
#define GP_INTSRC_INTSRC4_Msk            (0x1ul << GP_INTSRC_INTSRC4_Pos)                  /*!< GP_T::INTSRC: INTSRC4 Mask                */

#define GP_INTSRC_INTSRC5_Pos            (5)                                               /*!< GP_T::INTSRC: INTSRC5 Position            */
#define GP_INTSRC_INTSRC5_Msk            (0x1ul << GP_INTSRC_INTSRC5_Pos)                  /*!< GP_T::INTSRC: INTSRC5 Mask                */

#define GP_INTSRC_INTSRC6_Pos            (6)                                               /*!< GP_T::INTSRC: INTSRC6 Position            */
#define GP_INTSRC_INTSRC6_Msk            (0x1ul << GP_INTSRC_INTSRC6_Pos)                  /*!< GP_T::INTSRC: INTSRC6 Mask                */

#define GP_INTSRC_INTSRC7_Pos            (7)                                               /*!< GP_T::INTSRC: INTSRC7 Position            */
#define GP_INTSRC_INTSRC7_Msk            (0x1ul << GP_INTSRC_INTSRC7_Pos)                  /*!< GP_T::INTSRC: INTSRC7 Mask                */

#define GP_INTSRC_INTSRC8_Pos            (8)                                               /*!< GP_T::INTSRC: INTSRC8 Position            */
#define GP_INTSRC_INTSRC8_Msk            (0x1ul << GP_INTSRC_INTSRC8_Pos)                  /*!< GP_T::INTSRC: INTSRC8 Mask                */

#define GP_INTSRC_INTSRC9_Pos            (9)                                               /*!< GP_T::INTSRC: INTSRC9 Position            */
#define GP_INTSRC_INTSRC9_Msk            (0x1ul << GP_INTSRC_INTSRC9_Pos)                  /*!< GP_T::INTSRC: INTSRC9 Mask                */

#define GP_INTSRC_INTSRC10_Pos           (10)                                              /*!< GP_T::INTSRC: INTSRC10 Position           */
#define GP_INTSRC_INTSRC10_Msk           (0x1ul << GP_INTSRC_INTSRC10_Pos)                 /*!< GP_T::INTSRC: INTSRC10 Mask               */

#define GP_INTSRC_INTSRC11_Pos           (11)                                              /*!< GP_T::INTSRC: INTSRC11 Position           */
#define GP_INTSRC_INTSRC11_Msk           (0x1ul << GP_INTSRC_INTSRC11_Pos)                 /*!< GP_T::INTSRC: INTSRC11 Mask               */

#define GP_INTSRC_INTSRC12_Pos           (12)                                              /*!< GP_T::INTSRC: INTSRC12 Position           */
#define GP_INTSRC_INTSRC12_Msk           (0x1ul << GP_INTSRC_INTSRC12_Pos)                 /*!< GP_T::INTSRC: INTSRC12 Mask               */

#define GP_INTSRC_INTSRC13_Pos           (13)                                              /*!< GP_T::INTSRC: INTSRC13 Position           */
#define GP_INTSRC_INTSRC13_Msk           (0x1ul << GP_INTSRC_INTSRC13_Pos)                 /*!< GP_T::INTSRC: INTSRC13 Mask               */

#define GP_INTSRC_INTSRC14_Pos           (14)                                              /*!< GP_T::INTSRC: INTSRC14 Position           */
#define GP_INTSRC_INTSRC14_Msk           (0x1ul << GP_INTSRC_INTSRC14_Pos)                 /*!< GP_T::INTSRC: INTSRC14 Mask               */

#define GP_INTSRC_INTSRC15_Pos           (15)                                              /*!< GP_T::INTSRC: INTSRC15 Position           */
#define GP_INTSRC_INTSRC15_Msk           (0x1ul << GP_INTSRC_INTSRC15_Pos)                 /*!< GP_T::INTSRC: INTSRC15 Mask               */

#define GP_MODE_MODE0_Pos                (0)                                               /*!< GP_T::MODE: MODE0 Position                */
#define GP_MODE_MODE0_Msk                (0x3ul << GP_MODE_MODE0_Pos)                      /*!< GP_T::MODE: MODE0 Mask                    */

#define GP_MODE_MODE1_Pos                (2)                                               /*!< GP_T::MODE: MODE1 Position                */
#define GP_MODE_MODE1_Msk                (0x3ul << GP_MODE_MODE1_Pos)                      /*!< GP_T::MODE: MODE1 Mask                    */

#define GP_MODE_MODE2_Pos                (4)                                               /*!< GP_T::MODE: MODE2 Position                */
#define GP_MODE_MODE2_Msk                (0x3ul << GP_MODE_MODE2_Pos)                      /*!< GP_T::MODE: MODE2 Mask                    */

#define GP_MODE_MODE3_Pos                (6)                                               /*!< GP_T::MODE: MODE3 Position                */
#define GP_MODE_MODE3_Msk                (0x3ul << GP_MODE_MODE3_Pos)                      /*!< GP_T::MODE: MODE3 Mask                    */

#define GP_MODE_MODE4_Pos                (8)                                               /*!< GP_T::MODE: MODE4 Position                */
#define GP_MODE_MODE4_Msk                (0x3ul << GP_MODE_MODE4_Pos)                      /*!< GP_T::MODE: MODE4 Mask                    */

#define GP_MODE_MODE5_Pos                (10)                                              /*!< GP_T::MODE: MODE5 Position                */
#define GP_MODE_MODE5_Msk                (0x3ul << GP_MODE_MODE5_Pos)                      /*!< GP_T::MODE: MODE5 Mask                    */

#define GP_DINOFF_DINOFF0_Pos            (16)                                              /*!< GP_T::DINOFF: DINOFF0 Position            */
#define GP_DINOFF_DINOFF0_Msk            (0x1ul << GP_DINOFF_DINOFF0_Pos)                  /*!< GP_T::DINOFF: DINOFF0 Mask                */

#define GP_DINOFF_DINOFF1_Pos            (17)                                              /*!< GP_T::DINOFF: DINOFF1 Position            */
#define GP_DINOFF_DINOFF1_Msk            (0x1ul << GP_DINOFF_DINOFF1_Pos)                  /*!< GP_T::DINOFF: DINOFF1 Mask                */

#define GP_DINOFF_DINOFF2_Pos            (18)                                              /*!< GP_T::DINOFF: DINOFF2 Position            */
#define GP_DINOFF_DINOFF2_Msk            (0x1ul << GP_DINOFF_DINOFF2_Pos)                  /*!< GP_T::DINOFF: DINOFF2 Mask                */

#define GP_DINOFF_DINOFF3_Pos            (19)                                              /*!< GP_T::DINOFF: DINOFF3 Position            */
#define GP_DINOFF_DINOFF3_Msk            (0x1ul << GP_DINOFF_DINOFF3_Pos)                  /*!< GP_T::DINOFF: DINOFF3 Mask                */

#define GP_DINOFF_DINOFF4_Pos            (20)                                              /*!< GP_T::DINOFF: DINOFF4 Position            */
#define GP_DINOFF_DINOFF4_Msk            (0x1ul << GP_DINOFF_DINOFF4_Pos)                  /*!< GP_T::DINOFF: DINOFF4 Mask                */

#define GP_DINOFF_DINOFF5_Pos            (21)                                              /*!< GP_T::DINOFF: DINOFF5 Position            */
#define GP_DINOFF_DINOFF5_Msk            (0x1ul << GP_DINOFF_DINOFF5_Pos)                  /*!< GP_T::DINOFF: DINOFF5 Mask                */

#define GP_DINOFF_DINOFF6_Pos            (22)                                              /*!< GP_T::DINOFF: DINOFF6 Position            */
#define GP_DINOFF_DINOFF6_Msk            (0x1ul << GP_DINOFF_DINOFF6_Pos)                  /*!< GP_T::DINOFF: DINOFF6 Mask                */

#define GP_DINOFF_DINOFF7_Pos            (23)                                              /*!< GP_T::DINOFF: DINOFF7 Position            */
#define GP_DINOFF_DINOFF7_Msk            (0x1ul << GP_DINOFF_DINOFF7_Pos)                  /*!< GP_T::DINOFF: DINOFF7 Mask                */

#define GP_DOUT_DOUT0_Pos                (0)                                               /*!< GP_T::DOUT: DOUT0 Position                */
#define GP_DOUT_DOUT0_Msk                (0x1ul << GP_DOUT_DOUT0_Pos)                      /*!< GP_T::DOUT: DOUT0 Mask                    */

#define GP_DOUT_DOUT1_Pos                (1)                                               /*!< GP_T::DOUT: DOUT1 Position                */
#define GP_DOUT_DOUT1_Msk                (0x1ul << GP_DOUT_DOUT1_Pos)                      /*!< GP_T::DOUT: DOUT1 Mask                    */

#define GP_DOUT_DOUT2_Pos                (2)                                               /*!< GP_T::DOUT: DOUT2 Position                */
#define GP_DOUT_DOUT2_Msk                (0x1ul << GP_DOUT_DOUT2_Pos)                      /*!< GP_T::DOUT: DOUT2 Mask                    */

#define GP_DOUT_DOUT3_Pos                (3)                                               /*!< GP_T::DOUT: DOUT3 Position                */
#define GP_DOUT_DOUT3_Msk                (0x1ul << GP_DOUT_DOUT3_Pos)                      /*!< GP_T::DOUT: DOUT3 Mask                    */

#define GP_DOUT_DOUT4_Pos                (4)                                               /*!< GP_T::DOUT: DOUT4 Position                */
#define GP_DOUT_DOUT4_Msk                (0x1ul << GP_DOUT_DOUT4_Pos)                      /*!< GP_T::DOUT: DOUT4 Mask                    */

#define GP_DOUT_DOUT5_Pos                (5)                                               /*!< GP_T::DOUT: DOUT5 Position                */
#define GP_DOUT_DOUT5_Msk                (0x1ul << GP_DOUT_DOUT5_Pos)                      /*!< GP_T::DOUT: DOUT5 Mask                    */

#define GP_DOUT_DOUT6_Pos                (6)                                               /*!< GP_T::DOUT: DOUT6 Position                */
#define GP_DOUT_DOUT6_Msk                (0x1ul << GP_DOUT_DOUT6_Pos)                      /*!< GP_T::DOUT: DOUT6 Mask                    */

#define GP_DOUT_DOUT7_Pos                (7)                                               /*!< GP_T::DOUT: DOUT7 Position                */
#define GP_DOUT_DOUT7_Msk                (0x1ul << GP_DOUT_DOUT7_Pos)                      /*!< GP_T::DOUT: DOUT7 Mask                    */

#define GP_DATMSK_DATMSK0_Pos            (0)                                               /*!< GP_T::DATMSK: DATMSK0 Position            */
#define GP_DATMSK_DATMSK0_Msk            (0x1ul << GP_DATMSK_DATMSK0_Pos)                  /*!< GP_T::DATMSK: DATMSK0 Mask                */

#define GP_DATMSK_DATMSK1_Pos            (1)                                               /*!< GP_T::DATMSK: DATMSK1 Position            */
#define GP_DATMSK_DATMSK1_Msk            (0x1ul << GP_DATMSK_DATMSK1_Pos)                  /*!< GP_T::DATMSK: DATMSK1 Mask                */

#define GP_DATMSK_DATMSK2_Pos            (2)                                               /*!< GP_T::DATMSK: DATMSK2 Position            */
#define GP_DATMSK_DATMSK2_Msk            (0x1ul << GP_DATMSK_DATMSK2_Pos)                  /*!< GP_T::DATMSK: DATMSK2 Mask                */

#define GP_DATMSK_DATMSK3_Pos            (3)                                               /*!< GP_T::DATMSK: DATMSK3 Position            */
#define GP_DATMSK_DATMSK3_Msk            (0x1ul << GP_DATMSK_DATMSK3_Pos)                  /*!< GP_T::DATMSK: DATMSK3 Mask                */

#define GP_DATMSK_DATMSK4_Pos            (4)                                               /*!< GP_T::DATMSK: DATMSK4 Position            */
#define GP_DATMSK_DATMSK4_Msk            (0x1ul << GP_DATMSK_DATMSK4_Pos)                  /*!< GP_T::DATMSK: DATMSK4 Mask                */

#define GP_DATMSK_DATMSK5_Pos            (5)                                               /*!< GP_T::DATMSK: DATMSK5 Position            */
#define GP_DATMSK_DATMSK5_Msk            (0x1ul << GP_DATMSK_DATMSK5_Pos)                  /*!< GP_T::DATMSK: DATMSK5 Mask                */

#define GP_DATMSK_DATMSK6_Pos            (6)                                               /*!< GP_T::DATMSK: DATMSK6 Position            */
#define GP_DATMSK_DATMSK6_Msk            (0x1ul << GP_DATMSK_DATMSK6_Pos)                  /*!< GP_T::DATMSK: DATMSK6 Mask                */

#define GP_DATMSK_DATMSK7_Pos            (7)                                               /*!< GP_T::DATMSK: DATMSK7 Position            */
#define GP_DATMSK_DATMSK7_Msk            (0x1ul << GP_DATMSK_DATMSK7_Pos)                  /*!< GP_T::DATMSK: DATMSK7 Mask                */

#define GP_PIN_PIN0_Pos                  (0)                                               /*!< GP_T::PIN: PIN0 Position                  */
#define GP_PIN_PIN0_Msk                  (0x1ul << GP_PIN_PIN0_Pos)                        /*!< GP_T::PIN: PIN0 Mask                      */

#define GP_PIN_PIN1_Pos                  (1)                                               /*!< GP_T::PIN: PIN1 Position                  */
#define GP_PIN_PIN1_Msk                  (0x1ul << GP_PIN_PIN1_Pos)                        /*!< GP_T::PIN: PIN1 Mask                      */

#define GP_PIN_PIN2_Pos                  (2)                                               /*!< GP_T::PIN: PIN2 Position                  */
#define GP_PIN_PIN2_Msk                  (0x1ul << GP_PIN_PIN2_Pos)                        /*!< GP_T::PIN: PIN2 Mask                      */

#define GP_PIN_PIN3_Pos                  (3)                                               /*!< GP_T::PIN: PIN3 Position                  */
#define GP_PIN_PIN3_Msk                  (0x1ul << GP_PIN_PIN3_Pos)                        /*!< GP_T::PIN: PIN3 Mask                      */

#define GP_PIN_PIN4_Pos                  (4)                                               /*!< GP_T::PIN: PIN4 Position                  */
#define GP_PIN_PIN4_Msk                  (0x1ul << GP_PIN_PIN4_Pos)                        /*!< GP_T::PIN: PIN4 Mask                      */

#define GP_PIN_PIN5_Pos                  (5)                                               /*!< GP_T::PIN: PIN5 Position                  */
#define GP_PIN_PIN5_Msk                  (0x1ul << GP_PIN_PIN5_Pos)                        /*!< GP_T::PIN: PIN5 Mask                      */

#define GP_PIN_PIN6_Pos                  (6)                                               /*!< GP_T::PIN: PIN6 Position                  */
#define GP_PIN_PIN6_Msk                  (0x1ul << GP_PIN_PIN6_Pos)                        /*!< GP_T::PIN: PIN6 Mask                      */

#define GP_PIN_PIN7_Pos                  (7)                                               /*!< GP_T::PIN: PIN7 Position                  */
#define GP_PIN_PIN7_Msk                  (0x1ul << GP_PIN_PIN7_Pos)                        /*!< GP_T::PIN: PIN7 Mask                      */

#define GP_DBEN_DBEN0_Pos                (0)                                               /*!< GP_T::DBEN: DBEN0 Position                */
#define GP_DBEN_DBEN0_Msk                (0x1ul << GP_DBEN_DBEN0_Pos)                      /*!< GP_T::DBEN: DBEN0 Mask                    */

#define GP_DBEN_DBEN1_Pos                (1)                                               /*!< GP_T::DBEN: DBEN1 Position                */
#define GP_DBEN_DBEN1_Msk                (0x1ul << GP_DBEN_DBEN1_Pos)                      /*!< GP_T::DBEN: DBEN1 Mask                    */

#define GP_DBEN_DBEN2_Pos                (2)                                               /*!< GP_T::DBEN: DBEN2 Position                */
#define GP_DBEN_DBEN2_Msk                (0x1ul << GP_DBEN_DBEN2_Pos)                      /*!< GP_T::DBEN: DBEN2 Mask                    */

#define GP_DBEN_DBEN3_Pos                (3)                                               /*!< GP_T::DBEN: DBEN3 Position                */
#define GP_DBEN_DBEN3_Msk                (0x1ul << GP_DBEN_DBEN3_Pos)                      /*!< GP_T::DBEN: DBEN3 Mask                    */

#define GP_DBEN_DBEN4_Pos                (4)                                               /*!< GP_T::DBEN: DBEN4 Position                */
#define GP_DBEN_DBEN4_Msk                (0x1ul << GP_DBEN_DBEN4_Pos)                      /*!< GP_T::DBEN: DBEN4 Mask                    */

#define GP_DBEN_DBEN5_Pos                (5)                                               /*!< GP_T::DBEN: DBEN5 Position                */
#define GP_DBEN_DBEN5_Msk                (0x1ul << GP_DBEN_DBEN5_Pos)                      /*!< GP_T::DBEN: DBEN5 Mask                    */

#define GP_DBEN_DBEN6_Pos                (6)                                               /*!< GP_T::DBEN: DBEN6 Position                */
#define GP_DBEN_DBEN6_Msk                (0x1ul << GP_DBEN_DBEN6_Pos)                      /*!< GP_T::DBEN: DBEN6 Mask                    */

#define GP_DBEN_DBEN7_Pos                (7)                                               /*!< GP_T::DBEN: DBEN7 Position                */
#define GP_DBEN_DBEN7_Msk                (0x1ul << GP_DBEN_DBEN7_Pos)                      /*!< GP_T::DBEN: DBEN7 Mask                    */

#define GP_INTTYPE_TYPE0_Pos             (0)                                               /*!< GP_T::INTTYPE: TYPE0 Position             */
#define GP_INTTYPE_TYPE0_Msk             (0x1ul << GP_INTTYPE_TYPE0_Pos)                   /*!< GP_T::INTTYPE: TYPE0 Mask                 */

#define GP_INTTYPE_TYPE1_Pos             (1)                                               /*!< GP_T::INTTYPE: TYPE1 Position             */
#define GP_INTTYPE_TYPE1_Msk             (0x1ul << GP_INTTYPE_TYPE1_Pos)                   /*!< GP_T::INTTYPE: TYPE1 Mask                 */

#define GP_INTTYPE_TYPE2_Pos             (2)                                               /*!< GP_T::INTTYPE: TYPE2 Position             */
#define GP_INTTYPE_TYPE2_Msk             (0x1ul << GP_INTTYPE_TYPE2_Pos)                   /*!< GP_T::INTTYPE: TYPE2 Mask                 */

#define GP_INTTYPE_TYPE3_Pos             (3)                                               /*!< GP_T::INTTYPE: TYPE3 Position             */
#define GP_INTTYPE_TYPE3_Msk             (0x1ul << GP_INTTYPE_TYPE3_Pos)                   /*!< GP_T::INTTYPE: TYPE3 Mask                 */

#define GP_INTTYPE_TYPE4_Pos             (4)                                               /*!< GP_T::INTTYPE: TYPE4 Position             */
#define GP_INTTYPE_TYPE4_Msk             (0x1ul << GP_INTTYPE_TYPE4_Pos)                   /*!< GP_T::INTTYPE: TYPE4 Mask                 */

#define GP_INTTYPE_TYPE5_Pos             (5)                                               /*!< GP_T::INTTYPE: TYPE5 Position             */
#define GP_INTTYPE_TYPE5_Msk             (0x1ul << GP_INTTYPE_TYPE5_Pos)                   /*!< GP_T::INTTYPE: TYPE5 Mask                 */

#define GP_INTTYPE_TYPE6_Pos             (6)                                               /*!< GP_T::INTTYPE: TYPE6 Position             */
#define GP_INTTYPE_TYPE6_Msk             (0x1ul << GP_INTTYPE_TYPE6_Pos)                   /*!< GP_T::INTTYPE: TYPE6 Mask                 */

#define GP_INTTYPE_TYPE7_Pos             (7)                                               /*!< GP_T::INTTYPE: TYPE7 Position             */
#define GP_INTTYPE_TYPE7_Msk             (0x1ul << GP_INTTYPE_TYPE7_Pos)                   /*!< GP_T::INTTYPE: TYPE7 Mask                 */

#define GP_INTEN_FLIEN0_Pos              (0)                                               /*!< GP_T::INTEN: FLIEN0 Position              */
#define GP_INTEN_FLIEN0_Msk              (0x1ul << GP_INTEN_FLIEN0_Pos)                    /*!< GP_T::INTEN: FLIEN0 Mask                  */

#define GP_INTEN_FLIEN1_Pos              (1)                                               /*!< GP_T::INTEN: FLIEN1 Position              */
#define GP_INTEN_FLIEN1_Msk              (0x1ul << GP_INTEN_FLIEN1_Pos)                    /*!< GP_T::INTEN: FLIEN1 Mask                  */

#define GP_INTEN_FLIEN2_Pos              (2)                                               /*!< GP_T::INTEN: FLIEN2 Position              */
#define GP_INTEN_FLIEN2_Msk              (0x1ul << GP_INTEN_FLIEN2_Pos)                    /*!< GP_T::INTEN: FLIEN2 Mask                  */

#define GP_INTEN_FLIEN3_Pos              (3)                                               /*!< GP_T::INTEN: FLIEN3 Position              */
#define GP_INTEN_FLIEN3_Msk              (0x1ul << GP_INTEN_FLIEN3_Pos)                    /*!< GP_T::INTEN: FLIEN3 Mask                  */

#define GP_INTEN_FLIEN4_Pos              (4)                                               /*!< GP_T::INTEN: FLIEN4 Position              */
#define GP_INTEN_FLIEN4_Msk              (0x1ul << GP_INTEN_FLIEN4_Pos)                    /*!< GP_T::INTEN: FLIEN4 Mask                  */

#define GP_INTEN_FLIEN5_Pos              (5)                                               /*!< GP_T::INTEN: FLIEN5 Position              */
#define GP_INTEN_FLIEN5_Msk              (0x1ul << GP_INTEN_FLIEN5_Pos)                    /*!< GP_T::INTEN: FLIEN5 Mask                  */

#define GP_INTEN_FLIEN6_Pos              (6)                                               /*!< GP_T::INTEN: FLIEN6 Position              */
#define GP_INTEN_FLIEN6_Msk              (0x1ul << GP_INTEN_FLIEN6_Pos)                    /*!< GP_T::INTEN: FLIEN6 Mask                  */

#define GP_INTEN_FLIEN7_Pos              (7)                                               /*!< GP_T::INTEN: FLIEN7 Position              */
#define GP_INTEN_FLIEN7_Msk              (0x1ul << GP_INTEN_FLIEN7_Pos)                    /*!< GP_T::INTEN: FLIEN7 Mask                  */

#define GP_INTEN_RHIEN0_Pos              (16)                                              /*!< GP_T::INTEN: RHIEN0 Position              */
#define GP_INTEN_RHIEN0_Msk              (0x1ul << GP_INTEN_RHIEN0_Pos)                    /*!< GP_T::INTEN: RHIEN0 Mask                  */

#define GP_INTEN_RHIEN1_Pos              (17)                                              /*!< GP_T::INTEN: RHIEN1 Position              */
#define GP_INTEN_RHIEN1_Msk              (0x1ul << GP_INTEN_RHIEN1_Pos)                    /*!< GP_T::INTEN: RHIEN1 Mask                  */

#define GP_INTEN_RHIEN2_Pos              (18)                                              /*!< GP_T::INTEN: RHIEN2 Position              */
#define GP_INTEN_RHIEN2_Msk              (0x1ul << GP_INTEN_RHIEN2_Pos)                    /*!< GP_T::INTEN: RHIEN2 Mask                  */

#define GP_INTEN_RHIEN3_Pos              (19)                                              /*!< GP_T::INTEN: RHIEN3 Position              */
#define GP_INTEN_RHIEN3_Msk              (0x1ul << GP_INTEN_RHIEN3_Pos)                    /*!< GP_T::INTEN: RHIEN3 Mask                  */

#define GP_INTEN_RHIEN4_Pos              (20)                                              /*!< GP_T::INTEN: RHIEN4 Position              */
#define GP_INTEN_RHIEN4_Msk              (0x1ul << GP_INTEN_RHIEN4_Pos)                    /*!< GP_T::INTEN: RHIEN4 Mask                  */

#define GP_INTEN_RHIEN5_Pos              (21)                                              /*!< GP_T::INTEN: RHIEN5 Position              */
#define GP_INTEN_RHIEN5_Msk              (0x1ul << GP_INTEN_RHIEN5_Pos)                    /*!< GP_T::INTEN: RHIEN5 Mask                  */

#define GP_INTEN_RHIEN6_Pos              (22)                                              /*!< GP_T::INTEN: RHIEN6 Position              */
#define GP_INTEN_RHIEN6_Msk              (0x1ul << GP_INTEN_RHIEN6_Pos)                    /*!< GP_T::INTEN: RHIEN6 Mask                  */

#define GP_INTEN_RHIEN7_Pos              (23)                                              /*!< GP_T::INTEN: RHIEN7 Position              */
#define GP_INTEN_RHIEN7_Msk              (0x1ul << GP_INTEN_RHIEN7_Pos)                    /*!< GP_T::INTEN: RHIEN7 Mask                  */

#define GP_INTSRC_INTSRC0_Pos            (0)                                               /*!< GP_T::INTSRC: INTSRC0 Position            */
#define GP_INTSRC_INTSRC0_Msk            (0x1ul << GP_INTSRC_INTSRC0_Pos)                  /*!< GP_T::INTSRC: INTSRC0 Mask                */

#define GP_INTSRC_INTSRC1_Pos            (1)                                               /*!< GP_T::INTSRC: INTSRC1 Position            */
#define GP_INTSRC_INTSRC1_Msk            (0x1ul << GP_INTSRC_INTSRC1_Pos)                  /*!< GP_T::INTSRC: INTSRC1 Mask                */

#define GP_INTSRC_INTSRC2_Pos            (2)                                               /*!< GP_T::INTSRC: INTSRC2 Position            */
#define GP_INTSRC_INTSRC2_Msk            (0x1ul << GP_INTSRC_INTSRC2_Pos)                  /*!< GP_T::INTSRC: INTSRC2 Mask                */

#define GP_INTSRC_INTSRC3_Pos            (3)                                               /*!< GP_T::INTSRC: INTSRC3 Position            */
#define GP_INTSRC_INTSRC3_Msk            (0x1ul << GP_INTSRC_INTSRC3_Pos)                  /*!< GP_T::INTSRC: INTSRC3 Mask                */

#define GP_INTSRC_INTSRC4_Pos            (4)                                               /*!< GP_T::INTSRC: INTSRC4 Position            */
#define GP_INTSRC_INTSRC4_Msk            (0x1ul << GP_INTSRC_INTSRC4_Pos)                  /*!< GP_T::INTSRC: INTSRC4 Mask                */

#define GP_INTSRC_INTSRC5_Pos            (5)                                               /*!< GP_T::INTSRC: INTSRC5 Position            */
#define GP_INTSRC_INTSRC5_Msk            (0x1ul << GP_INTSRC_INTSRC5_Pos)                  /*!< GP_T::INTSRC: INTSRC5 Mask                */

#define GP_INTSRC_INTSRC6_Pos            (6)                                               /*!< GP_T::INTSRC: INTSRC6 Position            */
#define GP_INTSRC_INTSRC6_Msk            (0x1ul << GP_INTSRC_INTSRC6_Pos)                  /*!< GP_T::INTSRC: INTSRC6 Mask                */

#define GP_INTSRC_INTSRC7_Pos            (7)                                               /*!< GP_T::INTSRC: INTSRC7 Position            */
#define GP_INTSRC_INTSRC7_Msk            (0x1ul << GP_INTSRC_INTSRC7_Pos)                  /*!< GP_T::INTSRC: INTSRC7 Mask                */

#define GP_INTSRC_INTSRC8_Pos            (8)                                               /*!< GP_T::INTSRC: INTSRC8 Position            */
#define GP_INTSRC_INTSRC8_Msk            (0x1ul << GP_INTSRC_INTSRC8_Pos)                  /*!< GP_T::INTSRC: INTSRC8 Mask                */

#define GP_INTSRC_INTSRC9_Pos            (9)                                               /*!< GP_T::INTSRC: INTSRC9 Position            */
#define GP_INTSRC_INTSRC9_Msk            (0x1ul << GP_INTSRC_INTSRC9_Pos)                  /*!< GP_T::INTSRC: INTSRC9 Mask                */

#define GP_INTSRC_INTSRC10_Pos           (10)                                              /*!< GP_T::INTSRC: INTSRC10 Position           */
#define GP_INTSRC_INTSRC10_Msk           (0x1ul << GP_INTSRC_INTSRC10_Pos)                 /*!< GP_T::INTSRC: INTSRC10 Mask               */

#define GP_INTSRC_INTSRC11_Pos           (11)                                              /*!< GP_T::INTSRC: INTSRC11 Position           */
#define GP_INTSRC_INTSRC11_Msk           (0x1ul << GP_INTSRC_INTSRC11_Pos)                 /*!< GP_T::INTSRC: INTSRC11 Mask               */

#define GP_INTSRC_INTSRC12_Pos           (12)                                              /*!< GP_T::INTSRC: INTSRC12 Position           */
#define GP_INTSRC_INTSRC12_Msk           (0x1ul << GP_INTSRC_INTSRC12_Pos)                 /*!< GP_T::INTSRC: INTSRC12 Mask               */

#define GP_INTSRC_INTSRC13_Pos           (13)                                              /*!< GP_T::INTSRC: INTSRC13 Position           */
#define GP_INTSRC_INTSRC13_Msk           (0x1ul << GP_INTSRC_INTSRC13_Pos)                 /*!< GP_T::INTSRC: INTSRC13 Mask               */

#define GP_INTSRC_INTSRC14_Pos           (14)                                              /*!< GP_T::INTSRC: INTSRC14 Position           */
#define GP_INTSRC_INTSRC14_Msk           (0x1ul << GP_INTSRC_INTSRC14_Pos)                 /*!< GP_T::INTSRC: INTSRC14 Mask               */

#define GP_INTSRC_INTSRC15_Pos           (15)                                              /*!< GP_T::INTSRC: INTSRC15 Position           */
#define GP_INTSRC_INTSRC15_Msk           (0x1ul << GP_INTSRC_INTSRC15_Pos)                 /*!< GP_T::INTSRC: INTSRC15 Mask               */

#define GP_MODE_MODE0_Pos                (0)                                               /*!< GP_T::MODE: MODE0 Position                */
#define GP_MODE_MODE0_Msk                (0x3ul << GP_MODE_MODE0_Pos)                      /*!< GP_T::MODE: MODE0 Mask                    */

#define GP_MODE_MODE1_Pos                (2)                                               /*!< GP_T::MODE: MODE1 Position                */
#define GP_MODE_MODE1_Msk                (0x3ul << GP_MODE_MODE1_Pos)                      /*!< GP_T::MODE: MODE1 Mask                    */

#define GP_MODE_MODE2_Pos                (4)                                               /*!< GP_T::MODE: MODE2 Position                */
#define GP_MODE_MODE2_Msk                (0x3ul << GP_MODE_MODE2_Pos)                      /*!< GP_T::MODE: MODE2 Mask                    */

#define GP_MODE_MODE3_Pos                (6)                                               /*!< GP_T::MODE: MODE3 Position                */
#define GP_MODE_MODE3_Msk                (0x3ul << GP_MODE_MODE3_Pos)                      /*!< GP_T::MODE: MODE3 Mask                    */

#define GP_MODE_MODE4_Pos                (8)                                               /*!< GP_T::MODE: MODE4 Position                */
#define GP_MODE_MODE4_Msk                (0x3ul << GP_MODE_MODE4_Pos)                      /*!< GP_T::MODE: MODE4 Mask                    */

#define GP_MODE_MODE5_Pos                (10)                                              /*!< GP_T::MODE: MODE5 Position                */
#define GP_MODE_MODE5_Msk                (0x3ul << GP_MODE_MODE5_Pos)                      /*!< GP_T::MODE: MODE5 Mask                    */

#define GP_DINOFF_DINOFF0_Pos            (16)                                              /*!< GP_T::DINOFF: DINOFF0 Position            */
#define GP_DINOFF_DINOFF0_Msk            (0x1ul << GP_DINOFF_DINOFF0_Pos)                  /*!< GP_T::DINOFF: DINOFF0 Mask                */

#define GP_DINOFF_DINOFF1_Pos            (17)                                              /*!< GP_T::DINOFF: DINOFF1 Position            */
#define GP_DINOFF_DINOFF1_Msk            (0x1ul << GP_DINOFF_DINOFF1_Pos)                  /*!< GP_T::DINOFF: DINOFF1 Mask                */

#define GP_DINOFF_DINOFF2_Pos            (18)                                              /*!< GP_T::DINOFF: DINOFF2 Position            */
#define GP_DINOFF_DINOFF2_Msk            (0x1ul << GP_DINOFF_DINOFF2_Pos)                  /*!< GP_T::DINOFF: DINOFF2 Mask                */

#define GP_DINOFF_DINOFF3_Pos            (19)                                              /*!< GP_T::DINOFF: DINOFF3 Position            */
#define GP_DINOFF_DINOFF3_Msk            (0x1ul << GP_DINOFF_DINOFF3_Pos)                  /*!< GP_T::DINOFF: DINOFF3 Mask                */

#define GP_DINOFF_DINOFF4_Pos            (20)                                              /*!< GP_T::DINOFF: DINOFF4 Position            */
#define GP_DINOFF_DINOFF4_Msk            (0x1ul << GP_DINOFF_DINOFF4_Pos)                  /*!< GP_T::DINOFF: DINOFF4 Mask                */

#define GP_DINOFF_DINOFF5_Pos            (21)                                              /*!< GP_T::DINOFF: DINOFF5 Position            */
#define GP_DINOFF_DINOFF5_Msk            (0x1ul << GP_DINOFF_DINOFF5_Pos)                  /*!< GP_T::DINOFF: DINOFF5 Mask                */

#define GP_DINOFF_DINOFF6_Pos            (22)                                              /*!< GP_T::DINOFF: DINOFF6 Position            */
#define GP_DINOFF_DINOFF6_Msk            (0x1ul << GP_DINOFF_DINOFF6_Pos)                  /*!< GP_T::DINOFF: DINOFF6 Mask                */

#define GP_DINOFF_DINOFF7_Pos            (23)                                              /*!< GP_T::DINOFF: DINOFF7 Position            */
#define GP_DINOFF_DINOFF7_Msk            (0x1ul << GP_DINOFF_DINOFF7_Pos)                  /*!< GP_T::DINOFF: DINOFF7 Mask                */

#define GP_DOUT_DOUT0_Pos                (0)                                               /*!< GP_T::DOUT: DOUT0 Position                */
#define GP_DOUT_DOUT0_Msk                (0x1ul << GP_DOUT_DOUT0_Pos)                      /*!< GP_T::DOUT: DOUT0 Mask                    */

#define GP_DOUT_DOUT1_Pos                (1)                                               /*!< GP_T::DOUT: DOUT1 Position                */
#define GP_DOUT_DOUT1_Msk                (0x1ul << GP_DOUT_DOUT1_Pos)                      /*!< GP_T::DOUT: DOUT1 Mask                    */

#define GP_DOUT_DOUT2_Pos                (2)                                               /*!< GP_T::DOUT: DOUT2 Position                */
#define GP_DOUT_DOUT2_Msk                (0x1ul << GP_DOUT_DOUT2_Pos)                      /*!< GP_T::DOUT: DOUT2 Mask                    */

#define GP_DOUT_DOUT3_Pos                (3)                                               /*!< GP_T::DOUT: DOUT3 Position                */
#define GP_DOUT_DOUT3_Msk                (0x1ul << GP_DOUT_DOUT3_Pos)                      /*!< GP_T::DOUT: DOUT3 Mask                    */

#define GP_DOUT_DOUT4_Pos                (4)                                               /*!< GP_T::DOUT: DOUT4 Position                */
#define GP_DOUT_DOUT4_Msk                (0x1ul << GP_DOUT_DOUT4_Pos)                      /*!< GP_T::DOUT: DOUT4 Mask                    */

#define GP_DOUT_DOUT5_Pos                (5)                                               /*!< GP_T::DOUT: DOUT5 Position                */
#define GP_DOUT_DOUT5_Msk                (0x1ul << GP_DOUT_DOUT5_Pos)                      /*!< GP_T::DOUT: DOUT5 Mask                    */

#define GP_DOUT_DOUT6_Pos                (6)                                               /*!< GP_T::DOUT: DOUT6 Position                */
#define GP_DOUT_DOUT6_Msk                (0x1ul << GP_DOUT_DOUT6_Pos)                      /*!< GP_T::DOUT: DOUT6 Mask                    */

#define GP_DOUT_DOUT7_Pos                (7)                                               /*!< GP_T::DOUT: DOUT7 Position                */
#define GP_DOUT_DOUT7_Msk                (0x1ul << GP_DOUT_DOUT7_Pos)                      /*!< GP_T::DOUT: DOUT7 Mask                    */

#define GP_DATMSK_DATMSK0_Pos            (0)                                               /*!< GP_T::DATMSK: DATMSK0 Position            */
#define GP_DATMSK_DATMSK0_Msk            (0x1ul << GP_DATMSK_DATMSK0_Pos)                  /*!< GP_T::DATMSK: DATMSK0 Mask                */

#define GP_DATMSK_DATMSK1_Pos            (1)                                               /*!< GP_T::DATMSK: DATMSK1 Position            */
#define GP_DATMSK_DATMSK1_Msk            (0x1ul << GP_DATMSK_DATMSK1_Pos)                  /*!< GP_T::DATMSK: DATMSK1 Mask                */

#define GP_DATMSK_DATMSK2_Pos            (2)                                               /*!< GP_T::DATMSK: DATMSK2 Position            */
#define GP_DATMSK_DATMSK2_Msk            (0x1ul << GP_DATMSK_DATMSK2_Pos)                  /*!< GP_T::DATMSK: DATMSK2 Mask                */

#define GP_DATMSK_DATMSK3_Pos            (3)                                               /*!< GP_T::DATMSK: DATMSK3 Position            */
#define GP_DATMSK_DATMSK3_Msk            (0x1ul << GP_DATMSK_DATMSK3_Pos)                  /*!< GP_T::DATMSK: DATMSK3 Mask                */

#define GP_DATMSK_DATMSK4_Pos            (4)                                               /*!< GP_T::DATMSK: DATMSK4 Position            */
#define GP_DATMSK_DATMSK4_Msk            (0x1ul << GP_DATMSK_DATMSK4_Pos)                  /*!< GP_T::DATMSK: DATMSK4 Mask                */

#define GP_DATMSK_DATMSK5_Pos            (5)                                               /*!< GP_T::DATMSK: DATMSK5 Position            */
#define GP_DATMSK_DATMSK5_Msk            (0x1ul << GP_DATMSK_DATMSK5_Pos)                  /*!< GP_T::DATMSK: DATMSK5 Mask                */

#define GP_DATMSK_DATMSK6_Pos            (6)                                               /*!< GP_T::DATMSK: DATMSK6 Position            */
#define GP_DATMSK_DATMSK6_Msk            (0x1ul << GP_DATMSK_DATMSK6_Pos)                  /*!< GP_T::DATMSK: DATMSK6 Mask                */

#define GP_DATMSK_DATMSK7_Pos            (7)                                               /*!< GP_T::DATMSK: DATMSK7 Position            */
#define GP_DATMSK_DATMSK7_Msk            (0x1ul << GP_DATMSK_DATMSK7_Pos)                  /*!< GP_T::DATMSK: DATMSK7 Mask                */

#define GP_PIN_PIN0_Pos                  (0)                                               /*!< GP_T::PIN: PIN0 Position                  */
#define GP_PIN_PIN0_Msk                  (0x1ul << GP_PIN_PIN0_Pos)                        /*!< GP_T::PIN: PIN0 Mask                      */

#define GP_PIN_PIN1_Pos                  (1)                                               /*!< GP_T::PIN: PIN1 Position                  */
#define GP_PIN_PIN1_Msk                  (0x1ul << GP_PIN_PIN1_Pos)                        /*!< GP_T::PIN: PIN1 Mask                      */

#define GP_PIN_PIN2_Pos                  (2)                                               /*!< GP_T::PIN: PIN2 Position                  */
#define GP_PIN_PIN2_Msk                  (0x1ul << GP_PIN_PIN2_Pos)                        /*!< GP_T::PIN: PIN2 Mask                      */

#define GP_PIN_PIN3_Pos                  (3)                                               /*!< GP_T::PIN: PIN3 Position                  */
#define GP_PIN_PIN3_Msk                  (0x1ul << GP_PIN_PIN3_Pos)                        /*!< GP_T::PIN: PIN3 Mask                      */

#define GP_PIN_PIN4_Pos                  (4)                                               /*!< GP_T::PIN: PIN4 Position                  */
#define GP_PIN_PIN4_Msk                  (0x1ul << GP_PIN_PIN4_Pos)                        /*!< GP_T::PIN: PIN4 Mask                      */

#define GP_PIN_PIN5_Pos                  (5)                                               /*!< GP_T::PIN: PIN5 Position                  */
#define GP_PIN_PIN5_Msk                  (0x1ul << GP_PIN_PIN5_Pos)                        /*!< GP_T::PIN: PIN5 Mask                      */

#define GP_PIN_PIN6_Pos                  (6)                                               /*!< GP_T::PIN: PIN6 Position                  */
#define GP_PIN_PIN6_Msk                  (0x1ul << GP_PIN_PIN6_Pos)                        /*!< GP_T::PIN: PIN6 Mask                      */

#define GP_PIN_PIN7_Pos                  (7)                                               /*!< GP_T::PIN: PIN7 Position                  */
#define GP_PIN_PIN7_Msk                  (0x1ul << GP_PIN_PIN7_Pos)                        /*!< GP_T::PIN: PIN7 Mask                      */

#define GP_DBEN_DBEN0_Pos                (0)                                               /*!< GP_T::DBEN: DBEN0 Position                */
#define GP_DBEN_DBEN0_Msk                (0x1ul << GP_DBEN_DBEN0_Pos)                      /*!< GP_T::DBEN: DBEN0 Mask                    */

#define GP_DBEN_DBEN1_Pos                (1)                                               /*!< GP_T::DBEN: DBEN1 Position                */
#define GP_DBEN_DBEN1_Msk                (0x1ul << GP_DBEN_DBEN1_Pos)                      /*!< GP_T::DBEN: DBEN1 Mask                    */

#define GP_DBEN_DBEN2_Pos                (2)                                               /*!< GP_T::DBEN: DBEN2 Position                */
#define GP_DBEN_DBEN2_Msk                (0x1ul << GP_DBEN_DBEN2_Pos)                      /*!< GP_T::DBEN: DBEN2 Mask                    */

#define GP_DBEN_DBEN3_Pos                (3)                                               /*!< GP_T::DBEN: DBEN3 Position                */
#define GP_DBEN_DBEN3_Msk                (0x1ul << GP_DBEN_DBEN3_Pos)                      /*!< GP_T::DBEN: DBEN3 Mask                    */

#define GP_DBEN_DBEN4_Pos                (4)                                               /*!< GP_T::DBEN: DBEN4 Position                */
#define GP_DBEN_DBEN4_Msk                (0x1ul << GP_DBEN_DBEN4_Pos)                      /*!< GP_T::DBEN: DBEN4 Mask                    */

#define GP_DBEN_DBEN5_Pos                (5)                                               /*!< GP_T::DBEN: DBEN5 Position                */
#define GP_DBEN_DBEN5_Msk                (0x1ul << GP_DBEN_DBEN5_Pos)                      /*!< GP_T::DBEN: DBEN5 Mask                    */

#define GP_DBEN_DBEN6_Pos                (6)                                               /*!< GP_T::DBEN: DBEN6 Position                */
#define GP_DBEN_DBEN6_Msk                (0x1ul << GP_DBEN_DBEN6_Pos)                      /*!< GP_T::DBEN: DBEN6 Mask                    */

#define GP_DBEN_DBEN7_Pos                (7)                                               /*!< GP_T::DBEN: DBEN7 Position                */
#define GP_DBEN_DBEN7_Msk                (0x1ul << GP_DBEN_DBEN7_Pos)                      /*!< GP_T::DBEN: DBEN7 Mask                    */

#define GP_INTTYPE_TYPE0_Pos             (0)                                               /*!< GP_T::INTTYPE: TYPE0 Position             */
#define GP_INTTYPE_TYPE0_Msk             (0x1ul << GP_INTTYPE_TYPE0_Pos)                   /*!< GP_T::INTTYPE: TYPE0 Mask                 */

#define GP_INTTYPE_TYPE1_Pos             (1)                                               /*!< GP_T::INTTYPE: TYPE1 Position             */
#define GP_INTTYPE_TYPE1_Msk             (0x1ul << GP_INTTYPE_TYPE1_Pos)                   /*!< GP_T::INTTYPE: TYPE1 Mask                 */

#define GP_INTTYPE_TYPE2_Pos             (2)                                               /*!< GP_T::INTTYPE: TYPE2 Position             */
#define GP_INTTYPE_TYPE2_Msk             (0x1ul << GP_INTTYPE_TYPE2_Pos)                   /*!< GP_T::INTTYPE: TYPE2 Mask                 */

#define GP_INTTYPE_TYPE3_Pos             (3)                                               /*!< GP_T::INTTYPE: TYPE3 Position             */
#define GP_INTTYPE_TYPE3_Msk             (0x1ul << GP_INTTYPE_TYPE3_Pos)                   /*!< GP_T::INTTYPE: TYPE3 Mask                 */

#define GP_INTTYPE_TYPE4_Pos             (4)                                               /*!< GP_T::INTTYPE: TYPE4 Position             */
#define GP_INTTYPE_TYPE4_Msk             (0x1ul << GP_INTTYPE_TYPE4_Pos)                   /*!< GP_T::INTTYPE: TYPE4 Mask                 */

#define GP_INTTYPE_TYPE5_Pos             (5)                                               /*!< GP_T::INTTYPE: TYPE5 Position             */
#define GP_INTTYPE_TYPE5_Msk             (0x1ul << GP_INTTYPE_TYPE5_Pos)                   /*!< GP_T::INTTYPE: TYPE5 Mask                 */

#define GP_INTTYPE_TYPE6_Pos             (6)                                               /*!< GP_T::INTTYPE: TYPE6 Position             */
#define GP_INTTYPE_TYPE6_Msk             (0x1ul << GP_INTTYPE_TYPE6_Pos)                   /*!< GP_T::INTTYPE: TYPE6 Mask                 */

#define GP_INTTYPE_TYPE7_Pos             (7)                                               /*!< GP_T::INTTYPE: TYPE7 Position             */
#define GP_INTTYPE_TYPE7_Msk             (0x1ul << GP_INTTYPE_TYPE7_Pos)                   /*!< GP_T::INTTYPE: TYPE7 Mask                 */

#define GP_INTEN_FLIEN0_Pos              (0)                                               /*!< GP_T::INTEN: FLIEN0 Position              */
#define GP_INTEN_FLIEN0_Msk              (0x1ul << GP_INTEN_FLIEN0_Pos)                    /*!< GP_T::INTEN: FLIEN0 Mask                  */

#define GP_INTEN_FLIEN1_Pos              (1)                                               /*!< GP_T::INTEN: FLIEN1 Position              */
#define GP_INTEN_FLIEN1_Msk              (0x1ul << GP_INTEN_FLIEN1_Pos)                    /*!< GP_T::INTEN: FLIEN1 Mask                  */

#define GP_INTEN_FLIEN2_Pos              (2)                                               /*!< GP_T::INTEN: FLIEN2 Position              */
#define GP_INTEN_FLIEN2_Msk              (0x1ul << GP_INTEN_FLIEN2_Pos)                    /*!< GP_T::INTEN: FLIEN2 Mask                  */

#define GP_INTEN_FLIEN3_Pos              (3)                                               /*!< GP_T::INTEN: FLIEN3 Position              */
#define GP_INTEN_FLIEN3_Msk              (0x1ul << GP_INTEN_FLIEN3_Pos)                    /*!< GP_T::INTEN: FLIEN3 Mask                  */

#define GP_INTEN_FLIEN4_Pos              (4)                                               /*!< GP_T::INTEN: FLIEN4 Position              */
#define GP_INTEN_FLIEN4_Msk              (0x1ul << GP_INTEN_FLIEN4_Pos)                    /*!< GP_T::INTEN: FLIEN4 Mask                  */

#define GP_INTEN_FLIEN5_Pos              (5)                                               /*!< GP_T::INTEN: FLIEN5 Position              */
#define GP_INTEN_FLIEN5_Msk              (0x1ul << GP_INTEN_FLIEN5_Pos)                    /*!< GP_T::INTEN: FLIEN5 Mask                  */

#define GP_INTEN_FLIEN6_Pos              (6)                                               /*!< GP_T::INTEN: FLIEN6 Position              */
#define GP_INTEN_FLIEN6_Msk              (0x1ul << GP_INTEN_FLIEN6_Pos)                    /*!< GP_T::INTEN: FLIEN6 Mask                  */

#define GP_INTEN_FLIEN7_Pos              (7)                                               /*!< GP_T::INTEN: FLIEN7 Position              */
#define GP_INTEN_FLIEN7_Msk              (0x1ul << GP_INTEN_FLIEN7_Pos)                    /*!< GP_T::INTEN: FLIEN7 Mask                  */

#define GP_INTEN_RHIEN0_Pos              (16)                                              /*!< GP_T::INTEN: RHIEN0 Position              */
#define GP_INTEN_RHIEN0_Msk              (0x1ul << GP_INTEN_RHIEN0_Pos)                    /*!< GP_T::INTEN: RHIEN0 Mask                  */

#define GP_INTEN_RHIEN1_Pos              (17)                                              /*!< GP_T::INTEN: RHIEN1 Position              */
#define GP_INTEN_RHIEN1_Msk              (0x1ul << GP_INTEN_RHIEN1_Pos)                    /*!< GP_T::INTEN: RHIEN1 Mask                  */

#define GP_INTEN_RHIEN2_Pos              (18)                                              /*!< GP_T::INTEN: RHIEN2 Position              */
#define GP_INTEN_RHIEN2_Msk              (0x1ul << GP_INTEN_RHIEN2_Pos)                    /*!< GP_T::INTEN: RHIEN2 Mask                  */

#define GP_INTEN_RHIEN3_Pos              (19)                                              /*!< GP_T::INTEN: RHIEN3 Position              */
#define GP_INTEN_RHIEN3_Msk              (0x1ul << GP_INTEN_RHIEN3_Pos)                    /*!< GP_T::INTEN: RHIEN3 Mask                  */

#define GP_INTEN_RHIEN4_Pos              (20)                                              /*!< GP_T::INTEN: RHIEN4 Position              */
#define GP_INTEN_RHIEN4_Msk              (0x1ul << GP_INTEN_RHIEN4_Pos)                    /*!< GP_T::INTEN: RHIEN4 Mask                  */

#define GP_INTEN_RHIEN5_Pos              (21)                                              /*!< GP_T::INTEN: RHIEN5 Position              */
#define GP_INTEN_RHIEN5_Msk              (0x1ul << GP_INTEN_RHIEN5_Pos)                    /*!< GP_T::INTEN: RHIEN5 Mask                  */

#define GP_INTEN_RHIEN6_Pos              (22)                                              /*!< GP_T::INTEN: RHIEN6 Position              */
#define GP_INTEN_RHIEN6_Msk              (0x1ul << GP_INTEN_RHIEN6_Pos)                    /*!< GP_T::INTEN: RHIEN6 Mask                  */

#define GP_INTEN_RHIEN7_Pos              (23)                                              /*!< GP_T::INTEN: RHIEN7 Position              */
#define GP_INTEN_RHIEN7_Msk              (0x1ul << GP_INTEN_RHIEN7_Pos)                    /*!< GP_T::INTEN: RHIEN7 Mask                  */

#define GP_INTSRC_INTSRC0_Pos            (0)                                               /*!< GP_T::INTSRC: INTSRC0 Position            */
#define GP_INTSRC_INTSRC0_Msk            (0x1ul << GP_INTSRC_INTSRC0_Pos)                  /*!< GP_T::INTSRC: INTSRC0 Mask                */

#define GP_INTSRC_INTSRC1_Pos            (1)                                               /*!< GP_T::INTSRC: INTSRC1 Position            */
#define GP_INTSRC_INTSRC1_Msk            (0x1ul << GP_INTSRC_INTSRC1_Pos)                  /*!< GP_T::INTSRC: INTSRC1 Mask                */

#define GP_INTSRC_INTSRC2_Pos            (2)                                               /*!< GP_T::INTSRC: INTSRC2 Position            */
#define GP_INTSRC_INTSRC2_Msk            (0x1ul << GP_INTSRC_INTSRC2_Pos)                  /*!< GP_T::INTSRC: INTSRC2 Mask                */

#define GP_INTSRC_INTSRC3_Pos            (3)                                               /*!< GP_T::INTSRC: INTSRC3 Position            */
#define GP_INTSRC_INTSRC3_Msk            (0x1ul << GP_INTSRC_INTSRC3_Pos)                  /*!< GP_T::INTSRC: INTSRC3 Mask                */

#define GP_INTSRC_INTSRC4_Pos            (4)                                               /*!< GP_T::INTSRC: INTSRC4 Position            */
#define GP_INTSRC_INTSRC4_Msk            (0x1ul << GP_INTSRC_INTSRC4_Pos)                  /*!< GP_T::INTSRC: INTSRC4 Mask                */

#define GP_INTSRC_INTSRC5_Pos            (5)                                               /*!< GP_T::INTSRC: INTSRC5 Position            */
#define GP_INTSRC_INTSRC5_Msk            (0x1ul << GP_INTSRC_INTSRC5_Pos)                  /*!< GP_T::INTSRC: INTSRC5 Mask                */

#define GP_INTSRC_INTSRC6_Pos            (6)                                               /*!< GP_T::INTSRC: INTSRC6 Position            */
#define GP_INTSRC_INTSRC6_Msk            (0x1ul << GP_INTSRC_INTSRC6_Pos)                  /*!< GP_T::INTSRC: INTSRC6 Mask                */

#define GP_INTSRC_INTSRC7_Pos            (7)                                               /*!< GP_T::INTSRC: INTSRC7 Position            */
#define GP_INTSRC_INTSRC7_Msk            (0x1ul << GP_INTSRC_INTSRC7_Pos)                  /*!< GP_T::INTSRC: INTSRC7 Mask                */

#define GP_INTSRC_INTSRC8_Pos            (8)                                               /*!< GP_T::INTSRC: INTSRC8 Position            */
#define GP_INTSRC_INTSRC8_Msk            (0x1ul << GP_INTSRC_INTSRC8_Pos)                  /*!< GP_T::INTSRC: INTSRC8 Mask                */

#define GP_INTSRC_INTSRC9_Pos            (9)                                               /*!< GP_T::INTSRC: INTSRC9 Position            */
#define GP_INTSRC_INTSRC9_Msk            (0x1ul << GP_INTSRC_INTSRC9_Pos)                  /*!< GP_T::INTSRC: INTSRC9 Mask                */

#define GP_INTSRC_INTSRC10_Pos           (10)                                              /*!< GP_T::INTSRC: INTSRC10 Position           */
#define GP_INTSRC_INTSRC10_Msk           (0x1ul << GP_INTSRC_INTSRC10_Pos)                 /*!< GP_T::INTSRC: INTSRC10 Mask               */

#define GP_INTSRC_INTSRC11_Pos           (11)                                              /*!< GP_T::INTSRC: INTSRC11 Position           */
#define GP_INTSRC_INTSRC11_Msk           (0x1ul << GP_INTSRC_INTSRC11_Pos)                 /*!< GP_T::INTSRC: INTSRC11 Mask               */

#define GP_INTSRC_INTSRC12_Pos           (12)                                              /*!< GP_T::INTSRC: INTSRC12 Position           */
#define GP_INTSRC_INTSRC12_Msk           (0x1ul << GP_INTSRC_INTSRC12_Pos)                 /*!< GP_T::INTSRC: INTSRC12 Mask               */

#define GP_INTSRC_INTSRC13_Pos           (13)                                              /*!< GP_T::INTSRC: INTSRC13 Position           */
#define GP_INTSRC_INTSRC13_Msk           (0x1ul << GP_INTSRC_INTSRC13_Pos)                 /*!< GP_T::INTSRC: INTSRC13 Mask               */

#define GP_INTSRC_INTSRC14_Pos           (14)                                              /*!< GP_T::INTSRC: INTSRC14 Position           */
#define GP_INTSRC_INTSRC14_Msk           (0x1ul << GP_INTSRC_INTSRC14_Pos)                 /*!< GP_T::INTSRC: INTSRC14 Mask               */

#define GP_INTSRC_INTSRC15_Pos           (15)                                              /*!< GP_T::INTSRC: INTSRC15 Position           */
#define GP_INTSRC_INTSRC15_Msk           (0x1ul << GP_INTSRC_INTSRC15_Pos)                 /*!< GP_T::INTSRC: INTSRC15 Mask               */

#define GP_MODE_MODE0_Pos                (0)                                               /*!< GP_T::MODE: MODE0 Position                */
#define GP_MODE_MODE0_Msk                (0x3ul << GP_MODE_MODE0_Pos)                      /*!< GP_T::MODE: MODE0 Mask                    */

#define GP_MODE_MODE1_Pos                (2)                                               /*!< GP_T::MODE: MODE1 Position                */
#define GP_MODE_MODE1_Msk                (0x3ul << GP_MODE_MODE1_Pos)                      /*!< GP_T::MODE: MODE1 Mask                    */

#define GP_MODE_MODE2_Pos                (4)                                               /*!< GP_T::MODE: MODE2 Position                */
#define GP_MODE_MODE2_Msk                (0x3ul << GP_MODE_MODE2_Pos)                      /*!< GP_T::MODE: MODE2 Mask                    */

#define GP_MODE_MODE3_Pos                (6)                                               /*!< GP_T::MODE: MODE3 Position                */
#define GP_MODE_MODE3_Msk                (0x3ul << GP_MODE_MODE3_Pos)                      /*!< GP_T::MODE: MODE3 Mask                    */

#define GP_MODE_MODE4_Pos                (8)                                               /*!< GP_T::MODE: MODE4 Position                */
#define GP_MODE_MODE4_Msk                (0x3ul << GP_MODE_MODE4_Pos)                      /*!< GP_T::MODE: MODE4 Mask                    */

#define GP_MODE_MODE5_Pos                (10)                                              /*!< GP_T::MODE: MODE5 Position                */
#define GP_MODE_MODE5_Msk                (0x3ul << GP_MODE_MODE5_Pos)                      /*!< GP_T::MODE: MODE5 Mask                    */

#define GP_DINOFF_DINOFF0_Pos            (16)                                              /*!< GP_T::DINOFF: DINOFF0 Position            */
#define GP_DINOFF_DINOFF0_Msk            (0x1ul << GP_DINOFF_DINOFF0_Pos)                  /*!< GP_T::DINOFF: DINOFF0 Mask                */

#define GP_DINOFF_DINOFF1_Pos            (17)                                              /*!< GP_T::DINOFF: DINOFF1 Position            */
#define GP_DINOFF_DINOFF1_Msk            (0x1ul << GP_DINOFF_DINOFF1_Pos)                  /*!< GP_T::DINOFF: DINOFF1 Mask                */

#define GP_DINOFF_DINOFF2_Pos            (18)                                              /*!< GP_T::DINOFF: DINOFF2 Position            */
#define GP_DINOFF_DINOFF2_Msk            (0x1ul << GP_DINOFF_DINOFF2_Pos)                  /*!< GP_T::DINOFF: DINOFF2 Mask                */

#define GP_DINOFF_DINOFF3_Pos            (19)                                              /*!< GP_T::DINOFF: DINOFF3 Position            */
#define GP_DINOFF_DINOFF3_Msk            (0x1ul << GP_DINOFF_DINOFF3_Pos)                  /*!< GP_T::DINOFF: DINOFF3 Mask                */

#define GP_DINOFF_DINOFF4_Pos            (20)                                              /*!< GP_T::DINOFF: DINOFF4 Position            */
#define GP_DINOFF_DINOFF4_Msk            (0x1ul << GP_DINOFF_DINOFF4_Pos)                  /*!< GP_T::DINOFF: DINOFF4 Mask                */

#define GP_DINOFF_DINOFF5_Pos            (21)                                              /*!< GP_T::DINOFF: DINOFF5 Position            */
#define GP_DINOFF_DINOFF5_Msk            (0x1ul << GP_DINOFF_DINOFF5_Pos)                  /*!< GP_T::DINOFF: DINOFF5 Mask                */

#define GP_DINOFF_DINOFF6_Pos            (22)                                              /*!< GP_T::DINOFF: DINOFF6 Position            */
#define GP_DINOFF_DINOFF6_Msk            (0x1ul << GP_DINOFF_DINOFF6_Pos)                  /*!< GP_T::DINOFF: DINOFF6 Mask                */

#define GP_DINOFF_DINOFF7_Pos            (23)                                              /*!< GP_T::DINOFF: DINOFF7 Position            */
#define GP_DINOFF_DINOFF7_Msk            (0x1ul << GP_DINOFF_DINOFF7_Pos)                  /*!< GP_T::DINOFF: DINOFF7 Mask                */

#define GP_DOUT_DOUT0_Pos                (0)                                               /*!< GP_T::DOUT: DOUT0 Position                */
#define GP_DOUT_DOUT0_Msk                (0x1ul << GP_DOUT_DOUT0_Pos)                      /*!< GP_T::DOUT: DOUT0 Mask                    */

#define GP_DOUT_DOUT1_Pos                (1)                                               /*!< GP_T::DOUT: DOUT1 Position                */
#define GP_DOUT_DOUT1_Msk                (0x1ul << GP_DOUT_DOUT1_Pos)                      /*!< GP_T::DOUT: DOUT1 Mask                    */

#define GP_DOUT_DOUT2_Pos                (2)                                               /*!< GP_T::DOUT: DOUT2 Position                */
#define GP_DOUT_DOUT2_Msk                (0x1ul << GP_DOUT_DOUT2_Pos)                      /*!< GP_T::DOUT: DOUT2 Mask                    */

#define GP_DOUT_DOUT3_Pos                (3)                                               /*!< GP_T::DOUT: DOUT3 Position                */
#define GP_DOUT_DOUT3_Msk                (0x1ul << GP_DOUT_DOUT3_Pos)                      /*!< GP_T::DOUT: DOUT3 Mask                    */

#define GP_DOUT_DOUT4_Pos                (4)                                               /*!< GP_T::DOUT: DOUT4 Position                */
#define GP_DOUT_DOUT4_Msk                (0x1ul << GP_DOUT_DOUT4_Pos)                      /*!< GP_T::DOUT: DOUT4 Mask                    */

#define GP_DOUT_DOUT5_Pos                (5)                                               /*!< GP_T::DOUT: DOUT5 Position                */
#define GP_DOUT_DOUT5_Msk                (0x1ul << GP_DOUT_DOUT5_Pos)                      /*!< GP_T::DOUT: DOUT5 Mask                    */

#define GP_DOUT_DOUT6_Pos                (6)                                               /*!< GP_T::DOUT: DOUT6 Position                */
#define GP_DOUT_DOUT6_Msk                (0x1ul << GP_DOUT_DOUT6_Pos)                      /*!< GP_T::DOUT: DOUT6 Mask                    */

#define GP_DOUT_DOUT7_Pos                (7)                                               /*!< GP_T::DOUT: DOUT7 Position                */
#define GP_DOUT_DOUT7_Msk                (0x1ul << GP_DOUT_DOUT7_Pos)                      /*!< GP_T::DOUT: DOUT7 Mask                    */

#define GP_DATMSK_DATMSK0_Pos            (0)                                               /*!< GP_T::DATMSK: DATMSK0 Position            */
#define GP_DATMSK_DATMSK0_Msk            (0x1ul << GP_DATMSK_DATMSK0_Pos)                  /*!< GP_T::DATMSK: DATMSK0 Mask                */

#define GP_DATMSK_DATMSK1_Pos            (1)                                               /*!< GP_T::DATMSK: DATMSK1 Position            */
#define GP_DATMSK_DATMSK1_Msk            (0x1ul << GP_DATMSK_DATMSK1_Pos)                  /*!< GP_T::DATMSK: DATMSK1 Mask                */

#define GP_DATMSK_DATMSK2_Pos            (2)                                               /*!< GP_T::DATMSK: DATMSK2 Position            */
#define GP_DATMSK_DATMSK2_Msk            (0x1ul << GP_DATMSK_DATMSK2_Pos)                  /*!< GP_T::DATMSK: DATMSK2 Mask                */

#define GP_DATMSK_DATMSK3_Pos            (3)                                               /*!< GP_T::DATMSK: DATMSK3 Position            */
#define GP_DATMSK_DATMSK3_Msk            (0x1ul << GP_DATMSK_DATMSK3_Pos)                  /*!< GP_T::DATMSK: DATMSK3 Mask                */

#define GP_DATMSK_DATMSK4_Pos            (4)                                               /*!< GP_T::DATMSK: DATMSK4 Position            */
#define GP_DATMSK_DATMSK4_Msk            (0x1ul << GP_DATMSK_DATMSK4_Pos)                  /*!< GP_T::DATMSK: DATMSK4 Mask                */

#define GP_DATMSK_DATMSK5_Pos            (5)                                               /*!< GP_T::DATMSK: DATMSK5 Position            */
#define GP_DATMSK_DATMSK5_Msk            (0x1ul << GP_DATMSK_DATMSK5_Pos)                  /*!< GP_T::DATMSK: DATMSK5 Mask                */

#define GP_DATMSK_DATMSK6_Pos            (6)                                               /*!< GP_T::DATMSK: DATMSK6 Position            */
#define GP_DATMSK_DATMSK6_Msk            (0x1ul << GP_DATMSK_DATMSK6_Pos)                  /*!< GP_T::DATMSK: DATMSK6 Mask                */

#define GP_DATMSK_DATMSK7_Pos            (7)                                               /*!< GP_T::DATMSK: DATMSK7 Position            */
#define GP_DATMSK_DATMSK7_Msk            (0x1ul << GP_DATMSK_DATMSK7_Pos)                  /*!< GP_T::DATMSK: DATMSK7 Mask                */

#define GP_PIN_PIN0_Pos                  (0)                                               /*!< GP_T::PIN: PIN0 Position                  */
#define GP_PIN_PIN0_Msk                  (0x1ul << GP_PIN_PIN0_Pos)                        /*!< GP_T::PIN: PIN0 Mask                      */

#define GP_PIN_PIN1_Pos                  (1)                                               /*!< GP_T::PIN: PIN1 Position                  */
#define GP_PIN_PIN1_Msk                  (0x1ul << GP_PIN_PIN1_Pos)                        /*!< GP_T::PIN: PIN1 Mask                      */

#define GP_PIN_PIN2_Pos                  (2)                                               /*!< GP_T::PIN: PIN2 Position                  */
#define GP_PIN_PIN2_Msk                  (0x1ul << GP_PIN_PIN2_Pos)                        /*!< GP_T::PIN: PIN2 Mask                      */

#define GP_PIN_PIN3_Pos                  (3)                                               /*!< GP_T::PIN: PIN3 Position                  */
#define GP_PIN_PIN3_Msk                  (0x1ul << GP_PIN_PIN3_Pos)                        /*!< GP_T::PIN: PIN3 Mask                      */

#define GP_PIN_PIN4_Pos                  (4)                                               /*!< GP_T::PIN: PIN4 Position                  */
#define GP_PIN_PIN4_Msk                  (0x1ul << GP_PIN_PIN4_Pos)                        /*!< GP_T::PIN: PIN4 Mask                      */

#define GP_PIN_PIN5_Pos                  (5)                                               /*!< GP_T::PIN: PIN5 Position                  */
#define GP_PIN_PIN5_Msk                  (0x1ul << GP_PIN_PIN5_Pos)                        /*!< GP_T::PIN: PIN5 Mask                      */

#define GP_PIN_PIN6_Pos                  (6)                                               /*!< GP_T::PIN: PIN6 Position                  */
#define GP_PIN_PIN6_Msk                  (0x1ul << GP_PIN_PIN6_Pos)                        /*!< GP_T::PIN: PIN6 Mask                      */

#define GP_PIN_PIN7_Pos                  (7)                                               /*!< GP_T::PIN: PIN7 Position                  */
#define GP_PIN_PIN7_Msk                  (0x1ul << GP_PIN_PIN7_Pos)                        /*!< GP_T::PIN: PIN7 Mask                      */

#define GP_DBEN_DBEN0_Pos                (0)                                               /*!< GP_T::DBEN: DBEN0 Position                */
#define GP_DBEN_DBEN0_Msk                (0x1ul << GP_DBEN_DBEN0_Pos)                      /*!< GP_T::DBEN: DBEN0 Mask                    */

#define GP_DBEN_DBEN1_Pos                (1)                                               /*!< GP_T::DBEN: DBEN1 Position                */
#define GP_DBEN_DBEN1_Msk                (0x1ul << GP_DBEN_DBEN1_Pos)                      /*!< GP_T::DBEN: DBEN1 Mask                    */

#define GP_DBEN_DBEN2_Pos                (2)                                               /*!< GP_T::DBEN: DBEN2 Position                */
#define GP_DBEN_DBEN2_Msk                (0x1ul << GP_DBEN_DBEN2_Pos)                      /*!< GP_T::DBEN: DBEN2 Mask                    */

#define GP_DBEN_DBEN3_Pos                (3)                                               /*!< GP_T::DBEN: DBEN3 Position                */
#define GP_DBEN_DBEN3_Msk                (0x1ul << GP_DBEN_DBEN3_Pos)                      /*!< GP_T::DBEN: DBEN3 Mask                    */

#define GP_DBEN_DBEN4_Pos                (4)                                               /*!< GP_T::DBEN: DBEN4 Position                */
#define GP_DBEN_DBEN4_Msk                (0x1ul << GP_DBEN_DBEN4_Pos)                      /*!< GP_T::DBEN: DBEN4 Mask                    */

#define GP_DBEN_DBEN5_Pos                (5)                                               /*!< GP_T::DBEN: DBEN5 Position                */
#define GP_DBEN_DBEN5_Msk                (0x1ul << GP_DBEN_DBEN5_Pos)                      /*!< GP_T::DBEN: DBEN5 Mask                    */

#define GP_DBEN_DBEN6_Pos                (6)                                               /*!< GP_T::DBEN: DBEN6 Position                */
#define GP_DBEN_DBEN6_Msk                (0x1ul << GP_DBEN_DBEN6_Pos)                      /*!< GP_T::DBEN: DBEN6 Mask                    */

#define GP_DBEN_DBEN7_Pos                (7)                                               /*!< GP_T::DBEN: DBEN7 Position                */
#define GP_DBEN_DBEN7_Msk                (0x1ul << GP_DBEN_DBEN7_Pos)                      /*!< GP_T::DBEN: DBEN7 Mask                    */

#define GP_INTTYPE_TYPE0_Pos             (0)                                               /*!< GP_T::INTTYPE: TYPE0 Position             */
#define GP_INTTYPE_TYPE0_Msk             (0x1ul << GP_INTTYPE_TYPE0_Pos)                   /*!< GP_T::INTTYPE: TYPE0 Mask                 */

#define GP_INTTYPE_TYPE1_Pos             (1)                                               /*!< GP_T::INTTYPE: TYPE1 Position             */
#define GP_INTTYPE_TYPE1_Msk             (0x1ul << GP_INTTYPE_TYPE1_Pos)                   /*!< GP_T::INTTYPE: TYPE1 Mask                 */

#define GP_INTTYPE_TYPE2_Pos             (2)                                               /*!< GP_T::INTTYPE: TYPE2 Position             */
#define GP_INTTYPE_TYPE2_Msk             (0x1ul << GP_INTTYPE_TYPE2_Pos)                   /*!< GP_T::INTTYPE: TYPE2 Mask                 */

#define GP_INTTYPE_TYPE3_Pos             (3)                                               /*!< GP_T::INTTYPE: TYPE3 Position             */
#define GP_INTTYPE_TYPE3_Msk             (0x1ul << GP_INTTYPE_TYPE3_Pos)                   /*!< GP_T::INTTYPE: TYPE3 Mask                 */

#define GP_INTTYPE_TYPE4_Pos             (4)                                               /*!< GP_T::INTTYPE: TYPE4 Position             */
#define GP_INTTYPE_TYPE4_Msk             (0x1ul << GP_INTTYPE_TYPE4_Pos)                   /*!< GP_T::INTTYPE: TYPE4 Mask                 */

#define GP_INTTYPE_TYPE5_Pos             (5)                                               /*!< GP_T::INTTYPE: TYPE5 Position             */
#define GP_INTTYPE_TYPE5_Msk             (0x1ul << GP_INTTYPE_TYPE5_Pos)                   /*!< GP_T::INTTYPE: TYPE5 Mask                 */

#define GP_INTTYPE_TYPE6_Pos             (6)                                               /*!< GP_T::INTTYPE: TYPE6 Position             */
#define GP_INTTYPE_TYPE6_Msk             (0x1ul << GP_INTTYPE_TYPE6_Pos)                   /*!< GP_T::INTTYPE: TYPE6 Mask                 */

#define GP_INTTYPE_TYPE7_Pos             (7)                                               /*!< GP_T::INTTYPE: TYPE7 Position             */
#define GP_INTTYPE_TYPE7_Msk             (0x1ul << GP_INTTYPE_TYPE7_Pos)                   /*!< GP_T::INTTYPE: TYPE7 Mask                 */

#define GP_INTEN_FLIEN0_Pos              (0)                                               /*!< GP_T::INTEN: FLIEN0 Position              */
#define GP_INTEN_FLIEN0_Msk              (0x1ul << GP_INTEN_FLIEN0_Pos)                    /*!< GP_T::INTEN: FLIEN0 Mask                  */

#define GP_INTEN_FLIEN1_Pos              (1)                                               /*!< GP_T::INTEN: FLIEN1 Position              */
#define GP_INTEN_FLIEN1_Msk              (0x1ul << GP_INTEN_FLIEN1_Pos)                    /*!< GP_T::INTEN: FLIEN1 Mask                  */

#define GP_INTEN_FLIEN2_Pos              (2)                                               /*!< GP_T::INTEN: FLIEN2 Position              */
#define GP_INTEN_FLIEN2_Msk              (0x1ul << GP_INTEN_FLIEN2_Pos)                    /*!< GP_T::INTEN: FLIEN2 Mask                  */

#define GP_INTEN_FLIEN3_Pos              (3)                                               /*!< GP_T::INTEN: FLIEN3 Position              */
#define GP_INTEN_FLIEN3_Msk              (0x1ul << GP_INTEN_FLIEN3_Pos)                    /*!< GP_T::INTEN: FLIEN3 Mask                  */

#define GP_INTEN_FLIEN4_Pos              (4)                                               /*!< GP_T::INTEN: FLIEN4 Position              */
#define GP_INTEN_FLIEN4_Msk              (0x1ul << GP_INTEN_FLIEN4_Pos)                    /*!< GP_T::INTEN: FLIEN4 Mask                  */

#define GP_INTEN_FLIEN5_Pos              (5)                                               /*!< GP_T::INTEN: FLIEN5 Position              */
#define GP_INTEN_FLIEN5_Msk              (0x1ul << GP_INTEN_FLIEN5_Pos)                    /*!< GP_T::INTEN: FLIEN5 Mask                  */

#define GP_INTEN_FLIEN6_Pos              (6)                                               /*!< GP_T::INTEN: FLIEN6 Position              */
#define GP_INTEN_FLIEN6_Msk              (0x1ul << GP_INTEN_FLIEN6_Pos)                    /*!< GP_T::INTEN: FLIEN6 Mask                  */

#define GP_INTEN_FLIEN7_Pos              (7)                                               /*!< GP_T::INTEN: FLIEN7 Position              */
#define GP_INTEN_FLIEN7_Msk              (0x1ul << GP_INTEN_FLIEN7_Pos)                    /*!< GP_T::INTEN: FLIEN7 Mask                  */

#define GP_INTEN_RHIEN0_Pos              (16)                                              /*!< GP_T::INTEN: RHIEN0 Position              */
#define GP_INTEN_RHIEN0_Msk              (0x1ul << GP_INTEN_RHIEN0_Pos)                    /*!< GP_T::INTEN: RHIEN0 Mask                  */

#define GP_INTEN_RHIEN1_Pos              (17)                                              /*!< GP_T::INTEN: RHIEN1 Position              */
#define GP_INTEN_RHIEN1_Msk              (0x1ul << GP_INTEN_RHIEN1_Pos)                    /*!< GP_T::INTEN: RHIEN1 Mask                  */

#define GP_INTEN_RHIEN2_Pos              (18)                                              /*!< GP_T::INTEN: RHIEN2 Position              */
#define GP_INTEN_RHIEN2_Msk              (0x1ul << GP_INTEN_RHIEN2_Pos)                    /*!< GP_T::INTEN: RHIEN2 Mask                  */

#define GP_INTEN_RHIEN3_Pos              (19)                                              /*!< GP_T::INTEN: RHIEN3 Position              */
#define GP_INTEN_RHIEN3_Msk              (0x1ul << GP_INTEN_RHIEN3_Pos)                    /*!< GP_T::INTEN: RHIEN3 Mask                  */

#define GP_INTEN_RHIEN4_Pos              (20)                                              /*!< GP_T::INTEN: RHIEN4 Position              */
#define GP_INTEN_RHIEN4_Msk              (0x1ul << GP_INTEN_RHIEN4_Pos)                    /*!< GP_T::INTEN: RHIEN4 Mask                  */

#define GP_INTEN_RHIEN5_Pos              (21)                                              /*!< GP_T::INTEN: RHIEN5 Position              */
#define GP_INTEN_RHIEN5_Msk              (0x1ul << GP_INTEN_RHIEN5_Pos)                    /*!< GP_T::INTEN: RHIEN5 Mask                  */

#define GP_INTEN_RHIEN6_Pos              (22)                                              /*!< GP_T::INTEN: RHIEN6 Position              */
#define GP_INTEN_RHIEN6_Msk              (0x1ul << GP_INTEN_RHIEN6_Pos)                    /*!< GP_T::INTEN: RHIEN6 Mask                  */

#define GP_INTEN_RHIEN7_Pos              (23)                                              /*!< GP_T::INTEN: RHIEN7 Position              */
#define GP_INTEN_RHIEN7_Msk              (0x1ul << GP_INTEN_RHIEN7_Pos)                    /*!< GP_T::INTEN: RHIEN7 Mask                  */

#define GP_INTSRC_INTSRC0_Pos            (0)                                               /*!< GP_T::INTSRC: INTSRC0 Position            */
#define GP_INTSRC_INTSRC0_Msk            (0x1ul << GP_INTSRC_INTSRC0_Pos)                  /*!< GP_T::INTSRC: INTSRC0 Mask                */

#define GP_INTSRC_INTSRC1_Pos            (1)                                               /*!< GP_T::INTSRC: INTSRC1 Position            */
#define GP_INTSRC_INTSRC1_Msk            (0x1ul << GP_INTSRC_INTSRC1_Pos)                  /*!< GP_T::INTSRC: INTSRC1 Mask                */

#define GP_INTSRC_INTSRC2_Pos            (2)                                               /*!< GP_T::INTSRC: INTSRC2 Position            */
#define GP_INTSRC_INTSRC2_Msk            (0x1ul << GP_INTSRC_INTSRC2_Pos)                  /*!< GP_T::INTSRC: INTSRC2 Mask                */

#define GP_INTSRC_INTSRC3_Pos            (3)                                               /*!< GP_T::INTSRC: INTSRC3 Position            */
#define GP_INTSRC_INTSRC3_Msk            (0x1ul << GP_INTSRC_INTSRC3_Pos)                  /*!< GP_T::INTSRC: INTSRC3 Mask                */

#define GP_INTSRC_INTSRC4_Pos            (4)                                               /*!< GP_T::INTSRC: INTSRC4 Position            */
#define GP_INTSRC_INTSRC4_Msk            (0x1ul << GP_INTSRC_INTSRC4_Pos)                  /*!< GP_T::INTSRC: INTSRC4 Mask                */

#define GP_INTSRC_INTSRC5_Pos            (5)                                               /*!< GP_T::INTSRC: INTSRC5 Position            */
#define GP_INTSRC_INTSRC5_Msk            (0x1ul << GP_INTSRC_INTSRC5_Pos)                  /*!< GP_T::INTSRC: INTSRC5 Mask                */

#define GP_INTSRC_INTSRC6_Pos            (6)                                               /*!< GP_T::INTSRC: INTSRC6 Position            */
#define GP_INTSRC_INTSRC6_Msk            (0x1ul << GP_INTSRC_INTSRC6_Pos)                  /*!< GP_T::INTSRC: INTSRC6 Mask                */

#define GP_INTSRC_INTSRC7_Pos            (7)                                               /*!< GP_T::INTSRC: INTSRC7 Position            */
#define GP_INTSRC_INTSRC7_Msk            (0x1ul << GP_INTSRC_INTSRC7_Pos)                  /*!< GP_T::INTSRC: INTSRC7 Mask                */

#define GP_INTSRC_INTSRC8_Pos            (8)                                               /*!< GP_T::INTSRC: INTSRC8 Position            */
#define GP_INTSRC_INTSRC8_Msk            (0x1ul << GP_INTSRC_INTSRC8_Pos)                  /*!< GP_T::INTSRC: INTSRC8 Mask                */

#define GP_INTSRC_INTSRC9_Pos            (9)                                               /*!< GP_T::INTSRC: INTSRC9 Position            */
#define GP_INTSRC_INTSRC9_Msk            (0x1ul << GP_INTSRC_INTSRC9_Pos)                  /*!< GP_T::INTSRC: INTSRC9 Mask                */

#define GP_INTSRC_INTSRC10_Pos           (10)                                              /*!< GP_T::INTSRC: INTSRC10 Position           */
#define GP_INTSRC_INTSRC10_Msk           (0x1ul << GP_INTSRC_INTSRC10_Pos)                 /*!< GP_T::INTSRC: INTSRC10 Mask               */

#define GP_INTSRC_INTSRC11_Pos           (11)                                              /*!< GP_T::INTSRC: INTSRC11 Position           */
#define GP_INTSRC_INTSRC11_Msk           (0x1ul << GP_INTSRC_INTSRC11_Pos)                 /*!< GP_T::INTSRC: INTSRC11 Mask               */

#define GP_INTSRC_INTSRC12_Pos           (12)                                              /*!< GP_T::INTSRC: INTSRC12 Position           */
#define GP_INTSRC_INTSRC12_Msk           (0x1ul << GP_INTSRC_INTSRC12_Pos)                 /*!< GP_T::INTSRC: INTSRC12 Mask               */

#define GP_INTSRC_INTSRC13_Pos           (13)                                              /*!< GP_T::INTSRC: INTSRC13 Position           */
#define GP_INTSRC_INTSRC13_Msk           (0x1ul << GP_INTSRC_INTSRC13_Pos)                 /*!< GP_T::INTSRC: INTSRC13 Mask               */

#define GP_INTSRC_INTSRC14_Pos           (14)                                              /*!< GP_T::INTSRC: INTSRC14 Position           */
#define GP_INTSRC_INTSRC14_Msk           (0x1ul << GP_INTSRC_INTSRC14_Pos)                 /*!< GP_T::INTSRC: INTSRC14 Mask               */

#define GP_INTSRC_INTSRC15_Pos           (15)                                              /*!< GP_T::INTSRC: INTSRC15 Position           */
#define GP_INTSRC_INTSRC15_Msk           (0x1ul << GP_INTSRC_INTSRC15_Pos)                 /*!< GP_T::INTSRC: INTSRC15 Mask               */

#define GP_MODE_MODE0_Pos                (0)                                               /*!< GP_T::MODE: MODE0 Position                */
#define GP_MODE_MODE0_Msk                (0x3ul << GP_MODE_MODE0_Pos)                      /*!< GP_T::MODE: MODE0 Mask                    */

#define GP_MODE_MODE1_Pos                (2)                                               /*!< GP_T::MODE: MODE1 Position                */
#define GP_MODE_MODE1_Msk                (0x3ul << GP_MODE_MODE1_Pos)                      /*!< GP_T::MODE: MODE1 Mask                    */

#define GP_MODE_MODE2_Pos                (4)                                               /*!< GP_T::MODE: MODE2 Position                */
#define GP_MODE_MODE2_Msk                (0x3ul << GP_MODE_MODE2_Pos)                      /*!< GP_T::MODE: MODE2 Mask                    */

#define GP_MODE_MODE3_Pos                (6)                                               /*!< GP_T::MODE: MODE3 Position                */
#define GP_MODE_MODE3_Msk                (0x3ul << GP_MODE_MODE3_Pos)                      /*!< GP_T::MODE: MODE3 Mask                    */

#define GP_MODE_MODE4_Pos                (8)                                               /*!< GP_T::MODE: MODE4 Position                */
#define GP_MODE_MODE4_Msk                (0x3ul << GP_MODE_MODE4_Pos)                      /*!< GP_T::MODE: MODE4 Mask                    */

#define GP_MODE_MODE5_Pos                (10)                                              /*!< GP_T::MODE: MODE5 Position                */
#define GP_MODE_MODE5_Msk                (0x3ul << GP_MODE_MODE5_Pos)                      /*!< GP_T::MODE: MODE5 Mask                    */

#define GP_DINOFF_DINOFF0_Pos            (16)                                              /*!< GP_T::DINOFF: DINOFF0 Position            */
#define GP_DINOFF_DINOFF0_Msk            (0x1ul << GP_DINOFF_DINOFF0_Pos)                  /*!< GP_T::DINOFF: DINOFF0 Mask                */

#define GP_DINOFF_DINOFF1_Pos            (17)                                              /*!< GP_T::DINOFF: DINOFF1 Position            */
#define GP_DINOFF_DINOFF1_Msk            (0x1ul << GP_DINOFF_DINOFF1_Pos)                  /*!< GP_T::DINOFF: DINOFF1 Mask                */

#define GP_DINOFF_DINOFF2_Pos            (18)                                              /*!< GP_T::DINOFF: DINOFF2 Position            */
#define GP_DINOFF_DINOFF2_Msk            (0x1ul << GP_DINOFF_DINOFF2_Pos)                  /*!< GP_T::DINOFF: DINOFF2 Mask                */

#define GP_DINOFF_DINOFF3_Pos            (19)                                              /*!< GP_T::DINOFF: DINOFF3 Position            */
#define GP_DINOFF_DINOFF3_Msk            (0x1ul << GP_DINOFF_DINOFF3_Pos)                  /*!< GP_T::DINOFF: DINOFF3 Mask                */

#define GP_DINOFF_DINOFF4_Pos            (20)                                              /*!< GP_T::DINOFF: DINOFF4 Position            */
#define GP_DINOFF_DINOFF4_Msk            (0x1ul << GP_DINOFF_DINOFF4_Pos)                  /*!< GP_T::DINOFF: DINOFF4 Mask                */

#define GP_DINOFF_DINOFF5_Pos            (21)                                              /*!< GP_T::DINOFF: DINOFF5 Position            */
#define GP_DINOFF_DINOFF5_Msk            (0x1ul << GP_DINOFF_DINOFF5_Pos)                  /*!< GP_T::DINOFF: DINOFF5 Mask                */

#define GP_DINOFF_DINOFF6_Pos            (22)                                              /*!< GP_T::DINOFF: DINOFF6 Position            */
#define GP_DINOFF_DINOFF6_Msk            (0x1ul << GP_DINOFF_DINOFF6_Pos)                  /*!< GP_T::DINOFF: DINOFF6 Mask                */

#define GP_DINOFF_DINOFF7_Pos            (23)                                              /*!< GP_T::DINOFF: DINOFF7 Position            */
#define GP_DINOFF_DINOFF7_Msk            (0x1ul << GP_DINOFF_DINOFF7_Pos)                  /*!< GP_T::DINOFF: DINOFF7 Mask                */

#define GP_DOUT_DOUT0_Pos                (0)                                               /*!< GP_T::DOUT: DOUT0 Position                */
#define GP_DOUT_DOUT0_Msk                (0x1ul << GP_DOUT_DOUT0_Pos)                      /*!< GP_T::DOUT: DOUT0 Mask                    */

#define GP_DOUT_DOUT1_Pos                (1)                                               /*!< GP_T::DOUT: DOUT1 Position                */
#define GP_DOUT_DOUT1_Msk                (0x1ul << GP_DOUT_DOUT1_Pos)                      /*!< GP_T::DOUT: DOUT1 Mask                    */

#define GP_DOUT_DOUT2_Pos                (2)                                               /*!< GP_T::DOUT: DOUT2 Position                */
#define GP_DOUT_DOUT2_Msk                (0x1ul << GP_DOUT_DOUT2_Pos)                      /*!< GP_T::DOUT: DOUT2 Mask                    */

#define GP_DOUT_DOUT3_Pos                (3)                                               /*!< GP_T::DOUT: DOUT3 Position                */
#define GP_DOUT_DOUT3_Msk                (0x1ul << GP_DOUT_DOUT3_Pos)                      /*!< GP_T::DOUT: DOUT3 Mask                    */

#define GP_DOUT_DOUT4_Pos                (4)                                               /*!< GP_T::DOUT: DOUT4 Position                */
#define GP_DOUT_DOUT4_Msk                (0x1ul << GP_DOUT_DOUT4_Pos)                      /*!< GP_T::DOUT: DOUT4 Mask                    */

#define GP_DOUT_DOUT5_Pos                (5)                                               /*!< GP_T::DOUT: DOUT5 Position                */
#define GP_DOUT_DOUT5_Msk                (0x1ul << GP_DOUT_DOUT5_Pos)                      /*!< GP_T::DOUT: DOUT5 Mask                    */

#define GP_DOUT_DOUT6_Pos                (6)                                               /*!< GP_T::DOUT: DOUT6 Position                */
#define GP_DOUT_DOUT6_Msk                (0x1ul << GP_DOUT_DOUT6_Pos)                      /*!< GP_T::DOUT: DOUT6 Mask                    */

#define GP_DOUT_DOUT7_Pos                (7)                                               /*!< GP_T::DOUT: DOUT7 Position                */
#define GP_DOUT_DOUT7_Msk                (0x1ul << GP_DOUT_DOUT7_Pos)                      /*!< GP_T::DOUT: DOUT7 Mask                    */

#define GP_DATMSK_DATMSK0_Pos            (0)                                               /*!< GP_T::DATMSK: DATMSK0 Position            */
#define GP_DATMSK_DATMSK0_Msk            (0x1ul << GP_DATMSK_DATMSK0_Pos)                  /*!< GP_T::DATMSK: DATMSK0 Mask                */

#define GP_DATMSK_DATMSK1_Pos            (1)                                               /*!< GP_T::DATMSK: DATMSK1 Position            */
#define GP_DATMSK_DATMSK1_Msk            (0x1ul << GP_DATMSK_DATMSK1_Pos)                  /*!< GP_T::DATMSK: DATMSK1 Mask                */

#define GP_DATMSK_DATMSK2_Pos            (2)                                               /*!< GP_T::DATMSK: DATMSK2 Position            */
#define GP_DATMSK_DATMSK2_Msk            (0x1ul << GP_DATMSK_DATMSK2_Pos)                  /*!< GP_T::DATMSK: DATMSK2 Mask                */

#define GP_DATMSK_DATMSK3_Pos            (3)                                               /*!< GP_T::DATMSK: DATMSK3 Position            */
#define GP_DATMSK_DATMSK3_Msk            (0x1ul << GP_DATMSK_DATMSK3_Pos)                  /*!< GP_T::DATMSK: DATMSK3 Mask                */

#define GP_DATMSK_DATMSK4_Pos            (4)                                               /*!< GP_T::DATMSK: DATMSK4 Position            */
#define GP_DATMSK_DATMSK4_Msk            (0x1ul << GP_DATMSK_DATMSK4_Pos)                  /*!< GP_T::DATMSK: DATMSK4 Mask                */

#define GP_DATMSK_DATMSK5_Pos            (5)                                               /*!< GP_T::DATMSK: DATMSK5 Position            */
#define GP_DATMSK_DATMSK5_Msk            (0x1ul << GP_DATMSK_DATMSK5_Pos)                  /*!< GP_T::DATMSK: DATMSK5 Mask                */

#define GP_DATMSK_DATMSK6_Pos            (6)                                               /*!< GP_T::DATMSK: DATMSK6 Position            */
#define GP_DATMSK_DATMSK6_Msk            (0x1ul << GP_DATMSK_DATMSK6_Pos)                  /*!< GP_T::DATMSK: DATMSK6 Mask                */

#define GP_DATMSK_DATMSK7_Pos            (7)                                               /*!< GP_T::DATMSK: DATMSK7 Position            */
#define GP_DATMSK_DATMSK7_Msk            (0x1ul << GP_DATMSK_DATMSK7_Pos)                  /*!< GP_T::DATMSK: DATMSK7 Mask                */

#define GP_PIN_PIN0_Pos                  (0)                                               /*!< GP_T::PIN: PIN0 Position                  */
#define GP_PIN_PIN0_Msk                  (0x1ul << GP_PIN_PIN0_Pos)                        /*!< GP_T::PIN: PIN0 Mask                      */

#define GP_PIN_PIN1_Pos                  (1)                                               /*!< GP_T::PIN: PIN1 Position                  */
#define GP_PIN_PIN1_Msk                  (0x1ul << GP_PIN_PIN1_Pos)                        /*!< GP_T::PIN: PIN1 Mask                      */

#define GP_PIN_PIN2_Pos                  (2)                                               /*!< GP_T::PIN: PIN2 Position                  */
#define GP_PIN_PIN2_Msk                  (0x1ul << GP_PIN_PIN2_Pos)                        /*!< GP_T::PIN: PIN2 Mask                      */

#define GP_PIN_PIN3_Pos                  (3)                                               /*!< GP_T::PIN: PIN3 Position                  */
#define GP_PIN_PIN3_Msk                  (0x1ul << GP_PIN_PIN3_Pos)                        /*!< GP_T::PIN: PIN3 Mask                      */

#define GP_PIN_PIN4_Pos                  (4)                                               /*!< GP_T::PIN: PIN4 Position                  */
#define GP_PIN_PIN4_Msk                  (0x1ul << GP_PIN_PIN4_Pos)                        /*!< GP_T::PIN: PIN4 Mask                      */

#define GP_PIN_PIN5_Pos                  (5)                                               /*!< GP_T::PIN: PIN5 Position                  */
#define GP_PIN_PIN5_Msk                  (0x1ul << GP_PIN_PIN5_Pos)                        /*!< GP_T::PIN: PIN5 Mask                      */

#define GP_PIN_PIN6_Pos                  (6)                                               /*!< GP_T::PIN: PIN6 Position                  */
#define GP_PIN_PIN6_Msk                  (0x1ul << GP_PIN_PIN6_Pos)                        /*!< GP_T::PIN: PIN6 Mask                      */

#define GP_PIN_PIN7_Pos                  (7)                                               /*!< GP_T::PIN: PIN7 Position                  */
#define GP_PIN_PIN7_Msk                  (0x1ul << GP_PIN_PIN7_Pos)                        /*!< GP_T::PIN: PIN7 Mask                      */

#define GP_DBEN_DBEN0_Pos                (0)                                               /*!< GP_T::DBEN: DBEN0 Position                */
#define GP_DBEN_DBEN0_Msk                (0x1ul << GP_DBEN_DBEN0_Pos)                      /*!< GP_T::DBEN: DBEN0 Mask                    */

#define GP_DBEN_DBEN1_Pos                (1)                                               /*!< GP_T::DBEN: DBEN1 Position                */
#define GP_DBEN_DBEN1_Msk                (0x1ul << GP_DBEN_DBEN1_Pos)                      /*!< GP_T::DBEN: DBEN1 Mask                    */

#define GP_DBEN_DBEN2_Pos                (2)                                               /*!< GP_T::DBEN: DBEN2 Position                */
#define GP_DBEN_DBEN2_Msk                (0x1ul << GP_DBEN_DBEN2_Pos)                      /*!< GP_T::DBEN: DBEN2 Mask                    */

#define GP_DBEN_DBEN3_Pos                (3)                                               /*!< GP_T::DBEN: DBEN3 Position                */
#define GP_DBEN_DBEN3_Msk                (0x1ul << GP_DBEN_DBEN3_Pos)                      /*!< GP_T::DBEN: DBEN3 Mask                    */

#define GP_DBEN_DBEN4_Pos                (4)                                               /*!< GP_T::DBEN: DBEN4 Position                */
#define GP_DBEN_DBEN4_Msk                (0x1ul << GP_DBEN_DBEN4_Pos)                      /*!< GP_T::DBEN: DBEN4 Mask                    */

#define GP_DBEN_DBEN5_Pos                (5)                                               /*!< GP_T::DBEN: DBEN5 Position                */
#define GP_DBEN_DBEN5_Msk                (0x1ul << GP_DBEN_DBEN5_Pos)                      /*!< GP_T::DBEN: DBEN5 Mask                    */

#define GP_DBEN_DBEN6_Pos                (6)                                               /*!< GP_T::DBEN: DBEN6 Position                */
#define GP_DBEN_DBEN6_Msk                (0x1ul << GP_DBEN_DBEN6_Pos)                      /*!< GP_T::DBEN: DBEN6 Mask                    */

#define GP_DBEN_DBEN7_Pos                (7)                                               /*!< GP_T::DBEN: DBEN7 Position                */
#define GP_DBEN_DBEN7_Msk                (0x1ul << GP_DBEN_DBEN7_Pos)                      /*!< GP_T::DBEN: DBEN7 Mask                    */

#define GP_INTTYPE_TYPE0_Pos             (0)                                               /*!< GP_T::INTTYPE: TYPE0 Position             */
#define GP_INTTYPE_TYPE0_Msk             (0x1ul << GP_INTTYPE_TYPE0_Pos)                   /*!< GP_T::INTTYPE: TYPE0 Mask                 */

#define GP_INTTYPE_TYPE1_Pos             (1)                                               /*!< GP_T::INTTYPE: TYPE1 Position             */
#define GP_INTTYPE_TYPE1_Msk             (0x1ul << GP_INTTYPE_TYPE1_Pos)                   /*!< GP_T::INTTYPE: TYPE1 Mask                 */

#define GP_INTTYPE_TYPE2_Pos             (2)                                               /*!< GP_T::INTTYPE: TYPE2 Position             */
#define GP_INTTYPE_TYPE2_Msk             (0x1ul << GP_INTTYPE_TYPE2_Pos)                   /*!< GP_T::INTTYPE: TYPE2 Mask                 */

#define GP_INTTYPE_TYPE3_Pos             (3)                                               /*!< GP_T::INTTYPE: TYPE3 Position             */
#define GP_INTTYPE_TYPE3_Msk             (0x1ul << GP_INTTYPE_TYPE3_Pos)                   /*!< GP_T::INTTYPE: TYPE3 Mask                 */

#define GP_INTTYPE_TYPE4_Pos             (4)                                               /*!< GP_T::INTTYPE: TYPE4 Position             */
#define GP_INTTYPE_TYPE4_Msk             (0x1ul << GP_INTTYPE_TYPE4_Pos)                   /*!< GP_T::INTTYPE: TYPE4 Mask                 */

#define GP_INTTYPE_TYPE5_Pos             (5)                                               /*!< GP_T::INTTYPE: TYPE5 Position             */
#define GP_INTTYPE_TYPE5_Msk             (0x1ul << GP_INTTYPE_TYPE5_Pos)                   /*!< GP_T::INTTYPE: TYPE5 Mask                 */

#define GP_INTTYPE_TYPE6_Pos             (6)                                               /*!< GP_T::INTTYPE: TYPE6 Position             */
#define GP_INTTYPE_TYPE6_Msk             (0x1ul << GP_INTTYPE_TYPE6_Pos)                   /*!< GP_T::INTTYPE: TYPE6 Mask                 */

#define GP_INTTYPE_TYPE7_Pos             (7)                                               /*!< GP_T::INTTYPE: TYPE7 Position             */
#define GP_INTTYPE_TYPE7_Msk             (0x1ul << GP_INTTYPE_TYPE7_Pos)                   /*!< GP_T::INTTYPE: TYPE7 Mask                 */

#define GP_INTEN_FLIEN0_Pos              (0)                                               /*!< GP_T::INTEN: FLIEN0 Position              */
#define GP_INTEN_FLIEN0_Msk              (0x1ul << GP_INTEN_FLIEN0_Pos)                    /*!< GP_T::INTEN: FLIEN0 Mask                  */

#define GP_INTEN_FLIEN1_Pos              (1)                                               /*!< GP_T::INTEN: FLIEN1 Position              */
#define GP_INTEN_FLIEN1_Msk              (0x1ul << GP_INTEN_FLIEN1_Pos)                    /*!< GP_T::INTEN: FLIEN1 Mask                  */

#define GP_INTEN_FLIEN2_Pos              (2)                                               /*!< GP_T::INTEN: FLIEN2 Position              */
#define GP_INTEN_FLIEN2_Msk              (0x1ul << GP_INTEN_FLIEN2_Pos)                    /*!< GP_T::INTEN: FLIEN2 Mask                  */

#define GP_INTEN_FLIEN3_Pos              (3)                                               /*!< GP_T::INTEN: FLIEN3 Position              */
#define GP_INTEN_FLIEN3_Msk              (0x1ul << GP_INTEN_FLIEN3_Pos)                    /*!< GP_T::INTEN: FLIEN3 Mask                  */

#define GP_INTEN_FLIEN4_Pos              (4)                                               /*!< GP_T::INTEN: FLIEN4 Position              */
#define GP_INTEN_FLIEN4_Msk              (0x1ul << GP_INTEN_FLIEN4_Pos)                    /*!< GP_T::INTEN: FLIEN4 Mask                  */

#define GP_INTEN_FLIEN5_Pos              (5)                                               /*!< GP_T::INTEN: FLIEN5 Position              */
#define GP_INTEN_FLIEN5_Msk              (0x1ul << GP_INTEN_FLIEN5_Pos)                    /*!< GP_T::INTEN: FLIEN5 Mask                  */

#define GP_INTEN_FLIEN6_Pos              (6)                                               /*!< GP_T::INTEN: FLIEN6 Position              */
#define GP_INTEN_FLIEN6_Msk              (0x1ul << GP_INTEN_FLIEN6_Pos)                    /*!< GP_T::INTEN: FLIEN6 Mask                  */

#define GP_INTEN_FLIEN7_Pos              (7)                                               /*!< GP_T::INTEN: FLIEN7 Position              */
#define GP_INTEN_FLIEN7_Msk              (0x1ul << GP_INTEN_FLIEN7_Pos)                    /*!< GP_T::INTEN: FLIEN7 Mask                  */

#define GP_INTEN_RHIEN0_Pos              (16)                                              /*!< GP_T::INTEN: RHIEN0 Position              */
#define GP_INTEN_RHIEN0_Msk              (0x1ul << GP_INTEN_RHIEN0_Pos)                    /*!< GP_T::INTEN: RHIEN0 Mask                  */

#define GP_INTEN_RHIEN1_Pos              (17)                                              /*!< GP_T::INTEN: RHIEN1 Position              */
#define GP_INTEN_RHIEN1_Msk              (0x1ul << GP_INTEN_RHIEN1_Pos)                    /*!< GP_T::INTEN: RHIEN1 Mask                  */

#define GP_INTEN_RHIEN2_Pos              (18)                                              /*!< GP_T::INTEN: RHIEN2 Position              */
#define GP_INTEN_RHIEN2_Msk              (0x1ul << GP_INTEN_RHIEN2_Pos)                    /*!< GP_T::INTEN: RHIEN2 Mask                  */

#define GP_INTEN_RHIEN3_Pos              (19)                                              /*!< GP_T::INTEN: RHIEN3 Position              */
#define GP_INTEN_RHIEN3_Msk              (0x1ul << GP_INTEN_RHIEN3_Pos)                    /*!< GP_T::INTEN: RHIEN3 Mask                  */

#define GP_INTEN_RHIEN4_Pos              (20)                                              /*!< GP_T::INTEN: RHIEN4 Position              */
#define GP_INTEN_RHIEN4_Msk              (0x1ul << GP_INTEN_RHIEN4_Pos)                    /*!< GP_T::INTEN: RHIEN4 Mask                  */

#define GP_INTEN_RHIEN5_Pos              (21)                                              /*!< GP_T::INTEN: RHIEN5 Position              */
#define GP_INTEN_RHIEN5_Msk              (0x1ul << GP_INTEN_RHIEN5_Pos)                    /*!< GP_T::INTEN: RHIEN5 Mask                  */

#define GP_INTEN_RHIEN6_Pos              (22)                                              /*!< GP_T::INTEN: RHIEN6 Position              */
#define GP_INTEN_RHIEN6_Msk              (0x1ul << GP_INTEN_RHIEN6_Pos)                    /*!< GP_T::INTEN: RHIEN6 Mask                  */

#define GP_INTEN_RHIEN7_Pos              (23)                                              /*!< GP_T::INTEN: RHIEN7 Position              */
#define GP_INTEN_RHIEN7_Msk              (0x1ul << GP_INTEN_RHIEN7_Pos)                    /*!< GP_T::INTEN: RHIEN7 Mask                  */

#define GP_INTSRC_INTSRC0_Pos            (0)                                               /*!< GP_T::INTSRC: INTSRC0 Position            */
#define GP_INTSRC_INTSRC0_Msk            (0x1ul << GP_INTSRC_INTSRC0_Pos)                  /*!< GP_T::INTSRC: INTSRC0 Mask                */

#define GP_INTSRC_INTSRC1_Pos            (1)                                               /*!< GP_T::INTSRC: INTSRC1 Position            */
#define GP_INTSRC_INTSRC1_Msk            (0x1ul << GP_INTSRC_INTSRC1_Pos)                  /*!< GP_T::INTSRC: INTSRC1 Mask                */

#define GP_INTSRC_INTSRC2_Pos            (2)                                               /*!< GP_T::INTSRC: INTSRC2 Position            */
#define GP_INTSRC_INTSRC2_Msk            (0x1ul << GP_INTSRC_INTSRC2_Pos)                  /*!< GP_T::INTSRC: INTSRC2 Mask                */

#define GP_INTSRC_INTSRC3_Pos            (3)                                               /*!< GP_T::INTSRC: INTSRC3 Position            */
#define GP_INTSRC_INTSRC3_Msk            (0x1ul << GP_INTSRC_INTSRC3_Pos)                  /*!< GP_T::INTSRC: INTSRC3 Mask                */

#define GP_INTSRC_INTSRC4_Pos            (4)                                               /*!< GP_T::INTSRC: INTSRC4 Position            */
#define GP_INTSRC_INTSRC4_Msk            (0x1ul << GP_INTSRC_INTSRC4_Pos)                  /*!< GP_T::INTSRC: INTSRC4 Mask                */

#define GP_INTSRC_INTSRC5_Pos            (5)                                               /*!< GP_T::INTSRC: INTSRC5 Position            */
#define GP_INTSRC_INTSRC5_Msk            (0x1ul << GP_INTSRC_INTSRC5_Pos)                  /*!< GP_T::INTSRC: INTSRC5 Mask                */

#define GP_INTSRC_INTSRC6_Pos            (6)                                               /*!< GP_T::INTSRC: INTSRC6 Position            */
#define GP_INTSRC_INTSRC6_Msk            (0x1ul << GP_INTSRC_INTSRC6_Pos)                  /*!< GP_T::INTSRC: INTSRC6 Mask                */

#define GP_INTSRC_INTSRC7_Pos            (7)                                               /*!< GP_T::INTSRC: INTSRC7 Position            */
#define GP_INTSRC_INTSRC7_Msk            (0x1ul << GP_INTSRC_INTSRC7_Pos)                  /*!< GP_T::INTSRC: INTSRC7 Mask                */

#define GP_INTSRC_INTSRC8_Pos            (8)                                               /*!< GP_T::INTSRC: INTSRC8 Position            */
#define GP_INTSRC_INTSRC8_Msk            (0x1ul << GP_INTSRC_INTSRC8_Pos)                  /*!< GP_T::INTSRC: INTSRC8 Mask                */

#define GP_INTSRC_INTSRC9_Pos            (9)                                               /*!< GP_T::INTSRC: INTSRC9 Position            */
#define GP_INTSRC_INTSRC9_Msk            (0x1ul << GP_INTSRC_INTSRC9_Pos)                  /*!< GP_T::INTSRC: INTSRC9 Mask                */

#define GP_INTSRC_INTSRC10_Pos           (10)                                              /*!< GP_T::INTSRC: INTSRC10 Position           */
#define GP_INTSRC_INTSRC10_Msk           (0x1ul << GP_INTSRC_INTSRC10_Pos)                 /*!< GP_T::INTSRC: INTSRC10 Mask               */

#define GP_INTSRC_INTSRC11_Pos           (11)                                              /*!< GP_T::INTSRC: INTSRC11 Position           */
#define GP_INTSRC_INTSRC11_Msk           (0x1ul << GP_INTSRC_INTSRC11_Pos)                 /*!< GP_T::INTSRC: INTSRC11 Mask               */

#define GP_INTSRC_INTSRC12_Pos           (12)                                              /*!< GP_T::INTSRC: INTSRC12 Position           */
#define GP_INTSRC_INTSRC12_Msk           (0x1ul << GP_INTSRC_INTSRC12_Pos)                 /*!< GP_T::INTSRC: INTSRC12 Mask               */

#define GP_INTSRC_INTSRC13_Pos           (13)                                              /*!< GP_T::INTSRC: INTSRC13 Position           */
#define GP_INTSRC_INTSRC13_Msk           (0x1ul << GP_INTSRC_INTSRC13_Pos)                 /*!< GP_T::INTSRC: INTSRC13 Mask               */

#define GP_INTSRC_INTSRC14_Pos           (14)                                              /*!< GP_T::INTSRC: INTSRC14 Position           */
#define GP_INTSRC_INTSRC14_Msk           (0x1ul << GP_INTSRC_INTSRC14_Pos)                 /*!< GP_T::INTSRC: INTSRC14 Mask               */

#define GP_INTSRC_INTSRC15_Pos           (15)                                              /*!< GP_T::INTSRC: INTSRC15 Position           */
#define GP_INTSRC_INTSRC15_Msk           (0x1ul << GP_INTSRC_INTSRC15_Pos)                 /*!< GP_T::INTSRC: INTSRC15 Mask               */

#define GP_MODE_MODE0_Pos                (0)                                               /*!< GP_T::MODE: MODE0 Position                */
#define GP_MODE_MODE0_Msk                (0x3ul << GP_MODE_MODE0_Pos)                      /*!< GP_T::MODE: MODE0 Mask                    */

#define GP_MODE_MODE1_Pos                (2)                                               /*!< GP_T::MODE: MODE1 Position                */
#define GP_MODE_MODE1_Msk                (0x3ul << GP_MODE_MODE1_Pos)                      /*!< GP_T::MODE: MODE1 Mask                    */

#define GP_MODE_MODE2_Pos                (4)                                               /*!< GP_T::MODE: MODE2 Position                */
#define GP_MODE_MODE2_Msk                (0x3ul << GP_MODE_MODE2_Pos)                      /*!< GP_T::MODE: MODE2 Mask                    */

#define GP_MODE_MODE3_Pos                (6)                                               /*!< GP_T::MODE: MODE3 Position                */
#define GP_MODE_MODE3_Msk                (0x3ul << GP_MODE_MODE3_Pos)                      /*!< GP_T::MODE: MODE3 Mask                    */

#define GP_MODE_MODE4_Pos                (8)                                               /*!< GP_T::MODE: MODE4 Position                */
#define GP_MODE_MODE4_Msk                (0x3ul << GP_MODE_MODE4_Pos)                      /*!< GP_T::MODE: MODE4 Mask                    */

#define GP_MODE_MODE5_Pos                (10)                                              /*!< GP_T::MODE: MODE5 Position                */
#define GP_MODE_MODE5_Msk                (0x3ul << GP_MODE_MODE5_Pos)                      /*!< GP_T::MODE: MODE5 Mask                    */

#define GP_DINOFF_DINOFF0_Pos            (16)                                              /*!< GP_T::DINOFF: DINOFF0 Position            */
#define GP_DINOFF_DINOFF0_Msk            (0x1ul << GP_DINOFF_DINOFF0_Pos)                  /*!< GP_T::DINOFF: DINOFF0 Mask                */

#define GP_DINOFF_DINOFF1_Pos            (17)                                              /*!< GP_T::DINOFF: DINOFF1 Position            */
#define GP_DINOFF_DINOFF1_Msk            (0x1ul << GP_DINOFF_DINOFF1_Pos)                  /*!< GP_T::DINOFF: DINOFF1 Mask                */

#define GP_DINOFF_DINOFF2_Pos            (18)                                              /*!< GP_T::DINOFF: DINOFF2 Position            */
#define GP_DINOFF_DINOFF2_Msk            (0x1ul << GP_DINOFF_DINOFF2_Pos)                  /*!< GP_T::DINOFF: DINOFF2 Mask                */

#define GP_DINOFF_DINOFF3_Pos            (19)                                              /*!< GP_T::DINOFF: DINOFF3 Position            */
#define GP_DINOFF_DINOFF3_Msk            (0x1ul << GP_DINOFF_DINOFF3_Pos)                  /*!< GP_T::DINOFF: DINOFF3 Mask                */

#define GP_DINOFF_DINOFF4_Pos            (20)                                              /*!< GP_T::DINOFF: DINOFF4 Position            */
#define GP_DINOFF_DINOFF4_Msk            (0x1ul << GP_DINOFF_DINOFF4_Pos)                  /*!< GP_T::DINOFF: DINOFF4 Mask                */

#define GP_DINOFF_DINOFF5_Pos            (21)                                              /*!< GP_T::DINOFF: DINOFF5 Position            */
#define GP_DINOFF_DINOFF5_Msk            (0x1ul << GP_DINOFF_DINOFF5_Pos)                  /*!< GP_T::DINOFF: DINOFF5 Mask                */

#define GP_DINOFF_DINOFF6_Pos            (22)                                              /*!< GP_T::DINOFF: DINOFF6 Position            */
#define GP_DINOFF_DINOFF6_Msk            (0x1ul << GP_DINOFF_DINOFF6_Pos)                  /*!< GP_T::DINOFF: DINOFF6 Mask                */

#define GP_DINOFF_DINOFF7_Pos            (23)                                              /*!< GP_T::DINOFF: DINOFF7 Position            */
#define GP_DINOFF_DINOFF7_Msk            (0x1ul << GP_DINOFF_DINOFF7_Pos)                  /*!< GP_T::DINOFF: DINOFF7 Mask                */

#define GP_DOUT_DOUT0_Pos                (0)                                               /*!< GP_T::DOUT: DOUT0 Position                */
#define GP_DOUT_DOUT0_Msk                (0x1ul << GP_DOUT_DOUT0_Pos)                      /*!< GP_T::DOUT: DOUT0 Mask                    */

#define GP_DOUT_DOUT1_Pos                (1)                                               /*!< GP_T::DOUT: DOUT1 Position                */
#define GP_DOUT_DOUT1_Msk                (0x1ul << GP_DOUT_DOUT1_Pos)                      /*!< GP_T::DOUT: DOUT1 Mask                    */

#define GP_DOUT_DOUT2_Pos                (2)                                               /*!< GP_T::DOUT: DOUT2 Position                */
#define GP_DOUT_DOUT2_Msk                (0x1ul << GP_DOUT_DOUT2_Pos)                      /*!< GP_T::DOUT: DOUT2 Mask                    */

#define GP_DOUT_DOUT3_Pos                (3)                                               /*!< GP_T::DOUT: DOUT3 Position                */
#define GP_DOUT_DOUT3_Msk                (0x1ul << GP_DOUT_DOUT3_Pos)                      /*!< GP_T::DOUT: DOUT3 Mask                    */

#define GP_DOUT_DOUT4_Pos                (4)                                               /*!< GP_T::DOUT: DOUT4 Position                */
#define GP_DOUT_DOUT4_Msk                (0x1ul << GP_DOUT_DOUT4_Pos)                      /*!< GP_T::DOUT: DOUT4 Mask                    */

#define GP_DOUT_DOUT5_Pos                (5)                                               /*!< GP_T::DOUT: DOUT5 Position                */
#define GP_DOUT_DOUT5_Msk                (0x1ul << GP_DOUT_DOUT5_Pos)                      /*!< GP_T::DOUT: DOUT5 Mask                    */

#define GP_DOUT_DOUT6_Pos                (6)                                               /*!< GP_T::DOUT: DOUT6 Position                */
#define GP_DOUT_DOUT6_Msk                (0x1ul << GP_DOUT_DOUT6_Pos)                      /*!< GP_T::DOUT: DOUT6 Mask                    */

#define GP_DOUT_DOUT7_Pos                (7)                                               /*!< GP_T::DOUT: DOUT7 Position                */
#define GP_DOUT_DOUT7_Msk                (0x1ul << GP_DOUT_DOUT7_Pos)                      /*!< GP_T::DOUT: DOUT7 Mask                    */

#define GP_DATMSK_DATMSK0_Pos            (0)                                               /*!< GP_T::DATMSK: DATMSK0 Position            */
#define GP_DATMSK_DATMSK0_Msk            (0x1ul << GP_DATMSK_DATMSK0_Pos)                  /*!< GP_T::DATMSK: DATMSK0 Mask                */

#define GP_DATMSK_DATMSK1_Pos            (1)                                               /*!< GP_T::DATMSK: DATMSK1 Position            */
#define GP_DATMSK_DATMSK1_Msk            (0x1ul << GP_DATMSK_DATMSK1_Pos)                  /*!< GP_T::DATMSK: DATMSK1 Mask                */

#define GP_DATMSK_DATMSK2_Pos            (2)                                               /*!< GP_T::DATMSK: DATMSK2 Position            */
#define GP_DATMSK_DATMSK2_Msk            (0x1ul << GP_DATMSK_DATMSK2_Pos)                  /*!< GP_T::DATMSK: DATMSK2 Mask                */

#define GP_DATMSK_DATMSK3_Pos            (3)                                               /*!< GP_T::DATMSK: DATMSK3 Position            */
#define GP_DATMSK_DATMSK3_Msk            (0x1ul << GP_DATMSK_DATMSK3_Pos)                  /*!< GP_T::DATMSK: DATMSK3 Mask                */

#define GP_DATMSK_DATMSK4_Pos            (4)                                               /*!< GP_T::DATMSK: DATMSK4 Position            */
#define GP_DATMSK_DATMSK4_Msk            (0x1ul << GP_DATMSK_DATMSK4_Pos)                  /*!< GP_T::DATMSK: DATMSK4 Mask                */

#define GP_DATMSK_DATMSK5_Pos            (5)                                               /*!< GP_T::DATMSK: DATMSK5 Position            */
#define GP_DATMSK_DATMSK5_Msk            (0x1ul << GP_DATMSK_DATMSK5_Pos)                  /*!< GP_T::DATMSK: DATMSK5 Mask                */

#define GP_DATMSK_DATMSK6_Pos            (6)                                               /*!< GP_T::DATMSK: DATMSK6 Position            */
#define GP_DATMSK_DATMSK6_Msk            (0x1ul << GP_DATMSK_DATMSK6_Pos)                  /*!< GP_T::DATMSK: DATMSK6 Mask                */

#define GP_DATMSK_DATMSK7_Pos            (7)                                               /*!< GP_T::DATMSK: DATMSK7 Position            */
#define GP_DATMSK_DATMSK7_Msk            (0x1ul << GP_DATMSK_DATMSK7_Pos)                  /*!< GP_T::DATMSK: DATMSK7 Mask                */

#define GP_PIN_PIN0_Pos                  (0)                                               /*!< GP_T::PIN: PIN0 Position                  */
#define GP_PIN_PIN0_Msk                  (0x1ul << GP_PIN_PIN0_Pos)                        /*!< GP_T::PIN: PIN0 Mask                      */

#define GP_PIN_PIN1_Pos                  (1)                                               /*!< GP_T::PIN: PIN1 Position                  */
#define GP_PIN_PIN1_Msk                  (0x1ul << GP_PIN_PIN1_Pos)                        /*!< GP_T::PIN: PIN1 Mask                      */

#define GP_PIN_PIN2_Pos                  (2)                                               /*!< GP_T::PIN: PIN2 Position                  */
#define GP_PIN_PIN2_Msk                  (0x1ul << GP_PIN_PIN2_Pos)                        /*!< GP_T::PIN: PIN2 Mask                      */

#define GP_PIN_PIN3_Pos                  (3)                                               /*!< GP_T::PIN: PIN3 Position                  */
#define GP_PIN_PIN3_Msk                  (0x1ul << GP_PIN_PIN3_Pos)                        /*!< GP_T::PIN: PIN3 Mask                      */

#define GP_PIN_PIN4_Pos                  (4)                                               /*!< GP_T::PIN: PIN4 Position                  */
#define GP_PIN_PIN4_Msk                  (0x1ul << GP_PIN_PIN4_Pos)                        /*!< GP_T::PIN: PIN4 Mask                      */

#define GP_PIN_PIN5_Pos                  (5)                                               /*!< GP_T::PIN: PIN5 Position                  */
#define GP_PIN_PIN5_Msk                  (0x1ul << GP_PIN_PIN5_Pos)                        /*!< GP_T::PIN: PIN5 Mask                      */

#define GP_PIN_PIN6_Pos                  (6)                                               /*!< GP_T::PIN: PIN6 Position                  */
#define GP_PIN_PIN6_Msk                  (0x1ul << GP_PIN_PIN6_Pos)                        /*!< GP_T::PIN: PIN6 Mask                      */

#define GP_PIN_PIN7_Pos                  (7)                                               /*!< GP_T::PIN: PIN7 Position                  */
#define GP_PIN_PIN7_Msk                  (0x1ul << GP_PIN_PIN7_Pos)                        /*!< GP_T::PIN: PIN7 Mask                      */

#define GP_DBEN_DBEN0_Pos                (0)                                               /*!< GP_T::DBEN: DBEN0 Position                */
#define GP_DBEN_DBEN0_Msk                (0x1ul << GP_DBEN_DBEN0_Pos)                      /*!< GP_T::DBEN: DBEN0 Mask                    */

#define GP_DBEN_DBEN1_Pos                (1)                                               /*!< GP_T::DBEN: DBEN1 Position                */
#define GP_DBEN_DBEN1_Msk                (0x1ul << GP_DBEN_DBEN1_Pos)                      /*!< GP_T::DBEN: DBEN1 Mask                    */

#define GP_DBEN_DBEN2_Pos                (2)                                               /*!< GP_T::DBEN: DBEN2 Position                */
#define GP_DBEN_DBEN2_Msk                (0x1ul << GP_DBEN_DBEN2_Pos)                      /*!< GP_T::DBEN: DBEN2 Mask                    */

#define GP_DBEN_DBEN3_Pos                (3)                                               /*!< GP_T::DBEN: DBEN3 Position                */
#define GP_DBEN_DBEN3_Msk                (0x1ul << GP_DBEN_DBEN3_Pos)                      /*!< GP_T::DBEN: DBEN3 Mask                    */

#define GP_DBEN_DBEN4_Pos                (4)                                               /*!< GP_T::DBEN: DBEN4 Position                */
#define GP_DBEN_DBEN4_Msk                (0x1ul << GP_DBEN_DBEN4_Pos)                      /*!< GP_T::DBEN: DBEN4 Mask                    */

#define GP_DBEN_DBEN5_Pos                (5)                                               /*!< GP_T::DBEN: DBEN5 Position                */
#define GP_DBEN_DBEN5_Msk                (0x1ul << GP_DBEN_DBEN5_Pos)                      /*!< GP_T::DBEN: DBEN5 Mask                    */

#define GP_DBEN_DBEN6_Pos                (6)                                               /*!< GP_T::DBEN: DBEN6 Position                */
#define GP_DBEN_DBEN6_Msk                (0x1ul << GP_DBEN_DBEN6_Pos)                      /*!< GP_T::DBEN: DBEN6 Mask                    */

#define GP_DBEN_DBEN7_Pos                (7)                                               /*!< GP_T::DBEN: DBEN7 Position                */
#define GP_DBEN_DBEN7_Msk                (0x1ul << GP_DBEN_DBEN7_Pos)                      /*!< GP_T::DBEN: DBEN7 Mask                    */

#define GP_INTTYPE_TYPE0_Pos             (0)                                               /*!< GP_T::INTTYPE: TYPE0 Position             */
#define GP_INTTYPE_TYPE0_Msk             (0x1ul << GP_INTTYPE_TYPE0_Pos)                   /*!< GP_T::INTTYPE: TYPE0 Mask                 */

#define GP_INTTYPE_TYPE1_Pos             (1)                                               /*!< GP_T::INTTYPE: TYPE1 Position             */
#define GP_INTTYPE_TYPE1_Msk             (0x1ul << GP_INTTYPE_TYPE1_Pos)                   /*!< GP_T::INTTYPE: TYPE1 Mask                 */

#define GP_INTTYPE_TYPE2_Pos             (2)                                               /*!< GP_T::INTTYPE: TYPE2 Position             */
#define GP_INTTYPE_TYPE2_Msk             (0x1ul << GP_INTTYPE_TYPE2_Pos)                   /*!< GP_T::INTTYPE: TYPE2 Mask                 */

#define GP_INTTYPE_TYPE3_Pos             (3)                                               /*!< GP_T::INTTYPE: TYPE3 Position             */
#define GP_INTTYPE_TYPE3_Msk             (0x1ul << GP_INTTYPE_TYPE3_Pos)                   /*!< GP_T::INTTYPE: TYPE3 Mask                 */

#define GP_INTTYPE_TYPE4_Pos             (4)                                               /*!< GP_T::INTTYPE: TYPE4 Position             */
#define GP_INTTYPE_TYPE4_Msk             (0x1ul << GP_INTTYPE_TYPE4_Pos)                   /*!< GP_T::INTTYPE: TYPE4 Mask                 */

#define GP_INTTYPE_TYPE5_Pos             (5)                                               /*!< GP_T::INTTYPE: TYPE5 Position             */
#define GP_INTTYPE_TYPE5_Msk             (0x1ul << GP_INTTYPE_TYPE5_Pos)                   /*!< GP_T::INTTYPE: TYPE5 Mask                 */

#define GP_INTTYPE_TYPE6_Pos             (6)                                               /*!< GP_T::INTTYPE: TYPE6 Position             */
#define GP_INTTYPE_TYPE6_Msk             (0x1ul << GP_INTTYPE_TYPE6_Pos)                   /*!< GP_T::INTTYPE: TYPE6 Mask                 */

#define GP_INTTYPE_TYPE7_Pos             (7)                                               /*!< GP_T::INTTYPE: TYPE7 Position             */
#define GP_INTTYPE_TYPE7_Msk             (0x1ul << GP_INTTYPE_TYPE7_Pos)                   /*!< GP_T::INTTYPE: TYPE7 Mask                 */

#define GP_INTEN_FLIEN0_Pos              (0)                                               /*!< GP_T::INTEN: FLIEN0 Position              */
#define GP_INTEN_FLIEN0_Msk              (0x1ul << GP_INTEN_FLIEN0_Pos)                    /*!< GP_T::INTEN: FLIEN0 Mask                  */

#define GP_INTEN_FLIEN1_Pos              (1)                                               /*!< GP_T::INTEN: FLIEN1 Position              */
#define GP_INTEN_FLIEN1_Msk              (0x1ul << GP_INTEN_FLIEN1_Pos)                    /*!< GP_T::INTEN: FLIEN1 Mask                  */

#define GP_INTEN_FLIEN2_Pos              (2)                                               /*!< GP_T::INTEN: FLIEN2 Position              */
#define GP_INTEN_FLIEN2_Msk              (0x1ul << GP_INTEN_FLIEN2_Pos)                    /*!< GP_T::INTEN: FLIEN2 Mask                  */

#define GP_INTEN_FLIEN3_Pos              (3)                                               /*!< GP_T::INTEN: FLIEN3 Position              */
#define GP_INTEN_FLIEN3_Msk              (0x1ul << GP_INTEN_FLIEN3_Pos)                    /*!< GP_T::INTEN: FLIEN3 Mask                  */

#define GP_INTEN_FLIEN4_Pos              (4)                                               /*!< GP_T::INTEN: FLIEN4 Position              */
#define GP_INTEN_FLIEN4_Msk              (0x1ul << GP_INTEN_FLIEN4_Pos)                    /*!< GP_T::INTEN: FLIEN4 Mask                  */

#define GP_INTEN_FLIEN5_Pos              (5)                                               /*!< GP_T::INTEN: FLIEN5 Position              */
#define GP_INTEN_FLIEN5_Msk              (0x1ul << GP_INTEN_FLIEN5_Pos)                    /*!< GP_T::INTEN: FLIEN5 Mask                  */

#define GP_INTEN_FLIEN6_Pos              (6)                                               /*!< GP_T::INTEN: FLIEN6 Position              */
#define GP_INTEN_FLIEN6_Msk              (0x1ul << GP_INTEN_FLIEN6_Pos)                    /*!< GP_T::INTEN: FLIEN6 Mask                  */

#define GP_INTEN_FLIEN7_Pos              (7)                                               /*!< GP_T::INTEN: FLIEN7 Position              */
#define GP_INTEN_FLIEN7_Msk              (0x1ul << GP_INTEN_FLIEN7_Pos)                    /*!< GP_T::INTEN: FLIEN7 Mask                  */

#define GP_INTEN_RHIEN0_Pos              (16)                                              /*!< GP_T::INTEN: RHIEN0 Position              */
#define GP_INTEN_RHIEN0_Msk              (0x1ul << GP_INTEN_RHIEN0_Pos)                    /*!< GP_T::INTEN: RHIEN0 Mask                  */

#define GP_INTEN_RHIEN1_Pos              (17)                                              /*!< GP_T::INTEN: RHIEN1 Position              */
#define GP_INTEN_RHIEN1_Msk              (0x1ul << GP_INTEN_RHIEN1_Pos)                    /*!< GP_T::INTEN: RHIEN1 Mask                  */

#define GP_INTEN_RHIEN2_Pos              (18)                                              /*!< GP_T::INTEN: RHIEN2 Position              */
#define GP_INTEN_RHIEN2_Msk              (0x1ul << GP_INTEN_RHIEN2_Pos)                    /*!< GP_T::INTEN: RHIEN2 Mask                  */

#define GP_INTEN_RHIEN3_Pos              (19)                                              /*!< GP_T::INTEN: RHIEN3 Position              */
#define GP_INTEN_RHIEN3_Msk              (0x1ul << GP_INTEN_RHIEN3_Pos)                    /*!< GP_T::INTEN: RHIEN3 Mask                  */

#define GP_INTEN_RHIEN4_Pos              (20)                                              /*!< GP_T::INTEN: RHIEN4 Position              */
#define GP_INTEN_RHIEN4_Msk              (0x1ul << GP_INTEN_RHIEN4_Pos)                    /*!< GP_T::INTEN: RHIEN4 Mask                  */

#define GP_INTEN_RHIEN5_Pos              (21)                                              /*!< GP_T::INTEN: RHIEN5 Position              */
#define GP_INTEN_RHIEN5_Msk              (0x1ul << GP_INTEN_RHIEN5_Pos)                    /*!< GP_T::INTEN: RHIEN5 Mask                  */

#define GP_INTEN_RHIEN6_Pos              (22)                                              /*!< GP_T::INTEN: RHIEN6 Position              */
#define GP_INTEN_RHIEN6_Msk              (0x1ul << GP_INTEN_RHIEN6_Pos)                    /*!< GP_T::INTEN: RHIEN6 Mask                  */

#define GP_INTEN_RHIEN7_Pos              (23)                                              /*!< GP_T::INTEN: RHIEN7 Position              */
#define GP_INTEN_RHIEN7_Msk              (0x1ul << GP_INTEN_RHIEN7_Pos)                    /*!< GP_T::INTEN: RHIEN7 Mask                  */

#define GP_INTSRC_INTSRC0_Pos            (0)                                               /*!< GP_T::INTSRC: INTSRC0 Position            */
#define GP_INTSRC_INTSRC0_Msk            (0x1ul << GP_INTSRC_INTSRC0_Pos)                  /*!< GP_T::INTSRC: INTSRC0 Mask                */

#define GP_INTSRC_INTSRC1_Pos            (1)                                               /*!< GP_T::INTSRC: INTSRC1 Position            */
#define GP_INTSRC_INTSRC1_Msk            (0x1ul << GP_INTSRC_INTSRC1_Pos)                  /*!< GP_T::INTSRC: INTSRC1 Mask                */

#define GP_INTSRC_INTSRC2_Pos            (2)                                               /*!< GP_T::INTSRC: INTSRC2 Position            */
#define GP_INTSRC_INTSRC2_Msk            (0x1ul << GP_INTSRC_INTSRC2_Pos)                  /*!< GP_T::INTSRC: INTSRC2 Mask                */

#define GP_INTSRC_INTSRC3_Pos            (3)                                               /*!< GP_T::INTSRC: INTSRC3 Position            */
#define GP_INTSRC_INTSRC3_Msk            (0x1ul << GP_INTSRC_INTSRC3_Pos)                  /*!< GP_T::INTSRC: INTSRC3 Mask                */

#define GP_INTSRC_INTSRC4_Pos            (4)                                               /*!< GP_T::INTSRC: INTSRC4 Position            */
#define GP_INTSRC_INTSRC4_Msk            (0x1ul << GP_INTSRC_INTSRC4_Pos)                  /*!< GP_T::INTSRC: INTSRC4 Mask                */

#define GP_INTSRC_INTSRC5_Pos            (5)                                               /*!< GP_T::INTSRC: INTSRC5 Position            */
#define GP_INTSRC_INTSRC5_Msk            (0x1ul << GP_INTSRC_INTSRC5_Pos)                  /*!< GP_T::INTSRC: INTSRC5 Mask                */

#define GP_INTSRC_INTSRC6_Pos            (6)                                               /*!< GP_T::INTSRC: INTSRC6 Position            */
#define GP_INTSRC_INTSRC6_Msk            (0x1ul << GP_INTSRC_INTSRC6_Pos)                  /*!< GP_T::INTSRC: INTSRC6 Mask                */

#define GP_INTSRC_INTSRC7_Pos            (7)                                               /*!< GP_T::INTSRC: INTSRC7 Position            */
#define GP_INTSRC_INTSRC7_Msk            (0x1ul << GP_INTSRC_INTSRC7_Pos)                  /*!< GP_T::INTSRC: INTSRC7 Mask                */

#define GP_INTSRC_INTSRC8_Pos            (8)                                               /*!< GP_T::INTSRC: INTSRC8 Position            */
#define GP_INTSRC_INTSRC8_Msk            (0x1ul << GP_INTSRC_INTSRC8_Pos)                  /*!< GP_T::INTSRC: INTSRC8 Mask                */

#define GP_INTSRC_INTSRC9_Pos            (9)                                               /*!< GP_T::INTSRC: INTSRC9 Position            */
#define GP_INTSRC_INTSRC9_Msk            (0x1ul << GP_INTSRC_INTSRC9_Pos)                  /*!< GP_T::INTSRC: INTSRC9 Mask                */

#define GP_INTSRC_INTSRC10_Pos           (10)                                              /*!< GP_T::INTSRC: INTSRC10 Position           */
#define GP_INTSRC_INTSRC10_Msk           (0x1ul << GP_INTSRC_INTSRC10_Pos)                 /*!< GP_T::INTSRC: INTSRC10 Mask               */

#define GP_INTSRC_INTSRC11_Pos           (11)                                              /*!< GP_T::INTSRC: INTSRC11 Position           */
#define GP_INTSRC_INTSRC11_Msk           (0x1ul << GP_INTSRC_INTSRC11_Pos)                 /*!< GP_T::INTSRC: INTSRC11 Mask               */

#define GP_INTSRC_INTSRC12_Pos           (12)                                              /*!< GP_T::INTSRC: INTSRC12 Position           */
#define GP_INTSRC_INTSRC12_Msk           (0x1ul << GP_INTSRC_INTSRC12_Pos)                 /*!< GP_T::INTSRC: INTSRC12 Mask               */

#define GP_INTSRC_INTSRC13_Pos           (13)                                              /*!< GP_T::INTSRC: INTSRC13 Position           */
#define GP_INTSRC_INTSRC13_Msk           (0x1ul << GP_INTSRC_INTSRC13_Pos)                 /*!< GP_T::INTSRC: INTSRC13 Mask               */

#define GP_INTSRC_INTSRC14_Pos           (14)                                              /*!< GP_T::INTSRC: INTSRC14 Position           */
#define GP_INTSRC_INTSRC14_Msk           (0x1ul << GP_INTSRC_INTSRC14_Pos)                 /*!< GP_T::INTSRC: INTSRC14 Mask               */

#define GP_INTSRC_INTSRC15_Pos           (15)                                              /*!< GP_T::INTSRC: INTSRC15 Position           */
#define GP_INTSRC_INTSRC15_Msk           (0x1ul << GP_INTSRC_INTSRC15_Pos)                 /*!< GP_T::INTSRC: INTSRC15 Mask               */

#define GP_DBCTL_DBCLKSEL_Pos            (0)                                               /*!< GP_T::DBCTL: DBCLKSEL Position            */
#define GP_DBCTL_DBCLKSEL_Msk            (0xful << GP_DBCTL_DBCLKSEL_Pos)                  /*!< GP_T::DBCTL: DBCLKSEL Mask                */

#define GP_DBCTL_DBCLKSRC_Pos            (4)                                               /*!< GP_T::DBCTL: DBCLKSRC Position            */
#define GP_DBCTL_DBCLKSRC_Msk            (0x1ul << GP_DBCTL_DBCLKSRC_Pos)                  /*!< GP_T::DBCTL: DBCLKSRC Mask                */

#define GP_DBCTL_ICLKON_Pos              (5)                                               /*!< GP_T::DBCTL: ICLKON Position              */
#define GP_DBCTL_ICLKON_Msk              (0x1ul << GP_DBCTL_ICLKON_Pos)                    /*!< GP_T::DBCTL: ICLKON Mask                  */

/**@}*/ /* GP_CONST */
/**@}*/ /* end of GP register group */


/*---------------------- Inter-IC Bus Controller -------------------------*/
/**
    @addtogroup I2C Inter-IC Bus Controller(I2C)
    Memory Mapped Structure for I2C Controller
@{ */

/**
  * @brief Inter-integrated Circuit Interface
  */
typedef struct
{
    __IO uint32_t CON               ;//(I2C_BASE + 0x000)
    __IO uint32_t TAR               ;//(I2C_BASE + 0x004)
    __IO uint32_t SAR               ;//(I2C_BASE + 0x008)
    __IO uint32_t HS_MADDR          ;//(I2C_BASE + 0x00C)
    __IO uint32_t DATACMD           ;//(I2C_BASE + 0x010)
    __IO uint32_t SS_SCL_HCNT       ;//(I2C_BASE + 0x014)
    __IO uint32_t SS_SCL_LCNT       ;//(I2C_BASE + 0x018)
    __IO uint32_t FS_SCL_HCNT       ;//(I2C_BASE + 0x01C)
    __IO uint32_t FS_SCL_LCNT       ;//(I2C_BASE + 0x020)
    __IO uint32_t HS_SCL_HCNT       ;//(I2C_BASE + 0x024)
    __IO uint32_t HS_SCL_LCNT       ;//(I2C_BASE + 0x028)
    __IO uint32_t INTR_STAT         ;//(I2C_BASE + 0x02C)
    __IO uint32_t INTR_MASK         ;//(I2C_BASE + 0x030)
    __IO uint32_t RAW_INTR_STAT     ;//(I2C_BASE + 0x034)
    __IO uint32_t RX_TL             ;//(I2C_BASE + 0x038)
    __IO uint32_t TX_TL             ;//(I2C_BASE + 0x03C)
    __IO uint32_t CLR_INTR          ;//(I2C_BASE + 0x040)
    __IO uint32_t CLR_RX_UND        ;//(I2C_BASE + 0x044)
    __IO uint32_t CLR_RX_OVR        ;//(I2C_BASE + 0x048)
    __IO uint32_t CLR_TX_OVR        ;//(I2C_BASE + 0x04C)
    __IO uint32_t CLR_RD_REQ        ;//(I2C_BASE + 0x050)
    __IO uint32_t CLR_TX_ABRT       ;//(I2C_BASE + 0x054)
    __IO uint32_t CLR_RX_DONE       ;//(I2C_BASE + 0x058)
    __IO uint32_t CLR_ACTIVITY      ;//(I2C_BASE + 0x05C)
    __IO uint32_t CLR_STOP_DET      ;//(I2C_BASE + 0x060)
    __IO uint32_t CLR_START_DET     ;//(I2C_BASE + 0x064)
    __IO uint32_t CLR_GEN_CALL      ;//(I2C_BASE + 0x068)
    __IO uint32_t IC_ENABLE            ;//(I2C_BASE + 0x06C)
    __IO uint32_t STATUS            ;//(I2C_BASE + 0x070)
    __IO uint32_t TXFLR             ;//(I2C_BASE + 0x074)
    __IO uint32_t RXFLR             ;//(I2C_BASE + 0x078)
    __IO uint32_t SDA_HOLD          ;//(I2C_BASE + 0x07C)
    __IO uint32_t TX_ABRT_SRC       ;//(I2C_BASE + 0x080)
    __IO uint32_t SLV_DATA_NACK     ;//(I2C_BASE + 0x084)
    __IO uint32_t DMA_CR            ;//(I2C_BASE + 0x088)
    __IO uint32_t DMA_TDLR          ;//(I2C_BASE + 0x08C)
    __IO uint32_t DMA_RDLR          ;//(I2C_BASE + 0x090)
    __IO uint32_t SDA_SETUP         ;//(I2C_BASE + 0x094)
    __IO uint32_t ACK_GENERAL_CALL  ;//(I2C_BASE + 0x098)
    __IO uint32_t ENABLE_STATUS     ;//(I2C_BASE + 0x09C)
    __IO uint32_t FS_SPKLEN         ;//(I2C_BASE + 0x0A0)
    __IO uint32_t HS_SPKLEN         ;//(I2C_BASE + 0x0A4)
	__IO uint32_t CLR_RESTART_DET         ;//(I2C_BASE + 0x0A8)

}I2C_T;

/******************************************************************************/
/*                                                                            */
/*                      Inter-integrated Circuit Interface                    */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for I2C_CON register  ********************/
#define  I2C_CON_MASTER_MODE                            ((uint16_t)0x0001)            /*!<Master mode                             */

#define  I2C_CON_SPEED                       		    ((uint16_t)0x0006)            /*!<Speed                                   */
#define  I2C_CON_SPEED_0								((uint16_t)0x0002)
#define  I2C_CON_SPEED_1								((uint16_t)0x0004)

#define  I2C_CON_IC_10BITADDR_SLAVE                     ((uint16_t)0x0008)
#define  I2C_CON_IC_10BITADDR_MASTER                    ((uint16_t)0x0010)
#define  I2C_CON_IC_RESTART_EN                          ((uint16_t)0x0020)
#define  I2C_CON_IC_SLAVE_DISABLE                       ((uint16_t)0x0040)
#define  I2C_CON_STOP_DET_IFADDRESSED                   ((uint16_t)0x0080)
#define  I2C_CON_TX_EMPTY_CTRL                          ((uint16_t)0x0100)

/*******************  Bit definition for I2C_TAR register  ********************/
#define I2C_TAR_TAR                                    ((uint16_t)0x03ff)
#define I2C_TAR_IC_TAR_0                                  ((uint16_t)0x0001)
#define I2C_TAR_IC_TAR_1                                  ((uint16_t)0x0002)
#define I2C_TAR_IC_TAR_2                                  ((uint16_t)0x0004)
#define I2C_TAR_IC_TAR_3                                  ((uint16_t)0x0008)
#define I2C_TAR_IC_TAR_4                                  ((uint16_t)0x0010)
#define I2C_TAR_IC_TAR_5                                  ((uint16_t)0x0020)
#define I2C_TAR_IC_TAR_6                                  ((uint16_t)0x0040)
#define I2C_TAR_IC_TAR_7                                  ((uint16_t)0x0080)
#define I2C_TAR_IC_TAR_8                                  ((uint16_t)0x0100)
#define I2C_TAR_IC_TAR_9                                  ((uint16_t)0x0200)

#define I2C_TAR_GC_OR_START                               ((uint16_t)0x0400)
#define I2C_TAR_SPECIAL                                   ((uint16_t)0x0800)
#define I2C_TAR_IC_10BITADDR_MASTER                       ((uint16_t)0x1000)

/*******************  Bit definition for I2C_SAR register  ********************/
#define I2C_SAR_IC_SAR                                    ((uint16_t)0x03ff)
/*******************  Bit definition for I2C_HS_MADDR register  ********************/
#define I2C_HS_MADDR_IC_HS_MAR                            ((uint16_t)0x0007)
/*******************  Bit definition for I2C_DATA_CMD register  ********************/
#define I2C_DATA_CMD_DAT                                  ((uint16_t)0x00ff)
#define I2C_DATA_CMD_CMD_R                                  ((uint16_t)0x0100)
#define I2C_DATA_CMD_STOP                                 ((uint16_t)0x0200)
#define I2C_DATA_CMD_RESTART                              ((uint16_t)0x0400)

/*******************  Bit definition for I2C_SS_SCL_HCNT register  ********************/
#define I2C_SS_SCL_HCNT                                   ((uint16_t)0xffff)

/*******************  Bit definition for I2C_SS_SCL_LCNT register  ********************/
#define I2C_SS_SCL_LCNT                                   ((uint16_t)0xffff)

/*******************  Bit definition for I2C_FS_SCL_HCNT register  ********************/
#define I2C_FS_SCL_HCNT                                   ((uint16_t)0xffff)

/*******************  Bit definition for I2C_FS_SCL_LCNT register  ********************/
#define I2C_FS_SCL_LCNT                                   ((uint16_t)0xffff)

/*******************  Bit definition for I2C_HS_SCL_HCNT register  ********************/
#define I2C_HS_SCL_HCNT                                   ((uint16_t)0xffff)

/*******************  Bit definition for I2C_HS_SCL_LCNT register  ********************/
#define I2C_HS_SCL_LCNT                                   ((uint16_t)0xffff)

/*******************  Bit definition for I2C_INTR_STAT register  ********************/
#define I2C_INTR_STAT_R_RX_UNDER                                  ((uint16_t)0x0001)
#define I2C_INTR_STAT_R_RX_OVER                                   ((uint16_t)0x0002)
#define I2C_INTR_STAT_R_RX_FULL                                   ((uint16_t)0x0004)
#define I2C_INTR_STAT_R_TX_OVER                                   ((uint16_t)0x0008)
#define I2C_INTR_STAT_R_TX_EMPTY                                  ((uint16_t)0x0010)
#define I2C_INTR_STAT_R_RD_REQ                                    ((uint16_t)0x0020)
#define I2C_INTR_STAT_R_TX_ABRT                                   ((uint16_t)0x0040)
#define I2C_INTR_STAT_R_RX_DONE                                   ((uint16_t)0x0080)
#define I2C_INTR_STAT_R_ACTIVITY                                  ((uint16_t)0x0100)
#define I2C_INTR_STAT_R_STOP_DET                                  ((uint16_t)0x0200)
#define I2C_INTR_STAT_R_START_DET                                 ((uint16_t)0x0400)
#define I2C_INTR_STAT_R_GEN_CALL                                  ((uint16_t)0x0800)
#define I2C_INTR_STAT_R_RESTART_DET                               ((uint16_t)0x1000)
#define I2C_INTR_STAT_R_MST_ON_HOLD                               ((uint16_t)0x2000)


/*******************  Bit definition for I2C_INTR_MASK register  ********************/
#define I2C_INTR_MASK_M_RX_UNDER                                  ((uint16_t)0x0001)
#define I2C_INTR_MASK_M_RX_OVER                                   ((uint16_t)0x0002)
#define I2C_INTR_MASK_M_RX_FULL                                   ((uint16_t)0x0004)
#define I2C_INTR_MASK_M_TX_OVER                                   ((uint16_t)0x0008)
#define I2C_INTR_MASK_M_TX_EMPTY                                  ((uint16_t)0x0010)
#define I2C_INTR_MASK_M_RD_REQ                                    ((uint16_t)0x0020)
#define I2C_INTR_MASK_M_TX_ABRT                                   ((uint16_t)0x0040)
#define I2C_INTR_MASK_M_RX_DONE                                   ((uint16_t)0x0080)
#define I2C_INTR_MASK_M_ACTIVITY                                  ((uint16_t)0x0100)
#define I2C_INTR_MASK_M_STOP_DET                                  ((uint16_t)0x0200)
#define I2C_INTR_MASK_M_START_DET                                 ((uint16_t)0x0400)
#define I2C_INTR_MASK_M_GEN_CALL                                  ((uint16_t)0x0800)
#define I2C_INTR_MASK_M_RESTART_DET                               ((uint16_t)0x1000)
#define I2C_INTR_MASK_M_MST_ON_HOLD                               ((uint16_t)0x2000)

/*******************  Bit definition for I2C_RAW_INTR_STAT register  ********************/
#define I2C_RAW_INTR_STAT_RX_UNDER                              ((uint16_t)0x0001)
#define I2C_RAW_INTR_STAT_RX_OVER                               ((uint16_t)0x0002)
#define I2C_RAW_INTR_STAT_RX_FULL                               ((uint16_t)0x0004)
#define I2C_RAW_INTR_STAT_TX_OVER                               ((uint16_t)0x0008)
#define I2C_RAW_INTR_STAT_TX_EMPTY                              ((uint16_t)0x0010)
#define I2C_RAW_INTR_STAT_RD_REQ                                ((uint16_t)0x0020)
#define I2C_RAW_INTR_STAT_TX_ABRT                               ((uint16_t)0x0040)
#define I2C_RAW_INTR_STAT_RX_DONE                               ((uint16_t)0x0080)
#define I2C_RAW_INTR_STAT_ACTIVITY                              ((uint16_t)0x0100)
#define I2C_RAW_INTR_STAT_STOP_DET                              ((uint16_t)0x0200)
#define I2C_RAW_INTR_STAT_START_DET                             ((uint16_t)0x0400)
#define I2C_RAW_INTR_STAT_GEN_CALL                              ((uint16_t)0x0800)
#define I2C_RAW_INTR_STAT_RESTART_DET                           ((uint16_t)0x1000)
#define I2C_RAW_INTR_STAT_MST_ON_HOLD                           ((uint16_t)0x2000)

/*******************  Bit definition for I2C_RX_TL register  ********************/
#define I2C_RX_TL                                                 ((uint16_t)0x00ff)

/*******************  Bit definition for I2C_TX_TL register  ********************/
#define I2C_TX_TL                                                 ((uint16_t)0x00ff)

/*******************  Bit definition for I2C_CLR_INTR register  ********************/
#define I2C_CLR_INTR                                              ((uint16_t)0x0001)

/*******************  Bit definition for I2C_CLR_RX_UNDER register  ********************/
#define I2C_CLR_RX_UNDER                                          ((uint16_t)0x0001)

/*******************  Bit definition for I2C_CLR_RX_OVER register  ********************/
#define I2C_CLR_RX_UNDER                                          ((uint16_t)0x0001)

/*******************  Bit definition for I2C_CLR_TX_OVER register  ********************/
#define I2C_CLR_TX_OVER                                           ((uint16_t)0x0001)

/*******************  Bit definition for I2C_CLR_RD_REQ register  ********************/
#define I2C_CLR_RD_REQ                                            ((uint16_t)0x0001)

/*******************  Bit definition for I2C_CLR_TX_ABORT register  ********************/
#define I2C_CLR_TX_ABORT                                          ((uint16_t)0x0001)

/*******************  Bit definition for I2C_CLR_RX_DONE register  ********************/
#define I2C_CLR_RX_DONE                                           ((uint16_t)0x0001)

/*******************  Bit definition for I2C_CLR_ACTIVITY register  ********************/
#define I2C_CLR_ACTIVITY                                           ((uint16_t)0x0001)

/*******************  Bit definition for I2C_CLR_STOP_DET register  ********************/
#define I2C_CLR_STOP_DET                                           ((uint16_t)0x0001)

/*******************  Bit definition for I2C_CLR_START_DET register  ********************/
#define I2C_CLR_START_DET                                          ((uint16_t)0x0001)

/*******************  Bit definition for I2C_CLR_GEN_CALL register  ********************/
#define I2C_CLR_GEN_CALL                                           ((uint16_t)0x0001)
/*******************  Bit definition for I2C_CLR_RESTART_DET register  ********************/
#define I2C_CLR_RESTART_DET                                        ((uint16_t)0x0001)
/*******************  Bit definition for I2C_CLR_MST_ON_HOLD register  ********************/
#define I2C_CLR_MST_ON_HOLD                                        ((uint16_t)0x0001)


/*******************  Bit definition for I2C_ENABLE register  ********************/
#define I2C_ENABLE_ENABLE                                          ((uint16_t)0x0001)
#define I2C_ENABLE_ABORT                                           ((uint16_t)0x0002)
/*******************  Bit definition for I2C_STATUS register  ********************/
#define I2C_STATUS_ACTIVITY                                        ((uint16_t)0x0001)
#define I2C_STATUS_TFNF                                            ((uint16_t)0x0002)
#define I2C_STATUS_TFE                                             ((uint16_t)0x0004)
#define I2C_STATUS_RFNE                                            ((uint16_t)0x0008)
#define I2C_STATUS_RFE                                             ((uint16_t)0x0010)
#define I2C_STATUS_MST_ACTIVITY                                    ((uint16_t)0x0020)
#define I2C_STATUS_SLV_ACTIVITY                                    ((uint16_t)0x0040)

/*******************  Bit definition for I2C_TXFLR register  ********************/
#define I2C_TXFLR                                                  ((uint32_t)0x0000000f)

/*******************  Bit definition for I2C_RXFLR register  ********************/
#define I2C_RXFLR                                                  ((uint32_t)0x0000000f)

/*******************  Bit definition for I2C_SDA_HOLD register  ********************/
#define I2C_SDA_HOLD                                               ((uint32_t)0x0000ffff)

/*******************  Bit definition for I2C_TX_ABRT_SOURCE register  ********************/
#define I2C_TX_ABRT_SOURCE_7B_ADDR_NOACK                       ((uint32_t)0x00000001)
#define I2C_TX_ABRT_SOURCE_10ADDR1_NOACK                      ((uint32_t)0x00000002)
#define I2C_TX_ABRT_SOURCE_10ADDR2_NOACK                      ((uint32_t)0x00000004)
#define I2C_TX_ABRT_SOURCE_TXDATA_NOACK                       ((uint32_t)0x00000008)
#define I2C_TX_ABRT_SOURCE_GCALL_NOACK                        ((uint32_t)0x00000010)
#define I2C_TX_ABRT_SOURCE_GCALL_READ                         ((uint32_t)0x00000020)
#define I2C_TX_ABRT_SOURCE_HS_ACKDET                          ((uint32_t)0x00000040)
#define I2C_TX_ABRT_SOURCE_SBYTE_ACKDET                       ((uint32_t)0x00000080)
#define I2C_TX_ABRT_SOURCE_HS_NORSTRT                         ((uint32_t)0x00000100)
#define I2C_TX_ABRT_SOURCE_SBYTE_NORSTRT                      ((uint32_t)0x00000200)
#define I2C_TX_ABRT_SOURCE_10B_RD_NORSTRT                     ((uint32_t)0x00000400)
#define I2C_TX_ABRT_SOURCE_MASTER_DIS                         ((uint32_t)0x00000800)
#define I2C_TX_ABRT_SOURCE_ARB_LOST                           ((uint32_t)0x00001000)
#define I2C_TX_ABRT_SOURCE_SLVFLUSH_TXFIFO                    ((uint32_t)0x00002000)
#define I2C_TX_ABRT_SOURCE_SLV_ARBLOST                        ((uint32_t)0x00004000)
#define I2C_TX_ABRT_SOURCE_SLVRD_INTX                         ((uint32_t)0x00008000)
#define I2C_TX_ABRT_SOURCE_USER_ABRT                          ((uint32_t)0x00010000)

#define I2C_TX_ABRT_SOURCE_TX_FLUSH_CNT                            ((uint32_t)0xff000000)

/*******************  Bit definition for I2C_SLV_DATA_NACK_ONLY register  ********************/
#define I2C_SLV_DATA_NACK_ONLY                                     ((uint32_t)0x00000001)

/*******************  Bit definition for I2C_DMA_CR register  ********************/
#define I2C_DMA_CR_RDMAE                                           ((uint32_t)0x00000001)
#define I2C_DMA_CR_TDMAE                                           ((uint32_t)0x00000002)

/*******************  Bit definition for I2C_DMA_TDLR register  ********************/
#define I2C_DMA_TDLR                                               ((uint32_t)0x00000007)

/*******************  Bit definition for I2C_DMA_RDLR register  ********************/
#define I2C_DMA_RDLR                                               ((uint32_t)0x00000007)

/*******************  Bit definition for I2C_SDA_SETUP register  ********************/
#define I2C_SDA_SETUP                                              ((uint32_t)0x000000ff)

/*******************  Bit definition for I2C_ACK_GENERAL_CALL register  ********************/
#define I2C_ACK_GENERAL_CALL                                       ((uint32_t)0x00000001)

/*******************  Bit definition for I2C_ENABLE_STATUS register  ********************/
#define I2C_ENABLE_STATUS_IC_EN                                    ((uint32_t)0x00000001)
#define I2C_ENABLE_STATUS_SLV_RX_ABORTED                           ((uint32_t)0x00000002)
#define I2C_ENABLE_STATUS_SLV_FIFO_FILLED_AND_FLUSHED              ((uint32_t)0x00000004)

/*******************  Bit definition for I2C_FS_SPKLEN register  ********************/
#define I2C_FS_SPKLEN                                              ((uint32_t)0x0000000ff)

/*******************  Bit definition for I2C_HS_SPKLEN register  ********************/
#define I2C_HS_SPKLEN                                              ((uint32_t)0x0000000ff)

/*******************  Bit definition for I2C_COMP_PARAM1 register  ********************/
#define I2C_COMP_PARAM1_APB_DATA_WIDTH                             ((uint32_t)0x000000003)
#define I2C_COMP_PARAM1_MAX_SPEED_MODE                             ((uint32_t)0x000000006)
#define I2C_COMP_PARAM1_HC_COUNT_VALUES                            ((uint32_t)0x000000010)
#define I2C_COMP_PARAM1_INTR_IO                                    ((uint32_t)0x000000020)
#define I2C_COMP_PARAM1_HAS_DMA                                    ((uint32_t)0x000000040)
#define I2C_COMP_PARAM1_ADD_ENCODED_PARAMS                         ((uint32_t)0x000000080)
#define I2C_COMP_PARAM1_RX_BUFFER_DEPTH                            ((uint32_t)0x00000ff00)
#define I2C_COMP_PARAM1_TX_BUFFER_DEPTH                            ((uint32_t)0x000ff0000)

/*******************  Bit definition for I2C_COMP_VERSION register  ********************/
#define I2C_COMP_VERSION                                           ((uint32_t)0xffffffffff)

/*******************  Bit definition for I2C_COMP_TYPE register  ********************/
#define I2C_COMP_TYPE                                              ((uint32_t)0xffffffffff)


/*---------------------- INT Controller -------------------------*/
/**
    @addtogroup INT Interrupt Controller (INT)
    Memory Mapped Structure for INT Controller
@{ */

typedef struct {


    /**
     * IRQ0_SRC
     * ===================================================================================================
     * Offset: 0x00  IRQ0 (BOD) Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |BOD_INT   |IRQ0 Source Identity
     * |        |          |0 = IRQ0 source is not from BOD interrupt (BOD_INT).
     * |        |          |1 = IRQ0 source is from BOD interrupt (BOD_INT).
    */
    __I  uint32_t IRQ0_SRC;

    /**
     * IRQ1_SRC
     * ===================================================================================================
     * Offset: 0x04  IRQ1 (WDT) Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WDT_INT   |IRQ1 Source Identity
     * |        |          |0 = IRQ1 source is not from watchdog interrupt (WDT _INT).
     * |        |          |1 = IRQ1 source is from watchdog interrupt (WDT_INT).
     * |[1]     |WWDT_INT  |IRQ1 Source Identity
     * |        |          |0 = IRQ1 source is not from window watchdog interrupt (WWDT _INT).
     * |        |          |1 = IRQ1 source is from window watchdog interrupt (WWDT_INT).
    */
    __I  uint32_t IRQ1_SRC;

    /**
     * IRQ2_SRC
     * ===================================================================================================
     * Offset: 0x08  IRQ2 (EINT0) Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |EINT0     |IRQ2 Source Identity
     * |        |          |0 = IRQ2 source is not from external signal interrupt 0 from P3.2 (EINT0).
     * |        |          |1 = IRQ2 source is from external signal interrupt 0 from P3.2 (EINT0).
    */
    __I  uint32_t IRQ2_SRC;

    /**
     * IRQ3_SRC
     * ===================================================================================================
     * Offset: 0x0C  IRQ3 (EINT1) Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |EINT1     |IRQ3 Source Identity
     * |        |          |0 = IRQ3 source is not from external signal interrupt 1 from P5.2 (EINT1).
     * |        |          |1 = IRQ3 source is from external signal interrupt 1 from P5.2 (EINT1).
    */
    __I  uint32_t IRQ3_SRC;

    /**
     * IRQ4_SRC
     * ===================================================================================================
     * Offset: 0x10  IRQ4 (GP0/1) Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |GP0_INT   |IRQ4 Source Identity
     * |        |          |0 = IRQ4 source is not from GP0 interrupt (GP0_INT).
     * |        |          |1 = IRQ4 source is from GP0 interrupt (GP0_INT).
     * |[1]     |GP1_INT   |IRQ4 Source Identity
     * |        |          |0 = IRQ4 source is not from GP1 interrupt (GP1_INT).
     * |        |          |1 = IRQ4 source is from GP1 interrupt (GP1_INT).
    */
    __I  uint32_t IRQ4_SRC;

    /**
     * IRQ5_SRC
     * ===================================================================================================
     * Offset: 0x14  IRQ5 (GP2/3/4) Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |GP2_INT   |IRQ5 Source Identity
     * |        |          |0 = IRQ5 source is not from GP2 interrupt (GP2_INT).
     * |        |          |1 = IRQ5 source is from GP2 interrupt (GP2_INT).
     * |[1]     |GP3_INT   |IRQ5 Source Identity
     * |        |          |0 = IRQ5 source is not from GP3 interrupt (GP3_INT).
     * |        |          |1 = IRQ5 source is from GP3 interrupt (GP3_INT).
     * |[2]     |GP4_INT   |IRQ5 Source Identity
     * |        |          |0 = IRQ5 source is not from GP4 interrupt (GP4_INT).
     * |        |          |1 = IRQ5 source is from GP4 interrupt (GP4_INT).
    */
    __I  uint32_t IRQ5_SRC;

    /**
     * IRQ6_SRC
     * ===================================================================================================
     * Offset: 0x18  IRQ6 (PWM) Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PWM_INT   |IRQ6 Source Identity
     * |        |          |0 = IRQ6 source is not from PWM interrupt (PWM_INT).
     * |        |          |1 = IRQ6 source is from PWM interrupt (PWM_INT).
    */
    __I  uint32_t IRQ6_SRC;


     /// @cond HIDDEN_SYMBOLS
   __I   uint32_t RESERVE0[1];
    /// @endcond //HIDDEN_SYMBOLS

    /**
     * IRQ8_SRC
     * ===================================================================================================
     * Offset: 0x20  IRQ8 (TMR0) Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TMR0_INT  |IRQ8 Source Identity
     * |        |          |0 = IRQ8 source is not from Timer0 interrupt (TMR0_INT).
     * |        |          |1 = IRQ8 source is from Timer0 interrupt (TMR0_INT).
    */
    __I  uint32_t IRQ8_SRC;

    /**
     * IRQ9_SRC
     * ===================================================================================================
     * Offset: 0x24  IRQ9 (TMR1) Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TMR1_INT  |IRQ9 Source Identity
     * |        |          |0 = IRQ9 source is not from Timer1 interrupt (TMR1_INT).
     * |        |          |1 = IRQ9 source is from Timer1 interrupt (TMR1_INT).
    */
    __I  uint32_t IRQ9_SRC;


    /**
     * IRQ10_SRC
     * ===================================================================================================
     * Offset: 0x28  IRQ10 (TMR2) Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TMR2_INT  |IRQ10 Source Identity
     * |        |          |0 = IRQ10 source is not from Timer1 interrupt (TMR2_INT).
     * |        |          |1 = IRQ10 source is from Timer1 interrupt (TMR2_INT).
    */
    __I  uint32_t IRQ10_SRC;


    /**
     * IRQ11_SRC
     * ===================================================================================================
     * Offset: 0x2c  IRQ11 (EINT2) Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |EINT2_INT  |IRQ11 Source Identity
     * |        |          |0 = IRQ11 source is not from external signal interrupt 2 from P2.0 (EINT2).
     * |        |          |1 = IRQ11 source is from external signal interrupt 2 from P2.0 (EINT2).
    */
    __I  uint32_t IRQ11_SRC;



    /**
     * IRQ12_SRC
     * ===================================================================================================
     * Offset: 0x30  IRQ12 (UART0) Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |UART0_INT |IRQ12 Source Identity
     * |        |          |0 = IRQ12 source is not from UART0 interrupt (UART0_INT).
     * |        |          |1 = IRQ12 source is from UART0 interrupt (UART0_INT).
    */
    __I  uint32_t IRQ12_SRC;

    /**
     * IRQ13_SRC
     * ===================================================================================================
     * Offset: 0x34  IRQ13 (UART1) Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |UART1_INT |IRQ13 Source Identity
     * |        |          |0 = IRQ13 source is not from UART1 interrupt (UART1_INT).
     * |        |          |1 = IRQ13 source is from UART1 interrupt (UART1_INT).
    */
    __I  uint32_t IRQ13_SRC;

    /**
     * IRQ14_SRC
     * ===================================================================================================
     * Offset: 0x38  IRQ14 (SPI0) Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SPI0_INT   |IRQ14 Source Identity
     * |        |          |0 = IRQ14 source is not from SPI interrupt (SPI0_INT).
     * |        |          |1 = IRQ14 source is from SPI interrupt (SP0I_INT).
    */
    __I  uint32_t IRQ14_SRC;

 
 /**
  * IRQ15_SRC
  * ===================================================================================================
  * Offset: 0x38  IRQ15 (SPI1) Interrupt Source Identity
  * ---------------------------------------------------------------------------------------------------
  * |Bits    |Field     |Descriptions
  * | :----: | :----:   | :---- |
  * |[0]     |SPI1_INT   |IRQ15 Source Identity
  * |        |          |0 = IRQ15 source is not from SPI interrupt (SPI1_INT).
  * |        |          |1 = IRQ15 source is from SPI interrupt (SPI1_INT).
 */
 __I  uint32_t IRQ15_SRC;

    /**
     * IRQ16_SRC
     * ===================================================================================================
     * Offset: 0x40  IRQ16 (GP5) Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |GP5_INT   |IRQ16 Source Identity
     * |        |          |0 = IRQ16 source is not from GP5 interrupt (GP5_INT).
     * |        |          |1 = IRQ16 source is from GP5 interrupt (GP5_INT).
    */
    __I  uint32_t IRQ16_SRC;

     /// @cond HIDDEN_SYMBOLS
    uint32_t RESERVE1[1];
    /// @endcond //HIDDEN_SYMBOLS


    /**
     * IRQ18_SRC
     * ===================================================================================================
     * Offset: 0x48  IRQ18 (I2C0) Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |I2C0_INT  |IRQ18 Source Identity
     * |        |          |0 = IRQ18 source is not from I2C0 interrupt (I2C0_INT).
     * |        |          |1 = IRQ18 source is from I2C0 interrupt (I2C0_INT).
    */
    __I  uint32_t IRQ18_SRC;

    /**
     * IRQ19_SRC
     * ===================================================================================================
     * Offset: 0x4C  IRQ19 (I2C1) Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |I2C1_INT  |IRQ19 Source Identity
     * |        |          |0 = IRQ19 source is not from I2C1 interrupt (I2C1_INT).
     * |        |          |1 = IRQ19 source is from I2C1 interrupt (I2C1_INT).
    */
    __I  uint32_t IRQ19_SRC;

    /// @cond HIDDEN_SYMBOLS
    uint32_t RESERVE2[5];
    /// @endcond //HIDDEN_SYMBOLS

    /**
     * IRQ25_SRC
     * ===================================================================================================
     * Offset: 0x64  IRQ25 (ACMP) Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ACMP_INT  |IRQ25 Source Identity
     * |        |          |0 = IRQ25 source is not from ACMP interrupt (ACMP_INT).
     * |        |          |1 = IRQ25 source is from ACMP interrupt (ACMP_INT).
    */
    __I  uint32_t IRQ25_SRC;

    /// @cond HIDDEN_SYMBOLS
    uint32_t RESERVE3[2];
    /// @endcond //HIDDEN_SYMBOLS

    /**
     * IRQ28_SRC
     * ===================================================================================================
     * Offset: 0x70  IRQ28 (PWRWU) Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PWRWU_INT |IRQ28 Source Identity
     * |        |          |0 = IRQ28 source is not from PWRWU interrupt (PWRWU_INT).
     * |        |          |1 = IRQ28 source is from PWREU interrupt (PWRWU_INT).
    */
    __I  uint32_t IRQ28_SRC;

    /**
     * IRQ29_SRC
     * ===================================================================================================
     * Offset: 0x74  IRQ29 (ADC) Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ADC_INT   |IRQ29 Source Identity
     * |        |          |0 = IRQ29 source is not from ADC interrupt (ADC_INT).
     * |        |          |1 = IRQ29 source is from ADC interrupt (ADC_INT).
    */
    __I  uint32_t IRQ29_SRC;

} INTR_T;

/**
    @addtogroup INT_CONST INT Bit Field Definition
    Constant Definitions for INT Controller
@{ */

#define INT_IRQ0_SRC_BOD_INT_Pos                (0)                                               /*!< INT_T::IRQ0_SRC: BOD_INT Position         */
#define INT_IRQ0_SRC_BOD_INT_Msk                (0x1ul << INT_IRQ0_SRC_BOD_INT_Pos)               /*!< INT_T::IRQ0_SRC: BOD_INT Mask             */

#define INT_IRQ1_SRC_WDT_INT_Pos                (0)                                               /*!< INT_T::IRQ1_SRC: WDT_INT Position         */
#define INT_IRQ1_SRC_WDT_INT_Msk                (0x1ul << INT_IRQ1_SRC_WDT_INT_Pos)               /*!< INT_T::IRQ1_SRC: WDT_INT Mask             */

#define INT_IRQ1_SRC_WWDT_INT_Pos               (1)                                               /*!< INT_T::IRQ1_SRC: WWDT_INT Position        */
#define INT_IRQ1_SRC_WWDT_INT_Msk               (0x1ul << INT_IRQ1_SRC_WWDT_INT_Pos)              /*!< INT_T::IRQ1_SRC: WWDT_INT Mask            */

#define INT_IRQ2_SRC_EINT0_Pos                  (0)                                               /*!< INT_T::IRQ2_SRC: EINT0 Position           */
#define INT_IRQ2_SRC_EINT0_Msk                  (0x1ul << INT_IRQ2_SRC_EINT0_Pos)                 /*!< INT_T::IRQ2_SRC: EINT0 Mask               */

#define INT_IRQ3_SRC_EINT1_Pos                  (0)                                               /*!< INT_T::IRQ3_SRC: EINT1 Position           */
#define INT_IRQ3_SRC_EINT1_Msk                  (0x1ul << INT_IRQ3_SRC_EINT1_Pos)                 /*!< INT_T::IRQ3_SRC: EINT1 Mask               */

#define INT_IRQ4_SRC_GP0_INT_Pos                (0)                                               /*!< INT_T::IRQ4_SRC: GP0_INT Position         */
#define INT_IRQ4_SRC_GP0_INT_Msk                (0x1ul << INT_IRQ4_SRC_GP0_INT_Pos)               /*!< INT_T::IRQ4_SRC: GP0_INT Mask             */

#define INT_IRQ4_SRC_GP1_INT_Pos                (1)                                               /*!< INT_T::IRQ4_SRC: GP1_INT Position         */
#define INT_IRQ4_SRC_GP1_INT_Msk                (0x1ul << INT_IRQ4_SRC_GP1_INT_Pos)               /*!< INT_T::IRQ4_SRC: GP1_INT Mask             */

#define INT_IRQ5_SRC_GP2_INT_Pos                (0)                                               /*!< INT_T::IRQ5_SRC: GP2_INT Position         */
#define INT_IRQ5_SRC_GP2_INT_Msk                (0x1ul << INT_IRQ5_SRC_GP2_INT_Pos)               /*!< INT_T::IRQ5_SRC: GP2_INT Mask             */

#define INT_IRQ5_SRC_GP3_INT_Pos                (1)                                               /*!< INT_T::IRQ5_SRC: GP3_INT Position         */
#define INT_IRQ5_SRC_GP3_INT_Msk                (0x1ul << INT_IRQ5_SRC_GP3_INT_Pos)               /*!< INT_T::IRQ5_SRC: GP3_INT Mask             */

#define INT_IRQ5_SRC_GP4_INT_Pos                (2)                                               /*!< INT_T::IRQ5_SRC: GP4_INT Position         */
#define INT_IRQ5_SRC_GP4_INT_Msk                (0x1ul << INT_IRQ5_SRC_GP4_INT_Pos)               /*!< INT_T::IRQ5_SRC: GP4_INT Mask             */

#define INT_IRQ6_SRC_PWM_INT_Pos                (0)                                               /*!< INT_T::IRQ6_SRC: PWM_INT Position         */
#define INT_IRQ6_SRC_PWM_INT_Msk                (0x1ul << INT_IRQ6_SRC_PWM_INT_Pos)               /*!< INT_T::IRQ6_SRC: PWM_INT Mask             */

#define INT_IRQ8_SRC_TMR0_INT_Pos               (0)                                               /*!< INT_T::IRQ8_SRC: TMR0_INT Position        */
#define INT_IRQ8_SRC_TMR0_INT_Msk               (0x1ul << INT_IRQ8_SRC_TMR0_INT_Pos)              /*!< INT_T::IRQ8_SRC: TMR0_INT Mask            */

#define INT_IRQ9_SRC_TMR1_INT_Pos               (0)                                               /*!< INT_T::IRQ9_SRC: TMR1_INT Position        */
#define INT_IRQ9_SRC_TMR1_INT_Msk               (0x1ul << INT_IRQ9_SRC_TMR1_INT_Pos)              /*!< INT_T::IRQ9_SRC: TMR1_INT Mask            */

#define INT_IRQ10_SRC_TMR2_INT_Pos               (0)                                               /*!< INT_T::IRQ10_SRC: TMR2_INT Position        */
#define INT_IRQ10_SRC_TMR2_INT_Msk               (0x1ul << INT_IRQ10_SRC_TMR2_INT_Pos)              /*!< INT_T::IRQ10_SRC: TMR2_INT Mask            */

#define INT_IRQ11_SRC_EINT2_Pos                  (0)                                               /*!< INT_T::IRQ11_SRC: EINT2 Position           */
#define INT_IRQ11_SRC_EINT2_Msk                  (0x1ul << INT_IRQ11_SRC_EINT2_Pos)                 /*!< INT_T::IRQ11_SRC: EINT2 Mask               */

#define INT_IRQ12_SRC_UART0_INT_Pos             (0)                                               /*!< INT_T::IRQ12_SRC: UART0_INT Position      */
#define INT_IRQ12_SRC_UART0_INT_Msk             (0x1ul << INT_IRQ12_SRC_UART0_INT_Pos)            /*!< INT_T::IRQ12_SRC: UART0_INT Mask          */

#define INT_IRQ13_SRC_UART1_INT_Pos             (0)                                               /*!< INT_T::IRQ13_SRC: UART1_INT Position      */
#define INT_IRQ13_SRC_UART1_INT_Msk             (0x1ul << INT_IRQ13_SRC_UART1_INT_Pos)            /*!< INT_T::IRQ13_SRC: UART1_INT Mask          */

#define INT_IRQ14_SRC_SPI0_INT_Pos               (0)                                               /*!< INT_T::IRQ14_SRC: SPI0_INT Position        */
#define INT_IRQ14_SRC_SPI0_INT_Msk               (0x1ul << INT_IRQ14_SRC_SPI0_INT_Pos)              /*!< INT_T::IRQ14_SRC: SPI0_INT Mask            */

#define INT_IRQ15_SRC_SPI1_INT_Pos               (0)                                               /*!< INT_T::IRQ15_SRC: SPI1_INT Position        */
#define INT_IRQ15_SRC_SPI1_INT_Msk               (0x1ul << INT_IRQ15_SRC_SPI1_INT_Pos)              /*!< INT_T::IRQ15_SRC: SPI1_INT Mask            */

#define INT_IRQ16_SRC_GP5_INT_Pos               (0)                                               /*!< INT_T::IRQ16_SRC: GP5_INT Position        */
#define INT_IRQ16_SRC_GP5_INT_Msk               (0x1ul << INT_IRQ16_SRC_GP5_INT_Pos)              /*!< INT_T::IRQ16_SRC: GP5_INT Mask            */

#define INT_IRQ17_SRC_HIRC_TRIM_INT_Pos         (0)                                               /*!< INT_T::IRQ17_SRC: HIRC_TRIM_INT Position  */
#define INT_IRQ17_SRC_HIRC_TRIM_INT_Msk         (0x1ul << INT_IRQ17_SRC_HIRC_TRIM_INT_Pos)        /*!< INT_T::IRQ17_SRC: HIRC_TRIM_INT Mask      */

#define INT_IRQ18_SRC_I2C0_INT_Pos              (0)                                               /*!< INT_T::IRQ18_SRC: I2C0_INT Position       */
#define INT_IRQ18_SRC_I2C0_INT_Msk              (0x1ul << INT_IRQ18_SRC_I2C0_INT_Pos)             /*!< INT_T::IRQ18_SRC: I2C0_INT Mask           */

#define INT_IRQ19_SRC_I2C1_INT_Pos              (0)                                               /*!< INT_T::IRQ19_SRC: I2C1_INT Position       */
#define INT_IRQ19_SRC_I2C1_INT_Msk              (0x1ul << INT_IRQ19_SRC_I2C1_INT_Pos)             /*!< INT_T::IRQ19_SRC: I2C1_INT Mask           */

#define INT_IRQ25_SRC_ACMP_INT_Pos              (0)                                               /*!< INT_T::IRQ25_SRC: ACMP_INT Position       */
#define INT_IRQ25_SRC_ACMP_INT_Msk              (0x1ul << INT_IRQ25_SRC_ACMP_INT_Pos)             /*!< INT_T::IRQ25_SRC: ACMP_INT Mask           */

#define INT_IRQ28_SRC_PWRWU_INT_Pos             (0)                                               /*!< INT_T::IRQ28_SRC: PWRWU_INT Position      */
#define INT_IRQ28_SRC_PWRWU_INT_Msk             (0x1ul << INT_IRQ28_SRC_PWRWU_INT_Pos)            /*!< INT_T::IRQ28_SRC: PWRWU_INT Mask          */

#define INT_IRQ29_SRC_ADC_INT_Pos               (0)                                               /*!< INT_T::IRQ29_SRC: ADC_INT Position        */
#define INT_IRQ29_SRC_ADC_INT_Msk               (0x1ul << INT_IRQ29_SRC_ADC_INT_Pos)              /*!< INT_T::IRQ29_SRC: ADC_INT Mask            */

#define INT_CON_NMI_SEL_Pos                     (0)                                               /*!< INT_T::CON: NMI_SEL Position              */
#define INT_CON_NMI_SEL_Msk                     (0x1ful << INT_CON_NMI_SEL_Pos)                   /*!< INT_T::CON: NMI_SEL Mask                  */


/**@}*/ /* INT_CONST */
/**@}*/ /* end of INT register group */


/*---------------------- Pulse Width Modulation Controller -------------------------*/
/**
    @addtogroup PWM Pulse Width Modulation Controller(PWM)
    Memory Mapped Structure for PWM Controller
@{ */

typedef struct {


    /**
     * CLKPSC
     * ===================================================================================================
     * Offset: 0x00  PWM Clock Pre-scale Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |CLKPSC01  |Clock Prescaler 0 For PWM Counter 0 And 1
     * |        |          |Clock input is divided by (CLKPSC01 + 1) before it is fed to the corresponding PWM counter.
     * |        |          |If CLKPSC01 = 0, the clock prescaler 0 output clock will be stopped.
     * |        |          |So the corresponding PWM counter will also be stopped.
     * |[15:8]  |CLKPSC23  |Clock Prescaler 2 For PWM Counter 2 And 3
     * |        |          |Clock input is divided by (CLKPSC23 + 1) before it is fed to the corresponding PWM counter.
     * |        |          |If CLKPSC23 = 0, the clock prescaler 2 output clock will be stopped.
     * |        |          |So the corresponding PWM counter will also be stopped.
     * |[23:16] |CLKPSC45  |Clock Prescaler 4 For PWM Counter 4 And 5
     * |        |          |Clock input is divided by (CLKPSC45 + 1) before it is fed to the corresponding PWM counter.
     * |        |          |If CLKPSC45 = 0, the clock prescaler 4 output clock will be stopped.
     * |        |          |So the corresponding PWM counter will also be stopped.
    * |[31:24] |CLKPSC67  |Clock Prescaler 6 For PWM Counter 6 And 7
    * |        |          |Clock input is divided by (CLKPSC67 + 1) before it is fed to the corresponding PWM counter.
    * |        |          |If CLKPSC67 = 0, the clock prescaler 6 output clock will be stopped.
    * |        |          |So the corresponding PWM counter will also be stopped.
    */
    __IO uint32_t CLKPSC;

    /**
     * CLKDIV
     * ===================================================================================================
     * Offset: 0x04  PWM Clock Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |CLKDIV0   |Counter 0 Clock Divider Selection
     * |        |          |Select clock input for PWM counter.
     * |        |          |000 = Clock input / (CLKPSC01*2).
     * |        |          |001 = Clock input / (CLKPSC01*4).
     * |        |          |010 = Clock input / (CLKPSC01*8).
     * |        |          |011 = Clock input / (CLKPSC01*16).
     * |        |          |100 = Clock input / CLKPSC01.
     * |        |          |Others = Clock input.
     * |[6:4]   |CLKDIV1   |Counter 1 Clock Divider Selection
     * |        |          |Select clock input for PWM counter.
     * |        |          |000 = Clock input / (CLKPSC01*2).
     * |        |          |001 = Clock input / (CLKPSC01*4).
     * |        |          |010 = Clock input / (CLKPSC01*8).
     * |        |          |011 = Clock input / (CLKPSC01*16).
     * |        |          |100 = Clock input / CLKPSC01.
     * |        |          |Others = Clock input.
     * |[10:8]  |CLKDIV2   |Counter 2 Clock Divider Selection
     * |        |          |Select clock input for PWM counter.
     * |        |          |000 = Clock input / (CLKPSC23*2).
     * |        |          |001 = Clock input / (CLKPSC23*4).
     * |        |          |010 = Clock input / (CLKPSC23*8).
     * |        |          |011 = Clock input / (CLKPSC23*16).
     * |        |          |100 = Clock input / CLKPSC23.
     * |        |          |Others = Clock input.
     * |[14:12] |CLKDIV3   |Counter 3 Clock Divider Selection
     * |        |          |Select clock input for PWM counter.
     * |        |          |000 = Clock input / (CLKPSC23*2).
     * |        |          |001 = Clock input / (CLKPSC23*4).
     * |        |          |010 = Clock input / (CLKPSC23*8).
     * |        |          |011 = Clock input / (CLKPSC23*16).
     * |        |          |100 = Clock input / CLKPSC23.
     * |        |          |Others = Clock input.
     * |[18:16] |CLKDIV4   |Counter 4 Clock Divider Selection
     * |        |          |Select clock input for PWM counter.
     * |        |          |000 = Clock input / (CLKPSC45*2).
     * |        |          |001 = Clock input / (CLKPSC45*4).
     * |        |          |010 = Clock input / (CLKPSC45*8).
     * |        |          |011 = Clock input / (CLKPSC45*16).
     * |        |          |100 = Clock input / CLKPSC45.
     * |        |          |Others = Clock input.
     * |[22:20] |CLKDIV5   |Counter 5 Clock Divider Selection
     * |        |          |Select clock input for PWM counter.
     * |        |          |000 = Clock input / (CLKPSC45*2).
     * |        |          |001 = Clock input / (CLKPSC45*4).
     * |        |          |010 = Clock input / (CLKPSC45*8).
     * |        |          |011 = Clock input / (CLKPSC45*16).
     * |        |          |100 = Clock input / CLKPSC45.
     * |        |          |Others = Clock input.
     *|[26:24] |CLKDIV4   |Counter 6 Clock Divider Selection
     *|        |          |Select clock input for PWM counter.
     *|        |          |000 = Clock input / (CLKPSC67*2).
     *|        |          |001 = Clock input / (CLKPSC67*4).
     *|        |          |010 = Clock input / (CLKPSC67*8).
     *|        |          |011 = Clock input / (CLKPSC67*16).
     *|        |          |100 = Clock input / CLKPSC67.
     *|        |          |Others = Clock input.
     *|[30:28] |CLKDIV5   |Counter 7 Clock Divider Selection
     *|        |          |Select clock input for PWM counter.
     *|        |          |000 = Clock input / (CLKPSC67*2).
     *|        |          |001 = Clock input / (CLKPSC67*4).
     *|        |          |010 = Clock input / (CLKPSC67*8).
     *|        |          |011 = Clock input / (CLKPSC67*16).
     *|        |          |100 = Clock input / CLKPSC67.
     *|        |          |Others = Clock input.
     */
    __IO uint32_t CLKDIV;

    /**
     * CTL
     * ===================================================================================================
     * Offset: 0x08  PWM Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CNTEN0    |PWM Counter 0 Enable Start Run
     * |        |          |0 = Corresponding PWM counter running Stopped.
     * |        |          |1 = Corresponding PWM counter start run Enabled.
     * |[2]     |PINV0     |PWM0_CH0 Output Inverter Enable Bit
     * |        |          |0 = PWM0_CH0 output inverter Disabled.
     * |        |          |1 = PWM0_CH0 output inverter Enabled.
     * |[3]     |CNTMODE0  |PWM Counter 0 Auto-reload/One-shot Mode
     * |        |          |0 = One-shot mode.
     * |        |          |1 = Auto-reload mode.
     * |        |          |Note: If there is a rising transition at this bit, it will cause PERIOD0 and CMP0 cleared.
     * |[4]     |CNTEN1    |PWM Counter 1 Enable/Disable Start Run
     * |        |          |0 = Corresponding PWM counter running Stopped.
     * |        |          |1 = Corresponding PWM counter start run Enabled.
     * |[5]     |HCUPDT    |Half Cycle Update Enable for Center-aligned Type
     * |        |          |0 = Disable half cycle update PERIOD & CMP.
     * |        |          |1 = Enable half cycle update PERIOD & CMP.
     * |[6]     |PINV1     |PWM0_CH1 Output Inverter Enable Bit
     * |        |          |0 = PWM0_CH1 output inverter Disable.
     * |        |          |1 = PWM0_CH1 output inverter Enable.
     * |[7]     |CNTMODE1  |PWM Counter 1 Auto-reload/One-shot Mode
     * |        |          |0 = One-shot mode.
     * |        |          |1 = Auto-reload mode.
     * |        |          |Note: If there is a rising transition at this bit, it will cause PERIOD1 and CMP1 cleared.
     * |[8]     |CNTEN2    |PWM Counter 2 Enable Start Run
     * |        |          |0 = Corresponding PWM counter running Stopped.
     * |        |          |1 = Corresponding PWM counter start run Enabled.
     * |[10]    |PINV2     |PWM0_CH2 Output Inverter Enable Bit
     * |        |          |0 = PWM0_CH2 output inverter Disabled.
     * |        |          |1 = PWM0_CH2 output inverter Enabled.
     * |[11]    |CNTMODE2  |PWM Counter 2 Auto-reload/One-shot Mode
     * |        |          |0 = One-shot mode.
     * |        |          |1 = Auto-reload mode.
     * |        |          |Note: If there is a rising transition at this bit, it will cause PERIOD2 and CMP2 cleared.
     * |[12]    |CNTEN3    |PWM Counter 3 Enable Start Run
     * |        |          |0 = Corresponding PWM counter running Stopped.
     * |        |          |1 = Corresponding PWM counter start run Enabled.
     * |[14]    |PINV3     |PWM0_CH 3 Output Inverter Enable Bit
     * |        |          |0 = PWM0_CH3 output inverter Disabled.
     * |        |          |1 = PWM0_CH3 output inverter Enabled.
     * |[15]    |CNTMODE3  |PWM Counter 3 Auto-reload/One-shot Mode
     * |        |          |0 = One-shot mode.
     * |        |          |1 = Auto-reload mode.
     * |        |          |Note: If there is a rising transition at this bit, it will cause PERIOD3 and CMP3 cleared.
     * |[16]    |CNTEN4    |PWM Counter 4 Enable Start Run
     * |        |          |0 = Corresponding PWM counter running Stopped.
     * |        |          |1 = Corresponding PWM counter start run Enabled.
     * |[18]    |PINV4     |PWM0_CH4 Output Inverter Enable Bit
     * |        |          |0 = PWM0_CH4 output inverter Disabled.
     * |        |          |1 = PWM0_CH4 output inverter Enabled.
     * |[19]    |CNTMODE4  |PWM Counter 4 Auto-reload/One-shot Mode
     * |        |          |0 = One-shot mode.
     * |        |          |1 = Auto-reload mode.
     * |        |          |Note: If there is a rising transition at this bit, it will cause PERIOD4 and CMP4 cleared.
     * |[20]    |CNTEN5    |PWM Counter 5 Enable Start Run
     * |        |          |0 = Corresponding PWM counter running Stopped.
     * |        |          |1 = Corresponding PWM counter start run Enabled.
     * |[21]    |ASYMEN    |Asymmetric Mode In Center-aligned Type
     * |        |          |0 = Symmetric mode in center-aligned type.
     * |        |          |1 = Asymmetric mode in center-aligned type.
     * |[22]    |PINV5     |PWM0_CH5 Output Inverter Enable Bit
     * |        |          |0 = PWM0_CH5 output inverter Disabled.
     * |        |          |1 = PWM0_CH5 output inverter Enabled.
     * |[23]    |CNTMODE5  |PWM Counter 5 Auto-reload/One-shot Mode
     * |        |          |0 = One-shot mode.
     * |        |          |1 = Auto-reload mode.
     * |        |          |Note: If there is a rising transition at this bit, it will cause PERIOD5 and CMP5 cleared.
     *|[24]    |CNTEN6    |PWM Counter 6 Enable Start Run
     *|        |          |0 = Corresponding PWM counter running Stopped.
     *|        |          |1 = Corresponding PWM counter start run Enabled.
     *|[26]    |PINV6     |PWM0_CH6 Output Inverter Enable Bit
     *|        |          |0 = PWM0_CH6 output inverter Disabled.
     *|        |          |1 = PWM0_CH6 output inverter Enabled.
     *|[27]    |CNTMODE6  |PWM Counter 6 Auto-reload/One-shot Mode
     *|        |          |0 = One-shot mode.
     *|        |          |1 = Auto-reload mode.
     *|        |          |Note: If there is a rising transition at this bit, it will cause PERIOD6 and CMP6 cleared.
     *|[28]    |CNTEN7    |PWM Counter 7 Enable Start Run
     *|        |          |0 = Corresponding PWM counter running Stopped.
     *|        |          |1 = Corresponding PWM counter start run Enabled.
     *|[30]    |PINV7     |PWM0_CH7 Output Inverter Enable Bit
     *|        |          |0 = PWM0_CH7 output inverter Disabled.
     *|        |          |1 = PWM0_CH7 output inverter Enabled.
     *|[31]    |CNTMODE7  |PWM Counter 7 Auto-reload/One-shot Mode
     *|        |          |0 = One-shot mode.
     *|        |          |1 = Auto-reload mode.
     *|        |          |Note: If there is a rising transition at this bit, it will cause PERIOD7 and CMP7 cleared.
    */
    __IO uint32_t CTL;

    /**
     * PERIOD0
     * ===================================================================================================
     * Offset: 0x0C  PWM Counter Period Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |PERIOD0   |PWM Counter Period Value
     * |        |          |PERIODn determines the PWM counter period.
     * |        |          |Edge-aligned type:
     * |        |          |PWM frequency = HCLK/((prescale+1)*(clock divider))/( PERIODn+1); where xy, could be 01, 23, 45, 67 depending on the selected PWM channel.		// 45 -> 45, 67
     * |        |          |Duty ratio = (CMPn+1)/( PERIODn+1).
     * |        |          |CMPn >= PERIODn: PWM output is always low.
     * |        |          |CMPn < PERIODn: PWM low width = (PERIODn-CMPn) unit; PWM high width = (CMP+1) unit.
     * |        |          |CMPn = 0: PWM low width = (PERIODn) unit; PWM high width = 1 unit.
     * |        |          |Center-aligned type:
     * |        |          |PWM frequency = HCLK/((prescale+1)*(clock divider))/ (2*PERIODn+1); where xy, could be 01, 23, 45, 67 depending on the selected PWM channel.	// 45 -> 45, 67
     * |        |          |Duty ratio = (PERIODn - CMPn)/( PERIODn+1).
     * |        |          |CMPn >= PERIODn: PWM output is always low.
     * |        |          |CMPn < PERIODn: 
     * |	 low width = (CMPn + 1) * 2 unit; PWM high width = (PERIODn - CMPn) * 2 unit.
     * |        |          |CMPn = 0: PWM low width = 2 unit; PWM high width = (PERIODn) * 2 unit.
     * |        |          |(Unit = One PWM clock cycle).
     * |        |          |Note: Any write to PERIODn will take effect in the next PWM cycle.
     */
    __IO uint32_t PERIOD0;

    /**
     * PERIOD1
     * ===================================================================================================
     * Offset: 0x10  PWM Counter Period Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |PERIOD1   |PWM Counter Period Value
     * |        |          |PERIODn determines the PWM counter period.
     * |        |          |Edge-aligned type:
     * |        |          |PWM frequency = HCLK/((prescale+1)*(clock divider))/( PERIODn+1); where xy, could be 01, 23, 45, 67 depending on the selected PWM channel.		// 45 -> 45, 67
     * |        |          |Duty ratio = (CMPn+1)/( PERIODn+1).
     * |        |          |CMPn >= PERIODn: PWM output is always low.
     * |        |          |CMPn < PERIODn: PWM low width = (PERIODn-CMPn) unit; PWM high width = (CMP+1) unit.
     * |        |          |CMPn = 0: PWM low width = (PERIODn) unit; PWM high width = 1 unit.
     * |        |          |Center-aligned type:
     * |        |          |PWM frequency = HCLK/((prescale+1)*(clock divider))/ (2*PERIODn+1); where xy, could be 01, 23, 45, 67 depending on the selected PWM channel.	// 45 -> 45, 67
     * |        |          |Duty ratio = (PERIODn - CMPn)/( PERIODn+1).
     * |        |          |CMPn >= PERIODn: PWM output is always low.
     * |        |          |CMPn < PERIODn: PWM low width = (CMPn + 1) * 2 unit; PWM high width = (PERIODn - CMPn) * 2 unit.
     * |        |          |CMPn = 0: PWM low width = 2 unit; PWM high width = (PERIODn) * 2 unit.
     * |        |          |(Unit = One PWM clock cycle).
     * |        |          |Note: Any write to PERIODn will take effect in the next PWM cycle.
    */
    __IO uint32_t PERIOD1;

    /**
     * PERIOD2
     * ===================================================================================================
     * Offset: 0x14  PWM Counter Period Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |PERIOD2   |PWM Counter Period Value
     * |        |          |PERIODn determines the PWM counter period.
     * |        |          |Edge-aligned type:
     * |        |          |PWM frequency = HCLK/((prescale+1)*(clock divider))/( PERIODn+1); where xy, could be 01, 23, 45, 67 depending on the selected PWM channel.		// 45 -> 45, 67
     * |        |          |Duty ratio = (CMPn+1)/( PERIODn+1).
     * |        |          |CMPn >= PERIODn: PWM output is always low.
     * |        |          |CMPn < PERIODn: PWM low width = (PERIODn-CMPn) unit; PWM high width = (CMP+1) unit.
     * |        |          |CMPn = 0: PWM low width = (PERIODn) unit; PWM high width = 1 unit.
     * |        |          |Center-aligned type:
     * |        |          |PWM frequency = HCLK/((prescale+1)*(clock divider))/ (2*PERIODn+1); where xy, could be 01, 23, 45, 67 depending on the selected PWM channel.	// 45 -> 45, 67
     * |        |          |Duty ratio = (PERIODn - CMPn)/( PERIODn+1).
     * |        |          |CMPn >= PERIODn: PWM output is always low.
     * |        |          |CMPn < PERIODn: PWM low width = (CMPn + 1) * 2 unit; PWM high width = (PERIODn - CMPn) * 2 unit.
     * |        |          |CMPn = 0: PWM low width = 2 unit; PWM high width = (PERIODn) * 2 unit.
     * |        |          |(Unit = One PWM clock cycle).
     * |        |          |Note: Any write to PERIODn will take effect in the next PWM cycle.
    */
    __IO uint32_t PERIOD2;

    /**
     * PERIOD3
     * ===================================================================================================
     * Offset: 0x18  PWM Counter Period Register 3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |PERIOD3   |PWM Counter Period Value
     * |        |          |PERIODn determines the PWM counter period.
     * |        |          |Edge-aligned type:
     * |        |          |PWM frequency = HCLK/((prescale+1)*(clock divider))/( PERIODn+1); where xy, could be 01, 23, 45, 67 depending on the selected PWM channel.		// 45 -> 45, 67
     * |        |          |Duty ratio = (CMPn+1)/( PERIODn+1).
     * |        |          |CMPn >= PERIODn: PWM output is always low.
     * |        |          |CMPn < PERIODn: PWM low width = (PERIODn-CMPn) unit; PWM high width = (CMP+1) unit.
     * |        |          |CMPn = 0: PWM low width = (PERIODn) unit; PWM high width = 1 unit.
     * |        |          |Center-aligned type:
     * |        |          |PWM frequency = HCLK/((prescale+1)*(clock divider))/ (2*PERIODn+1); where xy, could be 01, 23, 45, 67 depending on the selected PWM channel.	// 45 -> 45, 67
     * |        |          |Duty ratio = (PERIODn - CMPn)/( PERIODn+1).
     * |        |          |CMPn >= PERIODn: PWM output is always low.
     * |        |          |CMPn < PERIODn: PWM low width = (CMPn + 1) * 2 unit; PWM high width = (PERIODn - CMPn) * 2 unit.
     * |        |          |CMPn = 0: PWM low width = 2 unit; PWM high width = (PERIODn) * 2 unit.
     * |        |          |(Unit = One PWM clock cycle).
     * |        |          |Note: Any write to PERIODn will take effect in the next PWM cycle.
    */
    __IO uint32_t PERIOD3;

    /**
     * PERIOD4
     * ===================================================================================================
     * Offset: 0x1C  PWM Counter Period Register 4
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |PERIOD4   |PWM Counter Period Value
     * |        |          |PERIODn determines the PWM counter period.
     * |        |          |Edge-aligned type:
     * |        |          |PWM frequency = HCLK/((prescale+1)*(clock divider))/( PERIODn+1); where xy, could be 01, 23, 45, 67 depending on the selected PWM channel.		// 45 -> 45, 67
     * |        |          |Duty ratio = (CMPn+1)/( PERIODn+1).
     * |        |          |CMPn >= PERIODn: PWM output is always low.
     * |        |          |CMPn < PERIODn: PWM low width = (PERIODn-CMPn) unit; PWM high width = (CMP+1) unit.
     * |        |          |CMPn = 0: PWM low width = (PERIODn) unit; PWM high width = 1 unit.
     * |        |          |Center-aligned type:
     * |        |          |PWM frequency = HCLK/((prescale+1)*(clock divider))/ (2*PERIODn+1); where xy, could be 01, 23, 45, 67 depending on the selected PWM channel.	// 45 -> 45, 67
     * |        |          |Duty ratio = (PERIODn - CMPn)/( PERIODn+1).
     * |        |          |CMPn >= PERIODn: PWM output is always low.
     * |        |          |CMPn < PERIODn: PWM low width = (CMPn + 1) * 2 unit; PWM high width = (PERIODn - CMPn) * 2 unit.
     * |        |          |CMPn = 0: PWM low width = 2 unit; PWM high width = (PERIODn) * 2 unit.
     * |        |          |(Unit = One PWM clock cycle).
     * |        |          |Note: Any write to PERIODn will take effect in the next PWM cycle.
    */
    __IO uint32_t PERIOD4;

    /**
     * PERIOD5
     * ===================================================================================================
     * Offset: 0x20  PWM Counter Period Register 5
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |PERIOD5   |PWM Counter Period Value
     * |        |          |PERIODn determines the PWM counter period.
     * |        |          |Edge-aligned type:
     * |        |          |PWM frequency = HCLK/((prescale+1)*(clock divider))/( PERIODn+1); where xy, could be 01, 23, 45, 67 depending on the selected PWM channel.		// 45 -> 45, 67
     * |        |          |Duty ratio = (CMPn+1)/( PERIODn+1).
     * |        |          |CMPn >= PERIODn: PWM output is always low.
     * |        |          |CMPn < PERIODn: PWM low width = (PERIODn-CMPn) unit; PWM high width = (CMP+1) unit.
     * |        |          |CMPn = 0: PWM low width = (PERIODn) unit; PWM high width = 1 unit.
     * |        |          |Center-aligned type:
     * |        |          |PWM frequency = HCLK/((prescale+1)*(clock divider))/ (2*PERIODn+1); where xy, could be 01, 23, 45, 67 depending on the selected PWM channel.	// 45 -> 45, 67
     * |        |          |Duty ratio = (PERIODn - CMPn)/( PERIODn+1).
     * |        |          |CMPn >= PERIODn: PWM output is always low.
     * |        |          |CMPn < PERIODn: PWM low width = (CMPn + 1) * 2 unit; PWM high width = (PERIODn - CMPn) * 2 unit.
     * |        |          |CMPn = 0: PWM low width = 2 unit; PWM high width = (PERIODn) * 2 unit.
     * |        |          |(Unit = One PWM clock cycle).
     * |        |          |Note: Any write to PERIODn will take effect in the next PWM cycle.
    */
    __IO uint32_t PERIOD5;

    /**
    * PERIOD6
    * ===================================================================================================
    * Offset: 0x24  PWM Counter Period Register 6
    * ---------------------------------------------------------------------------------------------------
    * |Bits    |Field     |Descriptions
    * | :----: | :----:   | :---- |
    * |[15:0]  |PERIOD6   |PWM Counter Period Value
    * |        |          |PERIODn determines the PWM counter period.
    * |        |          |Edge-aligned type:
    * |        |          |PWM frequency = HCLK/((prescale+1)*(clock divider))/( PERIODn+1); where xy, could be 01, 23, 45, 67 depending on the selected PWM channel.
    * |        |          |Duty ratio = (CMPn+1)/( PERIODn+1).
    * |        |          |CMPn >= PERIODn: PWM output is always low.
    * |        |          |CMPn < PERIODn: PWM low width = (PERIODn-CMPn) unit; PWM high width = (CMP+1) unit.
    * |        |          |CMPn = 0: PWM low width = (PERIODn) unit; PWM high width = 1 unit.
    * |        |          |Center-aligned type:
    * |        |          |PWM frequency = HCLK/((prescale+1)*(clock divider))/ (2*PERIODn+1); where xy, could be 01, 23, 45, 67 depending on the selected PWM channel.
    * |        |          |Duty ratio = (PERIODn - CMPn)/( PERIODn+1).
    * |        |          |CMPn >= PERIODn: PWM output is always low.
    * |        |          |CMPn < PERIODn: PWM low width = (CMPn + 1) * 2 unit; PWM high width = (PERIODn - CMPn) * 2 unit.
    * |        |          |CMPn = 0: PWM low width = 2 unit; PWM high width = (PERIODn) * 2 unit.
    * |        |          |(Unit = One PWM clock cycle).
    * |        |          |Note: Any write to PERIODn will take effect in the next PWM cycle.
    */
    __IO uint32_t PERIOD6;

    /**
    * PERIOD7
    * ===================================================================================================
    * Offset: 0x28  PWM Counter Period Register 7
    * ---------------------------------------------------------------------------------------------------
    * |Bits    |Field     |Descriptions
    * | :----: | :----:   | :---- |
    * |[15:0]  |PERIOD7   |PWM Counter Period Value
    * |        |          |PERIODn determines the PWM counter period.
    * |        |          |Edge-aligned type:
    * |        |          |PWM frequency = HCLK/((prescale+1)*(clock divider))/( PERIODn+1); where xy, could be 01, 23, 45, 67 depending on the selected PWM channel.
    * |        |          |Duty ratio = (CMPn+1)/( PERIODn+1).
    * |        |          |CMPn >= PERIODn: PWM output is always low.
    * |        |          |CMPn < PERIODn: PWM low width = (PERIODn-CMPn) unit; PWM high width = (CMP+1) unit.
    * |        |          |CMPn = 0: PWM low width = (PERIODn) unit; PWM high width = 1 unit.
    * |        |          |Center-aligned type:
    * |        |          |PWM frequency = HCLK/((prescale+1)*(clock divider))/ (2*PERIODn+1); where xy, could be 01, 23, 45, 67 depending on the selected PWM channel.
    * |        |          |Duty ratio = (PERIODn - CMPn)/( PERIODn+1).
    * |        |          |CMPn >= PERIODn: PWM output is always low.
    * |        |          |CMPn < PERIODn: PWM low width = (CMPn + 1) * 2 unit; PWM high width = (PERIODn - CMPn) * 2 unit.
    * |        |          |CMPn = 0: PWM low width = 2 unit; PWM high width = (PERIODn) * 2 unit.
    * |        |          |(Unit = One PWM clock cycle).
    * |        |          |Note: Any write to PERIODn will take effect in the next PWM cycle.
    */
    __IO uint32_t PERIOD7;

    /**
     * CMPDAT0
     * ===================================================================================================
* Offset: 0x2C  PWM Comparator Register 0		// 0x24 - > 0x2C
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CMP0      |PWM Comparator Register
     * |        |          |CMP determines the PWM duty.
     * |        |          |Edge-aligned type:
     * |        |          |PWM frequency = HCLK/((CLKPSCnm+1)*(clock divider))/( PERIODn+1); where nm, could be 01, 23, 45, 67 depending on the selected PWM channel.		// 45 -> 45, 67
     * |        |          |Duty ratio = (CMPn+1)/(PERIODn+1).
     * |        |          |CMPn >= PERIODn: PWM output is always low.
     * |        |          |CMPn < PERIODn: PWM low width = (PERIODn-CMPn) unit; PWM high width = (CMP+1) unit.
     * |        |          |CMPn = 0: PWM low width = (PERIODn) unit; PWM high width = 1 unit.
     * |        |          |Center-aligned type:
     * |        |          |PWM frequency = HCLK/((CLKPSCnm+1)*(clock divider)) /(2*PERIODn+1); where nm, could be 01, 23, 45, 67, depending on the selected PWM channel.	// 45 -> 45, 67
     * |        |          |Duty ratio = (PERIODn - CMPn)/(PERIODn+1).
     * |        |          |CMPn >= PERIODn: PWM output is always low.
     * |        |          |CMPn < PERIODn: PWM low width = (CMPn + 1) * 2 unit; PWM high width = (PERIODn - CMPn) * 2 unit.
     * |        |          |CMPn = 0: PWM low width = 2 unit; PWM high width = (PERIODn) * 2 unit.
     * |        |          |(Unit = One PWM clock cycle).
     * |        |          |Note: Any write to CMPn will take effect in the next PWM cycle.
     * |[31:16] |CMPD0     |PWM Comparator Register For Down Counter In Asymmetric Mode
     * |        |          |CMPn >= PERIODn: up counter PWM output is always low.
     * |        |          |CMPDn >= PERIODn: down counter PWM output is always low.
     * |        |          |Others: PWM output is always high.
    */
    __IO uint32_t CMPDAT0;

    /**
     * CMPDAT1
     * ===================================================================================================
     * Offset: 0x30  PWM Comparator Register 1		// 0x28 -> 0x30
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CMP1      |PWM Comparator Register
     * |        |          |CMP determines the PWM duty.
     * |        |          |Edge-aligned type:
     * |        |          |PWM frequency = HCLK/((CLKPSCnm+1)*(clock divider))/( PERIODn+1); where nm, could be 01, 23, 45, 67 depending on the selected PWM channel.		// 45 -> 45, 67
     * |        |          |Duty ratio = (CMPn+1)/(PERIODn+1).
     * |        |          |CMPn >= PERIODn: PWM output is always low.
     * |        |          |CMPn < PERIODn: PWM low width = (PERIODn-CMPn) unit; PWM high width = (CMP+1) unit.
     * |        |          |CMPn = 0: PWM low width = (PERIODn) unit; PWM high width = 1 unit.
     * |        |          |Center-aligned type:
     * |        |          |PWM frequency = HCLK/((CLKPSCnm+1)*(clock divider)) /(2*PERIODn+1); where nm, could be 01, 23, 45, 67, depending on the selected PWM channel.	// 45 -> 45, 67
     * |        |          |Duty ratio = (PERIODn - CMPn)/(PERIODn+1).
     * |        |          |CMPn >= PERIODn: PWM output is always low.
     * |        |          |CMPn < PERIODn: PWM low width = (CMPn + 1) * 2 unit; PWM high width = (PERIODn - CMPn) * 2 unit.
     * |        |          |CMPn = 0: PWM low width = 2 unit; PWM high width = (PERIODn) * 2 unit.
     * |        |          |(Unit = One PWM clock cycle).
     * |        |          |Note: Any write to CMPn will take effect in the next PWM cycle.
     * |[31:16] |CMPD1     |PWM Comparator Register For Down Counter In Asymmetric Mode
     * |        |          |CMPn >= PERIODn: up counter PWM output is always low.
     * |        |          |CMPDn >= PERIODn: down counter PWM output is always low.
     * |        |          |Others: PWM output is always high.
    */
    __IO uint32_t CMPDAT1;

    /**
     * CMPDAT2
     * ===================================================================================================
     * Offset: 0x34  PWM Comparator Register 2		// 0x2C -> 0x34
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CMP2      |PWM Comparator Register
     * |        |          |CMP determines the PWM duty.
     * |        |          |Edge-aligned type:
     * |        |          |PWM frequency = HCLK/((CLKPSCnm+1)*(clock divider))/( PERIODn+1); where nm, could be 01, 23, 45, 67 depending on the selected PWM channel.		// 45 -> 45, 67
     * |        |          |Duty ratio = (CMPn+1)/(PERIODn+1).
     * |        |          |CMPn >= PERIODn: PWM output is always low.
     * |        |          |CMPn < PERIODn: PWM low width = (PERIODn-CMPn) unit; PWM high width = (CMP+1) unit.
     * |        |          |CMPn = 0: PWM low width = (PERIODn) unit; PWM high width = 1 unit.
     * |        |          |Center-aligned type:
     * |        |          |PWM frequency = HCLK/((CLKPSCnm+1)*(clock divider)) /(2*PERIODn+1); where nm, could be 01, 23, 45, 67, depending on the selected PWM channel.	// 45 -> 45, 67
     * |        |          |Duty ratio = (PERIODn - CMPn)/(PERIODn+1).
     * |        |          |CMPn >= PERIODn: PWM output is always low.
     * |        |          |CMPn < PERIODn: PWM low width = (CMPn + 1) * 2 unit; PWM high width = (PERIODn - CMPn) * 2 unit.
     * |        |          |CMPn = 0: PWM low width = 2 unit; PWM high width = (PERIODn) * 2 unit.
     * |        |          |(Unit = One PWM clock cycle).
     * |        |          |Note: Any write to CMPn will take effect in the next PWM cycle.
     * |[31:16] |CMPD2     |PWM Comparator Register For Down Counter In Asymmetric Mode
     * |        |          |CMPn >= PERIODn: up counter PWM output is always low.
     * |        |          |CMPDn >= PERIODn: down counter PWM output is always low.
     * |        |          |Others: PWM output is always high.
    */
    __IO uint32_t CMPDAT2;

    /**
     * CMPDAT3
     * ===================================================================================================
     * Offset: 0x38  PWM Comparator Register 3		// 0x30 -> 0x38
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CMP3      |PWM Comparator Register
     * |        |          |CMP determines the PWM duty.
     * |        |          |Edge-aligned type:
     * |        |          |PWM frequency = HCLK/((CLKPSCnm+1)*(clock divider))/( PERIODn+1); where nm, could be 01, 23, 45, 67 depending on the selected PWM channel.		// 45 -> 45, 67
     * |        |          |Duty ratio = (CMPn+1)/(PERIODn+1).
     * |        |          |CMPn >= PERIODn: PWM output is always low.
     * |        |          |CMPn < PERIODn: PWM low width = (PERIODn-CMPn) unit; PWM high width = (CMP+1) unit.
     * |        |          |CMPn = 0: PWM low width = (PERIODn) unit; PWM high width = 1 unit.
     * |        |          |Center-aligned type:
     * |        |          |PWM frequency = HCLK/((CLKPSCnm+1)*(clock divider)) /(2*PERIODn+1); where nm, could be 01, 23, 45, 67, depending on the selected PWM channel.	// 45 -> 45, 67
     * |        |          |Duty ratio = (PERIODn - CMPn)/(PERIODn+1).
     * |        |          |CMPn >= PERIODn: PWM output is always low.
     * |        |          |CMPn < PERIODn: PWM low width = (CMPn + 1) * 2 unit; PWM high width = (PERIODn - CMPn) * 2 unit.
     * |        |          |CMPn = 0: PWM low width = 2 unit; PWM high width = (PERIODn) * 2 unit.
     * |        |          |(Unit = One PWM clock cycle).
     * |        |          |Note: Any write to CMPn will take effect in the next PWM cycle.
     * |[31:16] |CMPD3     |PWM Comparator Register For Down Counter In Asymmetric Mode
     * |        |          |CMPn >= PERIODn: up counter PWM output is always low.
     * |        |          |CMPDn >= PERIODn: down counter PWM output is always low.
     * |        |          |Others: PWM output is always high.
    */
    __IO uint32_t CMPDAT3;

    /**
     * CMPDAT4
     * ===================================================================================================
     * Offset: 0x3C  PWM Comparator Register 4		// 0x34 -> 0x3C
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CMP4      |PWM Comparator Register
     * |        |          |CMP determines the PWM duty.
     * |        |          |Edge-aligned type:
     * |        |          |PWM frequency = HCLK/((CLKPSCnm+1)*(clock divider))/( PERIODn+1); where nm, could be 01, 23, 45, 67 depending on the selected PWM channel.		// 45 -> 45, 67
     * |        |          |Duty ratio = (CMPn+1)/(PERIODn+1).
     * |        |          |CMPn >= PERIODn: PWM output is always low.
     * |        |          |CMPn < PERIODn: PWM low width = (PERIODn-CMPn) unit; PWM high width = (CMP+1) unit.
     * |        |          |CMPn = 0: PWM low width = (PERIODn) unit; PWM high width = 1 unit.
     * |        |          |Center-aligned type:
     * |        |          |PWM frequency = HCLK/((CLKPSCnm+1)*(clock divider)) /(2*PERIODn+1); where nm, could be 01, 23, 45, 67, depending on the selected PWM channel.	// 45 -> 45, 67
     * |        |          |Duty ratio = (PERIODn - CMPn)/(PERIODn+1).
     * |        |          |CMPn >= PERIODn: PWM output is always low.
     * |        |          |CMPn < PERIODn: PWM low width = (CMPn + 1) * 2 unit; PWM high width = (PERIODn - CMPn) * 2 unit.
     * |        |          |CMPn = 0: PWM low width = 2 unit; PWM high width = (PERIODn) * 2 unit.
     * |        |          |(Unit = One PWM clock cycle).
     * |        |          |Note: Any write to CMPn will take effect in the next PWM cycle.
     * |[31:16] |CMPD4     |PWM Comparator Register For Down Counter In Asymmetric Mode
     * |        |          |CMPn >= PERIODn: up counter PWM output is always low.
     * |        |          |CMPDn >= PERIODn: down counter PWM output is always low.
     * |        |          |Others: PWM output is always high.
    */
    __IO uint32_t CMPDAT4;

    /**
     * CMPDAT5
     * ===================================================================================================
    * Offset: 0x40  PWM Comparator Register 5		// 0x38 -> 0x40
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CMP5      |PWM Comparator Register
     * |        |          |CMP determines the PWM duty.
     * |        |          |Edge-aligned type:
     * |        |          |PWM frequency = HCLK/((CLKPSCnm+1)*(clock divider))/( PERIODn+1); where nm, could be 01, 23, 45, 67 depending on the selected PWM channel.		// 45 -> 45, 67
     * |        |          |Duty ratio = (CMPn+1)/(PERIODn+1).
     * |        |          |CMPn >= PERIODn: PWM output is always low.
     * |        |          |CMPn < PERIODn: PWM low width = (PERIODn-CMPn) unit; PWM high width = (CMP+1) unit.
     * |        |          |CMPn = 0: PWM low width = (PERIODn) unit; PWM high width = 1 unit.
     * |        |          |Center-aligned type:
     * |        |          |PWM frequency = HCLK/((CLKPSCnm+1)*(clock divider)) /(2*PERIODn+1); where nm, could be 01, 23, 45, 67, depending on the selected PWM channel.	// 45 -> 45, 67
     * |        |          |Duty ratio = (PERIODn - CMPn)/(PERIODn+1).
     * |        |          |CMPn >= PERIODn: PWM output is always low.
     * |        |          |CMPn < PERIODn: PWM low width = (CMPn + 1) * 2 unit; PWM high width = (PERIODn - CMPn) * 2 unit.
     * |        |          |CMPn = 0: PWM low width = 2 unit; PWM high width = (PERIODn) * 2 unit.
     * |        |          |(Unit = One PWM clock cycle).
     * |        |          |Note: Any write to CMPn will take effect in the next PWM cycle.
     * |[31:16] |CMPD5     |PWM Comparator Register For Down Counter In Asymmetric Mode
     * |        |          |CMPn >= PERIODn: up counter PWM output is always low.
     * |        |          |CMPDn >= PERIODn: down counter PWM output is always low.
     * |        |          |Others: PWM output is always high.
    */
    __IO uint32_t CMPDAT5;

    /**
    * CMPDAT6
    * ===================================================================================================
    * Offset: 0x44  PWM Comparator Register 6
    * ---------------------------------------------------------------------------------------------------
    * |Bits    |Field     |Descriptions
    * | :----: | :----:   | :---- |
    * |[15:0]  |CMP6      |PWM Comparator Register
    * |        |          |CMP determines the PWM duty.
    * |        |          |Edge-aligned type:
    * |        |          |PWM frequency = HCLK/((CLKPSCnm+1)*(clock divider))/( PERIODn+1); where nm, could be 01, 23, 45, 67 depending on the selected PWM channel.
    * |        |          |Duty ratio = (CMPn+1)/(PERIODn+1).
    * |        |          |CMPn >= PERIODn: PWM output is always low.
    * |        |          |CMPn < PERIODn: PWM low width = (PERIODn-CMPn) unit; PWM high width = (CMP+1) unit.
    * |        |          |CMPn = 0: PWM low width = (PERIODn) unit; PWM high width = 1 unit.
    * |        |          |Center-aligned type:
    * |        |          |PWM frequency = HCLK/((CLKPSCnm+1)*(clock divider)) /(2*PERIODn+1); where nm, could be 01, 23, 45, 67, depending on the selected PWM channel.
    * |        |          |Duty ratio = (PERIODn - CMPn)/(PERIODn+1).
    * |        |          |CMPn >= PERIODn: PWM output is always low.
    * |        |          |CMPn < PERIODn: PWM low width = (CMPn + 1) * 2 unit; PWM high width = (PERIODn - CMPn) * 2 unit.
    * |        |          |CMPn = 0: PWM low width = 2 unit; PWM high width = (PERIODn) * 2 unit.
    * |        |          |(Unit = One PWM clock cycle).
    * |        |          |Note: Any write to CMPn will take effect in the next PWM cycle.
    * |[31:16] |CMPD6     |PWM Comparator Register For Down Counter In Asymmetric Mode
    * |        |          |CMPn >= PERIODn: up counter PWM output is always low.
    * |        |          |CMPDn >= PERIODn: down counter PWM output is always low.
    * |        |          |Others: PWM output is always high.
    */
    __IO uint32_t CMPDAT6;

    /**
    * CMPDAT7
    * ===================================================================================================
    * Offset: 0x48  PWM Comparator Register 7
    * ---------------------------------------------------------------------------------------------------
    * |Bits    |Field     |Descriptions
    * | :----: | :----:   | :---- |
    * |[15:0]  |CMP7      |PWM Comparator Register
    * |        |          |CMP determines the PWM duty.
    * |        |          |Edge-aligned type:
    * |        |          |PWM frequency = HCLK/((CLKPSCnm+1)*(clock divider))/( PERIODn+1); where nm, could be 01, 23, 45, 67 depending on the selected PWM channel.
    * |        |          |Duty ratio = (CMPn+1)/(PERIODn+1).
    * |        |          |CMPn >= PERIODn: PWM output is always low.
    * |        |          |CMPn < PERIODn: PWM low width = (PERIODn-CMPn) unit; PWM high width = (CMP+1) unit.
    * |        |          |CMPn = 0: PWM low width = (PERIODn) unit; PWM high width = 1 unit.
    * |        |          |Center-aligned type:
    * |        |          |PWM frequency = HCLK/((CLKPSCnm+1)*(clock divider)) /(2*PERIODn+1); where nm, could be 01, 23, 45, 67, depending on the selected PWM channel.
    * |        |          |Duty ratio = (PERIODn - CMPn)/(PERIODn+1).
    * |        |          |CMPn >= PERIODn: PWM output is always low.
    * |        |          |CMPn < PERIODn: PWM low width = (CMPn + 1) * 2 unit; PWM high width = (PERIODn - CMPn) * 2 unit.
    * |        |          |CMPn = 0: PWM low width = 2 unit; PWM high width = (PERIODn) * 2 unit.
    * |        |          |(Unit = One PWM clock cycle).
    * |        |          |Note: Any write to CMPn will take effect in the next PWM cycle.
    * |[31:16] |CMPD7     |PWM Comparator Register For Down Counter In Asymmetric Mode
    * |        |          |CMPn >= PERIODn: up counter PWM output is always low.
    * |        |          |CMPDn >= PERIODn: down counter PWM output is always low.
    * |        |          |Others: PWM output is always high.
    */
    __IO uint32_t CMPDAT7;

    /**
    * CTL2
    * ===================================================================================================
    * Offset: 0x4C  PWM Control Register
    * ---------------------------------------------------------------------------------------------------
    * |Bits    |Field     |Descriptions
    * | :----: | :----:   | :---- |
    * |[17]    |PINTTYPE  |PWM Interrupt Type Selection
    * |        |          |0 = ZIFn will be set if PWM counter underflows.
    * |        |          |1 = ZIFn will be set if PWM counter matches PERIODn register.
    * |        |          |Note: This bit is effective when PWM is in center-aligned type only.
    * |[24]    |DTCNT01   |Dead-time 0 Counter Enable Bit (PWM0_CH0 And PWM0_CH1 Pair For PWMA Group)						// removed from CTL
    * |        |          |0 = Dead-time 0 generator Disabled.
    * |        |          |1 = Dead-time 0 generator Enabled.
    * |        |          |Note: When the dead-time generator is enabled, the pair of PWM0_CH0 and PWM0_CH1 becomes a complementary pair for PWMA group.
    * |[25]    |DTCNT23   |Dead-time 2 Counter Enable Bit (PWM0_CH2 And PWM0_CH3 Pair For PWMB Group)
    * |        |          |0 = Dead-time 2 generator Disabled.
    * |        |          |1 = Dead-time 2 generator Enabled.
    * |        |          |Note: When the dead-time generator is enabled, the pair of PWM0_CH2 and PWM0_CH3 becomes a complementary pair for PWMB group.
    * |[26]    |DTCNT45   |Dead-time 4 Counter Enable Bit (PWM0_CH4 And PWM0_CH5 Pair For PWMC Group)
    * |        |          |0 = Dead-time 4 generator Disabled.
    * |        |          |1 = Dead-time 4 generator Enabled.
    * |        |          |Note: When the dead-time generator is enabled, the pair of PWM0_CH4 and PWM0_CH5 becomes a complementary pair for PWMC group.
    * |[27]    |DTCNT67   |Dead-time 6 Counter Enable Bit (PWM0_CH6 And PWM0_CH7 Pair For PWMC Group)
    * |        |          |0 = Dead-time 6 generator Disabled.
    * |        |          |1 = Dead-time 6 generator Enabled.
    * |        |          |Note: When the dead-time generator is enabled, the pair of PWM0_CH6 and PWM0_CH7 becomes a complementary pair for PWMC group.
    * |[27]    |CNTCLR    |Clear PWM Counter Control Bit
    * |        |          |0 = Do not clear PWM counter.
    * |        |          |1 = All 16-bit PWM counters cleared to 0x0000.
    * |        |          |Note: It is automatically cleared by hardware.
    * |[29:28] |MODE      |PWM Operating Mode Select Bit
    * |        |          |00 = Independent mode.
    * |        |          |01 = Complementary mode.
    * |        |          |10 = Synchronized mode.
    * |        |          |11 = Reserved.
    * |[30]    |GROUPEN   |Group Function Enable Bit
    * |        |          |0 = The signals timing of all PWM channels are independent.
    * |        |          |1 = Unify the signals timing of PWM0_CH0, PWM0_CH2 and PWM0_CH4 in the same phase which is controlled by PWM0_CH0 and also unify the signals timing of PWM0_CH1, PWM0_CH3 and PWM0_CH5 in the same phase which is controlled by PWM0_CH1.
    * |[31]    |CNTTYPE   |PWM Counter-aligned Type Select Bit
    * |        |          |0 = Edge-aligned type.
    * |        |          |1 = Center-aligned type.
    */
    __IO uint32_t CTL2;


    /**
     * INTSTS
     * ===================================================================================================
     * Offset: 0x58  PWM Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |ZFn      |PWM Zero Point Flag					
     * |        |          |Flag is set by hardware when PWMn counter down count reaches zero point.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[15:8]  |CMPDFn   |PWM Compare Down Flag					
     * |        |          |Flag is set by hardware when PWMn counter down count reaches CMPn.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[23:18] |PFn      |PWM Period Flag						
     * |        |          |Flag is set by hardware when PWM0_CHn counter reaches PERIODn.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[31:24] |CMPUFn   |PWM Compare Up Flag					
     * |        |          |Flag is set by hardware when PWM0_CHn counter up count reaches CMPn.
     * |        |          |Note: This bit can be cleared by software writing 1.
    */
    __IO  uint32_t FLAG;	


    /**
     * INTEN
     * ===================================================================================================
     * Offset: 0x54  PWM Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |ZIENn     |PWM Zero Point Interrupt Enable Bit				
     * |        |          |0 = PWM0_CHn zero point interrupt Disabled.
     * |        |          |1 = PWM0_CHn zero point interrupt Enabled.
     * |[15:8]  |CMPDIENn  |PWM Compare Down Interrupt Enable Bit			
     * |        |          |0 = PWM0_CHn compare down interrupt Disabled.
     * |        |          |1 = PWM0_CHn compare down interrupt Enabled.
     * |[23:16] |PIENn     |PWM Period Interrupt Enable Bit					
     * |        |          |0 = PWM0_CHn period interrupt Disabled.
     * |        |          |1 = PWM0_CHn period interrupt Enabled.
     * |[31:24] |CMPUIENn  |PWM Compare Up Interrupt Enable Bit				
     * |        |          |0 = PWM0_CHn compare up interrupt Disabled.
     * |        |          |1 = PWM0_CHn compare up interrupt Enabled.
    */
    __IO uint32_t INTEN;

    /**
     * INTSTS
     * ===================================================================================================
     * Offset: 0x58  PWM Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |ZIFn      |PWM Zero Point Interrupt Flag					
     * |        |          |Flag is set by hardware when PWMn counter down count reaches zero point.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[15:8]  |CMPDIFn   |PWM Compare Down Interrupt Flag					
     * |        |          |Flag is set by hardware when PWMn counter down count reaches CMPn.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[23:18] |PIFn      |PWM Period Interrupt Flag						
     * |        |          |Flag is set by hardware when PWM0_CHn counter reaches PERIODn.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[31:24] |CMPUIFn   |PWM Compare Up Interrupt Flag					
     * |        |          |Flag is set by hardware when PWM0_CHn counter up count reaches CMPn.
     * |        |          |Note: This bit can be cleared by software writing 1.
    */
    __IO uint32_t INTSTS;

    /**
     * POEN
     * ===================================================================================================
     * Offset: 0x5C  PWM Output Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |POENn     |PWM Output Enable Bits							
     * |        |          |0 = PWM channel n output to pin Disabled.
     * |        |          |1 = PWM channel n output to pin Enabled.
     * |        |          |Note: The corresponding GPIO pin must be switched to PWM function.
    */
    __IO uint32_t POEN;

       /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVED0[1];
    /// @endcond //HIDDEN_SYMBOLS


    /**
     * DTCTL
     * ===================================================================================================
     * Offset: 0x64  PWM Dead-time Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |DTI01     |Dead-time Interval Register For Pair Of Channel0 And Channel1 (PWM0_CH0 And PWM0_CH1 Pair)
     * |        |          |These 8 bits determine dead-time length.
     * |        |          |The unit time of dead-time length is received from corresponding PWM_CLKDIV bits.
     * |[15:8]  |DTI23     |Dead-time Interval Register For Pair Of Channel2 And Channel3 (PWM0_CH2 And PWM0_CH3 Pair)
     * |        |          |These 8 bits determine dead-time length.
     * |        |          |The unit time of dead-time length is received from corresponding PWM_CLKDIV bits.
     * |[23:16] |DTI45     |Dead-time Interval Register For Pair Of Channel4 And Channel5 (PWM0_CH4 And PWM0_CH5 Pair)
     * |        |          |These 8 bits determine dead-time length.
     * |        |          |The unit time of dead-time length is received from corresponding PWM_CLKDIV bits.
     * |[31:24] |DTI67     |Dead-time Interval Register For Pair Of Channel6 And Channel7 (PWM0_CH6 And PWM0_CH7 Pair)
     * |        |          |These 8 bits determine dead-time length.
     * |        |          |The unit time of dead-time length is received from corresponding PWM_CLKDIV bits.
    */
    __IO uint32_t DTCTL;

    /**
     * ADCTCTL0
     * ===================================================================================================
     * Offset: 0x68  PWM Trigger Control Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CUTRGEN0  |Channel 0 Compare Up Trigger ADC Enable Bit
     * |        |          |Enable PWM trigger ADC function while channel0's counter matching CMP0 in up-count direction
     * |        |          |0 = PWM condition trigger ADC function Disabled.
     * |        |          |1 = PWM condition trigger ADC function Enabled.
     * |        |          |Note: This bit is only valid for PWM in center-aligned type.
     * |        |          |When PWM is in edged-aligned type, setting this bit is meaningless and will not take any effect.
     * |[1]     |CPTRGEN0  |Channel 0 Center Point Trigger ADC Enable Bit
     * |        |          |Enable PWM Trigger ADC Function While channel0's Counter Matching PERIOD0
     * |        |          |0 = PWM condition trigger ADC function Disabled.
     * |        |          |1 = PWM condition trigger ADC function Enabled.
     * |        |          |Note: This bit is only valid for PWM in center-aligned type.
     * |        |          |When PWM is in edged-aligned type, setting this bit is meaningless and will not take any effect.
     * |[2]     |CDTRGEN0  |Channel 0 Compare Down Trigger ADC Enable Bit
     * |        |          |Enable PWM trigger ADC function while channel0's counter matching CMP0 in down-count direction
     * |        |          |0 = PWM condition trigger ADC function Disabled.
     * |        |          |1 = PWM condition trigger ADC function Enabled.
     * |        |          |Note: This bit is valid for both center-aligned type and edged-aligned type.
     * |[3]     |ZPTRGEN0  |Channel 0 Zero Point Trigger ADC Enable Bit
     * |        |          |Enable PWM trigger ADC function while channel0's counter matching 0
     * |        |          |0 = PWM condition trigger ADC function Disabled.
     * |        |          |1 = PWM condition trigger ADC function Enabled.
     * |        |          |Note: This bit is valid for both center-aligned type and edged-aligned type.
     * |[8]     |CUTRGEN1  |Channel 1 Compare Up Trigger ADC Enable Bit
     * |        |          |Enable PWM trigger ADC function while channel1's counter matching CMP1 in up-count direction
     * |        |          |0 = PWM condition trigger ADC function Disabled.
     * |        |          |1 = PWM condition trigger ADC function Enabled.
     * |        |          |Note: This bit is only valid for PWM in center-aligned type.
     * |        |          |When PWM is in edged-aligned type, setting this bit is meaningless and will not take any effect.
     * |[9]     |CPTRGEN1  |Channel 1 Center Point Trigger ADC Enable Bit
     * |        |          |Enable PWM trigger ADC function while channel1's counter matching PERIOD1
     * |        |          |0 = PWM condition trigger ADC function Disabled.
     * |        |          |1 = PWM condition trigger ADC function Enabled.
     * |        |          |Note: This bit is only valid for PWM in center-aligned type.
     * |        |          |When PWM is in edged-aligned type, setting this bit is meaningless and will not take any effect.
     * |[10]    |CDTRGEN1  |Channel 1 Compare Down Trigger ADC Enable Bit
     * |        |          |Enable PWM trigger ADC function while channel1's counter matching CMP1 in down-count direction
     * |        |          |0 = PWM condition trigger ADC function Disabled.
     * |        |          |1 = PWM condition trigger ADC function Enabled.
     * |        |          |Note: This bit is valid for both center-aligned type and edged-aligned type.
     * |[11]    |ZPTRGEN1  |Channel 1 Zero Point Trigger ADC Enable Bit
     * |        |          |Enable PWM trigger ADC function While channel1's Counter Matching 0
     * |        |          |0 = PWM condition trigger ADC function Disabled.
     * |        |          |1 = PWM condition trigger ADC function Enabled.
     * |        |          |Note: This bit is valid for both center-aligned type and edged-aligned type.
     * |[16]    |CUTRGEN2  |Channel 2 Compare Up Trigger ADC Enable Bit
     * |        |          |Enable PWM trigger ADC function while channel2's counter matching CMP2 in up-count direction
     * |        |          |0 = PWM condition trigger ADC function Disabled.
     * |        |          |1 = PWM condition trigger ADC function Enabled.
     * |        |          |Note: This bit is only valid for PWM in center-aligned type.
     * |        |          |When PWM is in edged-aligned type, setting this bit is meaningless and will not take any effect.
     * |[17]    |CPTRGEN2  |Channel 2 Center Point Trigger ADC Enable Bit
     * |        |          |Enable PWM trigger ADC function while channel2's counter matching PERIOD2
     * |        |          |0 = PWM condition trigger ADC function Disabled.
     * |        |          |1 = PWM condition trigger ADC function Enabled.
     * |        |          |Note: This bit is only valid for PWM in center-aligned type.
     * |        |          |When PWM is in edged-aligned type, setting this bit is meaningless and will not take any effect.
     * |[18]    |CDTRGEN2  |Channel 2 Compare Down Trigger ADC Enable Bit
     * |        |          |Enable PWM trigger ADC function while channel2's counter matching CMP2 in down-count direction
     * |        |          |0 = PWM condition trigger ADC function Disabled.
     * |        |          |1 = PWM condition trigger ADC function Enabled.
     * |        |          |Note: This bit is valid for both center-aligned type and edged-aligned type.
     * |[19]    |ZPTRGEN2  |Channel 2 Zero Point Trigger ADC Enable Bit
     * |        |          |Enable PWM trigger ADC function while channel2's counter matching 0
     * |        |          |0 = PWM condition trigger ADC function Disabled.
     * |        |          |1 = PWM condition trigger ADC function Enabled.
     * |        |          |Note: This bit is valid for both center-aligned type and edged-aligned type.
     * |[24]    |CUTRGEN3  |Channel 3 Compare Up Trigger ADC Enable Bit
     * |        |          |Enable PWM trigger ADC function while channel3's counter matching CMP3 in up-count direction
     * |        |          |0 = PWM condition trigger ADC function Disabled.
     * |        |          |1 = PWM condition trigger ADC function Enabled.
     * |        |          |Note: This bit is only valid for PWM in center-aligned type.
     * |        |          |When PWM is in edged aligned type, setting this bit is meaningless and will not take any effect.
     * |[25]    |CPTRGEN3  |Channel 3 Center Point Trigger ADC Enable Bit
     * |        |          |Enable PWM trigger ADC function while channel3's counter matching PERIOD3
     * |        |          |0 = PWM condition trigger ADC function Disabled.
     * |        |          |1 = PWM condition trigger ADC function Enabled.
     * |        |          |Note: This bit is only valid for PWM in center-aligned type.
     * |        |          |When PWM is in edged aligned type, setting this bit is meaningless and will not take any effect.
     * |[26]    |CDTRGEN3  |Channel 3 Compare Down Trigger ADC Enable Bit
     * |        |          |Enable PWM trigger ADC function while channel3's counter matching CMP3 in down-count direction
     * |        |          |0 = PWM condition trigger ADC function Disabled.
     * |        |          |1 = PWM condition trigger ADC function Enabled.
     * |        |          |Note: This bit is valid for both center-aligned type and edged aligned type.
     * |[27]    |ZPTRGEN3  |Channel 3 Zero Point Trigger ADC Enable Bit
     * |        |          |Enable PWM trigger ADC function while channel3's counter matching 0
     * |        |          |0 = PWM condition trigger ADC function Disabled.
     * |        |          |1 = PWM condition trigger ADC function Enabled.
     * |        |          |Note: This bit is valid for both center-aligned type and edged aligned type.
    */
    __IO uint32_t ADCTCTL0;

    /**
     * ADCTCTL1
     * ===================================================================================================
     * Offset: 0x6C  PWM Trigger Control Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CUTRGEN4  |Channel 4 Compare Up Trigger ADC Enable Bit
     * |        |          |Enable PWM trigger ADC function while channel4's counter matching CMP4 in up-count direction
     * |        |          |0 = PWM condition trigger ADC function Disabled.
     * |        |          |1 = PWM condition trigger ADC function Enabled.
     * |        |          |Note: This bit is only valid for PWM in center-aligned type.
     * |        |          |When PWM is in edged-aligned type, setting this bit is meaningless and will not take any effect.
     * |[1]     |CPTRGEN4  |Channel 4 Center Point Trigger ADC Enable Bit
     * |        |          |Enable PWM trigger ADC function while channel4's counter matching PERIOD4
     * |        |          |0 = PWM condition trigger ADC function Disabled.
     * |        |          |1 = PWM condition trigger ADC function Enabled.
     * |        |          |Note: This bit is only valid for PWM in center-aligned type.
     * |        |          |When PWM is in edged-aligned type, setting this bit is meaningless and will not take any effect.
     * |[2]     |CDTRGEN4  |Channel 4 Compare Down Trigger ADC Enable Bit
     * |        |          |Enable PWM trigger ADC function while channel4's counter matching CMP4 in down-count direction
     * |        |          |0 = PWM condition trigger ADC function Disabled.
     * |        |          |1 = PWM condition trigger ADC function Enabled.
     * |        |          |Note: This bit is valid for both center-aligned type and edged-aligned type.
     * |[3]     |ZPTRGEN4  |Channel 4 Zero Point Trigger ADC Enable Bit
     * |        |          |Enable PWM trigger ADC function while channel4's counter matching 0
     * |        |          |0 = PWM condition trigger ADC function Disabled.
     * |        |          |1 = PWM condition trigger ADC function Enabled.
     * |        |          |Note: This bit is valid for both center-aligned type and edged-aligned type.
     * |[8]     |CUTRGEN5  |Channel 5 Compare Up Trigger ADC Enable Bit
     * |        |          |Enable PWM trigger ADC function while channel5's counter matching CMP5 in up-count direction
     * |        |          |0 = PWM condition trigger ADC function Disabled.
     * |        |          |1 = PWM condition trigger ADC function Enabled.
     * |        |          |Note: This bit is only valid for PWM in center-aligned type.
     * |        |          |When PWM is in edged-aligned type, setting this bit is meaningless and will not take any effect.
     * |[9]     |CPTRGEN5  |Channel 5 Center Point Trigger ADC Enable Bit
     * |        |          |Enable PWM trigger ADC function while channel5's counter matching PERIOD5
     * |        |          |0 = PWM condition trigger ADC function Disabled.
     * |        |          |1 = PWM condition trigger ADC function Enabled.
     * |        |          |Note: This bit is only valid for PWM in center-aligned type.
     * |        |          |When PWM is in edged-aligned type, setting this bit is meaningless and will not take any effect.
     * |[10]    |CDTRGEN5  |Channel 5 Compare Down Trigger ADC Enable Bit
     * |        |          |Enable PWM trigger ADC function while channel5's counter matching CMP5 in down-count direction
     * |        |          |0 = PWM condition trigger ADC function Disabled.
     * |        |          |1 = PWM condition trigger ADC function Enabled.
     * |        |          |Note: This bit is valid for both center-aligned type and edged-aligned type.
     * |[11]    |ZPTRGEN5  |Channel 5 Zero Point Trigger ADC Enable Bit
     * |        |          |Enable PWM trigger ADC function while channel5's counter matching 0
     * |        |          |0 = PWM condition trigger ADC function Disabled.
     * |        |          |1 = PWM condition trigger ADC function Enabled.
     * |        |          |Note: This bit is valid for both center-aligned type and edged-aligned type.
    * |[16]    |CUTRGEN6  |Channel 6 Compare Up Trigger ADC Enable Bit
    * |        |          |Enable PWM trigger ADC function while channel6's counter matching CMP6 in up-count direction
    * |        |          |0 = PWM condition trigger ADC function Disabled.
    * |        |          |1 = PWM condition trigger ADC function Enabled.
    * |        |          |Note: This bit is only valid for PWM in center-aligned type.
    * |        |          |When PWM is in edged-aligned type, setting this bit is meaningless and will not take any effect.
    * |[17]    |CPTRGEN6  |Channel 6 Center Point Trigger ADC Enable Bit
    * |        |          |Enable PWM trigger ADC function while channel6's counter matching PERIOD6
    * |        |          |0 = PWM condition trigger ADC function Disabled.
    * |        |          |1 = PWM condition trigger ADC function Enabled.
    * |        |          |Note: This bit is only valid for PWM in center-aligned type.
    * |        |          |When PWM is in edged-aligned type, setting this bit is meaningless and will not take any effect.
    * |[18]    |CDTRGEN6  |Channel 6 Compare Down Trigger ADC Enable Bit
    * |        |          |Enable PWM trigger ADC function while channel6's counter matching CMP6 in down-count direction
    * |        |          |0 = PWM condition trigger ADC function Disabled.
    * |        |          |1 = PWM condition trigger ADC function Enabled.
    * |        |          |Note: This bit is valid for both center-aligned type and edged-aligned type.
    * |[19]    |ZPTRGEN6  |Channel 6 Zero Point Trigger ADC Enable Bit
    * |        |          |Enable PWM trigger ADC function while channel6's counter matching 0
    * |        |          |0 = PWM condition trigger ADC function Disabled.
    * |        |          |1 = PWM condition trigger ADC function Enabled.
    * |        |          |Note: This bit is valid for both center-aligned type and edged-aligned type.
    * |[24]    |CUTRGEN7  |Channel 7 Compare Up Trigger ADC Enable Bit
    * |        |          |Enable PWM trigger ADC function while channel7's counter matching CMP7 in up-count direction
    * |        |          |0 = PWM condition trigger ADC function Disabled.
    * |        |          |1 = PWM condition trigger ADC function Enabled.
    * |        |          |Note: This bit is only valid for PWM in center-aligned type.
    * |        |          |When PWM is in edged aligned type, setting this bit is meaningless and will not take any effect.
    * |[25]    |CPTRGEN7  |Channel 7 Center Point Trigger ADC Enable Bit
    * |        |          |Enable PWM trigger ADC function while channel7's counter matching PERIOD7
    * |        |          |0 = PWM condition trigger ADC function Disabled.
    * |        |          |1 = PWM condition trigger ADC function Enabled.
    * |        |          |Note: This bit is only valid for PWM in center-aligned type.
    * |        |          |When PWM is in edged aligned type, setting this bit is meaningless and will not take any effect.
    * |[26]    |CDTRGEN7  |Channel 7 Compare Down Trigger ADC Enable Bit
    * |        |          |Enable PWM trigger ADC function while channel7's counter matching CMP7 in down-count direction
    * |        |          |0 = PWM condition trigger ADC function Disabled.
    * |        |          |1 = PWM condition trigger ADC function Enabled.
    * |        |          |Note: This bit is valid for both center-aligned type and edged aligned type.
    * |[27]    |ZPTRGEN7  |Channel 7 Zero Point Trigger ADC Enable Bit
    * |        |          |Enable PWM trigger ADC function while channel7's counter matching 0
    * |        |          |0 = PWM condition trigger ADC function Disabled.
    * |        |          |1 = PWM condition trigger ADC function Enabled.
    * |        |          |Note: This bit is valid for both center-aligned type and edged aligned type.
    */
    __IO uint32_t ADCTCTL1;

    /**
     * ADCTSTS0
     * ===================================================================================================
     * Offset: 0x70  PWM Trigger Status Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CUTRGF0   |Channel 0 Compare Up Trigger ADC Flag
     * |        |          |When the channel0's counter is counting up to CMP0, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[1]     |CPTRGF0   |Channel 0 Center Point Trigger ADC Flag
     * |        |          |When the channel0's counter is counting to PERIOD0, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[2]     |CDTRGF0   |Channel 0 Compare Down Trigger ADC Flag
     * |        |          |When the channel0's counter is counting down to CMP0, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[3]     |ZPTRGF0   |Channel 0 Zero Point Trigger ADC Flag
     * |        |          |When the channel0's counter is counting to zero point, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[8]     |CUTRGF1   |Channel 1 Compare Up Trigger ADC Flag
     * |        |          |When the channel1's counter is counting up to CMP1, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[9]     |CPTRGF1   |Channel 1 Center Point Trigger ADC Flag
     * |        |          |When the channel1's counter is counting to PERIOD1, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[10]    |CDTRGF1   |Channel 1 Compare Down Trigger ADC Flag
     * |        |          |When the channel1's counter is counting down to CMP1, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[11]    |ZPTRGF1   |Channel 1 Zero Point Trigger ADC Flag
     * |        |          |When the channel1's counter is counting to zero point, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[16]    |CUTRGF2   |Channel 2 Compare Up Trigger ADC Flag
     * |        |          |When the channel2's counter is counting up to CMP2, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[17]    |CPTRGF2   |Channel 2 Center Point Trigger ADC Flag
     * |        |          |When the channel2's counter is counting to PERIOD2, this bit will be set for trigger ADC. Note:
     * |        |          |This bit can be cleared by software writing 1.
     * |[18]    |CDTRGF2   |Channel 2 Compare Down Trigger ADC Flag
     * |        |          |When the channel2's counter is counting down to CMP2, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[19]    |ZPTRGF2   |Channel 2 Zero Point Trigger ADC Enable Bit
     * |        |          |When the channel2's counter is counting to zero point, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[24]    |CUTRGF3   |Channel 3 Compare Up Trigger ADC Flag
     * |        |          |When the channel3's counter is counting up to CMP3, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[25]    |CPTRGF3   |Channel 3 Center Point Trigger ADC Flag
     * |        |          |When the channel3's counter is counting to PERIOD3, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[26]    |CDTRGF3   |Channel 3 Compare Down Trigger ADC Flag
     * |        |          |When the channel3's counter is counting down to CMP3, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[27]    |ZPTRGF3   |Channel 3 Zero Point Trigger ADC Flag
     * |        |          |When the channel3's counter is counting to zero point, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
    */
    __IO uint32_t ADCTSTS0;

    /**
     * ADCTSTS1
     * ===================================================================================================
     * Offset: 0x74  PWM Trigger Status Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CUTRGF4   |Channel 4 Compare Up Trigger ADC Flag
     * |        |          |When the channel4's counter is counting up to CMP4, this bit will be set for trigger ADC. Note:
     * |        |          |This bit can be cleared by software writing 1.
     * |[1]     |CPTRGF4   |Channel 4 Center Point Trigger ADC Flag
     * |        |          |When the channel4's counter is counting to PERIOD4, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[2]     |CDTRGF4   |Channel 4 Compare Down Trigger ADC Flag
     * |        |          |When the channel4's counter is counting down to CMP4, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[3]     |ZPTRGF4   |Channel 4 Zero Point Trigger ADC Flag
     * |        |          |When the channel4's counter is counting to zero point, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[8]     |CUTRGF5   |Channel 5 Compare Up Trigger ADC Flag
     * |        |          |When the channel5's counter is counting up to CMP5, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[9]     |CPTRGF5   |Channel 5 Center Point Trigger ADC Flag
     * |        |          |When the channel5's counter is counting to PERIOD5, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[10]    |CDTRGF5   |Channel 5 Compare Down Trigger ADC Flag
     * |        |          |When the channel5's counter is counting down to CMP5, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[11]    |ZPTRGF5   |Channel 5 Zero Point Trigger ADC Flag
     * |        |          |When the channel5's counter is counting to zero point, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[16]    |CUTRGF6   |Channel 6 Compare Up Trigger ADC Flag
     * |        |          |When the channel6's counter is counting up to CMP6, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[17]    |CPTRGF6   |Channel 6 Center Point Trigger ADC Flag
     * |        |          |When the channel6's counter is counting to PERIOD6, this bit will be set for trigger ADC. Note:
     * |        |          |This bit can be cleared by software writing 1.
     * |[18]    |CDTRGF6   |Channel 6 Compare Down Trigger ADC Flag
     * |        |          |When the channel6's counter is counting down to CMP6, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[19]    |ZPTRGF6   |Channel 6 Zero Point Trigger ADC Enable Bit
     * |        |          |When the channel6's counter is counting to zero point, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[24]    |CUTRGF7   |Channel 7 Compare Up Trigger ADC Flag
     * |        |          |When the channel7's counter is counting up to CMP7, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[25]    |CPTRGF7   |Channel 7 Center Point Trigger ADC Flag
     * |        |          |When the channel7's counter is counting to PERIOD7, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[26]    |CDTRGF7   |Channel 7 Compare Down Trigger ADC Flag
     * |        |          |When the channel7's counter is counting down to CMP7, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
     * |[27]    |ZPTRGF7   |Channel 7 Zero Point Trigger ADC Flag
     * |        |          |When the channel7's counter is counting to zero point, this bit will be set for trigger ADC.
     * |        |          |Note: This bit can be cleared by software writing 1.
    */
    __IO uint32_t ADCTSTS1;


    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVED1[4];
    /// @endcond //HIDDEN_SYMBOLS


    /**
     * PCACTL
     * ===================================================================================================
     * Offset: 0x88  PWM Precise Center-Aligned Type Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PCAEN     |PWM Precise Center-aligned Type Enable Bit
     * |        |          |0 = Precise center-aligned type Disabled.
     * |        |          |1 = Precise center-aligned type Enabled.
    */
    __IO uint32_t PCACTL;


} PWM_T;

/**
    @addtogroup PWM_CONST PWM Bit Field Definition
    Constant Definitions for PWM Controller
@{ */

#define PWM_CLKPSC_CLKPSC01_Pos          (0)                                               /*!< PWM_T::CLKPSC: CLKPSC01 Position          */
#define PWM_CLKPSC_CLKPSC01_Msk          (0xfful << PWM_CLKPSC_CLKPSC01_Pos)               /*!< PWM_T::CLKPSC: CLKPSC01 Mask              */

#define PWM_CLKPSC_CLKPSC23_Pos          (8)                                               /*!< PWM_T::CLKPSC: CLKPSC23 Position          */
#define PWM_CLKPSC_CLKPSC23_Msk          (0xfful << PWM_CLKPSC_CLKPSC23_Pos)               /*!< PWM_T::CLKPSC: CLKPSC23 Mask              */

#define PWM_CLKPSC_CLKPSC45_Pos          (16)                                              /*!< PWM_T::CLKPSC: CLKPSC45 Position          */
#define PWM_CLKPSC_CLKPSC45_Msk          (0xfful << PWM_CLKPSC_CLKPSC45_Pos)               /*!< PWM_T::CLKPSC: CLKPSC45 Mask              */

#define PWM_CLKPSC_CLKPSC67_Pos          (24)                                              /*!< PWM_T::CLKPSC: CLKPSC67 Position          */
#define PWM_CLKPSC_CLKPSC67_Msk          (0xfful << PWM_CLKPSC_CLKPSC67_Pos)               /*!< PWM_T::CLKPSC: CLKPSC67 Mask              */

#define PWM_CLKDIV_CLKDIV0_Pos           (0)                                               /*!< PWM_T::CLKDIV: CLKDIV0 Position           */
#define PWM_CLKDIV_CLKDIV0_Msk           (0x7ul << PWM_CLKDIV_CLKDIV0_Pos)                 /*!< PWM_T::CLKDIV: CLKDIV0 Mask               */

#define PWM_CLKDIV_CLKDIV1_Pos           (4)                                               /*!< PWM_T::CLKDIV: CLKDIV1 Position           */
#define PWM_CLKDIV_CLKDIV1_Msk           (0x7ul << PWM_CLKDIV_CLKDIV1_Pos)                 /*!< PWM_T::CLKDIV: CLKDIV1 Mask               */

#define PWM_CLKDIV_CLKDIV2_Pos           (8)                                               /*!< PWM_T::CLKDIV: CLKDIV2 Position           */
#define PWM_CLKDIV_CLKDIV2_Msk           (0x7ul << PWM_CLKDIV_CLKDIV2_Pos)                 /*!< PWM_T::CLKDIV: CLKDIV2 Mask               */

#define PWM_CLKDIV_CLKDIV3_Pos           (12)                                              /*!< PWM_T::CLKDIV: CLKDIV3 Position           */
#define PWM_CLKDIV_CLKDIV3_Msk           (0x7ul << PWM_CLKDIV_CLKDIV3_Pos)                 /*!< PWM_T::CLKDIV: CLKDIV3 Mask               */

#define PWM_CLKDIV_CLKDIV4_Pos           (16)                                              /*!< PWM_T::CLKDIV: CLKDIV4 Position           */
#define PWM_CLKDIV_CLKDIV4_Msk           (0x7ul << PWM_CLKDIV_CLKDIV4_Pos)                 /*!< PWM_T::CLKDIV: CLKDIV4 Mask               */

#define PWM_CLKDIV_CLKDIV5_Pos           (20)                                              /*!< PWM_T::CLKDIV: CLKDIV5 Position           */
#define PWM_CLKDIV_CLKDIV5_Msk           (0x7ul << PWM_CLKDIV_CLKDIV5_Pos)                 /*!< PWM_T::CLKDIV: CLKDIV5 Mask               */


#define PWM_CLKDIV_CLKDIV6_Pos           (24)                                              /*!< PWM_T::CLKDIV: CLKDIV6 Position           */
#define PWM_CLKDIV_CLKDIV6_Msk           (0x7ul << PWM_CLKDIV_CLKDIV6_Pos)                 /*!< PWM_T::CLKDIV: CLKDIV6 Mask               */

#define PWM_CLKDIV_CLKDIV7_Pos           (28)                                              /*!< PWM_T::CLKDIV: CLKDIV7 Position           */
#define PWM_CLKDIV_CLKDIV7_Msk           (0x7ul << PWM_CLKDIV_CLKDIV7_Pos)                 /*!< PWM_T::CLKDIV: CLKDIV7 Mask               */

#define PWM_CTL_CNTEN0_Pos               (0)                                               /*!< PWM_T::CTL: CNTEN0 Position               */
#define PWM_CTL_CNTEN0_Msk               (0x1ul << PWM_CTL_CNTEN0_Pos)                     /*!< PWM_T::CTL: CNTEN0 Mask                   */

#define PWM_CTL_PINV0_Pos                (2)                                               /*!< PWM_T::CTL: PINV0 Position                */
#define PWM_CTL_PINV0_Msk                (0x1ul << PWM_CTL_PINV0_Pos)                      /*!< PWM_T::CTL: PINV0 Mask                    */

#define PWM_CTL_CNTMODE0_Pos             (3)                                               /*!< PWM_T::CTL: CNTMODE0 Position             */
#define PWM_CTL_CNTMODE0_Msk             (0x1ul << PWM_CTL_CNTMODE0_Pos)                   /*!< PWM_T::CTL: CNTMODE0 Mask                 */

#define PWM_CTL_CNTEN1_Pos               (4)                                               /*!< PWM_T::CTL: CNTEN1 Position               */
#define PWM_CTL_CNTEN1_Msk               (0x1ul << PWM_CTL_CNTEN1_Pos)                     /*!< PWM_T::CTL: CNTEN1 Mask                   */

#define PWM_CTL_HCUPDT_Pos               (5)                                               /*!< PWM_T::CTL: HCUPDT Position               */
#define PWM_CTL_HCUPDT_Msk               (0x1ul << PWM_CTL_HCUPDT_Pos)                     /*!< PWM_T::CTL: HCUPDT Mask                   */

#define PWM_CTL_PINV1_Pos                (6)                                               /*!< PWM_T::CTL: PINV1 Position                */
#define PWM_CTL_PINV1_Msk                (0x1ul << PWM_CTL_PINV1_Pos)                      /*!< PWM_T::CTL: PINV1 Mask                    */

#define PWM_CTL_CNTMODE1_Pos             (7)                                               /*!< PWM_T::CTL: CNTMODE1 Position             */
#define PWM_CTL_CNTMODE1_Msk             (0x1ul << PWM_CTL_CNTMODE1_Pos)                   /*!< PWM_T::CTL: CNTMODE1 Mask                 */

#define PWM_CTL_CNTEN2_Pos               (8)                                               /*!< PWM_T::CTL: CNTEN2 Position               */
#define PWM_CTL_CNTEN2_Msk               (0x1ul << PWM_CTL_CNTEN2_Pos)                     /*!< PWM_T::CTL: CNTEN2 Mask                   */

#define PWM_CTL_PINV2_Pos                (10)                                              /*!< PWM_T::CTL: PINV2 Position                */
#define PWM_CTL_PINV2_Msk                (0x1ul << PWM_CTL_PINV2_Pos)                      /*!< PWM_T::CTL: PINV2 Mask                    */

#define PWM_CTL_CNTMODE2_Pos             (11)                                              /*!< PWM_T::CTL: CNTMODE2 Position             */
#define PWM_CTL_CNTMODE2_Msk             (0x1ul << PWM_CTL_CNTMODE2_Pos)                   /*!< PWM_T::CTL: CNTMODE2 Mask                 */

#define PWM_CTL_CNTEN3_Pos               (12)                                              /*!< PWM_T::CTL: CNTEN3 Position               */
#define PWM_CTL_CNTEN3_Msk               (0x1ul << PWM_CTL_CNTEN3_Pos)                     /*!< PWM_T::CTL: CNTEN3 Mask                   */

#define PWM_CTL_PINV3_Pos                (14)                                              /*!< PWM_T::CTL: PINV3 Position                */
#define PWM_CTL_PINV3_Msk                (0x1ul << PWM_CTL_PINV3_Pos)                      /*!< PWM_T::CTL: PINV3 Mask                    */

#define PWM_CTL_CNTMODE3_Pos             (15)                                              /*!< PWM_T::CTL: CNTMODE3 Position             */
#define PWM_CTL_CNTMODE3_Msk             (0x1ul << PWM_CTL_CNTMODE3_Pos)                   /*!< PWM_T::CTL: CNTMODE3 Mask                 */

#define PWM_CTL_CNTEN4_Pos               (16)                                              /*!< PWM_T::CTL: CNTEN4 Position               */
#define PWM_CTL_CNTEN4_Msk               (0x1ul << PWM_CTL_CNTEN4_Pos)                     /*!< PWM_T::CTL: CNTEN4 Mask                   */

#define PWM_CTL_PINV4_Pos                (18)                                              /*!< PWM_T::CTL: PINV4 Position                */
#define PWM_CTL_PINV4_Msk                (0x1ul << PWM_CTL_PINV4_Pos)                      /*!< PWM_T::CTL: PINV4 Mask                    */

#define PWM_CTL_CNTMODE4_Pos             (19)                                              /*!< PWM_T::CTL: CNTMODE4 Position             */
#define PWM_CTL_CNTMODE4_Msk             (0x1ul << PWM_CTL_CNTMODE4_Pos)                   /*!< PWM_T::CTL: CNTMODE4 Mask                 */

#define PWM_CTL_CNTEN5_Pos               (20)                                              /*!< PWM_T::CTL: CNTEN5 Position               */
#define PWM_CTL_CNTEN5_Msk               (0x1ul << PWM_CTL_CNTEN5_Pos)                     /*!< PWM_T::CTL: CNTEN5 Mask                   */

#define PWM_CTL_ASYMEN_Pos               (21)                                              /*!< PWM_T::CTL: ASYMEN Position               */
#define PWM_CTL_ASYMEN_Msk               (0x1ul << PWM_CTL_ASYMEN_Pos)                     /*!< PWM_T::CTL: ASYMEN Mask                   */

#define PWM_CTL_PINV5_Pos                (22)                                              /*!< PWM_T::CTL: PINV5 Position                */
#define PWM_CTL_PINV5_Msk                (0x1ul << PWM_CTL_PINV5_Pos)                      /*!< PWM_T::CTL: PINV5 Mask                    */

#define PWM_CTL_CNTMODE5_Pos             (23)                                              /*!< PWM_T::CTL: CNTMODE5 Position             */
#define PWM_CTL_CNTMODE5_Msk             (0x1ul << PWM_CTL_CNTMODE5_Pos)                   /*!< PWM_T::CTL: CNTMODE5 Mask                 */

#define PWM_CTL_CNTEN6_Pos               (24)                                              /*!< PWM_T::CTL: CNTEN6 Position               */
#define PWM_CTL_CNTEN6_Msk               (0x1ul << PWM_CTL_CNTEN6_Pos)                     /*!< PWM_T::CTL: CNTEN6 Mask                   */

#define PWM_CTL_PINV6_Pos                (26)                                              /*!< PWM_T::CTL: PINV6 Position                */
#define PWM_CTL_PINV6_Msk                (0x1ul << PWM_CTL_PINV6_Pos)                      /*!< PWM_T::CTL: PINV6 Mask                    */

#define PWM_CTL_CNTMODE6_Pos             (27)                                              /*!< PWM_T::CTL: CNTMODE6 Position             */
#define PWM_CTL_CNTMODE6_Msk             (0x1ul << PWM_CTL_CNTMODE6_Pos)                   /*!< PWM_T::CTL: CNTMODE6 Mask                 */

#define PWM_CTL_CNTEN7_Pos               (28)                                              /*!< PWM_T::CTL: CNTEN7 Position               */
#define PWM_CTL_CNTEN7_Msk               (0x1ul << PWM_CTL_CNTEN7_Pos)                     /*!< PWM_T::CTL: CNTEN7 Mask                   */

#define PWM_CTL_PINV7_Pos                (30)                                              /*!< PWM_T::CTL: PINV7 Position                */
#define PWM_CTL_PINV7_Msk                (0x1ul << PWM_CTL_PINV7_Pos)                      /*!< PWM_T::CTL: PINV7 Mask                    */

#define PWM_CTL_CNTMODE7_Pos             (31)                                              /*!< PWM_T::CTL: CNTMODE7 Position             */
#define PWM_CTL_CNTMODE7_Msk             (0x1ul << PWM_CTL_CNTMODE7_Pos)                   /*!< PWM_T::CTL: CNTMODE7 Mask                 */

#define PWM_PERIOD0_PERIOD0_Pos          (0)                                               /*!< PWM_T::PERIOD0: PERIOD0 Position          */
#define PWM_PERIOD0_PERIOD0_Msk          (0xfffful << PWM_PERIOD0_PERIOD0_Pos)             /*!< PWM_T::PERIOD0: PERIOD0 Mask              */

#define PWM_PERIOD0_PERIOD1_Pos          (0)                                               /*!< PWM_T::PERIOD0: PERIOD1 Position          */
#define PWM_PERIOD0_PERIOD1_Msk          (0xfffful << PWM_PERIOD0_PERIOD1_Pos)             /*!< PWM_T::PERIOD0: PERIOD1 Mask              */

#define PWM_PERIOD0_PERIOD2_Pos          (0)                                               /*!< PWM_T::PERIOD0: PERIOD2 Position          */
#define PWM_PERIOD0_PERIOD2_Msk          (0xfffful << PWM_PERIOD0_PERIOD2_Pos)             /*!< PWM_T::PERIOD0: PERIOD2 Mask              */

#define PWM_PERIOD0_PERIOD3_Pos          (0)                                               /*!< PWM_T::PERIOD0: PERIOD3 Position          */
#define PWM_PERIOD0_PERIOD3_Msk          (0xfffful << PWM_PERIOD0_PERIOD3_Pos)             /*!< PWM_T::PERIOD0: PERIOD3 Mask              */

#define PWM_PERIOD0_PERIOD4_Pos          (0)                                               /*!< PWM_T::PERIOD0: PERIOD4 Position          */
#define PWM_PERIOD0_PERIOD4_Msk          (0xfffful << PWM_PERIOD0_PERIOD4_Pos)             /*!< PWM_T::PERIOD0: PERIOD4 Mask              */

#define PWM_PERIOD0_PERIOD5_Pos          (0)                                               /*!< PWM_T::PERIOD0: PERIOD5 Position          */
#define PWM_PERIOD0_PERIOD5_Msk          (0xfffful << PWM_PERIOD0_PERIOD5_Pos)             /*!< PWM_T::PERIOD0: PERIOD5 Mask              */

#define PWM_PERIOD0_PERIOD6_Pos          (0)                                               /*!< PWM_T::PERIOD0: PERIOD6 Position          */
#define PWM_PERIOD0_PERIOD6_Msk          (0xfffful << PWM_PERIOD0_PERIOD6_Pos)             /*!< PWM_T::PERIOD0: PERIOD6 Mask              */

#define PWM_PERIOD0_PERIOD7_Pos          (0)                                               /*!< PWM_T::PERIOD0: PERIOD7 Position          */
#define PWM_PERIOD0_PERIOD7_Msk          (0xfffful << PWM_PERIOD0_PERIOD7_Pos)             /*!< PWM_T::PERIOD0: PERIOD7 Mask              */

#define PWM_PERIOD1_PERIOD0_Pos          (0)                                               /*!< PWM_T::PERIOD1: PERIOD0 Position          */
#define PWM_PERIOD1_PERIOD0_Msk          (0xfffful << PWM_PERIOD1_PERIOD0_Pos)             /*!< PWM_T::PERIOD1: PERIOD0 Mask              */

#define PWM_PERIOD1_PERIOD1_Pos          (0)                                               /*!< PWM_T::PERIOD1: PERIOD1 Position          */
#define PWM_PERIOD1_PERIOD1_Msk          (0xfffful << PWM_PERIOD1_PERIOD1_Pos)             /*!< PWM_T::PERIOD1: PERIOD1 Mask              */

#define PWM_PERIOD1_PERIOD2_Pos          (0)                                               /*!< PWM_T::PERIOD1: PERIOD2 Position          */
#define PWM_PERIOD1_PERIOD2_Msk          (0xfffful << PWM_PERIOD1_PERIOD2_Pos)             /*!< PWM_T::PERIOD1: PERIOD2 Mask              */

#define PWM_PERIOD1_PERIOD3_Pos          (0)                                               /*!< PWM_T::PERIOD1: PERIOD3 Position          */
#define PWM_PERIOD1_PERIOD3_Msk          (0xfffful << PWM_PERIOD1_PERIOD3_Pos)             /*!< PWM_T::PERIOD1: PERIOD3 Mask              */

#define PWM_PERIOD1_PERIOD4_Pos          (0)                                               /*!< PWM_T::PERIOD1: PERIOD4 Position          */
#define PWM_PERIOD1_PERIOD4_Msk          (0xfffful << PWM_PERIOD1_PERIOD4_Pos)             /*!< PWM_T::PERIOD1: PERIOD4 Mask              */

#define PWM_PERIOD1_PERIOD5_Pos          (0)                                               /*!< PWM_T::PERIOD1: PERIOD5 Position          */
#define PWM_PERIOD1_PERIOD5_Msk          (0xfffful << PWM_PERIOD1_PERIOD5_Pos)             /*!< PWM_T::PERIOD1: PERIOD5 Mask              */

#define PWM_PERIOD1_PERIOD6_Pos          (0)                                               /*!< PWM_T::PERIOD1: PERIOD6 Position          */
#define PWM_PERIOD1_PERIOD6_Msk          (0xfffful << PWM_PERIOD1_PERIOD6_Pos)             /*!< PWM_T::PERIOD1: PERIOD6 Mask              */

#define PWM_PERIOD1_PERIOD7_Pos          (0)                                               /*!< PWM_T::PERIOD1: PERIOD7 Position          */
#define PWM_PERIOD1_PERIOD7_Msk          (0xfffful << PWM_PERIOD1_PERIOD7_Pos)             /*!< PWM_T::PERIOD1: PERIOD7 Mask              */

#define PWM_PERIOD2_PERIOD0_Pos          (0)                                               /*!< PWM_T::PERIOD2: PERIOD0 Position          */
#define PWM_PERIOD2_PERIOD0_Msk          (0xfffful << PWM_PERIOD2_PERIOD0_Pos)             /*!< PWM_T::PERIOD2: PERIOD0 Mask              */

#define PWM_PERIOD2_PERIOD1_Pos          (0)                                               /*!< PWM_T::PERIOD2: PERIOD1 Position          */
#define PWM_PERIOD2_PERIOD1_Msk          (0xfffful << PWM_PERIOD2_PERIOD1_Pos)             /*!< PWM_T::PERIOD2: PERIOD1 Mask              */

#define PWM_PERIOD2_PERIOD2_Pos          (0)                                               /*!< PWM_T::PERIOD2: PERIOD2 Position          */
#define PWM_PERIOD2_PERIOD2_Msk          (0xfffful << PWM_PERIOD2_PERIOD2_Pos)             /*!< PWM_T::PERIOD2: PERIOD2 Mask              */

#define PWM_PERIOD2_PERIOD3_Pos          (0)                                               /*!< PWM_T::PERIOD2: PERIOD3 Position          */
#define PWM_PERIOD2_PERIOD3_Msk          (0xfffful << PWM_PERIOD2_PERIOD3_Pos)             /*!< PWM_T::PERIOD2: PERIOD3 Mask              */

#define PWM_PERIOD2_PERIOD4_Pos          (0)                                               /*!< PWM_T::PERIOD2: PERIOD4 Position          */
#define PWM_PERIOD2_PERIOD4_Msk          (0xfffful << PWM_PERIOD2_PERIOD4_Pos)             /*!< PWM_T::PERIOD2: PERIOD4 Mask              */

#define PWM_PERIOD2_PERIOD5_Pos          (0)                                               /*!< PWM_T::PERIOD2: PERIOD5 Position          */
#define PWM_PERIOD2_PERIOD5_Msk          (0xfffful << PWM_PERIOD2_PERIOD5_Pos)             /*!< PWM_T::PERIOD2: PERIOD5 Mask              */

#define PWM_PERIOD2_PERIOD6_Pos          (0)                                               /*!< PWM_T::PERIOD2: PERIOD6 Position          */
#define PWM_PERIOD2_PERIOD6_Msk          (0xfffful << PWM_PERIOD2_PERIOD6_Pos)             /*!< PWM_T::PERIOD2: PERIOD6 Mask              */

#define PWM_PERIOD2_PERIOD7_Pos          (0)                                               /*!< PWM_T::PERIOD2: PERIOD7 Position          */
#define PWM_PERIOD2_PERIOD7_Msk          (0xfffful << PWM_PERIOD2_PERIOD7_Pos)             /*!< PWM_T::PERIOD2: PERIOD7 Mask              */

#define PWM_PERIOD3_PERIOD0_Pos          (0)                                               /*!< PWM_T::PERIOD3: PERIOD0 Position          */
#define PWM_PERIOD3_PERIOD0_Msk          (0xfffful << PWM_PERIOD3_PERIOD0_Pos)             /*!< PWM_T::PERIOD3: PERIOD0 Mask              */

#define PWM_PERIOD3_PERIOD1_Pos          (0)                                               /*!< PWM_T::PERIOD3: PERIOD1 Position          */
#define PWM_PERIOD3_PERIOD1_Msk          (0xfffful << PWM_PERIOD3_PERIOD1_Pos)             /*!< PWM_T::PERIOD3: PERIOD1 Mask              */

#define PWM_PERIOD3_PERIOD2_Pos          (0)                                               /*!< PWM_T::PERIOD3: PERIOD2 Position          */
#define PWM_PERIOD3_PERIOD2_Msk          (0xfffful << PWM_PERIOD3_PERIOD2_Pos)             /*!< PWM_T::PERIOD3: PERIOD2 Mask              */

#define PWM_PERIOD3_PERIOD3_Pos          (0)                                               /*!< PWM_T::PERIOD3: PERIOD3 Position          */
#define PWM_PERIOD3_PERIOD3_Msk          (0xfffful << PWM_PERIOD3_PERIOD3_Pos)             /*!< PWM_T::PERIOD3: PERIOD3 Mask              */

#define PWM_PERIOD3_PERIOD4_Pos          (0)                                               /*!< PWM_T::PERIOD3: PERIOD4 Position          */
#define PWM_PERIOD3_PERIOD4_Msk          (0xfffful << PWM_PERIOD3_PERIOD4_Pos)             /*!< PWM_T::PERIOD3: PERIOD4 Mask              */

#define PWM_PERIOD3_PERIOD5_Pos          (0)                                               /*!< PWM_T::PERIOD3: PERIOD5 Position          */
#define PWM_PERIOD3_PERIOD5_Msk          (0xfffful << PWM_PERIOD3_PERIOD5_Pos)             /*!< PWM_T::PERIOD3: PERIOD5 Mask              */

#define PWM_PERIOD3_PERIOD6_Pos          (0)                                               /*!< PWM_T::PERIOD3: PERIOD6 Position          */
#define PWM_PERIOD3_PERIOD6_Msk          (0xfffful << PWM_PERIOD3_PERIOD6_Pos)             /*!< PWM_T::PERIOD3: PERIOD6 Mask              */

#define PWM_PERIOD3_PERIOD7_Pos          (0)                                               /*!< PWM_T::PERIOD3: PERIOD7 Position          */
#define PWM_PERIOD3_PERIOD7_Msk          (0xfffful << PWM_PERIOD3_PERIOD7_Pos)             /*!< PWM_T::PERIOD3: PERIOD7 Mask              */

#define PWM_PERIOD4_PERIOD0_Pos          (0)                                               /*!< PWM_T::PERIOD4: PERIOD0 Position          */
#define PWM_PERIOD4_PERIOD0_Msk          (0xfffful << PWM_PERIOD4_PERIOD0_Pos)             /*!< PWM_T::PERIOD4: PERIOD0 Mask              */

#define PWM_PERIOD4_PERIOD1_Pos          (0)                                               /*!< PWM_T::PERIOD4: PERIOD1 Position          */
#define PWM_PERIOD4_PERIOD1_Msk          (0xfffful << PWM_PERIOD4_PERIOD1_Pos)             /*!< PWM_T::PERIOD4: PERIOD1 Mask              */

#define PWM_PERIOD4_PERIOD2_Pos          (0)                                               /*!< PWM_T::PERIOD4: PERIOD2 Position          */
#define PWM_PERIOD4_PERIOD2_Msk          (0xfffful << PWM_PERIOD4_PERIOD2_Pos)             /*!< PWM_T::PERIOD4: PERIOD2 Mask              */

#define PWM_PERIOD4_PERIOD3_Pos          (0)                                               /*!< PWM_T::PERIOD4: PERIOD3 Position          */
#define PWM_PERIOD4_PERIOD3_Msk          (0xfffful << PWM_PERIOD4_PERIOD3_Pos)             /*!< PWM_T::PERIOD4: PERIOD3 Mask              */

#define PWM_PERIOD4_PERIOD4_Pos          (0)                                               /*!< PWM_T::PERIOD4: PERIOD4 Position          */
#define PWM_PERIOD4_PERIOD4_Msk          (0xfffful << PWM_PERIOD4_PERIOD4_Pos)             /*!< PWM_T::PERIOD4: PERIOD4 Mask              */

#define PWM_PERIOD4_PERIOD5_Pos          (0)                                               /*!< PWM_T::PERIOD4: PERIOD5 Position          */
#define PWM_PERIOD4_PERIOD5_Msk          (0xfffful << PWM_PERIOD4_PERIOD5_Pos)             /*!< PWM_T::PERIOD4: PERIOD5 Mask              */

#define PWM_PERIOD4_PERIOD6_Pos          (0)                                               /*!< PWM_T::PERIOD4: PERIOD6 Position          */
#define PWM_PERIOD4_PERIOD6_Msk          (0xfffful << PWM_PERIOD4_PERIOD6_Pos)             /*!< PWM_T::PERIOD4: PERIOD6 Mask              */

#define PWM_PERIOD4_PERIOD7_Pos          (0)                                               /*!< PWM_T::PERIOD4: PERIOD7 Position          */
#define PWM_PERIOD4_PERIOD7_Msk          (0xfffful << PWM_PERIOD4_PERIOD7_Pos)             /*!< PWM_T::PERIOD4: PERIOD7 Mask              */

#define PWM_PERIOD5_PERIOD0_Pos          (0)                                               /*!< PWM_T::PERIOD5: PERIOD0 Position          */
#define PWM_PERIOD5_PERIOD0_Msk          (0xfffful << PWM_PERIOD5_PERIOD0_Pos)             /*!< PWM_T::PERIOD5: PERIOD0 Mask              */

#define PWM_PERIOD5_PERIOD1_Pos          (0)                                               /*!< PWM_T::PERIOD5: PERIOD1 Position          */
#define PWM_PERIOD5_PERIOD1_Msk          (0xfffful << PWM_PERIOD5_PERIOD1_Pos)             /*!< PWM_T::PERIOD5: PERIOD1 Mask              */

#define PWM_PERIOD5_PERIOD2_Pos          (0)                                               /*!< PWM_T::PERIOD5: PERIOD2 Position          */
#define PWM_PERIOD5_PERIOD2_Msk          (0xfffful << PWM_PERIOD5_PERIOD2_Pos)             /*!< PWM_T::PERIOD5: PERIOD2 Mask              */

#define PWM_PERIOD5_PERIOD3_Pos          (0)                                               /*!< PWM_T::PERIOD5: PERIOD3 Position          */
#define PWM_PERIOD5_PERIOD3_Msk          (0xfffful << PWM_PERIOD5_PERIOD3_Pos)             /*!< PWM_T::PERIOD5: PERIOD3 Mask              */

#define PWM_PERIOD5_PERIOD4_Pos          (0)                                               /*!< PWM_T::PERIOD5: PERIOD4 Position          */
#define PWM_PERIOD5_PERIOD4_Msk          (0xfffful << PWM_PERIOD5_PERIOD4_Pos)             /*!< PWM_T::PERIOD5: PERIOD4 Mask              */

#define PWM_PERIOD5_PERIOD5_Pos          (0)                                               /*!< PWM_T::PERIOD5: PERIOD5 Position          */
#define PWM_PERIOD5_PERIOD5_Msk          (0xfffful << PWM_PERIOD5_PERIOD5_Pos)             /*!< PWM_T::PERIOD5: PERIOD5 Mask              */

#define PWM_PERIOD5_PERIOD6_Pos          (0)                                               /*!< PWM_T::PERIOD5: PERIOD6 Position          */
#define PWM_PERIOD5_PERIOD6_Msk          (0xfffful << PWM_PERIOD6_PERIOD6_Pos)             /*!< PWM_T::PERIOD5: PERIOD6 Mask              */

#define PWM_PERIOD5_PERIOD7_Pos          (0)                                               /*!< PWM_T::PERIOD5: PERIOD7 Position          */
#define PWM_PERIOD5_PERIOD7_Msk          (0xfffful << PWM_PERIOD6_PERIOD7_Pos)             /*!< PWM_T::PERIOD5: PERIOD7 Mask              */

#define PWM_PERIOD6_PERIOD0_Pos          (0)                                               /*!< PWM_T::PERIOD6: PERIOD0 Position          */
#define PWM_PERIOD6_PERIOD0_Msk          (0xfffful << PWM_PERIOD6_PERIOD0_Pos)             /*!< PWM_T::PERIOD6: PERIOD0 Mask              */

#define PWM_PERIOD6_PERIOD1_Pos          (0)                                               /*!< PWM_T::PERIOD6: PERIOD1 Position          */
#define PWM_PERIOD6_PERIOD1_Msk          (0xfffful << PWM_PERIOD6_PERIOD1_Pos)             /*!< PWM_T::PERIOD6: PERIOD1 Mask              */

#define PWM_PERIOD6_PERIOD2_Pos          (0)                                               /*!< PWM_T::PERIOD6: PERIOD2 Position          */
#define PWM_PERIOD6_PERIOD2_Msk          (0xfffful << PWM_PERIOD6_PERIOD2_Pos)             /*!< PWM_T::PERIOD6: PERIOD2 Mask              */

#define PWM_PERIOD6_PERIOD3_Pos          (0)                                               /*!< PWM_T::PERIOD6: PERIOD3 Position          */
#define PWM_PERIOD6_PERIOD3_Msk          (0xfffful << PWM_PERIOD6_PERIOD3_Pos)             /*!< PWM_T::PERIOD6: PERIOD3 Mask              */

#define PWM_PERIOD6_PERIOD4_Pos          (0)                                               /*!< PWM_T::PERIOD6: PERIOD4 Position          */
#define PWM_PERIOD6_PERIOD4_Msk          (0xfffful << PWM_PERIOD6_PERIOD4_Pos)             /*!< PWM_T::PERIOD6: PERIOD4 Mask              */

#define PWM_PERIOD6_PERIOD5_Pos          (0)                                               /*!< PWM_T::PERIOD6: PERIOD5 Position          */
#define PWM_PERIOD6_PERIOD5_Msk          (0xfffful << PWM_PERIOD6_PERIOD5_Pos)             /*!< PWM_T::PERIOD6: PERIOD5 Mask              */

#define PWM_PERIOD6_PERIOD6_Pos          (0)                                               /*!< PWM_T::PERIOD6: PERIOD6 Position          */
#define PWM_PERIOD6_PERIOD6_Msk          (0xfffful << PWM_PERIOD6_PERIOD6_Pos)             /*!< PWM_T::PERIOD6: PERIOD6 Mask              */

#define PWM_PERIOD6_PERIOD7_Pos          (0)                                               /*!< PWM_T::PERIOD6: PERIOD7 Position          */
#define PWM_PERIOD6_PERIOD7_Msk          (0xfffful << PWM_PERIOD6_PERIOD7_Pos)             /*!< PWM_T::PERIOD6: PERIOD7 Mask              */

#define PWM_PERIOD7_PERIOD0_Pos          (0)                                               /*!< PWM_T::PERIOD7: PERIOD0 Position          */
#define PWM_PERIOD7_PERIOD0_Msk          (0xfffful << PWM_PERIOD7_PERIOD0_Pos)             /*!< PWM_T::PERIOD7: PERIOD0 Mask              */

#define PWM_PERIOD7_PERIOD1_Pos          (0)                                               /*!< PWM_T::PERIOD7: PERIOD1 Position          */
#define PWM_PERIOD7_PERIOD1_Msk          (0xfffful << PWM_PERIOD7_PERIOD1_Pos)             /*!< PWM_T::PERIOD7: PERIOD1 Mask              */

#define PWM_PERIOD7_PERIOD2_Pos          (0)                                               /*!< PWM_T::PERIOD7: PERIOD2 Position          */
#define PWM_PERIOD7_PERIOD2_Msk          (0xfffful << PWM_PERIOD7_PERIOD2_Pos)             /*!< PWM_T::PERIOD7: PERIOD2 Mask              */

#define PWM_PERIOD7_PERIOD3_Pos          (0)                                               /*!< PWM_T::PERIOD7: PERIOD3 Position          */
#define PWM_PERIOD7_PERIOD3_Msk          (0xfffful << PWM_PERIOD7_PERIOD3_Pos)             /*!< PWM_T::PERIOD7: PERIOD3 Mask              */

#define PWM_PERIOD7_PERIOD4_Pos          (0)                                               /*!< PWM_T::PERIOD7: PERIOD4 Position          */
#define PWM_PERIOD7_PERIOD4_Msk          (0xfffful << PWM_PERIOD7_PERIOD4_Pos)             /*!< PWM_T::PERIOD7: PERIOD4 Mask              */

#define PWM_PERIOD7_PERIOD5_Pos          (0)                                               /*!< PWM_T::PERIOD7: PERIOD5 Position          */
#define PWM_PERIOD7_PERIOD5_Msk          (0xfffful << PWM_PERIOD7_PERIOD5_Pos)             /*!< PWM_T::PERIOD7: PERIOD5 Mask              */

#define PWM_PERIOD7_PERIOD6_Pos          (0)                                               /*!< PWM_T::PERIOD7: PERIOD6 Position          */
#define PWM_PERIOD7_PERIOD6_Msk          (0xfffful << PWM_PERIOD7_PERIOD6_Pos)             /*!< PWM_T::PERIOD7: PERIOD6 Mask              */

#define PWM_PERIOD7_PERIOD7_Pos          (0)                                               /*!< PWM_T::PERIOD7: PERIOD7 Position          */
#define PWM_PERIOD7_PERIOD7_Msk          (0xfffful << PWM_PERIOD7_PERIOD7_Pos)             /*!< PWM_T::PERIOD7: PERIOD7 Mask              */

#define PWM_CMPDAT0_CMP0_Pos             (0)                                               /*!< PWM_T::CMPDAT0: CMP0 Position             */
#define PWM_CMPDAT0_CMP0_Msk             (0xfffful << PWM_CMPDAT0_CMP0_Pos)                /*!< PWM_T::CMPDAT0: CMP0 Mask                 */

#define PWM_CMPDAT0_CMP1_Pos             (0)                                               /*!< PWM_T::CMPDAT0: CMP1 Position             */
#define PWM_CMPDAT0_CMP1_Msk             (0xfffful << PWM_CMPDAT0_CMP1_Pos)                /*!< PWM_T::CMPDAT0: CMP1 Mask                 */

#define PWM_CMPDAT0_CMP2_Pos             (0)                                               /*!< PWM_T::CMPDAT0: CMP2 Position             */
#define PWM_CMPDAT0_CMP2_Msk             (0xfffful << PWM_CMPDAT0_CMP2_Pos)                /*!< PWM_T::CMPDAT0: CMP2 Mask                 */

#define PWM_CMPDAT0_CMP3_Pos             (0)                                               /*!< PWM_T::CMPDAT0: CMP3 Position             */
#define PWM_CMPDAT0_CMP3_Msk             (0xfffful << PWM_CMPDAT0_CMP3_Pos)                /*!< PWM_T::CMPDAT0: CMP3 Mask                 */

#define PWM_CMPDAT0_CMP4_Pos             (0)                                               /*!< PWM_T::CMPDAT0: CMP4 Position             */
#define PWM_CMPDAT0_CMP4_Msk             (0xfffful << PWM_CMPDAT0_CMP4_Pos)                /*!< PWM_T::CMPDAT0: CMP4 Mask                 */

#define PWM_CMPDAT0_CMP5_Pos             (0)                                               /*!< PWM_T::CMPDAT0: CMP5 Position             */
#define PWM_CMPDAT0_CMP5_Msk             (0xfffful << PWM_CMPDAT0_CMP5_Pos)                /*!< PWM_T::CMPDAT0: CMP5 Mask                 */

#define PWM_CMPDAT0_CMP6_Pos             (0)                                               /*!< PWM_T::CMPDAT0: CMP6 Position             */
#define PWM_CMPDAT0_CMP6_Msk             (0xfffful << PWM_CMPDAT0_CMP6_Pos)                /*!< PWM_T::CMPDAT0: CMP6 Mask                 */

#define PWM_CMPDAT0_CMP7_Pos             (0)                                               /*!< PWM_T::CMPDAT0: CMP7 Position             */
#define PWM_CMPDAT0_CMP7_Msk             (0xfffful << PWM_CMPDAT0_CMP7_Pos)                /*!< PWM_T::CMPDAT0: CMP7 Mask                 */

#define PWM_CMPDAT0_CMPD0_Pos            (16)                                              /*!< PWM_T::CMPDAT0: CMPD0 Position            */
#define PWM_CMPDAT0_CMPD0_Msk            (0xfffful << PWM_CMPDAT0_CMPD0_Pos)               /*!< PWM_T::CMPDAT0: CMPD0 Mask                */

#define PWM_CMPDAT0_CMPD1_Pos            (16)                                              /*!< PWM_T::CMPDAT0: CMPD1 Position            */
#define PWM_CMPDAT0_CMPD1_Msk            (0xfffful << PWM_CMPDAT0_CMPD1_Pos)               /*!< PWM_T::CMPDAT0: CMPD1 Mask                */

#define PWM_CMPDAT0_CMPD2_Pos            (16)                                              /*!< PWM_T::CMPDAT0: CMPD2 Position            */
#define PWM_CMPDAT0_CMPD2_Msk            (0xfffful << PWM_CMPDAT0_CMPD2_Pos)               /*!< PWM_T::CMPDAT0: CMPD2 Mask                */

#define PWM_CMPDAT0_CMPD3_Pos            (16)                                              /*!< PWM_T::CMPDAT0: CMPD3 Position            */
#define PWM_CMPDAT0_CMPD3_Msk            (0xfffful << PWM_CMPDAT0_CMPD3_Pos)               /*!< PWM_T::CMPDAT0: CMPD3 Mask                */

#define PWM_CMPDAT0_CMPD4_Pos            (16)                                              /*!< PWM_T::CMPDAT0: CMPD4 Position            */
#define PWM_CMPDAT0_CMPD4_Msk            (0xfffful << PWM_CMPDAT0_CMPD4_Pos)               /*!< PWM_T::CMPDAT0: CMPD4 Mask                */

#define PWM_CMPDAT0_CMPD5_Pos            (16)                                              /*!< PWM_T::CMPDAT0: CMPD5 Position            */
#define PWM_CMPDAT0_CMPD5_Msk            (0xfffful << PWM_CMPDAT0_CMPD5_Pos)               /*!< PWM_T::CMPDAT0: CMPD5 Mask                */

#define PWM_CMPDAT0_CMPD6_Pos            (16)                                              /*!< PWM_T::CMPDAT0: CMPD6 Position            */
#define PWM_CMPDAT0_CMPD6_Msk            (0xfffful << PWM_CMPDAT0_CMPD6_Pos)               /*!< PWM_T::CMPDAT0: CMPD6 Mask                */

#define PWM_CMPDAT0_CMPD7_Pos            (16)                                              /*!< PWM_T::CMPDAT0: CMPD7 Position            */
#define PWM_CMPDAT0_CMPD7_Msk            (0xfffful << PWM_CMPDAT0_CMPD7_Pos)               /*!< PWM_T::CMPDAT0: CMPD7 Mask                */

#define PWM_CMPDAT1_CMP0_Pos             (0)                                               /*!< PWM_T::CMPDAT1: CMP0 Position             */
#define PWM_CMPDAT1_CMP0_Msk             (0xfffful << PWM_CMPDAT1_CMP0_Pos)                /*!< PWM_T::CMPDAT1: CMP0 Mask                 */

#define PWM_CMPDAT1_CMP1_Pos             (0)                                               /*!< PWM_T::CMPDAT1: CMP1 Position             */
#define PWM_CMPDAT1_CMP1_Msk             (0xfffful << PWM_CMPDAT1_CMP1_Pos)                /*!< PWM_T::CMPDAT1: CMP1 Mask                 */

#define PWM_CMPDAT1_CMP2_Pos             (0)                                               /*!< PWM_T::CMPDAT1: CMP2 Position             */
#define PWM_CMPDAT1_CMP2_Msk             (0xfffful << PWM_CMPDAT1_CMP2_Pos)                /*!< PWM_T::CMPDAT1: CMP2 Mask                 */

#define PWM_CMPDAT1_CMP3_Pos             (0)                                               /*!< PWM_T::CMPDAT1: CMP3 Position             */
#define PWM_CMPDAT1_CMP3_Msk             (0xfffful << PWM_CMPDAT1_CMP3_Pos)                /*!< PWM_T::CMPDAT1: CMP3 Mask                 */

#define PWM_CMPDAT1_CMP4_Pos             (0)                                               /*!< PWM_T::CMPDAT1: CMP4 Position             */
#define PWM_CMPDAT1_CMP4_Msk             (0xfffful << PWM_CMPDAT1_CMP4_Pos)                /*!< PWM_T::CMPDAT1: CMP4 Mask                 */

#define PWM_CMPDAT1_CMP5_Pos             (0)                                               /*!< PWM_T::CMPDAT1: CMP5 Position             */
#define PWM_CMPDAT1_CMP5_Msk             (0xfffful << PWM_CMPDAT1_CMP5_Pos)                /*!< PWM_T::CMPDAT1: CMP5 Mask                 */

#define PWM_CMPDAT1_CMP6_Pos             (0)                                               /*!< PWM_T::CMPDAT1: CMP6 Position             */
#define PWM_CMPDAT1_CMP6_Msk             (0xfffful << PWM_CMPDAT1_CMP6_Pos)                /*!< PWM_T::CMPDAT1: CMP6 Mask                 */

#define PWM_CMPDAT1_CMP7_Pos             (0)                                               /*!< PWM_T::CMPDAT1: CMP7 Position             */
#define PWM_CMPDAT1_CMP7_Msk             (0xfffful << PWM_CMPDAT1_CMP7_Pos)                /*!< PWM_T::CMPDAT1: CMP7 Mask                 */

#define PWM_CMPDAT1_CMPD0_Pos            (16)                                              /*!< PWM_T::CMPDAT1: CMPD0 Position            */
#define PWM_CMPDAT1_CMPD0_Msk            (0xfffful << PWM_CMPDAT1_CMPD0_Pos)               /*!< PWM_T::CMPDAT1: CMPD0 Mask                */

#define PWM_CMPDAT1_CMPD1_Pos            (16)                                              /*!< PWM_T::CMPDAT1: CMPD1 Position            */
#define PWM_CMPDAT1_CMPD1_Msk            (0xfffful << PWM_CMPDAT1_CMPD1_Pos)               /*!< PWM_T::CMPDAT1: CMPD1 Mask                */

#define PWM_CMPDAT1_CMPD2_Pos            (16)                                              /*!< PWM_T::CMPDAT1: CMPD2 Position            */
#define PWM_CMPDAT1_CMPD2_Msk            (0xfffful << PWM_CMPDAT1_CMPD2_Pos)               /*!< PWM_T::CMPDAT1: CMPD2 Mask                */

#define PWM_CMPDAT1_CMPD3_Pos            (16)                                              /*!< PWM_T::CMPDAT1: CMPD3 Position            */
#define PWM_CMPDAT1_CMPD3_Msk            (0xfffful << PWM_CMPDAT1_CMPD3_Pos)               /*!< PWM_T::CMPDAT1: CMPD3 Mask                */

#define PWM_CMPDAT1_CMPD4_Pos            (16)                                              /*!< PWM_T::CMPDAT1: CMPD4 Position            */
#define PWM_CMPDAT1_CMPD4_Msk            (0xfffful << PWM_CMPDAT1_CMPD4_Pos)               /*!< PWM_T::CMPDAT1: CMPD4 Mask                */

#define PWM_CMPDAT1_CMPD5_Pos            (16)                                              /*!< PWM_T::CMPDAT1: CMPD5 Position            */
#define PWM_CMPDAT1_CMPD5_Msk            (0xfffful << PWM_CMPDAT1_CMPD5_Pos)               /*!< PWM_T::CMPDAT1: CMPD5 Mask                */

#define PWM_CMPDAT1_CMPD6_Pos            (16)                                              /*!< PWM_T::CMPDAT1: CMPD6 Position            */
#define PWM_CMPDAT1_CMPD6_Msk            (0xfffful << PWM_CMPDAT1_CMPD6_Pos)               /*!< PWM_T::CMPDAT1: CMPD6 Mask                */

#define PWM_CMPDAT1_CMPD7_Pos            (16)                                              /*!< PWM_T::CMPDAT1: CMPD7 Position            */
#define PWM_CMPDAT1_CMPD7_Msk            (0xfffful << PWM_CMPDAT1_CMPD7_Pos)               /*!< PWM_T::CMPDAT1: CMPD7 Mask                */

#define PWM_CMPDAT2_CMP0_Pos             (0)                                               /*!< PWM_T::CMPDAT2: CMP0 Position             */
#define PWM_CMPDAT2_CMP0_Msk             (0xfffful << PWM_CMPDAT2_CMP0_Pos)                /*!< PWM_T::CMPDAT2: CMP0 Mask                 */

#define PWM_CMPDAT2_CMP1_Pos             (0)                                               /*!< PWM_T::CMPDAT2: CMP1 Position             */
#define PWM_CMPDAT2_CMP1_Msk             (0xfffful << PWM_CMPDAT2_CMP1_Pos)                /*!< PWM_T::CMPDAT2: CMP1 Mask                 */

#define PWM_CMPDAT2_CMP2_Pos             (0)                                               /*!< PWM_T::CMPDAT2: CMP2 Position             */
#define PWM_CMPDAT2_CMP2_Msk             (0xfffful << PWM_CMPDAT2_CMP2_Pos)                /*!< PWM_T::CMPDAT2: CMP2 Mask                 */

#define PWM_CMPDAT2_CMP3_Pos             (0)                                               /*!< PWM_T::CMPDAT2: CMP3 Position             */
#define PWM_CMPDAT2_CMP3_Msk             (0xfffful << PWM_CMPDAT2_CMP3_Pos)                /*!< PWM_T::CMPDAT2: CMP3 Mask                 */

#define PWM_CMPDAT2_CMP4_Pos             (0)                                               /*!< PWM_T::CMPDAT2: CMP4 Position             */
#define PWM_CMPDAT2_CMP4_Msk             (0xfffful << PWM_CMPDAT2_CMP4_Pos)                /*!< PWM_T::CMPDAT2: CMP4 Mask                 */

#define PWM_CMPDAT2_CMP5_Pos             (0)                                               /*!< PWM_T::CMPDAT2: CMP5 Position             */
#define PWM_CMPDAT2_CMP5_Msk             (0xfffful << PWM_CMPDAT2_CMP5_Pos)                /*!< PWM_T::CMPDAT2: CMP5 Mask                 */

#define PWM_CMPDAT2_CMP6_Pos             (0)                                               /*!< PWM_T::CMPDAT2: CMP6 Position             */
#define PWM_CMPDAT2_CMP6_Msk             (0xfffful << PWM_CMPDAT2_CMP6_Pos)                /*!< PWM_T::CMPDAT2: CMP6 Mask                 */

#define PWM_CMPDAT2_CMP7_Pos             (0)                                               /*!< PWM_T::CMPDAT2: CMP7 Position             */
#define PWM_CMPDAT2_CMP7_Msk             (0xfffful << PWM_CMPDAT2_CMP7_Pos)                /*!< PWM_T::CMPDAT2: CMP7 Mask                 */

#define PWM_CMPDAT2_CMPD0_Pos            (16)                                              /*!< PWM_T::CMPDAT2: CMPD0 Position            */
#define PWM_CMPDAT2_CMPD0_Msk            (0xfffful << PWM_CMPDAT2_CMPD0_Pos)               /*!< PWM_T::CMPDAT2: CMPD0 Mask                */

#define PWM_CMPDAT2_CMPD1_Pos            (16)                                              /*!< PWM_T::CMPDAT2: CMPD1 Position            */
#define PWM_CMPDAT2_CMPD1_Msk            (0xfffful << PWM_CMPDAT2_CMPD1_Pos)               /*!< PWM_T::CMPDAT2: CMPD1 Mask                */

#define PWM_CMPDAT2_CMPD2_Pos            (16)                                              /*!< PWM_T::CMPDAT2: CMPD2 Position            */
#define PWM_CMPDAT2_CMPD2_Msk            (0xfffful << PWM_CMPDAT2_CMPD2_Pos)               /*!< PWM_T::CMPDAT2: CMPD2 Mask                */

#define PWM_CMPDAT2_CMPD3_Pos            (16)                                              /*!< PWM_T::CMPDAT2: CMPD3 Position            */
#define PWM_CMPDAT2_CMPD3_Msk            (0xfffful << PWM_CMPDAT2_CMPD3_Pos)               /*!< PWM_T::CMPDAT2: CMPD3 Mask                */

#define PWM_CMPDAT2_CMPD4_Pos            (16)                                              /*!< PWM_T::CMPDAT2: CMPD4 Position            */
#define PWM_CMPDAT2_CMPD4_Msk            (0xfffful << PWM_CMPDAT2_CMPD4_Pos)               /*!< PWM_T::CMPDAT2: CMPD4 Mask                */

#define PWM_CMPDAT2_CMPD5_Pos            (16)                                              /*!< PWM_T::CMPDAT2: CMPD5 Position            */
#define PWM_CMPDAT2_CMPD5_Msk            (0xfffful << PWM_CMPDAT2_CMPD5_Pos)               /*!< PWM_T::CMPDAT2: CMPD5 Mask                */

#define PWM_CMPDAT2_CMPD6_Pos            (16)                                              /*!< PWM_T::CMPDAT2: CMPD6 Position            */
#define PWM_CMPDAT2_CMPD6_Msk            (0xfffful << PWM_CMPDAT2_CMPD6_Pos)               /*!< PWM_T::CMPDAT2: CMPD6 Mask                */

#define PWM_CMPDAT2_CMPD7_Pos            (16)                                              /*!< PWM_T::CMPDAT2: CMPD7 Position            */
#define PWM_CMPDAT2_CMPD7_Msk            (0xfffful << PWM_CMPDAT2_CMPD7_Pos)               /*!< PWM_T::CMPDAT2: CMPD7 Mask                */

#define PWM_CMPDAT3_CMP0_Pos             (0)                                               /*!< PWM_T::CMPDAT3: CMP0 Position             */
#define PWM_CMPDAT3_CMP0_Msk             (0xfffful << PWM_CMPDAT3_CMP0_Pos)                /*!< PWM_T::CMPDAT3: CMP0 Mask                 */

#define PWM_CMPDAT3_CMP1_Pos             (0)                                               /*!< PWM_T::CMPDAT3: CMP1 Position             */
#define PWM_CMPDAT3_CMP1_Msk             (0xfffful << PWM_CMPDAT3_CMP1_Pos)                /*!< PWM_T::CMPDAT3: CMP1 Mask                 */

#define PWM_CMPDAT3_CMP2_Pos             (0)                                               /*!< PWM_T::CMPDAT3: CMP2 Position             */
#define PWM_CMPDAT3_CMP2_Msk             (0xfffful << PWM_CMPDAT3_CMP2_Pos)                /*!< PWM_T::CMPDAT3: CMP2 Mask                 */

#define PWM_CMPDAT3_CMP3_Pos             (0)                                               /*!< PWM_T::CMPDAT3: CMP3 Position             */
#define PWM_CMPDAT3_CMP3_Msk             (0xfffful << PWM_CMPDAT3_CMP3_Pos)                /*!< PWM_T::CMPDAT3: CMP3 Mask                 */

#define PWM_CMPDAT3_CMP4_Pos             (0)                                               /*!< PWM_T::CMPDAT3: CMP4 Position             */
#define PWM_CMPDAT3_CMP4_Msk             (0xfffful << PWM_CMPDAT3_CMP4_Pos)                /*!< PWM_T::CMPDAT3: CMP4 Mask                 */

#define PWM_CMPDAT3_CMP5_Pos             (0)                                               /*!< PWM_T::CMPDAT3: CMP5 Position             */
#define PWM_CMPDAT3_CMP5_Msk             (0xfffful << PWM_CMPDAT3_CMP5_Pos)                /*!< PWM_T::CMPDAT3: CMP5 Mask                 */

#define PWM_CMPDAT3_CMP6_Pos             (0)                                               /*!< PWM_T::CMPDAT3: CMP6 Position             */
#define PWM_CMPDAT3_CMP6_Msk             (0xfffful << PWM_CMPDAT3_CMP6_Pos)                /*!< PWM_T::CMPDAT3: CMP6 Mask                 */

#define PWM_CMPDAT3_CMP7_Pos             (0)                                               /*!< PWM_T::CMPDAT3: CMP7 Position             */
#define PWM_CMPDAT3_CMP7_Msk             (0xfffful << PWM_CMPDAT3_CMP7_Pos)                /*!< PWM_T::CMPDAT3: CMP7 Mask                 */

#define PWM_CMPDAT3_CMPD0_Pos            (16)                                              /*!< PWM_T::CMPDAT3: CMPD0 Position            */
#define PWM_CMPDAT3_CMPD0_Msk            (0xfffful << PWM_CMPDAT3_CMPD0_Pos)               /*!< PWM_T::CMPDAT3: CMPD0 Mask                */

#define PWM_CMPDAT3_CMPD1_Pos            (16)                                              /*!< PWM_T::CMPDAT3: CMPD1 Position            */
#define PWM_CMPDAT3_CMPD1_Msk            (0xfffful << PWM_CMPDAT3_CMPD1_Pos)               /*!< PWM_T::CMPDAT3: CMPD1 Mask                */

#define PWM_CMPDAT3_CMPD2_Pos            (16)                                              /*!< PWM_T::CMPDAT3: CMPD2 Position            */
#define PWM_CMPDAT3_CMPD2_Msk            (0xfffful << PWM_CMPDAT3_CMPD2_Pos)               /*!< PWM_T::CMPDAT3: CMPD2 Mask                */

#define PWM_CMPDAT3_CMPD3_Pos            (16)                                              /*!< PWM_T::CMPDAT3: CMPD3 Position            */
#define PWM_CMPDAT3_CMPD3_Msk            (0xfffful << PWM_CMPDAT3_CMPD3_Pos)               /*!< PWM_T::CMPDAT3: CMPD3 Mask                */

#define PWM_CMPDAT3_CMPD4_Pos            (16)                                              /*!< PWM_T::CMPDAT3: CMPD4 Position            */
#define PWM_CMPDAT3_CMPD4_Msk            (0xfffful << PWM_CMPDAT3_CMPD4_Pos)               /*!< PWM_T::CMPDAT3: CMPD4 Mask                */

#define PWM_CMPDAT3_CMPD5_Pos            (16)                                              /*!< PWM_T::CMPDAT3: CMPD5 Position            */
#define PWM_CMPDAT3_CMPD5_Msk            (0xfffful << PWM_CMPDAT3_CMPD5_Pos)               /*!< PWM_T::CMPDAT3: CMPD5 Mask                */

#define PWM_CMPDAT3_CMPD6_Pos            (16)                                              /*!< PWM_T::CMPDAT3: CMPD6 Position            */
#define PWM_CMPDAT3_CMPD6_Msk            (0xfffful << PWM_CMPDAT3_CMPD4_Pos)               /*!< PWM_T::CMPDAT3: CMPD6 Mask                */

#define PWM_CMPDAT3_CMPD7_Pos            (16)                                              /*!< PWM_T::CMPDAT3: CMPD7 Position            */
#define PWM_CMPDAT3_CMPD7_Msk            (0xfffful << PWM_CMPDAT3_CMPD7_Pos)               /*!< PWM_T::CMPDAT3: CMPD7 Mask                */

#define PWM_CMPDAT4_CMP0_Pos             (0)                                               /*!< PWM_T::CMPDAT4: CMP0 Position             */
#define PWM_CMPDAT4_CMP0_Msk             (0xfffful << PWM_CMPDAT4_CMP0_Pos)                /*!< PWM_T::CMPDAT4: CMP0 Mask                 */

#define PWM_CMPDAT4_CMP1_Pos             (0)                                               /*!< PWM_T::CMPDAT4: CMP1 Position             */
#define PWM_CMPDAT4_CMP1_Msk             (0xfffful << PWM_CMPDAT4_CMP1_Pos)                /*!< PWM_T::CMPDAT4: CMP1 Mask                 */

#define PWM_CMPDAT4_CMP2_Pos             (0)                                               /*!< PWM_T::CMPDAT4: CMP2 Position             */
#define PWM_CMPDAT4_CMP2_Msk             (0xfffful << PWM_CMPDAT4_CMP2_Pos)                /*!< PWM_T::CMPDAT4: CMP2 Mask                 */

#define PWM_CMPDAT4_CMP3_Pos             (0)                                               /*!< PWM_T::CMPDAT4: CMP3 Position             */
#define PWM_CMPDAT4_CMP3_Msk             (0xfffful << PWM_CMPDAT4_CMP3_Pos)                /*!< PWM_T::CMPDAT4: CMP3 Mask                 */

#define PWM_CMPDAT4_CMP4_Pos             (0)                                               /*!< PWM_T::CMPDAT4: CMP4 Position             */
#define PWM_CMPDAT4_CMP4_Msk             (0xfffful << PWM_CMPDAT4_CMP4_Pos)                /*!< PWM_T::CMPDAT4: CMP4 Mask                 */

#define PWM_CMPDAT4_CMP5_Pos             (0)                                               /*!< PWM_T::CMPDAT4: CMP5 Position             */
#define PWM_CMPDAT4_CMP5_Msk             (0xfffful << PWM_CMPDAT4_CMP5_Pos)                /*!< PWM_T::CMPDAT4: CMP5 Mask                 */

#define PWM_CMPDAT4_CMP6_Pos             (0)                                               /*!< PWM_T::CMPDAT4: CMP6 Position             */
#define PWM_CMPDAT4_CMP6_Msk             (0xfffful << PWM_CMPDAT4_CMP6_Pos)                /*!< PWM_T::CMPDAT4: CMP6 Mask                 */

#define PWM_CMPDAT4_CMP7_Pos             (0)                                               /*!< PWM_T::CMPDAT4: CMP7 Position             */
#define PWM_CMPDAT4_CMP7_Msk             (0xfffful << PWM_CMPDAT4_CMP7_Pos)                /*!< PWM_T::CMPDAT4: CMP7 Mask                 */

#define PWM_CMPDAT4_CMPD0_Pos            (16)                                              /*!< PWM_T::CMPDAT4: CMPD0 Position            */
#define PWM_CMPDAT4_CMPD0_Msk            (0xfffful << PWM_CMPDAT4_CMPD0_Pos)               /*!< PWM_T::CMPDAT4: CMPD0 Mask                */

#define PWM_CMPDAT4_CMPD1_Pos            (16)                                              /*!< PWM_T::CMPDAT4: CMPD1 Position            */
#define PWM_CMPDAT4_CMPD1_Msk            (0xfffful << PWM_CMPDAT4_CMPD1_Pos)               /*!< PWM_T::CMPDAT4: CMPD1 Mask                */

#define PWM_CMPDAT4_CMPD2_Pos            (16)                                              /*!< PWM_T::CMPDAT4: CMPD2 Position            */
#define PWM_CMPDAT4_CMPD2_Msk            (0xfffful << PWM_CMPDAT4_CMPD2_Pos)               /*!< PWM_T::CMPDAT4: CMPD2 Mask                */

#define PWM_CMPDAT4_CMPD3_Pos            (16)                                              /*!< PWM_T::CMPDAT4: CMPD3 Position            */
#define PWM_CMPDAT4_CMPD3_Msk            (0xfffful << PWM_CMPDAT4_CMPD3_Pos)               /*!< PWM_T::CMPDAT4: CMPD3 Mask                */

#define PWM_CMPDAT4_CMPD4_Pos            (16)                                              /*!< PWM_T::CMPDAT4: CMPD4 Position            */
#define PWM_CMPDAT4_CMPD4_Msk            (0xfffful << PWM_CMPDAT4_CMPD4_Pos)               /*!< PWM_T::CMPDAT4: CMPD4 Mask                */

#define PWM_CMPDAT4_CMPD5_Pos            (16)                                              /*!< PWM_T::CMPDAT4: CMPD5 Position            */
#define PWM_CMPDAT4_CMPD5_Msk            (0xfffful << PWM_CMPDAT4_CMPD5_Pos)               /*!< PWM_T::CMPDAT4: CMPD5 Mask                */

#define PWM_CMPDAT4_CMPD6_Pos            (16)                                              /*!< PWM_T::CMPDAT4: CMPD6 Position            */
#define PWM_CMPDAT4_CMPD6_Msk            (0xfffful << PWM_CMPDAT4_CMPD6_Pos)               /*!< PWM_T::CMPDAT4: CMPD6 Mask                */

#define PWM_CMPDAT4_CMPD7_Pos            (16)                                              /*!< PWM_T::CMPDAT4: CMPD7 Position            */
#define PWM_CMPDAT4_CMPD7_Msk            (0xfffful << PWM_CMPDAT4_CMPD7_Pos)               /*!< PWM_T::CMPDAT4: CMPD7 Mask                */

#define PWM_CMPDAT5_CMP0_Pos             (0)                                               /*!< PWM_T::CMPDAT5: CMP0 Position             */
#define PWM_CMPDAT5_CMP0_Msk             (0xfffful << PWM_CMPDAT5_CMP0_Pos)                /*!< PWM_T::CMPDAT5: CMP0 Mask                 */

#define PWM_CMPDAT5_CMP1_Pos             (0)                                               /*!< PWM_T::CMPDAT5: CMP1 Position             */
#define PWM_CMPDAT5_CMP1_Msk             (0xfffful << PWM_CMPDAT5_CMP1_Pos)                /*!< PWM_T::CMPDAT5: CMP1 Mask                 */

#define PWM_CMPDAT5_CMP2_Pos             (0)                                               /*!< PWM_T::CMPDAT5: CMP2 Position             */
#define PWM_CMPDAT5_CMP2_Msk             (0xfffful << PWM_CMPDAT5_CMP2_Pos)                /*!< PWM_T::CMPDAT5: CMP2 Mask                 */

#define PWM_CMPDAT5_CMP3_Pos             (0)                                               /*!< PWM_T::CMPDAT5: CMP3 Position             */
#define PWM_CMPDAT5_CMP3_Msk             (0xfffful << PWM_CMPDAT5_CMP3_Pos)                /*!< PWM_T::CMPDAT5: CMP3 Mask                 */

#define PWM_CMPDAT5_CMP4_Pos             (0)                                               /*!< PWM_T::CMPDAT5: CMP4 Position             */
#define PWM_CMPDAT5_CMP4_Msk             (0xfffful << PWM_CMPDAT5_CMP4_Pos)                /*!< PWM_T::CMPDAT5: CMP4 Mask                 */

#define PWM_CMPDAT5_CMP5_Pos             (0)                                               /*!< PWM_T::CMPDAT5: CMP5 Position             */
#define PWM_CMPDAT5_CMP5_Msk             (0xfffful << PWM_CMPDAT5_CMP5_Pos)                /*!< PWM_T::CMPDAT5: CMP5 Mask                 */

#define PWM_CMPDAT5_CMP6_Pos             (0)                                               /*!< PWM_T::CMPDAT5: CMP6 Position             */
#define PWM_CMPDAT5_CMP6_Msk             (0xfffful << PWM_CMPDAT5_CMP6_Pos)                /*!< PWM_T::CMPDAT5: CMP6 Mask                 */

#define PWM_CMPDAT5_CMP7_Pos             (0)                                               /*!< PWM_T::CMPDAT5: CMP7 Position             */
#define PWM_CMPDAT5_CMP7_Msk             (0xfffful << PWM_CMPDAT5_CMP7_Pos)                /*!< PWM_T::CMPDAT5: CMP7 Mask                 */

#define PWM_CMPDAT5_CMPD0_Pos            (16)                                              /*!< PWM_T::CMPDAT5: CMPD0 Position            */
#define PWM_CMPDAT5_CMPD0_Msk            (0xfffful << PWM_CMPDAT5_CMPD0_Pos)               /*!< PWM_T::CMPDAT5: CMPD0 Mask                */

#define PWM_CMPDAT5_CMPD1_Pos            (16)                                              /*!< PWM_T::CMPDAT5: CMPD1 Position            */
#define PWM_CMPDAT5_CMPD1_Msk            (0xfffful << PWM_CMPDAT5_CMPD1_Pos)               /*!< PWM_T::CMPDAT5: CMPD1 Mask                */

#define PWM_CMPDAT5_CMPD2_Pos            (16)                                              /*!< PWM_T::CMPDAT5: CMPD2 Position            */
#define PWM_CMPDAT5_CMPD2_Msk            (0xfffful << PWM_CMPDAT5_CMPD2_Pos)               /*!< PWM_T::CMPDAT5: CMPD2 Mask                */

#define PWM_CMPDAT5_CMPD3_Pos            (16)                                              /*!< PWM_T::CMPDAT5: CMPD3 Position            */
#define PWM_CMPDAT5_CMPD3_Msk            (0xfffful << PWM_CMPDAT5_CMPD3_Pos)               /*!< PWM_T::CMPDAT5: CMPD3 Mask                */

#define PWM_CMPDAT5_CMPD4_Pos            (16)                                              /*!< PWM_T::CMPDAT5: CMPD4 Position            */
#define PWM_CMPDAT5_CMPD4_Msk            (0xfffful << PWM_CMPDAT5_CMPD4_Pos)               /*!< PWM_T::CMPDAT5: CMPD4 Mask                */

#define PWM_CMPDAT5_CMPD5_Pos            (16)                                              /*!< PWM_T::CMPDAT5: CMPD5 Position            */
#define PWM_CMPDAT5_CMPD5_Msk            (0xfffful << PWM_CMPDAT5_CMPD5_Pos)               /*!< PWM_T::CMPDAT5: CMPD5 Mask                */

#define PWM_CMPDAT5_CMPD6_Pos            (16)                                              /*!< PWM_T::CMPDAT5: CMPD6 Position            */
#define PWM_CMPDAT5_CMPD6_Msk            (0xfffful << PWM_CMPDAT5_CMPD6_Pos)               /*!< PWM_T::CMPDAT5: CMPD6 Mask                */

#define PWM_CMPDAT5_CMPD7_Pos            (16)                                              /*!< PWM_T::CMPDAT5: CMPD7 Position            */
#define PWM_CMPDAT5_CMPD7_Msk            (0xfffful << PWM_CMPDAT5_CMPD7_Pos)               /*!< PWM_T::CMPDAT5: CMPD7 Mask                */

#define PWM_CMPDAT6_CMP0_Pos             (0)                                               /*!< PWM_T::CMPDAT6: CMP0 Position             */
#define PWM_CMPDAT6_CMP0_Msk             (0xfffful << PWM_CMPDAT6_CMP0_Pos)                /*!< PWM_T::CMPDAT6: CMP0 Mask                 */

#define PWM_CMPDAT6_CMP1_Pos             (0)                                               /*!< PWM_T::CMPDAT6: CMP1 Position             */
#define PWM_CMPDAT6_CMP1_Msk             (0xfffful << PWM_CMPDAT6_CMP1_Pos)                /*!< PWM_T::CMPDAT6: CMP1 Mask                 */

#define PWM_CMPDAT6_CMP2_Pos             (0)                                               /*!< PWM_T::CMPDAT6: CMP2 Position             */
#define PWM_CMPDAT6_CMP2_Msk             (0xfffful << PWM_CMPDAT6_CMP2_Pos)                /*!< PWM_T::CMPDAT6: CMP2 Mask                 */

#define PWM_CMPDAT6_CMP3_Pos             (0)                                               /*!< PWM_T::CMPDAT6: CMP3 Position             */
#define PWM_CMPDAT6_CMP3_Msk             (0xfffful << PWM_CMPDAT6_CMP3_Pos)                /*!< PWM_T::CMPDAT6: CMP3 Mask                 */

#define PWM_CMPDAT6_CMP4_Pos             (0)                                               /*!< PWM_T::CMPDAT6: CMP4 Position             */
#define PWM_CMPDAT6_CMP4_Msk             (0xfffful << PWM_CMPDAT6_CMP4_Pos)                /*!< PWM_T::CMPDAT6: CMP4 Mask                 */

#define PWM_CMPDAT6_CMP5_Pos             (0)                                               /*!< PWM_T::CMPDAT6: CMP5 Position             */
#define PWM_CMPDAT6_CMP5_Msk             (0xfffful << PWM_CMPDAT6_CMP5_Pos)                /*!< PWM_T::CMPDAT6: CMP5 Mask                 */

#define PWM_CMPDAT6_CMP6_Pos             (0)                                               /*!< PWM_T::CMPDAT6: CMP6 Position             */
#define PWM_CMPDAT6_CMP6_Msk             (0xfffful << PWM_CMPDAT6_CMP6_Pos)                /*!< PWM_T::CMPDAT6: CMP6 Mask                 */

#define PWM_CMPDAT6_CMP7_Pos             (0)                                               /*!< PWM_T::CMPDAT6: CMP7 Position             */
#define PWM_CMPDAT6_CMP7_Msk             (0xfffful << PWM_CMPDAT6_CMP7_Pos)                /*!< PWM_T::CMPDAT6: CMP7 Mask                 */

#define PWM_CMPDAT6_CMPD0_Pos            (16)                                              /*!< PWM_T::CMPDAT6: CMPD0 Position            */
#define PWM_CMPDAT6_CMPD0_Msk            (0xfffful << PWM_CMPDAT6_CMPD0_Pos)               /*!< PWM_T::CMPDAT6: CMPD0 Mask                */

#define PWM_CMPDAT6_CMPD1_Pos            (16)                                              /*!< PWM_T::CMPDAT6: CMPD1 Position            */
#define PWM_CMPDAT6_CMPD1_Msk            (0xfffful << PWM_CMPDAT6_CMPD1_Pos)               /*!< PWM_T::CMPDAT6: CMPD1 Mask                */

#define PWM_CMPDAT6_CMPD2_Pos            (16)                                              /*!< PWM_T::CMPDAT6: CMPD2 Position            */
#define PWM_CMPDAT6_CMPD2_Msk            (0xfffful << PWM_CMPDAT6_CMPD2_Pos)               /*!< PWM_T::CMPDAT6: CMPD2 Mask                */

#define PWM_CMPDAT6_CMPD3_Pos            (16)                                              /*!< PWM_T::CMPDAT6: CMPD3 Position            */
#define PWM_CMPDAT6_CMPD3_Msk            (0xfffful << PWM_CMPDAT6_CMPD3_Pos)               /*!< PWM_T::CMPDAT6: CMPD3 Mask                */

#define PWM_CMPDAT6_CMPD4_Pos            (16)                                              /*!< PWM_T::CMPDAT6: CMPD4 Position            */
#define PWM_CMPDAT6_CMPD4_Msk            (0xfffful << PWM_CMPDAT6_CMPD4_Pos)               /*!< PWM_T::CMPDAT6: CMPD4 Mask                */

#define PWM_CMPDAT6_CMPD5_Pos            (16)                                              /*!< PWM_T::CMPDAT6: CMPD5 Position            */
#define PWM_CMPDAT6_CMPD5_Msk            (0xfffful << PWM_CMPDAT6_CMPD5_Pos)               /*!< PWM_T::CMPDAT6: CMPD5 Mask                */

#define PWM_CMPDAT6_CMPD6_Pos            (16)                                              /*!< PWM_T::CMPDAT6: CMPD6 Position            */
#define PWM_CMPDAT6_CMPD6_Msk            (0xfffful << PWM_CMPDAT6_CMPD6_Pos)               /*!< PWM_T::CMPDAT6: CMPD6 Mask                */

#define PWM_CMPDAT6_CMPD7_Pos            (16)                                              /*!< PWM_T::CMPDAT6: CMPD7 Position            */
#define PWM_CMPDAT6_CMPD7_Msk            (0xfffful << PWM_CMPDAT6_CMPD7_Pos)               /*!< PWM_T::CMPDAT6: CMPD7 Mask                */

#define PWM_CMPDAT7_CMP0_Pos             (0)                                               /*!< PWM_T::CMPDAT7: CMP0 Position             */
#define PWM_CMPDAT7_CMP0_Msk             (0xfffful << PWM_CMPDAT7_CMP0_Pos)                /*!< PWM_T::CMPDAT7: CMP0 Mask                 */

#define PWM_CMPDAT7_CMP1_Pos             (0)                                               /*!< PWM_T::CMPDAT7: CMP1 Position             */
#define PWM_CMPDAT7_CMP1_Msk             (0xfffful << PWM_CMPDAT7_CMP1_Pos)                /*!< PWM_T::CMPDAT7: CMP1 Mask                 */

#define PWM_CMPDAT7_CMP2_Pos             (0)                                               /*!< PWM_T::CMPDAT7: CMP2 Position             */
#define PWM_CMPDAT7_CMP2_Msk             (0xfffful << PWM_CMPDAT7_CMP2_Pos)                /*!< PWM_T::CMPDAT7: CMP2 Mask                 */

#define PWM_CMPDAT7_CMP3_Pos             (0)                                               /*!< PWM_T::CMPDAT7: CMP3 Position             */
#define PWM_CMPDAT7_CMP3_Msk             (0xfffful << PWM_CMPDAT7_CMP3_Pos)                /*!< PWM_T::CMPDAT7: CMP3 Mask                 */

#define PWM_CMPDAT7_CMP4_Pos             (0)                                               /*!< PWM_T::CMPDAT7: CMP4 Position             */
#define PWM_CMPDAT7_CMP4_Msk             (0xfffful << PWM_CMPDAT7_CMP4_Pos)                /*!< PWM_T::CMPDAT7: CMP4 Mask                 */

#define PWM_CMPDAT7_CMP5_Pos             (0)                                               /*!< PWM_T::CMPDAT7: CMP5 Position             */
#define PWM_CMPDAT7_CMP5_Msk             (0xfffful << PWM_CMPDAT7_CMP5_Pos)                /*!< PWM_T::CMPDAT7: CMP5 Mask                 */

#define PWM_CMPDAT7_CMP6_Pos             (0)                                               /*!< PWM_T::CMPDAT7: CMP6 Position             */
#define PWM_CMPDAT7_CMP6_Msk             (0xfffful << PWM_CMPDAT7_CMP6_Pos)                /*!< PWM_T::CMPDAT7: CMP6 Mask                 */

#define PWM_CMPDAT7_CMP7_Pos             (0)                                               /*!< PWM_T::CMPDAT7: CMP7 Position             */
#define PWM_CMPDAT7_CMP7_Msk             (0xfffful << PWM_CMPDAT7_CMP7_Pos)                /*!< PWM_T::CMPDAT7: CMP7 Mask                 */

#define PWM_CMPDAT7_CMPD0_Pos            (16)                                              /*!< PWM_T::CMPDAT7: CMPD0 Position            */
#define PWM_CMPDAT7_CMPD0_Msk            (0xfffful << PWM_CMPDAT7_CMPD0_Pos)               /*!< PWM_T::CMPDAT7: CMPD0 Mask                */

#define PWM_CMPDAT7_CMPD1_Pos            (16)                                              /*!< PWM_T::CMPDAT7: CMPD1 Position            */
#define PWM_CMPDAT7_CMPD1_Msk            (0xfffful << PWM_CMPDAT7_CMPD1_Pos)               /*!< PWM_T::CMPDAT7: CMPD1 Mask                */

#define PWM_CMPDAT7_CMPD2_Pos            (16)                                              /*!< PWM_T::CMPDAT7: CMPD2 Position            */
#define PWM_CMPDAT7_CMPD2_Msk            (0xfffful << PWM_CMPDAT7_CMPD2_Pos)               /*!< PWM_T::CMPDAT7: CMPD2 Mask                */

#define PWM_CMPDAT7_CMPD3_Pos            (16)                                              /*!< PWM_T::CMPDAT7: CMPD3 Position            */
#define PWM_CMPDAT7_CMPD3_Msk            (0xfffful << PWM_CMPDAT7_CMPD3_Pos)               /*!< PWM_T::CMPDAT7: CMPD3 Mask                */

#define PWM_CMPDAT7_CMPD4_Pos            (16)                                              /*!< PWM_T::CMPDAT7: CMPD4 Position            */
#define PWM_CMPDAT7_CMPD4_Msk            (0xfffful << PWM_CMPDAT7_CMPD4_Pos)               /*!< PWM_T::CMPDAT7: CMPD4 Mask                */

#define PWM_CMPDAT7_CMPD5_Pos            (16)                                              /*!< PWM_T::CMPDAT7: CMPD5 Position            */
#define PWM_CMPDAT7_CMPD5_Msk            (0xfffful << PWM_CMPDAT7_CMPD5_Pos)               /*!< PWM_T::CMPDAT7: CMPD5 Mask                */

#define PWM_CMPDAT7_CMPD6_Pos            (16)                                              /*!< PWM_T::CMPDAT7: CMPD6 Position            */
#define PWM_CMPDAT7_CMPD6_Msk            (0xfffful << PWM_CMPDAT7_CMPD6_Pos)               /*!< PWM_T::CMPDAT7: CMPD6 Mask                */

#define PWM_CMPDAT7_CMPD7_Pos            (16)                                              /*!< PWM_T::CMPDAT7: CMPD7 Position            */
#define PWM_CMPDAT7_CMPD7_Msk            (0xfffful << PWM_CMPDAT7_CMPD7_Pos)               /*!< PWM_T::CMPDAT7: CMPD7 Mask                */

#define PWM_CTL2_BRKIEN_Pos              (16)                                              /*!< PWM_T::CTL2: BRKIEN Position              */
#define PWM_CTL2_BRKIEN_Msk              (0x1ul << PWM_CTL2_BRKIEN_Pos)                    /*!< PWM_T::CTL2: BRKIEN Mask                  */

#define PWM_CTL2_PINTTYPE_Pos            (17)                                              /*!< PWM_T::CTL2: PINTTYPE Position            */
#define PWM_CTL2_PINTTYPE_Msk            (0x1ul << PWM_CTL2_PINTTYPE_Pos)                  /*!< PWM_T::CTL2: PINTTYPE Mask                */

#define PWM_CTL2_DTCNT01_Pos              (24)                                             /*!< PWM_T::CTL2: DTCNT01 Position             */
#define PWM_CTL2_DTCNT01_Msk              (0x1ul << PWM_CTL2_DTCNT01_Pos)                  /*!< PWM_T::CTL2: DTCNT01 Mask                 */
                                                                                                                   
#define PWM_CTL2_DTCNT23_Pos              (25)                                             /*!< PWM_T::CTL2: DTCNT23 Position             */
#define PWM_CTL2_DTCNT23_Msk              (0x1ul << PWM_CTL2_DTCNT23_Pos)                  /*!< PWM_T::CTL2: DTCNT23 Mask                 */
                                                                                                                    
#define PWM_CTL2_DTCNT45_Pos              (26)                                             /*!< PWM_T::CTL2: DTCNT45 Position             */
#define PWM_CTL2_DTCNT45_Msk              (0x1ul << PWM_CTL2_DTCNT45_Pos)                  /*!< PWM_T::CTL2: DTCNT45 Mask                 */
                                                      
#define PWM_CTL2_DTCNT67_Pos              (27)                                             /*!< PWM_T::CTL2: DTCNT67 Position             */
#define PWM_CTL2_DTCNT67_Msk              (0x1ul << PWM_CTL2_DTCNT67_Pos)                  /*!< PWM_T::CTL2: DTCNT67 Mask                 */

#define PWM_CTL2_CNTCLR_Pos               (27)                                             /*!< PWM_T::CTL2: CNTCLR Position              */
#define PWM_CTL2_CNTCLR_Msk               (0x1ul << PWM_CTL2_CNTCLR_Pos)                   /*!< PWM_T::CTL2: CNTCLR Mask                  */
                                                                                                                   
#define PWM_CTL2_MODE_Pos                 (28)                                             /*!< PWM_T::CTL2: MODE Position                */
#define PWM_CTL2_MODE_Msk                 (0x3ul << PWM_CTL2_MODE_Pos)                     /*!< PWM_T::CTL2: MODE Mask                    */
                                                                                                                   
#define PWM_CTL2_GROUPEN_Pos              (30)                                             /*!< PWM_T::CTL2: GROUPEN Position             */
#define PWM_CTL2_GROUPEN_Msk              (0x1ul << PWM_CTL2_GROUPEN_Pos)                  /*!< PWM_T::CTL2: GROUPEN Mask                 */
                                                                                                                 
#define PWM_CTL2_CNTTYPE_Pos              (31)                                             /*!< PWM_T::CTL2: CNTTYPE Position             */
#define PWM_CTL2_CNTTYPE_Msk              (0x1ul << PWM_CTL2_CNTTYPE_Pos)                  /*!< PWM_T::CTL2: CNTTYPE Mask                 */

#define PWM_FLAG_ZF0_Pos              	 (0)                                               /*!< PWM_T::INTSTS: ZIF0 Position              */
#define PWM_FLAG_ZF0_Msk              	 (0x1ul << PWM_FLAG_ZF0_Pos)                    /*!< PWM_T::INTSTS: ZIF0 Mask                  */
 
#define PWM_FLAG_ZF1_Pos              	 (1)                                               /*!< PWM_T::INTSTS: ZIF1 Position              */
#define PWM_FLAG_ZF1_Msk              	 (0x1ul << PWM_FLAG_ZF1_Pos)                    /*!< PWM_T::INTSTS: ZIF1 Mask                  */
 
#define PWM_FLAG_ZF2_Pos              	 (2)                                               /*!< PWM_T::INTSTS: ZIF2 Position              */
#define PWM_FLAG_ZF2_Msk              	 (0x1ul << PWM_FLAG_ZF2_Pos)                    /*!< PWM_T::INTSTS: ZIF2 Mask                  */
 
#define PWM_FLAG_ZF3_Pos              	 (3)                                               /*!< PWM_T::INTSTS: ZIF3 Position              */
#define PWM_FLAG_ZF3_Msk              	 (0x1ul << PWM_FLAG_ZF3_Pos)                    /*!< PWM_T::INTSTS: ZIF3 Mask                  */
 
#define PWM_FLAG_ZF4_Pos              	 (4)                                               /*!< PWM_T::INTSTS: ZIF4 Position              */
#define PWM_FLAG_ZF4_Msk              	 (0x1ul << PWM_FLAG_ZF4_Pos)                    /*!< PWM_T::INTSTS: ZIF4 Mask                  */

#define PWM_FLAG_ZF5_Pos              	 (5)                                               /*!< PWM_T::INTSTS: ZIF5 Position              */
#define PWM_FLAG_ZF5_Msk              	 (0x1ul << PWM_FLAG_ZF5_Pos)                    /*!< PWM_T::INTSTS: ZIF5 Mask                  */
	 
#define PWM_FLAG_ZF6_Pos              	 (6)                                               /*!< PWM_T::INTSTS: ZIF6 Position              */
#define PWM_FLAG_ZF6_Msk              	 (0x1ul << PWM_FLAG_ZF6_Pos)                    /*!< PWM_T::INTSTS: ZIF6 Mask                  */

#define PWM_FLAG_ZF7_Pos              	 (7)                                               /*!< PWM_T::INTSTS: ZIF7 Position              */
#define PWM_FLAG_ZF7_Msk              	 (0x1ul << PWM_FLAG_ZF7_Pos)                    /*!< PWM_T::INTSTS: ZIF7 Mask                  */

#define PWM_FLAG_CMPDF0_Pos           	 (8)                                               /*!< PWM_T::INTSTS: CMPDIF0 Position           */
#define PWM_FLAG_CMPDF0_Msk           	 (0x1ul << PWM_FLAG_CMPDF0_Pos)                 /*!< PWM_T::INTSTS: CMPDIF0 Mask               */

#define PWM_FLAG_CMPDF1_Pos           	 (9)                                               /*!< PWM_T::INTSTS: CMPDIF1 Position           */
#define PWM_FLAG_CMPDF1_Msk           	 (0x1ul << PWM_FLAG_CMPDF1_Pos)                 /*!< PWM_T::INTSTS: CMPDIF1 Mask               */
 
#define PWM_FLAG_CMPDF2_Pos           	 (10)                                              /*!< PWM_T::INTSTS: CMPDIF2 Position           */
#define PWM_FLAG_CMPDF2_Msk           	 (0x1ul << PWM_FLAG_CMPDF2_Pos)                 /*!< PWM_T::INTSTS: CMPDIF2 Mask               */

#define PWM_FLAG_CMPDF3_Pos           	 (11)                                              /*!< PWM_T::INTSTS: CMPDIF3 Position           */
#define PWM_FLAG_CMPDF3_Msk           	 (0x1ul << PWM_FLAG_CMPDF3_Pos)                 /*!< PWM_T::INTSTS: CMPDIF3 Mask               */

#define PWM_FLAG_CMPDF4_Pos           	 (12)                                              /*!< PWM_T::INTSTS: CMPDIF4 Position           */
#define PWM_FLAG_CMPDF4_Msk           	 (0x1ul << PWM_FLAG_CMPDF4_Pos)                 /*!< PWM_T::INTSTS: CMPDIF4 Mask               */

#define PWM_FLAG_CMPDF5_Pos           	 (13)                                              /*!< PWM_T::INTSTS: CMPDIF5 Position           */
#define PWM_FLAG_CMPDF5_Msk           	 (0x1ul << PWM_FLAG_CMPDF5_Pos)                 /*!< PWM_T::INTSTS: CMPDIF5 Mask               */
 
#define PWM_FLAG_CMPDF6_Pos           	 (14)                                              /*!< PWM_T::INTSTS: CMPDIF6 Position           */
#define PWM_FLAG_CMPDF6_Msk           	 (0x1ul << PWM_FLAG_CMPDF6_Pos)                 /*!< PWM_T::INTSTS: CMPDIF6 Mask               */
 
#define PWM_FLAG_CMPDF7_Pos           	 (15)                                              /*!< PWM_T::INTSTS: CMPDIF7 Position           */
#define PWM_FLAG_CMPDF7_Msk           	 (0x1ul << PWM_FLAG_CMPDF7_Pos)                 /*!< PWM_T::INTSTS: CMPDIF7 Mask               */
 
#define PWM_FLAG_PF0_Pos              	 (16)                                              /*!< PWM_T::INTSTS: PIF0 Position              */
#define PWM_FLAG_PF0_Msk              	 (0x1ul << PWM_FLAG_PF0_Pos)                    /*!< PWM_T::INTSTS: PIF0 Mask                  */
 
#define PWM_FLAG_PF1_Pos              	 (17)                                              /*!< PWM_T::INTSTS: PIF1 Position              */
#define PWM_FLAG_PF1_Msk              	 (0x1ul << PWM_FLAG_PF1_Pos)                    /*!< PWM_T::INTSTS: PIF1 Mask                  */
	 
#define PWM_FLAG_PF2_Pos              	 (18)                                              /*!< PWM_T::INTSTS: PIF2 Position              */
#define PWM_FLAG_PF2_Msk              	 (0x1ul << PWM_FLAG_PF2_Pos)                    /*!< PWM_T::INTSTS: PIF2 Mask                  */
 
#define PWM_FLAG_PF3_Pos              	 (19)                                              /*!< PWM_T::INTSTS: PIF3 Position              */
#define PWM_FLAG_PF3_Msk              	 (0x1ul << PWM_FLAG_PF3_Pos)                    /*!< PWM_T::INTSTS: PIF3 Mask                  */

#define PWM_FLAG_PF4_Pos              	 (20)                                              /*!< PWM_T::INTSTS: PIF4 Position              */
#define PWM_FLAG_PF4_Msk              	 (0x1ul << PWM_FLAG_PF4_Pos)                    /*!< PWM_T::INTSTS: PIF4 Mask                  */
 
#define PWM_FLAG_PF5_Pos              	 (21)                                              /*!< PWM_T::INTSTS: PIF5 Position              */
#define PWM_FLAG_PF5_Msk              	 (0x1ul << PWM_FLAG_PF5_Pos)                    /*!< PWM_T::INTSTS: PIF5 Mask                  */
 
#define PWM_FLAG_PF6_Pos              	 (22)                                              /*!< PWM_T::INTSTS: PIF6 Position              */
#define PWM_FLAG_PF6_Msk              	 (0x1ul << PWM_FLAG_PF6_Pos)                    /*!< PWM_T::INTSTS: PIF6 Mask                  */
	 
#define PWM_FLAG_PF7_Pos              	 (23)                                              /*!< PWM_T::INTSTS: PIF7 Position              */
#define PWM_FLAG_PF7_Msk              	 (0x1ul << PWM_FLAG_PF7_Pos)                    /*!< PWM_T::INTSTS: PIF7 Mask                  */
	 
#define PWM_FLAG_CMPUF0_Pos           	 (24)                                              /*!< PWM_T::INTSTS: CMPUIF0 Position           */
#define PWM_FLAG_CMPUF0_Msk           	 (0x1ul << PWM_FLAG_CMPUF0_Pos)                 /*!< PWM_T::INTSTS: CMPUIF0 Mask               */
 
#define PWM_FLAG_CMPUF1_Pos           	 (25)                                              /*!< PWM_T::INTSTS: CMPUIF1 Position           */
#define PWM_FLAG_CMPUF1_Msk           	 (0x1ul << PWM_FLAG_CMPUF1_Pos)                 /*!< PWM_T::INTSTS: CMPUIF1 Mask               */
	 
#define PWM_FLAG_CMPUF2_Pos           	 (26)                                              /*!< PWM_T::INTSTS: CMPUIF2 Position           */
#define PWM_FLAG_CMPUF2_Msk           	 (0x1ul << PWM_FLAG_CMPUF2_Pos)                 /*!< PWM_T::INTSTS: CMPUIF2 Mask               */
 
#define PWM_FLAG_CMPUF3_Pos           	 (27)                                              /*!< PWM_T::INTSTS: CMPUIF3 Position           */
#define PWM_FLAG_CMPUF3_Msk           	 (0x1ul << PWM_FLAG_CMPUF3_Pos)                 /*!< PWM_T::INTSTS: CMPUIF3 Mask               */
	 
#define PWM_FLAG_CMPUF4_Pos           	 (28)                                              /*!< PWM_T::INTSTS: CMPUIF4 Position           */
#define PWM_FLAG_CMPUF4_Msk           	 (0x1ul << PWM_FLAG_CMPUF4_Pos)                 /*!< PWM_T::INTSTS: CMPUIF4 Mask               */
	 
#define PWM_FLAG_CMPUF5_Pos           	 (29)                                              /*!< PWM_T::INTSTS: CMPUIF5 Position           */
#define PWM_FLAG_CMPUF5_Msk           	 (0x1ul << PWM_FLAG_CMPUF5_Pos)                 /*!< PWM_T::INTSTS: CMPUIF5 Mask               */
	 
#define PWM_FLAG_CMPUF6_Pos           	 (30)                                              /*!< PWM_T::INTSTS: CMPUIF6 Position           */
#define PWM_FLAG_CMPUF6_Msk           	 (0x1ul << PWM_FLAG_CMPUF6_Pos)                 /*!< PWM_T::INTSTS: CMPUIF6 Mask               */
 
#define PWM_FLAG_CMPUF7_Pos           	 (31)                                              /*!< PWM_T::INTSTS: CMPUIF7 Position           */
#define PWM_FLAG_CMPUF7_Msk           	 (0x1ul << PWM_FLAG_CMPUF7_Pos)                 /*!< PWM_T::INTSTS: CMPUIF7 Mask               */

#define PWM_INTEN_ZIEN0_Pos              (0)                                               /*!< PWM_T::INTEN: ZIEN0 Position              */
#define PWM_INTEN_ZIEN0_Msk              (0x1ul << PWM_INTEN_ZIEN0_Pos)                    /*!< PWM_T::INTEN: ZIEN0 Mask                  */

#define PWM_INTEN_ZIEN1_Pos              (1)                                               /*!< PWM_T::INTEN: ZIEN1 Position              */
#define PWM_INTEN_ZIEN1_Msk              (0x1ul << PWM_INTEN_ZIEN1_Pos)                    /*!< PWM_T::INTEN: ZIEN1 Mask                  */

#define PWM_INTEN_ZIEN2_Pos              (2)                                               /*!< PWM_T::INTEN: ZIEN2 Position              */
#define PWM_INTEN_ZIEN2_Msk              (0x1ul << PWM_INTEN_ZIEN2_Pos)                    /*!< PWM_T::INTEN: ZIEN2 Mask                  */

#define PWM_INTEN_ZIEN3_Pos              (3)                                               /*!< PWM_T::INTEN: ZIEN3 Position              */
#define PWM_INTEN_ZIEN3_Msk              (0x1ul << PWM_INTEN_ZIEN3_Pos)                    /*!< PWM_T::INTEN: ZIEN3 Mask                  */

#define PWM_INTEN_ZIEN4_Pos              (4)                                               /*!< PWM_T::INTEN: ZIEN4 Position              */
#define PWM_INTEN_ZIEN4_Msk              (0x1ul << PWM_INTEN_ZIEN4_Pos)                    /*!< PWM_T::INTEN: ZIEN4 Mask                  */

#define PWM_INTEN_ZIEN5_Pos              (5)                                               /*!< PWM_T::INTEN: ZIEN5 Position              */
#define PWM_INTEN_ZIEN5_Msk              (0x1ul << PWM_INTEN_ZIEN5_Pos)                    /*!< PWM_T::INTEN: ZIEN5 Mask                  */


#define PWM_INTEN_ZIEN6_Pos              (6)                                               /*!< PWM_T::INTEN: ZIEN6 Position              */
#define PWM_INTEN_ZIEN6_Msk              (0x1ul << PWM_INTEN_ZIEN6_Pos)                    /*!< PWM_T::INTEN: ZIEN6 Mask                  */


#define PWM_INTEN_ZIEN7_Pos              (7)                                               /*!< PWM_T::INTEN: ZIEN7 Position              */
#define PWM_INTEN_ZIEN7_Msk              (0x1ul << PWM_INTEN_ZIEN7_Pos)                    /*!< PWM_T::INTEN: ZIEN7 Mask                  */

#define PWM_INTEN_CMPDIEN0_Pos           (8)                                               /*!< PWM_T::INTEN: CMPDIEN0 Position           */
#define PWM_INTEN_CMPDIEN0_Msk           (0x1ul << PWM_INTEN_CMPDIEN0_Pos)                 /*!< PWM_T::INTEN: CMPDIEN0 Mask               */

#define PWM_INTEN_CMPDIEN1_Pos           (9)                                               /*!< PWM_T::INTEN: CMPDIEN1 Position           */
#define PWM_INTEN_CMPDIEN1_Msk           (0x1ul << PWM_INTEN_CMPDIEN1_Pos)                 /*!< PWM_T::INTEN: CMPDIEN1 Mask               */

#define PWM_INTEN_CMPDIEN2_Pos           (10)                                              /*!< PWM_T::INTEN: CMPDIEN2 Position           */
#define PWM_INTEN_CMPDIEN2_Msk           (0x1ul << PWM_INTEN_CMPDIEN2_Pos)                 /*!< PWM_T::INTEN: CMPDIEN2 Mask               */

#define PWM_INTEN_CMPDIEN3_Pos           (11)                                              /*!< PWM_T::INTEN: CMPDIEN3 Position           */
#define PWM_INTEN_CMPDIEN3_Msk           (0x1ul << PWM_INTEN_CMPDIEN3_Pos)                 /*!< PWM_T::INTEN: CMPDIEN3 Mask               */

#define PWM_INTEN_CMPDIEN4_Pos           (12)                                              /*!< PWM_T::INTEN: CMPDIEN4 Position           */
#define PWM_INTEN_CMPDIEN4_Msk           (0x1ul << PWM_INTEN_CMPDIEN4_Pos)                 /*!< PWM_T::INTEN: CMPDIEN4 Mask               */

#define PWM_INTEN_CMPDIEN5_Pos           (13)                                              /*!< PWM_T::INTEN: CMPDIEN5 Position           */
#define PWM_INTEN_CMPDIEN5_Msk           (0x1ul << PWM_INTEN_CMPDIEN5_Pos)                 /*!< PWM_T::INTEN: CMPDIEN5 Mask               */


#define PWM_INTEN_CMPDIEN6_Pos           (14)                                              /*!< PWM_T::INTEN: CMPDIEN7 Position           */
#define PWM_INTEN_CMPDIEN6_Msk           (0x1ul << PWM_INTEN_CMPDIEN6_Pos)                 /*!< PWM_T::INTEN: CMPDIEN7 Mask               */

#define PWM_INTEN_CMPDIEN7_Pos           (15)                                              /*!< PWM_T::INTEN: CMPDIEN7 Position           */
#define PWM_INTEN_CMPDIEN7_Msk           (0x1ul << PWM_INTEN_CMPDIEN7_Pos)                 /*!< PWM_T::INTEN: CMPDIEN7 Mask               */

#define PWM_INTEN_PIEN0_Pos              (16)                                              /*!< PWM_T::INTEN: PIEN0 Position              */
#define PWM_INTEN_PIEN0_Msk              (0x1ul << PWM_INTEN_PIEN0_Pos)                    /*!< PWM_T::INTEN: PIEN0 Mask                  */

#define PWM_INTEN_PIEN1_Pos              (17)                                              /*!< PWM_T::INTEN: PIEN1 Position              */
#define PWM_INTEN_PIEN1_Msk              (0x1ul << PWM_INTEN_PIEN1_Pos)                    /*!< PWM_T::INTEN: PIEN1 Mask                  */

#define PWM_INTEN_PIEN2_Pos              (18)                                              /*!< PWM_T::INTEN: PIEN2 Position              */
#define PWM_INTEN_PIEN2_Msk              (0x1ul << PWM_INTEN_PIEN2_Pos)                    /*!< PWM_T::INTEN: PIEN2 Mask                  */

#define PWM_INTEN_PIEN3_Pos              (19)                                              /*!< PWM_T::INTEN: PIEN3 Position              */
#define PWM_INTEN_PIEN3_Msk              (0x1ul << PWM_INTEN_PIEN3_Pos)                    /*!< PWM_T::INTEN: PIEN3 Mask                  */

#define PWM_INTEN_PIEN4_Pos              (20)                                              /*!< PWM_T::INTEN: PIEN4 Position              */
#define PWM_INTEN_PIEN4_Msk              (0x1ul << PWM_INTEN_PIEN4_Pos)                    /*!< PWM_T::INTEN: PIEN4 Mask                  */

#define PWM_INTEN_PIEN5_Pos              (21)                                              /*!< PWM_T::INTEN: PIEN5 Position              */
#define PWM_INTEN_PIEN5_Msk              (0x1ul << PWM_INTEN_PIEN5_Pos)                    /*!< PWM_T::INTEN: PIEN5 Mask                  */


#define PWM_INTEN_PIEN6_Pos              (22)                                              /*!< PWM_T::INTEN: PIEN6 Position              */
#define PWM_INTEN_PIEN6_Msk              (0x1ul << PWM_INTEN_PIEN6_Pos)                    /*!< PWM_T::INTEN: PIEN6 Mask                  */


#define PWM_INTEN_PIEN7_Pos              (23)                                              /*!< PWM_T::INTEN: PIEN7 Position              */
#define PWM_INTEN_PIEN7_Msk              (0x1ul << PWM_INTEN_PIEN7_Pos)                    /*!< PWM_T::INTEN: PIEN7 Mask                  */

#define PWM_INTEN_CMPUIEN0_Pos           (24)                                              /*!< PWM_T::INTEN: CMPUIEN0 Position           */
#define PWM_INTEN_CMPUIEN0_Msk           (0x1ul << PWM_INTEN_CMPUIEN0_Pos)                 /*!< PWM_T::INTEN: CMPUIEN0 Mask               */

#define PWM_INTEN_CMPUIEN1_Pos           (25)                                              /*!< PWM_T::INTEN: CMPUIEN1 Position           */
#define PWM_INTEN_CMPUIEN1_Msk           (0x1ul << PWM_INTEN_CMPUIEN1_Pos)                 /*!< PWM_T::INTEN: CMPUIEN1 Mask               */

#define PWM_INTEN_CMPUIEN2_Pos           (26)                                              /*!< PWM_T::INTEN: CMPUIEN2 Position           */
#define PWM_INTEN_CMPUIEN2_Msk           (0x1ul << PWM_INTEN_CMPUIEN2_Pos)                 /*!< PWM_T::INTEN: CMPUIEN2 Mask               */

#define PWM_INTEN_CMPUIEN3_Pos           (27)                                              /*!< PWM_T::INTEN: CMPUIEN3 Position           */
#define PWM_INTEN_CMPUIEN3_Msk           (0x1ul << PWM_INTEN_CMPUIEN3_Pos)                 /*!< PWM_T::INTEN: CMPUIEN3 Mask               */

#define PWM_INTEN_CMPUIEN4_Pos           (28)                                              /*!< PWM_T::INTEN: CMPUIEN4 Position           */
#define PWM_INTEN_CMPUIEN4_Msk           (0x1ul << PWM_INTEN_CMPUIEN4_Pos)                 /*!< PWM_T::INTEN: CMPUIEN4 Mask               */

#define PWM_INTEN_CMPUIEN5_Pos           (29)                                              /*!< PWM_T::INTEN: CMPUIEN5 Position           */
#define PWM_INTEN_CMPUIEN5_Msk           (0x1ul << PWM_INTEN_CMPUIEN5_Pos)                 /*!< PWM_T::INTEN: CMPUIEN5 Mask               */

#define PWM_INTEN_CMPUIEN6_Pos           (30)                                              /*!< PWM_T::INTEN: CMPUIEN6 Position           */
#define PWM_INTEN_CMPUIEN6_Msk           (0x1ul << PWM_INTEN_CMPUIEN6_Pos)                 /*!< PWM_T::INTEN: CMPUIEN6 Mask               */

#define PWM_INTEN_CMPUIEN7_Pos           (31)                                              /*!< PWM_T::INTEN: CMPUIEN7 Position           */
#define PWM_INTEN_CMPUIEN7_Msk           (0x1ul << PWM_INTEN_CMPUIEN7_Pos)                 /*!< PWM_T::INTEN: CMPUIEN7 Mask               */

#define PWM_INTSTS_ZIF0_Pos              (0)                                               /*!< PWM_T::INTSTS: ZIF0 Position              */
#define PWM_INTSTS_ZIF0_Msk              (0x1ul << PWM_INTSTS_ZIF0_Pos)                    /*!< PWM_T::INTSTS: ZIF0 Mask                  */

#define PWM_INTSTS_ZIF1_Pos              (1)                                               /*!< PWM_T::INTSTS: ZIF1 Position              */
#define PWM_INTSTS_ZIF1_Msk              (0x1ul << PWM_INTSTS_ZIF1_Pos)                    /*!< PWM_T::INTSTS: ZIF1 Mask                  */

#define PWM_INTSTS_ZIF2_Pos              (2)                                               /*!< PWM_T::INTSTS: ZIF2 Position              */
#define PWM_INTSTS_ZIF2_Msk              (0x1ul << PWM_INTSTS_ZIF2_Pos)                    /*!< PWM_T::INTSTS: ZIF2 Mask                  */

#define PWM_INTSTS_ZIF3_Pos              (3)                                               /*!< PWM_T::INTSTS: ZIF3 Position              */
#define PWM_INTSTS_ZIF3_Msk              (0x1ul << PWM_INTSTS_ZIF3_Pos)                    /*!< PWM_T::INTSTS: ZIF3 Mask                  */

#define PWM_INTSTS_ZIF4_Pos              (4)                                               /*!< PWM_T::INTSTS: ZIF4 Position              */
#define PWM_INTSTS_ZIF4_Msk              (0x1ul << PWM_INTSTS_ZIF4_Pos)                    /*!< PWM_T::INTSTS: ZIF4 Mask                  */

#define PWM_INTSTS_ZIF5_Pos              (5)                                               /*!< PWM_T::INTSTS: ZIF5 Position              */
#define PWM_INTSTS_ZIF5_Msk              (0x1ul << PWM_INTSTS_ZIF5_Pos)                    /*!< PWM_T::INTSTS: ZIF5 Mask                  */

#define PWM_INTSTS_ZIF6_Pos              (6)                                               /*!< PWM_T::INTSTS: ZIF6 Position              */
#define PWM_INTSTS_ZIF6_Msk              (0x1ul << PWM_INTSTS_ZIF6_Pos)                    /*!< PWM_T::INTSTS: ZIF6 Mask                  */

#define PWM_INTSTS_ZIF7_Pos              (7)                                               /*!< PWM_T::INTSTS: ZIF7 Position              */
#define PWM_INTSTS_ZIF7_Msk              (0x1ul << PWM_INTSTS_ZIF7_Pos)                    /*!< PWM_T::INTSTS: ZIF7 Mask                  */

#define PWM_INTSTS_CMPDIF0_Pos           (8)                                               /*!< PWM_T::INTSTS: CMPDIF0 Position           */
#define PWM_INTSTS_CMPDIF0_Msk           (0x1ul << PWM_INTSTS_CMPDIF0_Pos)                 /*!< PWM_T::INTSTS: CMPDIF0 Mask               */

#define PWM_INTSTS_CMPDIF1_Pos           (9)                                               /*!< PWM_T::INTSTS: CMPDIF1 Position           */
#define PWM_INTSTS_CMPDIF1_Msk           (0x1ul << PWM_INTSTS_CMPDIF1_Pos)                 /*!< PWM_T::INTSTS: CMPDIF1 Mask               */

#define PWM_INTSTS_CMPDIF2_Pos           (10)                                              /*!< PWM_T::INTSTS: CMPDIF2 Position           */
#define PWM_INTSTS_CMPDIF2_Msk           (0x1ul << PWM_INTSTS_CMPDIF2_Pos)                 /*!< PWM_T::INTSTS: CMPDIF2 Mask               */

#define PWM_INTSTS_CMPDIF3_Pos           (11)                                              /*!< PWM_T::INTSTS: CMPDIF3 Position           */
#define PWM_INTSTS_CMPDIF3_Msk           (0x1ul << PWM_INTSTS_CMPDIF3_Pos)                 /*!< PWM_T::INTSTS: CMPDIF3 Mask               */

#define PWM_INTSTS_CMPDIF4_Pos           (12)                                              /*!< PWM_T::INTSTS: CMPDIF4 Position           */
#define PWM_INTSTS_CMPDIF4_Msk           (0x1ul << PWM_INTSTS_CMPDIF4_Pos)                 /*!< PWM_T::INTSTS: CMPDIF4 Mask               */

#define PWM_INTSTS_CMPDIF5_Pos           (13)                                              /*!< PWM_T::INTSTS: CMPDIF5 Position           */
#define PWM_INTSTS_CMPDIF5_Msk           (0x1ul << PWM_INTSTS_CMPDIF5_Pos)                 /*!< PWM_T::INTSTS: CMPDIF5 Mask               */

#define PWM_INTSTS_CMPDIF6_Pos           (14)                                              /*!< PWM_T::INTSTS: CMPDIF6 Position           */
#define PWM_INTSTS_CMPDIF6_Msk           (0x1ul << PWM_INTSTS_CMPDIF6_Pos)                 /*!< PWM_T::INTSTS: CMPDIF6 Mask               */

#define PWM_INTSTS_CMPDIF7_Pos           (15)                                              /*!< PWM_T::INTSTS: CMPDIF7 Position           */
#define PWM_INTSTS_CMPDIF7_Msk           (0x1ul << PWM_INTSTS_CMPDIF7_Pos)                 /*!< PWM_T::INTSTS: CMPDIF7 Mask               */

#define PWM_INTSTS_PIF0_Pos              (16)                                              /*!< PWM_T::INTSTS: PIF0 Position              */
#define PWM_INTSTS_PIF0_Msk              (0x1ul << PWM_INTSTS_PIF0_Pos)                    /*!< PWM_T::INTSTS: PIF0 Mask                  */

#define PWM_INTSTS_PIF1_Pos              (17)                                              /*!< PWM_T::INTSTS: PIF1 Position              */
#define PWM_INTSTS_PIF1_Msk              (0x1ul << PWM_INTSTS_PIF1_Pos)                    /*!< PWM_T::INTSTS: PIF1 Mask                  */

#define PWM_INTSTS_PIF2_Pos              (18)                                              /*!< PWM_T::INTSTS: PIF2 Position              */
#define PWM_INTSTS_PIF2_Msk              (0x1ul << PWM_INTSTS_PIF2_Pos)                    /*!< PWM_T::INTSTS: PIF2 Mask                  */

#define PWM_INTSTS_PIF3_Pos              (19)                                              /*!< PWM_T::INTSTS: PIF3 Position              */
#define PWM_INTSTS_PIF3_Msk              (0x1ul << PWM_INTSTS_PIF3_Pos)                    /*!< PWM_T::INTSTS: PIF3 Mask                  */

#define PWM_INTSTS_PIF4_Pos              (20)                                              /*!< PWM_T::INTSTS: PIF4 Position              */
#define PWM_INTSTS_PIF4_Msk              (0x1ul << PWM_INTSTS_PIF4_Pos)                    /*!< PWM_T::INTSTS: PIF4 Mask                  */

#define PWM_INTSTS_PIF5_Pos              (21)                                              /*!< PWM_T::INTSTS: PIF5 Position              */
#define PWM_INTSTS_PIF5_Msk              (0x1ul << PWM_INTSTS_PIF5_Pos)                    /*!< PWM_T::INTSTS: PIF5 Mask                  */

#define PWM_INTSTS_PIF6_Pos              (22)                                              /*!< PWM_T::INTSTS: PIF6 Position              */
#define PWM_INTSTS_PIF6_Msk              (0x1ul << PWM_INTSTS_PIF6_Pos)                    /*!< PWM_T::INTSTS: PIF6 Mask                  */

#define PWM_INTSTS_PIF7_Pos              (23)                                              /*!< PWM_T::INTSTS: PIF7 Position              */
#define PWM_INTSTS_PIF7_Msk              (0x1ul << PWM_INTSTS_PIF7_Pos)                    /*!< PWM_T::INTSTS: PIF7 Mask                  */

#define PWM_INTSTS_CMPUIF0_Pos           (24)                                              /*!< PWM_T::INTSTS: CMPUIF0 Position           */
#define PWM_INTSTS_CMPUIF0_Msk           (0x1ul << PWM_INTSTS_CMPUIF0_Pos)                 /*!< PWM_T::INTSTS: CMPUIF0 Mask               */

#define PWM_INTSTS_CMPUIF1_Pos           (25)                                              /*!< PWM_T::INTSTS: CMPUIF1 Position           */
#define PWM_INTSTS_CMPUIF1_Msk           (0x1ul << PWM_INTSTS_CMPUIF1_Pos)                 /*!< PWM_T::INTSTS: CMPUIF1 Mask               */

#define PWM_INTSTS_CMPUIF2_Pos           (26)                                              /*!< PWM_T::INTSTS: CMPUIF2 Position           */
#define PWM_INTSTS_CMPUIF2_Msk           (0x1ul << PWM_INTSTS_CMPUIF2_Pos)                 /*!< PWM_T::INTSTS: CMPUIF2 Mask               */

#define PWM_INTSTS_CMPUIF3_Pos           (27)                                              /*!< PWM_T::INTSTS: CMPUIF3 Position           */
#define PWM_INTSTS_CMPUIF3_Msk           (0x1ul << PWM_INTSTS_CMPUIF3_Pos)                 /*!< PWM_T::INTSTS: CMPUIF3 Mask               */

#define PWM_INTSTS_CMPUIF4_Pos           (28)                                              /*!< PWM_T::INTSTS: CMPUIF4 Position           */
#define PWM_INTSTS_CMPUIF4_Msk           (0x1ul << PWM_INTSTS_CMPUIF4_Pos)                 /*!< PWM_T::INTSTS: CMPUIF4 Mask               */

#define PWM_INTSTS_CMPUIF5_Pos           (29)                                              /*!< PWM_T::INTSTS: CMPUIF5 Position           */
#define PWM_INTSTS_CMPUIF5_Msk           (0x1ul << PWM_INTSTS_CMPUIF5_Pos)                 /*!< PWM_T::INTSTS: CMPUIF5 Mask               */

#define PWM_INTSTS_CMPUIF6_Pos           (30)                                              /*!< PWM_T::INTSTS: CMPUIF6 Position           */
#define PWM_INTSTS_CMPUIF6_Msk           (0x1ul << PWM_INTSTS_CMPUIF6_Pos)                 /*!< PWM_T::INTSTS: CMPUIF6 Mask               */

#define PWM_INTSTS_CMPUIF7_Pos           (31)                                              /*!< PWM_T::INTSTS: CMPUIF7 Position           */
#define PWM_INTSTS_CMPUIF7_Msk           (0x1ul << PWM_INTSTS_CMPUIF7_Pos)                 /*!< PWM_T::INTSTS: CMPUIF7 Mask               */

#define PWM_POEN_POEN0_Pos               (0)                                               /*!< PWM_T::POEN: POEN0 Position               */
#define PWM_POEN_POEN0_Msk               (0x1ul << PWM_POEN_POEN0_Pos)                     /*!< PWM_T::POEN: POEN0 Mask                   */

#define PWM_POEN_POEN1_Pos               (1)                                               /*!< PWM_T::POEN: POEN1 Position               */
#define PWM_POEN_POEN1_Msk               (0x1ul << PWM_POEN_POEN1_Pos)                     /*!< PWM_T::POEN: POEN1 Mask                   */

#define PWM_POEN_POEN2_Pos               (2)                                               /*!< PWM_T::POEN: POEN2 Position               */
#define PWM_POEN_POEN2_Msk               (0x1ul << PWM_POEN_POEN2_Pos)                     /*!< PWM_T::POEN: POEN2 Mask                   */

#define PWM_POEN_POEN3_Pos               (3)                                               /*!< PWM_T::POEN: POEN3 Position               */
#define PWM_POEN_POEN3_Msk               (0x1ul << PWM_POEN_POEN3_Pos)                     /*!< PWM_T::POEN: POEN3 Mask                   */

#define PWM_POEN_POEN4_Pos               (4)                                               /*!< PWM_T::POEN: POEN4 Position               */
#define PWM_POEN_POEN4_Msk               (0x1ul << PWM_POEN_POEN4_Pos)                     /*!< PWM_T::POEN: POEN4 Mask                   */

#define PWM_POEN_POEN5_Pos               (5)                                               /*!< PWM_T::POEN: POEN5 Position               */
#define PWM_POEN_POEN5_Msk               (0x1ul << PWM_POEN_POEN5_Pos)                     /*!< PWM_T::POEN: POEN5 Mask                   */

#define PWM_POEN_POEN6_Pos               (6)                                               /*!< PWM_T::POEN: POEN6 Position               */
#define PWM_POEN_POEN6_Msk               (0x1ul << PWM_POEN_POEN4_Pos)                     /*!< PWM_T::POEN: POEN6 Mask                   */

#define PWM_POEN_POEN7_Pos               (7)                                               /*!< PWM_T::POEN: POEN7 Position               */
#define PWM_POEN_POEN7_Msk               (0x1ul << PWM_POEN_POEN7_Pos)                     /*!< PWM_T::POEN: POEN7 Mask                   */

#define PWM_BRKCTL_BRK0EN_Pos            (0)                                               /*!< PWM_T::BRKCTL: BRK0EN Position            */
#define PWM_BRKCTL_BRK0EN_Msk            (0x1ul << PWM_BRKCTL_BRK0EN_Pos)                  /*!< PWM_T::BRKCTL: BRK0EN Mask                */

#define PWM_BRKCTL_BRK1EN_Pos            (1)                                               /*!< PWM_T::BRKCTL: BRK1EN Position            */
#define PWM_BRKCTL_BRK1EN_Msk            (0x1ul << PWM_BRKCTL_BRK1EN_Pos)                  /*!< PWM_T::BRKCTL: BRK1EN Mask                */

#define PWM_BRKCTL_BRK0SEL_Pos           (2)                                               /*!< PWM_T::BRKCTL: BRK0SEL Position           */
#define PWM_BRKCTL_BRK0SEL_Msk           (0x1ul << PWM_BRKCTL_BRK0SEL_Pos)                 /*!< PWM_T::BRKCTL: BRK0SEL Mask               */

#define PWM_BRKCTL_BRK1SEL_Pos           (3)                                               /*!< PWM_T::BRKCTL: BRK1SEL Position           */
#define PWM_BRKCTL_BRK1SEL_Msk           (0x1ul << PWM_BRKCTL_BRK1SEL_Pos)                 /*!< PWM_T::BRKCTL: BRK1SEL Mask               */

#define PWM_BRKCTL_BRKSTS_Pos            (7)                                               /*!< PWM_T::BRKCTL: BRKSTS Position            */
#define PWM_BRKCTL_BRKSTS_Msk            (0x1ul << PWM_BRKCTL_BRKSTS_Pos)                  /*!< PWM_T::BRKCTL: BRKSTS Mask                */

#define PWM_BRKCTL_BRKACT_Pos            (8)                                               /*!< PWM_T::BRKCTL: BRKACT Position            */
#define PWM_BRKCTL_BRKACT_Msk            (0x1ul << PWM_BRKCTL_BRKACT_Pos)                  /*!< PWM_T::BRKCTL: BRKACT Mask                */

#define PWM_BRKCTL_SWBRK_Pos             (9)                                               /*!< PWM_T::BRKCTL: SWBRK Position             */
#define PWM_BRKCTL_SWBRK_Msk             (0x1ul << PWM_BRKCTL_SWBRK_Pos)                   /*!< PWM_T::BRKCTL: SWBRK Mask                 */

#define PWM_BRKCTL_BRKIF0_Pos            (16)                                              /*!< PWM_T::BRKCTL: BRKIF0 Position            */	// INTSTS -> BRKCTL
#define PWM_BRKCTL_BRKIF0_Msk            (0x1ul << PWM_BRKCTL_BRKIF0_Pos)                  /*!< PWM_T::BRKSTS: BRKIF0 Mask                */	// INTSTS -> BRKCTL

#define PWM_BRKCTL_BRKIF1_Pos            (17)                                              /*!< PWM_T::BRKSTS: BRKIF1 Position            */	// INTSTS -> BRKCTL
#define PWM_BRKCTL_BRKIF1_Msk            (0x1ul << PWM_BRKCTL_BRKIF1_Pos)                  /*!< PWM_T::BRKSTS: BRKIF1 Mask                */	// INTSTS -> BRKCTL

#define PWM_BRKCTL_BRKF0_Pos             (18)                                              /*!< PWM_T::BRKCTL: BRKIF0 Position            */	// INTSTS -> BRKCTL
#define PWM_BRKCTL_BRKF0_Msk             (0x1ul << PWM_BRKCTL_BRKF0_Pos)                  /*!< PWM_T::BRKSTS: BRKIF0 Mask                */	// INTSTS -> BRKCTL

#define PWM_BRKCTL_BRKF1_Pos             (19)                                              /*!< PWM_T::BRKSTS: BRKIF1 Position            */	// INTSTS -> BRKCTL
#define PWM_BRKCTL_BRKF1_Msk             (0x1ul << PWM_BRKCTL_BRKF1_Pos)                  /*!< PWM_T::BRKSTS: BRKIF1 Mask 								*/											

#define PWM_BRKCTL_BKOD0_Pos             (24)                                              /*!< PWM_T::BRKCTL: BKOD0 Position             */
#define PWM_BRKCTL_BKOD0_Msk             (0x1ul << PWM_BRKCTL_BKOD0_Pos)                   /*!< PWM_T::BRKCTL: BKOD0 Mask                 */

#define PWM_BRKCTL_BKOD1_Pos             (25)                                              /*!< PWM_T::BRKCTL: BKOD1 Position             */
#define PWM_BRKCTL_BKOD1_Msk             (0x1ul << PWM_BRKCTL_BKOD1_Pos)                   /*!< PWM_T::BRKCTL: BKOD1 Mask                 */

#define PWM_BRKCTL_BKOD2_Pos             (26)                                              /*!< PWM_T::BRKCTL: BKOD2 Position             */
#define PWM_BRKCTL_BKOD2_Msk             (0x1ul << PWM_BRKCTL_BKOD2_Pos)                   /*!< PWM_T::BRKCTL: BKOD2 Mask                 */

#define PWM_BRKCTL_BKOD3_Pos             (27)                                              /*!< PWM_T::BRKCTL: BKOD3 Position             */
#define PWM_BRKCTL_BKOD3_Msk             (0x1ul << PWM_BRKCTL_BKOD3_Pos)                   /*!< PWM_T::BRKCTL: BKOD3 Mask                 */

#define PWM_BRKCTL_BKOD4_Pos             (28)                                              /*!< PWM_T::BRKCTL: BKOD4 Position             */
#define PWM_BRKCTL_BKOD4_Msk             (0x1ul << PWM_BRKCTL_BKOD4_Pos)                   /*!< PWM_T::BRKCTL: BKOD4 Mask                 */

#define PWM_BRKCTL_BKOD5_Pos             (29)                                              /*!< PWM_T::BRKCTL: BKOD5 Position             */
#define PWM_BRKCTL_BKOD5_Msk             (0x1ul << PWM_BRKCTL_BKOD5_Pos)                   /*!< PWM_T::BRKCTL: BKOD5 Mask                 */

#define PWM_BRKCTL_D6BKOD_Pos            (30)                                              /*!< PWM_T::BRKCTL: D6BKOD Position            */
#define PWM_BRKCTL_D6BKOD_Msk            (0x1ul << PWM_BRKCTL_D6BKOD_Pos)                  /*!< PWM_T::BRKCTL: D6BKOD Mask                */

#define PWM_BRKCTL_D7BKOD_Pos            (31)                                              /*!< PWM_T::BRKCTL: D7BKOD Position            */
#define PWM_BRKCTL_D7BKOD_Msk            (0x1ul << PWM_BRKCTL_D7BKOD_Pos)                  /*!< PWM_T::BRKCTL: D7BKOD Mask                */

#define PWM_DTCTL_DTI01_Pos              (0)                                               /*!< PWM_T::DTCTL: DTI01 Position              */
#define PWM_DTCTL_DTI01_Msk              (0xfful << PWM_DTCTL_DTI01_Pos)                   /*!< PWM_T::DTCTL: DTI01 Mask                  */

#define PWM_DTCTL_DTI23_Pos              (8)                                               /*!< PWM_T::DTCTL: DTI23 Position              */
#define PWM_DTCTL_DTI23_Msk              (0xfful << PWM_DTCTL_DTI23_Pos)                   /*!< PWM_T::DTCTL: DTI23 Mask                  */

#define PWM_DTCTL_DTI45_Pos              (16)                                              /*!< PWM_T::DTCTL: DTI45 Position              */
#define PWM_DTCTL_DTI45_Msk              (0xfful << PWM_DTCTL_DTI45_Pos)                   /*!< PWM_T::DTCTL: DTI45 Mask                  */

#define PWM_DTCTL_DTI67_Pos              (24)                                              /*!< PWM_T::DTCTL: DTI67 Position              */
#define PWM_DTCTL_DTI67_Msk              (0xfful << PWM_DTCTL_DTI67_Pos)                   /*!< PWM_T::DTCTL: DTI67 Mask                  */

#define PWM_ADCTCTL0_CUTRGEN0_Pos        (0)                                               /*!< PWM_T::ADCTCTL0: CUTRGEN0 Position        */
#define PWM_ADCTCTL0_CUTRGEN0_Msk        (0x1ul << PWM_ADCTCTL0_CUTRGEN0_Pos)              /*!< PWM_T::ADCTCTL0: CUTRGEN0 Mask            */

#define PWM_ADCTCTL0_CPTRGEN0_Pos        (1)                                               /*!< PWM_T::ADCTCTL0: CPTRGEN0 Position        */
#define PWM_ADCTCTL0_CPTRGEN0_Msk        (0x1ul << PWM_ADCTCTL0_CPTRGEN0_Pos)              /*!< PWM_T::ADCTCTL0: CPTRGEN0 Mask            */

#define PWM_ADCTCTL0_CDTRGEN0_Pos        (2)                                               /*!< PWM_T::ADCTCTL0: CDTRGEN0 Position        */
#define PWM_ADCTCTL0_CDTRGEN0_Msk        (0x1ul << PWM_ADCTCTL0_CDTRGEN0_Pos)              /*!< PWM_T::ADCTCTL0: CDTRGEN0 Mask            */

#define PWM_ADCTCTL0_ZPTRGEN0_Pos        (3)                                               /*!< PWM_T::ADCTCTL0: ZPTRGEN0 Position        */
#define PWM_ADCTCTL0_ZPTRGEN0_Msk        (0x1ul << PWM_ADCTCTL0_ZPTRGEN0_Pos)              /*!< PWM_T::ADCTCTL0: ZPTRGEN0 Mask            */

#define PWM_ADCTCTL0_CUTRGEN1_Pos        (8)                                               /*!< PWM_T::ADCTCTL0: CUTRGEN1 Position        */
#define PWM_ADCTCTL0_CUTRGEN1_Msk        (0x1ul << PWM_ADCTCTL0_CUTRGEN1_Pos)              /*!< PWM_T::ADCTCTL0: CUTRGEN1 Mask            */

#define PWM_ADCTCTL0_CPTRGEN1_Pos        (9)                                               /*!< PWM_T::ADCTCTL0: CPTRGEN1 Position        */
#define PWM_ADCTCTL0_CPTRGEN1_Msk        (0x1ul << PWM_ADCTCTL0_CPTRGEN1_Pos)              /*!< PWM_T::ADCTCTL0: CPTRGEN1 Mask            */

#define PWM_ADCTCTL0_CDTRGEN1_Pos        (10)                                              /*!< PWM_T::ADCTCTL0: CDTRGEN1 Position        */
#define PWM_ADCTCTL0_CDTRGEN1_Msk        (0x1ul << PWM_ADCTCTL0_CDTRGEN1_Pos)              /*!< PWM_T::ADCTCTL0: CDTRGEN1 Mask            */

#define PWM_ADCTCTL0_ZPTRGEN1_Pos        (11)                                              /*!< PWM_T::ADCTCTL0: ZPTRGEN1 Position        */
#define PWM_ADCTCTL0_ZPTRGEN1_Msk        (0x1ul << PWM_ADCTCTL0_ZPTRGEN1_Pos)              /*!< PWM_T::ADCTCTL0: ZPTRGEN1 Mask            */

#define PWM_ADCTCTL0_CUTRGEN2_Pos        (16)                                              /*!< PWM_T::ADCTCTL0: CUTRGEN2 Position        */
#define PWM_ADCTCTL0_CUTRGEN2_Msk        (0x1ul << PWM_ADCTCTL0_CUTRGEN2_Pos)              /*!< PWM_T::ADCTCTL0: CUTRGEN2 Mask            */

#define PWM_ADCTCTL0_CPTRGEN2_Pos        (17)                                              /*!< PWM_T::ADCTCTL0: CPTRGEN2 Position        */
#define PWM_ADCTCTL0_CPTRGEN2_Msk        (0x1ul << PWM_ADCTCTL0_CPTRGEN2_Pos)              /*!< PWM_T::ADCTCTL0: CPTRGEN2 Mask            */

#define PWM_ADCTCTL0_CDTRGEN2_Pos        (18)                                              /*!< PWM_T::ADCTCTL0: CDTRGEN2 Position        */
#define PWM_ADCTCTL0_CDTRGEN2_Msk        (0x1ul << PWM_ADCTCTL0_CDTRGEN2_Pos)              /*!< PWM_T::ADCTCTL0: CDTRGEN2 Mask            */

#define PWM_ADCTCTL0_ZPTRGEN2_Pos        (19)                                              /*!< PWM_T::ADCTCTL0: ZPTRGEN2 Position        */
#define PWM_ADCTCTL0_ZPTRGEN2_Msk        (0x1ul << PWM_ADCTCTL0_ZPTRGEN2_Pos)              /*!< PWM_T::ADCTCTL0: ZPTRGEN2 Mask            */

#define PWM_ADCTCTL0_CUTRGEN3_Pos        (24)                                              /*!< PWM_T::ADCTCTL0: CUTRGEN3 Position        */
#define PWM_ADCTCTL0_CUTRGEN3_Msk        (0x1ul << PWM_ADCTCTL0_CUTRGEN3_Pos)              /*!< PWM_T::ADCTCTL0: CUTRGEN3 Mask            */

#define PWM_ADCTCTL0_CPTRGEN3_Pos        (25)                                              /*!< PWM_T::ADCTCTL0: CPTRGEN3 Position        */
#define PWM_ADCTCTL0_CPTRGEN3_Msk        (0x1ul << PWM_ADCTCTL0_CPTRGEN3_Pos)              /*!< PWM_T::ADCTCTL0: CPTRGEN3 Mask            */

#define PWM_ADCTCTL0_CDTRGEN3_Pos        (26)                                              /*!< PWM_T::ADCTCTL0: CDTRGEN3 Position        */
#define PWM_ADCTCTL0_CDTRGEN3_Msk        (0x1ul << PWM_ADCTCTL0_CDTRGEN3_Pos)              /*!< PWM_T::ADCTCTL0: CDTRGEN3 Mask            */

#define PWM_ADCTCTL0_ZPTRGEN3_Pos        (27)                                              /*!< PWM_T::ADCTCTL0: ZPTRGEN3 Position        */
#define PWM_ADCTCTL0_ZPTRGEN3_Msk        (0x1ul << PWM_ADCTCTL0_ZPTRGEN3_Pos)              /*!< PWM_T::ADCTCTL0: ZPTRGEN3 Mask            */

#define PWM_ADCTCTL1_CUTRGEN4_Pos        (0)                                               /*!< PWM_T::ADCTCTL1: CUTRGEN4 Position        */
#define PWM_ADCTCTL1_CUTRGEN4_Msk        (0x1ul << PWM_ADCTCTL1_CUTRGEN4_Pos)              /*!< PWM_T::ADCTCTL1: CUTRGEN4 Mask            */

#define PWM_ADCTCTL1_CPTRGEN4_Pos        (1)                                               /*!< PWM_T::ADCTCTL1: CPTRGEN4 Position        */
#define PWM_ADCTCTL1_CPTRGEN4_Msk        (0x1ul << PWM_ADCTCTL1_CPTRGEN4_Pos)              /*!< PWM_T::ADCTCTL1: CPTRGEN4 Mask            */

#define PWM_ADCTCTL1_CDTRGEN4_Pos        (2)                                               /*!< PWM_T::ADCTCTL1: CDTRGEN4 Position        */
#define PWM_ADCTCTL1_CDTRGEN4_Msk        (0x1ul << PWM_ADCTCTL1_CDTRGEN4_Pos)              /*!< PWM_T::ADCTCTL1: CDTRGEN4 Mask            */

#define PWM_ADCTCTL1_ZPTRGEN4_Pos        (3)                                               /*!< PWM_T::ADCTCTL1: ZPTRGEN4 Position        */
#define PWM_ADCTCTL1_ZPTRGEN4_Msk        (0x1ul << PWM_ADCTCTL1_ZPTRGEN4_Pos)              /*!< PWM_T::ADCTCTL1: ZPTRGEN4 Mask            */

#define PWM_ADCTCTL1_CUTRGEN5_Pos        (8)                                               /*!< PWM_T::ADCTCTL1: CUTRGEN5 Position        */
#define PWM_ADCTCTL1_CUTRGEN5_Msk        (0x1ul << PWM_ADCTCTL1_CUTRGEN5_Pos)              /*!< PWM_T::ADCTCTL1: CUTRGEN5 Mask            */

#define PWM_ADCTCTL1_CPTRGEN5_Pos        (9)                                               /*!< PWM_T::ADCTCTL1: CPTRGEN5 Position        */
#define PWM_ADCTCTL1_CPTRGEN5_Msk        (0x1ul << PWM_ADCTCTL1_CPTRGEN5_Pos)              /*!< PWM_T::ADCTCTL1: CPTRGEN5 Mask            */

#define PWM_ADCTCTL1_CDTRGEN5_Pos        (10)                                              /*!< PWM_T::ADCTCTL1: CDTRGEN5 Position        */
#define PWM_ADCTCTL1_CDTRGEN5_Msk        (0x1ul << PWM_ADCTCTL1_CDTRGEN5_Pos)              /*!< PWM_T::ADCTCTL1: CDTRGEN5 Mask            */

#define PWM_ADCTCTL1_ZPTRGEN5_Pos        (11)                                              /*!< PWM_T::ADCTCTL1: ZPTRGEN5 Position        */
#define PWM_ADCTCTL1_ZPTRGEN5_Msk        (0x1ul << PWM_ADCTCTL1_ZPTRGEN5_Pos)              /*!< PWM_T::ADCTCTL1: ZPTRGEN5 Mask            */

#define PWM_ADCTCTL1_CUTRGEN6_Pos        (16)                                              /*!< PWM_T::ADCTCTL1: CUTRGEN6 Position        */
#define PWM_ADCTCTL1_CUTRGEN6_Msk        (0x1ul << PWM_ADCTCTL1_CUTRGEN2_Pos)              /*!< PWM_T::ADCTCTL1: CUTRGEN6 Mask            */

#define PWM_ADCTCTL1_CPTRGEN6_Pos        (17)                                              /*!< PWM_T::ADCTCTL1: CPTRGEN6 Position        */
#define PWM_ADCTCTL1_CPTRGEN6_Msk        (0x1ul << PWM_ADCTCTL1_CPTRGEN2_Pos)              /*!< PWM_T::ADCTCTL1: CPTRGEN6 Mask            */

#define PWM_ADCTCTL1_CDTRGEN6_Pos        (18)                                              /*!< PWM_T::ADCTCTL1: CDTRGEN6 Position        */
#define PWM_ADCTCTL1_CDTRGEN6_Msk        (0x1ul << PWM_ADCTCTL1_CDTRGEN2_Pos)              /*!< PWM_T::ADCTCTL1: CDTRGEN6 Mask            */

#define PWM_ADCTCTL1_ZPTRGEN6_Pos        (19)                                              /*!< PWM_T::ADCTCTL1: ZPTRGEN6 Position        */
#define PWM_ADCTCTL1_ZPTRGEN6_Msk        (0x1ul << PWM_ADCTCTL1_ZPTRGEN2_Pos)              /*!< PWM_T::ADCTCTL1: ZPTRGEN6 Mask            */

#define PWM_ADCTCTL1_CUTRGEN7_Pos        (24)                                              /*!< PWM_T::ADCTCTL1: CUTRGEN7 Position        */
#define PWM_ADCTCTL1_CUTRGEN7_Msk        (0x1ul << PWM_ADCTCTL1_CUTRGEN7_Pos)              /*!< PWM_T::ADCTCTL1: CUTRGEN7 Mask            */

#define PWM_ADCTCTL1_CPTRGEN7_Pos        (25)                                              /*!< PWM_T::ADCTCTL1: CPTRGEN7 Position        */
#define PWM_ADCTCTL1_CPTRGEN7_Msk        (0x1ul << PWM_ADCTCTL1_CPTRGEN7_Pos)              /*!< PWM_T::ADCTCTL1: CPTRGEN7 Mask            */

#define PWM_ADCTCTL1_CDTRGEN7_Pos        (26)                                              /*!< PWM_T::ADCTCTL1: CDTRGEN7 Position        */
#define PWM_ADCTCTL1_CDTRGEN7_Msk        (0x1ul << PWM_ADCTCTL1_CDTRGEN7_Pos)              /*!< PWM_T::ADCTCTL1: CDTRGEN7 Mask            */

#define PWM_ADCTCTL1_ZPTRGEN7_Pos        (27)                                              /*!< PWM_T::ADCTCTL1: ZPTRGEN7 Position        */
#define PWM_ADCTCTL1_ZPTRGEN7_Msk        (0x1ul << PWM_ADCTCTL1_ZPTRGEN7_Pos)              /*!< PWM_T::ADCTCTL1: ZPTRGEN7 Mask            */

#define PWM_ADCTSTS0_CUTRGF0_Pos         (0)                                               /*!< PWM_T::ADCTSTS0: CUTRGF0 Position         */
#define PWM_ADCTSTS0_CUTRGF0_Msk         (0x1ul << PWM_ADCTSTS0_CUTRGF0_Pos)               /*!< PWM_T::ADCTSTS0: CUTRGF0 Mask             */

#define PWM_ADCTSTS0_CPTRGF0_Pos         (1)                                               /*!< PWM_T::ADCTSTS0: CPTRGF0 Position         */
#define PWM_ADCTSTS0_CPTRGF0_Msk         (0x1ul << PWM_ADCTSTS0_CPTRGF0_Pos)               /*!< PWM_T::ADCTSTS0: CPTRGF0 Mask             */

#define PWM_ADCTSTS0_CDTRGF0_Pos         (2)                                               /*!< PWM_T::ADCTSTS0: CDTRGF0 Position         */
#define PWM_ADCTSTS0_CDTRGF0_Msk         (0x1ul << PWM_ADCTSTS0_CDTRGF0_Pos)               /*!< PWM_T::ADCTSTS0: CDTRGF0 Mask             */

#define PWM_ADCTSTS0_ZPTRGF0_Pos         (3)                                               /*!< PWM_T::ADCTSTS0: ZPTRGF0 Position         */
#define PWM_ADCTSTS0_ZPTRGF0_Msk         (0x1ul << PWM_ADCTSTS0_ZPTRGF0_Pos)               /*!< PWM_T::ADCTSTS0: ZPTRGF0 Mask             */

#define PWM_ADCTSTS0_CUTRGF1_Pos         (8)                                               /*!< PWM_T::ADCTSTS0: CUTRGF1 Position         */
#define PWM_ADCTSTS0_CUTRGF1_Msk         (0x1ul << PWM_ADCTSTS0_CUTRGF1_Pos)               /*!< PWM_T::ADCTSTS0: CUTRGF1 Mask             */

#define PWM_ADCTSTS0_CPTRGF1_Pos         (9)                                               /*!< PWM_T::ADCTSTS0: CPTRGF1 Position         */
#define PWM_ADCTSTS0_CPTRGF1_Msk         (0x1ul << PWM_ADCTSTS0_CPTRGF1_Pos)               /*!< PWM_T::ADCTSTS0: CPTRGF1 Mask             */

#define PWM_ADCTSTS0_CDTRGF1_Pos         (10)                                              /*!< PWM_T::ADCTSTS0: CDTRGF1 Position         */
#define PWM_ADCTSTS0_CDTRGF1_Msk         (0x1ul << PWM_ADCTSTS0_CDTRGF1_Pos)               /*!< PWM_T::ADCTSTS0: CDTRGF1 Mask             */

#define PWM_ADCTSTS0_ZPTRGF1_Pos         (11)                                              /*!< PWM_T::ADCTSTS0: ZPTRGF1 Position         */
#define PWM_ADCTSTS0_ZPTRGF1_Msk         (0x1ul << PWM_ADCTSTS0_ZPTRGF1_Pos)               /*!< PWM_T::ADCTSTS0: ZPTRGF1 Mask             */

#define PWM_ADCTSTS0_CUTRGF2_Pos         (16)                                              /*!< PWM_T::ADCTSTS0: CUTRGF2 Position         */
#define PWM_ADCTSTS0_CUTRGF2_Msk         (0x1ul << PWM_ADCTSTS0_CUTRGF2_Pos)               /*!< PWM_T::ADCTSTS0: CUTRGF2 Mask             */

#define PWM_ADCTSTS0_CPTRGF2_Pos         (17)                                              /*!< PWM_T::ADCTSTS0: CPTRGF2 Position         */
#define PWM_ADCTSTS0_CPTRGF2_Msk         (0x1ul << PWM_ADCTSTS0_CPTRGF2_Pos)               /*!< PWM_T::ADCTSTS0: CPTRGF2 Mask             */

#define PWM_ADCTSTS0_CDTRGF2_Pos         (18)                                              /*!< PWM_T::ADCTSTS0: CDTRGF2 Position         */
#define PWM_ADCTSTS0_CDTRGF2_Msk         (0x1ul << PWM_ADCTSTS0_CDTRGF2_Pos)               /*!< PWM_T::ADCTSTS0: CDTRGF2 Mask             */

#define PWM_ADCTSTS0_ZPTRGF2_Pos         (19)                                              /*!< PWM_T::ADCTSTS0: ZPTRGF2 Position         */
#define PWM_ADCTSTS0_ZPTRGF2_Msk         (0x1ul << PWM_ADCTSTS0_ZPTRGF2_Pos)               /*!< PWM_T::ADCTSTS0: ZPTRGF2 Mask             */

#define PWM_ADCTSTS0_CUTRGF3_Pos         (24)                                              /*!< PWM_T::ADCTSTS0: CUTRGF3 Position         */
#define PWM_ADCTSTS0_CUTRGF3_Msk         (0x1ul << PWM_ADCTSTS0_CUTRGF3_Pos)               /*!< PWM_T::ADCTSTS0: CUTRGF3 Mask             */

#define PWM_ADCTSTS0_CPTRGF3_Pos         (25)                                              /*!< PWM_T::ADCTSTS0: CPTRGF3 Position         */
#define PWM_ADCTSTS0_CPTRGF3_Msk         (0x1ul << PWM_ADCTSTS0_CPTRGF3_Pos)               /*!< PWM_T::ADCTSTS0: CPTRGF3 Mask             */

#define PWM_ADCTSTS0_CDTRGF3_Pos         (26)                                              /*!< PWM_T::ADCTSTS0: CDTRGF3 Position         */
#define PWM_ADCTSTS0_CDTRGF3_Msk         (0x1ul << PWM_ADCTSTS0_CDTRGF3_Pos)               /*!< PWM_T::ADCTSTS0: CDTRGF3 Mask             */

#define PWM_ADCTSTS0_ZPTRGF3_Pos         (27)                                              /*!< PWM_T::ADCTSTS0: ZPTRGF3 Position         */
#define PWM_ADCTSTS0_ZPTRGF3_Msk         (0x1ul << PWM_ADCTSTS0_ZPTRGF3_Pos)               /*!< PWM_T::ADCTSTS0: ZPTRGF3 Mask             */

#define PWM_ADCTSTS1_CUTRGF4_Pos         (0)                                               /*!< PWM_T::ADCTSTS1: CUTRGF4 Position         */
#define PWM_ADCTSTS1_CUTRGF4_Msk         (0x1ul << PWM_ADCTSTS1_CUTRGF4_Pos)               /*!< PWM_T::ADCTSTS1: CUTRGF4 Mask             */

#define PWM_ADCTSTS1_CPTRGF4_Pos         (1)                                               /*!< PWM_T::ADCTSTS1: CPTRGF4 Position         */
#define PWM_ADCTSTS1_CPTRGF4_Msk         (0x1ul << PWM_ADCTSTS1_CPTRGF4_Pos)               /*!< PWM_T::ADCTSTS1: CPTRGF4 Mask             */

#define PWM_ADCTSTS1_CDTRGF4_Pos         (2)                                               /*!< PWM_T::ADCTSTS1: CDTRGF4 Position         */
#define PWM_ADCTSTS1_CDTRGF4_Msk         (0x1ul << PWM_ADCTSTS1_CDTRGF4_Pos)               /*!< PWM_T::ADCTSTS1: CDTRGF4 Mask             */

#define PWM_ADCTSTS1_ZPTRGF4_Pos         (3)                                               /*!< PWM_T::ADCTSTS1: ZPTRGF4 Position         */
#define PWM_ADCTSTS1_ZPTRGF4_Msk         (0x1ul << PWM_ADCTSTS1_ZPTRGF4_Pos)               /*!< PWM_T::ADCTSTS1: ZPTRGF4 Mask             */

#define PWM_ADCTSTS1_CUTRGF5_Pos         (8)                                               /*!< PWM_T::ADCTSTS1: CUTRGF5 Position         */
#define PWM_ADCTSTS1_CUTRGF5_Msk         (0x1ul << PWM_ADCTSTS1_CUTRGF5_Pos)               /*!< PWM_T::ADCTSTS1: CUTRGF5 Mask             */

#define PWM_ADCTSTS1_CPTRGF5_Pos         (9)                                               /*!< PWM_T::ADCTSTS1: CPTRGF5 Position         */
#define PWM_ADCTSTS1_CPTRGF5_Msk         (0x1ul << PWM_ADCTSTS1_CPTRGF5_Pos)               /*!< PWM_T::ADCTSTS1: CPTRGF5 Mask             */

#define PWM_ADCTSTS1_CDTRGF5_Pos         (10)                                              /*!< PWM_T::ADCTSTS1: CDTRGF5 Position         */
#define PWM_ADCTSTS1_CDTRGF5_Msk         (0x1ul << PWM_ADCTSTS1_CDTRGF5_Pos)               /*!< PWM_T::ADCTSTS1: CDTRGF5 Mask             */

#define PWM_ADCTSTS1_ZPTRGF5_Pos         (11)                                              /*!< PWM_T::ADCTSTS1: ZPTRGF5 Position         */
#define PWM_ADCTSTS1_ZPTRGF5_Msk         (0x1ul << PWM_ADCTSTS1_ZPTRGF5_Pos)               /*!< PWM_T::ADCTSTS1: ZPTRGF5 Mask             */


#define PWM_ADCTSTS1_CUTRGF6_Pos         (16)                                              /*!< PWM_T::ADCTSTS0: CUTRGF6 Position         */
#define PWM_ADCTSTS1_CUTRGF6_Msk         (0x1ul << PWM_ADCTSTS7_CUTRGF6_Pos)               /*!< PWM_T::ADCTSTS0: CUTRGF6 Mask             */

#define PWM_ADCTSTS1_CPTRGF6_Pos         (17)                                              /*!< PWM_T::ADCTSTS0: CPTRGF6 Position         */
#define PWM_ADCTSTS1_CPTRGF6_Msk         (0x1ul << PWM_ADCTSTS7_CPTRGF6_Pos)               /*!< PWM_T::ADCTSTS0: CPTRGF6 Mask             */

#define PWM_ADCTSTS1_CDTRGF6_Pos         (18)                                              /*!< PWM_T::ADCTSTS0: CDTRGF6 Position         */
#define PWM_ADCTSTS1_CDTRGF6_Msk         (0x1ul << PWM_ADCTSTS7_CDTRGF6_Pos)               /*!< PWM_T::ADCTSTS0: CDTRGF6 Mask             */

#define PWM_ADCTSTS1_ZPTRGF6_Pos         (19)                                              /*!< PWM_T::ADCTSTS0: ZPTRGF6 Position         */
#define PWM_ADCTSTS1_ZPTRGF6_Msk         (0x1ul << PWM_ADCTSTS7_ZPTRGF6_Pos)               /*!< PWM_T::ADCTSTS0: ZPTRGF6 Mask             */

#define PWM_ADCTSTS1_CUTRGF7_Pos         (24)                                              /*!< PWM_T::ADCTSTS0: CUTRGF7 Position         */
#define PWM_ADCTSTS1_CUTRGF7_Msk         (0x1ul << PWM_ADCTSTS7_CUTRGF7_Pos)               /*!< PWM_T::ADCTSTS0: CUTRGF7 Mask             */

#define PWM_ADCTSTS1_CPTRGF7_Pos         (25)                                              /*!< PWM_T::ADCTSTS0: CPTRGF7 Position         */
#define PWM_ADCTSTS1_CPTRGF7_Msk         (0x1ul << PWM_ADCTSTS7_CPTRGF7_Pos)               /*!< PWM_T::ADCTSTS0: CPTRGF7 Mask             */

#define PWM_ADCTSTS1_CDTRGF7_Pos         (26)                                              /*!< PWM_T::ADCTSTS0: CDTRGF7 Position         */
#define PWM_ADCTSTS1_CDTRGF7_Msk         (0x1ul << PWM_ADCTSTS7_CDTRGF7_Pos)               /*!< PWM_T::ADCTSTS0: CDTRGF7 Mask             */

#define PWM_ADCTSTS1_ZPTRGF7_Pos         (27)                                              /*!< PWM_T::ADCTSTS0: ZPTRGF7 Position         */
#define PWM_ADCTSTS1_ZPTRGF7_Msk         (0x1ul << PWM_ADCTSTS7_ZPTRGF7_Pos)               /*!< PWM_T::ADCTSTS0: ZPTRGF7 Mask             */

#define PWM_PCACTL_PCAEN_Pos             (0)                                               /*!< PWM_T::PCACTL: PCAEN Position             */
#define PWM_PCACTL_PCAEN_Msk             (0x1ul << PWM_PCACTL_PCAEN_Pos) 

/**@}*/ /* PWM_CONST */
/**@}*/ /* end of PWM register group */


/*---------------------- Serial Peripheral Interface Controller -------------------------*/
/**
    @addtogroup SPI Serial Peripheral Interface Controller(SPI)
    Memory Mapped Structure for SPI Controller
@{ */

typedef struct {


    __IO uint32_t CTRL0; 			//0x00
    __IO uint32_t CTRL1;			//0x04

    __IO uint32_t SPIER;			//0x08

    __I  uint32_t RESERVED0[1]; 	// 0x0C

    __IO  uint32_t SER; 			//0x10
    __IO  uint32_t BAUDR;			//0x15

    __IO  uint32_t TXFTLR;		//0x18
    __IO  uint32_t RXFTLR;		//0x1C

	//read-only
	__I  uint32_t TXFLR;		//0x20
    __I  uint32_t RXFLR;		//0x24

	__I  uint32_t SR;			//0x28

    __IO  uint32_t IMR;			//0x2C

	__I  uint32_t ISR;			//0x30
	__I  uint32_t RISR;			//0x34
	__I  uint32_t TXOICR;		//0x38
	__I  uint32_t RXOICR;		//0x3C
	__I  uint32_t RXUICR;		//0x40
	__I  uint32_t MSTICR;		//0x44
	__I  uint32_t ICR;			//0x48

	__IO uint32_t DMACR;		//0x4C
	__IO uint32_t DMATDLR;		//0x50
	__IO uint32_t DMARDLR;		//0x54

    __I  uint32_t RESERVED1[2]; //0x58

    __IO uint32_t DR;			//0x60

} SPI_T;

/**
    @addtogroup SPI_CONST SPI Bit Field Definition
    Constant Definitions for SPI Controller
@{ */
//CTRL0
#define SPI_CTL0_SCPHA_Pos                 (6)
#define SPI_CTL0_SCPHA_Msk                 (0x1ul << SPI_CTL0_SCPHA_Pos)

#define SPI_CTL0_SCPOL_Pos                (7)
#define SPI_CTL0_SCPOL_Msk                (0x1ul << SPI_CTL0_SCPOL_Pos)

#define SPI_CTL0_TMOD_Pos                 (8)
#define SPI_CTL0_TMOD_Msk                 (0x3ul << SPI_CTL0_TMOD_Pos)

#define SPI_CTL0_SLV_OE_Pos               (10)
#define SPI_CTL0_SLV_OE_Msk               (0x3ul << SPI_CTL0_SLV_OE_Pos)

#define SPI_CTL0_DFS32_Pos                (16)
#define SPI_CTL0_DFS32_Msk                (0x1Ful << SPI_CTL0_DFS32_Pos)
//CTRL1
#define SPI_CTL1_NDF_Pos                  (0)
#define SPI_CTL1_NDF_Msk                  (0xFFFFul << SPI_CTL1_NDF_Pos)
//SPI_EN
#define SPI_SPIEN_EN_Pos              	  (0)
#define SPI_SPIEN_EN_Msk                  (0x1ul << SPI_SPIEN_EN_Pos)

//SER
#define SPI_SER_EN_Pos              	  (0)
#define SPI_SER_EN_Msk                    (0x1ul << SPI_SER_EN_Pos)

//BAUDR
#define SPI_BAUDR_SCKDV_Pos               (0)
#define SPI_BAUDR_SCKDV_Msk               (0xFFFFul << SPI_BAUDR_SCKDV_Pos)
//TXFTLR
#define SPI_TXFTLR_TFT_Pos                (0)
#define SPI_TXFTLR_TFT_Msk                (0xFul << SPI_TXFTLR_TFT_Pos)
//RXFTLR
#define SPI_RXFTLR_RFT_Pos                (0)
#define SPI_RXFTLR_RFT_Msk                (0xFul << SPI_RXFTLR_RFT_Pos)

//TXFLR
#define SPI_TXFLR_TFL_Pos			      (0)
#define SPI_TXFLR_TFL_Msk                 (0xFul << SPI_TXFLR_TFL_Pos)
//RXFLR
#define SPI_RXFLR_RFL_Pos			      (0)
#define SPI_RXFLR_RFL_Msk                 (0xFul << SPI_RXFLR_RFL_Pos)

//SR
#define SPI_SR_BUSY_Pos			      	  (0)
#define SPI_SR_BUSY_Msk                   (0x1ul << SPI_SR_BUSY_Pos)

#define SPI_SR_TFNF_Pos			      	  (1)
#define SPI_SR_TFNF_Msk                   (0x1ul << SPI_SR_TFNF_Pos)

#define SPI_SR_TFE_Pos			      	  (2)
#define SPI_SR_TFE_Msk                    (0x1ul << SPI_SR_TFE_Pos)

#define SPI_SR_RFNE_Pos			      	  (3)
#define SPI_SR_RFNE_Msk                   (0x1ul << SPI_SR_RFNE_Pos)

#define SPI_SR_RFF_Pos			      	  (4)
#define SPI_SR_RFF_Msk                    (0x1ul << SPI_SR_RFF_Pos)

#define SPI_SR_DCOL_Pos			      	  (5)
#define SPI_SR_DCOL_Msk                   (0x1ul << SPI_SR_DCOL_Pos)

//IMR
#define SPI_IMR_TXE_Pos			      	  (0)
#define SPI_IMR_TXE_Msk                   (0x1ul << SPI_IMR_TXE_Pos)

#define SPI_IMR_TXO_Pos			      	  (1)
#define SPI_IMR_TXO_Msk                   (0x1ul << SPI_IMR_TXO_Pos)

#define SPI_IMR_RXU_Pos			      	  (2)
#define SPI_IMR_RXU_Msk                   (0x1ul << SPI_IMR_RXU_Pos)

#define SPI_IMR_RXO_Pos			      	  (3)
#define SPI_IMR_RXO_Msk                   (0x1ul << SPI_IMR_RXO_Pos)

#define SPI_IMR_RXF_Pos			      	  (4)
#define SPI_IMR_RXF_Msk                   (0x1ul << SPI_IMR_RXF_Pos)

#define SPI_IMR_MST_Pos			      	  (5)
#define SPI_IMR_MST_Msk                   (0x1ul << SPI_IMR_MST_Pos)

#define SPI_IMR_ALL_INT_Pos               (0)
#define SPI_IMR_ALL_INT_Msk               (0x3Ful << SPI_IMR_ALL_INT_Pos)

//ISR
#define SPI_ISR_TXE_Pos			      	  (0)
#define SPI_ISR_TXE_Msk                   (0x1ul << SPI_ISR_TXE_Pos)

#define SPI_ISR_TXO_Pos			      	  (1)
#define SPI_ISR_TXO_Msk                   (0x1ul << SPI_ISR_TXO_Pos)

#define SPI_ISR_RXU_Pos			      	  (2)
#define SPI_ISR_RXU_Msk                   (0x1ul << SPI_ISR_RXU_Pos)

#define SPI_ISR_RXO_Pos			      	  (3)
#define SPI_ISR_RXO_Msk                   (0x1ul << SPI_ISR_RXO_Pos)

#define SPI_ISR_RXF_Pos			      	  (4)
#define SPI_ISR_RXF_Msk                   (0x1ul << SPI_ISR_RXF_Pos)

#define SPI_ISR_MST_Pos			      	  (5)
#define SPI_ISR_MST_Msk                   (0x1ul << SPI_ISR_MST_Pos)

#define SPI_ISR_ALL_INT_Pos               (0)
#define SPI_ISR_ALL_INT_Msk               (0x3Ful << SPI_ISR_ALL_INT_Pos)

//RISR
#define SPI_RISR_TXE_Pos			      (0)
#define SPI_RISR_TXE_Msk                  (0x1ul << SPI_RISR_TXE_Pos)

#define SPI_RISR_TXO_Pos			      (1)
#define SPI_RISR_TXO_Msk                  (0x1ul << SPI_RISR_TXO_Pos)

#define SPI_RISR_RXU_Pos			      (2)
#define SPI_RISR_RXU_Msk                  (0x1ul << SPI_RISR_RXU_Pos)

#define SPI_RISR_RXO_Pos		     	  (3)
#define SPI_RISR_RXO_Msk                  (0x1ul << SPI_RISR_RXO_Pos)

#define SPI_RISR_RXF_Pos     	      	  (4)
#define SPI_RISR_RXF_Msk                  (0x1ul << SPI_RISR_RXF_Pos)

#define SPI_RISR_MST_Pos	         	  (5)
#define SPI_RISR_MST_Msk                  (0x1ul << SPI_RISR_MST_Pos)

#define SPI_RISR_ALL_INT_Pos               (0)
#define SPI_RISR_ALL_INT_Msk               (0x3Ful << SPI_RISR_ALL_INT_Pos)

//DMACR
#define SPI_DMACR_RDMAE_Pos			      (0)
#define SPI_DMACR_RDMAE_Msk               (0x1ul << SPI_DMACR_RDMAE_Pos)

#define SPI_DMACR_TDMAE_Pos			      (1)
#define SPI_DMACR_TDMAE_Msk               (0x1ul << SPI_DMACR_TDMAE_Pos)

//DMATDLR
#define SPI_DMATDLR_DMATDL_Pos			  (0)
#define SPI_DMATDLR_DMATDL_Msk            (0xFul << SPI_DMATDLR_DMATDL_Pos)
//DMARDLR
#define SPI_DMARDLR_DMARDL_Pos			  (0)
#define SPI_DMARDLR_DMARDL_Msk            (0xFul << SPI_DMARDLR_DMARDL_Pos)

//DR
#define SPI_DMADR_DATA_Pos			      (0)
#define SPI_DMADR_DATA_Msk            (0xFFFFFFFFul << SPI_DMADR_DATA_Pos)

/**@}*/ /* SPI_CONST */
/**@}*/ /* end of SPI register group */


/*---------------------- System Manger Controller -------------------------*/
/**
    @addtogroup SYS System Manger Controller(SYS)
    Memory Mapped Structure for SYS Controller
@{ */

typedef struct {

    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVED0[1];
    /// @endcond //HIDDEN_SYMBOLS




    /**
     * RSTSTS
     * ===================================================================================================
     * Offset: 0x04  System Reset Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PORF      |POR Reset Flag
     * |        |          |The POR reset flag is set by the "Reset Signal" from the Power-on Reset (POR) Controller or bit CHIPRST (SYS_IPRST0[0]) to indicate the previous reset source.
     * |        |          |0 = No reset from POR or CHIPRST.
     * |        |          |1 = Power-on-Reset (POR) or CHIPRST had issued the reset signal to reset the system.
     * |        |          |Note: Software can write 1 to clear this bit to zero.
     * |[1]     |PINRF     |NRESET Pin Reset Flag
     * |        |          |The nRESET pin reset flag is set by the "Reset Signal" from the nRESET pin or Low-Voltage-Reset (LVR) to indicate the previous reset source.
     * |        |          |0 = No reset from nRESET pin or Low-Voltage-Reset (LVR).
     * |        |          |1 = Pin nRESET or LVR had issued the reset signal to reset the system.
     * |        |          |Note: Software can write 1 to clear this bit to zero.
     * |[2]     |WDTRF     |WDT Reset Flag
     * |        |          |The WDT reset flag is set by the "Reset Signal" from the Watchdog Timer or Window Watchdog Timer to indicate the previous reset source.
     * |        |          |0 = No reset from watchdog timer or window watchdog timer.
     * |        |          |1 = The watchdog timer or window watchdog timer had issued the reset signal to reset the system.
     * |        |          |Note: Software can write 1 to clear this bit to zero.
     * |[4]     |BODRF     |BOD Reset Flag
     * |        |          |The BOD reset flag is set by the "Reset Signal" from the Brown-out Detector to indicate the previous reset source.
     * |        |          |0 = No reset from BOD.
     * |        |          |1 = The BOD had issued the reset signal to reset the system.
     * |        |          |Note: Software can write 1 to clear this bit to zero.
     * |[5]     |SYSRF     |System Reset Flag
     * |        |          |The system reset flag is set by the "Reset Signal" from the Cortex-M0 Core to indicate the previous reset source.
     * |        |          |0 = No reset from Cortex-M0.
     * |        |          |1 = The Cortex-M0 had issued the reset signal to reset the system by writing 1 to the bit SYSRESETREQ (SCS_AIRCR[2]), Application Interrupt and Reset Control Register, address = 0xE000ED0C in system control registers of Cortex-M0 core.
     * |        |          |Note: Software can write 1 to clear this bit to zero.
     * |[7]     |CPURF     |CPU Reset Flag
     * |        |          |The CPU reset flag is set by hardware if software writes CPURST (SYS_IPRST0[1]) 1 to reset Cortex-M0 Core and Flash Memory Controller (FMC).
     * |        |          |0 = No reset from CPU.
     * |        |          |1 = The Cortex-M0 Core and FMC are reset by software setting CPURST to 1.
     * |        |          |Note: Software can write 1 to clear this bit to zero.
    */
    __IO uint32_t RSTSTS;

    /**
     * IPRST0
     * ===================================================================================================
     * Offset: 0x08  Peripheral Reset Control Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CHIPRST   |CHIP One-shot Reset (Write Protect)
     * |        |          |Setting this bit will reset the whole chip, including Processor core and all peripherals, and this bit will automatically return to 0 .
     * |        |          |The CHIPRST is the same as the POR reset, all the chip controllers is reset and the chip settings from flash are also reload.
     * |        |          |0 = Chip normal operation.
     * |        |          |1 = CHIP one-shot reset.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[1]     |CPURST    |Processor Core One-shot Reset (Write Protect)
     * |        |          |Setting this bit will only reset the processor core and Flash Memory Controller (FMC), and this bit will automatically return to 0 .
     * |        |          |0 = Processor core normal operation.
     * |        |          |1 = Processor core one-shot reset.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
    */
    __IO uint32_t IPRST0;

    /**
     * IPRST1
     * ===================================================================================================
     * Offset: 0x0C  Peripheral Reset Control Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |GPIORST   |GPIO (P0~P5) Controller Reset
     * |        |          |0 = GPIO controller normal operation.
     * |        |          |1 = GPIO controller reset.
     * |[2]     |TMR0RST   |Timer0 Controller Reset
     * |        |          |0 = Timer0 controller normal operation.
     * |        |          |1 = Timer0 controller reset.
     * |[3]     |TMR1RST   |Timer1 Controller Reset
     * |        |          |0 = Timer1 controller normal operation.
     * |        |          |1 = Timer1 controller reset.
     * |[4]     |TMR2RST   |Timer2 Controller Reset
     * |        |          |0 = Timer2 controller normal operation.
     * |        |          |1 = Timer2 controller reset.
     * |[8]     |I2C0RST   |I2C0 Controller Reset
     * |        |          |0 = I2C0 controller normal operation.
     * |        |          |1 = I2C0 controller reset.
     * |[9]     |I2C1RST   |I2C1 Controller Reset
     * |        |          |0 = I2C1 controller normal operation.
     * |        |          |1 = I2C1 controller reset.
     * |[12]    |SPI0RST   |SPI0 Controller Reset
     * |        |          |0 = SPI0 controller normal operation.
     * |        |          |1 = SPI0 controller reset.
     * |[13]    |SPI1RST   |SPI1 Controller Reset
     * |        |          |0 = SPI1 controller normal operation.
     * |        |          |1 = SPI1 controller reset.
     * |[16]    |UART0RST  |UART0 Controller Reset
     * |        |          |0 = UART0 controller normal operation.
     * |        |          |1 = UART0 controller reset.
     * |[17]    |UART1RST  |UART1 Controller Reset
     * |        |          |0 = UART1 controller normal operation.
     * |        |          |1 = UART1 controller reset.
     * |[18]    |WDTRST  |WDT Controller Reset
     * |        |          |0 = WDT controller normal operation.
     * |        |          |1 = WDT controller reset.
     * |[19]    |WWDTRST  |WWDT Controller Reset
     * |        |          |0 = WWDT controller normal operation.
     * |        |          |1 = WWDT controller reset.  
     * |[20]    |PWM0RST   |PWM0 Controller Reset
     * |        |          |0 = PWM0 controller normal operation.
     * |        |          |1 = PWM0 controller reset.
     * |[22]    |ACMPRST   |ACMP Controller Reset
     * |        |          |0 = ACMP controller normal operation.
     * |        |          |1 = ACMP controller reset.
     * |[28]    |ADCRST    |ADC Controller Reset
     * |        |          |0 = ADC controller normal operation.
     * |        |          |1 = ADC controller reset.
    */
    __IO uint32_t IPRST1;
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVED1[2];
    /// @endcond //HIDDEN_SYMBOLS


    /**
     * BODCTL
     * ===================================================================================================
     * Offset: 0x18  Brown-out Detector Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |BODEN     |Brown-out Detector Selection Extension (Initiated & Write-protected Bit)
     * |        |          |The default value is set by flash controller user configuration register config0 bit[23].
     * |        |          |If config0 bit[23] is set to 1, default value of BODEN is 0.
     * |        |          |If config0 bit[23] is set to 0, default value of BODEN is 1.
     * |        |          |0 = Brown-out detector threshold voltage is selected by the table defined in BODVL.
     * |        |          |1 = Brown-out detector threshold voltage is selected by the table defined as below.
     * |        |          |BODVL = 00 = Brown-out Detector threshold voltage is 2.15V.
     * |        |          |BODVL = 01 = Brown-out Detector threshold voltage is 2.40V.
     * |        |          |BODVL = 10 = Brown-out Detector threshold voltage is 2.67V.
     * |        |          |BODVL = 11 = Brown-out Detector threshold voltage is 3.03V.
     * |[2:1]   |BODVL     |Brown-out Detector Threshold Voltage Selection (Write Protect)
     * |        |          |The default value is set by flash controller user configuration register CBOV (CONFIG0[22:21]).
     * |        |          |00 =2.15V.
     * |        |          |01 = Brown-out Detector threshold voltage is 2.40V.
     * |        |          |10 = Brown-out Detector threshold voltage is 2.67V.
     * |        |          |11 = Brown-out Detector function Disabled.
     * |[3]     |BODRSTEN  |Brown-out Reset Enable Bit (Write Protect)
     * |        |          |The default value is set by flash controller user configuration register CBORST(CONFIG0[20]) bit.
     * |        |          |If config0 bit[20] is set to 1, default value of BODRSTEN is 0.
     * |        |          |If config0 bit[20] is set to 0, default value of BODRSTEN is 1.
     * |        |          |0 = Brown-out "INTERRUPT" function Enabled; when the Brown-out Detector function is enable and the detected voltage is lower than the threshold, then assert a signal to interrupt the Cortex-M0 CPU.
     * |        |          |1 = Brown-out "RESET" function Enabled; when the Brown-out Detector function is enable and the detected voltage is lower than the threshold then assert a signal to reset the chip.
     * |        |          |Note: When the BOD_EN is enabled and the interrupt is asserted, the interrupt will be kept till the BOD_EN is set to 0.
     * |        |          |The interrupt for CPU can be blocked by disabling the NVIC in CPU for BOD interrupt or disable the interrupt source by disabling the BOD_EN and then re-enabling the BOD_EN function if the BOD function is require.
     * |[4]     |BODIF     |Brown-out Detector Interrupt Flag
     * |        |          |0 = Brown-out Detector does not detect any voltage draft at VDD down through or up through the voltage of BODVL setting.
     * |        |          |1 = When Brown-out Detector detects the VDD is dropped through the voltage of BODVL setting or the VDD is raised up through the voltage of BODVL setting, this bit is set to 1 and the Brown-out interrupt is requested if Brown-out interrupt is enabled.
     * |[6]     |BODOUT    |Brown-out Detector Output Status
     * |        |          |1 = Brown-out Detector status output is 0, the detected voltage is higher than BODVL setting.
     * |        |          |0 = Brown-out Detector status output is 1, the detected voltage is lower than BODVL setting.
     * |[7]    |PDLVR    |Power Down Low-voltage-Reset (Write Protect
     * |        |          | The default value is set by flash controller user configuration register config0 bit[19].
     * |        |          |0 = Low-voltage-Reset is enable.
     * |        |          |1 = Low-voltage-Reset is disable.
    */
    __IO uint32_t BODCTL;
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVED2[5];
    /// @endcond //HIDDEN_SYMBOLS


    /**
     * P0_MFP
     * ===================================================================================================
     * Offset: 0x30  P0 Multiple Function and Input Type Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |p0_MFP    |
     * |        |          |
     * |        |          |
     * |[15:8]  |p0_ALT    |
     * |        |          |
		 * |[23:16] |p0_EXT    |
    */
    __IO uint32_t P0_MFP;

   
	     /**
     * P1_MFP
     * ===================================================================================================
     * Offset: 0x34  P1 Multiple Function and Input Type Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |p1_MFP    |
     * |        |          |
     * |        |          |
     * |[15:8]  |p1_ALT    |
     * |        |          |
		 * |[23:16] |p1_EXT    |
    */
    __IO uint32_t P1_MFP;

   
	    
	     /**
     * P2_MFP
     * ===================================================================================================
     * Offset: 0x38  P2 Multiple Function and Input Type Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |p2_MFP    |
     * |        |          |
     * |        |          |
     * |[15:8]  |p2_ALT    |
     * |        |          |
		 * |[23:16] |p2_EXT    |
    */
    __IO uint32_t P2_MFP;

   
	      /**
     * P3_MFP
     * ===================================================================================================
     * Offset: 0x3C  P3 Multiple Function and Input Type Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |p3_MFP    |
     * |        |          |
     * |        |          |
     * |[15:8]  |p3_ALT    |
     * |        |          |
		 * |[23:16] |p3_EXT    |
    */
    __IO uint32_t P3_MFP;

  
	      /**
     * P4_MFP
     * ===================================================================================================
     * Offset: 0x40  P4 Multiple Function and Input Type Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |p4_MFP    |
     * |        |          |
     * |        |          |
     * |[15:8]  |p4_ALT    |
     * |        |          |
		 * |[23:16] |p4_EXT    |
    */
    __IO uint32_t P4_MFP;

   
	 	 /**
     * P5_MFP
     * ===================================================================================================
     * Offset: 0x44  P5 Multiple Function and Input Type Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |p5_MFP    |
     * |        |          |
     * |        |          |
     * |[15:8]  |p5_ALT    |
     * |        |          |
		 * |[23:16] |p5_EXT    |
    */
    __IO uint32_t P5_MFP;

    /**
     * TESTCTL
     * ===================================================================================================
     * Offset: 0x48  TEST Use only
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]   	|LDT       |
     * |        |          |
     * |[1]    	|POR	   |
     * |        |          |
     * |[2]    	|BG3V      |
     * |        |          |
     * |[3]    	|10K	   |
     * |        |          |
     * |[4]    	|50M	   |
     * |        |          |
     * |[5]    	|CMPDATA   |
     * |        |          |
     * |[6]    	|CKCMP     |
     * |        |          |
     * |[7]    	|SH		   |
     * |        |          |
     * |[8]    	|ACMP1O    |
     * |        |          |
     * |[9]    	|ACMP0O    |
     * |        |          |
     * |[10]    |BODO      |
	 */
	__IO uint32_t TESTCTL;
	
    /**
     * BLDBCTL
     * ===================================================================================================
     * Offset: 0x4C  TEST Use only
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[5:0]   |BODDBSEL  |BOD De-Bounce (glitch) time Control register 
     * |        |          |[0]=1: about 2^4 PLL clock 
     * |        |          |[1]=1: about 2^7 PLL clock 
     * |        |          |[2]=1: about 2^9 PLL clock 
     * |        |          |[3]=1: about 2^11 PLL clock 
     * |        |          |[4]=1: about 2^13 PLL clock 
     * |        |          |[5]=1: about 2^15 PLL clock (default) 
     * |        |          |Note: If software enables more than one bit, 
     * |        |          |the bit with the smallest number will be selected 
     * |        |          |and the other enabled channels will be ignored.
     * | :----: | :----:   |
     * |[13:8]  |LVRDBSEL  |LVR De-Bounce (glitch) time Control register
     * |        |          |[0]=1: about 2^4 HIRC clock
     * |        |          |[1]=1: about 2^7 HIRC clock
     * |        |          |[2]=1: about 2^9 HIRC clock
     * |        |          |[3]=1: about 2^11 HIRC clock
     * |        |          |[4]=1: about 2^13 HIRC clock
     * |        |          |[5]=1: about 2^15 HIRC clock (default)
     * |        |          |Note: If software enables more than one bit, 
     * |        |          |the bit with the smallest number will be selected 
     * |        |          |and the other enabled channels will be ignored.
	 */
	__IO uint32_t BLDBCTL;
	
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVED3[44];
    /// @endcond //HIDDEN_SYMBOLS


    /**
     * REGLCTL
     * ===================================================================================================
     * Offset: 0x100  Register Write-Protection Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |REGLCTL   |Register Write-protection Code (Write Only)
     * |        |          |Some registers have write-protection function.
     * |        |          |Writing these registers have to disable the protected function by writing the sequence value 0x59, 0x16, 0x88 to this field.
     * |        |          |After this sequence is completed, the REGLCTL bit will be set to 1 and write-protection registers can be normal writ.
     * |        |          |Register Write-protection Disable Index (Read Only)
     * |        |          |0 = Write-protection Enabled for writing protected registers.
     * |        |          |Any write to the protected register is ignore.
     * |        |          |1 = Write-protection Disabled for writing protected registers.
     * |        |          |Protected registers are listed below:
     * |        |          |SYS_IPRST0 (0x5000_0008) : Peripheral Reset Control Resister 0
     * |        |          |SYS_BODCTL (0x5000_0018) : Brown-out Detector Control Register
     * |        |          |CLK_PWRCTL (0x5000_0200) : Bit[6] is not protected for power wake-up interrupt clear.
     * |        |          |CLK_APBCLK bit[0] (0x5000_0208) : Bit[0] is watchdog clock enable.
     * |        |          |CLK_CLKSEL0 (0x5000_0210) : HCLK and CPU STCLK clock source select.
     * |        |          |CLK_CLKSEL1 bit[1:0] (0x5000_0214) : Watchdog clock source select.
     * |        |          |NMI_SEL bit[8] (0x5000_0380) : NMI interrupt enable.
     * |        |          |ISPCON (0x5000_C000) : Flash ISP Control.
     * |        |          |ISPTRG (0x5000_C010) : ISP Trigger Control.
     * |        |          |WTCR (0x4000_4000) : Watchdog Timer Control.
     * |        |          |Note: The bits which are write-protected will be noted as" (Write Protect)" beside the description.
    */
    __O  uint32_t REGLCTL;

	__I  uint32_t STATUS;

} SYS_T;

/**
    @addtogroup SYS_CONST SYS Bit Field Definition
    Constant Definitions for SYS Controller
@{ */


#define SYS_RSTSTS_PORF_Pos              (0)                                               /*!< SYS_T::RSTSTS: PORF Position              */
#define SYS_RSTSTS_PORF_Msk              (0x1ul << SYS_RSTSTS_PORF_Pos)                    /*!< SYS_T::RSTSTS: PORF Mask                  */

#define SYS_RSTSTS_PINRF_Pos             (1)                                               /*!< SYS_T::RSTSTS: PINRF Position             */
#define SYS_RSTSTS_PINRF_Msk             (0x1ul << SYS_RSTSTS_PINRF_Pos)                   /*!< SYS_T::RSTSTS: PINRF Mask                 */

#define SYS_RSTSTS_WDTRF_Pos             (2)                                               /*!< SYS_T::RSTSTS: WDTRF Position             */
#define SYS_RSTSTS_WDTRF_Msk             (0x1ul << SYS_RSTSTS_WDTRF_Pos)                   /*!< SYS_T::RSTSTS: WDTRF Mask                 */

#define SYS_RSTSTS_BODRF_Pos             (4)                                               /*!< SYS_T::RSTSTS: BODRF Position             */
#define SYS_RSTSTS_BODRF_Msk             (0x1ul << SYS_RSTSTS_BODRF_Pos)                   /*!< SYS_T::RSTSTS: BODRF Mask                 */

#define SYS_RSTSTS_SYSRF_Pos             (5)                                               /*!< SYS_T::RSTSTS: SYSRF Position             */
#define SYS_RSTSTS_SYSRF_Msk             (0x1ul << SYS_RSTSTS_SYSRF_Pos)                   /*!< SYS_T::RSTSTS: SYSRF Mask                 */

#define SYS_RSTSTS_CPURF_Pos             (7)                                               /*!< SYS_T::RSTSTS: CPURF Position             */
#define SYS_RSTSTS_CPURF_Msk             (0x1ul << SYS_RSTSTS_CPURF_Pos)                   /*!< SYS_T::RSTSTS: CPURF Mask                 */

#define SYS_IPRST0_CHIPRST_Pos           (0)                                               /*!< SYS_T::IPRST0: CHIPRST Position           */
#define SYS_IPRST0_CHIPRST_Msk           (0x1ul << SYS_IPRST0_CHIPRST_Pos)                 /*!< SYS_T::IPRST0: CHIPRST Mask               */

#define SYS_IPRST0_CPURST_Pos            (1)                                               /*!< SYS_T::IPRST0: CPURST Position            */
#define SYS_IPRST0_CPURST_Msk            (0x1ul << SYS_IPRST0_CPURST_Pos)                  /*!< SYS_T::IPRST0: CPURST Mask                */

#define SYS_IPRST1_GPIORST_Pos           (1)                                               /*!< SYS_T::IPRST1: GPIORST Position           */
#define SYS_IPRST1_GPIORST_Msk           (0x1ul << SYS_IPRST1_GPIORST_Pos)                 /*!< SYS_T::IPRST1: GPIORST Mask               */

#define SYS_IPRST1_TMR0RST_Pos           (2)                                               /*!< SYS_T::IPRST1: TMR0RST Position           */
#define SYS_IPRST1_TMR0RST_Msk           (0x1ul << SYS_IPRST1_TMR0RST_Pos)                 /*!< SYS_T::IPRST1: TMR0RST Mask               */

#define SYS_IPRST1_TMR1RST_Pos           (3)                                               /*!< SYS_T::IPRST1: TMR1RST Position           */
#define SYS_IPRST1_TMR1RST_Msk           (0x1ul << SYS_IPRST1_TMR1RST_Pos)                 /*!< SYS_T::IPRST1: TMR1RST Mask               */

#define SYS_IPRST1_TMR2RST_Pos           (4)                                               /*!< SYS_T::IPRST1: TMR2RST Position           */
#define SYS_IPRST1_TMR2RST_Msk           (0x1ul << SYS_IPRST1_TMR2RST_Pos)                 /*!< SYS_T::IPRST1: TMR2RST Mask               */


#define SYS_IPRST1_I2C0RST_Pos           (8)                                               /*!< SYS_T::IPRST1: I2C0RST Position           */
#define SYS_IPRST1_I2C0RST_Msk           (0x1ul << SYS_IPRST1_I2C0RST_Pos)                 /*!< SYS_T::IPRST1: I2C0RST Mask               */

#define SYS_IPRST1_I2C1RST_Pos           (9)                                               /*!< SYS_T::IPRST1: I2C1RST Position           */
#define SYS_IPRST1_I2C1RST_Msk           (0x1ul << SYS_IPRST1_I2C1RST_Pos)                 /*!< SYS_T::IPRST1: I2C1RST Mask               */

#define SYS_IPRST1_SPI0RST_Pos           (12)                                              /*!< SYS_T::IPRST1: SPI0RST Position           */
#define SYS_IPRST1_SPI0RST_Msk           (0x1ul << SYS_IPRST1_SPI0RST_Pos)                 /*!< SYS_T::IPRST1: SPI0RST Mask               */

#define SYS_IPRST1_SPI1RST_Pos           (13)                                              /*!< SYS_T::IPRST1: SPI1RST Position           */
#define SYS_IPRST1_SPI1RST_Msk           (0x1ul << SYS_IPRST1_SPI1RST_Pos)                 /*!< SYS_T::IPRST1: SPI1RST Mask               */

#define SYS_IPRST1_SPI2RST_Pos           (14)                                              /*!< SYS_T::IPRST1: SPI2RST Position           */
#define SYS_IPRST1_SPI2RST_Msk           (0x1ul << SYS_IPRST1_SPI2RST_Pos)                 /*!< SYS_T::IPRST1: SPI2RST Mask               */

#define SYS_IPRST1_SPI3RST_Pos           (15)                                              /*!< SYS_T::IPRST1: SPI3RST Position           */
#define SYS_IPRST1_SPI3RST_Msk           (0x1ul << SYS_IPRST1_SPI3RST_Pos)                 /*!< SYS_T::IPRST1: SPI3RST Mask               */

#define SYS_IPRST1_UART0RST_Pos          (16)                                              /*!< SYS_T::IPRST1: UART0RST Position          */
#define SYS_IPRST1_UART0RST_Msk          (0x1ul << SYS_IPRST1_UART0RST_Pos)                /*!< SYS_T::IPRST1: UART0RST Mask              */

#define SYS_IPRST1_UART1RST_Pos          (17)                                              /*!< SYS_T::IPRST1: UART1RST Position          */
#define SYS_IPRST1_UART1RST_Msk          (0x1ul << SYS_IPRST1_UART1RST_Pos)                /*!< SYS_T::IPRST1: UART1RST Mask              */

#define SYS_IPRST1_WDTRST_Pos          	(18)                               
#define SYS_IPRST1_WDTRST_Msk          	(0x1ul << SYS_IPRST1_WDTRST_Pos) 

#define SYS_IPRST1_WWDTRST_Pos          	(19)                               
#define SYS_IPRST1_WWDTRST_Msk          	(0x1ul << SYS_IPRST1_WWDTRST_Pos) 

#define SYS_IPRST1_PWM0RST_Pos           (20)                                              /*!< SYS_T::IPRST1: PWM0RST Position           */
#define SYS_IPRST1_PWM0RST_Msk           (0x1ul << SYS_IPRST1_PWM0RST_Pos)                 /*!< SYS_T::IPRST1: PWM0RST Mask               */

#define SYS_IPRST1_ACMPRST_Pos           (22)                                              /*!< SYS_T::IPRST1: ACMPRST Position           */
#define SYS_IPRST1_ACMPRST_Msk           (0x1ul << SYS_IPRST1_ACMPRST_Pos)                 /*!< SYS_T::IPRST1: ACMPRST Mask               */

#define SYS_IPRST1_ADCRST_Pos            (28)                                              /*!< SYS_T::IPRST1: ADCRST Position            */
#define SYS_IPRST1_ADCRST_Msk            (0x1ul << SYS_IPRST1_ADCRST_Pos)                  /*!< SYS_T::IPRST1: ADCRST Mask                */



#define SYS_BODCTL_BODRSTEN_Pos          (3)                                               /*!< SYS_T::BODCTL: BODRSTEN Position          */
#define SYS_BODCTL_BODRSTEN_Msk          (0x1ul << SYS_BODCTL_BODRSTEN_Pos)                /*!< SYS_T::BODCTL: BODRSTEN Mask              */

#define SYS_BODCTL_BODIF_Pos             (4)                                               /*!< SYS_T::BODCTL: BODIF Position             */
#define SYS_BODCTL_BODIF_Msk             (0x1ul << SYS_BODCTL_BODIF_Pos)                   /*!< SYS_T::BODCTL: BODIF Mask                 */

#define SYS_BODCTL_BODOUT_Pos            (6)                                               /*!< SYS_T::BODCTL: BODOUT Position            */
#define SYS_BODCTL_BODOUT_Msk            (0x1ul << SYS_BODCTL_BODOUT_Pos)                  /*!< SYS_T::BODCTL: BODOUT Mask                */

#define SYS_BODCTL_PDLVR_Pos            (5)                                               /*!< SYS_T::BODCTL: PDLVR Position            */
#define SYS_BODCTL_PDLVR_Msk            (0x1ul << SYS_BODCTL_PDLVR_Pos)                  /*!< SYS_T::BODCTL: PDLVR Mask                */
#define ANAC_RCC_CTL_BODEN_Pos 			(24)
#define ANAC_RCC_CTL_BODEN_Msk 			(0x1ul << ANAC_RCC_CTL_BODEN_Pos)
#define ANAC_RCC_CTL_BODVL_Pos 			(25)
#define ANAC_RCC_CTL_BODVL_Msk 			(0x3ul << ANAC_RCC_CTL_BODVL_Pos)

#define SYS_P0_MFP_MFP_Pos               (0)                                               /*!< SYS_T::P0_MFP: MFP Position               */
#define SYS_P0_MFP_MFP_Msk               (0xfful << SYS_P0_MFP_MFP_Pos)                    /*!< SYS_T::P0_MFP: MFP Mask                   */

#define SYS_P0_MFP_ALT0_Pos              (8)                                               /*!< SYS_T::P0_MFP: ALT0 Position              */
#define SYS_P0_MFP_ALT0_Msk              (0x1ul << SYS_P0_MFP_ALT0_Pos)                    /*!< SYS_T::P0_MFP: ALT0 Mask                  */

#define SYS_P0_MFP_ALT1_Pos              (9)                                               /*!< SYS_T::P0_MFP: ALT1 Position              */
#define SYS_P0_MFP_ALT1_Msk              (0x1ul << SYS_P0_MFP_ALT1_Pos)                    /*!< SYS_T::P0_MFP: ALT1 Mask                  */

#define SYS_P0_MFP_ALT2_Pos              (10)                                              /*!< SYS_T::P0_MFP: ALT2 Position              */
#define SYS_P0_MFP_ALT2_Msk              (0x1ul << SYS_P0_MFP_ALT2_Pos)                    /*!< SYS_T::P0_MFP: ALT2 Mask                  */

#define SYS_P0_MFP_ALT3_Pos              (11)                                              /*!< SYS_T::P0_MFP: ALT3 Position              */
#define SYS_P0_MFP_ALT3_Msk              (0x1ul << SYS_P0_MFP_ALT3_Pos)                    /*!< SYS_T::P0_MFP: ALT3 Mask                  */

#define SYS_P0_MFP_ALT4_Pos              (12)                                              /*!< SYS_T::P0_MFP: ALT4 Position              */
#define SYS_P0_MFP_ALT4_Msk              (0x1ul << SYS_P0_MFP_ALT4_Pos)                    /*!< SYS_T::P0_MFP: ALT4 Mask                  */

#define SYS_P0_MFP_ALT5_Pos              (13)                                              /*!< SYS_T::P0_MFP: ALT5 Position              */
#define SYS_P0_MFP_ALT5_Msk              (0x1ul << SYS_P0_MFP_ALT5_Pos)                    /*!< SYS_T::P0_MFP: ALT5 Mask                  */

#define SYS_P0_MFP_ALT6_Pos              (14)                                              /*!< SYS_T::P0_MFP: ALT6 Position              */
#define SYS_P0_MFP_ALT6_Msk              (0x1ul << SYS_P0_MFP_ALT6_Pos)                    /*!< SYS_T::P0_MFP: ALT6 Mask                  */

#define SYS_P0_MFP_ALT7_Pos              (15)                                              /*!< SYS_T::P0_MFP: ALT7 Position              */
#define SYS_P0_MFP_ALT7_Msk              (0x1ul << SYS_P0_MFP_ALT7_Pos)                    /*!< SYS_T::P0_MFP: ALT7 Mask                  */

#define SYS_P1_MFP_MFP_Pos               (0)                                               /*!< SYS_T::P1_MFP: MFP Position               */
#define SYS_P1_MFP_MFP_Msk               (0xfful << SYS_P1_MFP_MFP_Pos)                    /*!< SYS_T::P1_MFP: MFP Mask                   */

#define SYS_P1_MFP_ALT0_Pos              (8)                                               /*!< SYS_T::P1_MFP: ALT0 Position              */
#define SYS_P1_MFP_ALT0_Msk              (0x1ul << SYS_P1_MFP_ALT0_Pos)                    /*!< SYS_T::P1_MFP: ALT0 Mask                  */

#define SYS_P1_MFP_ALT1_Pos              (9)                                               /*!< SYS_T::P1_MFP: ALT1 Position              */
#define SYS_P1_MFP_ALT1_Msk              (0x1ul << SYS_P1_MFP_ALT1_Pos)                    /*!< SYS_T::P1_MFP: ALT1 Mask                  */

#define SYS_P1_MFP_ALT2_Pos              (10)                                              /*!< SYS_T::P1_MFP: ALT2 Position              */
#define SYS_P1_MFP_ALT2_Msk              (0x1ul << SYS_P1_MFP_ALT2_Pos)                    /*!< SYS_T::P1_MFP: ALT2 Mask                  */

#define SYS_P1_MFP_ALT3_Pos              (11)                                              /*!< SYS_T::P1_MFP: ALT3 Position              */
#define SYS_P1_MFP_ALT3_Msk              (0x1ul << SYS_P1_MFP_ALT3_Pos)                    /*!< SYS_T::P1_MFP: ALT3 Mask                  */

#define SYS_P1_MFP_ALT4_Pos              (12)                                              /*!< SYS_T::P1_MFP: ALT4 Position              */
#define SYS_P1_MFP_ALT4_Msk              (0x1ul << SYS_P1_MFP_ALT4_Pos)                    /*!< SYS_T::P1_MFP: ALT4 Mask                  */

#define SYS_P1_MFP_ALT5_Pos              (13)                                              /*!< SYS_T::P1_MFP: ALT5 Position              */
#define SYS_P1_MFP_ALT5_Msk              (0x1ul << SYS_P1_MFP_ALT5_Pos)                    /*!< SYS_T::P1_MFP: ALT5 Mask                  */

#define SYS_P1_MFP_ALT6_Pos              (14)                                              /*!< SYS_T::P1_MFP: ALT6 Position              */
#define SYS_P1_MFP_ALT6_Msk              (0x1ul << SYS_P1_MFP_ALT6_Pos)                    /*!< SYS_T::P1_MFP: ALT6 Mask                  */

#define SYS_P1_MFP_ALT7_Pos              (15)                                              /*!< SYS_T::P1_MFP: ALT7 Position              */
#define SYS_P1_MFP_ALT7_Msk              (0x1ul << SYS_P1_MFP_ALT7_Pos)                    /*!< SYS_T::P1_MFP: ALT7 Mask                  */

#define SYS_P1_MFP_TYPE_Pos              (16)                                              /*!< SYS_T::P1_MFP: TYPE Position              */
#define SYS_P1_MFP_TYPE_Msk              (0xfful << SYS_P1_MFP_TYPE_Pos)                   /*!< SYS_T::P1_MFP: TYPE Mask                  */

#define SYS_P1_MFP_P12EXT_Pos            (26)                                              /*!< SYS_T::P1_MFP: P12EXT Position            */
#define SYS_P1_MFP_P12EXT_Msk            (0x1ul << SYS_P1_MFP_P12EXT_Pos)                  /*!< SYS_T::P1_MFP: P12EXT Mask                */

#define SYS_P1_MFP_P13EXT_Pos            (27)                                              /*!< SYS_T::P1_MFP: P13EXT Position            */
#define SYS_P1_MFP_P13EXT_Msk            (0x1ul << SYS_P1_MFP_P13EXT_Pos)                  /*!< SYS_T::P1_MFP: P13EXT Mask                */

#define SYS_P1_MFP_P14EXT_Pos            (28)                                              /*!< SYS_T::P1_MFP: P14EXT Position            */
#define SYS_P1_MFP_P14EXT_Msk            (0x1ul << SYS_P1_MFP_P14EXT_Pos)                  /*!< SYS_T::P1_MFP: P14EXT Mask                */


#define SYS_P2_MFP_ALT0_Pos              (8)                                               /*!< SYS_T::P2_MFP: ALT0 Position              */
#define SYS_P2_MFP_ALT0_Msk              (0x1ul << SYS_P2_MFP_ALT0_Pos)                    /*!< SYS_T::P2_MFP: ALT0 Mask                  */

#define SYS_P2_MFP_ALT1_Pos              (9)                                               /*!< SYS_T::P2_MFP: ALT1 Position              */
#define SYS_P2_MFP_ALT1_Msk              (0x1ul << SYS_P2_MFP_ALT1_Pos)                    /*!< SYS_T::P2_MFP: ALT1 Mask                  */


#define SYS_P2_MFP_ALT2_Pos              (10)                                              /*!< SYS_T::P2_MFP: ALT2 Position              */
#define SYS_P2_MFP_ALT2_Msk              (0x1ul << SYS_P2_MFP_ALT2_Pos)                    /*!< SYS_T::P2_MFP: ALT2 Mask                  */

#define SYS_P2_MFP_ALT3_Pos              (11)                                              /*!< SYS_T::P2_MFP: ALT3 Position              */
#define SYS_P2_MFP_ALT3_Msk              (0x1ul << SYS_P2_MFP_ALT3_Pos)                    /*!< SYS_T::P2_MFP: ALT3 Mask                  */

#define SYS_P2_MFP_ALT4_Pos              (12)                                              /*!< SYS_T::P2_MFP: ALT4 Position              */
#define SYS_P2_MFP_ALT4_Msk              (0x1ul << SYS_P2_MFP_ALT4_Pos)                    /*!< SYS_T::P2_MFP: ALT4 Mask                  */

#define SYS_P2_MFP_ALT5_Pos              (13)                                              /*!< SYS_T::P2_MFP: ALT5 Position              */
#define SYS_P2_MFP_ALT5_Msk              (0x1ul << SYS_P2_MFP_ALT5_Pos)                    /*!< SYS_T::P2_MFP: ALT5 Mask                  */

#define SYS_P2_MFP_ALT6_Pos              (14)                                              /*!< SYS_T::P2_MFP: ALT6 Position              */
#define SYS_P2_MFP_ALT6_Msk              (0x1ul << SYS_P2_MFP_ALT6_Pos)                    /*!< SYS_T::P2_MFP: ALT6 Mask                  */


#define SYS_P2_MFP_ALT7_Pos              (15)                                              /*!< SYS_T::P2_MFP: ALT7 Position              */
#define SYS_P2_MFP_ALT7_Msk              (0x1ul << SYS_P2_MFP_ALT7_Pos)                    /*!< SYS_T::P2_MFP: ALT7 Mask                  */


#define SYS_P3_MFP_MFP_Pos               (0)                                               /*!< SYS_T::P3_MFP: MFP Position               */
#define SYS_P3_MFP_MFP_Msk               (0xfful << SYS_P3_MFP_MFP_Pos)                    /*!< SYS_T::P3_MFP: MFP Mask                   */

#define SYS_P3_MFP_ALT0_Pos              (8)                                               /*!< SYS_T::P3_MFP: ALT0 Position              */
#define SYS_P3_MFP_ALT0_Msk              (0x1ul << SYS_P3_MFP_ALT0_Pos)                    /*!< SYS_T::P3_MFP: ALT0 Mask                  */

#define SYS_P3_MFP_ALT1_Pos              (9)                                               /*!< SYS_T::P3_MFP: ALT1 Position              */
#define SYS_P3_MFP_ALT1_Msk              (0x1ul << SYS_P3_MFP_ALT1_Pos)                    /*!< SYS_T::P3_MFP: ALT1 Mask                  */

#define SYS_P3_MFP_ALT2_Pos              (10)                                              /*!< SYS_T::P3_MFP: ALT2 Position              */
#define SYS_P3_MFP_ALT2_Msk              (0x1ul << SYS_P3_MFP_ALT2_Pos)                    /*!< SYS_T::P3_MFP: ALT2 Mask                  */

#define SYS_P3_MFP_ALT4_Pos              (12)                                              /*!< SYS_T::P3_MFP: ALT4 Position              */
#define SYS_P3_MFP_ALT4_Msk              (0x1ul << SYS_P3_MFP_ALT4_Pos)                    /*!< SYS_T::P3_MFP: ALT4 Mask                  */

#define SYS_P3_MFP_ALT5_Pos              (13)                                              /*!< SYS_T::P3_MFP: ALT5 Position              */
#define SYS_P3_MFP_ALT5_Msk              (0x1ul << SYS_P3_MFP_ALT5_Pos)                    /*!< SYS_T::P3_MFP: ALT5 Mask                  */

#define SYS_P3_MFP_ALT6_Pos              (14)                                              /*!< SYS_T::P3_MFP: ALT6 Position              */
#define SYS_P3_MFP_ALT6_Msk              (0x1ul << SYS_P3_MFP_ALT6_Pos)                    /*!< SYS_T::P3_MFP: ALT6 Mask                  */

#define SYS_P3_MFP_P32EXT_Pos            (26)                                              /*!< SYS_T::P3_MFP: P32EXT Position            */
#define SYS_P3_MFP_P32EXT_Msk            (0x1ul << SYS_P3_MFP_P32EXT_Pos)                  /*!< SYS_T::P3_MFP: P32EXT Mask                */

#define SYS_P4_MFP_MFP_Pos               (0)                                               /*!< SYS_T::P4_MFP: MFP Position               */
#define SYS_P4_MFP_MFP_Msk               (0xfful << SYS_P4_MFP_MFP_Pos)                    /*!< SYS_T::P4_MFP: MFP Mask                   */

#define SYS_P4_MFP_ALT6_Pos              (14)                                              /*!< SYS_T::P4_MFP: ALT6 Position              */
#define SYS_P4_MFP_ALT6_Msk              (0x1ul << SYS_P4_MFP_ALT6_Pos)                    /*!< SYS_T::P4_MFP: ALT6 Mask                  */

#define SYS_P4_MFP_ALT7_Pos              (15)                                              /*!< SYS_T::P4_MFP: ALT7 Position              */
#define SYS_P4_MFP_ALT7_Msk              (0x1ul << SYS_P4_MFP_ALT7_Pos)                    /*!< SYS_T::P4_MFP: ALT7 Mask                  */

#define SYS_P5_MFP_MFP_Pos               (0)                                               /*!< SYS_T::P5_MFP: MFP Position               */
#define SYS_P5_MFP_MFP_Msk               (0xfful << SYS_P5_MFP_MFP_Pos)                    /*!< SYS_T::P5_MFP: MFP Mask                   */

#define SYS_P5_MFP_ALT0_Pos              (8)                                               /*!< SYS_T::P5_MFP: ALT0 Position              */
#define SYS_P5_MFP_ALT0_Msk              (0x1ul << SYS_P5_MFP_ALT0_Pos)                    /*!< SYS_T::P5_MFP: ALT0 Mask                  */

#define SYS_P5_MFP_ALT1_Pos              (9)                                               /*!< SYS_T::P5_MFP: ALT1 Position              */
#define SYS_P5_MFP_ALT1_Msk              (0x1ul << SYS_P5_MFP_ALT1_Pos)                    /*!< SYS_T::P5_MFP: ALT1 Mask                  */

#define SYS_P5_MFP_ALT2_Pos              (10)                                              /*!< SYS_T::P5_MFP: ALT2 Position              */
#define SYS_P5_MFP_ALT2_Msk              (0x1ul << SYS_P5_MFP_ALT2_Pos)                    /*!< SYS_T::P5_MFP: ALT2 Mask                  */

#define SYS_P5_MFP_ALT3_Pos              (11)                                              /*!< SYS_T::P5_MFP: ALT3 Position              */
#define SYS_P5_MFP_ALT3_Msk              (0x1ul << SYS_P5_MFP_ALT3_Pos)                    /*!< SYS_T::P5_MFP: ALT3 Mask                  */

#define SYS_P5_MFP_ALT4_Pos              (12)                                              /*!< SYS_T::P5_MFP: ALT4 Position              */
#define SYS_P5_MFP_ALT4_Msk              (0x1ul << SYS_P5_MFP_ALT4_Pos)                    /*!< SYS_T::P5_MFP: ALT4 Mask                  */

#define SYS_P5_MFP_ALT5_Pos              (13)                                              /*!< SYS_T::P5_MFP: ALT5 Position              */
#define SYS_P5_MFP_ALT5_Msk              (0x1ul << SYS_P5_MFP_ALT5_Pos)                    /*!< SYS_T::P5_MFP: ALT5 Mask                  */

#define SYS_P5_MFP_ALT6_Pos              (14)                                              /*!< SYS_T::P5_MFP: ALT6 Position              */
#define SYS_P5_MFP_ALT6_Msk              (0x1ul << SYS_P5_MFP_ALT5_Pos)                    /*!< SYS_T::P5_MFP: ALT6 Mask                  */

#define SYS_P5_MFP_ALT7_Pos              (15)                                              /*!< SYS_T::P5_MFP: ALT7 Position              */
#define SYS_P5_MFP_ALT7_Msk              (0x1ul << SYS_P5_MFP_ALT5_Pos)                    /*!< SYS_T::P5_MFP: ALT7 Mask                  */

#define SYS_P5_MFP_P50EXT_Pos              (24)                                              /*!< SYS_T::P5_MFP: P50EXT Position              */
#define SYS_P5_MFP_P50EXT_Msk              (0x1ul << SYS_P5_MFP_P50EXT_Pos)                    /*!< SYS_T::P5_MFP: P50EXT Mask                  */

#define SYS_P5_MFP_P51EXT_Pos              (25)                                              /*!< SYS_T::P5_MFP: P51EXT Position              */
#define SYS_P5_MFP_P51EXT_Msk              (0x1ul << SYS_P5_MFP_P51EXT_Pos)                    /*!< SYS_T::P5_MFP: P51EXT Mask                  */

#define	 SYS_TESTCTL_LDT_Pos				(0)
#define	 SYS_TESTCTL_LDT_Msk				(0x1ul << SYS_TESTCTL_LDT_Pos)

#define	 SYS_TESTCTL_POR_Pos				(1)
#define	 SYS_TESTCTL_POR_Msk				(0x1ul << SYS_TESTCTL_POR_Pos)

#define	 SYS_TESTCTL_BG3V_Pos				(2)
#define	 SYS_TESTCTL_BG3V_Msk				(0x1ul << SYS_TESTCTL_BG3V_Pos)

#define	 SYS_TESTCTL_10K_Pos				(3)
#define	 SYS_TESTCTL_10K_Msk				(0x1ul << SYS_TESTCTL_10K_Pos)

#define	 SYS_TESTCTL_50M_Pos				(4)
#define	 SYS_TESTCTL_50M_Msk				(0x1ul << SYS_TESTCTL_50M_Pos)

#define	 SYS_TESTCTL_CMPDATA_Pos			(5)
#define	 SYS_TESTCTL_CMPDATA_Msk			(0x1ul << SYS_TESTCTL_CMPDATA_Pos)

#define	 SYS_TESTCTL_CKCMP_Pos				(6)
#define	 SYS_TESTCTL_CKCMP_Msk				(0x1ul << SYS_TESTCTL_CKCMP_Pos)

#define	 SYS_TESTCTL_SH_Pos					(7)
#define	 SYS_TESTCTL_SH_Msk					(0x1ul << SYS_TESTCTL_SH_Pos)

#define	 SYS_TESTCTL_ACMP1O_Pos				(8)
#define	 SYS_TESTCTL_ACMP1O_Msk				(0x1ul << SYS_TESTCTL_ACMP1O_Pos)

#define	 SYS_TESTCTL_ACMP0O_Pos				(9)
#define	 SYS_TESTCTL_ACMP0O_Msk				(0x1ul << SYS_TESTCTL_ACMP0O_Pos)

#define	 SYS_TESTCTL_BODO_Pos				(10)
#define	 SYS_TESTCTL_BODO_Msk				(0x1ul << SYS_TESTCTL_BODO_Pos)

#define	 SYS_BLDBCTL_BODDBSEL_Pos			(0)
#define	 SYS_BLDBCTL_BODDBSEL_Msk			(0x3Ful << SYS_BLDBCTL_BODDBSEL_Pos)

#define	 SYS_BLDBCTL_LVRDBSEL_Pos			(8)
#define	 SYS_BLDBCTL_LVRDBSEL_Msk			(0x3Ful << SYS_BLDBCTL_LVRDBSEL_Pos)

#define SYS_REGLCTL_REGLCTL_Pos          (0)                                               /*!< SYS_T::REGLCTL: REGLCTL Position          */
#define SYS_REGLCTL_REGLCTL_Msk          (0xfful << SYS_REGLCTL_REGLCTL_Pos)               /*!< SYS_T::REGLCTL: REGLCTL Mask              */

/**@}*/ /* SYS_CONST */
/**@}*/ /* end of SYS register group */


/*---------------------- Timer Controller -------------------------*/
/**
    @addtogroup TIMER Timer Controller(TIMER)
    Memory Mapped Structure for TIMER Controller
@{ */

typedef struct {


    /**
     * CTL
     * ===================================================================================================
     * Offset: 0x00  Timer0 Control and Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |PSC       |Prescale Counter
     * |        |          |Timer input clock or event source is divided by (PSC+1) before it is fed to the timer up counter.
     * |        |          |If this field is 0 (PSC = 0), then there is no scaling.
     * |[19]    |CAPSRC    |Capture Pin Source Select Bit
     * |        |          |0 = Capture Function source is from TMx_EXT (x= 0~1)
     * |        |          |pin.
     * |        |          |1 = Capture Function source is from internal ACMP output signal.
     * |        |          |User can set CAPSRCMP (TIMERx_EXTCTL[9]) to decide which ACMP output signal as timer capture source.
     * |[23]    |WKEN      |Wake-up Function Enable Bit
     * |        |          |If this bit is set to 1, while the timer interrupt flag TIF (TIMERx_INTSTS[0]) is 1 and INTEN (TIMERx_CTL[29]) is enabled, the timer interrupt signal will generate a wake-up trigger event to CPU.
     * |        |          |0 = Wake-up function Disabled if timer interrupt signal generated.
     * |        |          |1 = Wake-up function Enabled if timer interrupt signal generated.
     * |[24]    |EXTCNTEN  |Event Counter Mode Enable Bit
     * |        |          |This bit is for external counting pin function enabled.
     * |        |          |0 = Event counter mode Disabled.
     * |        |          |1 = Event counter mode Enabled.
     * |        |          |Note: When timer is used as an event counter, this bit should be set to 1 and select HCLK as timer clock source
     * |[25]    |ACTSTS    |Timer Active Status (Read Only)
     * |        |          |This bit indicates the 24-bit up counter status.
     * |        |          |0 = 24-bit up counter is not active.
     * |        |          |1 = 24-bit up counter is active.
     * |[26]    |RSTCNT    |Timer Counter Reset
     * |        |          |Setting this bit will reset the 24-bit up counter value CNT (TIMERx_CNT[23:0]) and also force CNTEN (TIMERx_CTL[30]) to 0 if ACTSTS (TIMERx_CTL[25]) is 1.
     * |        |          |0 = No effect.
     * |        |          |1 = Reset internal 8-bit prescale counter, 24-bit up counter value and CNTEN bit.
     * |[28:27] |OPMODE    |Timer Counting Mode Selection
     * |        |          |00 = The Timer controller is operated in one-shot mode.
     * |        |          |01 = The Timer controller is operated in periodic mode.
     * |        |          |10 = The Timer controller is operated in toggle-output mode.
     * |        |          |11 = The Timer controller is operated in continuous counting mode.
     * |[29]    |INTEN     |Timer Interrupt Enable Bit
     * |        |          |0 = Timer Interrupt Disabled.
     * |        |          |1 = Timer Interrupt Enabled.
     * |        |          |Note: If this bit is enabled, when the timer interrupt flag TIF is set to 1, the timer interrupt signal will be generated and inform CPU.
     * |[30]    |CNTEN     |Timer Counting Enable Bit
     * |        |          |0 = Stops/Suspends counting.
     * |        |          |1 = Starts counting.
     * |        |          |Note1: In stop status, and then setting CNTEN to 1 will enable the 24-bit up counter to keep counting from the last stop counting value.
     * |        |          |Note2: This bit is auto-cleared by hardware in one-shot mode (TIMERx_CTL[28:27] = 2'b00) when the timer interrupt flag TIF (TIMERx_INTSTS[0]) is generated.
     * |[31]    |ICEDEBUG  |ICE Debug Mode Acknowledge Disable Bit (Write Protect)
     * |        |          |0 = ICE debug mode acknowledgement effects TIMER counting.
     * |        |          |TIMER counter will be held while CPU is held by ICE.
     * |        |          |1 = ICE debug mode acknowledgement Disabled.
     * |        |          |TIMER counter will keep going no matter CPU is held by ICE or not.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
    */
    __IO uint32_t CTL;

    /**
     * CMP
     * ===================================================================================================
     * Offset: 0x04  Timer0 Compare Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[23:0]  |CMPDAT    |Timer Compared Value
     * |        |          |CMPDAT is a 24-bit compared value register.
     * |        |          |When the internal 24-bit up counter value is equal to CMPDAT value, the TIF (TIMERx_INTSTS[0] Timer Interrupt Flag) will set to .
     * |        |          |Time-out period = (Period of timer clock input) * (8-bit PSC + 1) * (24-bit CMPDAT).
     * |        |          |Note1: Never write 0x0 or 0x1 in CMPDAT field, or the core will run into unknown state.
     * |        |          |Note2: When timer is operating at continuous counting mode, the 24-bit up counter will keep counting continuously even if user writes a new value into CMPDAT field.
     * |        |          |But if timer is operating at other modes, the 24-bit up counter will restart counting from 0 and using newest CMPDAT value to be the timer compared value while user writes a new value into the CMPDAT fiel.
    */
    __IO uint32_t CMP;

    /**
     * INTSTS
     * ===================================================================================================
     * Offset: 0x08  Timer0 Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TIF       |Timer Interrupt Flag
     * |        |          |This bit indicates the interrupt flag status of Timer while 24-bit timer up counter CNT (TIMERx_CNT[23:0]) value reaches to CMPDAT (TIMERx_CMP[23:0]) value.
     * |        |          |0 = No effect.
     * |        |          |1 = CNT value matches the CMPDAT value, and interrupt occur.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * |[1]     |TWKF      |Timer Wake-up Flag
     * |        |          |This bit indicates the interrupt wake-up flag status of timer.
     * |        |          |0 = Timer does not cause CPU wake-up.
     * |        |          |1 = CPU wake-up from Idle or Power-down mode if timer time-out interrupt signal generated.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * |[2]     |TF      	 |Timer Flag
     * |        |          |This bit indicates the flag status of Timer while 24-bit timer up counter CNT (TIMERx_CNT[23:0]) value reaches to CMPDAT (TIMERx_CMP[23:0]) value.
     * |        |          |0 = Timer No effect.
     * |        |          |1 = CNT value matches the CMPDAT value.
     * |        |          |Note: This bit is cleared by writing 1 to it.
    */
    __IO uint32_t INTSTS;

    /**
     * CNT
     * ===================================================================================================
     * Offset: 0x0C  Timer0 Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[23:0]  |CNT       |Timer Data Register
     * |        |          |This field can be reflected the internal 24-bit timer counter value or external event input counter value from TMx (x=0~1) pin.
     * |        |          |If EXTCNTEN (TIMERx_CTL[24]) is 0, user can read CNT value for getting current 24- bit counter value.
     * |        |          |If EXTCNTEN (TIMERx_CTL[24]) is 1, user can read CNT value for getting current 24- bit event input counter value.
    */
    __I  uint32_t CNT;

    /**
     * CAP
     * ===================================================================================================
     * Offset: 0x10  Timer0 Capture Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[23:0]  |CAPDAT    |Timer Capture Data Register
     * |        |          |When CAPEN (TIMERx_EXTCTL[3]) bit is set, CAPFUNCS (TIMERx_EXTCTL[4]) bit is 0, and a transition on TMx_EXT pin matched the CAPEDGE (TIMERx_EXTCTL[2:1]) setting, CAPIF (TIMERx_EINTSTS[0]) will set to 1 and the current timer counter value CNT (TIMERx_CNT[23:0]) will be auto-loaded into this CAPDAT field.
    */
    __I  uint32_t CAP;

    /**
     * EXTCTL
     * ===================================================================================================
     * Offset: 0x14  Timer0 External Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CNTPHASE  |Timer External Count Phase
     * |        |          |This bit indicates the detection phase of external counting pin TMx (x= 0~1).
     * |        |          |0 = A falling edge of external counting pin will be counted.
     * |        |          |1 = A rising edge of external counting pin will be counted.
     * |[2:1]   |CAPEDGE   |Timer External Capture Pin Edge Detection
     * |        |          |00 = A falling edge on TMx_EXT (x= 0~1) pin will be detected.
     * |        |          |01 = A rising edge on TMx_EXT (x= 0~1) pin will be detected.
     * |        |          |10 = Either rising or falling edge on TMx_EXT (x= 0~1) pin will be detected.
     * |        |          |11 = Reserved.
     * |[3]     |CAPEN     |Timer External Capture Pin Enable Bit
     * |        |          |This bit enables the TMx_EXT pin.
     * |        |          |0 = TMx_EXT (x= 0~1) pin Disabled.
     * |        |          |1 = TMx_EXT (x= 0~1) pin Enabled.
     * |[4]     |CAPFUNCS  |Capture Function Select Bit
     * |        |          |0 = External Capture Mode Enabled.
     * |        |          |1 = External Reset Mode Enabled.
     * |        |          |Note1: When CAPFUNCS is 0, transition on TMx_EXT (x= 0~1) pin is using to save the 24-bit timer counter value to CAPDAT register.
     * |        |          |Note2: When CAPFUNCS is 1, transition on TMx_EXT (x= 0~1) pin is using to reset the 24-bit timer counter value.
     * |[5]     |CAPIEN    |Timer External Capture Interrupt Enable Bit
     * |        |          |0 = TMx_EXT (x= 0~1) pin detection Interrupt Disabled.
     * |        |          |1 = TMx_EXT (x= 0~1) pin detection Interrupt Enabled.
     * |        |          |Note: CAPIEN is used to enable timer external interrupt.
     * |        |          |If CAPIEN enabled, timer will generate an interrupt when CAPIF (TIMERx_EINTSTS[0]) is .
     * |        |          |For example, while CAPIEN = 1, CAPEN = 1, and CAPEDGE = 00, an 1 to 0 transition on the TMx_EXT pin will cause the CAPIF to be set then the interrupt signal is generated and sent to NVIC to inform CPU.
     * |[6]     |CAPDBEN   |Timer External Capture Pin De-bounce Enable Bit
     * |        |          |0 = TMx_EXT (x= 0~1) pin de-bounce Disabled.
     * |        |          |1 = TMx_EXT (x= 0~1) pin de-bounce Enabled.
     * |        |          |Note1: If this bit is enabled, the edge detection of TMx_EXT pin is detected with de-bounce circuit.
     * |        |          |Note2: The de-bounce circuit doesn't support ACMP output.
     * |[7]     |CNTDBEN   |Timer Counter Pin De-bounce Enable Bit
     * |        |          |0 = TMx (x= 0~1) pin de-bounce Disabled.
     * |        |          |1 = TMx (x= 0~1) pin de-bounce Enabled.
     * |        |          |Note: If this bit is enabled, the edge detection of TMx pin is detected with de-bounce circuit.
     * |[8]     |CAPSEL    |Capture Mode Select Bit
     * |        |          |0 = Timer counter reset function or free-counting mode of timer capture function.
     * |        |          |1 = Trigger-counting mode of timer capture function.
     * |[9]     |ACMPSSEL  |ACMP Source Selection to Trigger Capture Function
     * |        |          |For Timer 0:
     * |        |          |0 = Capture Function source is from ACMP0 output signal for TIMER.
     * |        |          |1 = Capture Function source is from ACMP1 output signal for TIMER.
    */
    __IO uint32_t EXTCTL;

    /**
     * EINTSTS
     * ===================================================================================================
     * Offset: 0x18  Timer0 External Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CAPIF     |Timer External Capture Interrupt Flag
     * |        |          |This bit indicates the timer external capture interrupt flag status.
     * |        |          |0 = TMx_EXT (x= 0~1) pin interrupt did not occur.
     * |        |          |1 = TMx_EXT (x= 0~1) pin interrupt occurred.
     * |        |          |Note1: This bit is cleared by writing 1 to it.
     * |        |          |Note2: When CAPEN (TIMERx_EXTCTL[3]) bit is set, CAPFUNCS (TIMERx_EXTCTL[4]) bit is 0, and a transition on TMx_EXT (x= 0~1) pin matched the CAPEDGE (TIMERx_EXTCTL[2:1]) setting, this bit will set to 1 by hardware.
     * |        |          |Note3: There is a new incoming capture event detected before CPU clearing the CAPIF status.
     * |        |          |If the above condition occurred, the Timer will keep register TIMERx_CAP unchanged and drop the new capture value.
     * |[1]     |CAPF      |Timer External Capture Flag
     * |        |          |This bit indicates the timer external capture flag status.
     * |        |          |0 = TMx_EXT (x= 0~1) pin did not occur.
     * |        |          |1 = TMx_EXT (x= 0~1) pin occurred.
     * |        |          |Note1: This bit is cleared by writing 1 to it.
     * |        |          |Note2: When CAPEN (TIMERx_EXTCTL[3]) bit is set, CAPFUNCS (TIMERx_EXTCTL[4]) bit is 0, and a transition on TMx_EXT (x= 0~1) pin matched the CAPEDGE (TIMERx_EXTCTL[2:1]) setting, this bit will set to 1 by hardware.
     * |        |          |Note3: There is a new incoming capture event detected before CPU clearing the CAPF status.
     * |        |          |If the above condition occurred, the Timer will keep register TIMERx_CAP unchanged and drop the new capture value.
*/
    __IO uint32_t EINTSTS;

} TIMER_T;

/**
    @addtogroup TIMER_CONST TIMER Bit Field Definition
    Constant Definitions for TIMER Controller
@{ */

#define TIMER_CTL_PSC_Pos                  (0)                                               /*!< TIMER_T::CTL: PSC Position                  */
#define TIMER_CTL_PSC_Msk                  (0xfful << TIMER_CTL_PSC_Pos)                     /*!< TIMER_T::CTL: PSC Mask                      */

#define TIMER_CTL_CAPSRC_Pos               (19)                                              /*!< TIMER_T::CTL: CAPSRC Position               */
#define TIMER_CTL_CAPSRC_Msk               (0x1ul << TIMER_CTL_CAPSRC_Pos)                   /*!< TIMER_T::CTL: CAPSRC Mask                   */

#define TIMER_CTL_WKEN_Pos                 (23)                                              /*!< TIMER_T::CTL: WKEN Position                 */
#define TIMER_CTL_WKEN_Msk                 (0x1ul << TIMER_CTL_WKEN_Pos)                     /*!< TIMER_T::CTL: WKEN Mask                     */

#define TIMER_CTL_EXTCNTEN_Pos             (24)                                              /*!< TIMER_T::CTL: EXTCNTEN Position             */
#define TIMER_CTL_EXTCNTEN_Msk             (0x1ul << TIMER_CTL_EXTCNTEN_Pos)                 /*!< TIMER_T::CTL: EXTCNTEN Mask                 */

#define TIMER_CTL_ACTSTS_Pos               (25)                                              /*!< TIMER_T::CTL: ACTSTS Position               */
#define TIMER_CTL_ACTSTS_Msk               (0x1ul << TIMER_CTL_ACTSTS_Pos)                   /*!< TIMER_T::CTL: ACTSTS Mask                   */

#define TIMER_CTL_RSTCNT_Pos               (26)                                              /*!< TIMER_T::CTL: RSTCNT Position               */
#define TIMER_CTL_RSTCNT_Msk               (0x1ul << TIMER_CTL_RSTCNT_Pos)                   /*!< TIMER_T::CTL: RSTCNT Mask                   */

#define TIMER_CTL_OPMODE_Pos               (27)                                              /*!< TIMER_T::CTL: OPMODE Position               */
#define TIMER_CTL_OPMODE_Msk               (0x3ul << TIMER_CTL_OPMODE_Pos)                   /*!< TIMER_T::CTL: OPMODE Mask                   */

#define TIMER_CTL_INTEN_Pos                (29)                                              /*!< TIMER_T::CTL: INTEN Position                */
#define TIMER_CTL_INTEN_Msk                (0x1ul << TIMER_CTL_INTEN_Pos)                    /*!< TIMER_T::CTL: INTEN Mask                    */

#define TIMER_CTL_CNTEN_Pos                (30)                                              /*!< TIMER_T::CTL: CNTEN Position                */
#define TIMER_CTL_CNTEN_Msk                (0x1ul << TIMER_CTL_CNTEN_Pos)                    /*!< TIMER_T::CTL: CNTEN Mask                    */

#define TIMER_CTL_ICEDEBUG_Pos             (31)                                              /*!< TIMER_T::CTL: ICEDEBUG Position             */
#define TIMER_CTL_ICEDEBUG_Msk             (0x1ul << TIMER_CTL_ICEDEBUG_Pos)                 /*!< TIMER_T::CTL: ICEDEBUG Mask                 */

#define TIMER_CMP_CMPDAT_Pos               (0)                                               /*!< TIMER_T::CMP: CMPDAT Position               */
#define TIMER_CMP_CMPDAT_Msk               (0xfffffful << TIMER_CMP_CMPDAT_Pos)              /*!< TIMER_T::CMP: CMPDAT Mask                   */

#define TIMER_INTSTS_TIF_Pos               (0)                                               /*!< TIMER_T::INTSTS: TIF Position               */
#define TIMER_INTSTS_TIF_Msk               (0x1ul << TIMER_INTSTS_TIF_Pos)                   /*!< TIMER_T::INTSTS: TIF Mask                   */

#define TIMER_INTSTS_TWKF_Pos              (1)                                               /*!< TIMER_T::INTSTS: TWKF Position              */
#define TIMER_INTSTS_TWKF_Msk              (0x1ul << TIMER_INTSTS_TWKF_Pos)                  /*!< TIMER_T::INTSTS: TWKF Mask                  */

#define TIMER_INTSTS_TF_Pos              	 (2)                                               /*!< TIMER_T::INTSTS: TF Position              */
#define TIMER_INTSTS_TF_Msk              	 (0x1ul << TIMER_INTSTS_TF_Pos)                  	 /*!< TIMER_T::INTSTS: TF Mask  								*/

#define TIMER_CNT_CNT_Pos                  (0)                                               /*!< TIMER_T::CNT: CNT Position                  */
#define TIMER_CNT_CNT_Msk                  (0xfffffful << TIMER_CNT_CNT_Pos)                 /*!< TIMER_T::CNT: CNT Mask                      */

#define TIMER_CAP_CAPDAT_Pos               (0)                                               /*!< TIMER_T::CAP: CAPDAT Position               */
#define TIMER_CAP_CAPDAT_Msk               (0xfffffful << TIMER_CAP_CAPDAT_Pos)              /*!< TIMER_T::CAP: CAPDAT Mask                   */

#define TIMER_EXTCTL_CNTPHASE_Pos          (0)                                               /*!< TIMER_T::EXTCTL: CNTPHASE Position          */
#define TIMER_EXTCTL_CNTPHASE_Msk          (0x1ul << TIMER_EXTCTL_CNTPHASE_Pos)              /*!< TIMER_T::EXTCTL: CNTPHASE Mask              */

#define TIMER_EXTCTL_CAPEDGE_Pos           (1)                                               /*!< TIMER_T::EXTCTL: CAPEDGE Position           */
#define TIMER_EXTCTL_CAPEDGE_Msk           (0x3ul << TIMER_EXTCTL_CAPEDGE_Pos)               /*!< TIMER_T::EXTCTL: CAPEDGE Mask               */

#define TIMER_EXTCTL_CAPEN_Pos             (3)                                               /*!< TIMER_T::EXTCTL: CAPEN Position             */
#define TIMER_EXTCTL_CAPEN_Msk             (0x1ul << TIMER_EXTCTL_CAPEN_Pos)                 /*!< TIMER_T::EXTCTL: CAPEN Mask                 */

#define TIMER_EXTCTL_CAPFUNCS_Pos          (4)                                               /*!< TIMER_T::EXTCTL: CAPFUNCS Position          */
#define TIMER_EXTCTL_CAPFUNCS_Msk          (0x1ul << TIMER_EXTCTL_CAPFUNCS_Pos)              /*!< TIMER_T::EXTCTL: CAPFUNCS Mask              */

#define TIMER_EXTCTL_CAPIEN_Pos            (5)                                               /*!< TIMER_T::EXTCTL: CAPIEN Position            */
#define TIMER_EXTCTL_CAPIEN_Msk            (0x1ul << TIMER_EXTCTL_CAPIEN_Pos)                /*!< TIMER_T::EXTCTL: CAPIEN Mask                */

#define TIMER_EXTCTL_CAPDBEN_Pos           (6)                                               /*!< TIMER_T::EXTCTL: CAPDBEN Position           */
#define TIMER_EXTCTL_CAPDBEN_Msk           (0x1ul << TIMER_EXTCTL_CAPDBEN_Pos)               /*!< TIMER_T::EXTCTL: CAPDBEN Mask               */

#define TIMER_EXTCTL_CNTDBEN_Pos           (7)                                               /*!< TIMER_T::EXTCTL: CNTDBEN Position           */
#define TIMER_EXTCTL_CNTDBEN_Msk           (0x1ul << TIMER_EXTCTL_CNTDBEN_Pos)               /*!< TIMER_T::EXTCTL: CNTDBEN Mask               */

#define TIMER_EXTCTL_CAPSEL_Pos            (8)                                               /*!< TIMER_T::EXTCTL: CAPSEL Position            */
#define TIMER_EXTCTL_CAPSEL_Msk            (0x1ul << TIMER_EXTCTL_CAPSEL_Pos)                /*!< TIMER_T::EXTCTL: CAPSEL Mask                */

#define TIMER_EXTCTL_ACMPSSEL_Pos          (9)                                               /*!< TIMER_T::EXTCTL: ACMPSSEL Position          */
#define TIMER_EXTCTL_ACMPSSEL_Msk          (0x1ul << TIMER_EXTCTL_ACMPSSEL_Pos)              /*!< TIMER_T::EXTCTL: ACMPSSEL Mask              */

#define TIMER_EINTSTS_CAPIF_Pos            (0)                                               /*!< TIMER_T::EINTSTS: CAPIF Position            */
#define TIMER_EINTSTS_CAPIF_Msk            (0x1ul << TIMER_EINTSTS_CAPIF_Pos)                /*!< TIMER_T::EINTSTS: CAPIF Mask                */

#define TIMER_EINTSTS_CAPF_Pos             (1)                                               /*!< TIMER_T::EINTSTS: CAPF Position            */
#define TIMER_EINTSTS_CAPF_Msk             (0x1ul << TIMER_EINTSTS_CAPF_Pos)                 /*!< TIMER_T::EINTSTS: CAPF Mask                */

/**@}*/ /* TIMER_CONST */
/**@}*/ /* end of TIMER register group */


/*---------------------- Universal Asynchronous Receiver/Transmitter Controller -------------------------*/
/**
    @addtogroup UART Universal Asynchronous Receiver/Transmitter Controller(UART)
    Memory Mapped Structure for UART Controller
@{ */

typedef struct {
     /**
     * RBR
     * ===================================================================================================
     * Offset: 0x00  UART Receive Buffer Register. This register can be accessed only when the DLAB bit (LCR[7]) is cleared.
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |DAT       |Data byte received on the serial input port (sin) in UART mode,
     * |        |          |or the serial infrared input (sir_in) in infrared mode.
     * |        |          |
     * |[8]     |MSB9      |Data byte received on the serial input port (sin) in UART mode for the MSB 9th bit.
     * |        |          |It is applicable only when UART_9BIT_DATA_EN=1.
     */
    /**
     * THR
     * ===================================================================================================
     * Offset: 0x00  UART Transmit Holding Register. This register can be accessed only when the DLAB bit (LCR[7]) is cleared.
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |DAT       |Transmit Holding Register
     * |        |          |Write :
     * |        |          |Data to be transmitted on the serial output port in UART mode or the serial infrared output in infrared mode.
     * |        |          |
     * |[8]     |MSB9      |Data to be transmitted on the serial output port in UART mode for the MSB 9th bit.
     * |        |          |It is applicable only when UART_9BIT_DATA_EN=1.
     */
    /**
     * DLL
     * ===================================================================================================
     * Offset: 0x00  Divisor Latch (Low) This register can be accessed only when the DLAB bit (LCR[7]) is set.
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   | DLL      |Lower 8 bits  of a 16-bit, read/write,
     * |        |          |Divisor Latch register that contains the baud rate divisor for the UART.
     * |        |          |
     */
    __IO uint32_t RBR_THR_DLL;  // receive buffer register
                                    // receive buffer register
                                    // divisor latch low          (0x00)
    /**
     * DLH
     * ===================================================================================================
     * Offset: 0x04  Divisor Latch (High) This register can be accessed only when the DLAB bit (LCR[7]) is set.
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   | DLH      |Upper 8-bits of a 16-bit, read/write,
     * |        |          |Divisor Latch register that contains the baud rate divisor for the UART.
     * |        |          |
     */
    /**
     * IER
     * ===================================================================================================
     * Offset: 0x04  Interrupt Enable Register  This register can be accessed only when the DLAB bit (LCR[7]) is cleared.
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7]     | EPTI     |Enable Programmable THRE Interrupt . This is used to enable/disable the generation of THRE Interrupt.
     * |        |          |
     * |        |          |
     * |[6:4]   | Reserved |
     * |        |          |
     * |[3]     | EMSI     |Enable Modem Status Interrupt. This is used to enable/disable the generation of Modem Status Interrupt.
     * |        |          |This is the fourth highest priority interrupt.
     * |[2]     | ERLSI    |Enable Receiver Line Status Interrupt. This is used to enable/disable the generation of Receiver Line Status Interrupt.
     * |        |          |This is the highest priority interrupt.
     * |[1]     | ETHREI   |Enable Transmit Holding Register Empty Interrupt. This is used to enable/disable the generation of Transmitter Holding Register Empty Interrupt.
     * |        |          |This is the third highest priority interrupt.
     * |[0]     | ERDAI    |Enable Received Data Available Interrupt. This is used to enable/disable the generation of Received Data Available Interrupt and the Character Timeout Interrupt.
     * |        |          |These are the second highest priority interrupts.
     */
    __IO uint32_t IER_DLH;      // interrupt enable register
                                    // divisor latch high         (0x04)
    /**
     * IIR
     * ===================================================================================================
     * Offset: 0x08  Interrupt Identity Register -- read-only
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:6]   | FIFOSE   |FIFOs Enabled. This is used to indicate whether the FIFOs are enabled or disabled..
     * |        |          |
     * |[5:4]   | Reserved |
     * |        |          |
     * |[3:0]   | IID      |Interrupt ID. This indicates the highest priority pending interrupt which can be one of the following types:
     * |        |          |0000 - modem status
     * |        |          |0001 - no interrupt pending
     * |        |          |0010 - THR empty
     * |        |          |0100 - received data available
     * |        |          |0110 - receiver line status
     * |        |          |0111 - reserved
     * |        |          |1100 - character timeout
     * |        |          |Bit 3 indicates an interrupt can only occur when the FIFOs are enabled and used to distinguish a Character Timeout condition interrupt.
     */

    /**
     * FCR
     * ===================================================================================================
     * Offset: 0x08  FIFO Control Register --write-only
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:6]   | RT       |RCVR Trigger:
     * |        |          |00 - 1 character in the FIFO
     * |        |          |01 - FIFO 1/4 full
     * |        |          |10 - FIFO 1/2 full
     * |        |          |11 - FIFO 2 less than full
     * |        |          |
     * |[5:4]   | TET      |TX Empty Trigger
     * |        |          |00 - FIFO empty
     * |        |          |01 - 2 Characters in the FIFO
     * |        |          |10 - FIFO 1/4 full
     * |        |          |11 - FIFO 1/2 full
     * |        |          |
     * |[3]     |Reserved  |
     * |[2]     | XFIFOR   |XMIT FIFO Reset. This resets the control portion of the transmit FIFO and treats the FIFO as empty.
     * |        |          |
     * |[1]     | RFIFOR   |RCVR FIFO Reset. This resets the control portion of the receive FIFO and treats the FIFO as empty.
     * |        |          |
     * |[0]     | FIFOE    |FIFO Enable. This enables/disables the transmit (XMIT) and receive (RCVR) FIFOs.
     * |        |          |Whenever the value of this bit is changed both the XMIT and RCVR controller portion of FIFOs is reset.
     */

    __IO uint32_t IIR_FCR;      // interrupt identity register

    /**
     * LCR
     * ===================================================================================================
     * Offset: 0x0C  UART Line Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7]     |DLAB      |Divisor Latch Access Bit.  Always writable, always readable.
     * |        |          |This bit is used to enable reading and writing of the Divisor Latch register (DLL and DLH/LPDLL and LPDLH) to set the baud rate of the UART.
     * |        |          |This bit must be cleared after initial baud rate setup in order to access other registers.
     * |        |          |
     * |[6]     |BC        |Break Control Bit. This is used to cause a break condition to be transmitted to the receiving device.
     * |        |          |
     * |[5]     |SP        |Stick Parity. Always writable, always readable. This bit is used to force parity value.
     * |        |          |
     * |[4]     |EPS       |Even Parity Select. Always writable, always readable.
     * |        |          |This is used to select between even and odd parity, when parity is enabled (PEN set to 1)
     * |        |          |
     * |        |          |
     * |[3]     |PEN       |Parity Enable. Always writable, always readable.
     * |        |          |This bit is used to enable and disable parity generation and detection in transmitted and received serial character respectively.
     * |        |          |
     * |        |          |
     * |[2]     |STOP      |Number of stop bits.  Always writable, always readable.
     * |        |          |This is used to select the number of stop bits per character that the peripheral transmits and receives.
     * |[1:0]   |DLS       |
     * |        |          |Data Length Select.  Always writable and readable.
     * |        |          |When DLS_E in LCR_EXT is set to 0, this register is used to select the number of data bits per character
     * |        |          |that the peripheral transmits and receives. The number
     * |        |          |of bits that may be selected are as follows:
     * |        |          |00 - 5 bits
     * |        |          |01 - 6 bits
     * |        |          |10 - 7 bits
     * |        |          |11 - 8 bits
    */
    __IO uint32_t LCR;
    /**
     * MCR
     * ===================================================================================================
     * Offset: 0x10  UART Modem Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[6]     |SIRE      |SIR Mode Enable.
     * |        |          |
     * |[5]     |AFCE      |Auto Flow Control Enable.
     * |        |          |
     * |[4]     |LB        |LoopBack Bit. This is used to put the UART into a diagnostic mode for test purposes..
     * |[3]     |OUT2      |
     * |[2]     |OUT1      |
     * |[1]     |RTS       |Request to Send. This is used to directly control the Request to Send (rts_n) output.
     * |        |          |The Request To Send (rts_n) output is used to inform the modem or data set that the UART is ready to exchange data.
     * |[0]     |DTR       |Data Terminal Ready. This is used to directly control the Data Terminal Ready (dtr_n) output.
     */
    __IO uint32_t MCR;
   /**
     * LSR
     * ===================================================================================================
     * Offset: 0x14  UART Line Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[8]     |ADDR_RCVD |Address Received bit.
     * |[7]     |RFE       |Receiver FIFO Error bit. .
     * |[6]     |TEMT      |Transmitter Empty bit.
     * |        |          |
     * |[5]     |THRE      |Transmit Holding Register Empty bit.
     * |        |          |
     * |[4]     |BI        |Break Interrupt bit.
     * |[3]     |FE        |Framing Error bit
     * |[2]     |PE        |Parity Error bit
     * |[1]     |OE        |Overrun error bit.
     * |        |          |
     * |[0]     |DR        |Data Ready bit. This is used to indicate that the receiver contains at least one character in the RBR or the receiver FIFO
     */
    __IO uint32_t LSR;
   /**
     * LSR
     * ===================================================================================================
     * Offset: 0x18  UART Line Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7]     |DCD       |Data Carrier Detect.
     * |[6]     |RI        |Ring Indicator.
     * |        |          |
     * |[5]     |DSR       |Data Set Ready
     * |        |          |
     * |[4]     |CTS       |Clear to Send. This is used to indicate the current state of the modem control line cts_n.
     * |        |          |This bit is the complement of cts_n. When the Clear to Send input (cts_n) is asserted it is an indication that the modem or data set is ready to exchange data with the uart.

     * |[3]     |DDCD      |Delta Data Carrier Detect
     * |[2]     |TERI      |Trailing Edge of Ring Indicator
     * |[1]     |DDSR      |Delta Data Set Ready
     * |        |          |
     * |[0]     |DCTS      |Delta Clear to Send
     */

    __IO uint32_t MSR;
   /**
     * SR
     * ===================================================================================================
     * Offset: 0x1C  Scratchpad Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  | SR    |This register is for programmers to use as a temporary storage space. It has no defined purpose in the uart
     */

    __IO uint32_t SR;
   /**
     * LPDLL
     * ===================================================================================================
     * Offset: 0x20  Divisor Latch (Low) This register can be accessed only when the DLAB bit (LCR[7]) is set.
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   | LPDLL    |Lower 8 bits  of a 16-bit, read/write,
     * |        |          |Low Power Divisor Latch register that contains the baud rate divisor for the UART,
     * |        |          |which must give a baud rate of 115.2K. This is required for SIR Low Power (minimum pulse width) detection at the receiver.
     */
    __IO uint32_t LPDLL;
   /**
     * LPDLH
     * ===================================================================================================
     * Offset: 0x24  Divisor Latch (High) This register can be accessed only when the DLAB bit (LCR[7]) is set.
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   | LPDLH    |Upper 8-bits of a 16-bit, read/write,
     * |        |          |Low Power Divisor Latch register that contains the baud rate divisor for the UART,
     * |        |          |which must give a baud rate of 115.2K. This is required for SIR Low Power (minimum pulse width) detection at the receiver.
     */
    __IO uint32_t LPDLH;

    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVED0[21];
    /// @endcond //HIDDEN_SYMBOLS
    /**
     * USR
     * ===================================================================================================
     * Offset: 0x7C  UART Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4]     |RFF       |Receive FIFO Full. This is used to indicate that the receive FIFO is completely full.
     * |[3]     |RFNE      |Receive FIFO Not Empty. This is used to indicate that the receive FIFO contains one or more entries
     * |[2]     |TFE       |Transmit FIFO Empty. This is used to indicate that the transmit FIFO is completely empty
     * |[1]     |TFNF      |Transmit FIFO Not Full. This is used to indicate that the transmit FIFO in not full.
     * |        |          |
     */
    __IO uint32_t USR;
   /**
     * TFL
     * ===================================================================================================
     * Offset: 0x80  Transmit FIFO Level
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   | TFL      |Transmit FIFO Level. This indicates the number of data entries in the transmit FIFO.
     * |        |          |
     */

    __IO uint32_t TFL;
   /**
     * RFL
     * ===================================================================================================
     * Offset: 0x84  Receive  FIFO Level
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   | RFL      |Receive FIFO Level. This indicates the number of data entries in the receive FIFO.
     * |        |          |
     */
    __IO uint32_t RFL;

    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVED1[8];
    /// @endcond //HIDDEN_SYMBOLS
   /**
     * DMASA
     * ===================================================================================================
     * Offset: 0xA8  DMA Software Acknowledge
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     | RFL      |This register is use to perform a DMA software acknowledge if a transfer needs to be terminated due to an error condition.
     * |        |          |
     */
    __IO uint32_t DMASA;

    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVED2[5];
    /// @endcond //HIDDEN_SYMBOLS
   /**
     * DLF
     * ===================================================================================================
     * Offset: 0xC0  Receive  FIFO Level
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   | DLF      |Fractional part of divisor.
     * |        |          |
     */
    __IO uint32_t DLF;
   /**
     * RAR
     * ===================================================================================================
     * Offset: 0xC4  Receive Address Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   | DLF      |This is an address matching register during receive mode.
     * |        |          |If the 9-th bit is set in the incoming character then the remaining 8-bits will be checked against this register value.
     */
    __IO uint32_t RAR;
   /**
     * TAR
     * ===================================================================================================
     * Offset: 0xC8  Transmit Address Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   | DLF      |This is an address matching register during transmit mode. If DLS_E
     * |        |          |(LCR_EXT[0]) bit is enabled, then uart sends the 9-bit character with 9-th bit set to 1 and
     * |        |          |remaining 8-bit address will be sent from this register provided 'SEND_ADDR' (LCR_EXT[2]) bit is set to 1.
     */
    __IO uint32_t TAR;
   /**
     * TAR
     * ===================================================================================================
     * Offset: 0xC8  Transmit Address Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3]     |TRANSMIT_ |Transmit mode control bit. This bit is used to control the type of transmit mode during 9-bit data transfers.
     * |        |MODE      |1 - In this mode of operation, Transmit Holding Register (THR) is 9-bit wide. You must ensure that the THR register is written correctly for address/data.
     * |        |          |Address: 9th bit is set to 1,Data: 9th bit is set to 0.
     * |        |          |NOTE: Transmit address register (TAR) is not applicable in this mode of operation
     * |        |          |0 - In this mode of operation, Transmit Holding Register (THR) is 8-bit wide.
     * |        |          |The user needs to program the address into Transmit Address Register (TAR) and data into the THR register.
     * |        |          |SEND_ADDR bit is used as a control knob to indicate the uart on when to send the address.
     * |        |          |
     * |[2]     |SEND_ADDR |Send address control bit. This bit is used as a control knob for the user to
     * |        |          |determine when to send the address during transmit mode.
     * |        |          |1 - 9-bit character will be transmitted with 9-th bit set to 1
     * |        |          |and the remaining 8-bits will match to what is being programmed in "Transmit Address Register".
     * |        |          |0 - 9-bit character will be transmitted with 9-th bit set to 0
     * |        |          |and the remaining 8- bits will be taken from the TxFIFO which is programmed through 8-bit wide THR register.
     * |        |          |NOTE:
     * |        |          |1. This bit is auto-cleared by the hardware, after sending out the address character. User is not expected to program this bit to 0.
     * |        |          |2. This field is applicable only when DLS_E bit is set to 1 and TRANSMIT_MODE is set to 0.
     * |        |          |
     * |[1]     |ADDR_MATCH|Address Match Mode. This bit is used to enable the address match feature during receive
     * |        |          |1 - Address match mode; uart will wait until the incoming character with 9-th bit set to 1.
     * |        |          |And, further checks to see if the address matches with what is programmed in "Receive Address Match Register".
     * |        |          |If match is found, then sub-sequent characters will be treated as valid data and uart starts receiving data.
     * |        |          |0 - Normal mode; uart will start to receive the data and 9-bit character will be formed
     * |        |          |and written into the receive RxFIFO. User is responsible to read the data and differentiate b/n address and data.
     * |        |          |NOTE: This field is applicable only when DLS_E is set to 1.
     * |        |          |
     * |[0]     | DLS_E    |Extension for DLS. This bit is used to enable 9-bit data for transmit and receive transfers.
     * |        |          |1 - 9 bits per character
     * |        |          |0 - Number of data bits selected by DLS
     */

    __IO uint32_t LCR_EXT;

} UART_T;

/**
    @addtogroup UART_CONST UART Bit Field Definition
    Constant Definitions for UART Controller
@{ */

#define UART_RBR_MSB9_Pos                (8)
#define UART_RBR_MSB9_Msk                (0x1ul << UART_RBR_MSB9_Pos)

#define UART_RBR_DAT_Pos                 (0)
#define UART_RBR_DAT_Msk                 (0xfful << UART_RBR_DAT_Pos)

#define UART_THR_MSB9_Pos                (8)
#define UART_THR_MSB9_Msk                (0x1ul << UART_THR_MSB9_Pos)

#define UART_THR_DAT_Pos                 (0)
#define UART_THR_DAT_Msk                 (0xfful << UART_THR_DAT_Pos)

#define UART_DLH_Pos                     (0)
#define UART_DLH_Msk                     (0xfful << UART_DLH_Pos)

#define UART_DLL_Pos                     (0)
#define UART_DLL_Msk                     (0xfful << UART_DLL_Pos)
//Interrupt enable
#define UART_IER_EPTI_Pos                (7)
#define UART_IER_EPTI_Msk                (0x1ul << UART_IER_EPTI_Pos)

#define UART_IER_EMSI_Pos                (3)
#define UART_IER_EMSI_Msk                (0x1ul << UART_IER_EMSI_Pos)
#define UART_IER_ERLSI_Pos               (2)
#define UART_IER_ERLSI_Msk               (0x1ul << UART_IER_ERLSI_Pos)
#define UART_IER_ETHREI_Pos              (1)
#define UART_IER_ETHREI_Msk              (0x1ul << UART_IER_ETHREI_Pos)
#define UART_IER_ERDAI_Pos               (0)
#define UART_IER_ERDAI_Msk               (0x1ul << UART_IER_ERDAI_Pos)

#define UART_IER_ALL_IRQ_Pos             (0)
#define UART_IER_ALL_IRQ_Msk             (0xful << UART_IER_ALL_IRQ_Pos)
//Interrput ID
#define UART_IIR_FIFOSE_Pos              (6)
#define UART_IIR_FIFOSE_Msk              (0x3ul << UART_IIR_FIFOSE_Pos)

#define UART_IIR_IID_Pos                 (0)
#define UART_IIR_IID_Msk                 (0xful << UART_IIR_IID_Pos)

//FIFO Control Register
#define UART_FCR_RT_Pos                 (6)
#define UART_FCR_RT_Msk                 (0x3ul << UART_FCR_RT_Pos)
#define UART_FCR_TET_Pos                (4)
#define UART_FCR_TET_Msk                (0x3ul << UART_FCR_TET_Pos)
#define UART_FCR_XFIFOR_Pos             (2)
#define UART_FCR_XFIFOR_Msk             (0x1ul << UART_FCR_XFIFOR_Pos)
#define UART_FCR_RFIFOR_Pos             (1)
#define UART_FCR_RFIFOR_Msk             (0x1ul << UART_FCR_RFIFOR_Pos)
#define UART_FCR_FIFOE_Pos              (0)
#define UART_FCR_FIFOE_Msk              (0x1ul << UART_FCR_FIFOE_Pos)
#define UART_FCR_DMAM_Pos               (3)
#define UART_FCR_DMAM_Msk               (0x1ul << UART_FCR_DMAM_Pos)

//Line Control Register
#define UART_LCR_DLAB_Pos              (7)
#define UART_LCR_DLAB_Msk              (0x1ul << UART_LCR_DLAB_Pos)
#define UART_LCR_BC_Pos                (6)
#define UART_LCR_BC_Msk                (0x1ul << UART_LCR_BC_Pos)
#define UART_LCR_SP_Pos                (5)
#define UART_LCR_SP_Msk                (0x1ul << UART_LCR_SP_Pos)
#define UART_LCR_EPS_Pos               (4)
#define UART_LCR_EPS_Msk               (0x1ul << UART_LCR_EPS_Pos)
#define UART_LCR_PEN_Pos               (3)
#define UART_LCR_PEN_Msk               (0x1ul << UART_LCR_PEN_Pos)
#define UART_LCR_STOP_Pos              (2)
#define UART_LCR_STOP_Msk              (0x1ul << UART_LCR_STOP_Pos)
#define UART_LCR_DLS_Pos               (0)
#define UART_LCR_DLS_Msk               (0x3ul << UART_LCR_STOP_Pos)

//Modem Control Register
#define UART_MCR_SIRE_Pos               (6)
#define UART_MCR_SIRE_Msk               (0x1ul << UART_MCR_SIRE_Pos)
#define UART_MCR_AFCE_Pos               (5)
#define UART_MCR_AFCE_Msk               (0x1ul << UART_MCR_AFCE_Pos)
#define UART_MCR_LB_Pos                 (4)
#define UART_MCR_LB_Msk                 (0x1ul << UART_MCR_LB_Pos)
#define UART_MCR_OUT2_Pos               (3)
#define UART_MCR_OUT2_Msk               (0x1ul << UART_MCR_OUT2_Pos)
#define UART_MCR_OUT1_Pos               (2)
#define UART_MCR_OUT1_Msk               (0x1ul << UART_MCR_OUT1_Pos)
#define UART_MCR_RTS_Pos                (1)
#define UART_MCR_RTS_Msk                (0x1ul << UART_MCR_RTS_Pos)
#define UART_MCR_DTR_Pos                (0)
#define UART_MCR_DTR_Msk                (0x1ul << UART_MCR_DTR_Pos)

//Line Status Register
#define UART_LSR_ADDR_RCVD_Pos          (8)
#define UART_LSR_ADDR_RCVD_Msk          (0x1ul << UART_LSR_ADDR_RCVD_Pos)
#define UART_LSR_RFE_Pos                (7)
#define UART_LSR_RFE_Msk                (0x1ul << UART_LSR_RFE_Pos)
#define UART_LSR_TEMT_Pos               (6)
#define UART_LSR_TEMT_Msk               (0x1ul << UART_LSR_TEMT_Pos)
#define UART_LSR_THRE_Pos               (5)
#define UART_LSR_THRE_Msk               (0x1ul << UART_LSR_THRE_Pos)
#define UART_LSR_BI_Pos                 (4)
#define UART_LSR_BI_Msk                 (0x1ul << UART_LSR_BI_Pos)
#define UART_LSR_FE_Pos                 (3)
#define UART_LSR_FE_Msk                 (0x1ul << UART_LSR_FE_Pos)
#define UART_LSR_PE_Pos                 (2)
#define UART_LSR_PE_Msk                 (0x1ul << UART_LSR_PE_Pos)
#define UART_LSR_OE_Pos                 (1)
#define UART_LSR_OE_Msk                 (0x1ul << UART_LSR_OE_Pos)
#define UART_LSR_DR_Pos                 (0)
#define UART_LSR_DR_Msk                 (0x1ul << UART_LSR_DR_Pos)

#define UART_LSR_LINE_STATUS_Pos        (0)
#define UART_LSR_LINE_STATUS_Msk        (0xFFul << UART_LSR_LINE_STATUS_Pos)


//Modem Status Register
#define UART_MSR_DCD_Pos                 (7)
#define UART_MSR_DCD_Msk                 (0x1ul << UART_MSR_DCD_Pos)
#define UART_MSR_RI_Pos                  (6)
#define UART_MSR_RI_Msk                  (0x1ul << UART_MSR_RI_Pos)
#define UART_MSR_DSR_Pos                 (5)
#define UART_MSR_DSR_Msk                 (0x1ul << UART_MSR_DSR_Pos)
#define UART_MSR_CTS_Pos                 (4)
#define UART_MSR_CTS_Msk                 (0x1ul << UART_MSR_CTS_Pos)
#define UART_MSR_DDCD_Pos                (3)
#define UART_MSR_DDCD_Msk                (0x1ul << UART_MSR_DDCD_Pos)
#define UART_MSR_TERI_Pos                (2)
#define UART_MSR_TERI_Msk                (0x1ul << UART_MSR_TERI_Pos)
#define UART_MSR_DDSR_Pos                (1)
#define UART_MSR_DDSR_Msk                (0x1ul << UART_MSR_DDSR_Pos)
#define UART_MSR_DCTS_Pos                (0)
#define UART_MSR_DCTS_Msk                (0x1ul << UART_MSR_DCTS_Pos)

//Scratchpad register
#define UART_SR_Pos                      (0)
#define UART_SR_Msk                      (0xFFFFFFFFul << UART_SR_Pos)

//Low Power Divisor Latch Low Register
#define UART_LPDLL_Pos                   (0)
#define UART_LPDLL_Msk                   (0xFFul << UART_LPDLL_Pos)

//Low Power Divisor Latch High Register
#define UART_LPDLH_Pos                   (0)
#define UART_LPDLH_Msk                   (0xFFul << UART_LPDLH_Pos)

//Uart Status Register
#define UART_USR_RFF_Pos                 (4)
#define UART_USR_RFF_Msk                 (0x1ul << UART_USR_RFF_Pos)
#define UART_USR_RFNE_Pos                (3)
#define UART_USR_RFNE_Msk                (0x1ul << UART_USR_RFNE_Pos)
#define UART_USR_TFE_Pos                 (2)
#define UART_USR_TFE_Msk                 (0x1ul << UART_USR_TFE_Pos)
#define UART_USR_TFNF_Pos                (1)
#define UART_USR_TFNF_Msk                (0x1ul << UART_USR_TFNF_Pos)

//Transmit FIFO Level
#define UART_TFL_Pos                     (0)
#define UART_TFL_Msk                     (0x1Ful << UART_TFL_Pos)

//Receive FIFO Level
#define UART_RFL_Pos                     (0)
#define UART_RFL_Msk                     (0x1Ful << UART_RFL_Pos)

//DMA software Acknowledge
#define UART_DMASA_Pos                   (0)
#define UART_DMASA_Msk                   (0x1ul << UART_DMASA_Pos)

//Divisor Latch Fraction Register
#define UART_DLF_Pos                     (0)
#define UART_DLF_Msk                     (0xFul << UART_DLF_Pos)

//Receive Address Register
#define UART_RAR_Pos                     (0)
#define UART_RAR_Msk                     (0xFFul << UART_RAR_Pos)

//Transmit Address Register
#define UART_TAR_Pos                     (0)
#define UART_TAR_Msk                     (0xFFul << UART_TAR_Pos)


// Line Extented Control Register

#define UART_LCR_EXT_TRANSMIT_MODE_Pos      (3)
#define UART_LCR_EXT_TRANSMIT_MODE_Msk      (0x1ul << UART_LCR_EXT_TRANSMIT_MODE_Pos)
#define UART_LCR_EXT_SEND_ADDR_Pos          (2)
#define UART_LCR_EXT_SEND_ADDR_Msk          (0x1ul << UART_LCR_EXT_SEND_ADDR_Pos)
#define UART_LCR_EXT_ADDR_MATCH_Pos         (1)
#define UART_LCR_EXT_SEND_MATCH_ADDR_Msk    (0x1ul << UART_LCR_EXT_ADDR_MATCH_Pos)
#define UART_LCR_EXT_DLS_E_Pos              (0)
#define UART_LCR_EXT_DLS_E_Msk              (0x1ul << UART_LCR_EXT_DLS_E_Pos)

/**@}*/ /* UART_CONST */
/**@}*/ /* end of UART register group */

/****
 * DESCRIPTION
 *  This is the structure used for accessing the dmac register
 *  portmap.
 */
typedef struct {

  /* Channel registers                                    */
  /* The offset address for each of the channel registers */
  /*  is shown for channel 0. For other channel numbers   */
  /*  use the following equation.                         */
  /*                                                      */
  /*    offset = (channel_num * 0x058) + channel_0 offset */
  /*                                                      */
  struct {
      __IO uint32_t SAR_L;     /* Source Address Reg      (0x000) */
      __IO uint32_t SAR_H;
      __IO uint32_t DAR_L;     /* Destination Address Reg (0x008) */
      __IO uint32_t DAR_H;
      #if 0
      __IO uint32_t LLP_L;     /* Linked List Pointer Reg (0x010) */
      __IO uint32_t LLP_H;
      #else
      __IO uint32_t RESERVED0[2];

      #endif
      __IO uint32_t CTL_L;     /* Control Reg             (0x018) */
      __IO uint32_t CTL_H;
      #if 0
      __IO uint32_t sstat_l;   /* Source Status Reg       (0x020) */
      __IO uint32_t sstat_h;
      __IO uint32_t dstat_l;   /* Destination Status Reg  (0x028) */
      __IO uint32_t dstat_h;
      __IO uint32_t sstatar_l; /* Source Status Addr Reg  (0x030) */
      __IO uint32_t sstatar_h;
      __IO uint32_t dstatar_l; /* Dest Status Addr Reg    (0x038) */
      __IO uint32_t dstatar_h;
      #else
      __IO uint32_t RESERVED1[8];
      #endif
      __IO uint32_t CFG_L;     /* Configuration Reg       (0x040) */
      __IO uint32_t CFG_H;
      #if 0
      __IO uint32_t sgr_l;     /* Source Gather Reg       (0x048) */
      __IO uint32_t sgr_h;
      __IO uint32_t dsr_l;     /* Destination Scatter Reg (0x050) */
      __IO uint32_t dsr_h;
      #else
      __IO uint32_t RESERVED2[4];
      #endif
  } CH[8];

  /* Interrupt Raw Status Registers */
  __IO uint32_t RAW_TFR_L;     /* Raw Status for IntTfr   (0x2c0) */
  __IO uint32_t RAW_TFR_H;
  __IO uint32_t RAW_BLOCK_L;   /* Raw Status for IntBlock (0x2c8) */
  __IO uint32_t RAW_BLOCK_H;
  __IO uint32_t RAW_SRCTRAN_L; /* Raw Status IntSrcTran   (0x2d0) */
  __IO uint32_t RAW_SRCTRAN_H;
  __IO uint32_t RAW_DSTTRAN_L; /* Raw Status IntDstTran   (0x2d8) */
  __IO uint32_t RAW_DSTTRAN_H;
  __IO uint32_t RAW_ERR_L;     /* Raw Status for IntErr   (0x2e0) */
  __IO uint32_t RAW_ERR_H;

  /* Interrupt Status Registers */
  __IO uint32_t STATUS_TFR_L;    /* Status for IntTfr     (0x2e8) */
  __IO uint32_t STATUS_TFR_H;
  __IO uint32_t STATUS_BLOCK_L;  /* Status for IntBlock   (0x2f0) */
  __IO uint32_t STATUS_BLOCK_H;
  __IO uint32_t STATUS_SRCTRAN_L;/* Status for IntSrcTran (0x2f8) */
  __IO uint32_t STATUS_SRCTRAN_H;
  __IO uint32_t STATUS_DSTTRAN_L;/* Status for IntDstTran (0x300) */
  __IO uint32_t STATUS_DSTTRAN_H;
  __IO uint32_t STATUS_ERR_L;    /* Status for IntErr     (0x308) */
  __IO uint32_t STATUS_ERR_H;

  /* Interrupt Mask Registers */
  __IO uint32_t MSK_TFR_L;      /* Mask for IntTfr       (0x310) */
  __IO uint32_t MSK_TFR_H;
  __IO uint32_t MSK_BLOCK_L;    /* Mask for IntBlock     (0x318) */
  __IO uint32_t MSK_BLOCK_H;
  __IO uint32_t MSK_SRCTRAN_L;  /* Mask for IntSrcTran   (0x320) */
  __IO uint32_t MSK_SRCTRAN_H;
  __IO uint32_t MSK_DSTTRAN_L;  /* Mask for IntDstTran   (0x328) */
  __IO uint32_t MSK_DSTTRAN_H;
  __IO uint32_t MSK_ERR_L;      /* Mask for IntErr       (0x330) */
  __IO uint32_t MSK_ERR_H;

  /* Interrupt Clear Registers */
  __IO uint32_t CLEAR_TFR_L;     /* Clear for IntTfr      (0x338) */
  __IO uint32_t CLEAR_TFR_H;
  __IO uint32_t CLEAR_BLOCK_L;   /* Clear for IntBlock    (0x340) */
  __IO uint32_t CLEAR_BLOCK_H;
  __IO uint32_t CLEAR_SRCTRAN_L; /* Clear for IntSrcTran  (0x348) */
  __IO uint32_t CLEAR_SRCTRAN_H;
  __IO uint32_t CLEAR_DSTTRAN_L; /* Clear for IntDstTran  (0x350) */
  __IO uint32_t CLEAR_DSTTRAN_H;
  __IO uint32_t CLEAR_ERR_L;     /* Clear for IntErr      (0x358) */
  __IO uint32_t CLEAR_ERR_H;
  __IO uint32_t STATUS_INT_L;    /* Combined Intr Status  (0x360) */
  __IO uint32_t STATUS_INT_H;
#if 0
  /* Software Handshaking Registers */
  __IO uint32_t req_src_reg_l; /* Src Sw Transaction Req  (0x368) */
  __IO uint32_t req_src_reg_h;
  __IO uint32_t req_dst_reg_l; /* Dest Sw Transaction Req (0x370) */
  __IO uint32_t req_dst_reg_h;
  __IO uint32_t sgl_rq_src_reg_l; /* Sgl Src Transac Req  (0x378) */
  __IO uint32_t sgl_rq_src_reg_h;
  __IO uint32_t sgl_rq_dst_reg_l; /* Sgl Dest Transac Req (0x380) */
  __IO uint32_t sgl_rq_dst_reg_h;
  __IO uint32_t lst_src_reg_l;   /* Last Src Transac Req  (0x388) */
  __IO uint32_t lst_src_reg_h;
  __IO uint32_t lst_dst_reg_l;   /* Last Dest Transac Req (0x390) */
  __IO uint32_t lst_dst_reg_h;

#else
    __IO uint32_t RESERVED3[12];
#endif
  __IO uint32_t DMA_CFG_REG_L; /* Configuration Register  (0x398) */
  __IO uint32_t DMA_CFG_REG_H;

  __IO uint32_t CH_EN_REG_L;   /* Channel Enable Register (0x3a0) */
  __IO uint32_t CH_EN_REG_H;

#if 0
  /* Misc Registers */
  __IO uint32_t dma_cfg_reg_l; /* Configuration Register  (0x398) */
  __IO uint32_t dma_cfg_reg_h;
  __IO uint32_t ch_en_reg_l;   /* Channel Enable Register (0x3a0) */
  __IO uint32_t ch_en_reg_h;
  __IO uint32_t dma_id_reg_l;    /* ID Register           (0x3a8) */
  __IO uint32_t dma_id_reg_h;
  __IO uint32_t dma_test_reg_l;  /* Test Register         (0x3b0) */
  __IO uint32_t dma_test_reg_h;
  __IO uint32_t old_version_id_l;/* legacy support        (0x3b8) */
  __IO uint32_t old_version_id_h;
  __IO uint32_t reserved_low;    /* reserved              (0x3c0) */
  __IO uint32_t reserved_high;
  __IO uint32_t dma_comp_params_6_l;/* hardware params    (0x3c8) */
  __IO uint32_t dma_comp_params_6_h;
  __IO uint32_t dma_comp_params_5_l;/* hardware params    (0x3d0) */
  __IO uint32_t dma_comp_params_5_h;
  __IO uint32_t dma_comp_params_4_l;/* hardware params    (0x3d8) */
  __IO uint32_t dma_comp_params_4_h;
  __IO uint32_t dma_comp_params_3_l;/* hardware params    (0x3e0) */
  __IO uint32_t dma_comp_params_3_h;
  __IO uint32_t dma_comp_params_2_l;/* hardware params    (0x3e8) */
  __IO uint32_t dma_comp_params_2_h;
  __IO uint32_t dma_comp_params_1_l;/* hardware params    (0x3f0) */
  __IO uint32_t dma_comp_params_1_h;
  __IO uint32_t dma_version_id_l;/* Version ID Register   (0x3f8) */
  __IO uint32_t dma_version_id_h;
#endif
}DMA_T;
/*****/

#define DMAC_DMACFGREG_L_DMA_EN_Pos         (0)
#define DMAC_DMACFGREG_L_DMA_EN_Msk         (0x1ul << DMAC_DMACFGREG_L_DMA_EN_Pos)
//channel
#define DMAC_CHENREG_L_CH_EN_Pos(ch)        (ch)
#define DMAC_CHENREG_L_CH_EN_Msk(ch)        (0x1ul << DMAC_CHENREG_L_CH_EN_Pos(ch))

#define DMAC_CHENREG_L_CH_EN_ALL_Pos        (0)
#define DMAC_CHENREG_L_CH_EN_ALL_Msk        (0xfful << DMAC_CHENREG_L_CH_EN_ALL_Pos)

#define DMAC_CHENREG_L_CH_EN_WE_Pos(ch)     ((ch)+8)
#define DMAC_CHENREG_L_CH_EN_WE_Msk(ch)     (0x1ul << DMAC_CHENREG_L_CH_EN_WE_Pos(ch))

#define DMAC_CHENREG_L_CH_EN_WE_ALL_Pos     (8)
#define DMAC_CHENREG_L_CH_EN_WE_ALL_Msk     (0xfful << DMAC_CHENREG_L_CH_EN_WE_ALL_Pos)

#define DMAC_SAR_L_SAR_Pos              (0)
#define DMAC_SAR_L_SAR_Msk              (0xfffffffful << DMAC_SAR_L_SAR_Pos)

#define DMAC_DAR_L_DAR_Pos              (0)
#define DMAC_DAR_L_DAR_Msk              (0xfffffffful << DMAC_DAR_L_DAR_Pos)
#if 0
#define DMAC_LLP_L_LMS              ((uint32_t)    0)
#define DMAC_LLP_L_LMS              ((uint32_t)    2)
#define DMAC_LLP_L_LOC              ((uint32_t)    2)
#define DMAC_LLP_L_LOC              ((uint32_t)   30)
#endif
#define DMAC_CTL_L_INT_EN_Pos           (0)
#define DMAC_CTL_L_INT_EN_Msk           (0x1ul << DMAC_CTL_L_INT_EN_Pos)

#define DMAC_CTL_L_DST_TR_WIDTH_Pos     (1)
#define DMAC_CTL_L_DST_TR_WIDTH_Msk     (0x3ul << DMAC_CTL_L_DST_TR_WIDTH_Pos)

#define DMAC_CTL_L_SRC_TR_WIDTH_Pos     (4)
#define DMAC_CTL_L_SRC_TR_WIDTH_Msk     (0x3ul << DMAC_CTL_L_SRC_TR_WIDTH_Pos)

#define DMAC_CTL_L_DINC_Pos             (7)
#define DMAC_CTL_L_DINC_Msk             (0x3ul << DMAC_CTL_L_DINC_Pos)

#define DMAC_CTL_L_SINC_Pos             (9)
#define DMAC_CTL_L_SINC_Msk             (0x3ul << DMAC_CTL_L_SINC_Pos)

#define DMAC_CTL_L_DEST_MSIZE_Pos       (11)
#define DMAC_CTL_L_DEST_MSIZE_Msk       (0x7ul << DMAC_CTL_L_DEST_MSIZE_Pos)

#define DMAC_CTL_L_SRC_MSIZE_Pos        (14)
#define DMAC_CTL_L_SRC_MSIZE_Msk        (0x7ul << DMAC_CTL_L_SRC_MSIZE_Pos)
#if 0
#define DMAC_CTL_L_SRC_GATHER_EN_Pos    (17)
#define DMAC_CTL_L_SRC_GATHER_EN_Msk    (1)
#define DMAC_CTL_L_DST_SCATTER_EN_Pos   (18)
#define DMAC_CTL_L_DST_SCATTER_EN_Msk   (1)
#endif
#define DMAC_CTL_L_TT_FC_Pos            (20)
#define DMAC_CTL_L_TT_FC_Msk            (0x7ul << DMAC_CTL_L_TT_FC_Pos)
#if 0
#define DMAC_CTL_L_DMS_Pos              (23)
#define DMAC_CTL_L_DMS_Msk              (0x3ul << DMAC_CTL_L_DMS_Pos)
#define DMAC_CTL_L_SMS_Pos              (25)
#define DMAC_CTL_L_SMS_Msk              (2)
#define DMAC_CTL_L_LLP_DST_EN_Pos       (27)
#define DMAC_CTL_L_LLP_DST_EN_Msk       (1)
#define DMAC_CTL_L_LLP_SRC_EN_Pos       (28)
#define DMAC_CTL_L_LLP_SRC_EN_Msk       (1)
#endif
#define DMAC_CTL_H_BLOCK_TS_Pos         (0)
#define DMAC_CTL_H_BLOCK_TS_Msk         (0xffful << DMAC_CTL_H_BLOCK_TS_Pos)

#define DMAC_CTL_H_DONE_Pos             (12)
#define DMAC_CTL_H_DONE_Msk             (0x1ul << DMAC_CTL_H_DONE_Pos)
#if 0
#define DMAC_SSTAT_L_SSTAT          ((uint32_t)    0)
#define DMAC_SSTAT_L_SSTAT          ((uint32_t)   32)

#define DMAC_DSTAT_L_DSTAT          ((uint32_t)    0)
#define DMAC_DSTAT_L_DSTAT          ((uint32_t)   32)

#define DMAC_SSTATAR_L_SSTATAR      ((uint32_t)    0)
#define DMAC_SSTATAR_L_SSTATAR      ((uint32_t)   32)

#define DMAC_DSTATAR_L_DSTATAR      ((uint32_t)    0)
#define DMAC_DSTATAR_L_DSTATAR      ((uint32_t)   32)
#endif
#define DMAC_CFG_L_CH_PRIOR_Pos         (5)
#define DMAC_CFG_L_CH_PRIOR_Msk         (0x7 << DMAC_CFG_L_CH_PRIOR_Pos)

#define DMAC_CFG_L_CH_SUSP_Pos          (8)
#define DMAC_CFG_L_CH_SUSP_Msk          (0x1ul << DMAC_CFG_L_CH_SUSP_Pos)

#define DMAC_CFG_L_FIFO_EMPTY_Pos       (9)
#define DMAC_CFG_L_FIFO_EMPTY_Msk       (0x1ul << DMAC_CFG_L_FIFO_EMPTY_Pos)

#define DMAC_CFG_L_HS_SEL_DST_Pos       (10)
#define DMAC_CFG_L_HS_SEL_DST_Msk       (0x1ul << DMAC_CFG_L_HS_SEL_DST_Pos)

#define DMAC_CFG_L_HS_SEL_SRC_Pos       (11)
#define DMAC_CFG_L_HS_SEL_SRC_Msk       (0x1ul << DMAC_CFG_L_HS_SEL_SRC_Pos)
#if 0
#define DMAC_CFG_L_LOCK_CH_L_Pos        12)
#define DMAC_CFG_L_LOCK_CH_L_Msk        (2)
#define DMAC_CFG_L_LOCK_B_L_Pos         (14)
#define DMAC_CFG_L_LOCK_B_L_Msk         (2)
#define DMAC_CFG_L_LOCK_CH_Pos          (16)
#define DMAC_CFG_L_LOCK_CH_Msk          (1)
#define DMAC_CFG_L_LOCK_B_Pos           (17)
#define DMAC_CFG_L_LOCK_B_Msk           (1)
#endif

#define DMAC_CFG_L_DST_HS_POL_Pos       (18)
#define DMAC_CFG_L_DST_HS_POL_Msk       (0x1ul << DMAC_CFG_L_DST_HS_POL_Pos)

#define DMAC_CFG_L_SRC_HS_POL_Pos       (19)
#define DMAC_CFG_L_SRC_HS_POL_Msk       (0x1ul << DMAC_CFG_L_SRC_HS_POL_Pos)

#if 0
#define DMAC_CFG_L_MAX_ABRST_Pos        (20)
#define DMAC_CFG_L_MAX_ABRST_Msk        (10)
#define DMAC_CFG_L_RELOAD_SRC_Pos       (30)
#define DMAC_CFG_L_RELOAD_SRC_Msk       (1)
#define DMAC_CFG_L_RELOAD_DST_Pos       (31)
#define DMAC_CFG_L_RELOAD_DST_Msk       (1)
#endif

#define DMAC_CFG_H_FCMODE_Pos           (0)
#define DMAC_CFG_H_FCMODE_Msk           (0x1ul << DMAC_CFG_H_FCMODE_Pos)

#define DMAC_CFG_H_FIFO_MODE_Pos        (1)
#define DMAC_CFG_H_FIFO_MODE_Msk        (0x1ul << DMAC_CFG_H_FIFO_MODE_Pos)

#define DMAC_CFG_H_PROTCTL_Pos          (2)
#define DMAC_CFG_H_PROTCTL_Msk          (0x7ul << DMAC_CFG_H_PROTCTL_Pos)
#if 0
#define DMAC_CFG_H_DS_UPD_EN_Pos        (5)
#define DMAC_CFG_H_DS_UPD_EN_Msk        (1)
#define DMAC_CFG_H_SS_UPD_EN_Pos        (6)
#define DMAC_CFG_H_SS_UPD_EN_Msk        (1)
#endif
#define DMAC_CFG_H_SRC_PER_Pos          (7)
#define DMAC_CFG_H_SRC_PER_Msk          (0xful << DMAC_CFG_H_SRC_PER_Pos)

#define DMAC_CFG_H_DEST_PER_Pos         (11)
#define DMAC_CFG_H_DEST_PER_Msk         (0xful << DMAC_CFG_H_DEST_PER_Pos)
#if 0
#define DMAC_SGR_L_SGI              ((uint32_t)    0)
#define DMAC_SGR_L_SGI              ((uint32_t)   20)
#define DMAC_SGR_L_SGC              ((uint32_t)   20)
#define DMAC_SGR_L_SGC              ((uint32_t)   12)

#define DMAC_DSR_L_DSI              ((uint32_t)    0)
#define DMAC_DSR_L_DSI              ((uint32_t)   20)
#define DMAC_DSR_L_DSC              ((uint32_t)   20)
#define DMAC_DSR_L_DSC              ((uint32_t)   12)
#endif
#define DMAC_INT_RAW_STAT_CLR_Pos(ch)   (ch)
#define DMAC_INT_RAW_STAT_CLR_Msk(ch)   (0x1ul << DMAC_INT_RAW_STAT_CLR_Pos(ch))

#define DMAC_INT_RAW_STAT_CLR_ALL_Pos   (0)
#define DMAC_INT_RAW_STAT_CLR_ALL_Msk   (0xfful << DMAC_INT_RAW_STAT_CLR_ALL_Pos)

#define DMAC_INT_MASK_L_Pos(ch)         (ch)
#define DMAC_INT_MASK_L_Msk(ch)         (0x1ul << DMAC_INT_MASK_L_Pos(ch))

#define DMAC_INT_MASK_L_ALL_Pos         (0)
#define DMAC_INT_MASK_L_ALL_Msk         (0xfful << DMAC_INT_MASK_L_ALL_Pos)

#define DMAC_INT_MASK_L_WE_Pos(ch)      ((ch)+8)
#define DMAC_INT_MASK_L_WE_Msk(ch)      (0x1ul << DMAC_INT_MASK_L_WE_Pos(ch))

#define DMAC_INT_MASK_L_WE_ALL_Pos      (8)
#define DMAC_INT_MASK_L_WE_ALL_Msk      (0xfful << DMAC_INT_MASK_L_WE_ALL_Pos)

#define DMAC_STATUSINT_L_TFR_Pos        (0)
#define DMAC_STATUSINT_L_TFR_Msk        (0x1ul << DMAC_STATUSINT_L_TFR_Pos)

#define DMAC_STATUSINT_L_BLOCK_Pos      (1)
#define DMAC_STATUSINT_L_BLOCK_Msk      (0x1ul << DMAC_STATUSINT_L_BLOCK_Pos)

#define DMAC_STATUSINT_L_SRCTRAN_Pos    (2)
#define DMAC_STATUSINT_L_SRCTRAN_Msk    (0x1ul << DMAC_STATUSINT_L_SRCTRAN_Pos)

#define DMAC_STATUSINT_L_DSTTRAN_Pos    (3)
#define DMAC_STATUSINT_L_DSTTRAN_Msk    (0x1ul << DMAC_STATUSINT_L_DSTTRAN_Pos)

#define DMAC_STATUSINT_L_ERR_Pos        (4)
#define DMAC_STATUSINT_L_ERR_Msk        (0x1ul << DMAC_STATUSINT_L_ERR_Pos)



/*---------------------- Watch Dog Timer Controller -------------------------*/
/**
    @addtogroup WDT Watch Dog Timer Controller(WDT)
    Memory Mapped Structure for WDT Controller
@{ */

typedef struct {


    /**
     * CTL
     * ===================================================================================================
     * Offset: 0x00  WDT Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RSTCNT    |Reset WDT Up Counter (Write Protect)
     * |        |          |0 = No effect.
     * |        |          |1 = Reset the internal 18-bit WDT up counter value.
     * |        |          |Note1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note2: This bit will be automatically cleared by hardware.
     * |[1]     |RSTEN     |WDT Time-out Reset Enable Bit (Write Protect)
     * |        |          |Setting this bit will enable the WDT time-out reset function If the WDT up counter value has not been cleared after the specific WDT reset delay period expires.
     * |        |          |0 = WDT time-out reset function Disabled.
     * |        |          |1 = WDT time-out reset function Enabled.
     * |        |          |Note: This bit is write-protected. Refer to the SYS_REGLCTL register.
     * |[2]     |RSTF      |WDT Time-out Reset Flag
     * |        |          |This bit indicates the system has been reset by WDT time-out reset or not.
     * |        |          |0 = WDT time-out reset did not occur.
     * |        |          |1 = WDT time-out reset occurred.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * |[3]     |IF        |WDT Time-out Interrupt Flag
     * |        |          |This bit will be set to 1 while WDT up counter value reaches the selected WDT time-out interval
     * |        |          |0 = WDT time-out interrupt did not occur.
     * |        |          |1 = WDT time-out interrupt occurred.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * |[4]     |WKEN      |WDT Time-out Wake-up Function Control (Write Protect)
     * |        |          |If this bit is set to 1, while WDT time-out interrupt flag IF (WDT_CTL[3]) is generated to 1 and interrupt enable bit INTEN (WDT_CTL[6]) is enabled, the WDT time-out interrupt signal will generate a wake-up trigger event to chip.
     * |        |          |0 = Wake-up trigger event Disabled if WDT time-out interrupt signal generated.
     * |        |          |1 = Wake-up trigger event Enabled if WDT time-out interrupt signal generated.
     * |        |          |Note1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note2: Chip can be woken-up by WDT time-out interrupt signal generated only if WDT clock source is selected to LIRC or LXT.
     * |[5]     |WKF       |WDT Time-out Wake-up Flag (Write Protect)
     * |        |          |This bit indicates the interrupt wake-up flag status of WDT
     * |        |          |0 = WDT does not cause chip wake-up.
     * |        |          |1 = Chip wake-up from Idle or Power-down mode if WDT time-out interrupt signal generated.
     * |        |          |Note1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note2: This bit is cleared by writing 1 to it.
     * |[6]     |INTEN     |WDT Time-out Interrupt Enable Bit (Write Protect)
     * |        |          |If this bit is enabled, the WDT time-out interrupt signal is generated and inform to CPU.
     * |        |          |0 = WDT time-out interrupt Disabled.
     * |        |          |1 = WDT time-out interrupt Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[7]     |WDTEN     |WDT Enable Bit (Write Protect)
     * |        |          |0 = WDT Disabled (This action will reset the internal up counter value).
     * |        |          |1 = WDT Enabled.
     * |        |          |Note1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note2: If CWDTEN[2:0] (combined by Config0[31] and Config0[4:3]) bits is not configure to 111, this bit is forced as 1 and user cannot change this bit to 0.
     * |[10:8]  |TOUTSEL   |WDT Time-out Interval Selection (Write Protect)
     * |        |          |These three bits select the time-out interval period for the WDT.
     * |        |          |000 = 2^4 * WDT_CLK.
     * |        |          |001 = 2^6 * WDT_CLK.
     * |        |          |010 = 2^8 * WDT_CLK.
     * |        |          |011 = 2^10 * WDT_CLK.
     * |        |          |100 = 2^12 * WDT_CLK.
     * |        |          |101 = 2^14 * WDT_CLK.
     * |        |          |110 = 2^16 * WDT_CLK.
     * |        |          |111 = 2^18 * WDT_CLK.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[16]     |TOF      |WDT Time-out Flag
     * |        |          |This bit will be set to 1 while WDT up counter value reaches the selected WDT time-out interval
     * |        |          |0 = WDT time-out did not occur.
     * |        |          |1 = WDT time-out occurred.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * |[31]    |ICEDEBUG  |ICE Debug Mode Acknowledge Disable Bit (Write Protect)
     * |        |          |0 = ICE debug mode acknowledgement affects WDT counting.
     * |        |          |WDT up counter will be held while CPU is held by ICE.
     * |        |          |1 = ICE debug mode acknowledgement Disabled.
     * |        |          |WDT up counter will keep going no matter CPU is held by ICE or not.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
    */
    __IO uint32_t CTL;

    /**
     * ALTCTL
     * ===================================================================================================
     * Offset: 0x04  WDT Alternative Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |RSTDSEL   |WDT Reset Delay Selection (Write Protect)
     * |        |          |When WDT time-out happened, user has a time named WDT Reset Delay Period to clear WDT counter by setting RSTCNT (WDT_CTL[0]) to prevent WDT time-out reset happened.
     * |        |          |User can select a suitable setting of RSTDSEL for different WDT Reset Delay Period.
     * |        |          |00 = WDT Reset Delay Period is 1025 * WDT_CLK.
     * |        |          |01 = WDT Reset Delay Period is 129 * WDT_CLK.
     * |        |          |10 = WDT Reset Delay Period is 17 * WDT_CLK.
     * |        |          |11 = WDT Reset Delay Period is 2 * WDT_CLK.
     * |        |          |Note1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note2: This register will be reset to 0 if WDT time-out reset happened.
    */
    __IO uint32_t ALTCTL;

} WDT_T;

/**
    @addtogroup WDT_CONST WDT Bit Field Definition
    Constant Definitions for WDT Controller
@{ */

#define WDT_CTL_RSTCNT_Pos               (0)                                               /*!< WDT_T::CTL: RSTCNT Position               */
#define WDT_CTL_RSTCNT_Msk               (0x1ul << WDT_CTL_RSTCNT_Pos)                     /*!< WDT_T::CTL: RSTCNT Mask                   */

#define WDT_CTL_RSTEN_Pos                (1)                                               /*!< WDT_T::CTL: RSTEN Position                */
#define WDT_CTL_RSTEN_Msk                (0x1ul << WDT_CTL_RSTEN_Pos)                      /*!< WDT_T::CTL: RSTEN Mask                    */

#define WDT_CTL_RSTF_Pos                 (2)                                               /*!< WDT_T::CTL: RSTF Position                 */
#define WDT_CTL_RSTF_Msk                 (0x1ul << WDT_CTL_RSTF_Pos)                       /*!< WDT_T::CTL: RSTF Mask                     */

#define WDT_CTL_IF_Pos                   (3)                                               /*!< WDT_T::CTL: IF Position                   */
#define WDT_CTL_IF_Msk                   (0x1ul << WDT_CTL_IF_Pos)                         /*!< WDT_T::CTL: IF Mask                       */

#define WDT_CTL_WKEN_Pos                 (4)                                               /*!< WDT_T::CTL: WKEN Position                 */
#define WDT_CTL_WKEN_Msk                 (0x1ul << WDT_CTL_WKEN_Pos)                       /*!< WDT_T::CTL: WKEN Mask                     */

#define WDT_CTL_WKF_Pos                  (5)                                               /*!< WDT_T::CTL: WKF Position                  */
#define WDT_CTL_WKF_Msk                  (0x1ul << WDT_CTL_WKF_Pos)                        /*!< WDT_T::CTL: WKF Mask                      */

#define WDT_CTL_INTEN_Pos                (6)                                               /*!< WDT_T::CTL: INTEN Position                */
#define WDT_CTL_INTEN_Msk                (0x1ul << WDT_CTL_INTEN_Pos)                      /*!< WDT_T::CTL: INTEN Mask                    */

#define WDT_CTL_WDTEN_Pos                (7)                                               /*!< WDT_T::CTL: WDTEN Position                */
#define WDT_CTL_WDTEN_Msk                (0x1ul << WDT_CTL_WDTEN_Pos)                      /*!< WDT_T::CTL: WDTEN Mask                    */

#define WDT_CTL_TOUTSEL_Pos              (8)                                               /*!< WDT_T::CTL: TOUTSEL Position              */
#define WDT_CTL_TOUTSEL_Msk              (0x7ul << WDT_CTL_TOUTSEL_Pos)                    /*!< WDT_T::CTL: TOUTSEL Mask                  */

#define WDT_CTL_TOF_Pos                  (16)                                               /*!< WDT_T::CTL: TOF Position                   */
#define WDT_CTL_TOF_Msk                  (0x1ul << WDT_CTL_TOF_Pos)                         /*!< WDT_T::CTL: TOF Mask                       */

#define WDT_CTL_ICEDEBUG_Pos             (31)                                              /*!< WDT_T::CTL: ICEDEBUG Position             */
#define WDT_CTL_ICEDEBUG_Msk             (0x1ul << WDT_CTL_ICEDEBUG_Pos)                   /*!< WDT_T::CTL: ICEDEBUG Mask                 */

#define WDT_ALTCTL_RSTDSEL_Pos           (0)                                               /*!< WDT_T::ALTCTL: RSTDSEL Position           */
#define WDT_ALTCTL_RSTDSEL_Msk           (0x3ul << WDT_ALTCTL_RSTDSEL_Pos)                 /*!< WDT_T::ALTCTL: RSTDSEL Mask               */

/**@}*/ /* WDT_CONST */
/**@}*/ /* end of WDT register group */


/*---------------------- Window Watchdog Timer -------------------------*/
/**
    @addtogroup WWDT Window Watchdog Timer(WWDT)
    Memory Mapped Structure for WWDT Controller
@{ */

typedef struct {


    /**
     * RLDCNT
     * ===================================================================================================
     * Offset: 0x00  WWDT Reload Counter Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |RLDCNT    |WWDT Reload Counter Register
     * |        |          |Writing 0x00005AA5 to this register will reload the WWDT counter value to 0x3F.
     * |        |          |Note: User can only write WWDT_RLDCNT register to reload WWDT counter value when current WWDT counter value between 0 and CMPDAT (WWDT_CTL[21:16]).
     * |        |          |If user writes WWDT_RLDCNT when current WWDT counter value is larger than CMPDAT, WWDT reset signal will generate immediately.
    */
    __O  uint32_t RLDCNT;

    /**
     * CTL
     * ===================================================================================================
     * Offset: 0x04  WWDT Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WWDTEN    |WWDT Enable Bit
     * |        |          |Set this bit to enable WWDT counter counting.
     * |        |          |0 = WWDT counter is stopped.
     * |        |          |1 = WWDT counter is starting counting.
     * |[1]     |INTEN     |WWDT Interrupt Enable Bit
     * |        |          |If this bit is enabled, the WWDT counter compare match interrupt signal is generated and inform to CPU.
     * |        |          |0 = WWDT counter compare match interrupt Disabled.
     * |        |          |1 = WWDT counter compare match interrupt Enabled.
     * |[11:8]  |PSCSEL    |WWDT Counter Prescale Period Select Bits
     * |        |          |0000 = Pre-scale is 1; Max time-out period is 1 * 64 * WWDT_CLK.
     * |        |          |0001 = Pre-scale is 2; Max time-out period is 2 * 64 * WWDT_CLK.
     * |        |          |0010 = Pre-scale is 4; Max time-out period is 4 * 64 * WWDT_CLK.
     * |        |          |0011 = Pre-scale is 8; Max time-out period is 8 * 64 * WWDT_CLK.
     * |        |          |0100 = Pre-scale is 16; Max time-out period is 16 * 64 * WWDT_CLK.
     * |        |          |0101 = Pre-scale is 32; Max time-out period is 32 * 64 * WWDT_CLK.
     * |        |          |0110 = Pre-scale is 64; Max time-out period is 64 * 64 * WWDT_CLK.
     * |        |          |0111 = Pre-scale is 128; Max time-out period is 128 * 64 * WWDT_CLK.
     * |        |          |1000 = Pre-scale is 192; Max time-out period is 192 * 64 * WWDT_CLK.
     * |        |          |1001 = Pre-scale is 256; Max time-out period is 256 * 64 * WWDT_CLK.
     * |        |          |1010 = Pre-scale is 384; Max time-out period is 384 * 64 * WWDT_CLK.
     * |        |          |1011 = Pre-scale is 512; Max time-out period is 512 * 64 * WWDT_CLK.
     * |        |          |1100 = Pre-scale is 768; Max time-out period is 768 * 64 * WWDT_CLK.
     * |        |          |1101 = Pre-scale is 1024; Max time-out period is 1024 * 64 * WWDT_CLK.
     * |        |          |1110 = Pre-scale is 1536; Max time-out period is 1536 * 64 * WWDT_CLK.
     * |        |          |1111 = Pre-scale is 2048; Max time-out period is 2048 * 64 * WWDT_CLK.
     * |[21:16] |CMPDAT    |WWDT Window Compare Bits
     * |        |          |Set this register to adjust the valid reload window.
     * |        |          |Note: User can only write WWDT_RLDCNT register to reload WWDT counter value when current WWDT counter value between 0 and CMPDAT.
     * |        |          |If user writes WWDT_RLDCNT register when current WWDT counter value larger than CMPDAT, WWDT reset signal will generate immediately.
     * |[31]    |ICEDEBUG  |ICE Debug Mode Acknowledge Disable Bit
     * |        |          |0 = ICE debug mode acknowledgement effects WWDT counting.
     * |        |          |WWDT down counter will be held while CPU is held by ICE.
     * |        |          |1 = ICE debug mode acknowledgement Disabled.
     * |        |          |WWDT down counter will keep going no matter CPU is held by ICE or not.
    */
    __IO uint32_t CTL;

    /**
     * STATUS
     * ===================================================================================================
     * Offset: 0x08  WWDT Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WWDTIF    |WWDT Compare Match Interrupt Flag
     * |        |          |This bit indicates the interrupt flag status of WWDT while WWDT counter value matches CMPDAT (WWDT_CTL[21:16]).
     * |        |          |0 = No effect.
     * |        |          |1 = WWDT counter value matches CMPDAT.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * |[1]     |WWDTRF    |WWDT Timer-out Reset Flag
     * |        |          |This bit indicates the system has been reset by WWDT time-out reset or not.
     * |        |          |0 = WWDT time-out reset did not occur.
     * |        |          |1 = WWDT time-out reset occurred.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * |[2]     |WWDTF     |WWDT Compare Match Flag
     * |        |          |This bit indicates the flag status of WWDT while WWDT counter value matches CMPDAT (WWDT_CTL[21:16]).
     * |        |          |0 = No effect.
     * |        |          |1 = WWDT counter value matches CMPDAT.
     * |        |          |Note: This bit is cleared by writing 1 to it.

    */
    __IO uint32_t STATUS;

    /**
     * CNT
     * ===================================================================================================
     * Offset: 0x0C  WWDT Counter Value Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[5:0]   |CNTDAT    |WWDT Counter Value
     * |        |          |CNTDAT will be updated continuously to monitor 6-bit WWDT down counter value.
    */
    __I  uint32_t CNT;

} WWDT_T;

/**
    @addtogroup WWDT_CONST WWDT Bit Field Definition
    Constant Definitions for WWDT Controller
@{ */

#define WWDT_RLDCNT_RLDCNT_Pos           (0)                                               /*!< WWDT_T::RLDCNT: RLDCNT Position           */
#define WWDT_RLDCNT_RLDCNT_Msk           (0xfffffffful << WWDT_RLDCNT_RLDCNT_Pos)          /*!< WWDT_T::RLDCNT: RLDCNT Mask               */

#define WWDT_CTL_WWDTEN_Pos              (0)                                               /*!< WWDT_T::CTL: WWDTEN Position              */
#define WWDT_CTL_WWDTEN_Msk              (0x1ul << WWDT_CTL_WWDTEN_Pos)                    /*!< WWDT_T::CTL: WWDTEN Mask                  */

#define WWDT_CTL_INTEN_Pos               (1)                                               /*!< WWDT_T::CTL: INTEN Position               */
#define WWDT_CTL_INTEN_Msk               (0x1ul << WWDT_CTL_INTEN_Pos)                     /*!< WWDT_T::CTL: INTEN Mask                   */

#define WWDT_CTL_PSCSEL_Pos              (8)                                               /*!< WWDT_T::CTL: PSCSEL Position              */
#define WWDT_CTL_PSCSEL_Msk              (0xful << WWDT_CTL_PSCSEL_Pos)                    /*!< WWDT_T::CTL: PSCSEL Mask                  */

#define WWDT_CTL_CMPDAT_Pos              (16)                                              /*!< WWDT_T::CTL: CMPDAT Position              */
#define WWDT_CTL_CMPDAT_Msk              (0x3ful << WWDT_CTL_CMPDAT_Pos)                   /*!< WWDT_T::CTL: CMPDAT Mask                  */

#define WWDT_CTL_ICEDEBUG_Pos            (31)                                              /*!< WWDT_T::CTL: ICEDEBUG Position            */
#define WWDT_CTL_ICEDEBUG_Msk            (0x1ul << WWDT_CTL_ICEDEBUG_Pos)                  /*!< WWDT_T::CTL: ICEDEBUG Mask                */

#define WWDT_STATUS_WWDTIF_Pos           (0)                                               /*!< WWDT_T::STATUS: WWDTIF Position           */
#define WWDT_STATUS_WWDTIF_Msk           (0x1ul << WWDT_STATUS_WWDTIF_Pos)                 /*!< WWDT_T::STATUS: WWDTIF Mask               */

#define WWDT_STATUS_WWDTRF_Pos           (1)                                               /*!< WWDT_T::STATUS: WWDTRF Position           */
#define WWDT_STATUS_WWDTRF_Msk           (0x1ul << WWDT_STATUS_WWDTRF_Pos)                 /*!< WWDT_T::STATUS: WWDTRF Mask               */

#define WWDT_STATUS_WWDTF_Pos            (2)                                               /*!< WWDT_T::STATUS: WWDTF Position           */
#define WWDT_STATUS_WWDTF_Msk            (0x1ul << WWDT_STATUS_WWDTF_Pos)                 /*!< WWDT_T::STATUS: WWDTF Mask               */

#define WWDT_CNT_CNTDAT_Pos              (0)                                               /*!< WWDT_T::CNT: CNTDAT Position              */
#define WWDT_CNT_CNTDAT_Msk              (0x3ful << WWDT_CNT_CNTDAT_Pos)                   /*!< WWDT_T::CNT: CNTDAT Mask                  */

/**@}*/ /* WWDT_CONST */
/**@}*/ /* end of WWDT register group */


#if defined ( __CC_ARM   )
#pragma no_anon_unions
#endif

/** @addtogroup PN020_PERIPHERAL_MEM_MAP PN020 Peripheral Memory Map
  Memory Mapped Structure for PN020 Series Peripheral
  @{
 */
/* Peripheral and SRAM base address */
#define FLASH_BASE            ((uint32_t)0x00000000)    ///< Flash base address
#define SRAM_BASE             ((uint32_t)0x20000000)    ///< SRAM base address
#define APB1PERIPH_BASE       ((uint32_t)0x40000000)    ///< APB1 base address
#define APB2PERIPH_BASE       ((uint32_t)0x40100000)    ///< APB2 base address
#define AHBPERIPH_BASE        ((uint32_t)0x50000000)    ///< AHB base address

/* Peripheral memory map */
#define WDT_BASE              (APB1PERIPH_BASE + 0x04000)    ///< WDT register base address
#define WWDT_BASE             (APB1PERIPH_BASE + 0x04100)    ///< WWDT register base address
#define TIMER0_BASE           (APB1PERIPH_BASE + 0x10000)    ///< TIMER0 register base address
#define TIMER1_BASE           (APB1PERIPH_BASE + 0x10020)    ///< TIMER1 register base address
#define TIMER2_BASE           (APB1PERIPH_BASE + 0x10040)    ///< TIMER2 register base address

#define UART0_BASE            (APB2PERIPH_BASE + 0x0000)    ///< UART0 register base address
#define UART1_BASE            (APB2PERIPH_BASE + 0x1000)    ///< UART1 register base address
#define I2C0_BASE             (APB2PERIPH_BASE + 0x2000)    ///< I2C0 register base address
#define I2C1_BASE             (APB2PERIPH_BASE + 0x3000)    ///< I2C1 register base address
#define SPI0_BASE             (APB2PERIPH_BASE + 0x4000)    ///< SPI register base address
#define SPI1_BASE             (APB2PERIPH_BASE + 0x5000)    ///< SPI register base address
#define SPI2_BASE             (APB2PERIPH_BASE + 0x6000)    ///< SPI register base address
#define SPI3_BASE             (APB2PERIPH_BASE + 0x7000)    ///< SPI register base address

#define PWM_BASE              (APB1PERIPH_BASE + 0x40000)    ///< PWM register base address
#define ACMP_BASE             (APB1PERIPH_BASE + 0xD0000)    ///< ACMP register base address
#define ADC_BASE              (APB1PERIPH_BASE + 0xE0000)    ///< ADC register base address

#define SYS_BASE              (AHBPERIPH_BASE + 0x00000)    ///< SYS register base address
#define CLK_BASE              (AHBPERIPH_BASE + 0x00200)    ///< CLK register base address
#define INTR_BASE             (AHBPERIPH_BASE + 0x00300)    ///< INT register base address
#define DMA_BASE              (AHBPERIPH_BASE + 0x02000)    ///< DMA register base address
#define P0_BASE               (AHBPERIPH_BASE + 0x04000)    ///< GPIO Port 0 register base address
#define P1_BASE               (AHBPERIPH_BASE + 0x04040)    ///< GPIO Port 1 register base address
#define P2_BASE               (AHBPERIPH_BASE + 0x04080)    ///< GPIO Port 2 register base address
#define P3_BASE               (AHBPERIPH_BASE + 0x040C0)    ///< GPIO Port 3 register base address
#define P4_BASE               (AHBPERIPH_BASE + 0x04100)    ///< GPIO Port 4 register base address
#define P5_BASE               (AHBPERIPH_BASE + 0x04140)    ///< GPIO Port 5 register base address
#define GPIO_DBNCECON_BASE    (AHBPERIPH_BASE + 0x04180)    ///< GPIO De-bounce register vase
#define GPIO_PIN_DATA_BASE    (AHBPERIPH_BASE + 0x04200)    ///< GPIO pin data register base address
#define GPIOBIT0_BASE         (AHBPERIPH_BASE + 0x04200)    ///< GPIO Port 0 bit access register base address
#define GPIOBIT1_BASE         (AHBPERIPH_BASE + 0x04220)    ///< GPIO Port 1 bit access register base address
#define GPIOBIT2_BASE         (AHBPERIPH_BASE + 0x04240)    ///< GPIO Port 2 bit access register base address
#define GPIOBIT3_BASE         (AHBPERIPH_BASE + 0x04260)    ///< GPIO Port 3 bit access register base address
#define GPIOBIT4_BASE         (AHBPERIPH_BASE + 0x04280)    ///< GPIO Port 4 bit access register base address
#define GPIOBIT5_BASE         (AHBPERIPH_BASE + 0x042A0)    ///< GPIO Port 5 bit access register base address
#define FMC_BASE              (AHBPERIPH_BASE + 0x0C000)    ///< FMC register base address


/*@}*/ /* end of group PN020_PERIPHERAL_MEM_MAP */


/** @addtogroup PN020_PERIPHERAL_DECLARATION PN020 Peripheral Declaration
  The Declaration of PN020 Series Peripheral
  @{
 */
#define WDT                   ((WDT_T *) WDT_BASE)              ///< Pointer to WDT register structure
#define WWDT                  ((WWDT_T *) WWDT_BASE)            ///< Pointer to WWDT register structure
#define TIMER0                ((TIMER_T *) TIMER0_BASE)         ///< Pointer to Timer 0 register structure
#define TIMER1                ((TIMER_T *) TIMER1_BASE)         ///< Pointer to Timer 1 register structure
#define TIMER2                ((TIMER_T *) TIMER2_BASE)         ///< Pointer to Timer 2 register structure
#define UART                  ((UART_T *) UART0_BASE)           ///< Pointer to UART0 register structure
#define UART0                 ((UART_T *) UART0_BASE)           ///< Pointer to UART0 register structure
#define UART1                 ((UART_T *) UART1_BASE)           ///< Pointer to UART1 register structure
#define I2C                   ((I2C_T *) I2C0_BASE)             ///< Pointer to I2C0 register structure
#define I2C0                  ((I2C_T *) I2C0_BASE)             ///< Pointer to I2C0 register structure
#define I2C1                  ((I2C_T *) I2C1_BASE)             ///< Pointer to I2C1 register structure
#define SPI                   ((SPI_T *) SPI0_BASE)              ///< Pointer to SPI register structure
#define SPI0                  ((SPI_T *) SPI0_BASE)              ///< Pointer to SPI register structure
#define SPI1                  ((SPI_T *) SPI1_BASE)              ///< Pointer to SPI register structure
#define SPI2                  ((SPI_T *) SPI2_BASE)             ///< Pointer to SPI(Slave)register structure
#define SPI3                  ((SPI_T *) SPI3_BASE)             ///< Pointer to SPI(Slave) register structure
#define PWM                   ((PWM_T *) PWM_BASE)              ///< Pointer to PWM register structure
#define ADC                   ((ADC_T *) ADC_BASE)              ///< Pointer to ADC register structure
#define ACMP                  ((ACMP_T *) ACMP_BASE)            ///< Pointer to ACMP register structure

#define SYS                   ((SYS_T *) SYS_BASE)              ///< Pointer to SYS register structure
#define CLK                   ((CLK_T *) CLK_BASE)              ///< Pointer to CLK register structure
#define INTR                  ((INTR_T *) INTR_BASE)            ///< Pointer to INT register structure

#define DMA                   ((DMA_T *) DMA_BASE)              ///< Pointer to DMA register structure
#define P0                    ((GPIO_T *) P0_BASE)              ///< Pointer to GPIO port 0 register structure
#define P1                    ((GPIO_T *) P1_BASE)              ///< Pointer to GPIO port 1 register structure
#define P2                    ((GPIO_T *) P2_BASE)              ///< Pointer to GPIO port 2 register structure
#define P3                    ((GPIO_T *) P3_BASE)              ///< Pointer to GPIO port 3 register structure
#define P4                    ((GPIO_T *) P4_BASE)              ///< Pointer to GPIO port 4 register structure
#define P5                    ((GPIO_T *) P5_BASE)              ///< Pointer to GPIO port 5 register structure
#define GPIO_DB               ((GPIO_DB_T *) GPIO_DBNCECON_BASE)  ///< Pointer to GPIO de-bounce register structure
#define FMC                   ((FMC_T *) FMC_BASE)              ///< Pointer to FMC register structure

/*@}*/ /* end of group PN020_PERIPHERAL_DECLARATION */
/*@}*/ /* end of group PN020_Peripherals */

/** @addtogroup PN020_IO_ROUTINE PN020 I/O Routines
  The Declaration of PN020 I/O Routines
  @{
 */

typedef volatile unsigned char  vu8;        ///< Define 8-bit unsigned volatile data type
typedef volatile unsigned short vu16;       ///< Define 16-bit unsigned volatile data type
typedef volatile unsigned long  vu32;       ///< Define 32-bit unsigned volatile data type

/**
  * @brief Get a 8-bit unsigned value from specified address
  * @param[in] addr Address to get 8-bit data from
  * @return  8-bit unsigned value stored in specified address
  */
#define M8(addr)  (*((vu8  *) (addr)))

/**
  * @brief Get a 16-bit unsigned value from specified address
  * @param[in] addr Address to get 16-bit data from
  * @return  16-bit unsigned value stored in specified address
  * @note The input address must be 16-bit aligned
  */
#define M16(addr) (*((vu16 *) (addr)))

/**
  * @brief Get a 32-bit unsigned value from specified address
  * @param[in] addr Address to get 32-bit data from
  * @return  32-bit unsigned value stored in specified address
  * @note The input address must be 32-bit aligned
  */
#define M32(addr) (*((vu32 *) (addr)))

/**
  * @brief Set a 32-bit unsigned value to specified I/O port
  * @param[in] port Port address to set 32-bit data
  * @param[in] value Value to write to I/O port
  * @return  None
  * @note The output port must be 32-bit aligned
  */
#define outpw(port,value)     *((volatile unsigned int *)(port)) = value

/**
  * @brief Get a 32-bit unsigned value from specified I/O port
  * @param[in] port Port address to get 32-bit data from
  * @return  32-bit unsigned value stored in specified I/O port
  * @note The input port must be 32-bit aligned
  */
#define inpw(port)            (*((volatile unsigned int *)(port)))

/**
  * @brief Set a 16-bit unsigned value to specified I/O port
  * @param[in] port Port address to set 16-bit data
  * @param[in] value Value to write to I/O port
  * @return  None
  * @note The output port must be 16-bit aligned
  */
#define outps(port,value)     *((volatile unsigned short *)(port)) = value

/**
  * @brief Get a 16-bit unsigned value from specified I/O port
  * @param[in] port Port address to get 16-bit data from
  * @return  16-bit unsigned value stored in specified I/O port
  * @note The input port must be 16-bit aligned
  */
#define inps(port)            (*((volatile unsigned short *)(port)))

/**
  * @brief Set a 8-bit unsigned value to specified I/O port
  * @param[in] port Port address to set 8-bit data
  * @param[in] value Value to write to I/O port
  * @return  None
  */
#define outpb(port,value)     *((volatile unsigned char *)(port)) = value

/**
  * @brief Get a 8-bit unsigned value from specified I/O port
  * @param[in] port Port address to get 8-bit data from
  * @return  8-bit unsigned value stored in specified I/O port
  */
#define inpb(port)            (*((volatile unsigned char *)(port)))

/**
  * @brief Set a 32-bit unsigned value to specified I/O port
  * @param[in] port Port address to set 32-bit data
  * @param[in] value Value to write to I/O port
  * @return  None
  * @note The output port must be 32-bit aligned
  */
#define outp32(port,value)    *((volatile unsigned int *)(port)) = value

/**
  * @brief Get a 32-bit unsigned value from specified I/O port
  * @param[in] port Port address to get 32-bit data from
  * @return  32-bit unsigned value stored in specified I/O port
  * @note The input port must be 32-bit aligned
  */
#define inp32(port)           (*((volatile unsigned int *)(port)))

/**
  * @brief Set a 16-bit unsigned value to specified I/O port
  * @param[in] port Port address to set 16-bit data
  * @param[in] value Value to write to I/O port
  * @return  None
  * @note The output port must be 16-bit aligned
  */
#define outp16(port,value)    *((volatile unsigned short *)(port)) = value

/**
  * @brief Get a 16-bit unsigned value from specified I/O port
  * @param[in] port Port address to get 16-bit data from
  * @return  16-bit unsigned value stored in specified I/O port
  * @note The input port must be 16-bit aligned
  */
#define inp16(port)           (*((volatile unsigned short *)(port)))

/**
  * @brief Set a 8-bit unsigned value to specified I/O port
  * @param[in] port Port address to set 8-bit data
  * @param[in] value Value to write to I/O port
  * @return  None
  */
#define outp8(port,value)     *((volatile unsigned char *)(port)) = value

/**
  * @brief Get a 8-bit unsigned value from specified I/O port
  * @param[in] port Port address to get 8-bit data from
  * @return  8-bit unsigned value stored in specified I/O port
  */
#define inp8(port)            (*((volatile unsigned char *)(port)))


/*@}*/ /* end of group PN020_IO_ROUTINE */

/******************************************************************************/
/*                Legacy Constants                                            */
/******************************************************************************/
/** @addtogroup PN020_legacy_Constants PN020 Legacy Constants
  PN020 Legacy Constants
  @{
*/

// typedef	enum
// {
// 	false	=	(0),
// 	true	=	(1),
// }bool;
/** @addtogroup Exported_types
  * @{
  */
typedef enum
{
	RESET = 0,
	SET   = 1
} FlagStatus, ITStatus;

typedef enum
{
	DISABLE = 0,
	ENABLE  = 1
} FunctionalState;
#define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))

/**
  * @}
  */

#ifndef NULL
#define NULL           (0)      ///< NULL pointer
#endif

#define TRUE           (1)      ///< Boolean true, define to use in API parameters or return value
#define FALSE          (0)      ///< Boolean false, define to use in API parameters or return value
/*
#define ENABLE         (1)      ///< Enable, define to use in API parameters
#define DISABLE        (0)      ///< Disable, define to use in API parameters
*/
/* Define one bit mask */
#define BIT0     (0x00000001UL)       ///< Bit 0 mask of an 32 bit integer
#define BIT1     (0x00000002UL)       ///< Bit 1 mask of an 32 bit integer
#define BIT2     (0x00000004UL)       ///< Bit 2 mask of an 32 bit integer
#define BIT3     (0x00000008UL)       ///< Bit 3 mask of an 32 bit integer
#define BIT4     (0x00000010UL)       ///< Bit 4 mask of an 32 bit integer
#define BIT5     (0x00000020UL)       ///< Bit 5 mask of an 32 bit integer
#define BIT6     (0x00000040UL)       ///< Bit 6 mask of an 32 bit integer
#define BIT7     (0x00000080UL)       ///< Bit 7 mask of an 32 bit integer
#define BIT8     (0x00000100UL)       ///< Bit 8 mask of an 32 bit integer
#define BIT9     (0x00000200UL)       ///< Bit 9 mask of an 32 bit integer
#define BIT10    (0x00000400UL)       ///< Bit 10 mask of an 32 bit integer
#define BIT11    (0x00000800UL)       ///< Bit 11 mask of an 32 bit integer
#define BIT12    (0x00001000UL)       ///< Bit 12 mask of an 32 bit integer
#define BIT13    (0x00002000UL)       ///< Bit 13 mask of an 32 bit integer
#define BIT14    (0x00004000UL)       ///< Bit 14 mask of an 32 bit integer
#define BIT15    (0x00008000UL)       ///< Bit 15 mask of an 32 bit integer
#define BIT16    (0x00010000UL)       ///< Bit 16 mask of an 32 bit integer
#define BIT17    (0x00020000UL)       ///< Bit 17 mask of an 32 bit integer
#define BIT18    (0x00040000UL)       ///< Bit 18 mask of an 32 bit integer
#define BIT19    (0x00080000UL)       ///< Bit 19 mask of an 32 bit integer
#define BIT20    (0x00100000UL)       ///< Bit 20 mask of an 32 bit integer
#define BIT21    (0x00200000UL)       ///< Bit 21 mask of an 32 bit integer
#define BIT22    (0x00400000UL)       ///< Bit 22 mask of an 32 bit integer
#define BIT23    (0x00800000UL)       ///< Bit 23 mask of an 32 bit integer
#define BIT24    (0x01000000UL)       ///< Bit 24 mask of an 32 bit integer
#define BIT25    (0x02000000UL)       ///< Bit 25 mask of an 32 bit integer
#define BIT26    (0x04000000UL)       ///< Bit 26 mask of an 32 bit integer
#define BIT27    (0x08000000UL)       ///< Bit 27 mask of an 32 bit integer
#define BIT28    (0x10000000UL)       ///< Bit 28 mask of an 32 bit integer
#define BIT29    (0x20000000UL)       ///< Bit 29 mask of an 32 bit integer
#define BIT30    (0x40000000UL)       ///< Bit 30 mask of an 32 bit integer
#define BIT31    (0x80000000UL)       ///< Bit 31 mask of an 32 bit integer

/* Byte Mask Definitions */
#define BYTE0_Msk              (0x000000FF)         ///< Mask to get bit0~bit7 from a 32 bit integer
#define BYTE1_Msk              (0x0000FF00)         ///< Mask to get bit8~bit15 from a 32 bit integer
#define BYTE2_Msk              (0x00FF0000)         ///< Mask to get bit16~bit23 from a 32 bit integer
#define BYTE3_Msk              (0xFF000000)         ///< Mask to get bit24~bit31 from a 32 bit integer

#define GET_BYTE0(u32Param)    ((u32Param & BYTE0_Msk)      )  /*!< Extract Byte 0 (Bit  0~ 7) from parameter u32Param */
#define GET_BYTE1(u32Param)    ((u32Param & BYTE1_Msk) >>  8)  /*!< Extract Byte 1 (Bit  8~15) from parameter u32Param */
#define GET_BYTE2(u32Param)    ((u32Param & BYTE2_Msk) >> 16)  /*!< Extract Byte 2 (Bit 16~23) from parameter u32Param */
#define GET_BYTE3(u32Param)    ((u32Param & BYTE3_Msk) >> 24)  /*!< Extract Byte 3 (Bit 24~31) from parameter u32Param */

/*@}*/ /* end of group PN020_legacy_Constants */

/*@}*/ /* end of group PN020_Definitions */

#ifdef __cplusplus
}
#endif


/******************************************************************************/
/*                         Peripheral header files                            */
/******************************************************************************/
#include "sys.h"
#include "clk.h"
#include "adc.h"
#include "fmc.h"
#include "gpio.h"
#include "i2c.h"
#include "pwm.h"
#include "spi.h"
#include "timer.h"
#include "uart.h"
#include "wdt.h"
#include "wwdt.h"
#include "dma.h"
#include "panble.h"
#endif  // __PN020SERIES_H__

/*** (C) COPYRIGHT 2016 Panchip Technology Corp. ***/

