/**************************************************************************//**
 * @file     PWM.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 16/02/24 1:43p $
 * @brief    PN102 series PWM driver source file
 *
 * @note
 * Copyright (C) 2016 Panchip Technology Corp. All rights reserved.
*****************************************************************************/
#include "PN102Series.h"

/** @addtogroup PN102_Device_Driver PN102 Device Driver
  @{
*/

/** @addtogroup PN102_PWM_Driver PWM Driver
  @{
*/


/** @addtogroup PN102_PWM_EXPORTED_FUNCTIONS PWM Exported Functions
  @{
*/

/**
 * @brief This function config PWM generator and get the nearest frequency in edge aligned auto-reload mode
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @param[in] u32Frequency Target generator frequency
 * @param[in] u32DutyCycle Target generator duty cycle percentage. Valid range are between 0 ~ 100. 10 means 10%, 20 means 20%...
 * @return Nearest frequency clock in nano second
 * @note Since every two channels, (0 & 1), (2 & 3), (4 & 5),  (6 & 7) shares a prescaler. Call this API to configure PWM frequency may affect
 *       existing frequency of other channel.
 */
uint32_t PWM_ConfigOutputChannel (PWM_T *pwm,
                                  uint32_t u32ChannelNum,
                                  uint32_t u32Frequency,
                                  uint32_t u32DutyCycle)
{
    uint32_t i = SystemCoreClock / u32Frequency;
    uint8_t  u8Divider = 1, u8Prescale = 0xFF;
    uint16_t u16CNR = 0xFFFF;

    for(; u8Divider < 17; u8Divider <<= 1) {  // clk divider could only be 1, 2, 4, 8, 16
        i = (SystemCoreClock / u32Frequency) / u8Divider;
        // If target value is larger than CNR * prescale, need to use a larger divider
        if(i > (0x10000 * 0x100))
            continue;

        // CNR = 0xFFFF + 1, get a prescaler that CNR value is below 0xFFFF
        u8Prescale = (i + 0xFFFF)/ 0x10000;

        // u8Prescale must at least be 2, otherwise the output stop
        if(u8Prescale < 3)
            u8Prescale = 2;

        i /= u8Prescale;

        if(i <= 0x10000) {
            if(i == 1)
                u16CNR = 1;     // Too fast, and PWM cannot generate expected frequency...
            else
                u16CNR = i;
            break;
        }

    }
    // Store return value here 'cos we're gonna change u8Divider & u8Prescale & u16CNR to the real value to fill into register
    i = SystemCoreClock / (u8Prescale * u8Divider * u16CNR);

    u8Prescale -= 1;
    u16CNR -= 1;
    // convert to real register value
    if(u8Divider == 1)
        u8Divider = 4;
    else if (u8Divider == 2)
        u8Divider = 0;
    else if (u8Divider == 4)
        u8Divider = 1;
    else if (u8Divider == 8)
        u8Divider = 2;
    else // 16
        u8Divider = 3;

    // every two channels share a prescaler
    PWM->CLKPSC = (PWM->CLKPSC & ~(PWM_CLKPSC_CLKPSC01_Msk << ((u32ChannelNum >> 1) * 8))) | (u8Prescale << ((u32ChannelNum >> 1) * 8));
    PWM->CLKDIV = (PWM->CLKDIV & ~(PWM_CLKDIV_CLKDIV0_Msk << (4 * u32ChannelNum))) | (u8Divider << (4 * u32ChannelNum));

		PWM->CTL |= (PWM_CTL_CNTMODE0_Msk << ((4 * u32ChannelNum)));//make pwm auto-reload mode
		PWM->CTL2 = (PWM->CTL2 & ~PWM_CTL2_CNTTYPE_Msk); // edge-aligned or center_aligned
        
    if(u32DutyCycle == 0)
        *((__IO uint32_t *)((((uint32_t) & ((pwm)->CMPDAT0)) + u32ChannelNum * 4))) = 0;
    else
        *((__IO uint32_t *)((((uint32_t) & ((pwm)->CMPDAT0)) + u32ChannelNum * 4))) = u32DutyCycle * (u16CNR + 1) / 100 - 1;

    *((__IO uint32_t *)((((uint32_t) & ((pwm)->PERIOD0)) + (u32ChannelNum) * 4))) = u16CNR;

    return(i);
}

/**
 * @brief This function config PWM generator and get the nearest frequency in edge aligned auto-reload mode
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @param[in] u32Frequency Target generator frequency
 * @param[in] u32DutyCycle Target generator duty cycle percentage. Valid range are between 0 ~ 100. 10 means 10%, 20 means 20%...
 * @return Nearest frequency clock in nano second
 * @note Since every two channels, (0 & 1), (2 & 3), (4 & 5),  (6 & 7) shares a prescaler. Call this API to configure PWM frequency may affect
 *       existing frequency of other channel.
 */
uint32_t PWM_ConfigOutputChannel_CenterAligned (PWM_T *pwm,
                                  uint32_t u32ChannelNum,
                                  uint32_t u32Frequency,
                                  uint32_t u32DutyCycle)
{
    uint32_t i = SystemCoreClock / u32Frequency;
    uint8_t  u8Divider = 1, u8Prescale = 0xFF;
    uint16_t u16CNR = 0xFFFF;

    for(; u8Divider < 17; u8Divider <<= 1) {  // clk divider could only be 1, 2, 4, 8, 16
        i = (SystemCoreClock / u32Frequency) / u8Divider;
        // If target value is larger than CNR * prescale, need to use a larger divider
        if(i > (0x10000 * 0x100))
            continue;

        // CNR = 0xFFFF + 1, get a prescaler that CNR value is below 0xFFFF
        u8Prescale = (i + 0xFFFF)/ 0x10000;

        // u8Prescale must at least be 2, otherwise the output stop
        if(u8Prescale < 3)
            u8Prescale = 2;

        i /= u8Prescale;

        if(i <= 0x10000) {
            if(i == 1)
                u16CNR = 1;     // Too fast, and PWM cannot generate expected frequency...
            else
                u16CNR = i/2;
            break;
        }

    }
    // Store return value here 'cos we're gonna change u8Divider & u8Prescale & u16CNR to the real value to fill into register
    i = SystemCoreClock / (u8Prescale * u8Divider * u16CNR)/2;

    u8Prescale -= 1;
    u16CNR -= 1;
    // convert to real register value
    if(u8Divider == 1)
        u8Divider = 4;
    else if (u8Divider == 2)
        u8Divider = 0;
    else if (u8Divider == 4)
        u8Divider = 1;
    else if (u8Divider == 8)
        u8Divider = 2;
    else // 16
        u8Divider = 3;

    // every two channels share a prescaler
    PWM->CLKPSC = (PWM->CLKPSC & ~(PWM_CLKPSC_CLKPSC01_Msk << ((u32ChannelNum >> 1) * 8))) | (u8Prescale << ((u32ChannelNum >> 1) * 8));
    PWM->CLKDIV = (PWM->CLKDIV & ~(PWM_CLKDIV_CLKDIV0_Msk << (4 * u32ChannelNum))) | (u8Divider << (4 * u32ChannelNum));

		PWM->CTL |= (PWM_CTL_CNTMODE0_Msk << ((4 * u32ChannelNum)));//make pwm auto-reload mode
		PWM->CTL2 = (PWM->CTL2 & ~PWM_CTL2_CNTTYPE_Msk); // edge-aligned or center_aligned
        PWM->CTL2 = (PWM->CTL2 |PWM_CTL2_CNTTYPE_Msk); //  center_aligned
    if(u32DutyCycle == 0)
        *((__IO uint32_t *)((((uint32_t) & ((pwm)->CMPDAT0)) + u32ChannelNum * 4))) = 0;
    else
        *((__IO uint32_t *)((((uint32_t) & ((pwm)->CMPDAT0)) + u32ChannelNum * 4))) = u16CNR-(u32DutyCycle * (u16CNR + 1) / 100) ;

    *((__IO uint32_t *)((((uint32_t) & ((pwm)->PERIOD0)) + (u32ChannelNum) * 4))) = u16CNR;

    return(i);
}


uint32_t PWM_ConfigOutputChannel_PreciseCenterAligned (PWM_T *pwm,
                                  uint32_t u32ChannelNum,
                                  uint32_t u32Frequency,
                                  uint32_t u32DutyCycle)
{
    uint32_t i = SystemCoreClock / u32Frequency;
    uint8_t  u8Divider = 1, u8Prescale = 0xFF;
    uint16_t u16CNR = 0xFFFF;

    for(; u8Divider < 17; u8Divider <<= 1) {  // clk divider could only be 1, 2, 4, 8, 16
        i = (SystemCoreClock / u32Frequency) / u8Divider;
        // If target value is larger than CNR * prescale, need to use a larger divider
        if(i > (0x10000 * 0x100))
            continue;

        // CNR = 0xFFFF + 1, get a prescaler that CNR value is below 0xFFFF
        u8Prescale = (i + 0xFFFF)/ 0x10000;

        // u8Prescale must at least be 2, otherwise the output stop
        if(u8Prescale < 3)
            u8Prescale = 2;

        i /= u8Prescale;

        if(i <= 0x10000) {
            if(i == 1)
                u16CNR = 1;     // Too fast, and PWM cannot generate expected frequency...
            else
                u16CNR = i;
            break;
        }

    }
    // Store return value here 'cos we're gonna change u8Divider & u8Prescale & u16CNR to the real value to fill into register
    i = SystemCoreClock / (u8Prescale * u8Divider * u16CNR);

    u8Prescale -= 1;
    u16CNR = u16CNR;
    // convert to real register value
    if(u8Divider == 1)
        u8Divider = 4;
    else if (u8Divider == 2)
        u8Divider = 0;
    else if (u8Divider == 4)
        u8Divider = 1;
    else if (u8Divider == 8)
        u8Divider = 2;
    else // 16
        u8Divider = 3;

    // every two channels share a prescaler
    PWM->CLKPSC = (PWM->CLKPSC & ~(PWM_CLKPSC_CLKPSC01_Msk << ((u32ChannelNum >> 1) * 8))) | (u8Prescale << ((u32ChannelNum >> 1) * 8));
    PWM->CLKDIV = (PWM->CLKDIV & ~(PWM_CLKDIV_CLKDIV0_Msk << (4 * u32ChannelNum))) | (u8Divider << (4 * u32ChannelNum));

		PWM->CTL |= (PWM_CTL_CNTMODE0_Msk << ((4 * u32ChannelNum)));//make pwm auto-reload mode
		PWM->CTL2 = (PWM->CTL2 & ~PWM_CTL2_CNTTYPE_Msk); // clear aligned mode
        PWM->CTL2 = (PWM->CTL2 |PWM_CTL2_CNTTYPE_Msk); // set center_aligned
        PWM->PCACTL = 0x1; //Precise center-aligned type Enabled.
    if(u32DutyCycle == 0)
        *((__IO uint32_t *)((((uint32_t) & ((pwm)->CMPDAT0)) + u32ChannelNum * 4))) = 0;
    else
        *((__IO uint32_t *)((((uint32_t) & ((pwm)->CMPDAT0)) + u32ChannelNum * 4))) = (u16CNR-(u16CNR*u32DutyCycle/100))/2-1 ;

    *((__IO uint32_t *)((((uint32_t) & ((pwm)->PERIOD0)) + (u32ChannelNum) * 4))) = u16CNR;

    return(i);
}


/**
 * @brief This function start PWM module
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel.
 *                           Bit 0 is channel 0, bit 1 is channel 1...
 * @return None
 */
void PWM_Start (PWM_T *pwm, uint32_t u32ChannelMask)
{
    uint32_t u32Mask = 0, i;
    for(i = 0; i < PWM_CHANNEL_NUM; i ++) {
        if(u32ChannelMask & (1 << i)) {
            u32Mask |= (PWM_CTL_CNTEN0_Msk << (i * 4));
        }
    }
    PWM->CTL |= u32Mask;
}

/**
 * @brief This function stop PWM module
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel.
 *                           Bit 0 is channel 0, bit 1 is channel 1...
 * @return None
 */
void PWM_Stop (PWM_T *pwm, uint32_t u32ChannelMask)
{
    uint32_t i;
    for(i = 0; i < PWM_CHANNEL_NUM; i ++) {
        if(u32ChannelMask & (1 << i)) {
            *((__IO uint32_t *)((((uint32_t) & ((pwm)->PERIOD0)) + (i) * 4))) = 0;
        }
    }
}

/**
 * @brief This function stop PWM generation immediately by clear channel enable bit
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel.
 *                           Bit 0 is channel 0, bit 1 is channel 1...
 * @return None
 */
void PWM_ForceStop (PWM_T *pwm, uint32_t u32ChannelMask)
{
    uint32_t u32Mask = 0, i;
    for(i = 0; i < PWM_CHANNEL_NUM; i ++) {
        if(u32ChannelMask & (1 << i)) {
            u32Mask |= (PWM_CTL_CNTEN0_Msk << (i * 4));
        }
    }
    PWM->CTL &= ~u32Mask;
}

/**
 * @brief This function enable selected channel to trigger ADC
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @param[in] u32Condition The condition to trigger ADC. Combination of following conditions:
 *                  - \ref PWM_TRIGGER_ADC_CNTR_IS_0
 *                  - \ref PWM_TRIGGER_ADC_CNTR_IS_CMR_D
 *                  - \ref PWM_TRIGGER_ADC_CNTR_IS_CNR
 *                  - \ref PWM_TRIGGER_ADC_CNTR_IS_CMR_U
 * @return None
 */
void PWM_EnableADCTrigger (PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Condition)
{
    if(u32ChannelNum < 4) {
        PWM->ADCTCTL0 = (PWM->ADCTCTL0 & ~((PWM_TRIGGER_ADC_CNTR_IS_0 |
                                            PWM_TRIGGER_ADC_CNTR_IS_CMR_D |
                                            PWM_TRIGGER_ADC_CNTR_IS_CNR |
                                            PWM_TRIGGER_ADC_CNTR_IS_CMR_U ) << (8 * u32ChannelNum))) | (u32Condition << (8 * u32ChannelNum));
    } else {
        PWM->ADCTCTL1 = (PWM->ADCTCTL1 & ~((PWM_TRIGGER_ADC_CNTR_IS_0 |
                                            PWM_TRIGGER_ADC_CNTR_IS_CMR_D |
                                            PWM_TRIGGER_ADC_CNTR_IS_CNR |
                                            PWM_TRIGGER_ADC_CNTR_IS_CMR_U ) << (8 * (u32ChannelNum - 4)))) | (u32Condition << (8 * (u32ChannelNum - 4)));
    }
}

/**
 * @brief This function disable selected channel to trigger ADC
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @return None
 */
void PWM_DisableADCTrigger (PWM_T *pwm, uint32_t u32ChannelNum)
{
    if(u32ChannelNum < 4) {
        PWM->ADCTCTL0 = (PWM->ADCTCTL0 & ~((PWM_TRIGGER_ADC_CNTR_IS_0 |
                                            PWM_TRIGGER_ADC_CNTR_IS_CMR_D |
                                            PWM_TRIGGER_ADC_CNTR_IS_CNR |
                                            PWM_TRIGGER_ADC_CNTR_IS_CMR_U ) << (8 * u32ChannelNum)));
    } else {
        PWM->ADCTCTL1 = (PWM->ADCTCTL1 & ~((PWM_TRIGGER_ADC_CNTR_IS_0 |
                                            PWM_TRIGGER_ADC_CNTR_IS_CMR_D |
                                            PWM_TRIGGER_ADC_CNTR_IS_CNR |
                                            PWM_TRIGGER_ADC_CNTR_IS_CMR_U ) << (8 * (u32ChannelNum - 4))));
    }
}

/**
 * @brief This function clear selected channel trigger ADC flag
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @param[in] u32Condition PWM triggered ADC flag to be cleared.
 * @return None
 */
void PWM_ClearADCTriggerFlag (PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Condition)
{
    if(u32ChannelNum < 4) {
        PWM->ADCTSTS0 |= (u32Condition << (8 * u32ChannelNum));
    } else {
        PWM->ADCTSTS1 |= (u32Condition << (8 * (u32ChannelNum - 4)));
    }
}

/**
 * @brief This function get selected channel trigger ADC flag
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @param[in] u32Condition PWM triggered ADC flag to be selected.
 * @return Get status of the selected channel trigger ADC
 */
uint32_t PWM_GetADCTriggerFlag (PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Condition)
{
    if(u32ChannelNum < 4) {
        return(PWM->ADCTSTS0 & (u32Condition << (8 * u32ChannelNum)) ? 1 : 0);
    } else {
        return(PWM->ADCTSTS1 & (u32Condition << (8 * (u32ChannelNum - 4))) ? 1 : 0);
    }
}


/**
 * @brief This function enables PWM output generation of selected channels
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel.
 *                           Set bit 0 to 1 enables channel 0 output, set bit 1 to 1 enables channel 1 output...
 * @return None
 */
void PWM_EnableOutput (PWM_T *pwm, uint32_t u32ChannelMask)
{
    PWM->POEN |= u32ChannelMask;
}

/**
 * @brief This function disables PWM output generation of selected channels
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel
 *                           Set bit 0 to 1 disables channel 0 output, set bit 1 to 1 disables channel 1 output...
 * @return None
 */
void PWM_DisableOutput (PWM_T *pwm, uint32_t u32ChannelMask)
{
    PWM->POEN &= ~u32ChannelMask;
}

/**
 * @brief This function enable Dead zone of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @param[in] u32Duration Dead Zone length in PWM clock count, valid values are between 0~0xFF, but 0 means there is no
 *                        dead zone.
 * @return None
 */
void PWM_EnableDeadZone (PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Duration)
{
    // every two channels shares the same setting
    u32ChannelNum >>= 1;
    // set duration
    PWM->DTCTL = (PWM->DTCTL & ~(PWM_DTCTL_DTI01_Msk << (8 * u32ChannelNum))) | (u32Duration << (8 * u32ChannelNum));
    // enable dead zone
    PWM->CTL2 |= (PWM_CTL2_DTCNT01_Msk << u32ChannelNum);
}

/**
 * @brief This function disable Dead zone of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @return None
 */
void PWM_DisableDeadZone (PWM_T *pwm, uint32_t u32ChannelNum)
{
    // every two channels shares the same setting
    u32ChannelNum >>= 1;
    // enable dead zone
    PWM->CTL2 &= ~(PWM_CTL2_DTCNT01_Msk << u32ChannelNum);
}

/**
 * @brief This function enable Compare_Down interrupt of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @return None
 */
void PWM_EnableCMPDInt (PWM_T *pwm, uint32_t u32ChannelNum)
{
    (pwm)->INTEN |= ((1 << PWM_INTEN_CMPDIEN0_Pos) << u32ChannelNum);
}

/**
 * @brief This function disable Compare_Down interrupt of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @return None
 */
void PWM_DisableCMPDInt (PWM_T *pwm, uint32_t u32ChannelNum)
{
    (pwm)->INTEN &= ~((1 << PWM_INTEN_CMPDIEN0_Pos) << u32ChannelNum);
}

/**
 * @brief This function clears Compare_Down interrupt flag of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @return None
 */
void PWM_ClearCMPDIntFlag (PWM_T *pwm, uint32_t u32ChannelNum)
{
    PWM->INTSTS = (PWM_INTSTS_CMPDIF0_Msk << u32ChannelNum);
}

/**
 * @brief This function get Compare_Down interrupt flag of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @return Compare_Down interrupt flag of specified channel
 * @retval 0 Compare_Down interrupt did not occurred
 * @retval 1 Compare_Down interrupt occurred
 */
uint32_t PWM_GetCMPDIntFlag (PWM_T *pwm, uint32_t u32ChannelNum)
{
    return(PWM->INTSTS & (PWM_INTSTS_CMPDIF0_Msk << u32ChannelNum) ? 1 : 0);
}

/**
 * @brief This function enable Period interrupt of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @return None
 */
void PWM_EnablePeriodInt (PWM_T *pwm, uint32_t u32ChannelNum)
{
    (pwm)->INTEN |= ((1 << PWM_INTEN_PIEN0_Pos) << u32ChannelNum);
}

/**
 * @brief This function disable Period interrupt of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @return None
 */
void PWM_DisablePeriodInt (PWM_T *pwm, uint32_t u32ChannelNum)
{
    (pwm)->INTEN &= ~((1 << PWM_INTEN_PIEN0_Pos) << u32ChannelNum);
}

/**
 * @brief This function clears Period interrupt flag of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @return None
 */
void PWM_ClearPeriodIntFlag (PWM_T *pwm, uint32_t u32ChannelNum)
{
    PWM->INTSTS = (PWM_INTSTS_PIF0_Msk << u32ChannelNum);
}

/**
 * @brief This function get Period interrupt flag of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @return Period interrupt flag of specified channel
 * @retval 0 Period interrupt did not occurred
 * @retval 1 Period interrupt occurred
 */
uint32_t PWM_GetPeriodIntFlag (PWM_T *pwm, uint32_t u32ChannelNum)
{
    return(PWM->INTSTS & (PWM_INTSTS_PIF0_Msk << u32ChannelNum) ? 1 : 0);
}



/**
 * @brief This function enable Zero interrupt of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @return None
 */
void PWM_EnableZeroInt (PWM_T *pwm, uint32_t u32ChannelNum)
{
    (pwm)->INTEN |= ((1 << PWM_INTEN_ZIEN0_Pos) << u32ChannelNum);
}

/**
 * @brief This function disable Zero interrupt of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @return None
 */
void PWM_DisableZeroInt (PWM_T *pwm, uint32_t u32ChannelNum)
{
    (pwm)->INTEN &= ~((1 << PWM_INTEN_ZIEN0_Pos) << u32ChannelNum);
}

/**
 * @brief This function clears Zero interrupt flag of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @return None
 */
void PWM_ClearZeroIntFlag (PWM_T *pwm, uint32_t u32ChannelNum)
{
    PWM->INTSTS = (PWM_INTSTS_ZIF0_Msk << u32ChannelNum);
}

/**
 * @brief This function get Zero interrupt flag of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @return Zero interrupt flag of specified channel
 * @retval 0 Zero interrupt did not occurred
 * @retval 1 Zero interrupt occurred
 */
uint32_t PWM_GetZeroIntFlag (PWM_T *pwm, uint32_t u32ChannelNum)
{
    return(PWM->INTSTS & (PWM_INTSTS_ZIF0_Msk << u32ChannelNum) ? 1 : 0);
}
//------------------------------------------------------
/**
 * @brief This function enable Compare_Up interrupt of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @return None
 */
void PWM_EnableCMPUInt (PWM_T *pwm, uint32_t u32ChannelNum)
{
    (pwm)->INTEN |= ((1 << PWM_INTEN_CMPUIEN0_Pos) << u32ChannelNum);
}

/**
 * @brief This function disable Compare_Up interrupt of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @return None
 */
void PWM_DisableCMPUInt (PWM_T *pwm, uint32_t u32ChannelNum)
{
    (pwm)->INTEN &= ~((1 << PWM_INTEN_CMPUIEN0_Pos) << u32ChannelNum);
}

/**
 * @brief This function clears Compare_Up interrupt flag of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @return None
 */
void PWM_ClearCMPUIntFlag (PWM_T *pwm, uint32_t u32ChannelNum)
{
    PWM->INTSTS = (PWM_INTSTS_CMPUIF0_Msk << u32ChannelNum);
}

/**
 * @brief This function get Compare_Up interrupt flag of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @return Compare_Up interrupt flag of specified channel
 * @retval 0 Compare_Up interrupt did not occurred
 * @retval 1 Compare_Up interrupt occurred
 */
uint32_t PWM_GetCMPUIntFlag (PWM_T *pwm, uint32_t u32ChannelNum)
{
    return(PWM->INTSTS & (PWM_INTSTS_CMPUIF0_Msk << u32ChannelNum) ? 1 : 0);
}




/**
 * @brief This function enable Central interrupt of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @param[in] u32IntPeriodType Period interrupt type, could be either
 *              - \ref PWM_PERIOD_INT_UNDERFLOW
 *              - \ref PWM_PERIOD_INT_MATCH_CNR
 * @return None
 * @note All channels share the same Central interrupt type setting.
 */
void PWM_EnableCenterInt (PWM_T *pwm, uint32_t u32ChannelNum,  uint32_t u32IntPeriodType)
{
 		PWM->CTL2 &=  ~PWM_CTL2_PINTTYPE_Msk;
		PWM->CTL2|= u32IntPeriodType;
		PWM->INTEN = (PWM->INTEN & ~(PWM_INTEN_PIEN0_Msk << u32ChannelNum)) | (PWM_INTEN_PIEN0_Msk << u32ChannelNum) ;   
    
    
}

/**
 * @brief This function disable Central interrupt of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @return None
 */
void PWM_DisableCenterInt (PWM_T *pwm, uint32_t u32ChannelNum)
{
    PWM->INTEN &= ~(PWM_INTEN_PIEN0_Msk << u32ChannelNum);
}

/**
 * @brief This function clear Central interrupt of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @return None
 */
void PWM_ClearCenterIntFlag (PWM_T *pwm, uint32_t u32ChannelNum)
{
    PWM->INTSTS = (PWM_INTSTS_PIF0_Msk << u32ChannelNum);
}

/**
 * @brief This function get Central interrupt of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~7
 * @return Central interrupt flag of specified channel
 * @retval 0 Central interrupt did not occurred
 * @retval 1 Central interrupt occurred
 */
uint32_t PWM_GetCenterIntFlag (PWM_T *pwm, uint32_t u32ChannelNum)
{
    return(PWM->INTSTS & (PWM_INTSTS_PIF0_Msk << u32ChannelNum) ? 1 : 0);
}


/*@}*/ /* end of group PN102_PWM_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group PN102_PWM_Driver */

/*@}*/ /* end of group PN102_Device_Driver */

/*** (C) COPYRIGHT 2016 Panchip Technology Corp. ***/
