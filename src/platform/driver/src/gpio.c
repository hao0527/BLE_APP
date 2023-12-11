/**************************************************************************//**
 * @file     gpio.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 16/02/25 17:21 $
 * @brief    PN102 series GPIO driver source file
 *
 * @note
 * Copyright (C) 2016 Panchip Technology Corp. All rights reserved.
*****************************************************************************/
#include "PN102Series.h"



/** @addtogroup PN102_Device_Driver PN102 Device Driver
  @{
*/

/** @addtogroup PN102_GPIO_Driver GPIO Driver
  @{
*/


/** @addtogroup PN102_GPIO_EXPORTED_FUNCTIONS GPIO Exported Functions
  @{
*/

/**
 * @brief       Set GPIO operation mode
 *
 * @param[in]   gpio        GPIO port. It could be \ref P0, \ref P1, \ref P2, \ref P3, \ref P4 or \ref P5.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port. \ref BIT0, \ref BIT1, \ref BIT2,.. \ref BIT7
 * @param[in]   u32Mode     Operation mode.
 *                          - \ref GPIO_MODE_INPUT,
 *                          - \ref GPIO_MODE_OUTPUT,
 *                          - \ref GPIO_MODE_OPEN_DRAIN,
 *                          - \ref GPIO_MODE_QUASI
 *
 * @return      None
 *
 * @details     This function is used to set specified GPIO operation mode.
 */
void GPIO_SetMode(GPIO_T *gpio, uint32_t u32PinMask, uint32_t u32Mode)
{
    uint32_t i;

    for (i=0; i<GPIO_PIN_MAX; i++) {
        if (u32PinMask & (1 << i)) {
            gpio->MODE = (gpio->MODE & ~(0x3 << (i << 1))) | (u32Mode << (i << 1));
        }
	
    }
//	if((GPIO_MODE_INPUT==u32Mode)|(GPIO_MODE_QUASI==u32Mode))
		gpio->DINOFF  &= ~ ( u32PinMask<<16 ); //gpio 检测使能
//    if((GPIO_MODE_INPUT==u32Mode)||(GPIO_MODE_QUASI==u32Mode))
//		gpio->DINOFF |=  u32PinMask;		//digital pull up path enabled 
    if((GPIO_MODE_QUASI==u32Mode))
			gpio->DINOFF |=  u32PinMask;		//digital pull up path enabled 
}

/**
 * @brief       Enable GPIO interrupt
 *
 * @param[in]   gpio            GPIO port. It could be \ref P0, \ref P1, \ref P2, \ref P3, \ref P4 or \ref P5.
 * @param[in]   u32Pin          The pin of specified GPIO port. It could be 0 ~ 7.
 * @param[in]   u32IntAttribs   The interrupt attribute of specified GPIO pin. It could be \n
 *                              - \ref GPIO_INT_RISING,
 *                              - \ref GPIO_INT_FALLING,
 *                              - \ref GPIO_INT_BOTH_EDGE,
 *                              - \ref GPIO_INT_HIGH,
 *                              - \ref GPIO_INT_LOW.
 *
 * @return      None
 *
 * @details     This function is used to enable specified GPIO pin interrupt.
 */
void GPIO_EnableInt(GPIO_T *gpio, uint32_t u32Pin, uint32_t u32IntAttribs)
{
    gpio->INTTYPE |= (((u32IntAttribs >> 24) & 0xFFUL) << u32Pin);//level/edge choose u32IntAttribs第六位决定
    gpio->INTEN |= ((u32IntAttribs & 0xFFFFFFUL) << u32Pin);//选择上升或者下降或者双沿触发 或 高电平或者低电平触发
}


/**
 * @brief       Disable GPIO interrupt
 *
 * @param[in]   gpio        GPIO port. It could be \ref P0, \ref P1, \ref P2, \ref P3, \ref P4 or \ref P5.
 * @param[in]   u32Pin      The pin of specified GPIO port. It could be 0 ~ 7.
 *
 * @return      None
 *
 * @details     This function is used to enable specified GPIO pin interrupt.
 */
void GPIO_DisableInt(GPIO_T *gpio, uint32_t u32Pin)
{
    gpio->INTTYPE &= ~(1UL << u32Pin);
    gpio->INTEN &= ~((0x00010001UL) << u32Pin);
}

void GPIO_InitInput(GPIO_T * GPIOx, uint32_t GPIO_Pin, GPIO_PULL_T GPIO_Pull)
{
    GPIO_SetMode(P0, GPIO_Pin, GPIO_MODE_INPUT);
}

void GPIO_InitOutput(GPIO_T * GPIOx, uint32_t GPIO_Pin, GPIO_VALUE_T GPIO_Value)
{
    GPIO_SetMode(GPIOx, GPIO_Pin, GPIO_MODE_OUTPUT);

    if (0 == GPIO_Value)
    {
        GPIO_ClearBits(GPIOx, GPIO_Pin);
    }
    else
    {
        GPIO_SetBits(GPIOx, GPIO_Pin);
    }
}

/**
 * @brief       Interrupt De-bounce Enable and Cycle Control
 *
 * @param[in]   gpio        GPIO port. It could be \ref P0, \ref P1, \ref P2, \ref P3, \ref P4 or \ref P5.
 * @param[in]   u32Pin      The pin of specified GPIO port. It could be 0 ~ 7.
 *
 * @return      None
 *
 * @details     This function is used to enable GPIO pin De-bounce function and set the De-bounce cycle.
 */
void GPIO_EnableDebounce(GPIO_T *gpio, uint32_t u32Pin)//使能去抖功能
{
    gpio->DBEN |= (1UL << u32Pin);//DBEN置位
    GPIO_DB->DBCTL = GPIO_DBCTL_DBCLKSEL_32768;//bit[4] 0 = De-bounce counter clock source is HCLK.1 = De-bounce counter clock source is 32 kHz internal low speed RC oscillator(LIRC).
}

/**
 * @brief       Interrupt De-bounce Enable and Cycle Control
 *
 * @param[in]   gpio        GPIO port. It could be \ref P0, \ref P1, \ref P2, \ref P3, \ref P4 or \ref P5.
 * @param[in]   u32Pin      The pin of specified GPIO port. It could be 0 ~ 7.
 *
 * @return      None
 *
 * @details     This function is used to enable GPIO pin De-bounce function and set the De-bounce cycle.
 */
void GPIO_DisableDebounce(GPIO_T *gpio, uint32_t u32Pin)//失能去抖功能
{
    gpio->DBEN &= ~(1UL << u32Pin);//DBEN清位
}
/**
 * @brief       GPIO Pull up enable
 *
 * @param[in]   gpio        GPIO port. It could be \ref P0, \ref P1, \ref P2, \ref P3, \ref P4 or \ref P5.
 * @param[in]   u32PinMask      The pin of specified GPIO port. It could be 0 ~ 7.
 * @param[in]   u8Mode      1 for GPIO_PULLUP_ENABLE ;0 for GPIO_PULLUP_DISABLE;
 *
 * @return      None
 *
 * @details     This function is used to enable GPIO pin De-bounce function and set the De-bounce cycle.
 */
void GPIO_PullUp(GPIO_T *gpio, uint32_t u32PinMask, uint8_t u8Mode)
{
    if(u8Mode == 0)
    {
        gpio->DINOFF &=~ u32PinMask;		//digital pull up path disenabled 
    }
    else
    {
        gpio->DINOFF |=  u32PinMask;		//digital pull up path enabled 
    }
}

#include <string.h>
uint8_t m_gpio_status = 0;
GPIO_T  m_gpios;

void GPIO_Store ( void )
{
    m_gpio_status = 1;
    memcpy ( &m_gpios, P1, sizeof(GPIO_T) );
}

void GPIO_Retract ( void )
{
    if ( 0 == m_gpio_status )
    {
        return;
    }
    memcpy ( P1, &m_gpios, sizeof(GPIO_T) );
}

/*@}*/ /* end of group PN102_GPIO_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group PN102_GPIO_Driver */

/*@}*/ /* end of group PN102_Device_Driver */

/*** (C) COPYRIGHT 2016 Panchip Technology Corp. ***/
