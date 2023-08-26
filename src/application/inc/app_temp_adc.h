/**
 ****************************************************************************************
 * @file    app_temp_adc.h
 * @brief   
 
 * @version 0.01
 * @date    2018/11/26
 * @history 
 * @note	   
 * detailed description 
 
 ****************************************************************************************
 * @attention
 *
 * Copyright (C) 2016 Shanghai Panchip Microelectronics Co., Ltd. All rights reserved.
 ****************************************************************************************
 */


/** @defgroup temp_adc
* @{
*/

#ifndef __APP_TEMP_ADC_H__
#define __APP_TEMP_ADC_H__


#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* __cplusplus */
	
#include "stdint.h"
#include "adc.h"

void temperature_init(void);
void rf_calibration_process(void);
void adc_temp_param_init(void);	

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */


#endif /* __APP_TEMP_ADC_H__ */

