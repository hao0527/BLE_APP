#ifndef __MCU_HAL_H_
#define __MCU_HAL_H_

#include "panble.h"

////////////////////////////////////////////配置adc/////////////////////////////////////////////
typedef enum{
	ADC_CH00 = 0,
	ADC_CH01,
	ADC_CH02,
	ADC_CH03,
	ADC_CH04,
	ADC_CH05,
	ADC_CH06,
	ADC_CH07,
}ADC_CHANNEL;
#pragma pack(1)
typedef struct{
	ADC_CHANNEL		adcChannel;
	uint16			filterLen;
	float			convRate;		// 转换率，adcCode * convRate = voltage
	//无需配置
	uint16			adcCode;
	uint32			adcCodeSum;
	uint16			filterCnt;
	uint16			rollCount;
	float			voltage;
}MCU_ADC_TAB;
#pragma pack()
void mcu_adc_init(MCU_ADC_TAB *p_table, uint8 tableNum);
float mcu_adc_get_voltage(uint8 index);
void mcu_adc_main(void);
void mcu_adc_isr(void);

#endif //__MCU_HAL_H_
