#ifndef __MCU_HAL_H_
#define __MCU_HAL_H_

#include "panble.h"


////////////////////////////////////////////gpio_user/////////////////////////////////////////////
void mcu_gpio_user_init(void);


////////////////////////////////////////////adc_driver/////////////////////////////////////////////
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
	float			convRate;		// ת���ʣ�adcCode * convRate = voltage
	//��������
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

////////////////////////////////////////////adc_user/////////////////////////////////////////////
enum {
	MCU_P10_ADC_CH1 = 0,
	MCU_P12_ADC_CH2,
};
void mcu_adc_user_init(void);

#endif //__MCU_HAL_H_
