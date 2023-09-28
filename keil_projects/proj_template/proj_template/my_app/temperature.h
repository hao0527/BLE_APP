#ifndef __TEMPERATURE_H_
#define __TEMPERATURE_H_

#include "panble.h"

// TemperTable�����С
#define TEMPER_TABLE_MAX_LEN		500
// TemperValue 0ֵ�¶� �����϶ȣ�
#define ZERO_TEMPER_VALUE_C			37.0f
// TemperValue ���� �����϶ȣ�
#define PRECISION_TEMPER_VALUE_C	0.05f
// TemperValue ת ��ʵ�¶� �����϶ȣ�
#define TEMPER_VALUE_TO_C(value)	(ZERO_TEMPER_VALUE_C + PRECISION_TEMPER_VALUE_C * value)

int8 temper_getTemperValue(uint16 cnt);
uint16 temper_getTemperCnt(void);
void temper_sampleTemper(void);
void temper_varInit(void);

#endif //__TEMPERATURE_H_
