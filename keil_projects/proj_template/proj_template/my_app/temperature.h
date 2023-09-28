#ifndef __TEMPERATURE_H_
#define __TEMPERATURE_H_

#include "panble.h"

// TemperTable数组大小
#define TEMPER_TABLE_MAX_LEN		500
// TemperValue 0值温度 （摄氏度）
#define ZERO_TEMPER_VALUE_C			37.0f
// TemperValue 精度 （摄氏度）
#define PRECISION_TEMPER_VALUE_C	0.05f
// TemperValue 转 真实温度 （摄氏度）
#define TEMPER_VALUE_TO_C(value)	(ZERO_TEMPER_VALUE_C + PRECISION_TEMPER_VALUE_C * value)

int8 temper_getTemperValue(uint16 cnt);
uint16 temper_getTemperCnt(void);
void temper_sampleTemper(void);
void temper_varInit(void);

#endif //__TEMPERATURE_H_
