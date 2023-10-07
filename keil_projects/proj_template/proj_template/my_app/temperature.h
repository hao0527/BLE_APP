#ifndef __TEMPERATURE_H_
#define __TEMPERATURE_H_

#include "panble.h"

// �ɼ��¶����ڣ���λ(1min)
#define SAMPLE_TEMPER_PERIOD        15
// TemperTable�����С
#define TEMPER_TABLE_MAX_LEN		500
// TemperValue 0ֵ�¶� �����϶ȣ�
#define ZERO_TEMPER_VALUE_C			37.0f
// TemperValue ���� �����϶ȣ�
#define PRECISION_TEMPER_VALUE_C	0.05f
// TemperValue ת ��ʵ�¶� �����϶ȣ�
#define TEMPER_VALUE_TO_C(value)	(ZERO_TEMPER_VALUE_C + PRECISION_TEMPER_VALUE_C * value)

#pragma pack(1)
typedef struct TemperCfg
{
	uint16 zeroTemperValue;
	uint16 precisionTemperValue;
	uint16 temperTableMaxLen;
	uint8 sampleTemperPeriod;
}TemperCfg_t;

typedef struct TemperReadCfg
{
	uint16 startCnt;	// �ӵڼ��β�����ʼ��ȡ
	uint16 readLen;		// ��ȡ�ĳ���
}TemperReadCfg_t;
#pragma pack()

extern const TemperCfg_t g_temperCfg;
extern TemperReadCfg_t g_temperReadCfg;

int8 temper_getTemperValue(uint16 cnt);
uint16 temper_getTemperCnt(void);
void temper_sampleTemperTimerCb(void);
int8 temper_sampleTemper(void);
void temper_resetInit(void);

#endif //__TEMPERATURE_H_
