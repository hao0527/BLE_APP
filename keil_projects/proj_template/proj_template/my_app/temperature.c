#include "temperature.h"
#include "mcu_hal.h"
#include "panip_config.h"
#include <string.h>
#include <math.h>

const TemperCfg_t g_temperCfg = {
	.zeroTemperValue = ZERO_TEMPER_VALUE_C * 1000,
	.precisionTemperValue = PRECISION_TEMPER_VALUE_C * 1000,
	.temperTableMaxLen = TEMPER_TABLE_MAX_LEN,
	.sampleTemperPeriod = SAMPLE_TEMPER_PERIOD,
};

TemperReadCfg_t g_temperReadCfg;

static int8 temperTable[TEMPER_TABLE_MAX_LEN];	// �����ʷ�¶�����
static uint16 temperCnt;	// ���¶���ɴ���
static uint8 temperTimerCnt;	// ��ʱ���ص��������

/**
 * @brief �����¶�ֵ��temperTable��ͬʱ����temperCnt
 * @param value �¶�ֵ
 */
static void saveTemperValue(int8 value)
{
	temperTable[temperCnt % TEMPER_TABLE_MAX_LEN] = value;
	temperCnt++;
}

#define TEMPER_VALUE_MAX_CFG 100
#define TEMPER_VALUE_MIN_CFG -100
#define TEMPER_VALUE_OVER_MAX 127
#define TEMPER_VALUE_LESS_MIN -128
/**
 * @brief ��ʵ�¶� �����϶ȣ� ת TemperValue�����������С��ʾ����ֵ
 * @param value ��ʵ�¶�
 * @return int8 ����TemperValue
 */
static inline int8 C_TO_TEMPER_VALUE(float value)
{
	int32 t;
	// round���� ��������
	t = (int32)round((value - ZERO_TEMPER_VALUE_C) / PRECISION_TEMPER_VALUE_C);
	if(t > TEMPER_VALUE_MAX_CFG)
		t = TEMPER_VALUE_OVER_MAX;
	else if(t < TEMPER_VALUE_MIN_CFG)
		t = TEMPER_VALUE_LESS_MIN;
	return (int8)t;
}

#define NTC_B_VALUE_CFG 4250
#define NTC_25_C_RES_CFG 100000
/**
 * @brief adc��ѹֵ���¶�ֵ��ת��
 * @attention �¶�ֵ������ʵ�¶ȣ���int8���͵ģ���ʵ�¶ȵ���ZERO_TEMPER_VALUE_C+�¶�ֵ*PRECISION_TEMPER_VALUE_C
 * @param voltage adc��ѹֵ(V)
 * @return int8 �����¶�ֵ
 */
static int8 adcVoltageToTemperValue(float voltage)
{
	float t;	// ���϶�(��C)
	float r;	// ĳ�¶�ʱntcʵ����ֵ(Ohm)
	int8 temperValue;

	// r = ( ( ( 6.25 - voltage ) / ( 6.25 + voltage ) ) * 100000 );	// �˷ŷ����ɼ��ĵ�ѹ��ntc��ֵ��ϵ
	r = 1000 * (1500 - 200 * voltage) / (8.75 + 3 * voltage);
	t = 1.0 / ( 1.0 / ( 273.15 + 25 ) + 1.0 / NTC_B_VALUE_CFG * log( r / ( NTC_25_C_RES_CFG ) ) ) - 273.15;
	temperValue = C_TO_TEMPER_VALUE(t);
	return temperValue;
}

#define TEMPER_VALUE_ERROR -127
/**
 * @brief ��ȡ�¶�ֵ
 * @param cnt �ڼ��β���
 * @return int8 �¶�ֵ��0��ʾZERO_TEMPER_VALUE_C�ȣ�����TEMPER_VALUE_ERROR��ʾ����
 * @note ��ʵ�¶ȵ���ZERO_TEMPER_VALUE_C+����ֵ*PRECISION_TEMPER_VALUE_C
 */
int8 temper_getTemperValue(uint16 cnt)
{
	if(cnt > temperCnt || cnt == 0 ||
	   (temperCnt > TEMPER_TABLE_MAX_LEN && temperCnt - cnt >= TEMPER_TABLE_MAX_LEN))	// �ж��Ƿ��TEMPER_TABLE_MAX_LEN��
		return TEMPER_VALUE_ERROR;
	return temperTable[(cnt - 1) % TEMPER_TABLE_MAX_LEN];
}

/**
 * @brief ��ȡ������ɴ���
 * @return uint16 ���ز�������
 */
uint16 temper_getTemperCnt(void)
{
	return temperCnt;
}

/**
 * @brief ÿ����SAMPLE_TEMPER_PERIOD�Σ�������һ���¶ȣ���洢
 * @note ��ʱ���ص�ÿ1min��һ�δ˽ӿ�
 */
void temper_sampleTemperTimerCb(void)
{
	float v;
	int8 t;
	
	if(++temperTimerCnt >= SAMPLE_TEMPER_PERIOD)
		temperTimerCnt = 0;	// ÿ��SAMPLE_TEMPER_PERIOD����������һ��
	else
		return;
	
	mcu_gpio_user_init();
	mcu_gpio_en_ldo(TRUE);
	mcu_adc_user_init();
	while(!mcu_adc_main());	// �������������
	mcu_gpio_en_ldo(FALSE);
	v = mcu_adc_get_voltage(MCU_P12_ADC_CH2);
	t = adcVoltageToTemperValue(v);
	saveTemperValue(t);
}

/**
 * @brief ������һ���¶ȣ����洢
 */
int8 temper_sampleTemper(void)
{
	float v;
	int8 t;

	mcu_gpio_user_init();
	mcu_gpio_en_ldo(TRUE);
	mcu_adc_user_init();
	while(!mcu_adc_main());	// �������������
	mcu_gpio_en_ldo(FALSE);
	v = mcu_adc_get_voltage(MCU_P12_ADC_CH2);
	t = adcVoltageToTemperValue(v);
	return t;
}

/**
 * @brief �����ϵ�ram��ʼ�������Ѳ���Ҫ����
 */
void temper_resetInit(void)
{
	temperCnt = 0;
	temperTimerCnt = 0;
	memset(&g_temperReadCfg, 0, sizeof(g_temperReadCfg));
	memset(temperTable, 0, sizeof(temperTable));
	temper_sampleTemperTimerCb();	// �����ɼ�һ���¶�
}
