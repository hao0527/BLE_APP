#include "temperature.h"
#include "mcu_hal.h"
#include <string.h>
#include <math.h>

static int8 temperTable[TEMPER_TABLE_MAX_LEN];	// �����ʷ�¶�����
static uint16 temperCnt;	// ���¶���ɴ���

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
/**
 * @brief ��ʵ�¶� �����϶ȣ� ת TemperValue
 * @return int8 ����TemperValue
 */
static inline int8 C_TO_TEMPER_VALUE(float value)
{
	int32 t;
	// round���� ��������
	t = (int32)round((value - ZERO_TEMPER_VALUE_C) / PRECISION_TEMPER_VALUE_C);
	if(t > TEMPER_VALUE_MAX_CFG)
		t = TEMPER_VALUE_MAX_CFG;
	else if(t < TEMPER_VALUE_MIN_CFG)
		t = TEMPER_VALUE_MIN_CFG;
	return (int8)t;
}

#define NTC_B_VALUE_CFG 4250
#define NTC_25_C_RES_CFG 100000
/**
 * @brief adc��ѹֵ���¶�ֵ��ת��
 * @attention �¶�ֵ������ʵ�¶ȣ���int8���͵ģ���ʵ�¶ȵ���ZERO_TEMPER_VALUE_C+�¶�ֵ*PRECISION_TEMPER_VALUE_C
 * @param voltage adc��ѹֵ
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

/**
 * @brief ��ȡ�¶�ֵ
 * @param cnt �ڼ��β���
 * @return int8 �¶�ֵ��0��ʾ37�ȣ�����-128(0x80)��ʾ����
 * @note ��ʵ�¶ȵ���ZERO_TEMPER_VALUE_C+����ֵ*PRECISION_TEMPER_VALUE_C
 */
int8 temper_getTemperValue(uint16 cnt)
{
	if(cnt > temperCnt || cnt == 0 ||
	   (temperCnt > TEMPER_TABLE_MAX_LEN && temperCnt - cnt >= TEMPER_TABLE_MAX_LEN))	// �ж��Ƿ��TEMPER_TABLE_MAX_LEN��
		return 0x80;
	return temperTable[(cnt - 1) % 500];
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
 * @brief �������ķ�ʽ�ɼ�һ���¶ȣ���洢
 */
void temper_sampleTemper(void)
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
	saveTemperValue(t);
}

/**
 * @brief �����ϵ�ram��ʼ�������Ѳ���Ҫ����
 */
void temper_resetInit(void)
{
	temperCnt = 0;
	memset(temperTable, 0, sizeof(temperTable));
	temper_sampleTemper();	// �����ɼ�һ���¶�
}
