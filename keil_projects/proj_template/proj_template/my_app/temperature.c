#include "temperature.h"
#include "mcu_hal.h"

int8 temperTable[TEMPER_TABLE_MAX_LEN];	// �����ʷ�¶�����
uint16 temperCnt;	// ���¶���ɴ���

/**
 * @brief �����¶�ֵ��temperTable��ͬʱ����temperCnt
 * @param value �¶�ֵ
 */
static void saveTemperValue(int8 value)
{
	;
}

/**
 * @brief adc��ѹֵ���¶�ֵ��ת��
 * @attention �¶�ֵ������ʵ�¶ȣ���int8���͵ģ���ʵ�¶ȵ���ZERO_TEMPER_VALUE_C+�¶�ֵ*PRECISION_TEMPER_VALUE_C
 * @param voltage adc��ѹֵ
 * @return int8 �����¶�ֵ
 */
static int8 adcVoltageToTemperValue(float voltage)
{
	return 0;
}

/**
 * @brief ��ȡ�¶�ֵ
 * @param cnt �ڼ��β���
 * @return int8 �¶�ֵ��0��ʾ37��
 * @note ��ʵ�¶ȵ���ZERO_TEMPER_VALUE_C+����ֵ*PRECISION_TEMPER_VALUE_C
 */
int8 temper_getTemperValue(uint16 cnt)
{
	return 0;
}

/**
 * @brief ��ȡ������ɴ���
 * @return uint16 ���ز�������
 */
uint16 temper_getTemperCnt(void)
{
	return 0;
}

/**
 * @brief �������ķ�ʽ�ɼ�һ���¶ȣ���洢
 */
void temper_sampleTemper(void)
{
	;
}

