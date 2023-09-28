#include "temperature.h"
#include "mcu_hal.h"

int8 temperTable[TEMPER_TABLE_MAX_LEN];	// 存放历史温度数据
uint16 temperCnt;	// 采温度完成次数

/**
 * @brief 保存温度值到temperTable，同时更新temperCnt
 * @param value 温度值
 */
static void saveTemperValue(int8 value)
{
	;
}

/**
 * @brief adc电压值到温度值的转换
 * @attention 温度值不是真实温度，是int8类型的，真实温度等于ZERO_TEMPER_VALUE_C+温度值*PRECISION_TEMPER_VALUE_C
 * @param voltage adc电压值
 * @return int8 返回温度值
 */
static int8 adcVoltageToTemperValue(float voltage)
{
	return 0;
}

/**
 * @brief 获取温度值
 * @param cnt 第几次采样
 * @return int8 温度值，0表示37度
 * @note 真实温度等于ZERO_TEMPER_VALUE_C+返回值*PRECISION_TEMPER_VALUE_C
 */
int8 temper_getTemperValue(uint16 cnt)
{
	return 0;
}

/**
 * @brief 获取采样完成次数
 * @return uint16 返回采样次数
 */
uint16 temper_getTemperCnt(void)
{
	return 0;
}

/**
 * @brief 以阻塞的方式采集一次温度，后存储
 */
void temper_sampleTemper(void)
{
	;
}

