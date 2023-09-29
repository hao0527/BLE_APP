#include "temperature.h"
#include "mcu_hal.h"
#include <string.h>
#include <math.h>

static int8 temperTable[TEMPER_TABLE_MAX_LEN];	// 存放历史温度数据
static uint16 temperCnt;	// 采温度完成次数

/**
 * @brief 保存温度值到temperTable，同时更新temperCnt
 * @param value 温度值
 */
static void saveTemperValue(int8 value)
{
	temperTable[temperCnt % TEMPER_TABLE_MAX_LEN] = value;
	temperCnt++;
}

#define TEMPER_VALUE_MAX_CFG 100
#define TEMPER_VALUE_MIN_CFG -100
/**
 * @brief 真实温度 （摄氏度） 转 TemperValue
 * @return int8 返回TemperValue
 */
static inline int8 C_TO_TEMPER_VALUE(float value)
{
	int32 t;
	// round函数 四舍五入
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
 * @brief adc电压值到温度值的转换
 * @attention 温度值不是真实温度，是int8类型的，真实温度等于ZERO_TEMPER_VALUE_C+温度值*PRECISION_TEMPER_VALUE_C
 * @param voltage adc电压值
 * @return int8 返回温度值
 */
static int8 adcVoltageToTemperValue(float voltage)
{
	float t;	// 摄氏度(°C)
	float r;	// 某温度时ntc实际阻值(Ohm)
	int8 temperValue;

	// r = ( ( ( 6.25 - voltage ) / ( 6.25 + voltage ) ) * 100000 );	// 运放分析采集的电压与ntc阻值关系
	r = 1000 * (1500 - 200 * voltage) / (8.75 + 3 * voltage);
	t = 1.0 / ( 1.0 / ( 273.15 + 25 ) + 1.0 / NTC_B_VALUE_CFG * log( r / ( NTC_25_C_RES_CFG ) ) ) - 273.15;
	temperValue = C_TO_TEMPER_VALUE(t);
	return temperValue;
}

/**
 * @brief 获取温度值
 * @param cnt 第几次采样
 * @return int8 温度值，0表示37度，返回-128(0x80)表示错误
 * @note 真实温度等于ZERO_TEMPER_VALUE_C+返回值*PRECISION_TEMPER_VALUE_C
 */
int8 temper_getTemperValue(uint16 cnt)
{
	if(cnt > temperCnt || cnt == 0 ||
	   (temperCnt > TEMPER_TABLE_MAX_LEN && temperCnt - cnt >= TEMPER_TABLE_MAX_LEN))	// 判断是否近TEMPER_TABLE_MAX_LEN次
		return 0x80;
	return temperTable[(cnt - 1) % 500];
}

/**
 * @brief 获取采样完成次数
 * @return uint16 返回采样次数
 */
uint16 temper_getTemperCnt(void)
{
	return temperCnt;
}

/**
 * @brief 以阻塞的方式采集一次温度，后存储
 */
void temper_sampleTemper(void)
{
	float v;
	int8 t;

	mcu_gpio_user_init();
	mcu_gpio_en_ldo(TRUE);
	mcu_adc_user_init();
	while(!mcu_adc_main());	// 阻塞到采样完成
	mcu_gpio_en_ldo(FALSE);
	v = mcu_adc_get_voltage(MCU_P12_ADC_CH2);
	t = adcVoltageToTemperValue(v);
	saveTemperValue(t);
}

/**
 * @brief 初次上电ram初始化，唤醒不需要调用
 */
void temper_resetInit(void)
{
	temperCnt = 0;
	memset(temperTable, 0, sizeof(temperTable));
	temper_sampleTemper();	// 阻塞采集一次温度
}
