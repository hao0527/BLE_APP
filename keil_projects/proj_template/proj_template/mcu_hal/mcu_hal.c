#include "mcu_hal.h"
#include "adc.h"

////////////////////////////////////////////配置adc/////////////////////////////////////////////
static uint8 mcuAdcTableNum = 0;
static MCU_ADC_TAB * p_mcuAdcTable = NULL;
static volatile uint16 mcuAdcUserChIdx = 0;	// 当前采集点的Table索引信息，p_mcuAdcTable[mcuAdcUserChIdx]
static uint32 mcuAdcOverTimeStamp = 0;

static void mcu_adc_start_channel_convert(ADC_CHANNEL channel)
{
    ADC_Open(ADC, 0, 0, 0x01 << channel);	// Enable channel
	ADC_START_CONV(ADC);
}

/***********************************************************************************************
*函数名: void mcu_adc_init(MCU_ADC_TAB *p_table, uint8 tableNum)
*功能描述:
*特别说明:
*函数参数:
*函数返回值:
*修改记录:
***********************************************************************************************/
void mcu_adc_init(MCU_ADC_TAB *p_table, uint8 tableNum)
{
	uint8 i = 0;
	mcuAdcTableNum = tableNum;
	p_mcuAdcTable = p_table;

	CLK_EnableModuleClock(ADC_MODULE);
    //Select ADC input range.1 means 0.4V~2.4V ;0 means 0.4V~1.4V.
    //0.4V~2.4V & 0.4V~1.4V both are theoretical value,the real range is determined by bandgap voltage.
    ADC_SelInputRange(ADC_INPUTRANGE_HIGH);
    // Power on ADC
    ADC_POWER_ON(ADC);
    // Enable ADC convert complete interrupt
    ADC_EnableInt(ADC, ADC_ADIF_INT);
    NVIC_EnableIRQ(ADC_IRQn);

	for(i = 0; i < mcuAdcTableNum; i++)
	{
		;	// 还差 gpio 配置和复用，默认为浮空输入，用户可以在外部手动配置这部分
	}
	mcuAdcUserChIdx = 0;
	mcu_adc_start_channel_convert(p_mcuAdcTable[mcuAdcUserChIdx].adcChannel);
	// mcuAdcOverTimeStamp = tim_get_count();
}

/***********************************************************************************************
*函数名: float mcu_adc_get_voltage(uint8 index)
*功能描述:
*特别说明:
*函数参数:
*函数返回值:
*修改记录:
***********************************************************************************************/
float mcu_adc_get_voltage(uint8 index)
{
	if(NULL==p_mcuAdcTable || index>=mcuAdcTableNum)
		return 0;
	return p_mcuAdcTable[index].voltage;
}

/***********************************************************************************************
*函数名: void mcu_adc_main(void)
*功能描述:
*特别说明:
*函数参数:
*函数返回值:
*修改记录:
***********************************************************************************************/
void mcu_adc_main(void)
{
	uint8 i = 0;
	
	if(NULL==p_mcuAdcTable)
		return;
	
	if(mcuAdcUserChIdx >= mcuAdcTableNum)
	{
		for(i = 0; i < mcuAdcTableNum; i++)
		{
			p_mcuAdcTable[i].filterCnt++;
			p_mcuAdcTable[i].adcCodeSum += p_mcuAdcTable[i].adcCode;
			if(p_mcuAdcTable[i].filterCnt >= p_mcuAdcTable[i].filterLen)
			{
				p_mcuAdcTable[i].voltage = p_mcuAdcTable[i].adcCodeSum / p_mcuAdcTable[i].filterCnt * p_mcuAdcTable[i].convRate;
				p_mcuAdcTable[i].rollCount++;
				p_mcuAdcTable[i].filterCnt = 0;
				p_mcuAdcTable[i].adcCodeSum = 0;
			}
		}
		mcuAdcUserChIdx = 0;
		mcu_adc_start_channel_convert(p_mcuAdcTable[mcuAdcUserChIdx].adcChannel);
		// mcuAdcOverTimeStamp = tim_get_count();
	}
	else
	{
		if(0/*tim_check_timenow(mcuAdcOverTimeStamp, 200)*/)
		{
			mcuAdcUserChIdx = 0;
			mcu_adc_start_channel_convert(p_mcuAdcTable[mcuAdcUserChIdx].adcChannel);
			// mcuAdcOverTimeStamp = tim_get_count();
		}
	}
}

/***********************************************************************************************
*函数名: void mcu_adc_isr(void)
*功能描述:
*特别说明:
*函数参数:
*函数返回值:
*修改记录:
***********************************************************************************************/
void mcu_adc_isr(void)
{
	uint16 adcCode = ADC_GET_CONVERSION_DATA(ADC, 0);	// 读取转换结果
	uint32 flag = ADC_GET_INT_FLAG(ADC, ADC_ADIF_INT);	// 读中断标志位
	ADC_CLR_INT_FLAG(ADC, flag);	// 清中断

	if(NULL==p_mcuAdcTable)
		return;
	
	p_mcuAdcTable[mcuAdcUserChIdx].adcCode = adcCode;
	mcuAdcUserChIdx++;
	if(mcuAdcUserChIdx < mcuAdcTableNum)
	{
		mcu_adc_start_channel_convert(p_mcuAdcTable[mcuAdcUserChIdx].adcChannel);
		// mcuAdcOverTimeStamp = tim_get_count();
	}
}

void ADC_IRQHandler(void)
{
	mcu_adc_isr();
}
