#include "mcu_hal.h"
#include "adc.h"
#include "stack_svc_api.h"

#define ARRAY_NUM(arr)		(sizeof(arr)/sizeof((arr)[0]))


////////////////////////////////////////////gpio_user/////////////////////////////////////////////
void mcu_gpio_user_init(void)
{
	// gpio init
	GPIO_PullUp(P1, BIT3, GPIO_PULLUP_ENABLE);
	GPIO_ENABLE_DIGITAL_PATH(P1, BIT3);
	SYS->P1_MFP &= ~(SYS_MFP_P13_Msk);
	SYS->P1_MFP |= SYS_MFP_P13_GPIO;
	GPIO_InitOutput(P1, BIT3, GPIO_HIGH_LEVEL);
	GPIO_SetBits(P1, BIT3);	// ldo en ����
}


////////////////////////////////////////////adc_driver/////////////////////////////////////////////
static uint8 mcuAdcTableNum = 0;
static MCU_ADC_TAB * p_mcuAdcTable = NULL;
static volatile uint16 mcuAdcUserChIdx = 0;	// ��ǰ�ɼ����Table������Ϣ��p_mcuAdcTable[mcuAdcUserChIdx]
static uint32 mcuAdcOverTimeStamp = 0;

static void mcu_adc_start_channel_convert(ADC_CHANNEL channel)
{
    ADC_Open(ADC, 0, 0, 0x01 << channel);	// Enable channel
	ADC_START_CONV(ADC);
}

/***********************************************************************************************
*������: void mcu_adc_init(MCU_ADC_TAB *p_table, uint8 tableNum)
*��������:
*�ر�˵��:
*��������:
*��������ֵ:
*�޸ļ�¼:
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
		;	// ���� gpio ���ú͸��ã�Ĭ��Ϊ�������룬�û��������ⲿ�ֶ������ⲿ��
	}
	mcuAdcUserChIdx = 0;
	mcu_adc_start_channel_convert(p_mcuAdcTable[mcuAdcUserChIdx].adcChannel);
	// mcuAdcOverTimeStamp = tim_get_count();
	
	// ע���жϴ�������Э��ջ�������жϻῨ����
	((interrupt_register_handler)SVC_interrupt_register)(ADC_IRQ, mcu_adc_isr);
}

/***********************************************************************************************
*������: float mcu_adc_get_voltage(uint8 index)
*��������:
*�ر�˵��:
*��������:
*��������ֵ:
*�޸ļ�¼:
***********************************************************************************************/
float mcu_adc_get_voltage(uint8 index)
{
	if(NULL==p_mcuAdcTable || index>=mcuAdcTableNum)
		return 0;
	return p_mcuAdcTable[index].voltage;
}

/***********************************************************************************************
*������: void mcu_adc_main(void)
*��������:
*�ر�˵��:
*��������:
*��������ֵ:
*�޸ļ�¼:
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
*������: void mcu_adc_isr(void)
*��������:
*�ر�˵��:
*��������:
*��������ֵ:
*�޸ļ�¼:
***********************************************************************************************/
void mcu_adc_isr(void)
{
	uint16 adcCode = ADC_GET_CONVERSION_DATA(ADC, 0);	// ��ȡת�����
	uint32 flag = ADC_GET_INT_FLAG(ADC, ADC_ADIF_INT);	// ���жϱ�־λ
	ADC_CLR_INT_FLAG(ADC, flag);	// ���ж�

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


////////////////////////////////////////////adc_user/////////////////////////////////////////////
#define MCU_AVDD_CFG 3.3
MCU_ADC_TAB adcTable[] = {
	{ADC_CH01, 100, MCU_AVDD_CFG/4096},
	{ADC_CH02, 100, MCU_AVDD_CFG/4096},
};

void mcu_adc_user_init(void)
{
	// gpio init
	GPIO_SetMode(P1, BIT0, GPIO_MODE_INPUT);
	GPIO_SetMode(P1, BIT2, GPIO_MODE_INPUT);
	GPIO_PullUp(P1, BIT0, GPIO_PULLUP_DISABLE);
	GPIO_PullUp(P1, BIT2, GPIO_PULLUP_DISABLE);
	GPIO_DISABLE_DIGITAL_PATH(P1, BIT0);
	GPIO_DISABLE_DIGITAL_PATH(P1, BIT2);
	SYS->P1_MFP &= ~(SYS_MFP_P10_Msk|SYS_MFP_P12_Msk);
	SYS->P1_MFP |= SYS_MFP_P10_ADC_CH1|SYS_MFP_P12_ADC_CH2;

	// adc init
	mcu_adc_init(adcTable, ARRAY_NUM(adcTable));
}
