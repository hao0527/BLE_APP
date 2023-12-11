#include "mcu_hal.h"
#include "adc.h"
#include "stack_svc_api.h"

#define ARRAY_NUM(arr)		(sizeof(arr)/sizeof((arr)[0]))


////////////////////////////////////////////gpio_user/////////////////////////////////////////////
void mcu_gpio_user_init(void)
{
	// gpio init
	GPIO_PullUp(P1, BIT0, GPIO_PULLUP_ENABLE);
	GPIO_ENABLE_DIGITAL_PATH(P1, BIT0);
	SYS->P1_MFP &= ~(SYS_MFP_P10_Msk);
	SYS->P1_MFP |= SYS_MFP_P10_GPIO;
	GPIO_InitOutput(P1, BIT0, GPIO_LOW_LEVEL);	// ��ע�⣺ԭ�����GPIO_LOW_LEVEL������ȷӦ���� GPIO_LOW_LEVEL=0
	P10 = 0;	// ldo Ĭ�ϲ���

	GPIO_PullUp(P1, BIT4, GPIO_PULLUP_ENABLE);
	GPIO_ENABLE_DIGITAL_PATH(P1, BIT4);
	SYS->P1_MFP &= ~(SYS_MFP_P14_Msk);
	SYS->P1_MFP |= SYS_MFP_P14_GPIO;
	GPIO_SetMode(P1, BIT4, GPIO_MODE_OUTPUT);	// ��͹���io����
//	GPIO_InitOutput(P1, BIT4, GPIO_HIGH_LEVEL);	// ��ע�⣺ԭ�����GPIO_HIGH_LEVEL������ȷӦ���� GPIO_HIGH_LEVEL=1
//	P14 = 1;	// led Ĭ�ϲ���

	GPIO_PullUp(P1, BIT5, GPIO_PULLUP_ENABLE);
	GPIO_ENABLE_DIGITAL_PATH(P1, BIT5);
	SYS->P1_MFP &= ~(SYS_MFP_P15_Msk);
	SYS->P1_MFP |= SYS_MFP_P15_GPIO;
	GPIO_SetMode(P1, BIT5, GPIO_MODE_OUTPUT);	// ��͹���io����
	// GPIO_InitOutput(P1, BIT5, GPIO_HIGH_LEVEL);	// P15 �� POW_EN ������Ҫһֱ���ߣ��ػ�ʱ����

	GPIO_PullUp(P3, BIT0, GPIO_PULLUP_DISABLE);	// �ⲿ�������ر��ڲ�������ֹ©����
	GPIO_ENABLE_DIGITAL_PATH(P3, BIT0);
	SYS->P1_MFP &= ~(SYS_MFP_P30_Msk);
	SYS->P1_MFP |= SYS_MFP_P30_GPIO;
	GPIO_SetMode(P3, BIT0, GPIO_MODE_INPUT);	// P30 �� KEY �ߵ�ƽΪ����
}

void mcu_gpio_en_ldo(bool en)
{
	if(en)
		P10 = 1;	// ldo en ����
	else
		P10 = 0;	// ldo en ����
}

void mcu_gpio_light_led(bool light)
{
	// ע�⣺ʹ��P14�����Pxn_PDIO�Ĵ����͹���IO���ֲſ��ԣ�ʹ��GPIO_SetBits()��GPIO_ClearBits()�ӿڲ���DOUT�Ĵ����ᵼ�µ͹���IO�����쳣
	if(light)
		P14 = 0;	// led ��
	else
		P14 = 1;	// led ��
}

void mcu_gpio_en_pow(bool en)
{
	if(en)
		P15 = 1;	// POW_EN ���߱��ֹ���
	else
		P15 = 0;	// POW_EN �����µ�
}

bool mcu_gpio_key_pressed(void)
{
	if(GPIO_GET_IN_DATA(P3) & BIT0)
		return TRUE;	// ����
	else
		return FALSE;	// δ����
}

////////////////////////////////////////////adc_driver/////////////////////////////////////////////
static uint8 mcuAdcTableNum = 0;
static MCU_ADC_TAB * p_mcuAdcTable = NULL;
static volatile uint16 mcuAdcUserChIdx = 0;	// ��ǰ�ɼ����Table������Ϣ��p_mcuAdcTable[mcuAdcUserChIdx]
static bool mcuAdcIsInited = FALSE;

/**
 * @brief ����ָ��ͨ���ĵ���ת��
 * @param channel adcͨ��
 */
static void mcu_adc_start_channel_convert(ADC_CHANNEL channel)
{
    ADC_Open(ADC, 0, 0, 0x01 << channel);	// Enable channel
	ADC_START_CONV(ADC);
}

/**
 * @brief adc��ʼ��
 * @param p_table adc���ñ��ַ
 * @param tableNum adc���ñ�Ԫ�ظ���
 */
void mcu_adc_init(MCU_ADC_TAB *p_table, uint8 tableNum)
{
	uint8 i = 0;
	mcuAdcTableNum = tableNum;
	p_mcuAdcTable = p_table;

	// gpio init
	for(i = 0; i < mcuAdcTableNum; i++)
	{
		// ��Ҫ�ⲿʹ��gpio��ʱ��
		GPIO_SetMode(p_table[i].gpio, p_table[i].pinMask, GPIO_MODE_INPUT);
		GPIO_PullUp(p_table[i].gpio, p_table[i].pinMask, GPIO_PULLUP_DISABLE);
		GPIO_DISABLE_DIGITAL_PATH(p_table[i].gpio, p_table[i].pinMask);
		*p_table[i].mfpReg &= ~p_table[i].mfpMask;
		*p_table[i].mfpReg |= p_table[i].mfpAdcCh;
	}
	
	// ע���жϴ�������Э��ջ�������жϻῨ����
	((interrupt_register_handler)SVC_interrupt_register)(ADC_IRQ, mcu_adc_isr);

	CLK_EnableModuleClock(ADC_MODULE);
    // Select ADC input range.1 means 0.4V~2.4V ;0 means 0.4V~1.4V.
    // 0.4V~2.4V & 0.4V~1.4V both are theoretical value,the real range is determined by bandgap voltage.
    ADC_SelInputRange(ADC_INPUTRANGE_HIGH);
	// Set Sample Time 16_ADC_CLOCK
	ADC_SetExtraSampleTime(ADC, 0, ADC_SAMPLE_CLOCK_16);
    // Power on ADC
    ADC_POWER_ON(ADC);
    // Enable ADC convert complete interrupt
    ADC_EnableInt(ADC, ADC_ADIF_INT);
    NVIC_EnableIRQ(ADC_IRQn);

	mcuAdcUserChIdx = 0;
	mcu_adc_start_channel_convert(p_mcuAdcTable[mcuAdcUserChIdx].adcChannel);
	mcuAdcIsInited = TRUE;
}

/**
 * @brief adc����ʼ��
 */
void mcu_adc_deinit(void)
{
	CLK_DisableModuleClock(ADC_MODULE);
    ADC_POWER_DOWN(ADC);
    NVIC_DisableIRQ(ADC_IRQn);
	mcuAdcIsInited = FALSE;
}

/**
 * @brief adc��ȡ��ѹֵ
 * 
 * @param index adc���ñ��е�Ԫ������
 * @return float ��ѹֵ unit:V
 */
float mcu_adc_get_voltage(uint8 index)
{
	if(NULL==p_mcuAdcTable || index>=mcuAdcTableNum)
		return 0;
	return p_mcuAdcTable[index].voltage;
}

#define ADC_SAMPLE_TIMES_CFG 20
/**
 * @brief adc�ɼ�һ�ֺ���ƽ�������ѹ��������һ��
 * @return ����0��ʾδ����������-1��ʾ�쳣������1��ʾ��������
 * @note ����ͨ������ƽ����ADC_SAMPLE_TIMES_CFG�κ���ADC���͹�������
 */
int8 mcu_adc_main(void)
{
	uint8 i = 0;
	bool adcReady = TRUE;
	
	if(NULL == p_mcuAdcTable)
		return -1;	// δ��ʼ��
	if(FALSE == mcuAdcIsInited)
		return 1;	// �ѹر�

	if(mcuAdcUserChIdx >= mcuAdcTableNum)
	{
		for(i = 0; i < mcuAdcTableNum; i++)
		{
			p_mcuAdcTable[i].filterCnt++;
			p_mcuAdcTable[i].adcCodeSum += p_mcuAdcTable[i].adcCode;
			if(p_mcuAdcTable[i].filterCnt >= p_mcuAdcTable[i].filterLen)	// �Ƿ�ɹ���ƽ�����ۼӴ���
			{
				p_mcuAdcTable[i].voltage = p_mcuAdcTable[i].adcCodeSum / p_mcuAdcTable[i].filterCnt * p_mcuAdcTable[i].convRate;
				p_mcuAdcTable[i].rollCount++;	// ÿƽ��1��rc++
				p_mcuAdcTable[i].filterCnt = 0;
				p_mcuAdcTable[i].adcCodeSum = 0;
			}
		}
		mcuAdcUserChIdx = 0;
		for(i = 0; i < mcuAdcTableNum; i++)
			if(p_mcuAdcTable[i].rollCount < ADC_SAMPLE_TIMES_CFG)	// �����������õĴ���
				adcReady = FALSE;
		if(adcReady == FALSE)
			mcu_adc_start_channel_convert(p_mcuAdcTable[mcuAdcUserChIdx].adcChannel);
		else {
			mcu_adc_deinit();	// ����ͨ���Ѳ��꣬��ADC
			return 1;
		}
	}
	return 0;
}

/**
 * @brief adc�жϻص�
 */
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
	}
}


////////////////////////////////////////////adc_user/////////////////////////////////////////////
#define MCU_AVDD_CFG 2.5f
const MCU_ADC_TAB adcCfgTable[] = {
	{ADC_CH02, 10, MCU_AVDD_CFG/4096, P1, BIT2, &SYS->P1_MFP, SYS_MFP_P12_Msk, SYS_MFP_P12_ADC_CH2},
	{ADC_CH03, 10, MCU_AVDD_CFG/4096, P1, BIT3, &SYS->P1_MFP, SYS_MFP_P13_Msk, SYS_MFP_P13_ADC_CH3},
};
MCU_ADC_TAB adcTable[ARRAY_NUM(adcCfgTable)];

void mcu_adc_user_init(void)
{
	// init global var
	memset(adcTable, 0, sizeof(adcTable));
	memcpy(adcTable, adcCfgTable, sizeof(adcTable));
	// adc init
	mcu_adc_init(adcTable, ARRAY_NUM(adcTable));
}
