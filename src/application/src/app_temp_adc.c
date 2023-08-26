#include <stdio.h>
#include <string.h>
#include "PN102Series.h"
#include "panip_config.h"
#include "peripherals.h"
#include "panip.h"
#include "arch.h"
#include "app_task.h"
#include "app.h"
#include "ke_env.h"
#include "rf.h"
#include "app_flash.h"
#include "mesh_flash.h"
#include "flash_manager.h"
#include "app_temp_adc.h"
#include "stack_svc_api.h"


#if(TEMP_CHANGE_CALIB)
/***********  ADC Calibrate **************/
#define ADC_TEMP_CH_NUM  8
#define ADC_BGP_CH_NUM  9
#define ADC_BGP_SAMPLE_CNT 10
#define STD_DEVI_SUM  4000

#define ADC_BGP_LOWER_BOUND_D 1990
#define ADC_BGP_UPPER_BOUND_D 2110
#define ADC_TEMP_SLOPE_D   (10)

#define ADC_BGP_LOWER_BOUND_B 1580
#define ADC_BGP_UPPER_BOUND_B 1710
#define ADC_TEMP_SLOPE_B   (12)

//PAN1020 CHIP Version <0=> PAN1020-B <1=> PAN1020-D <2=> AutoDetect
#define PAN1020_CHIP_VERSION  2  

uint32_t trim_code;
int32_t temp_code;
int32_t temperature;
uint8_t m_updating_flag;
uint16_t temperature_count;
uint8_t last_index;
uint8_t adc_count;
uint8_t m_temp_index;

void adc_temp_param_init(void)
{
	trim_code = 0;
	temp_code = 0;
	temperature = 0;
	m_updating_flag = 0;
	temperature_count = 0;
	last_index = 0;
	adc_count = 0;
	m_temp_index = 0;
}


/***about ADC bandgap***/
uint32_t ADC_Read_TrimCode()
{
	SYS_UnlockReg();
	FMC_Open();
	
    uint32_t temp = 0; 
    temp = FMC_ReadUID(ADC_CALIB_ADDR);
    
	FMC_Close();
	SYS_LockReg();
	
	return (temp & TRIM_CODE_MASK);
}

void ADC_Apply_TrimCode(uint32_t trim_code)
{
    ANAC->LDO_CTL &= ~BDG_TRIM_MASK;
    ANAC->LDO_CTL |= (trim_code << BDG_TRIM_POS);  
    ANAC->LDO2_CTL &= ~(7 << 5);
    ANAC->LDO2_CTL |= (3 << 5); 	// change dvdd vol 
    
    ANAC->ANAC_3VCTL |= 0x1;
    while(ANAC->ANAC_3VCTL & 0x01);
}
/***about ADC bandgap***/


/***about ADC temperature***/
void ADC_Init_Temperature(void)
{	
	CLK_EnableModuleClock(ADC_MODULE);
	CLK_EnableModuleClock(ANAC_MODULE);
    //INIT TEMPERATURE
    ANAC->RCC_CTL |= (1<<21);
	ANAC->RCC_CTL &= ~(0x3 << 22); // minimize gain_temp 
    ANAC->ANAC_3VCTL |= 0x01;
	while (ANAC->ANAC_3VCTL & 0x01);
}

uint32_t ADC_Read_Temperature()
{
	SYS_UnlockReg();
	FMC_Open();
	
    uint32_t temp = 0; 
    temp = FMC_ReadUID(ADC_TEMP_ADDR);
    	
	FMC_Close();
	SYS_LockReg();
	
	return ((temp & ADC_TEMP_MASK)>>ADC_TEMP_POS);
}

uint32_t ADC_Read_TemperatureCode()
{
	SYS_UnlockReg();
	FMC_Open();
	
    uint32_t temp = 0; 
    temp = FMC_ReadUID(ADC_TEMP_ADDR);
    
	FMC_Close();
	SYS_LockReg();
	
	return (temp & ADC_TEMPCODE_MASK);
}
/***about ADC temperature***/

//temperature program
void temperature_init()
{
	ADC_Init_Temperature();
	/** ADC hardware calibration**/
    trim_code = ADC_Read_TrimCode();
    //printf("trim code:%d\n",trim_code);
    /*****************************/
	/** ADC temperatute calibration**/
	temp_code = ADC_Read_TemperatureCode();
	//printf("temperature code:%d\n",temp_code);
	temperature = ADC_Read_Temperature();
	//printf("temperature:%d 'C\n",temperature);
	/*****************************/
    ADC_Apply_TrimCode(trim_code);
	
	// Enable channel 8 (temperature channel)
	ADC_Open(ADC, 0, 0, BIT8);
	
	ADC_SelInputRange(ADC_INPUTRANGE_HIGH);
	
	// Power on ADC
	ADC_POWER_ON(ADC);
	
}

uint8_t chip_version_read(void)
{
	FMC_Open();
    SYS_UnlockReg(); 
    FMC_ENABLE_ISP(); 
    uint32_t data = FMC_ReadUID(0x400068);
    uint8_t  temp = ((data & 0x000F0000) >> 16);
	
	return temp;
}
       
bool Check_ADC_Work_State(void)
{
	uint32_t bak_ch_en = ADC->CHEN;
	uint8_t i = ADC_BGP_SAMPLE_CNT;
	uint32_t bgp;
	uint32_t std_devi = 0;
	uint32_t avg = 0;
	uint32_t  statistic[ADC_BGP_SAMPLE_CNT];
    
    #if (PAN1020_CHIP_VERSION == 2)
    uint32_t bgp_low_bound;
    uint32_t bgp_high_bound;    
    if(chip_version_read() == 0xD)
    {
        bgp_low_bound = ADC_BGP_LOWER_BOUND_D;
        bgp_high_bound = ADC_BGP_UPPER_BOUND_D;
    }else
    {
        bgp_low_bound = ADC_BGP_LOWER_BOUND_B;
        bgp_high_bound = ADC_BGP_UPPER_BOUND_B;
    }
    #endif
	
	ADC->CHEN = 0; //Disable All ADC channel
	ADC_Open(ADC, 0, 0, BIT9);
	
	while(i--)
	{
		ADC_START_CONV(ADC);
		
		while (!ADC_IS_DATA_VALID(ADC, BIT9));

		bgp = (uint32_t)ADC_GET_CONVERSION_DATA(ADC, ADC_BGP_CH_NUM);
		
//		printf("bgp:%d\n",bgp);

		statistic[i] = bgp;
		avg += bgp;
	}
	
	avg = avg/ADC_BGP_SAMPLE_CNT;
//    printf("avg:%d\n",avg);
	i = ADC_BGP_SAMPLE_CNT;
	while(i--)
	{
		uint32_t tmp = (statistic[i] - avg);
		
		std_devi += tmp*tmp;
	}
    
//	printf("std_devi:%d\n",std_devi);
    
	ADC->CHEN = bak_ch_en;
    
    #if (PAN1020_CHIP_VERSION == 2)
	if ((std_devi < STD_DEVI_SUM) && (bgp_low_bound < avg) && ( avg < bgp_high_bound))
    #elif (PAN1020_CHIP_VERSION == 1)
    if ((std_devi < STD_DEVI_SUM) && (ADC_BGP_LOWER_BOUND_D < avg) && ( avg < ADC_BGP_UPPER_BOUND_D))
    #elif (PAN1020_CHIP_VERSION == 0)
	if ((std_devi < STD_DEVI_SUM) && (ADC_BGP_LOWER_BOUND_B < avg) && ( avg < ADC_BGP_UPPER_BOUND_B))
    #endif    
	{
		return true;
	}

	return false;
}

static int read_temperature()
{
   #if (PAN1020_CHIP_VERSION == 2)
    int32_t adc_temp_slope;
    if(chip_version_read() == 0xD)
    {
        adc_temp_slope = ADC_TEMP_SLOPE_D;
    }else
    {
        adc_temp_slope = ADC_TEMP_SLOPE_B;
    }
    #endif
    
	uint32_t bak_ch_en = ADC->CHEN;
	ADC->CHEN = 0; //Disable All ADC channel
	
	// Enable channel 8 (temperature channel)
	ADC_Open(ADC, 0, 0, BIT8);
	
	ADC_START_CONV(ADC);
	//Poll conversion result
	while (!ADC_IS_DATA_VALID(ADC, BIT8))
	{
	}
	int32_t vtemp =  (uint32_t)ADC_GET_CONVERSION_DATA(ADC, 8);
	
	ADC_STOP_CONV(ADC);
	ADC->CHEN = bak_ch_en;
    
    #if (PAN1020_CHIP_VERSION == 2)
	return (temp_code-vtemp)/adc_temp_slope + temperature;
    #elif (PAN1020_CHIP_VERSION == 1)
	return (temp_code-vtemp)/ADC_TEMP_SLOPE_D + temperature;
    #elif (PAN1020_CHIP_VERSION == 0)
	return (temp_code-vtemp)/ADC_TEMP_SLOPE_B + temperature;
    #endif
}

uint8_t temperature_convert_flash_index(int temp)
{
	uint8_t c_temp = temp + 40;
	uint8_t index = c_temp/20 + 1;
	return index;
}

void rf_dev_cal_reinit()
{
	uint8_t r_da_gain,r_da_verf_lb,r_da_verf_mb,r_guass_scale;
	
	((rf_freq_dev_cal_result)SVC_rf_freq_dev_cal_result)(&r_da_gain,&r_da_verf_lb,&r_da_verf_mb,&r_guass_scale);		//两点式校正后的结果
	
	//printf("2 [%d],[%d],[%d],[%d],[%d] [%08x]\n",m_temp_index,r_da_gain,r_da_verf_lb,r_da_verf_mb,r_guass_scale,
	//		(r_da_gain|(r_da_verf_lb << 8)|(r_da_verf_mb << 16)|(r_guass_scale << 24)));
	dev_cal_flash_write(m_temp_index, (r_da_gain|(r_da_verf_lb << 8)|(r_da_verf_mb << 16)|(r_guass_scale << 24)));
	//clear [14:20]
	ANAC->TX_CTL &= ~ 0x1fc000;

	//set DA_GAIN, DA_VREF_LB, DA_VREF_MB
	ANAC->TX_CTL |= (r_da_gain << 14) | (r_da_verf_lb << 15) | (r_da_verf_mb << 18);

	ANAC->TP_CTL &= ~ 0xf;
	ANAC->TP_CTL |= r_guass_scale;
	
	//disable 2 piont calibration
	ANAC->TP_CTL = ANAC->TP_CTL & 0xfffffE7f;
	
	set_ui32_reg(ANAC->MCU_RF, 0x00000100);
	set_ui32_reg(ANAC->AGC_CTL, 0x00000fff);
}

void rf_calibration_process()
{
	
	int32_t temp,index;
	uint32_t dev_cal_data;
	uint8_t r_da_gain,r_da_verf_lb,r_da_verf_mb,r_guass_scale;
	
	if (!ADC_IS_BUSY(ADC))
	{
		if (Check_ADC_Work_State())
		{
			temp = read_temperature();
		}
		else
		{
			//printf("ADC Error State\n");
			return;
		}
	}else
	{
		return;
	}
	
	//printf("temp:%d\n",temp);

	if(temp < -40)
	{
		temp = -39;
	}
	else if((120<= temp)&& (temp <=150))
	{
		temp = 119;
	}
	else if(temp > 150)
	{
		return;
	}
	
	index = temperature_convert_flash_index(temp);
	if(temperature_count == 3000)
	{
		temperature_count = 0;
	}
	else
	{
		temperature_count ++;
	}
	if(index == last_index)
	{
		adc_count++;
	}
	else
	{
		last_index = index;
		adc_count = 0;
	}
	
	if((adc_count == 50) && (m_temp_index != index))
	{
		//printf("temp : %d\n",temp);
		m_temp_index = index;
		if(dev_cal_flash_read(index,&dev_cal_data) == PAN_ERROR_NOT_FOUND)
		{	
			rf_dev_cal_reinit();
		}
		else
		{
			r_da_gain 		= (dev_cal_data & 0xff);
			r_da_verf_lb 	= ((dev_cal_data >> 8) & 0xff);
			r_da_verf_mb 	= ((dev_cal_data >> 16) & 0xff);
			r_guass_scale 	= ((dev_cal_data >> 24) & 0xff);
			
			//clear [14:20]
			ANAC->TX_CTL &= ~ 0x1fc000;
			//printf("3 [%d] [%d],[%d],[%d],[%d] [%08x]\n",index,r_da_gain,r_da_verf_lb,r_da_verf_mb,r_guass_scale,dev_cal_data);
			//set DA_GAIN, DA_VREF_LB, DA_VREF_MB
			ANAC->TX_CTL |= (r_da_gain << 14) | (r_da_verf_lb << 15) | (r_da_verf_mb << 18);

			ANAC->TP_CTL &= ~ 0xf;
			ANAC->TP_CTL |= r_guass_scale;
			
			//disable 2 piont calibration
			ANAC->TP_CTL = ANAC->TP_CTL & 0xfffffE7f;
			
			set_ui32_reg(ANAC->MCU_RF, 0x00000100);
			set_ui32_reg(ANAC->AGC_CTL, 0x00000fff);
		}
		adc_count = 0;
	}
}

#endif


