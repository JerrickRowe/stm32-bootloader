#include "bsp_power.h"

#include "stm32f4xx.h"

#define POWER_EN_Pin	GPIO_PIN_11
#define POWER_EN_Port	GPIOD
#define POWER_EN_ON		GPIO_PIN_SET
#define POWER_EN_OFF	GPIO_PIN_RESET

#define EXT_POWER_ADC_RCC_ENABLE()			__HAL_RCC_ADC1_CLK_ENABLE()
#define EXT_POWER_ADC_GPIO_RCC_ENABLE()		__HAL_RCC_GPIOC_CLK_ENABLE()
#define EXT_POWER_ADC_GPIO_PORT				GPIOC
#define EXT_POWER_ADC_GPIO_PIN				GPIO_PIN_0
#define EXT_POWER_ADC_CHANNEL				ADC_CHANNEL_10
#define EXT_POWER_ADC_AVG_LENGTH			10
#define EXT_POWER_ADC_FACTOR				(3.3f/491.4f)
#define EXT_POWER_ADC_OFFSET				(0.0f)

void bsp_power_HoldPower( void ){
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin   = POWER_EN_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(POWER_EN_Port, &GPIO_InitStruct);
	HAL_GPIO_WritePin(POWER_EN_Port,POWER_EN_Pin,POWER_EN_ON);
}

void bsp_power_ReleasePower( void ){
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin   = POWER_EN_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(POWER_EN_Port, &GPIO_InitStruct);
	HAL_GPIO_WritePin(POWER_EN_Port,POWER_EN_Pin,POWER_EN_OFF);
}



float bsp_power_GetExtPowerVoltage( void ){
	float ret = 0.0f;

	ADC_HandleTypeDef adc_handle;

	GPIO_InitTypeDef gpio_init_struct;
	EXT_POWER_ADC_GPIO_RCC_ENABLE();
	gpio_init_struct.Pin = EXT_POWER_ADC_GPIO_PIN;
	gpio_init_struct.Mode = GPIO_MODE_ANALOG;
	gpio_init_struct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init( EXT_POWER_ADC_GPIO_PORT, &gpio_init_struct );

	EXT_POWER_ADC_RCC_ENABLE();
	adc_handle.Instance = ADC1;
	adc_handle.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	adc_handle.Init.Resolution = ADC_RESOLUTION_12B;
	adc_handle.Init.ScanConvMode = DISABLE;
	adc_handle.Init.ContinuousConvMode = DISABLE;
	adc_handle.Init.DiscontinuousConvMode = ENABLE;
	adc_handle.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	adc_handle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	adc_handle.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	adc_handle.Init.NbrOfConversion = 1;
	adc_handle.Init.NbrOfDiscConversion = 1;
	HAL_ADC_Init( &adc_handle );

	uint32_t adc_sum=0, adc_avg=0, adc_cnt=EXT_POWER_ADC_AVG_LENGTH;
	
	ADC_ChannelConfTypeDef adc_channdel_cfg;
	adc_channdel_cfg.Channel = EXT_POWER_ADC_CHANNEL;
	adc_channdel_cfg.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	adc_channdel_cfg.Rank = 1;
	HAL_ADC_ConfigChannel( &adc_handle, &adc_channdel_cfg );
	// Dump invalid ADC values 
	adc_cnt=3;
	while( adc_cnt--  ){
		HAL_ADC_Start( &adc_handle );
		HAL_ADC_PollForConversion( &adc_handle, 200 );
	}
	// Get average ADC value
	adc_cnt=EXT_POWER_ADC_AVG_LENGTH;
	while( adc_cnt--  ){
		HAL_ADC_Start( &adc_handle );
		if( HAL_ADC_PollForConversion( &adc_handle, 200 ) != HAL_OK ){
			return (-1.0f);
		}
		adc_sum += HAL_ADC_GetValue( &adc_handle );
	}
	HAL_ADC_Stop( &adc_handle );
	adc_avg = adc_sum / EXT_POWER_ADC_AVG_LENGTH;
	ret = adc_avg*EXT_POWER_ADC_FACTOR + EXT_POWER_ADC_OFFSET;

	return ret;
}


