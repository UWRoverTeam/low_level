#include "adc.h"

static uint16_t measured1;
volatile static bool complete1;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if (hadc == adc1Handle)
		complete1 = true;
}

void adcInit()
{
	HAL_ADC_Start_IT(adc1Handle);
}

int adcUpdate()
{
	int ret = 0;
	if (complete1) {
		complete1 = false;
		measured1 = HAL_ADC_GetValue(adc1Handle);
		HAL_ADC_Start_IT(adc1Handle);
		ret += 1;
	}
	return ret;
}

uint16_t adcGet1()
{
	return measured1;
}
