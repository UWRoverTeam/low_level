#include "adc.h"

static uint16_t measured1, measured2;
volatile static bool complete1, complete2;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if (hadc == adc1Handle)
		complete1 = true;
	else if (hadc == adc2Handle)
		complete2 = true;
}

void adcInit()
{
	HAL_ADC_Start_IT(adc1Handle);
	HAL_ADC_Start_IT(adc2Handle);
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
	if (complete2) {
		complete2 = false;
		measured2 = HAL_ADC_GetValue(adc2Handle);
		HAL_ADC_Start_IT(adc2Handle);
		ret += 2;
	}
	return ret;
}

uint16_t adcGet1()
{
	return measured1;
}

uint16_t adcGet2()
{
	return measured2;
}
