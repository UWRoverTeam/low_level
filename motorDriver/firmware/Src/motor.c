#include "motor.h"

#ifdef USE_POLOLU_DRIVER

static void setPermille(uint16_t permille)
{
	//TODO remap for freq here
	__HAL_TIM_SET_COMPARE(tim3Handle, TIM_CHANNEL_1, permille);
}

void motorInit()
{
	setPermille(0);
	HAL_TIM_PWM_Start(tim3Handle, TIM_CHANNEL_1);
	HAL_GPIO_WritePin(POLOLU_SLEEP_GPIO_Port, POLOLU_SLEEP_Pin, 1);
}

int motorSetPower(int permille)
{
	if (permille < -1000 || permille > 1000)
		return -1;
	if (permille < 0) {
		HAL_GPIO_WritePin(POLOLU_DIR_GPIO_Port, POLOLU_DIR_Pin, 1);
		permille = -permille;
	} else {
		HAL_GPIO_WritePin(POLOLU_DIR_GPIO_Port, POLOLU_DIR_Pin, 0);
	}
	setPermille(permille);
	return 0;
}

bool getFault()
{
	return !HAL_GPIO_ReadPin(POLOLU_FAULT_GPIO_Port, POLOLU_FAULT_Pin);
}

#endif
