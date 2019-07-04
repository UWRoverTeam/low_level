#include "globals.h"

CAN_HandleTypeDef *can1Handle;
CAN_HandleTypeDef *can2Handle;

void led(uint8_t n, uint8_t state)
{
	if (n == 1)
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, !state);
	else if (n == 2)
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, !state);
	else if (n == 3)
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, !state);
}

void blink(uint8_t l, int n)
{
	for (int i = 0; i < n; ++i) {
		led(l, 1);
		HAL_Delay(20);
		led(l, 0);
		HAL_Delay(200);
	}
}
