#include "usrMain.h"

#include "adc.h"
#include "can.h"
#include "globals.h"

#define ENC_CNT (tim2Handle->Instance->CNT)

static uint32_t sendPeriodMs = 1;
volatile uint16_t indexPulsePosition = 0xffff;

void interruptExternal2()
{
	indexPulsePosition = ENC_CNT;
}

int usrMain()
{
	for (int i = 1; i <= 3; ++i)
		led(i, 0);
	HAL_Delay(500);

	for (int i = 1; i <= 3; ++i) {
		led(i, 1);
		HAL_Delay(100);
	}
	for (int i = 3; i >= 1; --i) {
		led(i, 0);
		HAL_Delay(100);
	}

	canInit(CANMODE_RAW, CANMODE_OFF);
	HAL_TIM_Encoder_Start(tim2Handle, TIM_CHANNEL_ALL);

	CanMessage toSend;
	toSend.id = 100;
	toSend.dlc = 4;
	uint32_t lastSendMs = HAL_GetTick();
	while (1) {
		if (canCheckInbox(can1Handle)) {
			CanMessage received;
			canGetNew(can1Handle, &received);
			uint16_t new_ENC_CNT = *((uint16_t*)received.data);
			int newIndexPulsePosition = 
				indexPulsePosition + new_ENC_CNT - ENC_CNT;
			if (newIndexPulsePosition > 0 && newIndexPulsePosition <= 39999)
				indexPulsePosition = newIndexPulsePosition;
			else
				indexPulsePosition = 0xffff;
			ENC_CNT = *((uint16_t*)received.data);
		}

		if (ENC_CNT / 200 % 2)
			led(1,1);
		else
			led(1,0);

		if (indexPulsePosition != 0xffff)
			led(2,1);
		else
			led(2,0);

		uint32_t nowMs = HAL_GetTick();
		if (nowMs - lastSendMs > sendPeriodMs) {
			lastSendMs = nowMs;
			*((uint16_t*)toSend.data) = ENC_CNT;
			*((uint16_t*)toSend.data + 1) = indexPulsePosition;
			canSendMessage(can1Handle, &toSend);
		}
	}
	return 0;
}
/*
#include "usrMain.h"

#include "adc.h"
#include "can.h"
#include "globals.h"

#define ENC_CNT (tim2Handle->Instance->CNT)

static uint32_t sendPeriodMs = 1;

int usrMain()
{
	for (int i = 1; i <= 3; ++i)
		led(i, 0);
	HAL_Delay(500);

	for (int i = 1; i <= 3; ++i) {
		led(i, 1);
		HAL_Delay(100);
	}
	for (int i = 3; i >= 1; --i) {
		led(i, 0);
		HAL_Delay(100);
	}

	canInit(CANMODE_RAW, CANMODE_OFF);
	HAL_TIM_Encoder_Start(tim2Handle, TIM_CHANNEL_ALL);

	CanMessage toSend;
	toSend.id = 100;
	toSend.dlc = 2;
	uint32_t lastSendMs = HAL_GetTick();
	while (1) {
		if (canCheckInbox(can1Handle)) {
			CanMessage received;
			canGetNew(can1Handle, &received);
			ENC_CNT = *((uint16_t*)received.data);
		}

		if (ENC_CNT / 2000 % 2)
			led(1,1);
		else
			led(1,0);

		if (ENC_CNT / 200 % 2)
			led(2,1);
		else
			led(2,0);

		uint32_t nowMs = HAL_GetTick();
		if (nowMs - lastSendMs > sendPeriodMs) {
			lastSendMs = nowMs;
			*((uint16_t*)toSend.data) = ENC_CNT;
			canSendMessage(can1Handle, &toSend);
		}
	}
	return 0;
}

*/