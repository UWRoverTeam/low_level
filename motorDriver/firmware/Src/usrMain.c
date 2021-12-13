#include "usrMain.h"

#include "adc.h"
#include "can.h"
#include "canteros.h"
#include "globals.h"
#include "motor.h"
#include "pid.h"

#define PRIMARY_ADDRESS 196
#define ADDRESSES_SIZE 2
static const uint16_t myAddresses[ADDRESSES_SIZE] = {
	180, //all arm actuators
	PRIMARY_ADDRESS
};
#define HEADERS_SIZE 8
static const uint8_t handledHeaders[HEADERS_SIZE] = {
	7, 17, 8, 18, 38, 9, 19, //power, position, speed
	20 //request current
};

typedef enum {
	OFF, POWER, POSITION
} Mode;

#define ENC_CNT_SCALE 39999
static uint16_t ENC_CNT;
volatile static uint16_t indexPulsePosition = 0xffff;
static Mode modeNow = OFF;

#define POSITION_KP 5.0f
#define POSITION_KI 0.0f
#define POSITION_KD 0.005f
static PidData positionPid;

#define PID_REVERSED false

#if PRIMARY_ADDRESS == 196 //base
 #define POSITION_KP 1.0f
 #define POSITION_KI 0.0f
 #define POSITION_KD 0.05f
#elif PRIMARY_ADDRESS == 195 //gripper_latitude
 #define POSITION_KP -1.0f
 #define POSITION_KI 0.0f
 #define POSITION_KD -0.01f
#elif PRIMARY_ADDRESS == 190 //arm_upper
 #define POSITION_KP -5.0f
 #define POSITION_KI 0.0f
 #define POSITION_KD -0.005f
#elif PRIMARY_ADDRESS == 188 //arm_lower
 #define POSITION_KP 5.0f
 #define POSITION_KI 0.0f
 #define POSITION_KD 0.005f
#else
 #define POSITION_KP 0.0f
 #define POSITION_KI 0.0f
 #define POSITION_KD 0.0f
#endif

static uint16_t positionInfoPeriodMs = 0;

static inline void handleMessage(const CanterosMessage* m)
{
	//-----Power-----//
	if (m->header == 7) { //set power
		modeNow = POWER;
		pidStop(&positionPid);
		int16_t power = ((int16_t)m->payload[0] << 8) | m->payload[1];
#if PRIMARY_ADDRESS == 192 || PRIMARY_ADDRESS == 193
		power /= 2;
#endif
		motorSetPower((int32_t)power * 1000 / -INT16_MIN);
		led(1, power == 0 ? 0 : 1);
	//-----Position-----//
	} else if (m->header == 8) { //set position
		uint16_t position = (m->payload[0] << 8) | m->payload[1];
		position = CLAMP_VALUE(position, 100, 3500);

		positionPid.setpoint = ((int32_t)position * ENC_CNT_SCALE / 3600);

		modeNow = POSITION;
		if (!positionPid.on) {
			positionPid.output = 0;
			pidStart(&positionPid);
		}
	} else if (m->header == 18) { //request position
		if (m->payload[0] == 0)
			positionInfoPeriodMs = 0;
		else
			positionInfoPeriodMs = 1000 / m->payload[0];
	} else if (m->header == 38) { //config position
		uint16_t position = (m->payload[0] << 8) | m->payload[1];

		ENC_CNT = ((int32_t)position * ENC_CNT_SCALE / 3600);

		CanMessage toSend;
		toSend.id = 1;
		toSend.dlc = 2;
		*((uint16_t*)toSend.data) = ENC_CNT;
		if (canSendMessage(can2Handle, &toSend))
			; //error sending config
	} else if (m->header == 35) { //P parameter
		int32_t int_data = m->payload[0]<<8 | m->payload[1];
		int_data = int_data << 16 | (m->payload[2]<<8 | m->payload[3]);
		positionPid.Kp = int_data / 10000.0f;
	} else if (m->header == 36) { //I parameter
		int32_t int_data = m->payload[0]<<8 | m->payload[1];
		int_data = int_data << 16 | (m->payload[2]<<8 | m->payload[3]);
		positionPid.Ki = int_data / 10000.0f;
	} else if (m->header == 37) { //D parameter
		int32_t int_data = m->payload[0]<<8 | m->payload[1];
		int_data = int_data << 16 | (m->payload[2]<<8 | m->payload[3]);
		positionPid.Kd = int_data / 10000.0f;
	}
}

int usrMain()
{
	memset(&positionPid, 0, sizeof(positionPid));
	positionPid.Kp = POSITION_KP;
	positionPid.Ki = POSITION_KI;
	positionPid.Kd = POSITION_KD;
	positionPid.outputMax = 1000.f;
	positionPid.outputMin = -1000.f;
	positionPid.period = 1;

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

	adcInit();
	motorInit();
	configCanteros(PRIMARY_ADDRESS, ADDRESSES_SIZE, myAddresses,
                   HEADERS_SIZE, handledHeaders);
	canInit(CANMODE_CANTEROS, CANMODE_RAW);
	canterosInit();

	uint16_t res1, res2;

	uint32_t lastReceivedMessageMs = HAL_GetTick();
	const uint32_t MOTOR_TIMEOUT_MS = 500, RESET_TIMEOUT_MS = UINT32_MAX;
	uint32_t lastSentPosInfoMs = HAL_GetTick();
	while (1) {
		int u = adcUpdate();
		if (u & 1) {
			res1 = adcGet1();
			(void)res1;
		}
		if (u & 2) {
			res2 = adcGet2();
			(void)res2;
		}

		canterosProcess();
		if (canterosCheckInbox()) {
			lastReceivedMessageMs = HAL_GetTick();
			CanterosMessage message;
			canterosGetNew(&message);
			handleMessage(&message);
		}

		if (canCheckInbox(can2Handle)) {
			CanMessage received;
			canGetNew(can2Handle, &received);
			ENC_CNT = *((uint16_t*)received.data);
			indexPulsePosition = *((uint16_t*)received.data + 1);
		}

		if (modeNow == POSITION) {
			positionPid.input = ENC_CNT;
			if (pidCompute(&positionPid))
				motorSetPower(positionPid.output * (PID_REVERSED ? -1 : 1));
			led(1,1);
		}

		uint32_t nowMs = HAL_GetTick();
		if (nowMs - lastReceivedMessageMs > MOTOR_TIMEOUT_MS) {
			motorSetPower(0);
			led(1, 0);

			modeNow = OFF;
			pidStop(&positionPid);
		}
		if (positionInfoPeriodMs && nowMs - lastSentPosInfoMs > positionInfoPeriodMs) {
			lastSentPosInfoMs = nowMs;
			CanterosMessage toSend;
			toSend.header = 28;
			toSend.payloadSize = 4;
			uint16_t posToSend = ((int32_t)ENC_CNT * 3600 / ENC_CNT_SCALE);
			uint16_t indexToSend = ((int32_t)indexPulsePosition * 3600 / ENC_CNT_SCALE);
			toSend.payload[0] = posToSend >> 8;
			toSend.payload[1] = posToSend & 0xff;
			toSend.payload[2] = indexToSend >> 8;
			toSend.payload[3] = indexToSend & 0xff;
			HAL_StatusTypeDef res = canterosSendMessage(&toSend);
			if (res) {
				//Error in sending status
				led(2, 1);
			}
		}
		if (nowMs - lastReceivedMessageMs > RESET_TIMEOUT_MS)
			HAL_NVIC_SystemReset();
	}
	return 0;
}
