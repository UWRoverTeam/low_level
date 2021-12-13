#include "usrMain.h"

#include "can.h"
#include "canteros.h"
#include "globals.h"
#include "pid.h"
#include "vesc.h"

#define PRIMARY_ADDRESS 204
#define ADDRESSES_SIZE 1
static const uint16_t myAddresses[ADDRESSES_SIZE] = {
	//128, //all drive wheels
	//129, //left drive wheels
	PRIMARY_ADDRESS
};
#define HEADERS_SIZE 8
static const uint8_t handledHeaders[HEADERS_SIZE] = {
	7, 17, 8, 18, 38, 9, 19, //power, position, speed
	20 //request current
};

#if PRIMARY_ADDRESS > 180
 #define ENCODER true
#else
 #define ENCODER true
#endif

const uint32_t MOTOR_TIMEOUT_MS = 500, RESET_TIMEOUT_MS = UINT32_MAX;

typedef enum {
	OFF, POWER, POSITION
} Mode;

#define ENC_CNT_SCALE 39999
volatile static uint16_t ENC_CNT;
volatile static uint16_t indexPulsePosition = 0xffff;
static Mode modeNow = OFF;

#define PID_REVERSED false 

#if PRIMARY_ADDRESS == 200 //base
 #define POSITION_KP -0.5f
 #define POSITION_KI 0.0f
 #define POSITION_KD -5.0f
#elif PRIMARY_ADDRESS == 201 //gripper_latitude
 #define POSITION_KP 1.0f
 #define POSITION_KI 0.0f
 #define POSITION_KD 1.0f
#else
 #define POSITION_KP 0.0f
 #define POSITION_KI 0.0f
 #define POSITION_KD 0.0f
#endif
static PidData positionPid;

static uint16_t positionInfoPeriodMs = 0;

uint16_t driverInfoPeriodMs = 0;

static inline void handleMessage(const CanterosMessage* m)
{
	//-----Power-----
	if (m->header == 7) { //set power
		modeNow = POWER;
		pidStop(&positionPid);
		int16_t power = ((int16_t)m->payload[0] << 8) | m->payload[1];
		vescSetPwm((int32_t)power * 1000 / -INT16_MIN);
		led(1, power == 0 ? 0 : 1);
	//-----Position-----
	} else if (ENCODER && m->header == 8) { //set position
		uint16_t position = (m->payload[0] << 8) | m->payload[1];
		position = CLAMP_VALUE(position, 100, 3500);

		positionPid.setpoint = ((int32_t)position * ENC_CNT_SCALE / 3600);

		modeNow = POSITION;
		if (!positionPid.on) {
			positionPid.output = 0;
			pidStart(&positionPid);
		}
	} else if (ENCODER && m->header == 18) { //request position
		if (m->payload[0] == 0)
			positionInfoPeriodMs = 0;
		else
			positionInfoPeriodMs = 1000 / m->payload[0];
	} else if (ENCODER && m->header == 38) { //config position
		uint16_t position = (m->payload[0] << 8) | m->payload[1];

		ENC_CNT = ((int32_t)position * ENC_CNT_SCALE / 3600);

		CanMessage toSend;
		toSend.id = 1;
		toSend.dlc = 2;
		*((uint16_t*)toSend.data) = ENC_CNT;
		if (canSendMessage(can2Handle, &toSend)) 
			; //error sending config
	} else if (ENCODER && m->header == 43) { //config P
		positionPid.Kp = *((float*)(m->payload));
	} else if (ENCODER && m->header == 44) { //config I
		positionPid.Ki = *((float*)(m->payload));
	} else if (ENCODER && m->header == 45) { //config D
		positionPid.Kd = *((float*)(m->payload));
	} else if (m->header == 20) { //request driver info
		if (m->payload[0] == 0)
			driverInfoPeriodMs = 0;
		else
			driverInfoPeriodMs = 1000 / m->payload[0];
	}
}

int usrMain()
{
	memset(&positionPid, 0, sizeof(positionPid));
	positionPid.Kp = POSITION_KP;
	positionPid.Ki = POSITION_KI;
	positionPid.Kd = POSITION_KD;
	positionPid.outputMax = 500.f; //base
	positionPid.outputMin = -500.f;
	positionPid.period = 10;

	for (int i = 1; i <= 3; ++i)
		led(i, 0);
	HAL_Delay(100);

	for (int i = 1; i <= 3; ++i) {
		led(i, 1);
		HAL_Delay(100);
	}
	for (int i = 3; i >= 1; --i) {
		led(i, 0);
		HAL_Delay(100);
	}

	configCanteros(PRIMARY_ADDRESS, ADDRESSES_SIZE, myAddresses, HEADERS_SIZE, handledHeaders);
	canInit(CANMODE_CANTEROS, ENCODER ? CANMODE_RAW : CANMODE_OFF);
	canterosInit();
	vescInit();
	HAL_Delay(100);
	sendingNodeStatus.health = HEALTH_OK;
	sendingNodeStatus.mode = MODE_OPERATIONAL;
	sendingNodeStatus.submode = 0;
	sendingNodeStatus.deviceSpecificCode = 0;

	uint32_t lastVescReqMs = HAL_GetTick();
	uint32_t lastReceivedMessageMs = HAL_GetTick();
	uint32_t lastSentInfoMs = HAL_GetTick();
	uint32_t lastSentPosInfoMs = HAL_GetTick();
	while (1) {
		canterosProcess();
		if (canterosCheckInbox()) {
			lastReceivedMessageMs = HAL_GetTick();
			CanterosMessage message;
			canterosGetNew(&message);
			handleMessage(&message);
		}

		if (ENCODER && canCheckInbox(can2Handle)) {
			CanMessage received;
			canGetNew(can2Handle, &received);
			ENC_CNT = *((uint16_t*)received.data);
			indexPulsePosition = *((uint16_t*)received.data + 1);
		}

		if (ENCODER && modeNow == POSITION) {
			positionPid.input = ENC_CNT;
			if (pidCompute(&positionPid))
				vescSetPwm(positionPid.output * (PID_REVERSED ? -1 : 1));
			led(1,1);
		}

		uint32_t nowMs = HAL_GetTick();
		if (nowMs - lastReceivedMessageMs > MOTOR_TIMEOUT_MS) {
			vescSetCurrent(0);
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
		if (nowMs - lastVescReqMs >= 100) {
			lastVescReqMs = nowMs;
			vescRequestValues();
		}
		if (driverInfoPeriodMs && nowMs - lastSentInfoMs > driverInfoPeriodMs) {
			lastSentInfoMs = nowMs;
			CanterosMessage toSend;
			toSend.header = 30;
			toSend.payloadSize = 7;
			memset(toSend.payload, 0, sizeof(toSend.payload));

			toSend.payload[0] = vescLastValues.tacho >> 24;
			toSend.payload[1] = vescLastValues.tacho >> 16;
			toSend.payload[2] = vescLastValues.tacho >> 8;
			toSend.payload[3] = vescLastValues.tacho;
			toSend.payload[4] = vescLastValues.vIn >> 8;
			toSend.payload[5] = vescLastValues.vIn;
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
