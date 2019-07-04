#ifndef VESC_H
#define VESC_H

#include "globals.h"

#define VESC_COMMAND_GET_VALUES 4
#define VESC_COMMAND_SET_DUTY 5
#define VESC_COMMAND_SET_CURRENT 6
#define VESC_COMMAND_SET_CURRENT_BRAKE 7
#define VESC_COMMAND_SET_RPM 8
#define VESC_COMMAND_REBOOT 28
#define VESC_COMMAND_ALIVE 29

typedef enum {
	VESC_FAULT_NONE = 0,
	VESC_FAULT_OVER_VOLTAGE,
	VESC_FAULT_UNDER_VOLTAGE,
	VESC_FAULT_DRV_FAULT,
	VESC_FAULT_ABS_OVER_CURRENT,
	VESC_FAULT_OVER_TEMP_FET,
	VESC_FAULT_OVER_TEMP_MOTOR
} VescFault;

typedef struct {
	int16_t tMosf[6]; //degrees C * 0.1
	int16_t tPcb; //degrees C * 0.1
	int32_t currentMotor; //mA
	int32_t currentBatt; //mA
	int16_t dutyNow; //pwm duty in permille
	int32_t erpm;
	int16_t vIn; //volts * 0.1
	int32_t ampHours; //mAh * 0.1
	int32_t ampHoursCharged; //mAh * 0.1
	int32_t wattHours; //mWh * 0.1
	int32_t wattHoursCharged; //mWh * 0.1
	int32_t tacho;
	int32_t tachoAbs;
	VescFault fault;
} VescValues;

extern uint8_t vescNewValuesReady;
extern VescValues vescLastValues;

void vescInit();
HAL_StatusTypeDef vescRequestValues();

HAL_StatusTypeDef vescSetPwm(int16_t permille);
HAL_StatusTypeDef vescSetCurrent(int32_t mA);
HAL_StatusTypeDef vescSetERPM(int32_t erpm);

void vescNextByte(uint8_t byte); //to be called from uart interrupt

#endif //VESC_H
