#ifndef CAN_H_INCLUDED
#define CAN_H_INCLUDED

#include "globals.h"

typedef struct {
	uint16_t id;
	uint8_t dlc;
	uint8_t data[8];
} CanMessage;

typedef enum {
	CANMODE_OFF,
	CANMODE_RAW,
	CANMODE_CANTEROS
} CanMode;

void configCanAddresses(uint8_t addressesSize, const uint16_t* addresses); //to be called from canteros function
int canInit(CanMode can1, CanMode can2); //canteros on can1 only, this does not initialize hardware
HAL_StatusTypeDef canSendMessage(CAN_HandleTypeDef* hcan, const CanMessage* toSend);
int canCheckInbox(CAN_HandleTypeDef* hcan);
int canGetNew(CAN_HandleTypeDef* hcan, CanMessage* toStore);

#endif //CAN_H_INCLUDED
