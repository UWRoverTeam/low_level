#ifndef CANTEROS_H_INCLUDED
#define CANTEROS_H_INCLUDED

#include "can.h"
#include "canterosDefs.h"
#include "globals.h"

typedef struct {
	uint16_t receiver;
	uint8_t header;
	uint8_t payloadSize;
	uint8_t payload[6];
} CanterosMessage;

extern NodeStatusMessage sendingNodeStatus;

void configCanteros(uint16_t primary, uint8_t addressesSize, const uint16_t* addresses,
                    uint8_t headersSize, const uint8_t* headers);
int canterosInit();
bool canterosProcess();

HAL_StatusTypeDef canterosSendMessage(const CanterosMessage* toSend);
int canterosCheckInbox();
int canterosGetNew(CanterosMessage* toStore);

#endif //CANTEROS_H_INCLUDED
