#include "canteros.h"

static uint16_t PRIMARY_ADDRESS;
static uint8_t ADDRESSES_SIZE;
static const uint16_t* myAddresses;
static uint8_t HEADERS_SIZE;
static const uint8_t* handledHeaders;

NodeStatusMessage sendingNodeStatus;

#define CANTEROS_MESSAGE_QUEUE_SIZE 16
typedef struct {
	uint8_t head, tail;
	CanterosMessage messages[CANTEROS_MESSAGE_QUEUE_SIZE];
} CanterosMessageQueue;
static CanterosMessageQueue queue;


static bool isHandled(uint16_t receiver, uint8_t header)
{
	bool ok = false;
	for (int i = 0; i < ADDRESSES_SIZE; ++i)
		if (receiver == myAddresses[i])
			ok = true;
	if (!ok)
		return false;

	ok = false;
	for (int i = 0; i < HEADERS_SIZE; ++i)
		if (header == handledHeaders[i])
			ok = true;
	return ok;
}

void configCanteros(uint16_t primary, uint8_t addressesSize, const uint16_t* addresses,
                    uint8_t headersSize, const uint8_t* headers)
{
	PRIMARY_ADDRESS = primary;
	ADDRESSES_SIZE = addressesSize;
	myAddresses = addresses;
	HEADERS_SIZE = headersSize;
	handledHeaders = headers;
	configCanAddresses(addressesSize, addresses);
}

int canterosInit()
{
	queue.head = 0;
	queue.tail = 0;
	return 0;
}

bool canterosProcess()
{
	static uint32_t lastNodeStatusMs = 0;
	uint32_t nowMs = HAL_GetTick();
	if (nowMs - lastNodeStatusMs > 1000) {
		lastNodeStatusMs = nowMs;
		sendingNodeStatus.uptimeSeconds = nowMs / 1000;

		CanterosMessage toSend;
		toSend.header = NODE_STATUS_HEADER;
		toSend.payloadSize = 7;
		toSend.payload[0] = PRIMARY_ADDRESS >> 3;
		toSend.payload[1] = PRIMARY_ADDRESS << 5;
		toSend.payload[2] = (sendingNodeStatus.uptimeSeconds & 0x00ff0000) >> 16;
		toSend.payload[3] = (sendingNodeStatus.uptimeSeconds & 0x0000ff00) >> 8;
		toSend.payload[4] = (sendingNodeStatus.uptimeSeconds & 0x000000ff) >> 0;
		toSend.payload[5] = (sendingNodeStatus.health << 6) | (sendingNodeStatus.mode << 3) | (sendingNodeStatus.submode);
		toSend.payload[6] = sendingNodeStatus.deviceSpecificCode;
		HAL_StatusTypeDef res = canterosSendMessage(&toSend);
		if (res) {
			//Error in sending status
			led(2, 1);
		}
	}

	bool gotNew = false;
	while (canCheckInbox(can1Handle)) {
		CanMessage incoming;
		if (canGetNew(can1Handle, &incoming) == 0 && incoming.dlc > 0 &&
		    isHandled(incoming.id, incoming.data[0])) {
			CanterosMessage* m = &(queue.messages[queue.tail]);
			m->receiver = incoming.id;
			m->header = incoming.data[0];
			m->payloadSize = incoming.dlc - 1;
			for (int i = 0; i < m->payloadSize; ++i)
				m->payload[i] = incoming.data[i + 1];
			++queue.tail;
			queue.tail %= CANTEROS_MESSAGE_QUEUE_SIZE;
			gotNew = true;
		}
	}
	return gotNew;
}

HAL_StatusTypeDef canterosSendMessage(const CanterosMessage* toSend)
{
	if (toSend->payloadSize > 7)
		return HAL_ERROR;

	CanMessage toSendLow;
	toSendLow.id = PRIMARY_ADDRESS | (1 << 10);
	toSendLow.dlc = toSend->payloadSize + 1;
	toSendLow.data[0] = toSend->header;
	for (int i = 0; i < toSend->payloadSize; ++i)
		toSendLow.data[i + 1] = toSend->payload[i];

	return canSendMessage(can1Handle, &toSendLow);
}

int canterosCheckInbox()
{
	return (CANTEROS_MESSAGE_QUEUE_SIZE + queue.tail - queue.head) % CANTEROS_MESSAGE_QUEUE_SIZE;
}

int canterosGetNew(CanterosMessage* toStore)
{
	if (canterosCheckInbox() <= 0)
		return -1;

	*toStore = queue.messages[queue.head];

	++queue.head;
	queue.head %= CANTEROS_MESSAGE_QUEUE_SIZE;

	return 0;
}

