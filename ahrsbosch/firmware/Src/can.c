#include "can.h"

static uint8_t ADDRESSES_SIZE;
static const uint16_t* myAddresses;

uint32_t lastCanError = HAL_CAN_ERROR_NONE;

#define CAN_MESSAGE_QUEUE_SIZE 16
typedef struct {
	uint8_t head, tail;
	CanMessage messages[CAN_MESSAGE_QUEUE_SIZE];
} CanMessageQueue;
volatile static CanMessageQueue queue1, queue2;

//HAL callbacks
void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* hcan)
{
// 	led(1,1);
}

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{
// 	led(2,1);

	volatile CanMessageQueue* q = (hcan == can1Handle) ? &queue1 : &queue2;

	q->messages[q->tail].id = hcan->pRxMsg->StdId;
	uint8_t bytes = hcan->pRxMsg->DLC;
	q->messages[q->tail].dlc = bytes;
	for (int i = 0; i < bytes; ++i)
		q->messages[q->tail].data[i] = hcan->pRxMsg->Data[i];

	++(q->tail);
	q->tail %= CAN_MESSAGE_QUEUE_SIZE;

	if (HAL_CAN_Receive_IT(hcan, CAN_FIFO0) != HAL_OK)
		led(2, 1);
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	lastCanError = hcan->ErrorCode;
	if (HAL_CAN_Receive_IT(hcan, CAN_FIFO0) != HAL_OK)
		led(3,1);
// 	_Error_Handler(__FILE__,__LINE__);
}

static void disableRX0Int(CAN_HandleTypeDef *hcan)
{
	HAL_NVIC_DisableIRQ((hcan == can1Handle) ? CAN1_RX0_IRQn : CAN2_RX0_IRQn);
	__NOP(); //maybe this is not needed, but eh
}
static void enableRX0Int(CAN_HandleTypeDef *hcan)
{
	HAL_NVIC_EnableIRQ((hcan == can1Handle) ? CAN1_RX0_IRQn : CAN2_RX0_IRQn);
}

static inline uint16_t shiftToLeft(uint16_t data)
{
	return (data << 5);
}

static void setupFilters1Canteros()
{
	//cant have more than 4 (i'm lazy and don't want to configure more than 1 bank)
	if (ADDRESSES_SIZE > 4)
		_Error_Handler(__FILE__,__LINE__);
	if (ADDRESSES_SIZE == 0) //no addresses means reject everything
		return;

	CAN_FilterConfTypeDef config;
	config.FilterNumber = 0;
	config.FilterMode = CAN_FILTERMODE_IDLIST;
	config.FilterScale = CAN_FILTERSCALE_16BIT;
	config.FilterFIFOAssignment = 0;
	config.FilterActivation = ENABLE;
	config.BankNumber = 14;

	//we always set 4 filters, if we have less addresses, end filters will be duplicated
	uint16_t nextAddr = shiftToLeft(myAddresses[0]);
	config.FilterIdHigh = nextAddr;

	if (ADDRESSES_SIZE > 1)
		nextAddr = shiftToLeft(myAddresses[1]);
	config.FilterIdLow = nextAddr;

	if (ADDRESSES_SIZE > 2)
		nextAddr = shiftToLeft(myAddresses[2]);
	config.FilterMaskIdHigh = nextAddr;

	if (ADDRESSES_SIZE > 3)
		nextAddr = shiftToLeft(myAddresses[3]);
	config.FilterMaskIdLow = nextAddr;

	if (HAL_CAN_ConfigFilter(can1Handle, &config) != HAL_OK)
		_Error_Handler(__FILE__,__LINE__);
}

static void setupFilters1Accept()
{
	CAN_FilterConfTypeDef config;
	config.FilterNumber = 0;
	config.FilterMode = CAN_FILTERMODE_IDMASK;
	config.FilterScale = CAN_FILTERSCALE_32BIT;
	config.FilterIdHigh = 0x0000;
	config.FilterIdLow = 0x0000;
	config.FilterMaskIdHigh = 0x0000;
	config.FilterMaskIdLow = 0x0000;
	config.FilterFIFOAssignment = 0;
	config.FilterActivation = ENABLE;
	config.BankNumber = 14;
	if (HAL_CAN_ConfigFilter(can1Handle, &config) != HAL_OK) {
		_Error_Handler(__FILE__,__LINE__);
	}
}

static void setupFilters2Accept()
{
	CAN_FilterConfTypeDef config;
	config.FilterNumber = 14;
	config.FilterMode = CAN_FILTERMODE_IDMASK;
	config.FilterScale = CAN_FILTERSCALE_32BIT;
	config.FilterIdHigh = 0x0000;
	config.FilterIdLow = 0x0000;
	config.FilterMaskIdHigh = 0x0000;
	config.FilterMaskIdLow = 0x0000;
	config.FilterFIFOAssignment = 0;
	config.FilterActivation = ENABLE;
	config.BankNumber = 14;
	//filter for CAN2 is managed by CAN1, number must be in slave filers
	if (HAL_CAN_ConfigFilter(can1Handle, &config) != HAL_OK) {
		_Error_Handler(__FILE__,__LINE__);
	}
}

void configCanAddresses(uint8_t addressesSize, const uint16_t* addresses)
{
	ADDRESSES_SIZE = addressesSize;
	myAddresses = addresses;
}

int canInit(CanMode can1, CanMode can2)
{
	queue1.head = 0;
	queue1.tail = 0;
	queue2.head = 0;
	queue2.tail = 0;

	static CanTxMsgTypeDef txMessage1, txMessage2;
	static CanRxMsgTypeDef rxMessage1, rxMessage2;
	if (can1 != CANMODE_OFF) {
		can1Handle->pTxMsg = &txMessage1;
		can1Handle->pRxMsg = &rxMessage1;
	}
	if (can2 != CANMODE_OFF) {
		can2Handle->pTxMsg = &txMessage2;
		can2Handle->pRxMsg = &rxMessage2;
	}

	if (can1 == CANMODE_CANTEROS && myAddresses == NULL)
		_Error_Handler(__FILE__, __LINE__);
	if (can1 == CANMODE_CANTEROS)
		setupFilters1Canteros();
	else if (can1 == CANMODE_RAW)
		setupFilters1Accept();

	if (can2 == CANMODE_RAW)
		setupFilters2Accept();

	if (can1 != CANMODE_OFF && HAL_CAN_Receive_IT(can1Handle, CAN_FIFO0))
		_Error_Handler(__FILE__,__LINE__);

	if (can2 != CANMODE_OFF && HAL_CAN_Receive_IT(can2Handle, CAN_FIFO0))
		_Error_Handler(__FILE__,__LINE__);

	return 0;
}

HAL_StatusTypeDef canSendMessage(CAN_HandleTypeDef* hcan, const CanMessage* toSend)
{
	hcan->pTxMsg->StdId = toSend->id;
	hcan->pTxMsg->RTR = CAN_RTR_DATA;
	hcan->pTxMsg->IDE = CAN_ID_STD;
	hcan->pTxMsg->DLC = toSend->dlc;
	for (int i = 0; i < toSend->dlc; ++i)
		hcan->pTxMsg->Data[i] = toSend->data[i];

	disableRX0Int(hcan);
	HAL_StatusTypeDef resTransmit = HAL_CAN_Transmit_IT(hcan);
	enableRX0Int(hcan);

	return resTransmit;
}

int canCheckInbox(CAN_HandleTypeDef* hcan)
{
	volatile CanMessageQueue* q = (hcan == can1Handle) ? &queue1 : &queue2;
	return (CAN_MESSAGE_QUEUE_SIZE + q->tail - q->head) % CAN_MESSAGE_QUEUE_SIZE;
}

int canGetNew(CAN_HandleTypeDef* hcan, CanMessage* toStore)
{
	if (canCheckInbox(hcan) <= 0)
		return -1;

	volatile CanMessageQueue* q = (hcan == can1Handle) ? &queue1 : &queue2;

	disableRX0Int(hcan);
	*toStore = q->messages[q->head];
	enableRX0Int(hcan);

	++(q->head);
	q->head %= CAN_MESSAGE_QUEUE_SIZE;

	return 0;
}
