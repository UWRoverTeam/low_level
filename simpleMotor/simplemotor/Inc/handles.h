
#ifndef HANDLES_H_INCLUDED
#define HANDLES_H_INCLUDED

#include "main.h"

extern volatile int32_t encoder_value, index_pulse_position;
extern uint32_t ticks;
extern UART_HandleTypeDef huart2;
extern uint8_t tx_busy;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;

extern CAN_HandleTypeDef hcan1;
extern CAN_TxHeaderTypeDef TxHeader; 
extern CAN_RxHeaderTypeDef RxHeader;
extern uint32_t TxMailbox;
extern uint8_t TxData[8];
extern uint8_t RxData[8];
#endif
