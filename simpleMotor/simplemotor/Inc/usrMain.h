#ifndef USR_MAIN_H_INCLUDED
#define USR_MAIN_H_INCLUDED

#include "stm32l4xx_hal.h"
#include "pid.h"

PIDController *pid;
void encoder_tick();
void control_tick();
void uart_byte_received(uint8_t byte);
void uart_analyse_buffer();
void check_tx_buffer();
void tx_cplt();
void uart_transmit(uint8_t *data, uint16_t len);

HAL_StatusTypeDef can_transmit(uint8_t header, uint8_t d1, uint8_t d2, uint8_t d3, 
				  uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7);
int usrMain();
float pid_source();

void pwm_setvalue(uint16_t value);
void set_power(float value);
void parse_can_message();

#endif /* __MAIN_H */
