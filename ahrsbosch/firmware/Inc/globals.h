#ifndef GLOBALS_H_INCLUDED
#define GLOBALS_H_INCLUDED

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "main.h"
#include "stm32f1xx_hal.h"

#include "handles.h"

extern CAN_HandleTypeDef *can1Handle;
extern CAN_HandleTypeDef *can2Handle;

#define CLAMP_VALUE(x, min, max) (x > max ? max : (x < min ? min : x))

void led(uint8_t n, uint8_t state);
void blink(uint8_t l, int n);

#endif //GLOBALS_H_INCLUDED
