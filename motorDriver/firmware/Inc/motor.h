#ifndef MOTOR_H_INCLUDED
#define MOTOR_H_INCLUDED

#include "globals.h"

#define USE_POLOLU_DRIVER

void motorInit();
int motorSetPower(int permille);
bool getFault();

#endif //MOTOR_H_INCLUDED
