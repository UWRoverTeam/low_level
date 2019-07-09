#ifndef PID_H_INCLUDED
#define PID_H_INCLUDED

#include "globals.h"

typedef struct {
	float Kp, Ki, Kd;
	float input, setpoint, output;
	float outputMax, outputMin;
	float integral;
	uint32_t lastCompute, period;
	float lastInput;
	bool on;
} PidData;

bool pidCompute(PidData* data);
void pidStart(PidData* data);
void pidStop(PidData* data);

#endif //PID_H_INCLUDED
