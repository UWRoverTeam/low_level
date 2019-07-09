#include "pid.h"

bool pidCompute(PidData* data)
{
	if (!data->on)
		return false;
	uint32_t now = HAL_GetTick();
	if (now < data->lastCompute + data->period)
		return false;
	data->lastCompute = now;

	float error = data->setpoint - data->input;

	data->integral += (data->Ki * error);
	data->integral = CLAMP_VALUE(data->integral, data->outputMin, data->outputMax);

	data->output = (data->Kp * error) + data->integral - (data->Kd * (data->input - data->lastInput));
	data->output = CLAMP_VALUE(data->output, data->outputMin, data->outputMax);

	data->lastInput = data->input;
	return true;
}

void pidStart(PidData* data)
{
	data->integral = data->output;
	data->integral = CLAMP_VALUE(data->integral, data->outputMin, data->outputMax);
	data->lastInput = data->input;
	data->on = true;
}

void pidStop(PidData* data)
{
	data->on = false;
}
