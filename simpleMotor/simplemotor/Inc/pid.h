#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct pid_controller {

	float p;
	float i;
	float d;
	float target;
	float output;
	uint8_t enabled;
	float currentFeedback;
	float lastFeedback;
	float error;
	float lastError;
	int32_t currentTime;
	int32_t lastTime;
	float integralCumulation;
	float maxCumulation;
	float cycleDerivative;
	float maxOutputSweep;
	float lastOutput;
	float deltaTime;
	float deadZone;

	uint8_t inputBounded;
	float inputLowerBound;
	float inputUpperBound;
	uint8_t outputBounded;
	float outputLowerBound;
	float outputUpperBound;
	uint8_t feedbackWrapped;
	float feedbackWrapLowerBound;
	float feedbackWrapUpperBound;

	//uint8_t timeFunctionRegistered;
	float (*pidSource)();
	void (*pidOutput)(float output);
	//uint32_t (*getSystemTime)();

} PIDController;

PIDController *createPIDController(float p, float i, float d, float (*pidSource)(void), void (*pidOutput)(float output));

void tick(PIDController *controller);
void setEnabled(PIDController *controller, uint8_t e);
float getProportionalComponent(PIDController *controller);
float getIntegralComponent(PIDController *controller);
float getDerivativeComponent(PIDController *controller);
void setMaxIntegralCumulation(PIDController *controller, float max);

void setInputBounds(PIDController *controller, float lower, float upper);
void setOutputBounds(PIDController *controller, float lower, float upper);
void setFeedbackWrapBounds(PIDController *controller, float lower, float upper);

void setPIDSource(PIDController *controller, float (*pidSource)());
void setPIDOutput(PIDController *controller, void (*pidOutput)(float output));
void registerTimeFunction(PIDController *controller, uint32_t (*getSystemTime)(void));

#endif // PID_H
