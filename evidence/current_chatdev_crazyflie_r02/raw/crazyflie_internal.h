/* DOCSTRING: Internal host-model symbols shared between generated Crazyflie source files. */
#ifndef CRAZYFLIE_INTERNAL_H
#define CRAZYFLIE_INTERNAL_H

#include "6_generated_code.h"

extern volatile uint32_t g_systemTickMs;
extern State g_estimatedState;
extern SensorData g_currentSensors;
extern Setpoint g_activeSetpoint;

void commanderGetSetpoint(Setpoint *out);
void sensorsAcquireInternal(SensorData *out);
void stateEstimatorStepInternal(uint32_t stabilizerStep, State *state, SensorData *sensors);

#endif
