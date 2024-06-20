#ifndef PID_H
#define PID_H

#include <stm32f4xx_hal.h>
#include <math.h>

double PID_controller(double setpoint, double actual_position, uint8_t pidMode);
double PID_controllerH(double setpoint, double actual_position, uint8_t pidMode);

#endif
