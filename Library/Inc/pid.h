/**
 * @file	pid.h
 * @author	INVICTO TEAM (Arduino 2021)
 * @brief	Header file of pid.c.
 */

#ifndef PID_H
#define PID_H

#include <stm32f4xx_hal.h>
#include <math.h>

double PID_controller(double setpoint, double actual_position, double Kp, double Ki, double Kd);
double PID_controllerH(double setpoint, double actual_position, double Kp);

#endif
