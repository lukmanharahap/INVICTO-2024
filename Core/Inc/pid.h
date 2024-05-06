#ifndef PID_H
#define PID_H

#include <stm32f4xx_hal.h>
#include <math.h>

typedef struct
{
	/* Controller gains */
	double Kp;
	double Ki;
	double Kd;

	/* Derivative low-pass filter time constant */
	double tau;

	/* Output limits */
	double limMin;
	double limMax;

	/* Integrator limits */
	double limMinInt;
	double limMaxInt;

	/* Sample time (in seconds) */
	double T;

	/* Controller "memory" */
	double integrator;
	double prevError; /* Required for integrator */
	double differentiator;
	double prevMeasurement; /* Required for differentiator */

	/* Controller output */
	double out;
} PIDController;

double PID_for_tuning(double setpoint, double measurement, double KP, double KI, double KD);
double PID_for_tuningH(double setpoint, double measurement, double KP, double KI, double KD);
double PID_controller(double setpoint, double actual_position, uint8_t pidMode);
double PID_controllerH(double setpoint, double actual_position);
void PIDController_Init(PIDController *pid);
double PIDController_Update(PIDController *pid, double setpoint, double measurement);

#endif
