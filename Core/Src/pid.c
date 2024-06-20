#include "pid.h"

/* Variable for PID*/
double integral = 0.0, derivative = 0.0, prev_error = 0.0;
/* END */

double PID_controller(double setpoint, double actual_position, uint8_t pidMode)
{
	double Kp = 1.0, Ki = 0.0, Kd = 0.0;
	double dt = 0.01;

	switch(pidMode)
	{
	case 1:
		Kp = 0.8, Ki = 0.5, Kd = 0.0;
		break;
	case 2:
		Kp = 1.2, Ki = 1.2, Kd = 0.0;
		break;
	case 3:
		Kp = 1.5, Ki = 1.5, Kd = 0.0;
		break;
	case 4:
		Kp = 1.2, Ki = 0.0, Kd = 0.0;
		break;
	}
    // Proportional
    double error = setpoint - actual_position;

//    if(fabs(error) < 200) error = 0;
    // Integral
    integral += error * dt;

    // Derivative
	derivative = (error - prev_error) / dt;

    double output = Kp * error + Ki * integral + Kd * derivative;

    prev_error = error;

    return output;
}

double PID_controllerH(double setpoint, double actual_position, uint8_t pidMode)
{
	double Kp = 1.0;
	double error = setpoint - actual_position;

	switch(pidMode)
	{
	case 1:
		Kp = 1.4;
		break;
	case 2:
		Kp = 1.2;
		break;
	case 3:
		Kp = 1.5;
		break;
	default:
		Kp = 0.5;
		break;
	}

	if(error > 180)
	{
		error -= 360;
	}
	else if(error < -180)
	{
		error += 360;
	}

//	if(fabs(error) < 1) error = 0;

    // Proportional
	double P = Kp * error;

    return P;
}
