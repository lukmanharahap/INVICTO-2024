#include "pid.h"

/* Variable for PID*/
double integral = 0.0, derivative = 0.0, prev_error = 0.0;
/* END */

double PID_controller(double setpoint, double actual_position, double Kp, double Ki, double Kd)
{
	double dt = 0.01;

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

double PID_controllerH(double setpoint, double actual_position, double Kp)
{
	double error = setpoint - actual_position;

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
