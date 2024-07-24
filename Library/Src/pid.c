/**
 * @file	pid.c
 * @author	INVICTO TEAM (Arduino 2021)
 * @brief	PID library.
 * 			This file provide PID Control algorithms.
 */

#include <Library/Inc/pid.h>

/* Variable for PID*/
double integral = 0.0, derivative = 0.0, prev_error = 0.0;
/* END */

/**
 * @brief	Computes the PID control signal.
 *
 * This function implements a standard PID controller. It computes the error
 * between the setpoint and the actual position, then calculates the integral
 * and derivative terms. The control signal is the sum of the proportional,
 * integral, and derivative terms.
 *
 * @param	setpoint The desired target value.
 * @param	actual_position The current value being controlled.
 * @param	Kp Proportional gain.
 * @param	Ki Integral gain.
 * @param	Kd Derivative gain.
 *
 * @return	The computed control signal.
 */
double PID_controller(double setpoint, double actual_position, double Kp, double Ki, double Kd)
{
	double dt = 0.01;

    // Proportional
    double error = setpoint - actual_position;

    // Integral
    integral += error * dt;

    // Derivative
	derivative = (error - prev_error) / dt;

    double output = Kp * error + Ki * integral + Kd * derivative;

    prev_error = error;

    return output;
}

/**
 * @brief	Computes the proportional control signal for heading with angle wrapping.
 *
 * This function implements a proportional controller specifically for angles.
 * It computes the error between the setpoint and the actual position, taking
 * into account the circular nature of angles by wrapping the error within the
 * range of -180 to 180 degrees.
 *
 * @param	setpoint The desired target angle.
 * @param	actual_position The current angle being controlled.
 * @param	Kp Proportional gain.
 *
 * @return	The computed control signal.
 */
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

    // Proportional
	double P = Kp * error;

    return P;
}
