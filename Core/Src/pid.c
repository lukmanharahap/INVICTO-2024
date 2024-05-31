#include "pid.h"

/* Variable for PID*/
double integral = 0.0, derivative = 0.0, prev_error = 0.0;
double prevMeasurement = 0.0;
double Kp = 0.0, Ki = 0.0, Kd = 0.0;
double tau = 0.1;
double T = 0.01;
/* END */

double PID_for_tuning(double setpoint, double measurement, double KP, double KI, double KD)
{
    double error = setpoint - measurement;

    // Proportional
    double proportional = KP * error;

    // Integral
    integral = integral + 0.5 * KI * T * (error + prev_error);

    // Derivative
	derivative = -(2.0 * KD * (measurement - prevMeasurement) + (2.0 * tau - T) * derivative) / (2.0 * tau + T);

    double output = proportional + integral + derivative;

    prev_error = error;
	prevMeasurement = measurement;

    return output;
}

double PID_for_tuningH(double setpoint, double measurement, double KP, double KI, double KD)
{
    double error = 0.0;

	if((setpoint - measurement) > 180)
	{
	    error = -(setpoint + measurement);
	}
	else if((setpoint - measurement) < -180)
	{
		error = setpoint + measurement;
	}
	else
	{
	    error = setpoint - measurement;
	}

    // Proportional
    double proportional = KP * error;

    // Integral
    integral = integral + 0.5 * KI * T * (error + prev_error);

    // Derivative
	derivative = -(2.0 * KD * (measurement - prevMeasurement) + (2.0 * tau - T) * derivative) / (2.0 * tau + T);

    double output = proportional + integral + derivative;

    prev_error = error;
	prevMeasurement = measurement;

    return output;
}

double PID_controller(double setpoint, double actual_position, uint8_t pidMode)
{
	switch(pidMode)
	{
	case 1:
		if(setpoint < 1000)
		{
			Kp = 1.5, Ki = 0.0, Kd = 0.0;
			tau = 0.1;
		}
		else
		{
			Kp = 0.2, Ki = 0.0, Kd = 0.0;
			tau = 0.1;
		}

//		Kp = 1000/fabs(setpoint - actual_position), Ki = 0.0, Kd = 0.0;
//		tau = 0.1;
//		if(setpoint < 100)
//		{
//			Kp = 550/fabs(setpoint - actual_position), Ki = 0.0, Kd = 0.0;
//			tau = 0.1;
//		}
//		else
//		{
//			Kp = 15, Ki = 0.0, Kd = 0.0;
//			tau = 0.1;
//		}
		break;
	case 2:
		if(setpoint > 0 && setpoint < 120)
		{
			Kp = 30, Ki = 0.0, Kd = 0.0;
			tau = 0.1;
		}
		else if(setpoint >= 120 && setpoint < 500)
		{
			Kp = 22, Ki = 0.0, Kd = 0.0;
			tau = 0.1;
		}
		else if(setpoint >= 500 && setpoint < 1000)
		{
			Kp = 15, Ki = 0.0, Kd = 0.0;
			tau = 0.1;
		}
		else if(setpoint >= 1000 && setpoint < 2500)
		{
			Kp = 1.5, Ki = 0.0, Kd = 0.0;
			tau = 0.1;
		}
		else
		{
			Kp = 0.6, Ki = 0.0, Kd = 0.0;
			tau = 0.1;
		}
		break;
	case 3:
		if(setpoint >= 0 && setpoint < 100)
		{
			Kp = 40.0, Ki = 0.0, Kd = 0.0;
			tau = 0.1;
		}
		else if(setpoint >= 100 && setpoint < 250)
		{
			Kp = 15.0, Ki = 0.0, Kd = 0.0;
			tau = 0.1;
		}
		else if(setpoint >= 250 && setpoint < 750)
		{
			Kp = 5.0, Ki = 0.0, Kd = 0.0;
			tau = 0.1;
		}
		else if(setpoint >= 750 && setpoint < 1250)
		{
			Kp = 3.0, Ki = 0.0, Kd = 0.0;
			tau = 0.1;
		}
		else
		{
			Kp = 1.2, Ki = 0.0, Kd = 0.0;
			tau = 0.1;
		}
		break;
	case 4:
		if(setpoint < 100)
		{
			Kp = 500/fabs(setpoint - actual_position), Ki = 0.0, Kd = 0.0;
			tau = 0.1;
		}
		else if(setpoint < 500)
		{
			Kp = 1200/fabs(setpoint - actual_position), Ki = 0.0, Kd = 0.0;
			tau = 0.1;
		}
		else
		{
			Kp = 2400/fabs(setpoint - actual_position), Ki = 0.0, Kd = 0.0;
			tau = 0.1;
		}
		break;
	case 5:
		Kp = 2.0, Ki = 0.1, Kd = 0.0;
		tau = 0.1;
		break;
	case 6:
		Kp = 0.4, Ki = 0.0, Kd = 0.0;
		tau = 0.1;
		break;
	}
    double error = setpoint - actual_position;

    // Proportional
    double proportional = Kp * error;

    // Integral
    integral = integral + 0.5 * Ki * T * (error + prev_error);

    // Derivative
	derivative = -(2.0 * Kd * (actual_position - prevMeasurement) + (2.0 * tau - T) * derivative) / (2.0 * tau + T);

    double output = proportional + integral + derivative;

    prev_error = error;
	prevMeasurement = actual_position;

    return output;
}

double PID_controllerH(double setpoint, double actual_position, uint8_t pidMode)
{
	double error, P;
	switch(pidMode)
	{
	case 1:
		if(setpoint >= 30 && setpoint < 45){Kp = 0.43;}
		else if(setpoint >= 45 && setpoint < 60){Kp = 0.33;}
		else if(setpoint >= 60 && setpoint < 75){Kp = 0.28;}
		else if(setpoint >= 75 && setpoint < 90){Kp = 0.24;}
		else if(setpoint >= 90 && setpoint < 105){Kp = 0.2;}
		else if(setpoint >= 105 && setpoint < 120){Kp = 0.18;}
		else if(setpoint >= 120 && setpoint < 135){Kp = 0.14;}
		else if(setpoint >= 135 && setpoint < 150){Kp = 0.12;}
		else if(setpoint >= 150 && setpoint < 180){Kp = 0.1;}
		else{Kp = 0.56;}
		break;
	case 2:
		if(setpoint >= 30 && setpoint < 45){Kp = 0.83;}
		else if(setpoint >= 45 && setpoint < 60){Kp = 0.75;}
		else if(setpoint >= 60 && setpoint < 75){Kp = 0.68;}
		else if(setpoint >= 75 && setpoint < 90){Kp = 0.6;}
		else if(setpoint >= 90 && setpoint < 105){Kp = 0.52;}
		else if(setpoint >= 105 && setpoint < 120){Kp = 0.45;}
		else if(setpoint >= 120 && setpoint < 135){Kp = 0.4;}
		else if(setpoint >= 135 && setpoint < 150){Kp = 0.34;}
		else if(setpoint >= 150 && setpoint < 180){Kp = 0.3;}
		else{Kp = 1.0;}
		break;
	case 3:
		if(setpoint >= 30 && setpoint < 45){Kp = 0.43;}
		else if(setpoint >= 45 && setpoint < 60){Kp = 0.35;}
		else if(setpoint >= 60 && setpoint < 75){Kp = 0.3;}
		else if(setpoint >= 75 && setpoint < 90){Kp = 0.25;}
		else if(setpoint >= 90 && setpoint < 105){Kp = 0.19;}
		else if(setpoint >= 105 && setpoint < 120){Kp = 0.15;}
		else if(setpoint >= 120 && setpoint < 135){Kp = 0.11;}
		else if(setpoint >= 135 && setpoint < 150){Kp = 0.1;}
		else if(setpoint >= 150 && setpoint < 180){Kp = 0.08;}
		else{Kp = 0.6;}
		break;
	case 4:
		if(setpoint >= 30 && setpoint < 45){Kp = 0.43;}
		else if(setpoint >= 45 && setpoint < 60){Kp = 0.35;}
		else if(setpoint >= 60 && setpoint < 75){Kp = 0.3;}
		else if(setpoint >= 75 && setpoint < 90){Kp = 0.25;}
		else if(setpoint >= 90 && setpoint < 105){Kp = 0.19;}
		else if(setpoint >= 105 && setpoint < 120){Kp = 0.15;}
		else if(setpoint >= 120 && setpoint < 135){Kp = 0.11;}
		else if(setpoint >= 135 && setpoint < 150){Kp = 0.1;}
		else if(setpoint >= 150 && setpoint < 180){Kp = 0.08;}
		else{Kp = 0.6;}
		break;
	}

	if((setpoint - actual_position) > 180)
	{
	    error = -(setpoint + actual_position);
	}
	else if((setpoint - actual_position) < -180)
	{
		error = setpoint + actual_position;
	}
	else
	{
	    error = setpoint - actual_position;
	}

    // Proportional
	P = Kp * error;

    return P;
}

void PIDController_Init(PIDController *pid)
{
	/* Clear controller variables */
	pid->integrator = 0.0f;
	pid->prevError = 0.0f;

	pid->differentiator = 0.0f;
	pid->prevMeasurement = 0.0f;

	pid->T = 0.01f;
	pid->out = 0.0f;
}

double PIDController_Update(PIDController *pid, double setpoint, double measurement)
{
	/*
	 * Error signal
	 */
	double error = setpoint - measurement;

	/*
	 * Proportional
	 */
	double proportional = pid->Kp * error;

	/*
	 * Integral
	 */
	pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (error + pid->prevError);

	/* Anti-wind-up via integrator clamping */
	if (pid->integrator > pid->limMaxInt)
	{

		pid->integrator = pid->limMaxInt;
	}
	else if (pid->integrator < pid->limMinInt)
	{

		pid->integrator = pid->limMinInt;
	}

	/*
	 * Derivative (band-limited differentiator)
	 */

	pid->differentiator = -(2.0f * pid->Kd * (measurement - pid->prevMeasurement) /* Note: derivative on measurement, therefore minus sign in front of equation! */
							+ (2.0f * pid->tau - pid->T) * pid->differentiator) /
						  (2.0f * pid->tau + pid->T);

	/*
	 * Compute output and apply limits
	 */
	pid->out = proportional + pid->integrator + pid->differentiator;

	if (pid->out > pid->limMax)
	{

		pid->out = pid->limMax;
	}
	else if (pid->out < pid->limMin)
	{

		pid->out = pid->limMin;
	}

	/* Store error and measurement for later use */
	pid->prevError = error;
	pid->prevMeasurement = measurement;

	/* Return controller output */
	return pid->out;
}
