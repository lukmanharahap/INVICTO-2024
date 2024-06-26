#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <stm32f4xx_hal.h>
#include <stdio.h>
#include <math.h>
#include "invicto_motor.h"
#include "odometry.h"

typedef struct
{
	int distance;
	int angle;
	bool detected;
	int ballInSilo;
	double x;
	double y;
} Silo;

int map(int st1, int fn1, int st2, int fn2, int value);
double lowPassFilter(double newValue, double oldValue, double alpha);
double rampTrapezoidal(double current, double target, double maxVel, double maxAccel, double maxDecel, double dt);
double limitRateOfChange(double current, double target, double maxChange);
void smoothVelocity(double* Vx, double* Vy, double* W, double smoothingFactor);
double limitAcceleration(double currentVelocity, double targetVelocity, double maxAccel);
bool atTargetExternal(robotPosition setpoint, robotPosition position, double xyTolerance, double Htolerance);
bool atTargetPosition(EKF setpoint, EKF position, double xyTolerance, double Htolerance);
void lookForTheBall(double targetAngle1, double targetAngle2, double currentAngle);
void servo_write(int angle);
void PID_External(robotPosition setpoint, double Kp, double Ki, double Kd, double KpH, double smoothingFactor, double maxVelocity);
void PID_Internal(robotPosition setpoint, double Kp, double Ki, double Kd, double KpH, double smoothingFactor, double maxVelocity);
void PID_Kalman(robotPosition setpoint, double Kp, double Ki, double Kd, double KpH, double smoothingFactor, double maxVelocity);
void PID_KFtocoordinate(EKF setpoint, double Kp, double Ki, double Kd, double KpH, double smoothingFactor, double maxVelocity);
void PID_moveToCoordinate(EKF *setpoint, double Kp, double Ki, double Kd, double KpH, double tolerance, double maxVelocity);
void PID_gotoX(double setpoint_x, double Kp, double Ki, double Kd);
void PID_gotoY(double setpoint_y, double Kp, double Ki, double Kd);
void PID_setDegree(double setpoint_h, double KpH);
void focusToTheBall();
void initializeSilos();
Silo detectAndStoreSilo();
void placeBallInSilo(robotPosition setpoint, double Kp, double Ki, double Kd, double KpH);
void findAndTakeBall();
void displaySilo();

#endif
