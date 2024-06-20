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
bool atTargetPosition(robotPosition setpoint, robotPosition position, double xyTolerance, double Htolerance);
double limitAcceleration(double currentVelocity, double targetVelocity, double maxAccel);
void servo_write(int angle);
void PID_Kalman(robotPosition setpoint, uint8_t pidMode);
void PID_KFtocoordinate(robotPosition setpoint, uint8_t pidMode, double smoothingFactor);
void PID_moveToCoordinate(EKF *setpoint, uint8_t pidMode, double tolerance, uint16_t amount);
void PID_gotoX(double setpoint_x, uint8_t pidMode);
void PID_gotoY(double setpoint_y, uint8_t pidMode);
void PID_setDegree(double setpoint_h);
void focusToTheBall();
void initializeSilos();
Silo detectAndStoreSilo();
void placeBallInSilo(robotPosition setpoint, uint8_t pidMode);
void findAndTakeBall(robotPosition setpoint, uint8_t pidMode);
void displaySilo();

#endif
