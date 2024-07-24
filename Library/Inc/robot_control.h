/**
 * @file	robot_control.h
 * @author	INVICTO TEAM (Arduino 2021)
 * @brief	Header file of robot_control.c.
 */

#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include <stm32f4xx_hal.h>
#include <stdio.h>
#include <math.h>
#include <Library/Inc/invicto_motor.h>
#include <Library/Inc/odometry.h>

/**
 * @brief Silo structure definition
 */
typedef struct
{
	int distance;
	int angle;
	bool detected;
	int ballInSilo;
	double x;
	double y;
} Silo;

/**
 * @brief PID parameter structure definition
 */
typedef struct
{
	double KP;
	double KI;
	double KD;
	double KpH;
	double smoothingFactor;
	int maxVelocity;
	double xyTolerance;
	double hTolerance;
} PID_parameter;

/**
 * @brief function definition
 */
int map(int st1, int fn1, int st2, int fn2, int value);
double rampTrapezoidal(double current, double target, double maxVel, double maxAccel, double maxDecel, double dt);
double limitRateOfChange(double current, double target, double maxChange);
void smoothVelocity(double* Vx, double* Vy, double* W, double smoothingFactor);
double limitAcceleration(double currentVelocity, double targetVelocity, double maxAccel);
bool atTargetEG(external_global setpoint, external_global position, double xyTolerance, double hTolerance);
bool atTargetEL(external_local setpoint, external_local position, double xyTolerance, double hTolerance);
void lookForTheBall(double targetAngle1, double targetAngle2, double currentAngle);
void servo_write(int angle);
void PID_EG(external_global setpoint, double Kp, double Ki, double Kd, double KpH, double smoothingFactor, double maxVelocity);
void PID_EL(external_local setpoint, double Kp, double Ki, double Kd, double KpH, double smoothingFactor, double maxVelocity);
void PID_IG(internal_global setpoint, double Kp, double Ki, double Kd, double KpH, double smoothingFactor, double maxVelocity);
void PID_IL(internal_local setpoint, double Kp, double Ki, double Kd, double KpH, double smoothingFactor, double maxVelocity);
void PID_KFtocoordinate(EKF setpoint, double Kp, double Ki, double Kd, double KpH, double smoothingFactor, double maxVelocity);
void PID_moveToCoordinate(external_global *setpoint, PID_parameter *parameters, uint16_t numPoints);
void PID_setDegree(double setpoint_h, double KpH);
void focusToTheBall();
void initializeSilos();
Silo detectAndStoreSilo();
void placeBallInSilo(external_global setpoint, double Kp, double Ki, double Kd, double KpH, double smoothingFactor, double maxVelocity);
void findAndTakeBall(external_global *findBall);
void throwTheBall(external_global whereTo, double Kp, double Ki, double Kd, double KpH);
void displayBall();
void displaySilo();

#endif
