#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <stm32f4xx_hal.h>
#include <stdio.h>
#include <math.h>
#include "invicto_motor.h"
#include "odometry.h"

#define gotoBall 0
#define findBall 1
#define stopRobot 2

int map(int st1, int fn1, int st2, int fn2, int value);
double lowPassFilter(double newValue, double oldValue, double alpha);
double rampTrapezoidal(double current, double target, double maxVel, double maxAccel, double maxDecel, double dt);
double limitRateOfChange(double current, double target, double maxChange);
void smoothVelocity(double* Vx, double* Vy, double* W, double smoothingFactor);
void servo_write(int angle);
void nyoba_gerak(EKF setpoint, uint8_t pidMode);
void PID_Tuning(EKF setpoint, double KP, double KI, double KD);
void PID_movetoCoordinate(robotPosition setpoint, uint8_t type, uint8_t pidMode, double smoothingFactor);
void PID_Kalman(EKF setpoint, uint8_t pidMode);
void PID_nyoba(EKF setpoint, uint8_t pidMode);
void PID_KFtocoordinate(EKF setpoint, uint8_t pidMode);
void PID_steptoCoordinate(EKF *setpoint, uint8_t pidMode, double tolerance, uint16_t amount);
void PID_gotoX(double setpoint_x, uint8_t pidMode);
void PID_gotoY(double setpoint_y, uint8_t pidMode);
void PID_setDegree(double setpoint_h);
void PID_coba(EKF setpoint, uint8_t pidMode, double lookaheadDistance);
void focusToTheBall();
void findtheBall();
void findSilo();

#endif
