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

void smoothVelocity(double* Vx, double* Vy, double* W, double smoothingFactor);
void PID_Tuning(EKF setpoint, double KP, double KI, double KD);
void PID_movetoCoordinate(robotPosition setpoint, uint8_t type, uint8_t pidMode, double smoothingFactor);
void PID_Kalman(EKF setpoint, uint8_t pidMode);
void PID_KFtocoordinate(EKF setpoint, uint8_t pidMode);
void PID_steptoCoordinate(EKF *setpoint, uint8_t pidMode, double tolerance, uint16_t amount);
void PID_gotoX(double setpoint_x, uint8_t pidMode);
void PID_gotoY(double setpoint_y, uint8_t pidMode);
void PID_setDegree(double setpoint_h);
void PID_coba(EKF setpoint, uint8_t pidMode, double lookaheadDistance);
void findtheBall();
void findSilo();

#endif
