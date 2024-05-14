#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <stm32f4xx_hal.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
//#include "robot_control.h"
#include "lcd_i2c.h"
#include "invicto_motor.h"
#include "pid.h"

#define global		1
#define local		2
#define in_global	3
#define in_local	4

// Define state vector dimension
#define STATE_DIM 3

// Define measurement vector dimension
#define MEASUREMENT_DIM 3

typedef struct
{
    double x_global;
    double y_global;

    double x_local;
	double y_local;

    double x_in_global;
    double y_in_global;

    double x_in_local;
    double y_in_local;

	double h_en;
    double h;
} robotPosition;

// State vector [x, y, theta]
typedef struct {
    double x;
    double y;
    double theta;
} StateVector;

// Measurement vector [x, y, theta]
typedef struct {
    double x;
    double y;
    double theta;
} MeasurementVector;

typedef struct {
    double x;
    double y;
    double h;
} EKF;

robotPosition odometry();
void displayKalman(EKF position);
StateVector stateTransition(StateVector X);
MeasurementVector measurementFunction(StateVector X);
EKF extendedKalmanFilter();
void cek(EKF position);
void cek2(EKF setpoint, EKF position);
void displayCounter();
void displayPosition(robotPosition position, uint8_t type);

#endif
