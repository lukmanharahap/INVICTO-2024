#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <stm32f4xx_hal.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
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
    double x;
    double y;
    double h;
} external_global;

typedef struct
{
    double x;
	double y;
	double h;
} external_local;

typedef struct
{
    double x;
    double y;
	double h;
} internal_global;

typedef struct
{
    double x;
    double y;
	double h;
} internal_local;

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

external_global odometry_eg();
external_local odometry_el();
internal_global odometry_ig();
internal_local odometry_il();
void displayKalman(EKF position);
StateVector stateTransition(StateVector X);
MeasurementVector measurementFunction(StateVector X);
EKF extendedKalmanFilter();
bool detectSlippage(external_global position, double threshold);
EKF odometry_fusion();
void cek();
void cek2(external_global position_eg, external_local position_el);
void displayCounter();
void display_EG();
void display_EL();

#endif
