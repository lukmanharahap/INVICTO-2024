/**
 * @file	odometry.h
 * @author	INVICTO TEAM (Arduino 2021)
 * @brief	Header file of odometry.c.
 */

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <stm32f4xx_hal.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <Library/Inc/lcd_i2c.h>
#include <Library/Inc/invicto_motor.h>
#include <Library/Inc/pid.h>

#define global		1
#define local		2
#define in_global	3
#define in_local	4

// Define state vector dimension
#define STATE_DIM 3

// Define measurement vector dimension
#define MEASUREMENT_DIM 3

/**
 * @brief external_global structure definition
 */
typedef struct
{
    double x;
    double y;
    double h;
} external_global;

/**
 * @brief external_local structure definition
 */
typedef struct
{
    double x;
	double y;
	double h;
} external_local;

/**
 * @brief internal_global structure definition
 */
typedef struct
{
    double x;
    double y;
	double h;
} internal_global;

/**
 * @brief internal_local structure definition
 */
typedef struct
{
    double x;
    double y;
	double h;
} internal_local;

/**
 * @brief StateVector structure definition
 */
typedef struct {
    double x;
    double y;
    double theta;
} StateVector;

/**
 * @brief MeasurementVector structure definition
 */
typedef struct {
    double x;
    double y;
    double theta;
} MeasurementVector;

/**
 * @brief EKF structure definition
 */
typedef struct {
    double x;
    double y;
    double h;
} EKF;

/**
 * @brief function definition
 */
external_global odometry_eg();
external_local odometry_el();
internal_global odometry_ig();
internal_local odometry_il();
StateVector stateTransition(StateVector X);
MeasurementVector measurementFunction(StateVector X);
EKF extendedKalmanFilter();
EKF odometry_fusion();
void displayCounter();
void display_EG();
void display_EL();

#endif
