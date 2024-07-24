/**
 * @file	odometry.c
 * @author	INVICTO TEAM (Arduino 2021)
 * @brief	Odometry library.
 * 			This file provide odometry using external or internal encoders with MPU6050 integration.
 */

#include <Library/Inc/odometry.h>

/* Variable for Encoder external */
#define PPR 2000
#define diameter 58 // Diameter of encoder wheel in mm
#define e1_e2 380 // distance between encoder 1 & 2 in mm
#define e12_e3 230 // distance between the midpoint of encoder 1 & 2 and encoder 3 in mm
const double mm_per_tick = M_PI * diameter / PPR;
double xGlobal = 0.0, yGlobal = 0.0;
double xLocal = 0.0, yLocal = 0.0;
double heading = 0.0;
static int oldEnc1 = 0, oldEnc2 = 0, oldEnc3 = 0;
extern volatile int counter1, counter2, counter3;
extern double sensorData[3];
char buffer[10];
/* END */

/* Variable for Encoder internal */
#define PPR_IN 7
#define diameter_IN 15.2 // Diameter of encoder wheel in mm
#define m1_m3 650 // distance between motor 1 and 3 / 2 and 4 in mm
#define half_robot 250 // half of robot in mm
const double mm_per_tick_IN = M_PI * diameter_IN / PPR_IN;
static int oldEncIN1 = 0, oldEncIN2 = 0, oldEncIN3 = 0, oldEncIN4 = 0;
extern volatile int counterIN1, counterIN2, counterIN3, counterIN4;
double xGlobalIN = 0.0, yGlobalIN = 0.0;
double xLocalIN = 0.0, yLocalIN = 0.0;
double headingIN = 0.0;
/* END */

extern int camera[18];
extern int sensorMEGA[4];

/**
 * @brief	Calculates the global position of the robot using external encoders.
 *
 * This function calculates the robot's global position based on the readings
 * from the external encoders and updates the global coordinates (xGlobal, yGlobal) and heading.
 * It converts the encoder ticks to distance and angle changes,
 * then updates the global position using trigonometric transformations.
 *
 * @param	None.
 * @return	external_global structure containing the current global position (x, y) and heading (h) of the robot.
 */
external_global odometry_eg()
{
	external_global currentPosition;
    double yaw = sensorData[0] * M_PI / 180.0;

    /* ENCODER EXTERNAL */
    int dn1 = counter1 - oldEnc1;
    int dn2 = counter2 - oldEnc2;
    int dn3 = counter3 - oldEnc3;

    oldEnc1 = counter1;
    oldEnc2 = counter2;
    oldEnc3 = counter3;

    double dtheta = mm_per_tick * (dn2 - dn1) / e1_e2;
    double dx = mm_per_tick * (dn3 + (dn2 - dn1) * e12_e3 / e1_e2);
    double dy = mm_per_tick * (dn1 + dn2) / 2.0;
    /* END */

//	double theta = heading + (dtheta / 2.0);
	xGlobal += dx * cos(yaw) + dy * sin(yaw);
	yGlobal += -dx * sin(yaw) + dy * cos(yaw);
	heading += dtheta;

	currentPosition.x = xGlobal;
	currentPosition.y = yGlobal;
	currentPosition.h = sensorData[0];

	return currentPosition;
}

/**
 * @brief	Calculates the local position of the robot using external encoders.
 *
 * This function calculates the robot's local position based on the readings
 * from the external encoders and updates the local coordinates (xLocal, yLocal) and heading.
 * It converts the encoder ticks to distance and angle changes, then updates the
 * local position using trigonometric transformations.
 *
 * @param	None.
 * @return	external_local structure containing the current local position (x, y) and heading (h) of the robot.
 */
external_local odometry_el()
{
	external_local currentPosition;

    /* ENCODER EXTERNAL */
    int dn1 = counter1 - oldEnc1;
    int dn2 = counter2 - oldEnc2;
    int dn3 = counter3 - oldEnc3;

    oldEnc1 = counter1;
    oldEnc2 = counter2;
    oldEnc3 = counter3;

    double dtheta = mm_per_tick * (dn2 - dn1) / e1_e2;
    double dx = mm_per_tick * (dn3 + (dn2 - dn1) * e12_e3 / e1_e2);
    double dy = mm_per_tick * (dn1 + dn2) / 2.0;

    double distance = hypot(dx, dy);
	double direction = atan2(dy, dx);
	double theta_local = direction - dtheta;
    /* END */

	xLocal += distance * cos(theta_local);
	yLocal += distance * sin(theta_local);

	currentPosition.x = xLocal;
	currentPosition.y = yLocal;
	currentPosition.h = sensorData[0];

	return currentPosition;
}

/**
 * @brief	Calculates the global position of the robot using internal encoders.
 *
 * This function calculates the robot's global position based on the readings
 * from the internal encoders and updates the global coordinates (xGlobalIN, yGlobalIN) and heading.
 * It converts the encoder ticks to distance and angle changes,
 * then updates the global position using trigonometric transformations.
 *
 * @param	None.
 * @return	internal_global structure containing the current global position (x, y) and heading (h) of the robot.
 */
internal_global odometry_ig()
{
	internal_global currentPosition;
    double yaw = sensorData[0] * M_PI / 180.0;

    /* ENCODER INTERNAL */
    int dn1_in = counterIN1 - oldEncIN1;
    int dn2_in = counterIN2 - oldEncIN2;
    int dn3_in = counterIN3 - oldEncIN3;
    int dn4_in = counterIN4 - oldEncIN4;

    oldEncIN1 = counterIN1;
    oldEncIN2 = counterIN2;
    oldEncIN3 = counterIN3;
    oldEncIN4 = counterIN4;

    double dthetaIN = mm_per_tick_IN * ((-dn1_in + dn3_in) + (dn2_in - dn4_in)) / (m1_m3 * 4);
    double dxIN = mm_per_tick_IN * ((-dn1_in + dn2_in - dn3_in + dn4_in)/4 * cos(M_PI_4) + ((-dn1_in + dn3_in) + (dn2_in - dn4_in)) / (m1_m3 * 4));
    double dyIN = mm_per_tick_IN * (dn1_in + dn2_in + dn3_in + dn4_in)/4 * sin(M_PI_4);
    /* END */

//	double thetaIN = headingIN + (dthetaIN / 2.0);
	xGlobalIN += dxIN * cos(yaw) + dyIN * sin(yaw);
	yGlobalIN += -dxIN * sin(yaw) + dyIN * cos(yaw);
	headingIN += dthetaIN;

	currentPosition.x = xGlobalIN;
	currentPosition.y = yGlobalIN;
	currentPosition.h = sensorData[0];

	return currentPosition;
}

/**
 * @brief	Calculates the local position of the robot using internal encoders.
 *
 * This function calculates the robot's local position based on the readings
 * from the internal encoders and updates the local coordinates (xLocalIN, yLocalIN) and heading.
 * It converts the encoder ticks to distance and angle changes, then updates the
 * local position using trigonometric transformations.
 *
 * @param	None.
 * @return	internal_local structure containing the current local position (x, y) and heading (h) of the robot.
 */
internal_local odometry_il()
{
	internal_local currentPosition;

    /* ENCODER INTERNAL */
    int dn1_in = counterIN1 - oldEncIN1;
    int dn2_in = counterIN2 - oldEncIN2;
    int dn3_in = counterIN3 - oldEncIN3;
    int dn4_in = counterIN4 - oldEncIN4;

    oldEncIN1 = counterIN1;
    oldEncIN2 = counterIN2;
    oldEncIN3 = counterIN3;
    oldEncIN4 = counterIN4;

    double dthetaIN = mm_per_tick_IN * ((-dn1_in + dn3_in) + (dn2_in - dn4_in)) / (m1_m3 * 4);
    double dxIN = mm_per_tick_IN * ((-dn1_in + dn2_in - dn3_in + dn4_in)/4 * cos(M_PI_4) + ((-dn1_in + dn3_in) + (dn2_in - dn4_in)) / (m1_m3 * 4));
    double dyIN = mm_per_tick_IN * (dn1_in + dn2_in + dn3_in + dn4_in)/4 * sin(M_PI_4);

    double distanceIN = hypot(dxIN, dyIN);
	double directionIN = atan2(dyIN, dxIN);
	double theta_localIN = directionIN - dthetaIN;
    /* END */

	xLocalIN += distanceIN * cos(theta_localIN);
	yLocalIN += distanceIN * sin(theta_localIN);

    currentPosition.x = xLocalIN;
    currentPosition.y = yLocalIN;
	currentPosition.h = sensorData[0];

	return currentPosition;
}

double getVelocity()
{
	static uint32_t lastTimer = 0;

    /* ENCODER INTERNAL */
    int dn1_in = counterIN1 - oldEncIN1;
    int dn2_in = counterIN2 - oldEncIN2;
    int dn3_in = counterIN3 - oldEncIN3;
    int dn4_in = counterIN4 - oldEncIN4;
    oldEncIN1 = counterIN1;
    oldEncIN2 = counterIN2;
    oldEncIN3 = counterIN3;
    oldEncIN4 = counterIN4;

    double dxIN = mm_per_tick_IN * (-dn1_in + dn2_in - dn3_in + dn4_in)/4 * cos(M_PI_4);
    double dyIN = mm_per_tick_IN * (dn1_in + dn2_in + dn3_in + dn4_in)/4 * sin(M_PI_4);

    double distanceIN = hypot(dxIN, dyIN);

    /* ENCODER EXTERNAL */
    int dn1 = counter1 - oldEnc1;
    int dn2 = counter2 - oldEnc2;
    int dn3 = counter3 - oldEnc3;
    oldEnc1 = counter1;
    oldEnc2 = counter2;
    oldEnc3 = counter3;

    double dx = mm_per_tick * (dn3 + (dn2 - dn1) * e12_e3 / e1_e2);
    double dy = mm_per_tick * (dn1 + dn2) / 2.0;

    double distance = hypot(dx, dy);

    uint32_t timer = HAL_GetTick();
    double velocity = (distance + distanceIN) / (2*(timer - lastTimer));

    return velocity;
}

// Process noise covariance matrix (Q)
const double Q[STATE_DIM][STATE_DIM] = {
    {0.1, 0.0, 0.0},
    {0.0, 0.1, 0.0},
    {0.0, 0.0, 0.1}
};

// Measurement noise covariance matrix (R)
const double R[MEASUREMENT_DIM][MEASUREMENT_DIM] = {
    {0.1, 0.0, 0.0},
    {0.0, 0.1, 0.0},
    {0.0, 0.0, 0.1}
};

// State transition function f
StateVector stateTransition(StateVector X)
{
    StateVector X_next;
    external_global position = odometry_eg();
//    X_next.x = X.x + v * cos(X.theta) * dt + 0.5 * ax * pow(dt, 2) * cos(X.theta);
//    X_next.y = X.y + v * sin(X.theta) * dt + 0.5 * ay * pow(dt, 2) * sin(X.theta);
//    X_next.theta = X.theta + heading;
    X_next.x = X.x + position.x;
    X_next.y = X.y + position.y;
    X_next.theta = X.theta + position.h;
    return X_next;
}

// Measurement function h
MeasurementVector measurementFunction(StateVector X)
{
    MeasurementVector Z;
    Z.x = X.x;
    Z.y = X.y;
    Z.theta = X.theta;
    return Z;
}

// Extended Kalman Filter
EKF extendedKalmanFilter()
{
	EKF currentPosition;
	external_global position_eg = odometry_eg();
	internal_global position_ig = odometry_ig();

	double P[STATE_DIM][STATE_DIM] = {
		{1.0, 0.0, 0.0},
		{0.0, 1.0, 0.0},
		{0.0, 0.0, 1.0}};

    // Prediction step
	StateVector X = {position_ig.x, position_ig.y, 0.0};
    StateVector X_pred = stateTransition(X);
    double P_pred[STATE_DIM][STATE_DIM] = {
        {P[0][0] + Q[0][0], P[0][1], P[0][2]},
        {P[1][0], P[1][1] + Q[1][1], P[1][2]},
        {P[2][0], P[2][1], P[2][2] + Q[2][2]}
    };

    // Update step
	MeasurementVector Z = {position_ig.x, position_ig.y, 0.0};
    MeasurementVector Z_pred = measurementFunction(X_pred);
    double Y[MEASUREMENT_DIM] = {Z.x - Z_pred.x, Z.y - Z_pred.y, Z.theta - Z_pred.theta};
    double S[MEASUREMENT_DIM][MEASUREMENT_DIM] = {
        {P_pred[0][0] + R[0][0], P_pred[0][1], P_pred[0][2]},
        {P_pred[1][0], P_pred[1][1] + R[1][1], P_pred[1][2]},
        {P_pred[2][0], P_pred[2][1], P_pred[2][2] + R[2][2]}
    };
    double K[STATE_DIM][MEASUREMENT_DIM] = {
        {P_pred[0][0] / S[0][0], P_pred[0][1] / S[1][1], P_pred[0][2] / S[2][2]},
        {P_pred[1][0] / S[0][0], P_pred[1][1] / S[1][1], P_pred[1][2] / S[2][2]},
        {P_pred[2][0] / S[0][0], P_pred[2][1] / S[1][1], P_pred[2][2] / S[2][2]}
    };

    // Update state estimate
    X.x = X_pred.x + K[0][0] * Y[0] + K[0][1] * Y[1] + K[0][2] * Y[2];
    X.y = X_pred.y + K[1][0] * Y[0] + K[1][1] * Y[1] + K[1][2] * Y[2];
    X.theta = X_pred.theta + K[2][0] * Y[0] + K[2][1] * Y[1] + K[2][2] * Y[2];

    // Update covariance matrix
    P[0][0] = P_pred[0][0] - K[0][0] * S[0][0] * K[0][0] - K[0][1] * S[1][1] * K[0][1] - K[0][2] * S[2][2] * K[0][2];
    P[0][1] = P_pred[0][1] - K[0][0] * S[0][0] * K[1][0] - K[0][1] * S[1][1] * K[1][1] - K[0][2] * S[2][2] * K[1][2];
    P[0][2] = P_pred[0][2] - K[0][0] * S[0][0] * K[2][0] - K[0][1] * S[1][1] * K[2][1] - K[0][2] * S[2][2] * K[2][2];
    P[1][0] = P_pred[1][0] - K[1][0] * S[0][0] * K[0][0] - K[1][1] * S[1][1] * K[0][1] - K[1][2] * S[2][2] * K[0][2];
    P[1][1] = P_pred[1][1] - K[1][0] * S[0][0] * K[1][0] - K[1][1] * S[1][1] * K[1][1] - K[1][2] * S[2][2] * K[1][2];
    P[1][2] = P_pred[1][2] - K[1][0] * S[0][0] * K[2][0] - K[1][1] * S[1][1] * K[2][1] - K[1][2] * S[2][2] * K[2][2];
    P[2][0] = P_pred[2][0] - K[2][0] * S[0][0] * K[0][0] - K[2][1] * S[1][1] * K[0][1] - K[2][2] * S[2][2] * K[0][2];
    P[2][1] = P_pred[2][1] - K[2][0] * S[0][0] * K[1][0] - K[2][1] * S[1][1] * K[1][1] - K[2][2] * S[2][2] * K[1][2];
    P[2][2] = P_pred[2][2] - K[2][0] * S[0][0] * K[2][0] - K[2][1] * S[1][1] * K[2][1] - K[2][2] * S[2][2] * K[2][2];

    currentPosition.x = X.x;
    currentPosition.y = X.y;
    currentPosition.h = position_eg.h;

    return currentPosition;
}

EKF odometry_fusion()
{
	EKF currentPosition;
	external_global position_eg = odometry_eg();
	internal_global position_ig = odometry_ig();

	currentPosition.x = (position_eg.x + position_ig.x)/2.0;
	currentPosition.y = (position_eg.y + position_ig.y)/2.0;
	currentPosition.h = position_eg.h;

	return currentPosition;
}

void displayCounter()
{
	lcd_set_cursor(0, 0);
	sprintf(buffer, "I1:%d", counterIN1);
	lcd_write_string(buffer);
	lcd_set_cursor(1, 0);
	sprintf(buffer, "I2:%d", counterIN2);
	lcd_write_string(buffer);
	lcd_set_cursor(2, 0);
	sprintf(buffer, "I3:%d", counterIN3);
	lcd_write_string(buffer);
	lcd_set_cursor(3, 0);
	sprintf(buffer, "I4:%d", counterIN4);
	lcd_write_string(buffer);
	lcd_set_cursor(0, 10);
	sprintf(buffer, "E1:%d", counter1);
	lcd_write_string(buffer);
	lcd_set_cursor(1, 10);
	sprintf(buffer, "E2:%d", counter2);
	lcd_write_string(buffer);
	lcd_set_cursor(2, 10);
	sprintf(buffer, "E3:%d", counter3);
	lcd_write_string(buffer);
}

void display_EG()
{
	external_global position = odometry_eg();

	lcd_set_cursor(0, 0);
	sprintf(buffer, "X:%.2f", position.x);
	lcd_write_string(buffer);
	lcd_set_cursor(1, 0);
	sprintf(buffer, "Y:%.2f", position.y);
	lcd_write_string(buffer);
	lcd_set_cursor(2, 0);
	sprintf(buffer, "H:%.2f", position.h);
	lcd_write_string(buffer);
}

void display_EL()
{
	external_local position = odometry_el();

	lcd_set_cursor(0, 0);
	sprintf(buffer, "X:%.2f", position.x);
	lcd_write_string(buffer);
	lcd_set_cursor(1, 0);
	sprintf(buffer, "Y:%.2f", position.y);
	lcd_write_string(buffer);
	lcd_set_cursor(2, 0);
	sprintf(buffer, "H:%.2f", position.h);
	lcd_write_string(buffer);
	lcd_set_cursor(3, 0);
	sprintf(buffer, "P:%.2f", sensorData[1]);
	lcd_write_string(buffer);
	lcd_set_cursor(3, 10);
	sprintf(buffer, "R:%.2f", sensorData[2]);
	lcd_write_string(buffer);
}
