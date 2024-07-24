/**
 * @file	robot_control.c
 * @author	INVICTO TEAM (Arduino 2021)
 * @brief	Robot control library.
 * 			This file provide all robot control mechanisms including:
 * 			 + Autonomous robot movement using Odometry and PID Controller
 * 			 + Camera focus mechanism so that the ball position can be followed by the camera
 * 			 + Detect blue/red ball and take it to get into the silo
 * 			 + Dummy ball throwing mechanism if it was picked up by the robot
 */

#include <Library/inc/robot_control.h>

extern TIM_HandleTypeDef htim2;
#define SILO_NOT_DETECTED_DISTANCE 99990
#define SILO_NOT_DETECTED_ANGLE 999
#define MAX_SILOS 5

extern double sensorData[3];
extern int camera[18];
extern int sensorMEGA[4];
char buffCAM[10];
uint8_t lookingMode = 1;

Silo silos[MAX_SILOS];

/**
 * @brief	Convert range value of a variable to another range value.
 *
 * @param 	afterFrom after convert value (desired value).
 * @param 	afterTo after convert value (desired value).
 * @param 	beforeFrom before convert value.
 * @param 	beforeTo before convert value.
 * @param 	value variable to convert.
 *
 * @return 	The new converted value.
 */
int map(int afterFrom, int afterTo, int beforeFrom, int beforeTo, int value)
{
    return (1.0 * (value - afterFrom)) / ((afterTo - afterFrom) * 1.0) * (beforeTo - beforeFrom) + beforeFrom;
}

/**
 * @brief	Calculates the new velocity for a trapezoidal velocity profile.
 *
 * This function computes the next velocity value in a trapezoidal velocity profile given the current velocity, target position,
 * maximum velocity, maximum acceleration, maximum deceleration, and time step. It ensures smooth acceleration and deceleration.
 *
 * @param	current The current velocity.
 * @param	target The target position.
 * @param	maxVel The maximum velocity.
 * @param	maxAccel The maximum acceleration.
 * @param	maxDecel The maximum deceleration.
 * @param	dt The time step.
 *
 * @return	The new velocity.
 */
double rampTrapezoidal(double current, double target, double maxVel, double maxAccel, double maxDecel, double dt)
{
    double delta = target - current;
    double targetVel = 0;

    if (fabs(delta) < 1e-5)
    { // Close enough to target
        return 0;
    }

    double direction = (delta > 0) ? 1 : -1;
    double distanceToTarget = fabs(delta);

    // Determine the maximum velocity that can be reached given the distance to the target
    double velLimitByDistance = sqrt(2 * maxDecel * distanceToTarget);
    targetVel = fmin(maxVel, velLimitByDistance);

    // Accelerate or decelerate to the target velocity
    if (current < targetVel)
    {
        current += maxAccel * dt;
        if (current > targetVel)
        {
            current = targetVel;
        }
    }
    else if (current > targetVel)
    {
        current -= maxDecel * dt;
        if (current < targetVel)
        {
            current = targetVel;
        }
    }

    return current * direction;
}

/**
 * @brief	Limits the rate of change between the current value and the target value.
 *
 * This function ensures that the change from the current value to the target value does not exceed the specified maximum rate of change.
 * If the difference exceeds the maximum change, it returns a new value that is incremented by the maximum change in the appropriate direction.
 *
 * @param	current The current value.
 * @param	target The target value.
 * @param	maxChange The maximum allowed change per step.
 *
 * @return	The new value, limited by the maximum rate of change.
 */
double limitRateOfChange(double current, double target, double maxChange)
{
    if (fabs(target - current) > maxChange)
    {
        return current + (target > current ? maxChange : -maxChange);
    }
    return target;
}

/**
 * @brief	Smooths the velocities using a simple low-pass filter.
 *
 * This function applies a low-pass filter to the velocities Vx, Vy, and W to smooth them out.
 * It updates the velocities in place using a smoothing factor.
 *
 * @param	Vx Pointer to the velocity in the x direction.
 * @param	Vy Pointer to the velocity in the y direction.
 * @param	W Pointer to the angular velocity.
 * @param	smoothingFactor The factor used for smoothing, a value between 0 and 1.
 *
 * @return	None.
 */
void smoothVelocity(double* Vx, double* Vy, double* W, double smoothingFactor)
{
    double prevVx = 0.0;
    double prevVy = 0.0;
    double prevW = 0.0;

    *Vx = (1.0 - smoothingFactor) * prevVx + smoothingFactor * (*Vx);
    *Vy = (1.0 - smoothingFactor) * prevVy + smoothingFactor * (*Vy);
    *W = (1.0 - smoothingFactor) * prevW + smoothingFactor * (*W);

    prevVx = *Vx;
    prevVy = *Vy;
    prevW = *W;
}

/**
 * @brief	Limits the acceleration between the current and target velocities.
 *
 * This function ensures that the change in velocity does not exceed the maximum allowed acceleration.
 * If the difference between the current and target velocities is greater than the maximum acceleration,
 * it adjusts the current velocity by the maximum acceleration in the appropriate direction.
 *
 * @param	currentVelocity The current velocity.
 * @param	targetVelocity The target velocity.
 * @param	maxAccel The maximum allowed acceleration.
 *
 * @return	The new velocity, limited by the maximum acceleration.
 */
double limitAcceleration(double currentVelocity, double targetVelocity, double maxAccel)
{
    double delta = targetVelocity - currentVelocity;
    if (fabs(delta) > maxAccel)
    {
        return currentVelocity + (delta > 0 ? maxAccel : -maxAccel);
    }
    return targetVelocity;
}

/**
 * @brief	Checking the robot position if it has reached target position.
 *
 * This function check the position of the robot in a global coordinate system with external encoders and returning boolean value.
 *
 * @param	setpoint The target position in mm {x, y, h}.
 * @param	position The current position of the robot in mm {x, y, h}.
 * @param	xyTolerance The tolerance value in x and y direction to considered the robot has reached the target position.
 * @param	hTolerance The tolerance value to considered the robot has reached the heading target.
 *
 * @return	True	-> has reached target position.
 * 			False	-> has not reached the target position.
 */
bool atTargetEG(external_global setpoint, external_global position, double xyTolerance, double hTolerance)
{
	double error_x = setpoint.x - position.x;
	double error_y = setpoint.y - position.y;
	double error_h = fabs(setpoint.h - position.h);
	double distance = hypot(error_x, error_y);
	return distance < xyTolerance && error_h < hTolerance;
}
/**
 * @brief	Checking the robot position if it has reached target position.
 *
 * This function check the position of the robot in a local coordinate system with external encoders and returning boolean value.
 *
 * @param	setpoint The target position in mm {x, y, h}.
 * @param	position The current position of the robot in mm {x, y, h}.
 * @param	xyTolerance The tolerance value in x and y direction to considered the robot has reached the target position.
 * @param	hTolerance The tolerance value to considered the robot has reached the heading target.
 *
 * @return	True	-> has reached target position.
 * 			False	-> has not reached the target position.
 */
bool atTargetEL(external_local setpoint, external_local position, double xyTolerance, double hTolerance)
{
	double error_x = setpoint.x - position.x;
	double error_y = setpoint.y - position.y;
	double error_h = fabs(setpoint.h - position.h);
	double distance = hypot(error_x, error_y);
	return distance < xyTolerance && error_h < hTolerance;
}

/**
 * @brief	Find ball position mechanism.
 *
 * This function implements find ball mechanism, the robot heading move to 2 different desired heading target alternately.
 *
 * @param	targetAngle1 The first heading target.
 * @param	targetAngle2 The second heading target.
 * @param	currentAngle The current heading.
 *
 * @return	None.
 */
void lookForTheBall(double targetAngle1, double targetAngle2, double currentAngle)
{
    double targetAngle;
    uint8_t nextMode;

    switch (lookingMode)
    {
        case 1:
        	targetAngle = targetAngle2;
            nextMode = 2;
            break;
        case 2:
        	targetAngle = targetAngle1;
            nextMode = 1;
            break;
        default:
            return; // Invalid mode, do nothing
    }

    double W = PID_controllerH(targetAngle, currentAngle, 0.7);
    Inverse_Kinematics(0, 0, W);
    if (fabs(targetAngle - currentAngle) < 5.0)
    {
        lookingMode = nextMode;
    }
}

/**
 * @brief	Set desired servo angle.
 *
 * This function using for setting servo angle position.
 *
 * @param	angle Desired angle.
 * @return	None.
 */
void servo_write(int angle)
{
	int i = map(0, 180, 30, 130, angle);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, i);
}

/**
 * @brief	Executes a PID control loop for a global coordinate using external encoders.
 *
 * This function computes the necessary velocities to move towards a setpoint using PID control for x, y, and heading.
 * It also takes into account the smoothing of velocities and limits the velocities based on IMU pitch and roll data.
 *
 * @param	setpoint The target position in mm {x, y, h}.
 * @param	Kp Proportional gain for the PID controller.
 * @param	Ki Integral gain for the PID controller.
 * @param	Kd Derivative gain for the PID controller.
 * @param	KpH Proportional gain for the heading.
 * @param	smoothingFactor The factor used for smoothing velocities.
 * @param	maxVelocity The maximum allowed velocity.
 *
 * @return	None.
 */
void PID_EG(external_global setpoint, double Kp, double Ki, double Kd, double KpH, double smoothingFactor, double maxVelocity)
{
	// get global current position from external encoders.
    external_global currentPosition = odometry_eg();

    // pitch and roll value from MPU6050.
    double pitch = sensorData[1] * 300;
    double roll = sensorData[2] * 300;

    // error position from robot current position to the target position.
    double error_x = setpoint.x - currentPosition.x;
    double error_y = setpoint.y - currentPosition.y;

    // current heading position of the robot in radian.
    double current_heading_rad = currentPosition.h * M_PI / 180.0;

    // distance value from the error calculation in x and y direction.
    double distance = hypot(error_x, error_y);

    // safe velocity that allowed for reaching the desired position.
    double velocityFactor = fmin(distance, maxVelocity) / distance;

    // velocity values from x and y direction and applied maximum velocity.
    double Vx_local = PID_controller(setpoint.x, currentPosition.x, Kp, Ki, Kd) * velocityFactor;
    double Vy_local = PID_controller(setpoint.y, currentPosition.y, Kp, Ki, Kd) * velocityFactor;

    // convert velocity values to a global coordinate system.
    // you can look on rotation(clockwise) formula in geometric transformation.
    double Vx = Vx_local * cos(current_heading_rad) - Vy_local * sin(current_heading_rad);
    double Vy = Vx_local * sin(current_heading_rad) + Vy_local * cos(current_heading_rad);
    double W = PID_controllerH(setpoint.h, currentPosition.h, KpH);

    // velocity increases if the robot is on uphill terrain.
    if(roll > 0)	{Vx -= roll;}
    else			{Vx += roll;}
    if(pitch > 0)	{Vy += pitch;}
    else			{Vy -= pitch;}

    // applied smoothing velocity to the robot so that it can increase the speed gradually.
    smoothVelocity(&Vx, &Vy, &W, smoothingFactor);
    Inverse_Kinematics(Vx, Vy, W);
}

void PID_EL(external_local setpoint, double Kp, double Ki, double Kd, double KpH, double smoothingFactor, double maxVelocity)
{
	// get local current position from external encoders.
	external_local currentPosition = odometry_el();

	double pitch = sensorData[1] * 300;
	double roll = sensorData[2] * 300;

    double error_x = setpoint.x - currentPosition.x;
    double error_y = setpoint.y - currentPosition.y;

    double heading = atan2(error_y, error_x);
    double current_heading_rad = currentPosition.h * M_PI / 180.0;
    double heading_rad = heading - current_heading_rad;

    double distance = hypot(error_x, error_y);
    double velocityFactor = fmin(distance, maxVelocity) / distance;

    double Vx = PID_controller(setpoint.x, currentPosition.x, Kp, Ki, Kd) * cos(heading_rad) * velocityFactor;
    double Vy = PID_controller(setpoint.y, currentPosition.y, Kp, Ki, Kd) * sin(heading_rad) * velocityFactor;
    double W = PID_controllerH(setpoint.h, currentPosition.h, KpH);

    if(roll > 0)	{Vx -= roll;}
    else			{Vx += roll;}
    if(pitch > 0)	{Vy += pitch;}
    else			{Vy -= pitch;}

    smoothVelocity(&Vx, &Vy, &W, smoothingFactor);
    Inverse_Kinematics(Vx, Vy, W);
}

void PID_IG(internal_global setpoint, double Kp, double Ki, double Kd, double KpH, double smoothingFactor, double maxVelocity)
{
	// get global current position from internal encoders.
	internal_global currentPosition = odometry_ig();

	double pitch = sensorData[1] * 300;
	double roll = sensorData[2] * 300;

    double error_x = setpoint.x - currentPosition.x;
    double error_y = setpoint.y - currentPosition.y;
    double current_heading_rad = currentPosition.h * M_PI / 180.0;

    double distance = hypot(error_x, error_y);
    double velocityFactor = fmin(distance, maxVelocity) / distance;

    double Vx_local = PID_controller(setpoint.x, currentPosition.x, Kp, Ki, Kd) * velocityFactor;
    double Vy_local = PID_controller(setpoint.y, currentPosition.y, Kp, Ki, Kd) * velocityFactor;

    double Vx = Vx_local * cos(current_heading_rad) - Vy_local * sin(current_heading_rad);
    double Vy = Vx_local * sin(current_heading_rad) + Vy_local * cos(current_heading_rad);
    double W = PID_controllerH(setpoint.h, currentPosition.h, KpH);

    if(roll > 0)	{Vx -= roll;}
    else			{Vx += roll;}
    if(pitch > 0)	{Vy += pitch;}
    else			{Vy -= pitch;}

    smoothVelocity(&Vx, &Vy, &W, smoothingFactor);
    Inverse_Kinematics(Vx, Vy, W);
}

void PID_IL(internal_local setpoint, double Kp, double Ki, double Kd, double KpH, double smoothingFactor, double maxVelocity)
{
	// get local current position from internal encoders.
	internal_local currentPosition = odometry_il();

	double pitch = sensorData[1] * 300;
	double roll = sensorData[2] * 300;

    double error_x = setpoint.x - currentPosition.x;
    double error_y = setpoint.y - currentPosition.y;

    double heading = atan2(error_y, error_x);
    double current_heading_rad = currentPosition.h * M_PI / 180.0;
    double heading_rad = heading - current_heading_rad;

    double distance = hypot(error_x, error_y);
    double velocityFactor = fmin(distance, maxVelocity) / distance;

    double Vx = PID_controller(setpoint.x, currentPosition.x, Kp, Ki, Kd) * cos(heading_rad) * velocityFactor;
    double Vy = PID_controller(setpoint.y, currentPosition.y, Kp, Ki, Kd) * sin(heading_rad) * velocityFactor;
    double W = PID_controllerH(setpoint.h, currentPosition.h, KpH);

    if(roll > 0)	{Vx -= roll;}
    else			{Vx += roll;}
    if(pitch > 0)	{Vy += pitch;}
    else			{Vy -= pitch;}

    smoothVelocity(&Vx, &Vy, &W, smoothingFactor);
    Inverse_Kinematics(Vx, Vy, W);
}

void PID_KFtocoordinate(EKF setpoint, double Kp, double Ki, double Kd, double KpH, double smoothingFactor, double maxVelocity)
{
	// get global current position from internal and external encoders combined.
    EKF currentPosition = odometry_fusion();

	double pitch = sensorData[1] * 300;
	double roll = sensorData[2] * 300;

    double error_x = setpoint.x - currentPosition.x;
    double error_y = setpoint.y - currentPosition.y;
    double current_heading_rad = currentPosition.h * M_PI / 180.0;

    double distance = hypot(error_x, error_y);
    double velocityFactor = fmin(distance, maxVelocity) / distance;

    double Vx_local = fabs(PID_controller(setpoint.x, currentPosition.x, Kp, Ki, Kd)) * velocityFactor;
    double Vy_local = fabs(PID_controller(setpoint.y, currentPosition.y, Kp, Ki, Kd)) * velocityFactor;

    double Vx = Vx_local * cos(current_heading_rad) - Vy_local * sin(current_heading_rad);
    double Vy = Vx_local * sin(current_heading_rad) + Vy_local * cos(current_heading_rad);
    double W = PID_controllerH(setpoint.h, currentPosition.h, KpH);

    if(roll > 0)	{Vx -= roll;}
    else			{Vx += roll;}
    if(pitch > 0)	{Vy += pitch;}
    else			{Vy -= pitch;}

    smoothVelocity(&Vx, &Vy, &W, smoothingFactor);
    Inverse_Kinematics(Vx, Vy, W);
}

/**
 * @brief	Moves the robot to a series of coordinates using PID control.
 *
 * This function guides the robot to a series of setpoints using PID control for x, y, and heading.
 * It adjusts velocities based on the current position and IMU data, applies smoothing to the velocities,
 * and updates the target point once the current target is reached.
 *
 * @param	setpoint Array of target coordinates and headings.
 * @param	parameters Array of PID parameters for each setpoint.
 * @param	numPoints The number of setpoints in the setpoint array.
 *
 * @return	None.
 */
void PID_moveToCoordinate(external_global *setpoint, PID_parameter *parameters, uint16_t numPoints)
{
    static uint8_t thread = 0;
    if(thread >= numPoints)
    {
        Inverse_Kinematics(0, 0, 0);
        return;
    }

	// get global current position from external encoders.
    external_global currentPosition = odometry_eg();

    double pitch = sensorData[1] * 300;
    double roll = sensorData[2] * 300;

    double error_x = setpoint[thread].x - currentPosition.x;
    double error_y = setpoint[thread].y - currentPosition.y;
    double current_heading_rad = currentPosition.h * M_PI / 180.0;

    double distance = hypot(error_x, error_y);
    double velocityFactor = fmin(distance, parameters[thread].maxVelocity) / distance;

    double Vx_local = PID_controller(setpoint[thread].x, currentPosition.x, parameters[thread].KP, parameters[thread].KI, parameters[thread].KD) * velocityFactor;
    double Vy_local = PID_controller(setpoint[thread].y, currentPosition.y, parameters[thread].KP, parameters[thread].KI, parameters[thread].KD) * velocityFactor;

    double Vx = Vx_local * cos(current_heading_rad) - Vy_local * sin(current_heading_rad);
    double Vy = Vx_local * sin(current_heading_rad) + Vy_local * cos(current_heading_rad);
    double W = PID_controllerH(setpoint[thread].h, currentPosition.h, parameters[thread].KpH);

    if(roll > 0)	{Vx -= roll;}
    else			{Vx += roll;}
    if(pitch > 0)	{Vy += pitch;}
    else			{Vy -= pitch;}

    // Check if the robot has reached current target or not, if it is the robot go to the next target position.
    if(atTargetEG(setpoint[thread], currentPosition, parameters[thread].xyTolerance, parameters[thread].hTolerance))
    {
        thread++;
    }
    else
    {
        smoothVelocity(&Vx, &Vy, &W, parameters[thread].smoothingFactor);
        Inverse_Kinematics(Vx, Vy, W);
    }
}

/**
 * @brief	Moves the robot heading to desired heading.
 *
 * This function change robot heading.
 *
 * @param	setpoint_h Target heading.
 * @param	KpH Kp parameter.
 *
 * @return	None.
 */
void PID_setDegree(double setpoint_h, double KpH)
{
	external_global currentPosition = odometry_eg();

    double W = PID_controllerH(setpoint_h, currentPosition.h, KpH);
    Inverse_Kinematics(0, 0, W);
}

/**
 * @brief	Adjusts the focus of a servo motor to track a ball detected by the camera.
 *
 * This function calculates the position and angle of a ball based on camera input and adjusts a servo motor to maintain focus on the ball.
 * If the ball is not detected, it sets the servo to a default position.
 *
 * @param	None.
 * @return	None.
 */
void focusToTheBall()
{
    // Get ball distance from camera and convert to mm.
    int ballDistance = camera[0] * 10;

    // Get ball angle from camera.
    int ballAngle = camera[1];

    // Calculate ball position in y direction based on robot POV.
    int yBALL = cos(ballAngle * M_PI / 180.0) * ballDistance;

    // Distance from camera to the ground.
    int gndtoCam = 518; // in mm
//    static bool increasing = true;
//    static int i = 95;

    if(ballDistance > 0)
    {
        // Calculate focus angle and map it to servo range.
        double focus = atan2(yBALL, gndtoCam) * 180.0 / M_PI;
        int focusMapping = map(0, 90, 45, 130, focus);
        servo_write(focusMapping);
    }
    else
    {
        // Set servo to default position if ball is not detected.
        servo_write(97);
//		if(increasing)
//		{
//			i+=5;
//			if(i >= 125) increasing = false;
//		}
//		else
//		{
//			i-=5;
//			if(i <= 95) increasing = true;
//		}
//		servo_write(i);
    }
}

/**
 * @brief	Set silos initialize values.
 *
 * This function set the initial values of the silos.
 *
 * @param	None.
 * @return	None.
 */
void initializeSilos()
{
    for (int i = 0; i < MAX_SILOS; i++)
    {
        silos[i].distance = SILO_NOT_DETECTED_DISTANCE;
        silos[i].angle = SILO_NOT_DETECTED_ANGLE;
        silos[i].detected = false;
        silos[i].ballInSilo = 0;
        silos[i].x = silos[i].distance * sin(silos[i].angle * M_PI/180.0);
        silos[i].y = silos[i].distance * cos(silos[i].angle * M_PI/180.0);
    }
}

/**
 * @brief	Detects silos and stores their information.
 *
 * This function scans for silos using camera data and updates the global coordinates of each detected silo.
 * It returns the nearest detected silo with the ball less than 3 in it.
 *
 * @param	None.
 *
 * @return	The nearest silo with the ball less than 3 in it. Data that stored including:
 * 			 + silo distance
 * 			 + silo angle
 * 			 + boolean value (is the silo detected or not)
 * 			 + amount of balls in best silo
 * 			 + silo x global coordinate
 * 			 + silo y global coordinate
 */
Silo detectAndStoreSilo()
{
	// initialize default silos data.
    Silo bestSilo;
    bestSilo.distance = SILO_NOT_DETECTED_DISTANCE;
    bestSilo.angle = SILO_NOT_DETECTED_ANGLE;
    bestSilo.detected = false;

    // set camera to default position.
    servo_write(120);

    // get global current position of the robot based on external encoders.
    external_global position = odometry_eg();

    // store distance of each silos that detected in mm.
    int siloDistances[MAX_SILOS] = {camera[3] * 10, camera[5] * 10, camera[7] * 10, camera[9] * 10, camera[11] * 10}; // convert to mm

    // store angle of each silos that detected in degree.
    int siloAngles[MAX_SILOS] = {camera[4], camera[6], camera[8], camera[10], camera[12]};

    // get amount of ball in each silo.
    int ballExistence[MAX_SILOS] = {camera[13], camera[14], camera[15], camera[16], camera[17]};

    // check each silos data.
    for (int i = 0; i < MAX_SILOS; i++)
    {
    	// check if distance and angle of the silo is not similar as its default value.
        if (siloDistances[i] != SILO_NOT_DETECTED_DISTANCE && siloAngles[i] != SILO_NOT_DETECTED_ANGLE)
        {
            silos[i].distance = siloDistances[i];
            silos[i].angle = siloAngles[i];
            silos[i].detected = true;
            silos[i].ballInSilo = ballExistence[i];

            // calculate global coordinates of the detected silo.
            silos[i].x = position.x + siloDistances[i] * sin(siloAngles[i] * M_PI / 180.0);
            silos[i].y = position.y + siloDistances[i] * cos(siloAngles[i] * M_PI / 180.0);

            // check if this is the nearest silo and the ball in it less than 3.
            if (silos[i].ballInSilo < 3 && silos[i].distance < bestSilo.distance)
            {
                bestSilo = silos[i];
            }
        }
        else
        {
            silos[i].detected = false;
        }
    }
    // returning best silo data.
    return bestSilo;
}

/**
 * @brief	Moves the robot to the best detected silo and places the ball inside.
 *
 * This function uses PID control to navigate the robot to the best detected silo.
 * If the robot reaches the silo or the silo is not detected, it follows predefined behaviors.
 *
 * @param	setpoint The position that robot can detected all of the silos.
 * @param	Kp The proportional gain for PID control.
 * @param	Ki The integral gain for PID control.
 * @param	Kd The derivative gain for PID control.
 * @param	KpH The proportional gain for heading PID control.
 * @param	smoothingFactor The factor used for velocity smoothing.
 * @param	maxVelocity The maximum velocity of the robot.
 *
 * @return	None.
 */
void placeBallInSilo(external_global setpoint, double Kp, double Ki, double Kd, double KpH, double smoothingFactor, double maxVelocity)
{
    // get global current position of the robot based on external encoders.
    external_global position = odometry_eg();

    // get best silo data.
    Silo bestSilo = detectAndStoreSilo();
    double Vx = 0.0, Vy = 0.0, W = 0.0;

    static uint32_t lastTime = 0;
    uint32_t timer = HAL_GetTick();

    setMotorSpeed(1, 0);
    setMotorSpeed(2, 0);
    setMotorSpeed(7, 0);

    // check if best silo is detected or not.
    if(bestSilo.detected)
    {
    	// store best silo data and calculate its error.
        double targetX = bestSilo.x;
        double targetY = bestSilo.y;
        double error_x = targetX - position.x;
        double error_y = targetY - position.y;

        // calculate distance of best silo.
        double distance = hypot(error_x, error_y);
        // applied maximum velocity for the robot to go to the best silo.
        double velocityFactor = fmin(distance, maxVelocity) / distance;

        // applied PID Controller.
        Vx = PID_controller(targetX, position.x, Kp, Ki, Kd) * velocityFactor;
        Vy = PID_controller(targetY, position.y, Kp+1, Ki, Kd) * velocityFactor;
        W = PID_controllerH(setpoint.h, position.h, KpH);

        // check if the best silo distance is less than 400 mm, then the robot just moving forward.
        if(bestSilo.distance <= 400) {Vx = 0; Vy = 2000; W = 0;}

        smoothVelocity(&Vx, &Vy, &W, smoothingFactor);
        Inverse_Kinematics(Vx, Vy, W);
        lastTime = timer;
    }

    // check if the best silo was recently detected.
    // In this condition, camera is not detected best silo anymore so just moving forward to approach the silo.
    else if(timer - lastTime <= 1600)
    {
        Inverse_Kinematics(0, 2000, 0);
    }
    // check if the best silo is not detected, then move the robot to the position that can detected all of the silos.
    else
    {
        PID_EG(setpoint, Kp, Ki, Kd, KpH, 0.75, 3000);
    }
}

/**
 * @brief	Finds and takes the ball using the robot's camera and PID control.
 *
 * This function uses camera data to detect a ball and moves the robot towards it using PID control.
 * If the ball is not detected, the robot searches for the ball using predefined search patterns.
 *
 * @param	findBall An array of target positions to search for the ball.
 * @return	None.
 */
void findAndTakeBall(external_global *findBall)
{
    /*
     * Camera data:
     * camera[0]: ball distance (in cm)
     * camera[1]: ball angle (in degree)
     * camera[2]: ball existence (inside the robot)
     * camera[3]: silo distance (in cm)
     * camera[4]: silo angle (in degree)
     */

    // get global current position of the robot based on external encoders.
    external_global position = odometry_eg();

    static uint32_t lastTimeBallSeen = 0;
    static uint32_t searchStartTime = 0;
    static uint8_t searchMode = 1;
    static uint8_t lastSearchMode = 1;

    uint32_t timer = HAL_GetTick();

    // get ball distance and angle.
    int ballDistance = camera[0] * 10; // convert to mm
    int ballAngle = camera[1];

    double Vx = 0.0;
    double Vy = 0.0;
    double W = 0.0;

    // applied following ball position.
    focusToTheBall();

    // check if the ball is detected.
    if (ballDistance > 0)
    {
    	// store x and y ball coordinate.
        double xBall = ballDistance * sin(ballAngle * M_PI / 180.0);
        double yBall = ballDistance * cos(ballAngle * M_PI / 180.0);

        // applied PID Controller.
        Vx = PID_controller(xBall, 0.0, 1.5, 0.0, 0.0);
        Vy = PID_controller(yBall, 0.0, 2.2, 0.0, 0.0) * 1.2;
        W = PID_controllerH(ballAngle, 0.0, 1.5);

        // check if the ball distance is less than 400, then just moving forward.
        if(ballDistance < 400) {Vx = 0; Vy = 2000; W = 0;}

        // applied approaching ball position and take it.
        Inverse_Kinematics(Vx, Vy, W);
        setMotorSpeed(1, -2500);
        lastTimeBallSeen = timer;
        searchStartTime = timer;
    }

    // check if the ball was recently seen, then just keep moving forward.
    else if (timer - lastTimeBallSeen <= 700)
    {
        Inverse_Kinematics(0, 2000, 0);
        setMotorSpeed(1, -2500);
    }

    // check if the ball is not detected, then stop the robot and search for the ball.
    else
    {
        setMotorSpeed(1, 0);
        setMotorSpeed(2, 0);
        setMotorSpeed(7, 0);

        if (searchStartTime == 0) {searchStartTime = timer;}

        /*
         * Searching for the ball with 4 case.
         * case 1: The robot heading is moving to the left and right.
         * case 2: The robot go to predefined target position.
         * case 3: The robot go to predefined target position.
         * case 4: The robot go to predefined target position.
         */
        switch(searchMode)
        {
        case 1:
            if(timer - searchStartTime >= 6000)
            {
                searchMode += lastSearchMode;
            }
            if(lastSearchMode == 1)
            {
                lookForTheBall(10.0, 170.0, position.h);
            }
            else if(lastSearchMode == 2)
            {
                lookForTheBall(-100.0, 100.0, position.h);
            }
            else if(lastSearchMode == 3)
            {
                lookForTheBall(-70.0, 70.0, position.h);
            }
            break;

        case 2:
            lastSearchMode = 2;
            PID_EG(findBall[1], 2.8, 0.0, 0.0, 1.3, 0.8, 2700);
            searchStartTime = timer;

            //check if the robot has reached predefined position, if it is then applied case 1.
            if(atTargetEG(findBall[1], position, 500, 10))
            {
                searchMode = 1;
            }
            break;

        case 3:
            lastSearchMode = 3;
            PID_EG(findBall[2], 2.0, 0.0, 0.0, 1.3, 0.8, 2700);
            searchStartTime = timer;

            //check if the robot has reached predefined position, if it is then applied case 1.
            if(atTargetEG(findBall[2], position, 500, 10))
            {
                searchMode = 1;
            }
            break;

        case 4:
            lastSearchMode = 1;
            PID_EG(findBall[0], 2.8, 0.0, 0.0, 1.3, 0.8, 2700);
            searchStartTime = timer;

            //check if the robot has reached predefined position, if it is then applied case 1.
            if(atTargetEG(findBall[0], position, 500, 10))
            {
                searchMode = 1;
            }
            break;
        }
    }
}

/**
 * @brief	Moves the robot to a specified location and throws a ball when at the target.
 *
 * This function applied throwing away the dummy ball at the target position if it is taken by the robot.
 *
 * @param	whereTo A structure containing the target coordinates {x, y, h}.
 * @param	Kp Proportional gain for the PID controller.
 * @param	Ki Integral gain for the PID controller.
 * @param	Kd Derivative gain for the PID controller.
 * @param	KpH Proportional gain for the heading PID controller.
 *
 * @return	None.
 */
void throwTheBall(external_global whereTo, double Kp, double Ki, double Kd, double KpH)
{
    external_global position = odometry_eg();

    if(atTargetEG(whereTo, position, 500, 5))
    {
        Inverse_Kinematics(0, 0, 0);
        setMotorSpeed(1, -2400);
        setMotorSpeed(2, -2000);
        setMotorSpeed(7, -2800);
        HAL_Delay(2500);
    }
    else
    {
        PID_EG(whereTo, Kp, Ki, Kd, KpH, 0.8, 2800);
    }
}

/**
 * @brief	Displays ball information from the camera on the LCD screen.
 *
 * This function retrieves ball information from the camera data and displays
 * it on an LCD screen. The displayed information includes the ball distance,
 * angle, and existence as detected by the camera.
 *
 * @param	None.
 * @return	None.
 */
void displayBall()
{
    lcd_set_cursor(0, 0);
    sprintf(buffCAM, "1:%d", camera[0]);
    lcd_write_string(buffCAM);

    lcd_set_cursor(1, 0);
    sprintf(buffCAM, "2:%d", camera[1]);
    lcd_write_string(buffCAM);

    lcd_set_cursor(2, 0);
    sprintf(buffCAM, "3:%d", camera[2]);
    lcd_write_string(buffCAM);

    lcd_set_cursor(3, 0);
    sprintf(buffCAM, "4:%d", camera[3]);
    lcd_write_string(buffCAM);
}

/**
 * @brief	Detects the nearest silo and displays its information on the LCD screen.
 *
 * This function detects the nearest silo using the `detectAndStoreSilo` function and
 * displays the silo's distance, angle, detection status, and ball presence status on
 * an LCD screen.
 *
 * @param	None.
 * @return	None.
 */
void displaySilo()
{
    Silo silos = detectAndStoreSilo();

    lcd_set_cursor(0, 0);
    sprintf(buffCAM, "Dist:%d", silos.distance);
    lcd_write_string(buffCAM);

    lcd_set_cursor(1, 0);
    sprintf(buffCAM, "Angle:%d", silos.angle);
    lcd_write_string(buffCAM);

    lcd_set_cursor(2, 0);
    sprintf(buffCAM, "Detect:%d", silos.detected);
    lcd_write_string(buffCAM);

    lcd_set_cursor(3, 0);
    sprintf(buffCAM, "Ball:%d", silos.ballInSilo);
    lcd_write_string(buffCAM);
}
