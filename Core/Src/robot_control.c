#include <robot_control.h>

extern TIM_HandleTypeDef htim2;
#define SILO_NOT_DETECTED_DISTANCE 99990
#define SILO_NOT_DETECTED_ANGLE 999
#define MAX_SILOS 5

extern double sensorData[3];
extern int camera[13];
extern int sensorMEGA[5];
char buffCAM[10];
uint8_t lookingMode = 1;

Silo silos[MAX_SILOS];

int map(int st1, int fn1, int st2, int fn2, int value)
{
    return (1.0 * (value - st1)) / ((fn1 - st1) * 1.0) * (fn2 - st2) + st2;
}

double lowPassFilter(double newValue, double oldValue, double alpha)
{
	return alpha * newValue + (1 - alpha) * oldValue;
}

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

double limitRateOfChange(double current, double target, double maxChange)
{
    if (fabs(target - current) > maxChange)
    {
        return current + (target > current ? maxChange : -maxChange);
    }
    return target;
}

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

double limitAcceleration(double currentVelocity, double targetVelocity, double maxAccel)
{
    double delta = targetVelocity - currentVelocity;
    if (fabs(delta) > maxAccel)
    {
        return currentVelocity + (delta > 0 ? maxAccel : -maxAccel);
    }
    return targetVelocity;
}

bool atTargetExternal(robotPosition setpoint, robotPosition position, double xyTolerance, double Htolerance)
{
	double error_x = setpoint.x_global - position.x_global;
	double error_y = setpoint.y_global - position.y_global;
	double error_h = fabs(setpoint.h - position.h);
	double distance = hypot(error_x, error_y);
	return distance < xyTolerance && error_h < Htolerance;
}

bool atTargetPosition(EKF setpoint, EKF position, double xyTolerance, double Htolerance)
{
	double error_x = setpoint.x - position.x;
	double error_y = setpoint.y - position.y;
	double error_h = fabs(setpoint.h - position.h);
	double distance = hypot(error_x, error_y);
	return distance < xyTolerance && error_h < Htolerance;
}

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

    double W = PID_controllerH(targetAngle, currentAngle, 1.3);
    putar(0, 0, W);
    if (fabs(targetAngle - currentAngle) < 5.0)
    {
        lookingMode = nextMode;
    }
}

void servo_write(int angle)
{
	int i = map(0, 180, 10, 65, angle);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, i);
}

void PID_External(robotPosition setpoint, double Kp, double Ki, double Kd, double KpH, double smoothingFactor, double maxVelocity)
{
    robotPosition currentPosition = odometry();

	double pitch = sensorData[1] * 300; // IMU pitch
	double roll = sensorData[2] * 300; // IMU roll

    double error_x = setpoint.x_global - currentPosition.x_global;
    double error_y = setpoint.y_global - currentPosition.y_global;

    double heading = atan2(error_y, error_x);
    double current_heading_rad = currentPosition.h * M_PI / 180.0;
    double heading_rad = heading - current_heading_rad;

    double distance = hypot(error_x, error_y);
    double velocityFactor = fmin(distance, maxVelocity) / distance;

    double Vx_local = PID_controller(setpoint.x_global, currentPosition.x_global, Kp, Ki, Kd) * cos(heading_rad) * velocityFactor;
    double Vy_local = PID_controller(setpoint.y_global, currentPosition.y_global, Kp, Ki, Kd) * sin(heading_rad) * velocityFactor;

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

void PID_Internal(robotPosition setpoint, double Kp, double Ki, double Kd, double KpH, double smoothingFactor, double maxVelocity)
{
    robotPosition currentPosition = odometry();

	double pitch = sensorData[1] * 300; // IMU pitch
	double roll = sensorData[2] * 300; // IMU roll

    double error_x = setpoint.x_in_global - currentPosition.x_in_global;
    double error_y = setpoint.y_in_global - currentPosition.y_in_global;

    double heading = atan2(error_y, error_x);
    double current_heading_rad = currentPosition.h * M_PI / 180.0;
    double heading_rad = heading - current_heading_rad;

    double distance = hypot(error_x, error_y);
    double velocityFactor = fmin(distance, maxVelocity) / distance;

    double Vx_local = PID_controller(setpoint.x_in_global, currentPosition.x_in_global, Kp, Ki, Kd) * cos(heading_rad) * velocityFactor;
    double Vy_local = PID_controller(setpoint.y_in_global, currentPosition.y_in_global, Kp, Ki, Kd) * sin(heading_rad) * velocityFactor;

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

void PID_KFtocoordinate(EKF setpoint, double Kp, double Ki, double Kd, double KpH, double smoothingFactor, double maxVelocity)
{
    EKF currentPosition = extendedKalmanFilter();
//	robotPosition currentPosition = odometry();

	double pitch = sensorData[1] * 300; // IMU pitch
	double roll = sensorData[2] * 300; // IMU roll

    double error_x = setpoint.x - currentPosition.x;
    double error_y = setpoint.y - currentPosition.y;

    double heading = atan2(error_y, error_x);
    double current_heading_rad = currentPosition.h * M_PI / 180.0; // IMU yaw
    double heading_rad = heading - current_heading_rad;

    double distance = hypot(error_x, error_y);
    double velocityFactor = fmin(distance, maxVelocity) / distance;

    double Vx_local = fabs(PID_controller(setpoint.x, currentPosition.x, Kp, Ki, Kd)) * cos(heading_rad) * velocityFactor;
    double Vy_local = fabs(PID_controller(setpoint.y, currentPosition.y, Kp, Ki, Kd)) * sin(heading_rad) * velocityFactor;

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

void PID_moveToCoordinate(EKF *setpoint, double Kp, double Ki, double Kd, double KpH, double tolerance, double maxVelocity)
{
    static uint8_t index = 0;
    uint16_t amount = sizeof(setpoint);
    if (index >= amount)
    {
        Inverse_Kinematics(0, 0, 0); // Stop the robot
        return;
    }

    EKF currentPosition = extendedKalmanFilter();

    double error_x = setpoint[index].x - currentPosition.x;
    double error_y = setpoint[index].y - currentPosition.y;

    double heading = atan2(error_y, error_x);
    double current_heading_rad = currentPosition.h * M_PI / 180.0;
    double heading_rad = heading - current_heading_rad;

    double distance = hypot(error_x, error_y);
    double velocityFactor = fmin(distance, maxVelocity) / distance;

    double Vx_local = fabs(PID_controller(setpoint[index].x, currentPosition.x, Kp, Ki, Kd)) * cos(heading_rad) * velocityFactor;
    double Vy_local = fabs(PID_controller(setpoint[index].y, currentPosition.y, Kp, Ki, Kd)) * sin(heading_rad) * velocityFactor;

    double Vx = Vx_local * cos(current_heading_rad) - Vy_local * sin(current_heading_rad);
    double Vy = Vx_local * sin(current_heading_rad) + Vy_local * cos(current_heading_rad);
    double W = PID_controllerH(setpoint[index].h, currentPosition.h, KpH);

    if (atTargetPosition(setpoint[index], currentPosition, tolerance, 1))
    {
        index++;
    }
    else
    {
        Inverse_Kinematics(Vx, Vy, W);
    }
}

void PID_gotoX(double setpoint_x, double Kp, double Ki, double Kd)
{
	robotPosition currentPosition = odometry();

	double sin_h = sin(currentPosition.h * M_PI / 180.0);
	double cos_h = cos(currentPosition.h * M_PI / 180.0);

	double Vx = PID_controller(setpoint_x, currentPosition.x_global, Kp, Ki, Kd) * cos_h;
    double Vy = PID_controller(0.0, currentPosition.y_global, Kp, Ki, Kd) * sin_h;
    double W = PID_controllerH(0.0, currentPosition.h, 1.5);
    Inverse_Kinematics(Vx, Vy, W);
}

void PID_gotoY(double setpoint_y, double Kp, double Ki, double Kd)
{
//	EKF currentPosition = extendedKalmanFilter();
	robotPosition currentPosition = odometry();

	double yaw_rad = currentPosition.h * M_PI / 180.0;
	double sin_h = sin(yaw_rad);
	double cos_h = cos(yaw_rad);

	double Vx = PID_controller(0.0, currentPosition.x_global, Kp, Ki, Kd) * sin_h;
    double Vy = PID_controller(setpoint_y, currentPosition.y_global, Kp, Ki, Kd) * cos_h;
    double W = PID_controllerH(0.0, currentPosition.h, 1.5);

    Vx -= W * sin(yaw_rad);
    Vy += W * cos(yaw_rad);

    Inverse_Kinematics(-Vx, Vy, W);
}

void PID_setDegree(double setpoint_h, double KpH)
{
//	EKF currentPosition = extendedKalmanFilter();
	robotPosition currentPosition = odometry();

    double W = PID_controllerH(setpoint_h, currentPosition.h, KpH);
    putar(0, 0, W);
//    Inverse_Kinematics(0, 0, W);
}

void focusToTheBall()
{
    int ballDistance = camera[0] * 10; // convert to mm
    int ballAngle = camera[1];
    int ballExistence = camera[2];
    int yBALL = cos(ballAngle * M_PI / 180.0) * ballDistance;

    int gndtoCam = 518; // in mm
//    static bool increasing = true;
//    static int i = 95;

    if(ballExistence != 0)
    {
        double focus = atan2(yBALL, gndtoCam) * 180.0 / M_PI;
        int focusMapping = map(0, 90, 50, 134, focus); // 50, 134
        servo_write(focusMapping);
    }
    else
    {
    	servo_write(110);
//    	if(increasing)
//    	{
//    		i+=5;
//    		if(i >= 125) increasing = false;
//    	}
//    	else
//    	{
//    		i-=5;
//    		if(i <= 95) increasing = true;
//    	}
//    	servo_write(i);
    }
}

// Initialize silos array
void initializeSilos()
{
    for (int i = 0; i < MAX_SILOS; i++)
    {
        silos[i].distance = SILO_NOT_DETECTED_DISTANCE;
        silos[i].angle = SILO_NOT_DETECTED_ANGLE;
        silos[i].detected = false;
        silos[i].ballInSilo = 0;
        silos[i].x = silos[i].distance * sin(silos[i].angle * M_PI/180);
        silos[i].y = silos[i].distance * cos(silos[i].angle * M_PI/180);
    }
}

Silo detectAndStoreSilo()
{
    Silo bestSilo;
    bestSilo.distance = SILO_NOT_DETECTED_DISTANCE;
    bestSilo.angle = SILO_NOT_DETECTED_ANGLE;
    bestSilo.detected = false;
    servo_write(126);

    robotPosition position = odometry();
    int siloDistances[MAX_SILOS] = {camera[3] * 10, camera[5] * 10, camera[7] * 10, camera[9] * 10, camera[11] * 10}; // convert to mm
    int siloAngles[MAX_SILOS] = {camera[4], camera[6], camera[8], camera[10], camera[12]};

//    int ballDistance = camera[0] * 10; // convert to mm
//    int ballAngle = camera[1];
    int ballExistence = camera[2];
//    double xBall = position.x_global + ballDistance * sin(ballAngle * M_PI / 180.0);

    for (int i = 0; i < MAX_SILOS; i++)
    {
        if (siloDistances[i] != SILO_NOT_DETECTED_DISTANCE && siloAngles[i] != SILO_NOT_DETECTED_ANGLE)
        {
            silos[i].distance = siloDistances[i];
            silos[i].angle = siloAngles[i];
            silos[i].detected = true;
            silos[i].ballInSilo = ballExistence;
            // Calculate global coordinates of the detected silo
            silos[i].x = position.x_global + siloDistances[i] * sin(siloAngles[i] * M_PI / 180.0);
            silos[i].y = position.y_global + siloDistances[i] * cos(siloAngles[i] * M_PI / 180.0);

            // Check if this is the nearest silo
            if (silos[i].distance < bestSilo.distance)
            {
                bestSilo = silos[i];
            }
        }
        else
        {
            silos[i].detected = false;
        }
    }
    return bestSilo;
}

void placeBallInSilo(robotPosition setpoint, double Kp, double Ki, double Kd, double KpH)
{
    robotPosition position = odometry();
    Silo bestSilo = detectAndStoreSilo();
    double Vx = 0.0, Vy = 0.0, W = 0.0;

    static uint32_t lastTime = 0;
    uint32_t timer = HAL_GetTick();

    setMotorSpeed(1, 0);
    setMotorSpeed(2, 0);
    setMotorSpeed(7, 0);

    if(bestSilo.detected && bestSilo.ballInSilo < 3)
    {
        double targetX = bestSilo.x;
        double targetY = bestSilo.y;
    //    double targetH = atan2(targetY - position.y_global, targetX - position.x_global) * 180.0 / M_PI;

        Vx = PID_controller(targetX, position.x_global, Kp, Ki, Kd);
        Vy = PID_controller(targetY, position.y_global, 1.6, Ki, Kd);
        W = PID_controllerH(90.0, position.h, KpH);

        if(bestSilo.distance <= 400) {Vx = 0; Vy = 1000; W = 0;}

        smoothVelocity(&Vx, &Vy, &W, 0.75);
        Inverse_Kinematics(Vx, Vy, W);
        lastTime = timer;
    }
    else if(timer - lastTime <= 1600)
    {
        Inverse_Kinematics(0, 1000, 0);
    }
    else
    {
    	PID_External(setpoint, Kp, Ki, Kd, KpH, 0.75, 3000);
    }
}

void findAndTakeBall()
{
    /*
     * Camera data:
     * camera[0]: ball distance (in cm)
     * camera[1]: ball angle (in degree)
     * camera[2]: ball existence (number of balls)
     * camera[3]: silo distance (in cm)
     * camera[4]: silo angle (in degree)
     */

    robotPosition position = odometry();

    static uint32_t lastTimeBallSeen = 0;
    static uint32_t searchStartTime = 0;
    static uint8_t searchMode = 1;
    static uint8_t lastSearchMode = 1;

    uint32_t timer = HAL_GetTick();

    int ballDistance = camera[0] * 10; // convert to mm
    int ballAngle = camera[1];
    int ballExistence = camera[2];

    double Vx = 0.0;
    double Vy = 0.0;
    double W = 0.0;

    robotPosition findBall1 = {-2800.0, 0.0, -90.0};
    robotPosition findBall2 = {-2800.0, 900.0, -179.0};
    robotPosition findBall3 = {-2800.0, -900.0, 0.0};

    focusToTheBall();

    if (ballExistence != 0)
    {
        double xBall = ballDistance * sin(ballAngle * M_PI / 180.0);
        double yBall = ballDistance * cos(ballAngle * M_PI / 180.0);

        Vx = PID_controller(xBall, 0.0, 1.0, 0.0, 0.0);
        Vy = PID_controller(yBall, 0.0, 1.4, 0.0, 0.0);
        W = PID_controllerH(ballAngle, 0.0, 1.0);

        if(ballDistance <= 400) {Vx = 0; Vy = 1000; W = 0;}

        Inverse_Kinematics(Vx, Vy, W);
        setMotorSpeed(1, -1000);
        lastTimeBallSeen = timer;
        searchStartTime = timer;
    }
    else if (timer - lastTimeBallSeen <= 700)
    {
        // If the ball was recently seen, keep moving forward
        Inverse_Kinematics(0, 1300, 0);
        setMotorSpeed(1, -1000);
    }
    else
    {
        // Stop the robot and search for the ball
        setMotorSpeed(1, 0);
        setMotorSpeed(2, 0);
        setMotorSpeed(7, 0);

        if (searchStartTime == 0) {searchStartTime = timer;}

        switch(searchMode)
        {
        case 1:
//        	putar(0, 0, 65);
        	if(timer - searchStartTime >= 6000)
        	{
        		searchMode += lastSearchMode;
        	}
        	if(lastSearchMode == 1)
        	{
        		lookForTheBall(-10.0, -170.0, position.h);
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
        	PID_External(findBall2, 1.8, 0.0, 0.0, 1.3, 0.8, 3000);
        	searchStartTime = timer;
        	if(atTargetExternal(findBall2, position, 100, 10))
        	{
        		searchMode = 1;
        	}
        	break;
        case 3:
        	lastSearchMode = 3;
        	PID_External(findBall3, 1.8, 0.0, 0.0, 1.3, 0.8, 3000);
        	searchStartTime = timer;
        	if(atTargetExternal(findBall3, position, 100, 10))
        	{
        		searchMode = 1;
        	}
        	break;
        case 4:
        	lastSearchMode = 1;
        	PID_External(findBall1, 1.8, 0.0, 0.0, 1.3, 0.8, 3000);
        	searchStartTime = timer;
        	if(atTargetExternal(findBall1, position, 100, 10))
        	{
        		searchMode = 1;
        	}
        	break;
        }
//        if (timer - searchStartTime >= 8000)
//        {
//            // If searching for more than 8 seconds, move to setpoint
//            PID_KFtocoordinate(setpoint, Kp, Ki, Kd, KpH, 0.65);
//        }
//        else
//        {
//        	putar(0, 0, 400);
//            // Implement a search pattern to find the ball
//            switch(searchMode)
//            {
//                case 1:
//                    W = PID_controllerH(-10.0, position.h, 1) / 4;
//                    putar(0, 0, W);
//                    if (fabs(-10.0 - position.h) < 5.0)
//                    {
//                        searchMode = 2;
//                    }
//                    break;
//                case 2:
//                    W = PID_controllerH(-170.0, position.h, 1) / 4;
//                    putar(0, 0, W);
//                    if (fabs(-170.0 - position.h) < 5.0)
//                    {
//                        searchMode = 1;
//                    }
//                    break;
//            }
//        }
    }
}

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
