#include <robot_control.h>

extern int camera[5];

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

void PID_Tuning(EKF setpoint, double KP, double KI, double KD)
{
	EKF position = extendedKalmanFilter();

	double Vx = PID_for_tuning(setpoint.x, position.x, KP, KI, KD);
	double Vy = PID_for_tuning(setpoint.y, position.y, KP, KI, KD);
	double W = PID_for_tuningH(setpoint.h, position.h, KP, KI, KD);
	Inverse_Kinematics(Vx, Vy, W);
}

void PID_movetoCoordinate(robotPosition setpoint, uint8_t type, uint8_t pidMode, double smoothingFactor)
{
    robotPosition currentPosition = odometry();
    double Vx = 0.0;
    double Vy = 0.0;
    double W  = 0.0;

    switch (type) {
        case global:
            Vx = PID_controller(setpoint.x_global, currentPosition.x_global, pidMode);
            Vy = PID_controller(setpoint.y_global, currentPosition.y_global, pidMode);
            W = PID_controllerH(setpoint.h, currentPosition.h);
            break;
        case local:
            Vx = PID_controller(setpoint.x_local, currentPosition.x_local, pidMode);
            Vy = PID_controller(setpoint.y_local, currentPosition.y_local, pidMode);
            W = PID_controllerH(setpoint.h, currentPosition.h);
            break;
        case in_global:
            Vx = PID_controller(setpoint.x_in_global, currentPosition.x_in_global, pidMode);
            Vy = PID_controller(setpoint.y_in_global, currentPosition.y_in_global, pidMode);
            W = PID_controllerH(setpoint.h, currentPosition.h);
            break;
        case in_local:
            Vx = PID_controller(setpoint.x_in_local, currentPosition.x_in_local, pidMode);
            Vy = PID_controller(setpoint.y_in_local, currentPosition.y_in_local, pidMode);
            W = PID_controllerH(setpoint.h, currentPosition.h);
            break;
    }

    smoothVelocity(&Vx, &Vy, &W, smoothingFactor);
    Inverse_Kinematics(Vx, Vy, W);
}

void PID_Kalman(EKF setpoint, uint8_t pidMode)
{
	EKF position = extendedKalmanFilter();
	double Vx = 0;
	double Vy = 0;
	double W = 0;

//	int leftRange = 0;
//	int frontRange = 0;
//	int rightRange = 0;
//
//	if(frontRange < 30)
//	{
//		if(leftRange > rightRange)
//		{
//			Vx = -PID_controller(30.0, rightRange, 5);
//		}
//		else
//		{
//			Vx = PID_controller(30.0, leftRange, 5);
//		}
//	}
//	else if(leftRange < 30)
//	{
//		Vx = PID_controller(30.0, leftRange, 5);
//	}
//	else if(rightRange < 30)
//	{
//		Vx = -PID_controller(30.0, rightRange, 5);
//	}
//	else
//	{
//		Vx = PID_controller(setpoint.x, position.x, pidMode);
//		Vy = PID_controller(setpoint.y, position.y, pidMode);
//		W = PID_controllerH(setpoint.h, position.h);
//	}

	Vx = PID_controller(setpoint.x, position.x, pidMode);
	Vy = PID_controller(setpoint.y, position.y, pidMode);
	W = PID_controllerH(setpoint.h, position.h);

    smoothVelocity(&Vx, &Vy, &W, 0.5);
	Inverse_Kinematics(Vx, Vy, W);
}

void PID_KFtocoordinate(EKF setpoint, uint8_t pidMode)
{
    EKF currentPosition = extendedKalmanFilter();

    double error_x = setpoint.x - currentPosition.x;
    double error_y = setpoint.y - currentPosition.y;

    double setpoint_angle = atan2(error_y, error_x) * 180 / M_PI;
    double heading_error = setpoint_angle - currentPosition.h;

    if (heading_error > 180)		heading_error -= 360;
    else if (heading_error < -180)	heading_error += 360;

    double W = PID_controllerH(heading_error, currentPosition.h);

    double distance_to_setpoint = hypot(error_x, error_y);

    setpoint.h *= M_PI / 180;

    double Vx = distance_to_setpoint * cos(setpoint.h);
    double Vy = distance_to_setpoint * sin(setpoint.h);

    Vx += PID_controller(setpoint.x, currentPosition.x, pidMode);
    Vy += PID_controller(setpoint.y, currentPosition.y, pidMode);

    Inverse_Kinematics(Vx, Vy, W);
}

void PID_steptoCoordinate(EKF *setpoint, uint8_t pidMode, double tolerance, uint16_t amount)
{
	EKF currentPosition = extendedKalmanFilter();
	static uint8_t index = 0;

	double Vx = PID_controller(setpoint[index].x, currentPosition.x, pidMode);
	double Vy = PID_controller(setpoint[index].y, currentPosition.y, pidMode);
	double W = PID_controllerH(setpoint[index].h, currentPosition.h);
	if(fabs(setpoint[index].x - currentPosition.x) < tolerance && fabs(setpoint[index].y - currentPosition.y) < tolerance && fabs(setpoint[index].h - currentPosition.h) < 0.5)
	{
		Inverse_Kinematics(0, 0, 0);
		index++;
	}
	else if(index > amount)
	{
		Inverse_Kinematics(0, 0, 0);
	}
	else
	{
		Inverse_Kinematics(Vx, Vy, W);
	}
}

void PID_gotoX(double setpoint_x, uint8_t pidMode)
{
	EKF currentPosition = extendedKalmanFilter();

    double Vx = PID_controller(setpoint_x, currentPosition.x, pidMode);
    Inverse_Kinematics(Vx, 0, 0);
}

void PID_gotoY(double setpoint_y, uint8_t pidMode)
{
	EKF currentPosition = extendedKalmanFilter();

    double Vy = PID_controller(setpoint_y, currentPosition.y, pidMode);
    Inverse_Kinematics(0, Vy, 0);
}

void PID_setDegree(double setpoint_h)
{
	EKF currentPosition = extendedKalmanFilter();

    double W = PID_controllerH(setpoint_h, currentPosition.h);
    Inverse_Kinematics(0, 0, W);
}

void PID_coba(EKF setpoint, uint8_t pidMode, double lookaheadDistance)
{
	EKF position = extendedKalmanFilter();

	double Vx = PID_controller(setpoint.x, position.x, pidMode);
	double Vy = PID_controller(setpoint.y, position.y, pidMode);
	double W = PID_controllerH(setpoint.h, position.h);

    // Calculate lookahead point
    double lookaheadX = position.x + lookaheadDistance * cos(position.h);
    double lookaheadY = position.y + lookaheadDistance * sin(position.h);
    double lookaheaderrorX = setpoint.x - lookaheadX;
    double lookaheaderrorY = setpoint.y - lookaheadY;

    // Calculate orientation towards lookahead point
    double targetOrientation = atan2(lookaheadY - position.y, lookaheadX - position.x);

    // Calculate error between current orientation and target orientation
    double orientationError = targetOrientation - position.h;

    // Adjust velocities based on orientation error
    Vx += PID_controller(lookaheaderrorX, 0.0, pidMode);
    Vy += PID_controller(lookaheaderrorY, 0.0, pidMode);
    W += PID_controllerH(orientationError, 0.0); // Adjusting for orientation error

    // Apply inverse kinematics
    Inverse_Kinematics(Vx, Vy, W);
}

void findtheBall()
{
    /*
     * Camera data:
     * camera[0]: ball distance (in cm)
     * camera[1]: ball angle (in degree)
     * camera[2]: ball existence (number of balls)
     * camera[3]: silo distance (in cm)
     * camera[4]: silo angle (in degree)
     */

    static double lastBallAngle = 0, lastxBall = 0, lastyBall = 0;

    int ballDistance = camera[0] * 10; // convert to mm
    int ballAngle = camera[1];
    int ballExistence = camera[2];
    int xBALL = sin(ballAngle * M_PI / 180) * ballDistance;
    int yBALL = cos(ballAngle * M_PI / 180) * ballDistance;
    static bool motorState = false;
    static bool motorCorrWithoutBall = false;
    static bool motorCorrWithBall = false;

    uint32_t timer = HAL_GetTick();
    uint32_t lastTime = 0;

    double Vx = PID_controller(xBALL, 0, 1);
    double Vy = PID_controller(yBALL, 0, 1);
    double W = PID_controllerH(ballAngle, 0);

    if (ballExistence != 0)
    {
    	lastxBall = xBALL;
    	lastyBall = yBALL;
    	lastBallAngle = ballAngle;

    	W = PID_controllerH(ballAngle, 0.0);
    	setMotorSpeed(5, -500);
        if(abs(ballAngle) < 20)
        {
        	start(0, 1600, 0, 4);
        }
        else
        {
        	if(timer - lastTime >= 1000)
        	{
            	lastTime = timer;
            	if(motorCorrWithBall)
            	{
                	putar(0, 0, W);
                	setMotorSpeed(5, 0);
                	motorCorrWithBall = false;
            	}
            	else
            	{
                	putar(0, 0, 0);
                	setMotorSpeed(5, 0);
                	motorCorrWithBall = true;
            	}
        	}
        }
    }
    else
    {
        Vx = PID_controller(lastxBall, 0, 1);
        Vy = PID_controller(lastyBall, 0, 1);
        W = PID_controllerH(lastBallAngle, 0.0);

        if(abs(lastBallAngle) >= 10)
        {
        	if(timer - lastTime >= 1000)
        	{
            	lastTime = timer;
            	if(motorCorrWithoutBall)
            	{
                	setMotorSpeed(5, 0);
                	putar(0, 0, W);
                	motorCorrWithoutBall = false;
            	}
            	else
            	{
                	setMotorSpeed(5, 0);
                	putar(0, 0, 0);
                	motorCorrWithoutBall = true;
            	}
        	}
        }
        else if(lastBallAngle != 0 && abs(lastBallAngle) < 10)
        {
        	setMotorSpeed(5, -500);
        	start(0, 1600, 0, 4);
        }
        else
        {
        	if(timer - lastTime >= 1000)
        	{
        		lastTime = timer;
        		if(motorState)
        		{
                	setMotorSpeed(5, 0);
                	putar(0, 0, 75);
                	motorState = false;
        		}
        		else
        		{
                	setMotorSpeed(5, 0);
                	putar(0, 0, 0);
                	motorState = true;
        		}
        	}
        }
    }
}

void findSilo()
{
    /*
     * Camera data:
     * camera[0]: ball distance (in cm)
     * camera[1]: ball angle (in degree)
     * camera[2]: ball existence (number of balls)
     * camera[3]: silo distance (in cm)
     * camera[4]: silo angle (in degree)
     */

    static double lastSiloAngle = 0, lastxSilo = 0, lastySilo = 0;

    int siloDistance = camera[3] * 10; // convert to mm
    int siloAngle = camera[4];
//    int siloExistence = camera[2];
    int xSILO = sin(siloAngle * M_PI / 180) * siloDistance;
    int ySILO = cos(siloAngle * M_PI / 180) * siloDistance;
    static bool motorState = false;
    static bool motorCorrWithoutSilo = false;
    static bool motorCorrWithSilo = false;

    uint32_t timer = HAL_GetTick();
    uint32_t lastTime = 0;

    double Vx = PID_controller(xSILO, 0, 1);
    double Vy = PID_controller(ySILO, 0, 1);
    double W = PID_controllerH(siloAngle, 0);

    if (siloAngle != 0 && siloDistance != 0)
    {
    	lastxSilo = xSILO;
    	lastySilo = ySILO;
    	lastSiloAngle = siloAngle;

    	W = PID_controllerH(siloAngle, 0.0);
    	setMotorSpeed(5, -500);
        if(abs(siloAngle) < 20)
        {
        	start(0, 1600, 0, 4);
        }
        else
        {
        	if(timer - lastTime >= 1000)
        	{
            	lastTime = timer;
            	if(motorCorrWithSilo)
            	{
                	putar(0, 0, W);
                	setMotorSpeed(5, 0);
                	motorCorrWithSilo = false;
            	}
            	else
            	{
                	putar(0, 0, 0);
                	setMotorSpeed(5, 0);
                	motorCorrWithSilo = true;
            	}
        	}
        }
    }
    else
    {
        Vx = PID_controller(lastxSilo, 0, 1);
        Vy = PID_controller(lastySilo, 0, 1);
        W = PID_controllerH(lastSiloAngle, 0.0);

        if(abs(lastSiloAngle) >= 10)
        {
        	if(timer - lastTime >= 1000)
        	{
            	lastTime = timer;
            	if(motorCorrWithoutSilo)
            	{
                	setMotorSpeed(5, 0);
                	putar(0, 0, W);
                	motorCorrWithoutSilo = false;
            	}
            	else
            	{
                	setMotorSpeed(5, 0);
                	putar(0, 0, 0);
                	motorCorrWithoutSilo = true;
            	}
        	}
        }
        else if(lastSiloAngle != 0 && abs(lastSiloAngle) < 10)
        {
        	setMotorSpeed(5, -500);
        	start(0, 1600, 0, 4);
        }
        else
        {
        	if(timer - lastTime >= 1000)
        	{
        		lastTime = timer;
        		if(motorState)
        		{
                	setMotorSpeed(5, 0);
                	putar(0, 0, 75);
                	motorState = false;
        		}
        		else
        		{
                	setMotorSpeed(5, 0);
                	putar(0, 0, 0);
                	motorState = true;
        		}
        	}
        }
    }
}
