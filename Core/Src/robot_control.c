#include <robot_control.h>

extern TIM_HandleTypeDef htim2;

static double prevAngleBall = 0.0;
double kpAngle = 0.0;
extern int camera[5];

int map(int st1, int fn1, int st2, int fn2, int value)
{
    return (1.0*(value-st1))/((fn1-st1)*1.0) * (fn2-st2)+st2;
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

void servo_write(int angle)
{
	int i = map(0, 180, 25, 125, angle);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, i);
}

void nyoba_gerak(EKF setpoint, uint8_t pidMode)
{
	EKF position = extendedKalmanFilter();

	double Vx = 0.0;
	double Vy = 0.0;
	double W = 0.0;

    uint32_t timer = HAL_GetTick();
    uint32_t lastTime = 0;
    static bool motorhState = false;
    static bool motorxState = false;

	Vx = PID_controller(setpoint.x, position.x, pidMode);
	Vy = PID_controller(setpoint.y, position.y, pidMode);
	W = PID_controllerH(setpoint.h, position.h);

	if(position.y >= 2000 && position.y <= 3000)
	{
		if(fabs(position.h) > 1.5)
		{
	    	if(timer - lastTime >= 1000)
	    	{
	        	lastTime = timer;
	        	if(motorhState)
	        	{
	        		Inverse_Kinematics(0, 0, W);
	            	motorhState = false;
	        	}
	        	else
	        	{
	        		Inverse_Kinematics(0, 0, 0);
	            	motorhState = true;
	        	}
	    	}
		}
		else if(fabs(position.x) > 50)
		{
	    	if(timer - lastTime >= 1000)
	    	{
	        	lastTime = timer;
	        	if(motorxState)
	        	{
	        		Inverse_Kinematics(Vx, 0, 0);
	            	motorxState = false;
	        	}
	        	else
	        	{
	        		Inverse_Kinematics(0, 0, 0);
	            	motorxState = true;
	        	}
	    	}
		}
	}
	else
	{
		Inverse_Kinematics(Vx, Vy, W);
	}
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
	double Vx = 0.0;
	double Vy = 0.0;
	double W = 0.0;

	Vx = PID_controller(setpoint.x, position.x, pidMode);
	Vy = PID_controller(setpoint.y, position.y, pidMode);
	W = PID_controllerH(setpoint.h, position.h);

	Inverse_Kinematics(Vx, Vy, W);
}

void PID_nyoba(EKF setpoint, uint8_t pidMode)
{
	EKF position = extendedKalmanFilter();
	double Vx = 0.0;
	double Vy = 0.0;
	double W = 0.0;

    uint32_t timer = HAL_GetTick();
    uint32_t lastTime = 0;
    static bool motorhState = false;
    static bool motorxState = false;

	Vx = PID_controller(setpoint.x, position.x, pidMode);
	Vy = PID_controller(setpoint.y, position.y, pidMode);
	W = PID_controllerH(setpoint.h, position.h);

	double Vx_filtered = 0.0, Vy_filtered = 0.0, W_filtered = 0.0;
	Vx_filtered = lowPassFilter(Vx, Vx_filtered, 0.3);
	Vy_filtered = lowPassFilter(Vy, Vy_filtered, 0.3);
	W_filtered = lowPassFilter(W, W_filtered, 0.3);

	if(fabs(position.h) > 1.5)
	{
    	if(timer - lastTime >= 1000)
    	{
        	lastTime = timer;
        	if(motorhState)
        	{
        		Inverse_Kinematics(0, 0, W_filtered);
            	motorhState = false;
        	}
        	else
        	{
        		Inverse_Kinematics(0, 0, 0);
            	motorhState = true;
        	}
    	}
	}
	else if(fabs(position.x) > 40)
	{
    	if(timer - lastTime >= 1000)
    	{
        	lastTime = timer;
        	if(motorxState)
        	{
        		Inverse_Kinematics(Vx_filtered, 0, 0);
            	motorxState = false;
        	}
        	else
        	{
        		Inverse_Kinematics(0, 0, 0);
            	motorxState = true;
        	}
    	}
	}
	else
	{
		Inverse_Kinematics(0, Vy_filtered, 0);
	}
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
	if(fabs(setpoint[index].x - currentPosition.x) < tolerance && fabs(setpoint[index].y - currentPosition.y) < tolerance && fabs(setpoint[index].h - currentPosition.h) < 2)
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

    double lookaheadX = position.x + lookaheadDistance * cos(position.h);
    double lookaheadY = position.y + lookaheadDistance * sin(position.h);
    double lookaheaderrorX = setpoint.x - lookaheadX;
    double lookaheaderrorY = setpoint.y - lookaheadY;

    double targetOrientation = atan2(lookaheadY - position.y, lookaheadX - position.x);

    double orientationError = targetOrientation - position.h;

    Vx += PID_controller(lookaheaderrorX, 0.0, pidMode);
    Vy += PID_controller(lookaheaderrorY, 0.0, pidMode);
    W += PID_controllerH(orientationError, 0.0);

    Inverse_Kinematics(Vx, Vy, W);
}

void focusToTheBall()
{
    int ballDistance = camera[0] * 10; // convert to mm
    int ballAngle = camera[1];
    int ballExistence = camera[2];

    double kp = 0.05;
    int gndtoCam = 358; // in mm

    int yBALL = cos(ballAngle * M_PI / 180.0) * ballDistance;
    double focus = atan2(gndtoCam, yBALL) * 180.0 / M_PI;
    prevAngleBall = focus;
    int focusMapping = map(0, 90, 180, 90, prevAngleBall);
    kpAngle = focusMapping + kp * (focusMapping - kpAngle);
    servo_write(kpAngle);
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

    double Vx = PID_controller((double)xBALL, 0.0, 1);
    double Vy = PID_controller((double)yBALL, 0.0, 1);
    double W = PID_controllerH((double)ballAngle, 0.0);

    if (ballExistence != 0)
    {
    	lastxBall = xBALL;
    	lastyBall = yBALL;
    	lastBallAngle = ballAngle;

    	focusToTheBall();
    	setMotorSpeed(1, -500);
        if(ballAngle < 20 && ballAngle > 0)
        {
        	Inverse_Kinematics(0, 1600, 0);
//        	start(0, 1600, 0, 1);
        }
        else
        {
        	if(timer - lastTime >= 1000)
        	{
            	lastTime = timer;
            	if(motorCorrWithBall)
            	{
                	putar(0, 0, W);
                	setMotorSpeed(1, 0);
                	motorCorrWithBall = false;
            	}
            	else
            	{
                	putar(0, 0, 0);
                	setMotorSpeed(1, 0);
                	motorCorrWithBall = true;
            	}
        	}
        }
    }
    else
    {
        Vx = PID_controller(lastxBall, 0.0, 1);
        Vy = PID_controller(lastyBall, 0.0, 1);
        W = PID_controllerH(lastBallAngle, 0.0);

        servo_write(145);
        if(!(lastBallAngle < 20 && lastBallAngle > 0))
        {
        	if(timer - lastTime >= 1000)
        	{
            	lastTime = timer;
            	if(motorCorrWithoutBall)
            	{
                	setMotorSpeed(1, 0);
                	putar(0, 0, W);
                	motorCorrWithoutBall = false;
            	}
            	else
            	{
                	setMotorSpeed(1, 0);
                	putar(0, 0, 0);
                	motorCorrWithoutBall = true;
            	}
        	}
        }
        else if(lastBallAngle != 0 && lastBallAngle > 0 && lastBallAngle < 20)
        {
        	setMotorSpeed(1, -500);
        	Inverse_Kinematics(0, 1600, 0);
        }
        else
        {
        	if(timer - lastTime >= 1000)
        	{
        		lastTime = timer;
        		if(motorState)
        		{
                	setMotorSpeed(1, 0);
                	putar(0, 0, 85);
                	motorState = false;
        		}
        		else
        		{
                	setMotorSpeed(1, 0);
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
    int xSILO = sin(siloAngle * M_PI / 180) * siloDistance;
    int ySILO = cos(siloAngle * M_PI / 180) * siloDistance;
    static bool motorState = false;
    static bool motorCorrWithoutSilo = false;
    static bool motorCorrWithSilo = false;

    uint32_t timer = HAL_GetTick();
    uint32_t lastTime = 0;

    double Vx = PID_controller((double)xSILO, 0.0, 1);
    double Vy = PID_controller((double)ySILO, 0.0, 1);
    double W = PID_controllerH((double)siloAngle, 0.0);
    servo_write(160);

    if (siloAngle != 0 && siloDistance != 0)
    {
    	lastxSilo = xSILO;
    	lastySilo = ySILO;
    	lastSiloAngle = siloAngle;

    	W = PID_controllerH(siloAngle, 0.0);
        if(abs(siloAngle) < 7 && abs(xSILO) < 10)
        {
        	Inverse_Kinematics(0, 1600, 0);
        	if(siloDistance < 50)
        	{
        		Inverse_Kinematics(0, 0, 0);
            	setMotorSpeed(2, -1000);
            	setMotorSpeed(3, -1000);
        	}
        }
        else
        {
        	if(timer - lastTime >= 1000)
        	{
            	lastTime = timer;
            	if(motorCorrWithSilo)
            	{
            		Inverse_Kinematics(Vx, 0, W);
                	setMotorSpeed(2, 0);
                	setMotorSpeed(3, 0);
                	motorCorrWithSilo = false;
            	}
            	else
            	{
                	putar(0, 0, 0);
                	setMotorSpeed(2, 0);
                	setMotorSpeed(3, 0);
                	motorCorrWithSilo = true;
            	}
        	}
        }
    }
    else
    {
        Vx = PID_controller(lastxSilo < 0 ? lastxSilo++ : lastxSilo--, 0.0, 1);
        Vy = PID_controller(lastySilo < 0 ? lastySilo++ : lastySilo--, 0.0, 1);
        W = PID_controllerH(lastSiloAngle < 0 ? lastSiloAngle++ : lastSiloAngle--, 0.0);

        if(abs(lastSiloAngle) >= 7)
        {
        	if(timer - lastTime >= 1000)
        	{
            	lastTime = timer;
            	if(motorCorrWithoutSilo)
            	{
                	setMotorSpeed(2, 0);
                	setMotorSpeed(3, 0);
            		Inverse_Kinematics(Vx, 0, W);
                	motorCorrWithoutSilo = false;
            	}
            	else
            	{
                	setMotorSpeed(2, 0);
                	setMotorSpeed(3, 0);
                	putar(0, 0, 0);
                	motorCorrWithoutSilo = true;
            	}
        	}
        }
        else if(lastSiloAngle != 0 && abs(lastSiloAngle) < 7 && abs(lastxSilo) < 10)
        {
        	start(0, 1600, 0, 1);
        	if(lastySilo < 50)
        	{
        		Inverse_Kinematics(0, 0, 0);
            	setMotorSpeed(2, -1000);
            	setMotorSpeed(3, -1000);
        	}
        }
        else
        {
        	if(timer - lastTime >= 1000)
        	{
        		lastTime = timer;
        		if(motorState)
        		{
                	setMotorSpeed(2, 0);
                	setMotorSpeed(3, 0);
                	putar(0, 0, 85);
                	motorState = false;
        		}
        		else
        		{
                	setMotorSpeed(2, 0);
                	setMotorSpeed(3, 0);
                	putar(0, 0, 0);
                	motorState = true;
        		}
        	}
        }
    }
}
