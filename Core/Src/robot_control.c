#include <robot_control.h>

extern TIM_HandleTypeDef htim2;

static double prevAngleBall = 0.0;
double kpAngle = 0.0;
extern double sensorData[2];
extern int camera[13];
extern int sensorMEGA[4];
char buffCAM[10];
static uint8_t findBallMode = 1;

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
	W = PID_controllerH(setpoint.h, position.h, 1);

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
            W = PID_controllerH(setpoint.h, currentPosition.h, 1);
            break;
        case local:
            Vx = PID_controller(setpoint.x_local, currentPosition.x_local, pidMode);
            Vy = PID_controller(setpoint.y_local, currentPosition.y_local, pidMode);
            W = PID_controllerH(setpoint.h, currentPosition.h, 1);
            break;
        case in_global:
            Vx = PID_controller(setpoint.x_in_global, currentPosition.x_in_global, pidMode);
            Vy = PID_controller(setpoint.y_in_global, currentPosition.y_in_global, pidMode);
            W = PID_controllerH(setpoint.h, currentPosition.h, 1);
            break;
        case in_local:
            Vx = PID_controller(setpoint.x_in_local, currentPosition.x_in_local, pidMode);
            Vy = PID_controller(setpoint.y_in_local, currentPosition.y_in_local, pidMode);
            W = PID_controllerH(setpoint.h, currentPosition.h, 1);
            break;
    }

    smoothVelocity(&Vx, &Vy, &W, smoothingFactor);
    Inverse_Kinematics(Vx, Vy, W);
}

void PID_Kalman(EKF setpoint, uint8_t pidMode)
{
    EKF position = extendedKalmanFilter();
//    const int FRONT_THRESHOLD = 25;
//    const int LEFT_THRESHOLD = 25;
//    int frontDistance = sensorMEGA[0];
//    int leftDistance = sensorMEGA[1];

    double Vx = 0.0;
    double Vy = 0.0;
    double W = 0.0;

    Vx = PID_controller(setpoint.x, position.x, pidMode);
    Vy = PID_controller(setpoint.y, position.y, pidMode);
    W = PID_controllerH(setpoint.h, position.h, 3);

//    if (frontDistance < FRONT_THRESHOLD && frontDistance > 0) {Vy = 0.0;}
//    else if (leftDistance < LEFT_THRESHOLD && leftDistance > 0) {Vx = 0.0;}
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
	W = PID_controllerH(setpoint.h, position.h, 1);

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

    double W = PID_controllerH(heading_error, currentPosition.h, 1);

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
	uint8_t index = 0;

	double Vx = PID_controller(setpoint[index].x, currentPosition.x, pidMode);
	double Vy = PID_controller(setpoint[index].y, currentPosition.y, pidMode);
	double W = PID_controllerH(setpoint[index].h, currentPosition.h, 1);
	if(fabs(setpoint[index].x - currentPosition.x) < tolerance && fabs(setpoint[index].y - currentPosition.y) < tolerance && fabs(setpoint[index].h - currentPosition.h) < 1)
	{
		Inverse_Kinematics(0, 0, 0);
		index++;
	}
	else if(index > amount)
	{
		Inverse_Kinematics(0, 0, 0);
		index = amount;
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

    double W = PID_controllerH(setpoint_h, currentPosition.h, 3);
    putar(0, 0, W);
//    Inverse_Kinematics(0, 0, W);
}

void PID_coba(EKF setpoint, uint8_t pidMode, double lookaheadDistance)
{
	EKF position = extendedKalmanFilter();

	double Vx = PID_controller(setpoint.x, position.x, pidMode);
	double Vy = PID_controller(setpoint.y, position.y, pidMode);
	double W = PID_controllerH(setpoint.h, position.h, 1);

    double lookaheadX = position.x + lookaheadDistance * cos(position.h);
    double lookaheadY = position.y + lookaheadDistance * sin(position.h);
    double lookaheaderrorX = setpoint.x - lookaheadX;
    double lookaheaderrorY = setpoint.y - lookaheadY;

    double targetOrientation = atan2(lookaheadY - position.y, lookaheadX - position.x);

    double orientationError = targetOrientation - position.h;

    Vx += PID_controller(lookaheaderrorX, 0.0, pidMode);
    Vy += PID_controller(lookaheaderrorY, 0.0, pidMode);
    W += PID_controllerH(orientationError, 0.0, 1);

    Inverse_Kinematics(Vx, Vy, W);
}

void focusToTheBall()
{
    int ballDistance = camera[0] * 10; // convert to mm
    int ballAngle = camera[1];
    int ballExistence = camera[2];

    double kp = 0.05;
    int gndtoCam = 362; // in mm

    if(ballExistence != 0)
    {
        int yBALL = cos(ballAngle * M_PI / 180.0) * ballDistance;
        double focus = atan2(gndtoCam, yBALL) * 180.0 / M_PI;
        prevAngleBall = focus;
        int focusMapping = map(0, 90, 70, 0, prevAngleBall);
        kpAngle = focusMapping + kp * (focusMapping - kpAngle);
        servo_write(kpAngle);
    }
    else
    {
    	servo_write(55);
    }
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

	EKF position = extendedKalmanFilter();

    static uint32_t lastTimeBallSeen = 0;
    static uint32_t lastTime = 0;

    uint32_t timer = HAL_GetTick();

    int ballDistance = camera[0] * 10; // convert to mm
    int ballAngle = camera[1];
    int ballExistence = camera[2];

    double Vx = 0.0;
    double Vy = 0.0;
    double W = 0.0;
	focusToTheBall();

	if(ballExistence != 0)
	{
		lastTimeBallSeen = timer;
		lastTime = timer;

		Vy = 1200;
		Vx = Vy * tan(ballAngle * M_PI / 180);
		W = PID_controllerH(ballAngle, 0.0, 1);

		// PALING AMAN
		if (ballDistance < 1500 && fabs(ballAngle) > 4)
		{
			baru(0, 0, W);
			HAL_Delay(150);
			baru(0, 0, 0);
			HAL_Delay(100);
		}
		else if (fabs(ballAngle) <= 4)
		{
			baru(0, Vy, 0);
		}
		else
		{
			baru(Vx, Vy, W);
			HAL_Delay(25);
		}
//		if(ballDistance < 500)
//		{
//			servo_write(35);
//		}
		setMotorSpeed(1, -1000);
	}
	else if (timer - lastTimeBallSeen <= 700)
	{
		lastTime = timer;
		start(0, 1200, 0, 1);
		setMotorSpeed(1, -1000);
	}
	else
	{
		setMotorSpeed(1, 0);
//		static bool motorState = false;
		switch(findBallMode)
		{
		case 1:
			PID_setDegree(-10.0);
			HAL_Delay(200);
			Inverse_Kinematics(0, 0, 0);
			HAL_Delay(100);
			if(position.h > -20.0)
			{
				findBallMode = 2;
			}
			break;
		case 2:
			PID_setDegree(-170.0);
			HAL_Delay(200);
			Inverse_Kinematics(0, 0, 0);
			HAL_Delay(100);
			if(position.h < -160.0)
			{
				findBallMode = 1;
			}
			break;
		}

		// AMANN //
//		if (timer - lastTime >= 100)
//		{
//			lastTime = timer;
//			motorState = !motorState;
//			putar(0, 0, motorState ? 65 : 0);
//		}
	}
}

void findSilo()
{
    /*
     * Silo distance from the left of the robot
     * Silo counted from left
     * 1: 21
     * 2: 96
     * 3: 169
     * 4: 237
     * 5: 307
     */

    static uint32_t lastTimeSiloSeen = 0;
    static uint32_t lastTime = 0;

    EKF position = extendedKalmanFilter();
    uint32_t timer = HAL_GetTick();

    // Read distances and angles for the five silos
    int siloDistances[5] = {camera[3] * 10, camera[5] * 10, camera[7] * 10, camera[9] * 10, camera[11] * 10}; // convert to mm
    int siloAngles[5] = {camera[4], camera[6], camera[8], camera[10], camera[12]};

    int xSILO = 0;
    int ySILO = 0;
    double siloAngle = 0.0;
    int siloDistance = 0;
    int closestSiloIndex = -1;

    // Find the closest detected silo by angle
    for (int i = 0; i < 5; i++)
    {
        if (siloDistances[i] != 99990 && (closestSiloIndex == -1 || abs(siloAngles[i]) < abs(siloAngles[closestSiloIndex])))
        {
            closestSiloIndex = i;
        }
    }

    if (closestSiloIndex != -1)
    {
        siloAngle = siloAngles[closestSiloIndex];
        siloDistance = siloDistances[closestSiloIndex];
        xSILO = sin(siloAngle * M_PI / 180) * siloDistance;
        ySILO = cos(siloAngle * M_PI / 180) * siloDistance;
    }

//    int frontDistance = sensorMEGA[0];
//    int leftDistance = sensorMEGA[1];

    double Vx = 0.0;
    double Vy = 0.0;
    double W = 0.0;
    servo_write(80);

    if (closestSiloIndex != -1 && siloDistance != 99990)
    {
        lastTimeSiloSeen = timer;
        lastTime = timer;

        double tuning = 1.0;
        if(siloAngle <= 20) {tuning = 2.0;}
        Vy = 900;
        Vx = Vy * tan(siloAngle * M_PI / 180) * tuning;
    	W = PID_controllerH(90.0, position.h, 2);

    	if(fabs(90.0 - position.h) >= 10)		{Vx = 0; Vy = 0;}
    	else if(fabs(siloAngle) >= 2)			{Vy = 0; W = 0;}
    	else									{W = 0;}
    	baru(Vx, Vy, W);
    }
	else if (timer - lastTimeSiloSeen <= 3000)
	{
		lastTimeSiloSeen = timer;
		start(0, 800, 0, 1);
	}
	else
	{
        setMotorSpeed(1, 0);
        setMotorSpeed(2, 0);
        static bool motorState = false;
        if (timer - lastTime >= 500)
        {
        	lastTime = timer;
            motorState = !motorState;
            Inverse_Kinematics(motorState ? 800 : -800, 0, 0);
        }
	}
}

//void findtheBall()
//{
//    /*
//     * Camera data:
//     * camera[0]: ball distance (in cm)
//     * camera[1]: ball angle (in degree)
//     * camera[2]: ball existence (number of balls)
//     * camera[3]: silo distance (in cm)
//     * camera[4]: silo angle (in degree)
//     */
//
//    static BallState currentState = STATE_FIND_BALL;
//    static uint32_t lastTimeBallSeen = 0;
//    static double lastBallAngle = 0.0, lastxBall = 0.0, lastyBall = 0.0;
//
//    uint32_t timer = HAL_GetTick();
//
//    int ballDistance = camera[0] * 10; // convert to mm
//    int ballAngle = camera[1];
//    int ballExistence = camera[2];
//    int xBALL = sin(ballAngle * M_PI / 180) * ballDistance;
//    int yBALL = cos(ballAngle * M_PI / 180) * ballDistance;
//
//    double Vx = 0.0;
//    double Vy = 0.0;
//    double W = 0.0;
//	focusToTheBall();
//
//    W = PID_controllerH((double)ballAngle, 11.0, 1);
//
//    switch (currentState)
//    {
//        case STATE_FIND_BALL:
//            setMotorSpeed(1, 0);
//
//            if (ballExistence != 0)
//            {
//                // Ball detected
//                lastxBall = xBALL;
//                lastyBall = yBALL;
//                lastBallAngle = ballAngle;
//                lastTimeBallSeen = timer;
////                HAL_Delay(10);
//                currentState = STATE_MOVE_TOWARDS_BALL;
//            }
//            // Enter search mode if the ball is not detected for more than 2 seconds
//            else
//            {
//                if (timer - lastTimeBallSeen >= 1000)
//                {
//                    currentState = STATE_SEARCH_BALL;
//                }
//            }
//            break;
//
//        case STATE_MOVE_TOWARDS_BALL:
//			if (ballExistence != 0)
//			{
//				// Update last known ball position
//				lastxBall = xBALL;
//				lastyBall = yBALL;
//				lastBallAngle = ballAngle;
//				lastTimeBallSeen = timer;
//
//				// Focus on the ball and move towards it
//				focusToTheBall();
//                Vy = 1000;
//                Vx = Vy * tan(ballAngle * M_PI / 180);
//                W = PID_controllerH(ballAngle, 11.0, 1);
//                if (yBALL < 1500 && (ballAngle < 13 && ballAngle > 8))
//                {
//                	Vx = 0;
//                }
//                baru(Vx, Vy, W);
////                HAL_Delay(10);
//
////					float disTime;
////					int vel = 1000; int velx,vely,velw;
////
////					if (ballDistance >= 800) {disTime = 1.1;}
////					else if (ballDistance >= 600 && ballDistance < 800) {disTime = 0.9;}
////					else if (ballDistance >= 400 && ballDistance < 600) {disTime = 0.76;}
////					else if (ballDistance >= 200 && ballDistance < 400) {disTime = 0.65;}
////					else {disTime = 0.5;}
////
////					Inverse_Kinematics(Vx*disTime, Vy*disTime, W*disTime);
////
////					if (ballAngle >= 40) {velw=18;}
////					else if (ballAngle >= 30 && ballAngle < 40) {velw=13;}
////					else if (ballAngle >=20 && ballAngle < 30) {velw=7;}
////					else if (ballAngle > 10 && ballAngle < 20) {velw=0;}
////					else if (ballAngle > 0 && ballAngle <= 10) {velw=-2;}
////
////					if (ballAngle <= -40) {velw=-18;}
////					else if (ballAngle <= -30 && ballAngle > -40) {velw=-13;}
////					else if (ballAngle <= -20 && ballAngle > -30) {velw=-7;}
////					else if (ballAngle <= -10 && ballAngle > -20) {velw=-4;}
////					else if (ballAngle < 0 && ballAngle > -10) {velw=-3;}
////
////					vely = vel;
////					vely *= disTime;
////					velx = vely * tan(ballAngle * M_PI / 180);
////
////					Inverse_Kinematics(velx, vely, velw);
//
//				setMotorSpeed(1, -1500);
//			}
//			else if (timer - lastTimeBallSeen <= 100)
//			{
//				start(0, 900, 0, 1);
//				setMotorSpeed(1, -1500);
//			}
//			else
//			{
//				currentState = STATE_FIND_BALL;
//			}
//			break;
//
//        case STATE_SEARCH_BALL:
//            // Search for the ball by rotating
//            setMotorSpeed(1, 0);
//            static bool motorState = false;
//            if (timer - lastTimeBallSeen >= 100)
//            {
//                lastTimeBallSeen = timer;
//                motorState = !motorState;
//                putar(0, 0, motorState ? 95 : 0);
//            }
//            // If the ball is detected during search, transition to moving towards it
//            if (ballExistence != 0)
//            {
//                currentState = STATE_FIND_BALL;
//            }
//            break;
//    }
//}

//void findSilo()
//{
//    /*
//     * Silo distance from the left of the robot
//     * Silo counted from left
//     * 1: 21
//     * 2: 96
//     * 3: 169
//     * 4: 237
//     * 5: 307
//     */
//
//    static SiloState currentState = STATE_FIND_SILO;
//    static uint32_t lastTimeSiloSeen = 0;
//    static double lastSiloAngle = 0.0, lastxSilo = 0.0, lastySilo = 0.0;
//
//    EKF position = extendedKalmanFilter();
//    uint32_t timer = HAL_GetTick();
//
//    // Read distances and angles for the five silos
//    int siloDistances[5] = {camera[3] * 10, camera[5] * 10, camera[7] * 10, camera[9] * 10, camera[11] * 10}; // convert to mm
//    int siloAngles[5] = {camera[4], camera[6], camera[8], camera[10], camera[12]};
//
//    int xSILO = 0;
//    int ySILO = 0;
//    double siloAngle = 0.0;
//    int siloDistance = 0;
//    int closestSiloIndex = -1;
//
//    // Find the closest detected silo by angle
//    for (int i = 0; i < 5; i++)
//    {
//        if (siloDistances[i] != 99990 && (closestSiloIndex == -1 || abs(siloAngles[i]) < abs(siloAngles[closestSiloIndex])))
//        {
//            closestSiloIndex = i;
//        }
//    }
//
//    if (closestSiloIndex != -1)
//    {
//        siloAngle = siloAngles[closestSiloIndex];
//        siloDistance = siloDistances[closestSiloIndex];
//        xSILO = sin(siloAngle * M_PI / 180) * siloDistance;
//        ySILO = cos(siloAngle * M_PI / 180) * siloDistance;
//    }
//
//    int frontDistance = sensorMEGA[0];
//    int leftDistance = sensorMEGA[1];
//
//    double Vx = 0.0;
//    double Vy = 0.0;
//    double W = 0.0;
//    servo_write(70);
//
//    switch (currentState)
//    {
//        case STATE_FIND_SILO:
//            if (closestSiloIndex != -1 && siloDistance != 99990)
//            {
//                // Silo detected
//                lastxSilo = xSILO;
//                lastySilo = ySILO;
//                lastSiloAngle = siloAngle;
//                lastTimeSiloSeen = timer;
////                HAL_Delay(10);
//                currentState = STATE_MOVE_TOWARDS_SILO;
//            }
//            else
//            {
//                if (timer - lastTimeSiloSeen > 1000)
//                {
//                    currentState = STATE_SEARCH_SILO;
//                }
//            }
//            break;
//
//        case STATE_MOVE_TOWARDS_SILO:
//            if (closestSiloIndex != -1 && siloDistance != 99990)
//            {
//                // Update last known silo position
//                lastxSilo = xSILO;
//                lastySilo = ySILO;
//                lastSiloAngle = siloAngle;
//                lastTimeSiloSeen = timer;
//
//                Vy = 1000;
//                Vx = Vy * tan(siloAngle * M_PI / 180);
//                W = PID_controllerH(siloAngle, 0.0, 1);
//                if (ySILO >= 1800)
//                {
//                    double gain = sensorData[1] * 100;
//                    Vy = 1700 + gain;
//                }
//                else
//                {
//                	W = PID_controllerH(90.0, position.h, 1);
//                }
//                baru(Vx, Vy, W);
////                HAL_Delay(10);
//            }
//            else
//            {
//                // If the silo is lost, use the last known position for up to 2 seconds
//            	if (timer - lastTimeSiloSeen <= 3000)
//                {
//                    if ((frontDistance != 0 && frontDistance <= 20))
//                    {
//                        setMotorSpeed(1, -800);
//                        setMotorSpeed(2, -800);
//                        Inverse_Kinematics(0, 0, 0);
//                    }
//                    else
//                    {
//                    	start(0, 800, 0, 1);
//                    }
//                }
//                else
//                {
//                    currentState = STATE_FIND_SILO;
//                }
//            }
//            break;
//
//        case STATE_SEARCH_SILO:
//            // Search for the silo by rotating
//            static bool motorState = false;
//            if (timer - lastTimeSiloSeen >= 100)
//            {
//                lastTimeSiloSeen = timer;
//                motorState = !motorState;
//                putar(0, 0, motorState ? 85 : 0);
//            }
//            // If the silo is detected during search, transition to moving towards it
//            if (closestSiloIndex != -1 && siloDistance != 99990)
//            {
////            	Inverse_Kinematics(0, 0, 0);
//                currentState = STATE_MOVE_TOWARDS_SILO;
//            }
//            break;
//    }
//}

