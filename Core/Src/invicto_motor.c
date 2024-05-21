#include "invicto_motor.h"

extern TIM_HandleTypeDef htim1, htim2, htim8;
#define NUM_MOTORS 10

Motor motors[NUM_MOTORS] =
    {
        {GPIOC, GPIO_PIN_14, GPIOC, GPIO_PIN_15, TIM_CHANNEL_1, &htim1},	//1
        {GPIOE, GPIO_PIN_6, GPIOC, GPIO_PIN_13, TIM_CHANNEL_2, &htim8},		//2
        {GPIOE, GPIO_PIN_4, GPIOE, GPIO_PIN_5, TIM_CHANNEL_3, &htim1},		//3
        {GPIOC, GPIO_PIN_12, GPIOD, GPIO_PIN_0, TIM_CHANNEL_4, &htim8},		//4
        {GPIOE, GPIO_PIN_3, GPIOE, GPIO_PIN_2, TIM_CHANNEL_1, &htim8},		//5
        {GPIOD, GPIO_PIN_1, GPIOD, GPIO_PIN_2, TIM_CHANNEL_2, &htim1},		//6
        {GPIOE, GPIO_PIN_1, GPIOE, GPIO_PIN_0, TIM_CHANNEL_3, &htim8},		//7
        {GPIOC, GPIO_PIN_5, GPIOC, GPIO_PIN_4, TIM_CHANNEL_4, &htim1},		//8
        {GPIOD, GPIO_PIN_3, GPIOD, GPIO_PIN_4, TIM_CHANNEL_3, &htim2},		//9
        {GPIOB, GPIO_PIN_1, GPIOB, GPIO_PIN_0, TIM_CHANNEL_4, &htim2}		//10
};

void motorDirection(uint8_t motor, uint8_t direction)
{
    Motor *currentMotor = &motors[motor - 1];
    GPIO_PinState dir1_state;
    GPIO_PinState dir2_state;
    switch (direction)
    {
    case forward:
        dir1_state = GPIO_PIN_RESET;
        dir2_state = GPIO_PIN_SET;
        break;
    case backward:
        dir1_state = GPIO_PIN_SET;
        dir2_state = GPIO_PIN_RESET;
        break;
    default:
        dir1_state = GPIO_PIN_RESET;
        dir2_state = GPIO_PIN_RESET;
        break;
    }
    HAL_GPIO_WritePin(currentMotor->dir1_port, currentMotor->dir1_pin, dir1_state);
    HAL_GPIO_WritePin(currentMotor->dir2_port, currentMotor->dir2_pin, dir2_state);
}

void setMotorSpeed(uint8_t motor, double speed)
{
    Motor *currentMotor = &motors[motor - 1];
    if (speed > 0)
    {
        motorDirection(motor, forward);
    }
    else if (speed < 0)
    {
        motorDirection(motor, backward);
        speed = -speed;
    }
    else
    {
        motorDirection(motor, stop);
    }
    __HAL_TIM_SET_COMPARE(currentMotor->timer, currentMotor->channel, (uint32_t) speed);
}

void Inverse_Kinematics(int Vx, int Vy, int W)
{
     double R = 7.6;

     double M1 = -sin(1 * M_PI_4) * Vx + cos(1 * M_PI_4) * Vy - R * W;
     double M2 = -sin(3 * M_PI_4) * Vx + cos(3 * M_PI_4) * Vy - R * W;
     double M3 = -sin(5 * M_PI_4) * Vx + cos(5 * M_PI_4) * Vy - R * W;
     double M4 = -sin(7 * M_PI_4) * Vx + cos(7 * M_PI_4) * Vy - R * W;

     double V1 = (M1 > 550 || M1 < -550) ? fmin(fmax(M1, -2600), 2600) : ((M1 < -0) ? -550 : ((M1 > 0) ? 550 : 0));
     double V2 = (M2 > 550 || M2 < -550) ? fmin(fmax(M2, -2600), 2600) : ((M2 < -0) ? -550 : ((M2 > 0) ? 550 : 0));
     double V3 = (M3 > 550 || M3 < -550) ? fmin(fmax(M3, -2600), 2600) : ((M3 < -0) ? -550 : ((M3 > 0) ? 550 : 0));
     double V4 = (M4 > 550 || M4 < -550) ? fmin(fmax(M4, -2600), 2600) : ((M4 < -0) ? -550 : ((M4 > 0) ? 550 : 0));

     setMotorSpeed(6, V1);
     setMotorSpeed(8, V2);
     setMotorSpeed(4, V3);
     setMotorSpeed(5, V4);
}

void coba(int Vx, int Vy, int W)
{
	 robotPosition currentPos = odometry();
     double R = 7.6;
     double theta = atan2(currentPos.y_global, currentPos.x_global);

     double M1 = -sin(theta + 1 * M_PI_4) * Vx + cos(theta + 1 * M_PI_4) * Vy - R * W;
     double M2 = -sin(theta + 3 * M_PI_4) * Vx + cos(theta + 3 * M_PI_4) * Vy - R * W;
     double M3 = -sin(theta + 5 * M_PI_4) * Vx + cos(theta + 5 * M_PI_4) * Vy - R * W;
     double M4 = -sin(theta + 7 * M_PI_4) * Vx + cos(theta + 7 * M_PI_4) * Vy - R * W;

     double V1 = (M1 > 550 || M1 < -550) ? fmin(fmax(M1, -2600), 2600) : ((M1 < -0) ? -550 : ((M1 > 0) ? 550 : 0));
     double V2 = (M2 > 550 || M2 < -550) ? fmin(fmax(M2, -2600), 2600) : ((M2 < -0) ? -550 : ((M2 > 0) ? 550 : 0));
     double V3 = (M3 > 550 || M3 < -550) ? fmin(fmax(M3, -2600), 2600) : ((M3 < -0) ? -550 : ((M3 > 0) ? 550 : 0));
     double V4 = (M4 > 550 || M4 < -550) ? fmin(fmax(M4, -2600), 2600) : ((M4 < -0) ? -550 : ((M4 > 0) ? 550 : 0));

     setMotorSpeed(6, V1);
     setMotorSpeed(8, V2);
     setMotorSpeed(4, V3);
     setMotorSpeed(5, V4);
}

void putar(int Vx, int Vy, int W)
{
     double R = 7.6;

     double M1 = -sin(1 * M_PI_4) * Vx + cos(1 * M_PI_4) * Vy - R * W;
     double M2 = -sin(3 * M_PI_4) * Vx + cos(3 * M_PI_4) * Vy - R * W;
     double M3 = -sin(5 * M_PI_4) * Vx + cos(5 * M_PI_4) * Vy - R * W;
     double M4 = -sin(7 * M_PI_4) * Vx + cos(7 * M_PI_4) * Vy - R * W;

     double V1 = (M1 > 450 || M1 < -450) ? fmin(fmax(M1, -2600), 2600) : ((M1 < -0) ? -450 : ((M1 > 0) ? 450 : 0));
     double V2 = (M2 > 450 || M2 < -450) ? fmin(fmax(M2, -2600), 2600) : ((M2 < -0) ? -450 : ((M2 > 0) ? 450 : 0));
     double V3 = (M3 > 450 || M3 < -450) ? fmin(fmax(M3, -2600), 2600) : ((M3 < -0) ? -450 : ((M3 > 0) ? 450 : 0));
     double V4 = (M4 > 450 || M4 < -450) ? fmin(fmax(M4, -2600), 2600) : ((M4 < -0) ? -450 : ((M4 > 0) ? 450 : 0));

     setMotorSpeed(6, V1);
     setMotorSpeed(8, V2);
     setMotorSpeed(4, V3);
     setMotorSpeed(5, V4);
}

void start(int Vx, int Vy, int W, uint8_t battery)
{
    double R = 7.6;
    double M1, M2, M3, M4;

    switch (battery)
    {
		case 1: // 24.3 < x < 24.6
			M1 = (-sin(1 * M_PI_4) * Vx + cos(1 * M_PI_4) * Vy - R * W);
			M2 = (-sin(3 * M_PI_4) * Vx + cos(3 * M_PI_4) * Vy - R * W)*1.1;
			M3 = (-sin(5 * M_PI_4) * Vx + cos(5 * M_PI_4) * Vy - R * W)*1.1;
			M4 = (-sin(7 * M_PI_4) * Vx + cos(7 * M_PI_4) * Vy - R * W);
			break;
    	case 2: // > 24.3
		    M1 = -sin(1 * M_PI_4) * Vx + cos(1 * M_PI_4) * Vy - R * W;
		    M2 = (-sin(3 * M_PI_4) * Vx + cos(3 * M_PI_4) * Vy - R * W)*1.04;
		    M3 = (-sin(5 * M_PI_4) * Vx + cos(5 * M_PI_4) * Vy - R * W)*1.04;
		    M4 = -sin(7 * M_PI_4) * Vx + cos(7 * M_PI_4) * Vy - R * W;
    		break;
		case 3: // 23.8 < x < 24.3
		    M1 = -sin(1 * M_PI_4) * Vx + cos(1 * M_PI_4) * Vy - R * W;
		    M2 = (-sin(3 * M_PI_4) * Vx + cos(3 * M_PI_4) * Vy - R * W)*1.062;
		    M3 = (-sin(5 * M_PI_4) * Vx + cos(5 * M_PI_4) * Vy - R * W);
		    M4 = -sin(7 * M_PI_4) * Vx + cos(7 * M_PI_4) * Vy - R * W;
			break;
		case 4: // 23.4 < x < 23.8
		    M1 = -sin(1 * M_PI_4) * Vx + cos(1 * M_PI_4) * Vy - R * W;
		    M2 = (-sin(3 * M_PI_4) * Vx + cos(3 * M_PI_4) * Vy - R * W)*1.06;
		    M3 = (-sin(5 * M_PI_4) * Vx + cos(5 * M_PI_4) * Vy - R * W)*1.03;
		    M4 = -sin(7 * M_PI_4) * Vx + cos(7 * M_PI_4) * Vy - R * W;
		    break;
		case 5: // 23.5 < x < 23.8
		    M1 = -sin(1 * M_PI_4) * Vx + cos(1 * M_PI_4) * Vy - R * W;
		    M2 = (-sin(3 * M_PI_4) * Vx + cos(3 * M_PI_4) * Vy - R * W)*1.02;
		    M3 = (-sin(5 * M_PI_4) * Vx + cos(5 * M_PI_4) * Vy - R * W)*1.04;
		    M4 = -sin(7 * M_PI_4) * Vx + cos(7 * M_PI_4) * Vy - R * W;
		    break;
		case 11:
			M1 = (-sin(1 * M_PI_4) * Vx + cos(1 * M_PI_4) * Vy - R * W);
			M2 = (-sin(3 * M_PI_4) * Vx + cos(3 * M_PI_4) * Vy - R * W);
			M3 = (-sin(5 * M_PI_4) * Vx + cos(5 * M_PI_4) * Vy - R * W);
			M4 = (-sin(7 * M_PI_4) * Vx + cos(7 * M_PI_4) * Vy - R * W);
			break;
	}

    double V1 = (M1 > 500 || M1 < -500) ? fmin(fmax(M1, -2600), 2600) : ((M1 < -0) ? -500 : ((M1 > 0) ? 500 : 0));
    double V2 = (M2 > 500 || M2 < -500) ? fmin(fmax(M2, -2600), 2600) : ((M2 < -0) ? -500 : ((M2 > 0) ? 500 : 0));
    double V3 = (M3 > 500 || M3 < -500) ? fmin(fmax(M3, -2600), 2600) : ((M3 < -0) ? -500 : ((M3 > 0) ? 500 : 0));
    double V4 = (M4 > 500 || M4 < -500) ? fmin(fmax(M4, -2600), 2600) : ((M4 < -0) ? -500 : ((M4 > 0) ? 500 : 0));

    setMotorSpeed(6, V1);
    setMotorSpeed(8, V2);
    setMotorSpeed(4, V3);
    setMotorSpeed(5, V4);
}

void nanjak(int Vx, int Vy, int W)
{
    double R = 7.6;

    double M1 = (-sin(1 * M_PI_4) * Vx + cos(1 * M_PI_4) * Vy - R * W);
    double M2 = (-sin(3 * M_PI_4) * Vx + cos(3 * M_PI_4) * Vy - R * W)*1.1;
    double M3 = (-sin(5 * M_PI_4) * Vx + cos(5 * M_PI_4) * Vy - R * W)*1.2;
    double M4 = (-sin(7 * M_PI_4) * Vx + cos(7 * M_PI_4) * Vy - R * W);

    double V1 = (M1 > 500 || M1 < -500) ? fmin(fmax(M1, -2600), 2600) : ((M1 < -0) ? -500 : ((M1 > 0) ? 500 : 0));
    double V2 = (M2 > 500 || M2 < -500) ? fmin(fmax(M2, -2600), 2600) : ((M2 < -0) ? -500 : ((M2 > 0) ? 500 : 0));
    double V3 = (M3 > 500 || M3 < -500) ? fmin(fmax(M3, -2600), 2600) : ((M3 < -0) ? -500 : ((M3 > 0) ? 500 : 0));
    double V4 = (M4 > 500 || M4 < -500) ? fmin(fmax(M4, -2600), 2600) : ((M4 < -0) ? -500 : ((M4 > 0) ? 500 : 0));

    setMotorSpeed(6, V1);
    setMotorSpeed(8, V2);
    setMotorSpeed(4, V3);
    setMotorSpeed(5, V4);
}
