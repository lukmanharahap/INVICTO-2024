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
        {GPIOC, GPIO_PIN_4, GPIOC, GPIO_PIN_5, TIM_CHANNEL_4, &htim1},		//8
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

void Inverse_Kinematics(double Vx, double Vy, double W)
{
    double R = 7.6;
    double minThreshold = 1000;
    double wheelMaxSpeed = 5500;

    double M1 = (-sin(M_PI_4) * Vx + cos(M_PI_4) * Vy - R * W);
    double M2 = (-sin(3 * M_PI_4) * Vx + cos(3 * M_PI_4) * Vy - R * W);
    double M3 = (-sin(5 * M_PI_4) * Vx * 0.78 + cos(5 * M_PI_4) * Vy - R * W); //0.952702703
    double M4 = (-sin(7 * M_PI_4) * Vx + cos(7 * M_PI_4) * Vy - R * W); // 0.893632381

    double maxM = fmax(fabs(M1), fmax(fabs(M2), fmax(fabs(M3), fabs(M4))));

    if (maxM > wheelMaxSpeed) {
        double scale = wheelMaxSpeed / maxM;
        M1 *= scale;
        M2 *= scale;
        M3 *= scale;
        M4 *= scale;
    }

    double V1 = (fabs(M1) > minThreshold) ? M1 : (M1 < 0) ? -minThreshold : ((M1 > 0) ? minThreshold : 0);
    double V2 = (fabs(M2) > minThreshold) ? M2 : (M2 < 0) ? -minThreshold : ((M2 > 0) ? minThreshold : 0);
    double V3 = (fabs(M3) > minThreshold) ? M3 : (M3 < 0) ? -minThreshold : ((M3 > 0) ? minThreshold : 0);
    double V4 = (fabs(M4) > minThreshold) ? M4 : (M4 < 0) ? -minThreshold : ((M4 > 0) ? minThreshold : 0);

    setMotorSpeed(6, (int)V1);
    setMotorSpeed(5, (int)V2);
    setMotorSpeed(4, (int)V3);
    setMotorSpeed(8, (int)V4);
}

void left(double Vx, double Vy, double W)
{
    double R = 7.6;
    double minThreshold = 1000;
    double wheelMaxSpeed = 5500;

    double M1 = (-sin(M_PI_4) * Vx + cos(M_PI_4) * Vy - R * W);
    double M2 = (-sin(3 * M_PI_4) * Vx + cos(3 * M_PI_4) * Vy - R * W);
    double M3 = (-sin(5 * M_PI_4) * Vx * 0.9 + cos(5 * M_PI_4) * Vy * 1.1 - R * W); //0.952702703
    double M4 = (-sin(7 * M_PI_4) * Vx + cos(7 * M_PI_4) * Vy - R * W); // 0.893632381

    double maxM = fmax(fabs(M1), fmax(fabs(M2), fmax(fabs(M3), fabs(M4))));

    if (maxM > wheelMaxSpeed) {
        double scale = wheelMaxSpeed / maxM;
        M1 *= scale;
        M2 *= scale;
        M3 *= scale;
        M4 *= scale;
    }

    double V1 = (fabs(M1) > minThreshold) ? M1 : (M1 < 0) ? -minThreshold : ((M1 > 0) ? minThreshold : 0);
    double V2 = (fabs(M2) > minThreshold) ? M2 : (M2 < 0) ? -minThreshold : ((M2 > 0) ? minThreshold : 0);
    double V3 = (fabs(M3) > minThreshold) ? M3 : (M3 < 0) ? -minThreshold : ((M3 > 0) ? minThreshold : 0);
    double V4 = (fabs(M4) > minThreshold) ? M4 : (M4 < 0) ? -minThreshold : ((M4 > 0) ? minThreshold : 0);

    setMotorSpeed(6, (int)V1);
    setMotorSpeed(5, (int)V2);
    setMotorSpeed(4, (int)V3);
    setMotorSpeed(8, (int)V4);
}

void trying(double Vx, double Vy, double W, double desiredHeading, double KpH)
{
	external_global currentPosition = odometry_eg();

    double R = 7.6;
    double minThreshold = 1000;
    double wheelMaxSpeed = 5500;

    // Read current heading
    double yaw = currentPosition.h;

    // Calculate heading error
    double headingError = desiredHeading - yaw;
    if (headingError > 180) headingError -= 360;
    if (headingError < -180) headingError += 360;

    // Adjust rotational speed based on heading error
    W += KpH * headingError; // KpH is a heading correction gain

    double M1 = (-sin(M_PI_4) * Vx * 1.25 + cos(M_PI_4) * Vy - R * W);
    double M2 = (-sin(3 * M_PI_4) * Vx * 1.4 + cos(3 * M_PI_4) * Vy - R * W);
    double M3 = (-sin(5 * M_PI_4) * Vx + cos(5 * M_PI_4) * Vy - R * W);
    double M4 = (-sin(7 * M_PI_4) * Vx + cos(7 * M_PI_4) * Vy - R * W);

    double maxM = fmax(fabs(M1), fmax(fabs(M2), fmax(fabs(M3), fabs(M4))));

    if (maxM > wheelMaxSpeed) {
        double scale = wheelMaxSpeed / maxM;
        M1 *= scale;
        M2 *= scale;
        M3 *= scale;
        M4 *= scale;
    }

    double V1 = (fabs(M1) > minThreshold) ? M1 : (M1 < 0) ? -minThreshold : ((M1 > 0) ? minThreshold : 0);
    double V2 = (fabs(M2) > minThreshold) ? M2 : (M2 < 0) ? -minThreshold : ((M2 > 0) ? minThreshold : 0);
    double V3 = (fabs(M3) > minThreshold) ? M3 : (M3 < 0) ? -minThreshold : ((M3 > 0) ? minThreshold : 0);
    double V4 = (fabs(M4) > minThreshold) ? M4 : (M4 < 0) ? -minThreshold : ((M4 > 0) ? minThreshold : 0);

    setMotorSpeed(6, (int)V1);
    setMotorSpeed(5, (int)V2);
    setMotorSpeed(4, (int)V3);
    setMotorSpeed(8, (int)V4);
}

void right(double Vx, double Vy, double W)
{
    double R = 7.6;
    double minThreshold = 1100;
    double wheelMaxSpeed = 5500;

    double M1 = (-sin(M_PI_4) * Vx + cos(M_PI_4) * Vy - R * W);
    double M2 = (-sin(3 * M_PI_4) * Vx + cos(3 * M_PI_4) * Vy - R * W);
    double M3 = (-sin(5 * M_PI_4) * Vx + cos(5 * M_PI_4) * Vy - R * W)*1.1059674776;
    double M4 = (-sin(7 * M_PI_4) * Vx + cos(7 * M_PI_4) * Vy - R * W)*0.8789318513;

    double maxM = fmax(fabs(M1), fmax(fabs(M2), fmax(fabs(M3), fabs(M4))));

    if (maxM > wheelMaxSpeed) {
        double scale = wheelMaxSpeed / maxM;
        M1 *= scale;
        M2 *= scale;
        M3 *= scale;
        M4 *= scale;
    }

    double V1 = (fabs(M1) > minThreshold) ? M1 : (M1 < 0) ? -minThreshold : ((M1 > 0) ? minThreshold : 0);
    double V2 = (fabs(M2) > minThreshold) ? M2 : (M2 < 0) ? -minThreshold : ((M2 > 0) ? minThreshold : 0);
    double V3 = (fabs(M3) > minThreshold) ? M3 : (M3 < 0) ? -minThreshold : ((M3 > 0) ? minThreshold : 0);
    double V4 = (fabs(M4) > minThreshold) ? M4 : (M4 < 0) ? -minThreshold : ((M4 > 0) ? minThreshold : 0);

    setMotorSpeed(6, (int)V1);
    setMotorSpeed(5, (int)V2);
    setMotorSpeed(4, (int)V3);
    setMotorSpeed(8, (int)V4);
}

void murni(double Vx, double Vy, double W)
{
    double R = 7.6;
    double minThreshold = 1100;
    double wheelMaxSpeed = 5500;

    double M1 = (-sin(M_PI_4) * Vx + cos(M_PI_4) * Vy - R * W);
    double M2 = (-sin(3 * M_PI_4) * Vx + cos(3 * M_PI_4) * Vy - R * W);
    double M3 = (-sin(5 * M_PI_4) * Vx + cos(5 * M_PI_4) * Vy - R * W);
    double M4 = (-sin(7 * M_PI_4) * Vx + cos(7 * M_PI_4) * Vy - R * W);

    double maxM = fmax(fabs(M1), fmax(fabs(M2), fmax(fabs(M3), fabs(M4))));

    if (maxM > wheelMaxSpeed) {
        double scale = wheelMaxSpeed / maxM;
        M1 *= scale;
        M2 *= scale;
        M3 *= scale;
        M4 *= scale;
    }

    double V1 = (fabs(M1) > minThreshold) ? M1 : (M1 < 0) ? -minThreshold : ((M1 > 0) ? minThreshold : 0);
    double V2 = (fabs(M2) > minThreshold) ? M2 : (M2 < 0) ? -minThreshold : ((M2 > 0) ? minThreshold : 0);
    double V3 = (fabs(M3) > minThreshold) ? M3 : (M3 < 0) ? -minThreshold : ((M3 > 0) ? minThreshold : 0);
    double V4 = (fabs(M4) > minThreshold) ? M4 : (M4 < 0) ? -minThreshold : ((M4 > 0) ? minThreshold : 0);

    setMotorSpeed(6, (int)V1);
    setMotorSpeed(5, (int)V2);
    setMotorSpeed(4, (int)V3);
    setMotorSpeed(8, (int)V4);
}

void putar(int Vx, int Vy, int W)
{
     double R = 7.6;
     int minSpeed = 580;
     int maxSpeed = 1000;

     double M1 = -sin(1 * M_PI_4) * Vx + cos(1 * M_PI_4) * Vy - R * W;
     double M2 = -sin(3 * M_PI_4) * Vx + cos(3 * M_PI_4) * Vy - R * W;
     double M3 = -sin(5 * M_PI_4) * Vx + cos(5 * M_PI_4) * Vy - R * W;
     double M4 = -sin(7 * M_PI_4) * Vx + cos(7 * M_PI_4) * Vy - R * W;

     double V1 = (M1 > minSpeed || M1 < -minSpeed) ? fmin(fmax(M1, -maxSpeed), maxSpeed) : ((M1 < -0) ? -minSpeed : ((M1 > 0) ? minSpeed : 0));
     double V2 = (M2 > minSpeed || M2 < -minSpeed) ? fmin(fmax(M2, -maxSpeed), maxSpeed) : ((M2 < -0) ? -minSpeed : ((M2 > 0) ? minSpeed : 0));
     double V3 = (M3 > minSpeed || M3 < -minSpeed) ? fmin(fmax(M3, -maxSpeed), maxSpeed) : ((M3 < -0) ? -minSpeed : ((M3 > 0) ? minSpeed : 0));
     double V4 = (M4 > minSpeed || M4 < -minSpeed) ? fmin(fmax(M4, -maxSpeed), maxSpeed) : ((M4 < -0) ? -minSpeed : ((M4 > 0) ? minSpeed : 0));

     setMotorSpeed(6, (int)V1);
     setMotorSpeed(5, (int)V2);
     setMotorSpeed(4, (int)V3);
     setMotorSpeed(8, (int)V4);
}
