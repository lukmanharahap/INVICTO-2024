#ifndef INVICTO_MOTOR_H
#define INVICTO_MOTOR_H

#include <stm32f4xx_hal.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include "odometry.h"

#define stop 0
#define backward 1
#define forward 2

typedef struct
{
    GPIO_TypeDef *dir1_port;
    uint16_t dir1_pin;
    GPIO_TypeDef *dir2_port;
    uint16_t dir2_pin;
    uint8_t channel;
    TIM_HandleTypeDef *timer;
} Motor;

void motorDirection(uint8_t motor, uint8_t direction);
void setMotorSpeed(uint8_t motor, double speed);
void Inverse_Kinematics(double Vx, double Vy, double W);
void left(double Vx, double Vy, double W);
void trying(double Vx, double Vy, double W, double desiredHeading, double KpH);
void right(double Vx, double Vy, double W);
void murni(double Vx, double Vy, double W);
void baru(double Vx, double Vy, double W);
void start(int Vx, int Vy, int W, uint8_t battery);
void kanan(int Vx, int Vy, int W);
void nanjak(int Vx, int Vy, int W);
void putar(int Vx, int Vy, int W);
void coba(int Vx, int Vy, int W);

#endif
