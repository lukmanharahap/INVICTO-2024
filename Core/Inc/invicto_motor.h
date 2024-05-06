#ifndef INVICTO_MOTOR_H
#define INVICTO_MOTOR_H

#include <stm32f4xx_hal.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

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
void Inverse_Kinematics(int Vx, int Vy, int W);
void start(int Vx, int Vy, int W, uint8_t battery);
void nanjak(int Vx, int Vy, int W);

#endif
