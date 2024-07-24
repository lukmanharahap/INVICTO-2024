/**
 * @file	invicto_motor.h
 * @author	INVICTO TEAM (Arduino 2021)
 * @brief	Header file of invicto_motor.c.
 */

#ifndef INVICTO_MOTOR_H
#define INVICTO_MOTOR_H

#include <stm32f4xx_hal.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <Library/Inc/odometry.h>

#define stop 0
#define backward 1
#define forward 2

/**
 * @brief Motor structure definition
 */
typedef struct
{
    GPIO_TypeDef *dir1_port;
    uint16_t dir1_pin;
    GPIO_TypeDef *dir2_port;
    uint16_t dir2_pin;
    uint8_t channel;
    TIM_HandleTypeDef *timer;
} Motor;

/**
 * @brief function definition
 */
void motorDirection(uint8_t motor, uint8_t direction);
void setMotorSpeed(uint8_t motor, double speed);
void Inverse_Kinematics(double Vx, double Vy, double W);

#endif
