/*
 * servo.h
 *
 *  Created on: Apr 22, 2024
 *      Author: abdelrahman
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include "stm32f103x6.h"
#include "stm32f103x6_PWM_driver.h"

#define SERVO_0_ANGLE_DUTY_CYCLE        1.5
#define SERVO_180_ANGLE_DUTY_CYCLE        9
#define SERVO_FREQUENCY                  50
#define ANGLE_STEP                     	416   // (uint16_t)(((SERVO_180_ANGLE_DUTY_CYCLE - SERVO_0_ANGLE_DUTY_CYCLE) *10000) / 180)
#define SERVO_ANGLE_TO_DUTY_CYCLE(ANGLE) ((ANGLE * ANGLE_STEP + 15000)/(float)10000)

typedef struct {
	TIM2_5_TypeDef *TIM;
	uint8_t TIM_channel;
	uint8_t curr_angle;
}Servo_Config;

// ----------------------- APIs -----------------------

/**
 * @brief Initializes the servo motor with the specified configuration.
 *
 * This function initializes the servo motor with the specified configuration,
 * setting up the PWM module of the associated timer peripheral.
 *
 * @param config Pointer to the Servo_Config structure containing the servo motor configuration parameters.
 */
void HAL_Servo_Init(Servo_Config* config);

/**
 * @brief Writes the specified goal angle to the servo motor.
 *
 * This function writes the specified goal angle to the servo motor by adjusting
 * the PWM duty cycle of the associated timer peripheral.
 *
 * @param config Pointer to the Servo_Config structure containing the servo motor configuration.
 * @param goal_angle The desired goal angle to be set for the servo motor.
 */
void HAL_Servo_Write(Servo_Config* config, uint8_t goal_angle);

/**
 * @brief Reads the current angle of the servo motor.
 *
 * This function reads the current angle of the servo motor from its configuration.
 *
 * @param config Pointer to the Servo_Config structure containing the servo motor configuration.
 *
 * @return uint8_t The current angle of the servo motor.
 */
uint8_t HAL_Servo_Read(Servo_Config* config);

#endif /* INC_SERVO_H_ */
