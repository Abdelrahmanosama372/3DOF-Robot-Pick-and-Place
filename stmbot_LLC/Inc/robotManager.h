/*
 * robotManager.h
 *
 *
 *      Author: abdelrahman
 */

#ifndef ROBOTMANAGER_H_
#define ROBOTMANAGER_H_

#include "servo.h"
#include "stm32f103x6.h"
#include "stm32f103x6_USART_driver.h"

/** @brief Buffer size for commands. */
#define COMMAND_BUFFER_SIZE 16

/** @brief Number of servos. */
#define NUM_SERVOS 4

/**
 * @brief Structure representing the robot manager configuration.
 */
typedef struct {
	Servo_Config servos[NUM_SERVOS];
	uint8_t command[COMMAND_BUFFER_SIZE]; 	// command in form "###;###;###;###" -> every ### is a servo motor angle
	uint8_t command_readyToRead;  			// flag to indicate whether data is ready to be read or not.
	uint8_t servo_angles[4];
}robotManager;

/**
 * @brief Initializes the robot manager with the specified UART and TIM configurations.
 *
 * This function initializes the servos and UART configurations. It sets up the servo
 * channels, initializes the UART with the given settings, and configures the GPIO pins for UART.
 *
 * @param rm Pointer to the robot manager structure to initialize.
 * @param UART Pointer to the USART peripheral to use.
 * @param TIM Pointer to the TIM peripheral to use.
 */
void robotManager_init(robotManager* rm, USART_TypeDef *UART, TIM2_5_TypeDef *TIM);

/**
 * @brief Runs the robot manager to process commands and update servo positions.
 *
 * This function checks if a command is ready to be read, parses the command to extract servo
 * angles, and updates the servo positions accordingly.
 *
 * @param rm Pointer to the robot manager structure.
 */
void robotManager_run(robotManager* rm);

#endif /* ROBOTMANAGER_H_ */
