/*
 * servo.c
 *
 *  Created on: Apr 22, 2024
 *      Author: abdelrahman
 */


#include "Inc/servo.h"


void HAL_Servo_Init(Servo_Config* config){
	TIM_PWM_Config pwm_config;
	pwm_config.channel = config->TIM_channel;
	pwm_config.channel_polarity = TIM_CHANNEL_POLARITY_ACTIVE_HIGH;
	pwm_config.TIM_clock_prescalar = TIM_CLOCK_PRESCALAR_0;
	pwm_config.pwm_mode = TIM_MODE_PWM1;
	pwm_config.duty_cyle = 0;
	pwm_config.frequency = SERVO_FREQUENCY;

	MCAL_TIM_PWM_Init(config->TIM, &pwm_config);
}

void HAL_Servo_Write(Servo_Config* config, uint8_t goal_angle){
	MCAL_TIM_PWM_Set_DutyCycle(config->TIM, config->TIM_channel, SERVO_ANGLE_TO_DUTY_CYCLE(goal_angle));
	config->curr_angle = goal_angle;
}

uint8_t HAL_Servo_Read(Servo_Config* config){
	return config->curr_angle;
}
