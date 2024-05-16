/*
 * stm32f103x6_PWM_driver.c
 *
 *
 *      Author: Abdelrahman osama
 */

#include "stm32f103x6_PWM_driver.h"
#include "stm32f103x6_gpio_driver.h"
#include "stm32f103x6_RCC_driver.h"

#define REG_16BIT_MAX_VAL    65535U

void MCAL_TIM_GPIO_Init(TIM2_5_TypeDef *TIM, uint16_t channel);
TIM_PWM_Config *GP_config[3];

PWM_Status MCAL_TIM_PWM_Init(TIM2_5_TypeDef *TIM, TIM_PWM_Config* config) {

	// duty cycle
	if(config->duty_cyle > 100)
		return PWM_ERROR_INVALID_DUTY_CYCLE;

	// calculate TIM clock
	config->TIM_clock = MCAL_RCC_get_PCLK1(RCC);
	if(config->TIM_clock_prescalar == TIM_CLOCK_PRESCALAR_2){
		config->TIM_clock /= 2;
	}else if(config->TIM_clock_prescalar == TIM_CLOCK_PRESCALAR_4) {
		config->TIM_clock /= 4;
	}

	// enable clock for TIM + set TIM gpio pins configuration
	if(TIM == TIM2){
		RCC_TIM2_ENABLE();
		GP_config[0] = config;
		MCAL_TIM_GPIO_Init(TIM2, config->channel);
	}else if(TIM == TIM3){
		RCC_TIM3_ENABLE();
		GP_config[1] = config;
		MCAL_TIM_GPIO_Init(TIM3, config->channel);
	}else if(TIM == TIM4){
		RCC_TIM4_ENABLE();
		GP_config[2] = config;
		MCAL_TIM_GPIO_Init(TIM4, config->channel);
	}


	// TIM configurations
	// TIMs 2..5 by default are configured for up counting and Edge-aligned mode
	// Auto-reload preload enable
	TIM->CR1 |= 0x80;

	// channel configurations
	// setting channel polarity + enabling the channel
	TIM->CCER |= ((config->channel_polarity | 0x1) << (config->channel * 4));

	// setting PWM mode + Output compare register preload enable
	if(config->channel == TIM_CHANNEL1){
		TIM->CCMR1 |= (config->pwm_mode | 0x8);
	}else if(config->channel == TIM_CHANNEL2){
		TIM->CCMR1 |= (config->pwm_mode | 0x8) << 8;
	}else if(config->channel == TIM_CHANNEL3){
		TIM->CCMR2 |= (config->pwm_mode | 0x8);
	}else {
		TIM->CCMR2 |= (config->pwm_mode | 0x8) << 8;
	}

	PWM_Status status = MCAL_TIM_PWM_Set_Frequency(TIM, config->frequency);
	if(status == PWM_ERROR_INVALID_FREQUENCY)
		return PWM_ERROR_INVALID_FREQUENCY;

	// no check for duty cycle as it is checked at the begining
	status = MCAL_TIM_PWM_Set_DutyCycle(TIM, config->channel, config->duty_cyle);

	// set TIM clock Prescalar
	TIM->CR1 |= config->TIM_clock_prescalar;

	// enable TIM
	TIM->CR1 |= 0x1;

	return PWM_SUCCESS;
}


PWM_Status MCAL_TIM_PWM_Set_Frequency(TIM2_5_TypeDef *TIM, uint32_t frequency){
	uint32_t clock;
	if(TIM == TIM2){
		clock = GP_config[0]->TIM_clock;
	}else if(TIM == TIM3){
		clock = GP_config[1]->TIM_clock;
	}else if(TIM == TIM4){
		clock = GP_config[2]->TIM_clock;
	}

	uint16_t prescalar;
	for(prescalar = 1; prescalar < REG_16BIT_MAX_VAL; prescalar++ ){
		uint32_t ARR_val = (clock / (prescalar * frequency));
		if(ARR_val < REG_16BIT_MAX_VAL){
			TIM->PSC = prescalar;
			TIM->ARR = ARR_val;
			break;
		}
	}

    if (prescalar == REG_16BIT_MAX_VAL)
    	return PWM_ERROR_INVALID_FREQUENCY;

    return PWM_SUCCESS;
}


PWM_Status MCAL_TIM_PWM_Set_DutyCycle(TIM2_5_TypeDef *TIM, uint8_t channel, float duty_cycle){
	if(duty_cycle > 100)
		return PWM_ERROR_INVALID_DUTY_CYCLE;

	if(channel == TIM_CHANNEL1){
		TIM->CCR1 = ((TIM->ARR * duty_cycle * 1000) / 100000);
	}else if(channel == TIM_CHANNEL2){
		TIM->CCR2 = ((TIM->ARR * duty_cycle * 1000) / 100000);
	}else if(channel == TIM_CHANNEL3){
		TIM->CCR3 = ((TIM->ARR * duty_cycle * 1000) / 100000);
	}else if(channel == TIM_CHANNEL4){
		TIM->CCR4 = ((TIM->ARR * duty_cycle * 1000) / 100000);
	}

	return PWM_SUCCESS;
}

void MCAL_TIM_GPIO_Init(TIM2_5_TypeDef *TIM, uint16_t channel) {
	GPIO_TypeDef *GPIO;
	uint16_t pin_number;

	if (TIM == TIM2){
		switch(channel){
			 case TIM_CHANNEL1:
				GPIO = GPIOA;
				pin_number = GPIO_PIN_0;
				break;
			 case TIM_CHANNEL2:
				GPIO = GPIOA;
				pin_number = GPIO_PIN_1;
				break;
			 case TIM_CHANNEL3:
				GPIO = GPIOA;
				pin_number = GPIO_PIN_2;
				break;
			 case TIM_CHANNEL4:
				GPIO = GPIOA;
				pin_number = GPIO_PIN_3;
				break;
		}
	}else if (TIM == TIM3){
		switch(channel){
			 case TIM_CHANNEL1:
				GPIO = GPIOA;
				pin_number = GPIO_PIN_6;
				break;
			 case TIM_CHANNEL2:
				GPIO = GPIOA;
				pin_number = GPIO_PIN_7;
				break;
			 case TIM_CHANNEL3:
				GPIO = GPIOB;
				pin_number = GPIO_PIN_0;
				break;
			 case TIM_CHANNEL4:
				GPIO = GPIOB;
				pin_number = GPIO_PIN_1;
				break;
		}

	}else if (TIM == TIM4){
		switch(channel){
			 case TIM_CHANNEL1:
				GPIO = GPIOB;
				pin_number = GPIO_PIN_6;
				break;
			 case TIM_CHANNEL2:
				GPIO = GPIOB;
				pin_number = GPIO_PIN_7;
				break;
			 case TIM_CHANNEL3:
				GPIO = GPIOB;
				pin_number = GPIO_PIN_8;
				break;
			 case TIM_CHANNEL4:
				GPIO = GPIOB;
				pin_number = GPIO_PIN_9;
				break;
		}
	}

	GPIO_Pin_Config config;
	config.Pin_Mode = Alternate_Function_Output_PP;
	config.Pin_Speed = GPIO_Pin_Max_speed50M;
	config.Pin_Number = pin_number;
	MCAL_GPIO_Init(GPIO, &config);
}

