/*
 * stm32f103x6_PWM_driver.h
 *
 *
 *      Author: Abdelrahman osama
 */

// this driver suitable for TIMER 2..5

#ifndef INC_STM32F103X6_PWM_DRIVER_H_
#define INC_STM32F103X6_PWM_DRIVER_H_

#include "stm32f103x6.h"


// --- configuration struct ---
typedef struct {
	uint8_t channel;      			// set channel for PWM output must be set @ref TIM_Channels

	uint8_t channel_polarity;       // set channel polarity must be set @ref TIM_Channels_polarity

	uint8_t TIM_clock_prescalar;    // set TIM clock prescalar must be set @ref TIM_clock_prescalar

	uint8_t pwm_mode;      			// set channel PWM mode must be set @ref PWM_Modes

	uint32_t frequency;				// set frequency of PWM signal

	float duty_cyle;			    // set duty cycle of PWM must be set as percentage for example 80.235 which is 80.235%
									// float point resolution must not exceed 3 decimal points

	uint32_t TIM_clock;             // TIM instance clock -> no need to set it with value
}TIM_PWM_Config;


// ---Error handling ---
typedef enum {
    PWM_SUCCESS,
    PWM_ERROR_INVALID_FREQUENCY,
    PWM_ERROR_INVALID_DUTY_CYCLE,
} PWM_Status;


// -------------- Reference Macros --------------
// @ref TIM_Channels
#define TIM_CHANNEL1                 0
#define TIM_CHANNEL2				 1
#define TIM_CHANNEL3				 2
#define TIM_CHANNEL4                 3

// @ref TIM_Channels_polarity
#define TIM_CHANNEL_POLARITY_ACTIVE_HIGH  0
#define TIM_CHANNEL_POLARITY_ACTIVE_LOW   2

// @ref TIM_clock_prescalar
#define TIM_CLOCK_PRESCALAR_0		   (0)
#define TIM_CLOCK_PRESCALAR_2		   (1<<8)
#define TIM_CLOCK_PRESCALAR_4		   (2<<8)

// @ref PWM_Modes
#define TIM_MODE_PWM1                 (6 << 4)  // In upcounting, channel is active as long as TIMx_CNT<TIMx_CCR1
#define TIM_MODE_PWM2					(7 << 4)  // In upcounting, channel is inactive as long as TIMx_CNT<TIMx_CCR1


//  ------------------------------ APIs ------------------------------

/**
 * @brief Initializes the PWM module of the specified timer peripheral.
 *
 * This function initializes the PWM module of the specified timer peripheral
 * according to the provided configuration.
 *
 * @param TIM Pointer to the TIM2_5_TypeDef instance representing the timer peripheral.
 * @param config Pointer to the TIM_PWM_Config structure containing the PWM configuration parameters.
 *
 * @return PWM_Status Returns PWM_SUCCESS if initialization is successful, otherwise returns an error code.
 */
PWM_Status MCAL_TIM_PWM_Init(TIM2_5_TypeDef *TIM, TIM_PWM_Config* config);

/**
 * @brief Sets the frequency of the PWM signal generated by the timer peripheral.
 *
 * This function sets the frequency of the PWM signal generated by the timer peripheral.
 *
 * @param Timer Pointer to the TIM2_5_TypeDef instance representing the timer peripheral.
 * @param frequency The desired frequency of the PWM signal.
 *
 * @return PWM_Status Returns PWM_SUCCESS if setting the frequency is successfully set, otherwise returns PWM_ERROR_INVALID_FREQUENCY error code.
 */
PWM_Status MCAL_TIM_PWM_Set_Frequency(TIM2_5_TypeDef *Timer, uint32_t frequency);

/**
 * @brief Sets the duty cycle of a specific PWM channel.
 *
 * This function sets the duty cycle of a specific PWM channel of the timer peripheral.
 *
 * @param TIM Pointer to the TIM2_5_TypeDef instance representing the timer peripheral.
 * @param channel The PWM channel to set the duty cycle for.
 * @param duty_cycle The duty cycle to set, specified as a percentage (e.g., 80.235 for 80.235%).
 *
 * @return PWM_Status Returns PWM_SUCCESS if setting the duty cycle is successfully set, otherwise returns PWM_ERROR_INVALID_DUTY_CYCLE error code.
 */
PWM_Status MCAL_TIM_PWM_Set_DutyCycle(TIM2_5_TypeDef *TIM, uint8_t channel, float duty_cycle);

#endif /* INC_STM32F103X6_EXTI_DRIVER_H_ */
