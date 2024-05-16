/*
 * stm32f103x6_gpio_driver.c
 *
 *  Created on: Sep 28, 2022
 *      Author: Abdelrahman osama
 */



#include "stm32f103x6_gpio_driver.h"

uint8_t Get_CRLH_loc(int pin_number)
{
	switch(pin_number)
	{
	case GPIO_PIN_0 :
		return 0;
		break;

	case GPIO_PIN_1 :
		return 4;
		break;

	case GPIO_PIN_2 :
		return 8;
		break;

	case GPIO_PIN_3 :
		return 12;
		break;

	case GPIO_PIN_4 :
		return 16;
		break;

	case GPIO_PIN_5 :
		return 20;
		break;

	case GPIO_PIN_6 :
		return 24;
		break;

	case GPIO_PIN_7 :
		return 28;
		break;

	case GPIO_PIN_8 :
		return 0;
		break;

	case GPIO_PIN_9 :
		return 4;
		break;

	case GPIO_PIN_10 :
		return 8;
		break;

	case GPIO_PIN_11 :
		return 12;
		break;

	case GPIO_PIN_12 :
		return 16;
		break;

	case GPIO_PIN_13 :
		return 20;
		break;

	case GPIO_PIN_14 :
		return 24;
		break;

	case GPIO_PIN_15 :
		return 28;
		break;
	}

	return 0;
}





void MCAL_GPIO_Init(GPIO_TypeDef *GPIOx , GPIO_Pin_Config *pinconfig)
{
	// check on pin if at CRL or CRH --------
	volatile uint32_t *config_register = NULL;
	uint8_t PIN_config;
	config_register = (pinconfig->Pin_Number > GPIO_PIN_7) ?   &GPIOx->CRH : &GPIOx->CRL;


	(*config_register) &= ~(0xF<<Get_CRLH_loc(pinconfig->Pin_Number)); //reset the specific bits to write on them
	// input pin
	if (pinconfig->Pin_Mode == Analog_Mode || pinconfig->Pin_Mode == Floating_Input || pinconfig->Pin_Mode == Input_Pull_Up || pinconfig->Pin_Mode == Input_Pull_Down)
	{
		PIN_config = 0;  // input pin
		if (pinconfig->Pin_Mode == Input_Pull_Up)
		{
			PIN_config |= (pinconfig->Pin_Mode<<2);
			GPIOx->ODR |= pinconfig->Pin_Number;
		}
		else if( pinconfig->Pin_Mode == Input_Pull_Down)
		{
			PIN_config |= ((pinconfig->Pin_Mode-1)<<2);
			GPIOx->ODR &= (~pinconfig->Pin_Number);
		}
		else
		{
			PIN_config |= (pinconfig->Pin_Mode<<2);
		}

		(*config_register) |= (PIN_config << Get_CRLH_loc(pinconfig->Pin_Number));
	}
	else // output pin
	{
		PIN_config = (((pinconfig->Pin_Mode)-0x4)<<2) | ((pinconfig->Pin_Speed)<<0);
		(*config_register) |= (PIN_config << Get_CRLH_loc(pinconfig->Pin_Number));

	}
}


void MCAL_GPIO_DeInit(GPIO_TypeDef * GPIOx)
{

}

uint8_t MCAL_GPIO_Read_Pin(GPIO_TypeDef * GPIOx , uint16_t pinnumber )
{
	uint8_t bit_status;
	if((GPIOx->IDR & pinnumber) !=  (uint32_t)GPIO_PIN_RESET)
	{
		bit_status = GPIO_PIN_SET;
	}
	else
	{
		bit_status = GPIO_PIN_RESET;
	}
	return bit_status;
}

uint16_t MCAL_GPIO_Read_Port(GPIO_TypeDef * GPIOx)
{
	uint16_t port_value;
	port_value = (uint16_t) GPIOx->IDR;
	return port_value;
}

void MCAL_GPIO_Write_Pin(GPIO_TypeDef * GPIOx ,  uint16_t pinnumber , uint8_t value)
{
	if(value != GPIO_PIN_RESET)
	{
		GPIOx->ODR |= pinnumber;
	}
	else
	{
		GPIOx->ODR &= ~(pinnumber);
	}
}

void MCAL_GPIO_Write_Port(GPIO_TypeDef * GPIOx  , uint16_t value)
{
	GPIOx->ODR = (uint32_t)value ;
}
void MCAL_GPIO_Toggle_Pin(GPIO_TypeDef * GPIOx ,  uint16_t pinnumber)
{
	GPIOx->ODR ^= pinnumber;
}
uint8_t MCAL_GPIO_Lock_Pin(GPIO_TypeDef * GPIOx , uint16_t pinnumber)
{
	uint16_t temp;
	GPIOx->LCKR |= pinnumber;
	// locking sequence
	GPIOx->LCKR |= (1<<16);
	GPIOx->LCKR &= ~(1<<16);
	GPIOx->LCKR |= (1<<16);
	// Read 0
	temp =  GPIOx->LCKR ;
	// Read 1
	temp = (GPIOx->LCKR >> 16);
	if (temp)
	{
		return PIN_LOCK_ENABLED;
	}
	else
	{
		return PIN_LOCK_NOT_ENABLED;
	}
}

