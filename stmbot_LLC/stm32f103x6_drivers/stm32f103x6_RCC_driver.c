/*
 * stm32f103x6_RCC_driver.c
 *
 *
 *      Author: Abdelrahman osama
 */

#include "stm32f103x6_RCC_driver.h"

/*HPRE[3:0]: AHB prescaler
Set and cleared by software to control AHB clock division factor.
0xxx: SYSCLK not divided
1000: SYSCLK divided by 2
1001: SYSCLK divided by 4
1010: SYSCLK divided by 8
1011: SYSCLK divided by 16
1100: SYSCLK divided by 64
1101: SYSCLK divided by 128
1110: SYSCLK divided by 256
1111: SYSCLK divided by 512*/

uint16_t AHB_Prescaler[16]={1, 1, 1, 1, 1, 1, 1, 1, 2, 4, 8, 16, 64, 128, 256, 512};

/*0xx: HCLK not divided
100: HCLK divided by 2
101: HCLK divided by 4
110: HCLK divided by 8
111: HCLK divided by 16*/

uint8_t APB_1_or_2_Prescaler[8]={1, 1, 1, 1, 2, 4, 8, 16};

uint32_t MCAL_RCC_get_SYSCLK(RCC_TypeDef *_RCC)
{
	if(_RCC->CR & RCC_HSI_clock)
		return 8000000;
	// assumed that HSI is the running clock always
	else
		return 0;

}
uint32_t MCAL_RCC_get_HCLK(RCC_TypeDef *_RCC)
{
	uint8_t AHB_prescaler_index;
	// calculate the SYSCLK
	uint32_t SYSCLK = MCAL_RCC_get_SYSCLK(_RCC);

	// getting the AHB clock prescaler
	AHB_prescaler_index = (_RCC->CFGR & (0xF << 4)) >> 4;

	return SYSCLK/AHB_Prescaler[AHB_prescaler_index];
}

uint32_t MCAL_RCC_get_PCLK1(RCC_TypeDef *_RCC)
{
	// calculate the HCLK
	uint32_t HCLK = MCAL_RCC_get_HCLK(_RCC);

	// getting the APB1 clock prescaler
	uint8_t APB1_prescaler_index = (uint8_t)((_RCC->CFGR & (0x7 << 8)) >> 8);

	return HCLK/APB_1_or_2_Prescaler[APB1_prescaler_index];
}

uint32_t MCAL_RCC_get_PCLK2(RCC_TypeDef *_RCC)
{
	// calculate the HCLK
	uint32_t HCLK = MCAL_RCC_get_HCLK(_RCC);

	// getting the APB2 clock prescaler
	uint8_t APB2_prescaler_index = (uint8_t)((_RCC->CFGR & (0x7 << 11)) >> 11);

	return HCLK/APB_1_or_2_Prescaler[APB2_prescaler_index];
}

