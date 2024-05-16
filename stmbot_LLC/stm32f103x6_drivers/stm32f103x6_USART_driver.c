/*
 * stm32f103x6_USART_driver.c
 *
 *
 *      Author: Abdelrahman osama
 */



#include "stm32f103x6_USART_driver.h"

//=====================================================================================
//								Generic Variables
//=====================================================================================

USART_config Global_USARTS_config[3];


//=====================================================================================
//									APIs definitions
//=====================================================================================

void MCAL_USART_Init(USART_TypeDef *USARTx,USART_config *USART_config)
{
	// saving version of the USART configuration in a global variable
	if (USARTx == USART1)
	{
		RCC_USART1_ENABLE();
		Global_USARTS_config[0] = *USART_config;
	}

	else if (USARTx == USART2)
	{
		RCC_USART2_ENABLE();
		Global_USARTS_config[1] = *USART_config;
	}

	else
	{
		// USART3
		RCC_USART3_ENABLE();
		Global_USARTS_config[2] = *USART_config;
	}

	// Enable the USART by writing the UE bit in USART_CR1 register to 1
	USARTx->CR1 |= (1 << 13);

	// Program the M bit in USART_CR1 to define the word length
	USARTx->CR1 |= USART_config->Data_Length;

	// Program the number of stop bits in USART_CR2
	USARTx->CR2 |= USART_config->Stop_Bits;

	// Select the desired baud rate using the USART_BRR register
	if( USARTx == USART1 )
	{
		USARTx->BRR = USART_BRR_val(MCAL_RCC_get_PCLK2(RCC),USART_config->Baud_Rate);
	}
	else
	{
		USARTx->BRR = USART_BRR_val(MCAL_RCC_get_PCLK1(RCC),USART_config->Baud_Rate);
	}


	// configure USART mode TX or RX or both
	USARTx->CR1 |= USART_config->USART_MODE;

	// configure parity bit even or odd or no parity
	USARTx->CR1 |= USART_config->Parity;

	// USART interrupt configuration
	if (USART_config->Interrupt_Enable != USART_Interrupt_disable)
	{
		// Allow USART to generate interrupt
		USARTx->CR1 |= USART_config->Interrupt_Enable;

		// Enable the corresponding interrupt mask in the NVIC
		if (USARTx == USART1)
		{
			NVIC_IRQ37_USART1_ENABLE;
		}

		else if (USARTx == USART2)
		{
			NVIC_IRQ38_USART2_ENABLE;
		}

		else
		{
			// USART3
			NVIC_IRQ39_USART3_ENABLE;
		}

	}

}

void MCAL_USART_Deinit(USART_TypeDef *USARTx)
{
	if (USARTx == USART1)
	{
		// clearing the corresponding interrupt mask in the NVIC
		NVIC_IRQ37_USART1_DISABLE;
		// reset the USART
		RCC_USART1_RESET();
	}

	else if (USARTx == USART2)
	{
		// clearing the corresponding interrupt mask in the NVIC
		NVIC_IRQ38_USART2_DISABLE;
		// reset the USART
		RCC_USART2_RESET();
	}

	else
	{
		// USART3
		// clearing the corresponding interrupt mask in the NVIC
		NVIC_IRQ39_USART3_DISABLE;
		// reset the USART
		RCC_USART3_RESET();
	}

}

void MCAL_USART_TX(USART_TypeDef *USARTx,uint8_t *data, uint32_t data_len, Data_Handling Handler_mech)
{
	if( Handler_mech == Polling)
	{
		uint32_t i;
		for(i=0; i<data_len; i++)
		{
			// check transmitter data register is empty (TXE = 1)
			while( !(USARTx->SR & (1 << 7)) );
			// transmit data
			USARTx->DR = (*data & 0xFF);
			data++;
		}
	}
	// check transmission is completed (TC = 1)
	while(!(USARTx->SR & (1 << 6)) );

}

void MCAL_USART_RX(USART_TypeDef *USARTx, uint16_t *data, uint32_t data_len, Data_Handling Handler_mech)
{
	if( Handler_mech == Polling)
	{
		uint32_t i;
		uint8_t USART_config_index;
		// check on the UASRT number transmitting to check parity configuration
		if (USARTx == USART1)
		{
			USART_config_index = 0;
		}

		else if (USARTx == USART2)
		{
			USART_config_index = 1;
		}

		else
		{
			USART_config_index = 2;
		}


		if (Global_USARTS_config[USART_config_index].Parity == USART_Frame_parity_DISABLE)
		{
			if (Global_USARTS_config[USART_config_index].Data_Length == USART_Frame_Length_9)
			{
				for(i=0; i<data_len; i++)
				{
					// polling on Received data is ready to be read (RXNE=1)
					while (!(USARTx->SR & (1 << 5)));
					// receiving data (9 bits)
					*data = (USARTx->DR & 0x1FF);
				}

			}
			else
			{
				// USART_Frame_Length_8
				for(i=0; i<data_len; i++)
				{
					// polling on Received data is ready to be read (RXNE=1)
					while (!(USARTx->SR & (1 << 5)));
					// receiving data (8 bits)
					*data = (USARTx->DR & 0xFF);
				}
			}
		}
		else
		{
			if (Global_USARTS_config[USART_config_index].Data_Length == USART_Frame_Length_9)
			{
				for(i=0; i<data_len; i++)
				{
					// polling on Received data is ready to be read (RXNE=1)
					while (!(USARTx->SR & (1 << 5)));
					// receiving data (8 bits)
					*data = (USARTx->DR & 0xFF);
				}

			}
			else
			{
				// USART_Frame_Length_8
				for(i=0; i<data_len; i++)
				{
					// polling on Received data is ready to be read (RXNE=1)
					while (!(USARTx->SR & (1 << 5)));
					// receiving data (7 bits)
					*data = (USARTx->DR & 0x7F);
				}
			}

		}
	}
	else {
		*data = (USARTx->DR & 0xFF);
	}

}


void MCAL_USART_GPIO_Set_Pins(USART_TypeDef *USARTx)
{
	GPIO_Pin_Config pin_config;

	if( USARTx == USART1 )
	{
		// Enable Clock to GPIOA
		RCC_GPIOA_ENABLE();

		// configure TX or RX or both pins depending on USART_mode
		if(Global_USARTS_config[0].USART_MODE == USART_MODE_RX )
		{
			// configure RX pin
			pin_config.Pin_Mode = Input_Pull_Up;
			pin_config.Pin_Number = GPIO_PIN_10;
			MCAL_GPIO_Init(GPIOA, &pin_config);
		}

		else if (Global_USARTS_config[0].USART_MODE == USART_MODE_TX)
		{
			// configure TX pin
			pin_config.Pin_Mode = Alternate_Function_Output_PP;
			pin_config.Pin_Number = GPIO_PIN_9;
			pin_config.Pin_Speed = GPIO_Pin_Max_Speed10M;
			MCAL_GPIO_Init(GPIOA, &pin_config);
		}

		else
		{
			// configure TX pin
			pin_config.Pin_Mode = Alternate_Function_Output_PP;
			pin_config.Pin_Number = GPIO_PIN_9;
			pin_config.Pin_Speed = GPIO_Pin_Max_Speed10M;
			MCAL_GPIO_Init(GPIOA, &pin_config);

			// configure RX pin
			pin_config.Pin_Mode = Input_Pull_Up;
			pin_config.Pin_Number = GPIO_PIN_10;
			MCAL_GPIO_Init(GPIOA, &pin_config);
		}

		if(Global_USARTS_config[0].HW_FlowControl != USART_HW_FC_disable)
		{
			// configure HW flow control pins
			if (Global_USARTS_config[0].HW_FlowControl == USART_CTS_Enable)
			{
				// configure CTS pin
				pin_config.Pin_Mode = Input_Pull_Up;
				pin_config.Pin_Number = GPIO_PIN_11;
				MCAL_GPIO_Init(GPIOA, &pin_config);
			}

			else if (Global_USARTS_config[0].HW_FlowControl == USART_RTS_Enable)
			{
				// configure RTS pin
				pin_config.Pin_Mode = Alternate_Function_Output_PP;
				pin_config.Pin_Number = GPIO_PIN_12;
				pin_config.Pin_Speed = GPIO_Pin_Max_Speed10M;
				MCAL_GPIO_Init(GPIOA, &pin_config);
			}

			else
			{
				// configure CTS pin
				pin_config.Pin_Mode = Input_Pull_Up;
				pin_config.Pin_Number = GPIO_PIN_11;
				MCAL_GPIO_Init(GPIOA, &pin_config);

				// configure RTS pin
				pin_config.Pin_Mode = Alternate_Function_Output_PP;
				pin_config.Pin_Number = GPIO_PIN_12;
				pin_config.Pin_Speed = GPIO_Pin_Max_Speed10M;
				MCAL_GPIO_Init(GPIOA, &pin_config);
			}
		}

	}

	if( USARTx == USART2 )
	{
		// Enable Clock to GPIOA
		RCC_GPIOA_ENABLE();

		// configure TX or RX or both pins depending on USART_mode
		if(Global_USARTS_config[1].USART_MODE == USART_MODE_RX )
		{
			// configure RX pin
			pin_config.Pin_Mode = Input_Pull_Up;
			pin_config.Pin_Number = GPIO_PIN_3;
			MCAL_GPIO_Init(GPIOA, &pin_config);
		}

		else if (Global_USARTS_config[1].USART_MODE == USART_MODE_TX)
		{
			// configure TX pin
			pin_config.Pin_Mode = Alternate_Function_Output_PP;
			pin_config.Pin_Number = GPIO_PIN_2;
			pin_config.Pin_Speed = GPIO_Pin_Max_Speed10M;
			MCAL_GPIO_Init(GPIOA, &pin_config);
		}

		else
		{
			// configure TX pin
			pin_config.Pin_Mode = Alternate_Function_Output_PP;
			pin_config.Pin_Number = GPIO_PIN_2;
			pin_config.Pin_Speed = GPIO_Pin_Max_Speed10M;
			MCAL_GPIO_Init(GPIOA, &pin_config);

			// configure RX pin
			pin_config.Pin_Mode = Input_Pull_Up;
			pin_config.Pin_Number = GPIO_PIN_3;
			MCAL_GPIO_Init(GPIOA, &pin_config);
		}

		if(Global_USARTS_config[1].HW_FlowControl != USART_HW_FC_disable)
		{
			// configure HW flow control pins
			if (Global_USARTS_config[1].HW_FlowControl == USART_CTS_Enable)
			{
				// configure CTS pin
				pin_config.Pin_Mode = Input_Pull_Up;
				pin_config.Pin_Number = GPIO_PIN_0;
				MCAL_GPIO_Init(GPIOA, &pin_config);
			}

			else if (Global_USARTS_config[1].HW_FlowControl == USART_RTS_Enable)
			{
				// configure RTS pin
				pin_config.Pin_Mode = Alternate_Function_Output_PP;
				pin_config.Pin_Number = GPIO_PIN_1;
				pin_config.Pin_Speed = GPIO_Pin_Max_Speed10M;
				MCAL_GPIO_Init(GPIOA, &pin_config);
			}

			else
			{
				// configure CTS pin
				pin_config.Pin_Mode = Input_Pull_Up;
				pin_config.Pin_Number = GPIO_PIN_0;
				MCAL_GPIO_Init(GPIOA, &pin_config);

				// configure RTS pin
				pin_config.Pin_Mode = Alternate_Function_Output_PP;
				pin_config.Pin_Number = GPIO_PIN_1;
				pin_config.Pin_Speed = GPIO_Pin_Max_Speed10M;
				MCAL_GPIO_Init(GPIOA, &pin_config);
			}
		}

	}

	if( USARTx == USART3 )
	{
		// Enable Clock to GPIOB
		RCC_GPIOB_ENABLE();

		// configure TX or RX or both pins depending on USART_mode
		if(Global_USARTS_config[2].USART_MODE == USART_MODE_RX )
		{
			// configure RX pin
			pin_config.Pin_Mode = Input_Pull_Up;
			pin_config.Pin_Number = GPIO_PIN_11;
			MCAL_GPIO_Init(GPIOB, &pin_config);
		}

		else if (Global_USARTS_config[2].USART_MODE == USART_MODE_TX)
		{
			// configure TX pin
			pin_config.Pin_Mode = Alternate_Function_Output_PP;
			pin_config.Pin_Number = GPIO_PIN_10;
			pin_config.Pin_Speed = GPIO_Pin_Max_Speed10M;
			MCAL_GPIO_Init(GPIOB, &pin_config);
		}

		else
		{
			// configure TX pin
			pin_config.Pin_Mode = Alternate_Function_Output_PP;
			pin_config.Pin_Number = GPIO_PIN_10;
			pin_config.Pin_Speed = GPIO_Pin_Max_Speed10M;
			MCAL_GPIO_Init(GPIOB, &pin_config);

			// configure RX pin
			pin_config.Pin_Mode = Input_Pull_Up;
			pin_config.Pin_Number = GPIO_PIN_11;
			MCAL_GPIO_Init(GPIOB, &pin_config);
		}

		if(Global_USARTS_config[2].HW_FlowControl != USART_HW_FC_disable)
		{
			// configure HW flow control pins
			if (Global_USARTS_config[2].HW_FlowControl == USART_CTS_Enable)
			{
				// configure CTS pin
				pin_config.Pin_Mode = Input_Pull_Up;
				pin_config.Pin_Number = GPIO_PIN_13;
				MCAL_GPIO_Init(GPIOB, &pin_config);
			}

			else if (Global_USARTS_config[2].HW_FlowControl == USART_RTS_Enable)
			{
				// configure RTS pin
				pin_config.Pin_Mode = Alternate_Function_Output_PP;
				pin_config.Pin_Number = GPIO_PIN_14;
				pin_config.Pin_Speed = GPIO_Pin_Max_Speed10M;
				MCAL_GPIO_Init(GPIOB, &pin_config);
			}

			else
			{
				// configure CTS pin
				pin_config.Pin_Mode = Input_Pull_Up;
				pin_config.Pin_Number = GPIO_PIN_13;
				MCAL_GPIO_Init(GPIOB, &pin_config);

				// configure RTS pin
				pin_config.Pin_Mode = Alternate_Function_Output_PP;
				pin_config.Pin_Number = GPIO_PIN_14;
				pin_config.Pin_Speed = GPIO_Pin_Max_Speed10M;
				MCAL_GPIO_Init(GPIOB, &pin_config);
			}
		}

	}

}


//=====================================================================================
//									IRQ HANDLERS
//=====================================================================================

void USART1_IRQHandler(void)
{
	Global_USARTS_config[0].p_callback(USART1->DR);
}

void USART2_IRQHandler(void)
{
	Global_USARTS_config[1].p_callback(USART2->DR);
}

void USART3_IRQHandler(void)
{
	Global_USARTS_config[2].p_callback(USART3->DR);
}
