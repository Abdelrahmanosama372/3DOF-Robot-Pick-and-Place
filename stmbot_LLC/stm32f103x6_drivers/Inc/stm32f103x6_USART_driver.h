/*
 * stm32f103x6_USART_driver.h
 *
 *
 *      Author: Abdelrahman osama
 */

#ifndef INC_STM32F103X6_USART_DRIVER_H_
#define INC_STM32F103X6_USART_DRIVER_H_

#include "stm32f103x6.h"
#include "stm32f103x6_gpio_driver.h"
#include "stm32f103x6_RCC_driver.h"


// USART baudrate register value calculation
#define USARTDIV(_PCLK_,baudrate)     				  (_PCLK_/(16.0*baudrate))
#define USARTDIV_mul_100(_PCLK_,baudrate) 			  ((uint32_t) ((USARTDIV(_PCLK_,baudrate)*100)))
#define USARTDIV_Mantissa(_PCLK_,baudrate)    		  ((uint32_t) USARTDIV(_PCLK_,baudrate))
#define USARTDIV_Mantissa_mul_100(_PCLK_,baudrate)    ((uint32_t) USARTDIV_Mantissa(_PCLK_,baudrate) *100)
#define USARTDIV_Fraction(_PCLK_,baudrate)    		  (uint32_t)  (((USARTDIV_mul_100(_PCLK_,baudrate) - USARTDIV_Mantissa_mul_100(_PCLK_,baudrate))*16)/100)
#define USART_BRR_val(_PCLK_,baudrate) 				  (((USARTDIV_Mantissa(_PCLK_,baudrate) & 0xFFF)  << 4 ) | (USARTDIV_Fraction(_PCLK_,baudrate) & 0xF))

typedef struct
{

	uint8_t USART_MODE;                // specifies whether TX or RX or both are used
									   // this parameter must be set based on @ref USART_MODES

	uint32_t Data_Length;               // specifies whether data length is 8 or 9 bits
									   // this parameter must be set based on @ref Data_Lenghts

	uint32_t Stop_Bits;                 // specifies number of stop bits
									   // this parameter must be set based on @ref Frame_Stop_Bits

	uint32_t Parity;                    // specifies odd or even parity if enabled also specifies parity enable/disable state
									   // this parameter must be set based on @ref Frame_Parity

	uint32_t Baud_Rate;                 // specifies the value of the baud rate
									   // this parameter must be set based on @ref Baud_Rates

	uint16_t HW_FlowControl;			// specifies the usage of RTS and CTS pins
										// this parameter must be set based on @ref USART_HW_FlowControl

	uint32_t Interrupt_Enable;          // specifies whether IRQ is enabled for the USART
									   // this parameter must be set based on @ref USART_IRQs

	void (*p_callback)(uint8_t data);          // specifies a pointer to call back function on IRQ


}USART_config;



//================================
//		Reference macros
//================================

// USART_MODES

#define USART_MODE_RX            				(uint32_t)(1 << 2)
#define USART_MODE_TX            				(uint32_t)(1 << 3)
#define USART_MODE_RX_TX         				(uint32_t)((1 << 2) | ( 1 << 3))

// Data_Lenghts

#define USART_Frame_Length_8     				0
#define USART_Frame_Length_9     				(uint32_t)(1 << 12)

// Frame_Stop_Bits

#define USART_Frame_StopBit_half				(uint32_t)(1 << 12)
#define USART_Frame_StopBit_one					(uint32_t)(0)
#define USART_Frame_StopBit_one_half			(uint32_t)(3 << 12)
#define USART_Frame_StopBit_two					(uint32_t)(2 << 12)

// Frame_Parity

#define USART_Frame_EVEN_parity_ENABLE          (uint32_t)(1 << 10)
#define USART_Frame_ODD_parity_ENABLE			(uint32_t)(1 << 9 || 1 << 10)
#define USART_Frame_parity_DISABLE				(uint32_t)(0)

// Baud_Rates
#define USART_Baude_Rate_2400               2400
#define USART_Baude_Rate_9600     		    9600
#define USART_Baude_Rate_19200              19200
#define USART_Baude_Rate_57600              57600
#define USART_Baude_Rate_115200            	115200
#define USART_Baude_Rate_230400 			230400
#define USART_Baude_Rate_460800     		460800
#define USART_Baude_Rate_921600        		921600
#define USART_Baude_Rate_2250000    		2250000
#define USART_Baude_Rate_4500000 			4500000

// USART_HW_FlowControl
#define USART_HW_FC_disable					(uint32_t)(0)
#define USART_CTS_Enable                    (uint32_t)(1 << 9)
#define USART_RTS_Enable                    (uint32_t)(1 << 8)
#define USART_CTS_RTS_Enable 				(uint32_t)(1 << 9 || 1 << 8)

// USART_IRQs
#define USART_Interrupt_disable               (uint32_t)(0)
#define USART_Transmit_DR_empty               (uint32_t)(1 << 7)            // empty data register // Bit 7 TXEIE: TXE interrupt enable
#define USART_Transmission_complete			  (uint32_t)(1 << 6)			// Bit 6 TCIE: Transmission complete interrupt enable
#define USART_Received_data_ready 			  (uint32_t)(1 << 5)			// Bit 5 RXNEIE: RXNE interrupt enable
#define USART_Overrun_error 				  (uint32_t)(1 << 5)			// Bit 5 RXNEIE: RXNE interrupt enable
#define USART_Parity_error                    (uint32_t)(1 << 8)            // Bit 8 PEIE: PE interrupt enable

typedef enum
{
	Polling,
	Interrupt
}Data_Handling;

//=====================================================================================
//									APIs Prototypes
//=====================================================================================

/**
 * @brief Initializes the specified USART peripheral.
 *
 * @param USARTx Pointer to the USART peripheral (USART1, USART2, USART3).
 * @param USART_config Pointer to a USART_config structure that contains the configuration information for the specified USART peripheral.
 *
 * @retval None
 */
void MCAL_USART_Init(USART_TypeDef *USARTx, USART_config *USART_config);

/**
 * @brief Deinitializes the specified USART peripheral.
 *
 * @param USARTx Pointer to the USART peripheral (USART1, USART2, USART3).
 *
 * @retval None
 */
void MCAL_USART_Deinit(USART_TypeDef *USARTx);

/**
 * @brief Transmits data through the specified USART peripheral.
 *
 * @param USARTx Pointer to the USART peripheral (USART1, USART2, USART3).
 * @param data Pointer to the data to be transmitted.
 * @param data_len Length of the data to be transmitted.
 * @param Handler_mech Specifies whether to use polling or interrupt-based handling.
 *
 * @retval None
 */
void MCAL_USART_TX(USART_TypeDef *USARTx, uint8_t *data, uint32_t data_len, Data_Handling Handler_mech);

/**
 * @brief Receives data from the specified USART peripheral.
 *
 * @param USARTx Pointer to the USART peripheral (USART1, USART2, USART3).
 * @param data Pointer to the buffer where received data will be stored.
 * @param data_len Length of the data to be received.
 * @param Handler_mech Specifies whether to use polling or interrupt-based handling.
 *
 * @retval None
 */
void MCAL_USART_RX(USART_TypeDef *USARTx, uint16_t *data, uint32_t data_len, Data_Handling Handler_mech);

/**
 * @brief Configures the GPIO pins for the specified USART peripheral.
 *
 * @param USARTx Pointer to the USART peripheral (USART1, USART2, USART3).
 *
 * @retval None
 */
void MCAL_USART_GPIO_Set_Pins(USART_TypeDef *USARTx);





#endif /* INC_STM32F103X6_USART_DRIVER_H_ */
