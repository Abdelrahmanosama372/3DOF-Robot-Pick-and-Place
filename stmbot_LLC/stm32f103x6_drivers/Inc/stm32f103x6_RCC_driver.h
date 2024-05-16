/*
 * stm32f103x6_RCC_driver.h
 *
 *      Author: Abdelrahman Osama
 */

/**
 * @file stm32f103x6_RCC_driver.h
 * @brief Partial implementation of RCC driver for STM32F103x6.
 * @note Assumes HSI is the running clock.
 */

#ifndef INC_STM32F103X6_RCC_DRIVER_H_
#define INC_STM32F103X6_RCC_DRIVER_H_

#include "stm32f103x6.h"

/** @brief HSI clock enable bit mask. */
#define RCC_HSI_clock       (uint32_t)(1 << 0)  // HSION: Internal high-speed clock enable

/**
 * @brief Retrieves the system clock frequency.
 *
 * @param _RCC Pointer to the RCC_TypeDef structure that contains the configuration information for the RCC peripheral.
 * @return The system clock frequency in Hz.
 */
uint32_t MCAL_RCC_get_SYSCLK(RCC_TypeDef *_RCC);

/**
 * @brief Retrieves the AHB clock frequency (HCLK).
 *
 * @param _RCC Pointer to the RCC_TypeDef structure that contains the configuration information for the RCC peripheral.
 * @return The AHB clock frequency in Hz.
 */
uint32_t MCAL_RCC_get_HCLK(RCC_TypeDef *_RCC);

/**
 * @brief Retrieves the APB1 peripheral clock frequency (PCLK1).
 *
 * @param _RCC Pointer to the RCC_TypeDef structure that contains the configuration information for the RCC peripheral.
 * @return The APB1 peripheral clock frequency in Hz.
 */
uint32_t MCAL_RCC_get_PCLK1(RCC_TypeDef *_RCC);

/**
 * @brief Retrieves the APB2 peripheral clock frequency (PCLK2).
 *
 * @param _RCC Pointer to the RCC_TypeDef structure that contains the configuration information for the RCC peripheral.
 * @return The APB2 peripheral clock frequency in Hz.
 */
uint32_t MCAL_RCC_get_PCLK2(RCC_TypeDef *_RCC);

#endif /* INC_STM32F103X6_RCC_DRIVER_H
