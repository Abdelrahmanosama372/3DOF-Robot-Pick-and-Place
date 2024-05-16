################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../stm32f103x6_drivers/stm32f103x6_PWM_driver.c \
../stm32f103x6_drivers/stm32f103x6_RCC_driver.c \
../stm32f103x6_drivers/stm32f103x6_USART_driver.c \
../stm32f103x6_drivers/stm32f103x6_gpio_driver.c 

OBJS += \
./stm32f103x6_drivers/stm32f103x6_PWM_driver.o \
./stm32f103x6_drivers/stm32f103x6_RCC_driver.o \
./stm32f103x6_drivers/stm32f103x6_USART_driver.o \
./stm32f103x6_drivers/stm32f103x6_gpio_driver.o 

C_DEPS += \
./stm32f103x6_drivers/stm32f103x6_PWM_driver.d \
./stm32f103x6_drivers/stm32f103x6_RCC_driver.d \
./stm32f103x6_drivers/stm32f103x6_USART_driver.d \
./stm32f103x6_drivers/stm32f103x6_gpio_driver.d 


# Each subdirectory must supply rules for building sources it contributes
stm32f103x6_drivers/%.o stm32f103x6_drivers/%.su stm32f103x6_drivers/%.cyclo: ../stm32f103x6_drivers/%.c stm32f103x6_drivers/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -c -I../Inc -I"/home/abdelrahman/workspaces/stm/test/stm32f103x6_drivers/Inc" -I"/home/abdelrahman/workspaces/stm/test/HAL/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-stm32f103x6_drivers

clean-stm32f103x6_drivers:
	-$(RM) ./stm32f103x6_drivers/stm32f103x6_PWM_driver.cyclo ./stm32f103x6_drivers/stm32f103x6_PWM_driver.d ./stm32f103x6_drivers/stm32f103x6_PWM_driver.o ./stm32f103x6_drivers/stm32f103x6_PWM_driver.su ./stm32f103x6_drivers/stm32f103x6_RCC_driver.cyclo ./stm32f103x6_drivers/stm32f103x6_RCC_driver.d ./stm32f103x6_drivers/stm32f103x6_RCC_driver.o ./stm32f103x6_drivers/stm32f103x6_RCC_driver.su ./stm32f103x6_drivers/stm32f103x6_USART_driver.cyclo ./stm32f103x6_drivers/stm32f103x6_USART_driver.d ./stm32f103x6_drivers/stm32f103x6_USART_driver.o ./stm32f103x6_drivers/stm32f103x6_USART_driver.su ./stm32f103x6_drivers/stm32f103x6_gpio_driver.cyclo ./stm32f103x6_drivers/stm32f103x6_gpio_driver.d ./stm32f103x6_drivers/stm32f103x6_gpio_driver.o ./stm32f103x6_drivers/stm32f103x6_gpio_driver.su

.PHONY: clean-stm32f103x6_drivers

