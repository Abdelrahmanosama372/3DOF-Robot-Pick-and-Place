/*
 * robotManager.c
 *
 *
 *      Author: abdelrahman
 */

#include "robotManager.h"

uint8_t *GP_command;
uint8_t *GP_command_readyToRead;

void RM_USARTCallBack(uint8_t data);
void parse_command(robotManager *rm);

void robotManager_init(robotManager* rm, USART_TypeDef *UART, TIM2_5_TypeDef *TIM){
	// servos initialization
	rm->servos[0].TIM_channel = TIM_CHANNEL1;
	rm->servos[1].TIM_channel = TIM_CHANNEL2;
	rm->servos[2].TIM_channel = TIM_CHANNEL3;
	rm->servos[3].TIM_channel = TIM_CHANNEL4;

	uint8_t i;
	for(i=0; i<4; i++){
		rm->servos[i].TIM = TIM;
		rm->servos[i].curr_angle = 0;
		HAL_Servo_Init(&rm->servos[i]);
	}


	// UART initialization
	USART_config uart_config;
	uart_config.USART_MODE = USART_MODE_RX_TX;
	uart_config.Baud_Rate = USART_Baude_Rate_115200;
	uart_config.Data_Length = USART_Frame_Length_8;
	uart_config.Parity = USART_Frame_parity_DISABLE;
	uart_config.HW_FlowControl = USART_HW_FC_disable;
	uart_config.Stop_Bits = USART_Frame_StopBit_one;
	uart_config.Interrupt_Enable = USART_Received_data_ready;
	uart_config.p_callback = RM_USARTCallBack;

	MCAL_USART_Init(UART, &uart_config);
	MCAL_USART_GPIO_Set_Pins(USART1);

	rm->command_readyToRead = 0;
	GP_command = rm->command;
	GP_command_readyToRead = &rm->command_readyToRead;
}

void robotManager_run(robotManager *rm){
	if(!rm->command_readyToRead)
		return;

	parse_command(rm);

	uint8_t i;
	for(i=0; i<4; i++){
		HAL_Servo_Write(&rm->servos[i], rm->servo_angles[i]);
	}

	// set flag data_readyToRead to 0 to continue reading data form uart
	rm->command_readyToRead = 0;
}


void parse_command(robotManager *rm){
	char *data_ptr = (char*)rm->command;

	rm->servo_angles[0] = (uint8_t)atoi(data_ptr);
	while(*(data_ptr++) != ';');

	rm->servo_angles[1] = (uint8_t)atoi(data_ptr);
	while(*(data_ptr++) != ';');

	rm->servo_angles[2] = (uint8_t)atoi(data_ptr);
	while(*(data_ptr++) != ';');

	rm->servo_angles[3] = (uint8_t)atoi(data_ptr);
}

void RM_USARTCallBack(uint8_t data){

	static int data_index = 0;
	// this handle the case of data receiving while parsing the previous command
	// but this case will not happen as control loop is set to 50HZ in the controller_manager in ROS
	if(*GP_command_readyToRead){
		return;
	}

	if(data == ' '){
		*GP_command_readyToRead = 1;
		data_index = 0;
		return;
	}

	GP_command[data_index++]=data;
}
