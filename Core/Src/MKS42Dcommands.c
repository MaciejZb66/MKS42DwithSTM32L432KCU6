/*
 * MKS42Dcommands.c
 *
 *  Created on: Feb 21, 2024
 *      Author: 4mzr2
 */
#include "MKS42Dcommands.h"


bool flag;
uint16_t one_full_rotation_pulses = (uint16_t)(one_rotation_in_degrees * motor_step / motor_type);
uint8_t data_status;
uint8_t receive_length = 0;
uint8_t transmit[8];
uint8_t receive[9];
uint8_t buff[2];
uint8_t indx;
enum UART_status status;
uint16_t encoder_value = 0;
int32_t encoder_rotations = 0;
float angle_en = 0;
int32_t read_rotation = 0;
float angle = 0;
int16_t read_error = 0;
float angle_err = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	memcpy(receive+indx, buff, 1);
	indx++;
	if(indx >= 9 || indx > receive_length){
		indx = 0;
		status = UART_error;
	}
	if(indx == receive_length){
		indx = 0;
		status = UART_ready;
	}
	HAL_UART_Receive_IT(huart, buff, 1);
}

uint8_t CRC_calc(uint8_t length){
	uint8_t sum = 0;
	for(int i = 0; i < length; i++){
		sum += transmit[i];
	}
	return sum;
}

void MKS_init(void){
	  HAL_UART_Init(&huart1);
	  flag = true;
	  indx = 0;
	  HAL_UART_Receive_IT(&huart1, buff, 1);
	  do{ }while(status != UART_ready);
	  MKS_set_param(Enable_move, 0x01);
}

void MKS_read_param(uint8_t param, uint8_t length_of_param){
	if(status == UART_ready){
		status = UART_processing;
		transmit[0] = Address;
		transmit[1] = param;
		transmit[2] = CRC_calc(2);
		receive_length = length_of_param;
		HAL_UART_Transmit_IT(Used_UART, transmit, 3);
	}else{
		status = UART_busy;
	}
}


void MKS_set_param(uint8_t param, uint8_t value){
	if(status == UART_ready){
		status = UART_processing;
		transmit[0] = Address;
		transmit[1] = param;
		transmit[2] = value;
		transmit[3] = CRC_calc(3);
		receive_length = response_length;
		HAL_UART_Transmit_IT(Used_UART, transmit, 4);
	}else{
		status = UART_busy;
	}
}

void MKS_rotate(uint16_t rot, uint8_t speed, bool clockwise){
	if(status == UART_ready){
		status = UART_processing;
		uint32_t pulses;
		if(clockwise){
			speed &= 0x7F;
		}else{
			speed |= 0x80;
		}
		pulses = rot * one_full_rotation_pulses / one_rotation_in_degrees;
		transmit[0] = Address;
		transmit[1] = Rotate;
		transmit[2] = (uint8_t)speed;
		transmit[3] = (uint8_t)(pulses >> 24);
		transmit[4] = (uint8_t)(pulses >> 16);
		transmit[5] = (uint8_t)(pulses >> 8);
		transmit[6] = (uint8_t)(pulses);
		transmit[7] = CRC_calc(7);
		receive_length = response_length;
		HAL_UART_Transmit_IT(Used_UART, transmit, 8);
	}else{
		status = UART_busy;
	}
}

void MKS_set_rotation_speed(uint8_t speed, bool clockwise){
	if(status == UART_ready){
		status = UART_processing;
		if(clockwise){
			speed &= 0x7F;
		}else{
			speed |= 0x80;
		}
		transmit[0] = Address;
		transmit[1] = Set_rotation;
		transmit[2] = (uint8_t)speed;
		transmit[3] = CRC_calc(3);
		receive_length = response_length;
		HAL_UART_Transmit_IT(Used_UART, transmit, 4);
	}else{
		status = UART_busy;
	}
}

void MKS_stop(void){
	if(status == UART_ready){
		status = UART_processing;
		transmit[0] = Address;
		transmit[1] = Stop;
		transmit[2] = CRC_calc(2);
		receive_length = response_length;
		HAL_UART_Transmit_IT(Used_UART, transmit, 3);
	}else{
		status = UART_busy;
	}
}

struct Encoder MKS_get_encoder_value(void){
	struct Encoder En;
	MKS_read_param(En_value, En_value_length);
	En.encoder_rotations = (int32_t)((receive[1] << 24) + (receive[2] << 16) + (receive[3] << 8) + receive[4]);
	En.encoder_value = (uint16_t)((receive[5] << 8) + receive[6]);
	En.encoder_angle = (float)(encoder_value)/(encoder_quality/one_rotation_in_degrees);
	return En;
}
