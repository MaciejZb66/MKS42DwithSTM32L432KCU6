/*
 * MKS42Dcommands.c
 *
 *  Created on: Feb 21, 2024
 *      Author: 4mzr2
 */
#include "MKS42Dcommands.h"


bool flag;
bool is_uart_busy;
uint8_t receive_length = 0;
uint8_t transmit[8];
uint8_t receive[9];
uint8_t buff[2];
uint8_t indx;
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
	}
	if(indx == receive_length){
		indx = 0;
		is_uart_busy = false;
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


void MKS_read_param(uint8_t param, uint8_t length_of_param){
	do{
		if(is_uart_busy == false){
			transmit[0] = Address;
			transmit[1] = param;
			transmit[2] = CRC_calc(2);
			receive_length = length_of_param;
			is_uart_busy = true;
			HAL_UART_Transmit_IT(Used_UART, transmit, 3);
		}
	}while(is_uart_busy);
}


void MKS_set_param(uint8_t param, uint8_t value){
	do{
		if(is_uart_busy == false){
			transmit[0] = Address;
			transmit[1] = param;
			transmit[2] = value;
			transmit[3] = CRC_calc(3);
			receive_length = response_length;
			is_uart_busy = true;
			HAL_UART_Transmit_IT(Used_UART, transmit, 4);
		}
	}while(is_uart_busy);


}

void MKS_rotate(uint16_t rot, uint8_t speed, bool clockwise){
	do{
		if(is_uart_busy == false){
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
			is_uart_busy = true;
			HAL_UART_Transmit_IT(Used_UART, transmit, 8);
		}
	}while(is_uart_busy);
}

void MKS_set_rotation_speed(uint8_t speed, bool clockwise){
	do{
		if(is_uart_busy == false){
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
			is_uart_busy = true;
			HAL_UART_Transmit_IT(Used_UART, transmit, 4);
		}
	}while(is_uart_busy);
}

void MKS_stop(void){
	do{
		if(is_uart_busy == false){
			transmit[0] = Address;
			transmit[1] = Stop;
			transmit[2] = CRC_calc(2);
			receive_length = response_length;
			is_uart_busy = true;
			HAL_UART_Transmit_IT(Used_UART, transmit, 3);
		}
	}while(is_uart_busy == false);

}

