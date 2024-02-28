/*
 * MKS42Dcommands.h
 *
 *  Created on: Feb 21, 2024
 *      Author: 4mzr2
 */
#pragma once
//includes
#include "stm32l4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>
#include "tim.h"
#include "usart.h"
#include "string.h"

/*
 * WARNING! MKS servo 42D have open drain TX data line
 * Add 2k resistor between 3v3 and MKS TX - STM RX data line
*/
//customizable defines
#define Used_UART &huart1
#define Timer_timeout
#ifdef Timer_timeout
	#define Used_timer &htim6
#endif

//used defines
#define Address 0xE0
#define En_value 0x30
#define En_value_length 8
#define Position_angle 0x36
#define Position_angle_length 6
#define Position_error 0x39
#define Position_error_length 4
#define Enable_move 0xF3
#define Set_rotation 0xF6
#define Stop 0xF7
#define Rotate 0xFD
#define response_length 3
#define motor_step 16.0f
#define motor_type 1.8f
#define one_rotation_in_degrees 360.0f
#define encoder_quality (float)(1<<16)
//define enum type
enum UART_status {
	UART_ready = 0, UART_processing = 1, UART_busy = 2, UART_error = 3
};

//used structs
struct Encoder{
	uint16_t encoder_value;
	int32_t encoder_rotations;
	float encoder_angle;
};

//prototypes of functions
uint8_t CRC_calc(uint8_t length);
void MKS_UART_wait(void);
void MKS_init(void);
void MKS_read_param(uint8_t param, uint8_t length_of_param);
void MKS_read_param_F(uint8_t param, uint8_t length_of_param);
void MKS_set_param(uint8_t param, uint8_t value);
void MKS_set_param_F(uint8_t param, uint8_t value);
void MKS_rotate(uint16_t rot, uint8_t speed, bool clockwise);
void MKS_rotate_F(uint16_t rot, uint8_t speed, bool clockwise);
void MKS_set_rotation_speed(uint8_t speed, bool clockwise);
void MKS_set_rotation_speed_F(uint8_t speed, bool clockwise);
void MKS_stop(void);
void MKS_stop_F(void);
struct Encoder MKS_get_encoder_value(void);

