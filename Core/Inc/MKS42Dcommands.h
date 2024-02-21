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
#include "usart.h"
#include "string.h"

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
#define one_full_rotation_pulses 3200
#define one_rotation_in_degrees 360.0f
#define encoder_quality (float)(1<<16)


//prototypes of functions

uint8_t CRC_calc(uint8_t length);
void MKS_read_param(uint8_t param, uint8_t length_of_param);
void MKS_set_param(uint8_t param, uint8_t value);
void MKS_rotate(uint16_t rot, uint8_t speed, bool clockwise);
void MKS_set_rotation_speed(uint8_t speed, bool clockwise);
void MKS_stop(void);
