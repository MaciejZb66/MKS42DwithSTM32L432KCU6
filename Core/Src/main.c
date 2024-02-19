/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Address 0xE0
#define En_value 0x30
#define En_value_length 9
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
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
bool flag;
uint8_t tester;
uint8_t transmit[8];
uint8_t receive[8];
uint8_t buff[2];
uint8_t indx;
uint16_t encoder_value = 0;
int32_t encoder_rotations = 0;
float angle_en=0;
int32_t read_rotation = 0;
float angle = 0;
int16_t read_error = 0;
float angle_err = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	memcpy(receive+indx, buff, 1);
	if(++indx >= 8){
		indx = 0;
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
	transmit[0] = Address;
	transmit[1] = param;
	transmit[2] = CRC_calc(2);
	HAL_UART_Transmit_IT(&huart1, transmit, 3);
	do{
		if(indx > length_of_param){
			break;
		}
	}while(indx != length_of_param - 1);
	indx = 0;
	//HAL_UART_Receive_IT(&huart1, receive, length_of_param);
	HAL_Delay(10);
}

//void MKS_check_read_param(uint8_t param, uint8_t length_of_param){
//	transmit[0] = Address;
//	transmit[1] = param;
//	transmit[2] = CRC_calc(2);
//	do{
//		HAL_UART_Transmit_IT(&huart1, transmit, 3);
//		HAL_UART_Receive_IT(&huart1, receive, length_of_param);
//	}while(receive[length_of_param] != CRC_calc(length_of_param));
//}

void MKS_set_param(uint8_t param, uint8_t value){
	transmit[0] = Address;
	transmit[1] = param;
	transmit[2] = value;
	transmit[3] = CRC_calc(3);
	HAL_UART_Transmit_IT(&huart1, transmit, 4);
	do{
		if(indx > response_length){
			break;
		}
	}while(indx != response_length);
	indx = 0;
	//HAL_UART_Receive_IT(&huart1, receive, 3);
}

void MKS_rotate(uint16_t rot, uint8_t speed, bool clockwise){
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
	HAL_UART_Transmit_IT(&huart1, transmit, 8);
	do{
		if(indx > response_length){
			break;
		}
	}while(indx != response_length - 1);
	indx = 0;
	//HAL_UART_Receive_IT(&huart1, receive, 3);
}

void MKS_set_rotation_speed(uint8_t speed, bool clockwise){
	if(clockwise){
		speed &= 0x7F;
	}else{
		speed |= 0x80;
	}
	transmit[0] = Address;
	transmit[1] = Set_rotation;
	transmit[2] = (uint8_t)speed;
	transmit[3] = CRC_calc(3);
	HAL_UART_Transmit_IT(&huart1, transmit, 4);
	do{
		if(indx > response_length){
			break;
		}
	}while(indx != response_length - 1);
	indx = 0;
}

void MKS_stop(void){
	transmit[0] = Address;
	transmit[1] = Stop;
	transmit[2] = CRC_calc(2);
	HAL_UART_Transmit_IT(&huart1, transmit, 3);
	do{
		if(indx > response_length){
			break;
		}
	}while(indx != response_length - 1);
	indx = 0;
	//HAL_UART_Receive_IT(&huart1, receive, 3);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Init(&huart1);
  flag = true;
  indx = 0;
  HAL_UART_Receive_IT(&huart1, buff, 1);
  MKS_set_param(0x90, 0x02);

  MKS_set_param(Enable_move, 0x01);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(read_rotation < 400000 && read_rotation > -400000){
		  //MKS_rotate(18, 15, flag);
		  MKS_set_rotation_speed(10, flag);
	  }
	  HAL_Delay(100);
	  MKS_read_param(Position_angle, Position_angle_length);
	  MKS_read_param(Position_angle, Position_angle_length);
	  read_rotation = (int32_t)((receive[1] << 24) + (receive[2] << 16) + (receive[3] << 8) + receive[4]);
	  angle = (float)(read_rotation)/(encoder_quality/one_rotation_in_degrees);
	  HAL_Delay(10);
	  MKS_read_param(Position_error, Position_error_length);
	  MKS_read_param(Position_error, Position_error_length);
	  read_error = (int16_t)((receive[1] << 8) + (receive[2]));
	  angle_err = (float)(read_error)/(encoder_quality/one_rotation_in_degrees);
	  HAL_Delay(10);
	  MKS_read_param(En_value, En_value_length);
	  MKS_read_param(En_value, En_value_length);
	  encoder_rotations = (int32_t)((receive[1] << 24) + (receive[2] << 16) + (receive[3] << 8) + receive[4]);
	  encoder_value = (uint16_t)((receive[5] << 8) + receive[6]);
	  angle_en = (float)(encoder_value)/(encoder_quality/one_rotation_in_degrees);
	  tester = 0;
	  HAL_Delay(400);

//	  if(angle > 10){
//		  flag = true;
//	  }
//	  if(angle < -180){
//		  flag = false;
//	  }

	  if(encoder_rotations >= 1){
		  flag = true;
		  MKS_stop();
	  }
	  if(encoder_rotations <= -1){
		  flag = false;
		  MKS_stop();
	  }
	  if(read_rotation > 800000 || read_rotation < -800000){
		  MKS_stop();
		  MKS_set_param(Enable_move, 0);
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
