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
#include "tim.h"
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
extern bool flag;
extern enum UART_status statuss;
extern uint8_t tester;
extern uint8_t transmit[8];
extern uint8_t receive[9];
extern uint8_t buff[2];
extern uint8_t indx;
extern uint16_t encoder_value;
extern int32_t encoder_rotations;
extern float angle_en;
extern int32_t read_rotation;
extern float angle;
extern int16_t read_error;
extern float angle_err;
bool is_pressed;
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
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  MKS_init();
  MKS_set_rotation_speed_F(94, true);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  if(read_rotation < 400000 && read_rotation > -400000){
//		  //MKS_rotate(18, 15, flag);
//		  MKS_set_rotation_speed_F(10, flag);
//		  //HAL_Delay(10);
//	  }
//	  MKS_read_param(Position_angle, Position_angle_length);
//	  read_rotation = (int32_t)((receive[1] << 24) + (receive[2] << 16) + (receive[3] << 8) + receive[4]);
//	  angle = (float)(read_rotation)/(encoder_quality/one_rotation_in_degrees);
	  MKS_read_param_F(Position_error, Position_error_length);
	  read_error = (int16_t)((receive[1] << 8) + (receive[2]));
	  angle_err = (float)(read_error)/(encoder_quality/one_rotation_in_degrees);
	  MKS_read_param_F(En_value, En_value_length);
	  HAL_Delay(3);
	  encoder_rotations = (int32_t)((receive[1] << 24) + (receive[2] << 16) + (receive[3] << 8) + receive[4]);
	  encoder_value = (uint16_t)((receive[5] << 8) + receive[6]);
	  angle_en = (float)(encoder_value)/(encoder_quality/one_rotation_in_degrees);
//	  if(encoder_rotations >= 1){
//		  flag = true;
//		  if(HAL_GPIO_ReadPin(Button_1_GPIO_Port, Button_1_Pin) == GPIO_PIN_RESET){
//			  MKS_stop_F();
//		  }else{
//			  MKS_set_rotation_speed_F(94, flag);
//		  }
//	  }
//	  if(encoder_rotations <= -1){
//		  flag = false;
//		  if(HAL_GPIO_ReadPin(Button_1_GPIO_Port, Button_1_Pin) == GPIO_PIN_RESET){
//			  MKS_stop_F();
//		  }else{
//		  MKS_set_rotation_speed_F(94, flag);
//		  }
//	  }

	  if(HAL_GPIO_ReadPin(Button_1_GPIO_Port, Button_1_Pin) == GPIO_PIN_RESET){
		  flag = false;
		  MKS_set_rotation_speed_F(94, flag);
	  }else{
		  if(HAL_GPIO_ReadPin(Button_2_GPIO_Port, Button_2_Pin) == GPIO_PIN_RESET){
			  flag = true;
			  MKS_set_rotation_speed_F(94, flag);
		  }else{
			  MKS_stop_F();
		  }
	  }


//	  if(HAL_GPIO_ReadPin(Button_1_GPIO_Port, Button_1_Pin) == GPIO_PIN_RESET){
//		  MKS_stop_F();
//	  }
	  if(encoder_rotations >= 20 || encoder_rotations <= -20){
		  MKS_stop_F();
		  MKS_set_param_F(Enable_move, 0);
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
