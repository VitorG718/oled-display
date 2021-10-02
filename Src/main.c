/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SSD1331.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	uint8_t red;
	uint8_t green;
	uint8_t blue;
}ColorTypedef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define OP_COMMAND 				0U
#define OP_DATA						1U
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
void write_cmd(uint8_t data) {
	HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	
	HAL_SPI_Transmit_IT(&hspi1, &data, 1);
	
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
}

void DSP_Init() {
	HAL_GPIO_WritePin(RES_GPIO_Port, RES_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);	
	HAL_GPIO_WritePin(RES_GPIO_Port, RES_Pin, GPIO_PIN_SET);
/*	
	write_cmd(0xA4);
	write_cmd(0xAF);
	write_cmd(0xA1);
	write_cmd(0x00);
	write_cmd(0xA0);
	write_cmd(0x7A);
	*/
	SSD1331_init();
}

void drawLine(uint8_t c1, uint8_t r1, uint8_t c2, uint8_t r2, ColorTypedef color) {
	write_cmd(0x21);
	write_cmd(c1);
	write_cmd(r1);
	write_cmd(c2);
	write_cmd(r2);
	write_cmd(color.red);
	write_cmd(color.green);
	write_cmd(color.green);
}

void drawFrame(uint8_t c1, uint8_t r1, uint8_t c2, uint8_t r2, ColorTypedef border, ColorTypedef fill) {
	write_cmd(0x26);
	write_cmd(0x01);
	write_cmd(0x22);
	write_cmd(c1);
	write_cmd(r1);
	write_cmd(c2);
	write_cmd(r2);
	write_cmd(border.red);
	write_cmd(border.green);
	write_cmd(border.blue);
	write_cmd(fill.red);
	write_cmd(fill.green);
	write_cmd(fill.blue);
}

void clear(void) {
	ColorTypedef black = {0, 0, 0};
	drawFrame(0,0,95,63, black, black);
  HAL_Delay(1);
}

void wait_and_clear(void) {
  HAL_Delay(1000);
	clear();  
}

void drawMyName() {
	ColorTypedef blue = {0,50,0};
	
	// V
	drawLine(2, 2, 6, 2, blue);
	drawLine(2, 2, 10, 20, blue);
	drawLine(10, 20, 14, 20, blue);
	drawLine(22, 2, 14, 20, blue);
	drawLine(18, 2, 22, 2, blue);
	drawLine(6, 2, 12, 16, blue);
	drawLine(12, 16, 18, 2, blue);
	
	
	// I
	drawLine(26, 2, 30, 2, blue);
	drawLine(26, 2, 26, 20, blue);
	drawLine(26, 20, 30, 20, blue);
	drawLine(30, 2, 30, 20, blue);
	
	// T
	drawLine(34, 2, 34, 6, blue);
	drawLine(34, 2, 54, 2, blue);
	drawLine(54, 2, 54, 6, blue);
	drawLine(42, 20, 46, 20, blue);
	drawLine(34, 6, 42, 6, blue);
	drawLine(46, 6, 54, 6, blue);
	drawLine(42, 6, 42, 20, blue);
	drawLine(46, 6, 46, 20, blue);
	
	// O
	drawLine(58, 2, 58, 20, blue);
	drawLine(62, 6, 62, 16, blue);
	drawLine(62, 6, 66, 6, blue);
	drawLine(66, 6, 66, 16, blue);
	drawLine(62, 16, 66, 16, blue);
	drawLine(70, 2, 70, 20, blue);
	drawLine(58, 2, 70, 2, blue);
	drawLine(58, 20, 70, 20, blue);
	
	// R
	drawLine(74, 2, 94, 2, blue);
	drawLine(74, 2, 74, 20, blue);
	drawLine(94, 2, 94, 14, blue);
	drawLine(74, 20, 78, 20, blue);
	drawLine(78, 20, 78, 14, blue);
	drawLine(90, 20, 94, 20, blue);
	drawLine(78, 14, 82, 14, blue);
	drawLine(90, 20, 82, 14, blue);
	drawLine(94, 20, 88, 14, blue);
	drawLine(88, 14, 94, 14, blue);
	drawLine(78, 6, 90, 6, blue);
	drawLine(78, 6, 78, 10, blue);
	drawLine(90, 6, 90, 10, blue);
	drawLine(78, 10, 90, 10, blue);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	DSP_Init();
	clear();

/*
	ColorTypedef red = {50,0,0};
	ColorTypedef blue = {0,50,0};
	drawFrame(0,0,96,64, red, blue);
	wait_and_clear();	
*/

	drawMyName();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
