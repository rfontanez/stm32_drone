/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "BNO080.h"
#include "Quaternion.h"
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//implement _write function so that printf will work
//takes the pointer location for the first char in the string to print
int _write(int file, char* p, int len)
{
	for (int i = 0; i < len; i++)
	{
		// Wait until transmit data register is empty
		while (!LL_USART_IsActiveFlag_TXE(USART6)) {}
		//adds to the initial pointer location for every char in string
		LL_USART_TransmitData8(USART6, *(p+i));
//		HAL_Delay(1);
	}
	return len;
}

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//	flag to know if we've got data - initialized in stm32f4xx_it.c
	extern uint8_t uart6_rx_flag;
//	variable to store the received data
	extern uint8_t uart6_rx_data;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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

//  int temp = 0;
  float flt = 8.000f;
  float q[4];
  float quatRadianAccuracy;


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
  MX_TIM3_Init();
  MX_USART6_UART_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  LL_TIM_EnableCounter(TIM3);
  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);

  TIM3->PSC = 2000;
  HAL_Delay(100);
  TIM3->PSC = 1500;
  HAL_Delay(100);
  TIM3->PSC = 1000;
  HAL_Delay(100);

  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);

  LL_USART_EnableIT_RXNE(USART6);

  //initialize BNO080 - includes SPI2 and GPIO configurations
  BNO080_Initialization();
  //Set output to be rotation vector and rate to 2500ms which is the max 400Hz output rate
  BNO080_enableRotationVector(2500);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);

	    // Wait until transmit data register is empty
//	  while (!LL_USART_IsActiveFlag_TXE(USART6)) {}

//	  LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2);

//	  LL_USART_TransmitData8(USART6, 'A');
//
//	  // Optionally send newline
//	  while (!LL_USART_IsActiveFlag_TXE(USART6)) {}
//	  LL_USART_TransmitData8(USART6, '\r');
//
//	  while (!LL_USART_IsActiveFlag_TXE(USART6)) {}
//	  LL_USART_TransmitData8(USART6, '\n');
//
//	  if (uart6_rx_flag == 1) {
//
//		  //reset the flag that indicates received data
//		  uart6_rx_flag = 0;
//
//		  //send it back to PC to confirm (basically just to debug)
////		  LL_USART_TransmitData8(USART6, 'r');
////		  LL_USART_TransmitData8(USART6, ':');
////		  while (!LL_USART_IsActiveFlag_TXE(USART6)) {}
////		  LL_USART_TransmitData8(USART6, '\n');
////		  while (!LL_USART_IsActiveFlag_TXE(USART6)) {}
////		  LL_USART_TransmitData8(USART6, uart6_rx_data);

//		  switch(uart6_rx_data)
//		  {
//		  case '0': HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2); break;
//		  case '1': LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4); break;
//		  case '2': LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4); break;
//		  default: break;
//		  }
//	  }
//
//	  printf("%f\n", flt+=0.001);
//	  HAL_Delay(1000);

	  //check if BNO080 has data for us
	  if (BNO080_dataAvailable() == 1) {
//		  //store the raw values of the quaternion
		  q[0] = BNO080_getQuatI();
		  q[1] = BNO080_getQuatJ();
		  q[2] = BNO080_getQuatK();
		  q[3] = BNO080_getQuatReal();

//		  //store the accuracy - not sure why but spark fun does
		  quatRadianAccuracy = BNO080_getQuatRadianAccuracy();

//		  //send raw values to be turned into Euler angles
//		  //this stores the roll, pitch, and yaw values globally
		  Quaternion_Update(&q[0]);

//		  //print them as ints to avoid printf float errors
		  printf("R:%d, P:%d, Y:%d\n", (int)(BNO080_Roll*100), (int)(BNO080_Pitch*100), (int)(BNO080_Yaw*100));

	  }



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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
#ifdef USE_FULL_ASSERT
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
