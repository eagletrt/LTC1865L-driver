/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "eth.h"
#include "usart.h"
#include "spi.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum {
  LTC1865L_DIFF,
  LTC1865L_DIFF_INVERTED,
  LTC1865L_SE_CH1,
  LTC1865L_SE_CH2
} LTC1865L_channel_t;

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

HAL_StatusTypeDef LTC1865L_select_channel(LTC1865L_channel_t c) {
  uint8_t channel_selection_bits[2] = {0x00};
  switch(c) {
  case LTC1865L_SE_CH1:
	  channel_selection_bits[0] = 0xFF;
	  channel_selection_bits[1] = 0xFF;
	  break;
  case LTC1865L_SE_CH2:
	  channel_selection_bits[0] = 0xAA;
	  channel_selection_bits[1] = 0xAA;
	  break;
  case LTC1865L_DIFF:
	  channel_selection_bits[0] = 0x00;
	  channel_selection_bits[1] = 0x00;
	  break;
  case LTC1865L_DIFF_INVERTED:
	  channel_selection_bits[0] = 0x55;
	  channel_selection_bits[1] = 0x55;
	  break;
  default:
	  break;
  }
  HAL_GPIO_WritePin(GPIOA, SPI_CS_Pin, GPIO_PIN_SET);
  HAL_Delay(2);
  HAL_GPIO_WritePin(GPIOA, SPI_CS_Pin, GPIO_PIN_RESET);
  return HAL_SPI_Transmit(&hspi1, channel_selection_bits, 2, 100);
}

uint16_t LTC1865L_read() {
  uint16_t cell_value = 0;
  HAL_GPIO_WritePin(GPIOA, SPI_CS_Pin, GPIO_PIN_SET);
  HAL_Delay(2);
  HAL_GPIO_WritePin(GPIOA, SPI_CS_Pin, GPIO_PIN_RESET);
  if (HAL_SPI_Receive(&hspi1, (uint8_t*) &cell_value, 2, 100) != HAL_OK) {
  	  Error_Handler();
  }
  return cell_value;
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
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_HS_USB_Init();
  MX_LPUART1_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

#define LEN 2048
  char buffer[LEN];
  uint16_t ltc_raw_values[4] = {0};
  long unsigned int converted[4] = {0};

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // READ CHANNEL 1
	  if (LTC1865L_select_channel(LTC1865L_SE_CH1) != HAL_OK) {
		  Error_Handler();
	  }
	  HAL_Delay(2);
	  uint16_t cval = LTC1865L_read();
#define SWAP_BYTES_ENABLED 1
#if SWAP_BYTES_ENABLED == 0
	  ltc_raw_values[0] = cval;
#else
	  ltc_raw_values[0] = (cval << 8) | (cval >> 8);
#endif
	  converted[0] = ltc_raw_values[0] * 2900 / 65536;

	  // READ CHANNEL 2
	  if (LTC1865L_select_channel(LTC1865L_SE_CH2) != HAL_OK) {
		  Error_Handler();
	  }
	  HAL_Delay(2);
	  cval = LTC1865L_read();
#if SWAP_BYTES_ENABLED == 0
	  ltc_raw_values[1] = cval;
#else
	  ltc_raw_values[1] = (cval << 8) | (cval >> 8);
#endif
	  converted[1] = ltc_raw_values[1] * 2900 / 65536;

	  // READ DIFF MODE
	  if (LTC1865L_select_channel(LTC1865L_DIFF) != HAL_OK) {
		  Error_Handler();
	  }
	  HAL_Delay(2);
	  cval = LTC1865L_read();
#if SWAP_BYTES_ENABLED == 0
	  ltc_raw_values[2] = cval;
#else
	  ltc_raw_values[2] = (cval << 8) | (cval >> 8);
#endif
	  converted[2] = ltc_raw_values[2] * 2900 / 65536;

	  // READ DIFF MODE INVERTED
	  if (LTC1865L_select_channel(LTC1865L_DIFF_INVERTED) != HAL_OK) {
		  Error_Handler();
	  }
	  HAL_Delay(2);
	  cval = LTC1865L_read();
#if SWAP_BYTES_ENABLED == 0
	  ltc_raw_values[3] = cval;
#else
	  ltc_raw_values[3] = (cval << 8) | (cval >> 8);
#endif
	  converted[3] = ltc_raw_values[3] * 2900 / 65536;

	  snprintf(buffer, LEN, "ch0: %#x; %lu mV | ch1: %#x; ch1: %lu mV | diff: %#x; ch1: %lu mV | diff inverted: %#x; ch1: %lu mV \n",
			  ltc_raw_values[0], converted[0], ltc_raw_values[1],
			  converted[1], ltc_raw_values[2], converted[2], ltc_raw_values[3], converted[3]);
	  HAL_UART_Transmit(&huart3, (const uint8_t *)buffer, strlen(buffer), 500);
	  HAL_Delay(250);
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 68;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 6144;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
