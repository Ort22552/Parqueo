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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ili9341.h"
#include <stdbool.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ESP32_I2C_ADDRESS 0x08
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
extern uint8_t parqueo[];
extern uint8_t carros[];
extern uint8_t automovil[];

char espacios[8];

char dataparqueos;

uint8_t espacioslibres =4;
uint8_t clicp =0;

bool carro1=false;
bool carro2=false;
bool carro3=false;
bool carro4=false;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void segmentos(uint8_t num);
void mandari2c(void);
//void traduccioncontrol(void);
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

	LCD_Init();

	LCD_Clear(0x00);
	LCD_Bitmap(0, 0, 320, 240, parqueo);
	itoa(espacioslibres,espacios,8);
	LCD_Print(espacios, 265, 85,2,0xffff, 0x0000);
	//FillRect(0, 0, 319, 239, 0xFF800);
	//FillRect(50, 60, 20, 20, 0xF800);
	//FillRect(70, 60, 20, 20, 0x07E0);
	//FillRect(90, 60, 20, 20, 0x001F);
	//LCD_Bitmap(0, 0, 320, 240, fondo);

	/*LCD_Bitmap(0, 0, 320, 240, inicio);
	LCD_Print("Press X", 120, 140, 1, 0xffff, 0x0000);
	LCD_Print("Press O", 120, 155, 1, 0xffff, 0x0000);*/

	//LCD_Sprite(100, 100, 15, 15, mira, 2, 0, 0, 0);
	//LCD_Sprite(pato1x, pato1y, 35, 37, pato, 2, 0, 0, 0);

	// FillRect(0, 0, 319, 206, 0x1911);
	//HAL_ADC_Start_DMA(&hadc1, (uint16_t*) ADCv, 1);

	//HAL_UART_Receive_DMA(&huart3, rx_buffer, 8);

	/*
	HAL_Delay(1);
	itoa(ADCv, vladc,16);
	LCD_Print(vladc, 30, 230, 1, 0x001F, 0xCAB9);*/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		itoa(espacioslibres,espacios,8);
		LCD_Print(espacios, 280,185,2,0xffff, 0x0000);
		segmentos(espacioslibres);

		if (carro1==true) {
			LCD_Bitmap(2, 1, 62, 150, automovil);
			HAL_GPIO_WritePin(P1G_GPIO_Port, P1G_Pin, 1);
			HAL_GPIO_WritePin(P1R_GPIO_Port, P1R_Pin, 0);
		} else {
			FillRect(2, 1, 62, 150, 0x0000);
			HAL_GPIO_WritePin(P1R_GPIO_Port, P1R_Pin, 1);
			HAL_GPIO_WritePin(P1G_GPIO_Port, P1G_Pin, 0);
		}
		if (carro2==true) {
			LCD_Bitmap(82, 1, 62, 150, automovil);
			HAL_GPIO_WritePin(P2G_GPIO_Port, P2G_Pin, 1);
			HAL_GPIO_WritePin(P2R_GPIO_Port, P2R_Pin, 0);
		} else {
			FillRect(82, 1, 62, 150, 0x0000);
			HAL_GPIO_WritePin(P2R_GPIO_Port, P2R_Pin, 1);
			HAL_GPIO_WritePin(P2G_GPIO_Port, P2G_Pin, 0);
		}
		if (carro3==true) {
			LCD_Bitmap(162, 1, 62, 150, automovil);
			HAL_GPIO_WritePin(P3G_GPIO_Port, P3G_Pin, 1);
			HAL_GPIO_WritePin(P3R_GPIO_Port, P3R_Pin, 0);
		} else {
			FillRect(162, 1, 62, 150, 0x0000);
			HAL_GPIO_WritePin(P3R_GPIO_Port, P3R_Pin, 1);
			HAL_GPIO_WritePin(P3G_GPIO_Port, P3G_Pin, 0);
		}
		if (carro4==true) {
			LCD_Bitmap(242, 1, 62, 150, automovil);
			HAL_GPIO_WritePin(P4G_GPIO_Port, P4G_Pin, 1);
			HAL_GPIO_WritePin(P4R_GPIO_Port, P4R_Pin, 0);
		} else {
			FillRect(242, 1, 62, 150, 0x0000);
			HAL_GPIO_WritePin(P4R_GPIO_Port, P4R_Pin, 1);
			HAL_GPIO_WritePin(P4G_GPIO_Port, P4G_Pin, 0);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, P3R_Pin|P3G_Pin|LCD_RST_Pin|P4R_Pin
                          |P4G_Pin|LCD_D1_Pin|P1R_Pin|P1G_Pin
                          |P2R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_RD_Pin|LCD_WR_Pin|LCD_RS_Pin|LCD_D7_Pin
                          |LCD_D0_Pin|LCD_D2_Pin|SegB_Pin|SegA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_CS_Pin|SegE_Pin|SegD_Pin|LCD_D6_Pin
                          |SegC_Pin|SegG_Pin|SegF_Pin|LCD_D3_Pin
                          |LCD_D5_Pin|LCD_D4_Pin|SD_SS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(P2G_GPIO_Port, P2G_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : P3R_Pin P3G_Pin P4R_Pin P4G_Pin
                           P1R_Pin P1G_Pin P2R_Pin */
  GPIO_InitStruct.Pin = P3R_Pin|P3G_Pin|P4R_Pin|P4G_Pin
                          |P1R_Pin|P1G_Pin|P2R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RST_Pin LCD_D1_Pin */
  GPIO_InitStruct.Pin = LCD_RST_Pin|LCD_D1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RD_Pin LCD_WR_Pin LCD_RS_Pin LCD_D7_Pin
                           LCD_D0_Pin */
  GPIO_InitStruct.Pin = LCD_RD_Pin|LCD_WR_Pin|LCD_RS_Pin|LCD_D7_Pin
                          |LCD_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Parqueo4_Pin */
  GPIO_InitStruct.Pin = Parqueo4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Parqueo4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_CS_Pin LCD_D6_Pin LCD_D3_Pin LCD_D5_Pin
                           LCD_D4_Pin SD_SS_Pin */
  GPIO_InitStruct.Pin = LCD_CS_Pin|LCD_D6_Pin|LCD_D3_Pin|LCD_D5_Pin
                          |LCD_D4_Pin|SD_SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SegE_Pin SegD_Pin SegC_Pin SegG_Pin
                           SegF_Pin */
  GPIO_InitStruct.Pin = SegE_Pin|SegD_Pin|SegC_Pin|SegG_Pin
                          |SegF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Parqueo1_Pin Parqueo2_Pin Parqueo3_Pin */
  GPIO_InitStruct.Pin = Parqueo1_Pin|Parqueo2_Pin|Parqueo3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_D2_Pin SegB_Pin SegA_Pin */
  GPIO_InitStruct.Pin = LCD_D2_Pin|SegB_Pin|SegA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : P2G_Pin */
  GPIO_InitStruct.Pin = P2G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(P2G_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
    bool nuevo_estado;

    if(GPIO_Pin == Parqueo1_Pin){
        nuevo_estado = ((HAL_GPIO_ReadPin(GPIOC, Parqueo1_Pin)) == GPIO_PIN_SET);
        if (carro1 != nuevo_estado) {
            carro1 = nuevo_estado;
            espacioslibres += nuevo_estado ? -1 : 1;
            mandari2c();
        }
    }

    else if(GPIO_Pin == Parqueo2_Pin){
        nuevo_estado = (HAL_GPIO_ReadPin(GPIOC, Parqueo2_Pin) == GPIO_PIN_SET);
        if (carro2 != nuevo_estado) {
            carro2 = nuevo_estado;
            espacioslibres += nuevo_estado ? -1 : 1;
            mandari2c();
        }
    }

    else if(GPIO_Pin == Parqueo3_Pin){
        nuevo_estado = (HAL_GPIO_ReadPin(GPIOC, Parqueo3_Pin) == GPIO_PIN_SET);
        if (carro3 != nuevo_estado) {
            carro3 = nuevo_estado;
            espacioslibres += nuevo_estado ? -1 : 1;
            mandari2c();
        }
    }

    else if(GPIO_Pin == Parqueo4_Pin){
        nuevo_estado = (HAL_GPIO_ReadPin(GPIOC, Parqueo4_Pin) == GPIO_PIN_SET);
        if (carro4 != nuevo_estado) {
            carro4 = nuevo_estado;
            espacioslibres += nuevo_estado ? -1 : 1;
            mandari2c();
        }
    }
}


void segmentos(uint8_t num){
	switch (num) {
		case 0:
			HAL_GPIO_WritePin(SegA_GPIO_Port, SegA_Pin, 1);
			HAL_GPIO_WritePin(SegB_GPIO_Port, SegB_Pin, 1);
			HAL_GPIO_WritePin(SegC_GPIO_Port, SegC_Pin, 1);
			HAL_GPIO_WritePin(SegD_GPIO_Port, SegD_Pin, 1);
			HAL_GPIO_WritePin(SegE_GPIO_Port, SegE_Pin, 1);
			HAL_GPIO_WritePin(SegF_GPIO_Port, SegF_Pin, 1);
			HAL_GPIO_WritePin(SegF_GPIO_Port, SegG_Pin, 0);
			break;
		case 1:
			HAL_GPIO_WritePin(SegA_GPIO_Port, SegA_Pin, 0);
			HAL_GPIO_WritePin(SegB_GPIO_Port, SegB_Pin, 1);
			HAL_GPIO_WritePin(SegC_GPIO_Port, SegC_Pin, 1);
			HAL_GPIO_WritePin(SegD_GPIO_Port, SegD_Pin, 0);
			HAL_GPIO_WritePin(SegE_GPIO_Port, SegE_Pin, 0);
			HAL_GPIO_WritePin(SegF_GPIO_Port, SegF_Pin, 0);
			HAL_GPIO_WritePin(SegF_GPIO_Port, SegG_Pin, 0);
			break;
		case 2:
			HAL_GPIO_WritePin(SegA_GPIO_Port, SegA_Pin, 1);
			HAL_GPIO_WritePin(SegB_GPIO_Port, SegB_Pin, 1);
			HAL_GPIO_WritePin(SegC_GPIO_Port, SegC_Pin, 0);
			HAL_GPIO_WritePin(SegD_GPIO_Port, SegD_Pin, 1);
			HAL_GPIO_WritePin(SegE_GPIO_Port, SegE_Pin, 1);
			HAL_GPIO_WritePin(SegF_GPIO_Port, SegF_Pin, 0);
			HAL_GPIO_WritePin(SegF_GPIO_Port, SegG_Pin, 1);
			break;
		case 3:
			HAL_GPIO_WritePin(SegA_GPIO_Port, SegA_Pin, 1);
			HAL_GPIO_WritePin(SegB_GPIO_Port, SegB_Pin, 1);
			HAL_GPIO_WritePin(SegC_GPIO_Port, SegC_Pin, 1);
			HAL_GPIO_WritePin(SegD_GPIO_Port, SegD_Pin, 1);
			HAL_GPIO_WritePin(SegE_GPIO_Port, SegE_Pin, 0);
			HAL_GPIO_WritePin(SegF_GPIO_Port, SegF_Pin, 0);
			HAL_GPIO_WritePin(SegF_GPIO_Port, SegG_Pin, 1);
			break;
		case 4:
			HAL_GPIO_WritePin(SegA_GPIO_Port, SegA_Pin, 0);
			HAL_GPIO_WritePin(SegB_GPIO_Port, SegB_Pin, 1);
			HAL_GPIO_WritePin(SegC_GPIO_Port, SegC_Pin, 1);
			HAL_GPIO_WritePin(SegD_GPIO_Port, SegD_Pin, 0);
			HAL_GPIO_WritePin(SegE_GPIO_Port, SegE_Pin, 0);
			HAL_GPIO_WritePin(SegF_GPIO_Port, SegF_Pin, 1);
			HAL_GPIO_WritePin(SegF_GPIO_Port, SegG_Pin, 1);
			break;
		default:
			break;
	}
}

void mandari2c(void){
	uint8_t nibble = (carro1 << 3) | (carro2 << 2) | (carro3 << 1) | carro4;
	char ascii_char;
	if (nibble < 10) {
	    ascii_char = '0' + nibble;
	} else {
	    ascii_char = '0' + (nibble);
	}
	HAL_StatusTypeDef result = HAL_I2C_Master_Transmit(&hi2c1, ESP32_I2C_ADDRESS << 1, (uint8_t*)&ascii_char, 1, HAL_MAX_DELAY);
}
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
	while (1) {
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
