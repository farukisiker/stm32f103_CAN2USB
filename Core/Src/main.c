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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "usbd_cdc.h"
#include "CANSPI.h"
#include "MCP2515.h"
#include <stdio.h>
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
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
uCAN_MSG txMessage;
uCAN_MSG rxMessage;
uint8_t rec_buffer[64];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
static void Reset_USB(void);
void convertToHex(unsigned char value, char* hexString, size_t size);
void convert32ToHex(uint32_t value, char* hexString, size_t size);

uint8_t convertToU8(const char* hexString, size_t NumberOfCharacters);
uint32_t convertToU32(const char* hexString, size_t NumberOfCharacters) ;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
USBD_HandleTypeDef USBD_Device;


uint8_t CDC_BUF[60];

void usb_rec_mcp_transmit(void)
{
	uint32_t id = 0;

	if (rec_buffer[1] == 'x')
	{
		id = convertToU32((const char*)rec_buffer, 10);
		if (id<=0x7FFU) {
			txMessage.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B; //dEXTENDED_CAN_MSG_ID_2_0B
		} else {
			txMessage.frame.idType = dEXTENDED_CAN_MSG_ID_2_0B; //dEXTENDED_CAN_MSG_ID_2_0B
		}
		txMessage.frame.id = id;
		txMessage.frame.dlc = 	convertToU8((const char*)(rec_buffer+14), 2);
		txMessage.frame.data0 = convertToU8((const char*)(&(rec_buffer[19])), 2);
		txMessage.frame.data1 = convertToU8((const char*)(&(rec_buffer[24])), 2);
		txMessage.frame.data2 = convertToU8((const char*)(&(rec_buffer[29])), 2);
		txMessage.frame.data3 = convertToU8((const char*)(&(rec_buffer[34])), 2);
		txMessage.frame.data4 = convertToU8((const char*)(&(rec_buffer[39])), 2);
		txMessage.frame.data5 = convertToU8((const char*)(&(rec_buffer[44])), 2);
		txMessage.frame.data6 = convertToU8((const char*)(&(rec_buffer[49])), 2);
		txMessage.frame.data7 = convertToU8((const char*)(&(rec_buffer[54])), 2);
		CANSPI_Transmit(&txMessage);
		memset(rec_buffer,0,sizeof(rec_buffer));
	}
}

void mcp2515_RX0IE_callback(void)
{
	if(CANSPI_Receive(&rxMessage))
	{
		char id[11] = {0};
		char dlc[5] = {0};
		char data0[5] = {0};
		char data1[5] = {0};
		char data2[5] = {0};
		char data3[5] = {0};
		char data4[5] = {0};
		char data5[5] = {0};
		char data6[5] = {0};
		char data7[5] = {0};
		convert32ToHex(rxMessage.frame.id, id, 12);
		convertToHex(rxMessage.frame.dlc, dlc, 5);
		convertToHex(rxMessage.frame.data0,data0,5);
		convertToHex(rxMessage.frame.data1,data1,5);
		convertToHex(rxMessage.frame.data2,data2,5);
		convertToHex(rxMessage.frame.data3,data3,5);
		convertToHex(rxMessage.frame.data4,data4,5);
		convertToHex(rxMessage.frame.data5,data5,5);
		convertToHex(rxMessage.frame.data6,data6,5);
		convertToHex(rxMessage.frame.data7,data7,5);

		switch (rxMessage.frame.dlc) {
			case 0:
				sprintf((char*)CDC_BUF,"%s %s %s",id,dlc,"\n");
				break;
			case 1:
				sprintf((char*)CDC_BUF,"%s %s %s %s",id,dlc,data0,"\n");
				break;
			case 2:
				sprintf((char*)CDC_BUF,"%s %s %s %s %s",id,dlc,data0,data1,"\n");
				break;
			case 3:
				sprintf((char*)CDC_BUF,"%s %s %s %s %s %s",id,dlc,
						data0,
						data1,
						data2,
						"\n");
				break;
			case 4:
				sprintf((char*)CDC_BUF,"%s %s %s %s %s %s %s",id,dlc,
						data0,
						data1,
						data2,
						data3,
						"\n");
				break;
			case 5:
				sprintf((char*)CDC_BUF,"%s %s %s %s %s %s %s %s",id,dlc,
						data0,
						data1,
						data2,
						data3,
						data4,
						"\n");
				break;
			case 6:
				sprintf((char*)CDC_BUF,"%s %s %s %s %s %s %s %s %s",id,dlc,
						data0,
						data1,
						data2,
						data3,
						data4,
						data5,
						"\n");
				break;
			case 7:
				sprintf((char*)CDC_BUF,"%s %s %s %s %s %s %s %s %s %s",id,dlc,
						data0,
						data1,
						data2,
						data3,
						data4,
						data5,
						data6,
						"\n");
				break;
			case 8:
				sprintf((char*)CDC_BUF,"%s %s %s %s %s %s %s %s %s %s %s",id,dlc,
						data0,
						data1,
						data2,
						data3,
						data4,
						data5,
						data6,
						data7,
						"\n"
						);
				break;

			default:
				break;
		}

		CDC_Transmit_FS((uint8_t*)CDC_BUF, (uint16_t)(strlen((const char*)CDC_BUF)));
		memset(CDC_BUF,0,sizeof(CDC_BUF));

	}
}

void convert32ToHex(uint32_t value, char* hexString, size_t size) {
    snprintf(hexString, size, "0x%08lXh", value);
}
void convertToHex(unsigned char value, char* hexString, size_t size){
	snprintf(hexString, size, "0x%02X", value);
}
uint8_t convertToU8(const char* hexString, size_t NumberOfCharacters) {
    uint8_t value = 0;

    for (size_t i = 0; i < NumberOfCharacters; i++) {
        char c = hexString[i];

        if (c >= '0' && c <= '9') {
            value = (value << 4) + (c - '0');
        }
        else if (c >= 'A' && c <= 'F') {
            value = (value << 4) + (c - 'A' + 10);
        }
        else if (c >= 'a' && c <= 'f') {
            value = (value << 4) + (c - 'a' + 10);
        }
        else {
            // Handle error: Invalid character encountered
            // You can return a default value or handle the error as needed
            return 0;
        }
    }

    return value;
}
uint32_t convertToU32(const char* hexString, size_t NumberOfCharacters) {
	uint32_t value = 0;

	    // Skip the "0x" prefix if present
	    if (NumberOfCharacters >= 2 && hexString[0] == '0' && (hexString[1] == 'x' || hexString[1] == 'X')) {
	        hexString += 2;
	        NumberOfCharacters -= 2;
	    }

	    for (size_t i = 0; i < NumberOfCharacters; i++) {
	        char c = hexString[i];

	        if (c >= '0' && c <= '9') {
	            value = (value << 4) + (c - '0');
	        }
	        else if (c >= 'A' && c <= 'F') {
	            value = (value << 4) + (c - 'A' + 10);
	        }
	        else if (c >= 'a' && c <= 'f') {
	            value = (value << 4) + (c - 'a' + 10);
	        }
	        else {
	            // Handle error: Invalid character encountered
	            // You can return a default value or handle the error as needed
	            return 0;
	        }
	    }

	    return value;
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
  Reset_USB();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  CANSPI_Initialize();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //CDC_Transmit_FS((uint8_t*)("heart beat\n"), 11);

	  //HAL_Delay(100);
	  usb_rec_mcp_transmit();
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CAN_CS_GPIO_Port, CAN_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CAN_CS_Pin */
  GPIO_InitStruct.Pin = CAN_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CAN_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CAN_INT_Pin */
  GPIO_InitStruct.Pin = CAN_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CAN_INT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

}

/* USER CODE BEGIN 4 */
static void Reset_USB(void)
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	  /*Configure GPIO pin : LED_Pin */

	  GPIO_InitStruct.Pin = USB_DP_PIN_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	  HAL_GPIO_Init(USB_DP_PIN_GPIO_Port, &GPIO_InitStruct);
	  HAL_GPIO_WritePin(USB_DP_PIN_GPIO_Port, USB_DP_PIN_Pin, 0);
	  HAL_Delay(5);
	  HAL_GPIO_WritePin(USB_DP_PIN_GPIO_Port, USB_DP_PIN_Pin, 1);
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
