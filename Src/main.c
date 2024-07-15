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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "data_convert.h"
#include "FlashF446re.h"
#include <stdbool.h>
#include <string.h>
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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint32_t Address=0;


uint8_t Data_rx[75]={};
volatile _Bool flag=RESET, flag_uart=RESET;
volatile uint16_t rx_len=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**Functions in the FlashF446re.c
 * */
HAL_StatusTypeDef Flash_Program_Byte(DataToFlash *Flashdata, uint16_t *size);
HAL_StatusTypeDef flash_control(DataToFlash *Flashdata, uint32_t size);

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */



	HAL_StatusTypeDef status=HAL_OK, status_flash;
	DataStatus status2=start_ok;												//DataStatus- is an enumeration in the file data_convert.h

	DataToFlash Flashdata[1043];												//array of structures for storing the received data
																				//DatatoFlash 	in the file data_convert.h
	//uint8_t data[1000];

	//uint8_t Data_tx[555]="default";
	uint8_t status_data[]="flash_data_correct";
	uint8_t status_flash_error[]="flash_error_of_write";
	uint8_t status_flash_error_data[]="flash_incorrect_recorded_data_in_Flash";
	uint8_t status_str[]="tranfer_OK\n";
	uint8_t status_error_crc[]="crc_error\n";
	uint8_t status_error[]="timeout\n";

	uint16_t len=0, size=0;

	uint32_t Addr_data=0;
	uint32_t appJumpAddress;
	//uint32_t Address_data[100];

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
  HAL_UARTEx_ReceiveToIdle_IT(&huart2,Data_rx, 25);

  while (1)
  {
	 if(flag)
	 {
		 /*
		  *Write in Flash the received data. Data received checking.
		  * */
		 status = Flash_Program_Byte(Flashdata, &size);    									// write in Flash the received data
		 if(status == HAL_OK)
		 {
			 status_flash = flash_control(Flashdata,size);											// Checking recorded data
			 if(status_flash == HAL_OK)
			 {
				 HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
				 HAL_UART_Transmit(&huart2,status_data, strlen((const char *)status_data),100);				// Displaying message in UART (COM-port)

				 HAL_Delay(1);
				 void (*GoToApp)(void);
				 appJumpAddress = *((volatile uint32_t*)(ADDR_FLASH_SECTOR_4+4));
				 GoToApp = (void (*)(void))appJumpAddress;
				 HAL_DeInit();																		// deinitialization HAL
				 __disable_irq(); 																	// deinitialization of interruptions
				 __set_MSP(*((volatile uint32_t*)ADDR_FLASH_SECTOR_4 ));							// Set new address of stack
				 GoToApp();																			// give of management
			}
			else
			{
				HAL_UART_Transmit(&huart2,status_flash_error_data, strlen((const char *)status_flash_error_data),100); //Displaying message in UART (COM-port)
			}
		 }
		 else
		 {
			 HAL_UART_Transmit(&huart2,status_flash_error, strlen((const char *)status_flash_error),100);  //Displaying message in UART (COM-port)
		 }
	 	 flag=!flag;
	 }
	 if (flag_uart==SET)
	 {
		 while(Data_rx[len]!='\n')															// Count received string
		 {
			 len++;
		 }
		 status2=DataConverter(Data_rx,len,Flashdata, &size, &Addr_data);					// parsing data
		 if((status2==error_addr) || (status2==error_data))
		 {
			 if (status2==error_crc)
			 {
				 HAL_UART_Transmit(&huart2,status_error_crc, strlen((const char *)status_error_crc),100); //Displaying message in UART (COM-port)
				 break;
			 }
			 else
			 {
				 HAL_UART_Transmit(&huart2,status_error, strlen((const char *)status_error),100);			//Displaying message in UART (COM-port)
			 	 break;
			 }
		 }
		 else if(status2==end_ok)
		 {
			 HAL_UART_Transmit(&huart2,status_str, strlen((const char *)status_str), 100);				//Displaying message in UART (COM-port)
		 }
		 len=0;
		 flag_uart=RESET;
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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)											//Callback of button
{
  /* Prevent unused argument(s) compilation warning*/
	if(GPIO_Pin==B1_Pin)
	{
		flag=!flag;
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	}
  /* NOTE: This function Should not be modified, when the callback is needed,
		   the HAL_GPIO_EXTI_Callback could be implemented in the user file*/
}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)								// обработчки по окончнаю приема всех данных
//{
  /* Prevent unused argument(s) compilation warning*/
	/*if(huart == &huart2)
	{
		rx_len=50-huart->RxXferCount;
		__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
		HAL_UARTEx_ReceiveToIdle_IT(&huart2,Data_rx, 25);
		flag_uart=SET;
	}*/
  /* NOTE: This function should not be modified, when the callback is needed,
      the HAL_UART_RxCpltCallback could be implemented in the user file*/

//}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)				// обработчик события приема по UART, срабатывает по IDLE и RxCplt
{																						//
  /* Prevent unused argument(s) compilation warning */
	if(huart == &huart2)
	{
		rx_len=50-huart->RxXferCount;
		//__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
		HAL_UARTEx_ReceiveToIdle_IT(&huart2,Data_rx, 50);
		flag_uart=SET;
	}

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_UARTEx_RxEventCallback can be implemented in the user file.
   */
}


/*	For idle UART
 * 	Потому что не было calback фукнции в файле "stm32f4xx_hal_uart.c" на idle
 * 	пришлось самому написать данную фукнцию (взята из �?нтернета).
 * 	В файле "stm32f4xx_hal_uart.c" это строка 2584-2589
 * 	В файле "stm32f4xx_hal_uart.h" это строка 755
 */

/*void HAL_UART_IDLE_Callback(UART_HandleTypeDef *huart) {
	if (huart == &huart2) {
		//__HAL_UART_DISABLE_IT(&huart2, UART_IT_IDLE);
		rx_len=50-huart->RxXferCount;
		if(huart2.gState!=HAL_UART_STATE_BUSY_TX)
		{
			HAL_UART_Transmit_IT(&huart2, rx_len, sizeof(rx_len));
		}
		HAL_UART_AbortReceive_IT(&huart2);
		__HAL_UART_CLEAR_IDLEFLAG(&huart2);
		__HAL_UART_ENABLE_IT(&huart2,UART_IT_IDLE);
		HAL_UART_Receive_IT(&huart2, Data_rx, 50);
		//HAL_UARTEx_ReceiveToIdle_IT(&huart2, Data_rx, 75);
		flag_uart=SET;

	}
}*/


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
