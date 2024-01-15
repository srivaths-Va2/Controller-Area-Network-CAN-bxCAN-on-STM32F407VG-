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
#include <string.h>
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// Declaring the function prototypes
void CAN1_Init(void);
void CAN1_Tx(void);
void CAN1_Filter_Config(void);
void send_response(uint32_t std_id);
void LED_output(uint8_t led_no);

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */
CAN_RxHeaderTypeDef RxHeader;
uint8_t req_counter = 0;
uint8_t led_no;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM6_Init(void);
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
  MX_CAN1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  CAN1_Init();
  CAN1_Filter_Config();


  /* USER CODE END 2 */
  if(HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY|CAN_IT_RX_FIFO0_MSG_PENDING|CAN_IT_BUSOFF) != HAL_OK)
	  Error_Handler();

  if(HAL_CAN_Start(&hcan1) != HAL_OK)
	  Error_Handler();

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */

  return 0;
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV16;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 10000;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin LED4_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// CAN1 initialization function
void CAN1_Init(void)
{
	hcan1.Instance = CAN1;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	hcan1.Init.AutoBusOff = ENABLE;
	hcan1.Init.AutoRetransmission = ENABLE;
	hcan1.Init.AutoWakeUp = DISABLE;
	hcan1.Init.ReceiveFifoLocked = DISABLE;
	hcan1.Init.TimeTriggeredMode = DISABLE;
	hcan1.Init.TransmitFifoPriority = DISABLE;

	// Settings related to CAN1 timing
	hcan1.Init.Prescaler = 3;
	hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
	hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;

	if(HAL_CAN_Init(&hcan1) != HAL_OK)
	{
		Error_Handler();
	}
}

void CAN1_Tx(void)
{
	CAN_TxHeaderTypeDef TxHeader;
	uint32_t Txmailbox;
	uint8_t message;

	// Defining the parameters for TxHeader
	TxHeader.StdId = 0x65D;
	TxHeader.DLC = 1;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;

	message = ++led_no;

	if(led_no == 4)
	{
		led_no = 0;
	}

	if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, &message, &Txmailbox) != HAL_OK)
	{
		Error_Handler();
	}
}

void CAN1_Filter_Config(void)
{
	CAN_FilterTypeDef can1_filter_init;

	// Initializing the filter variables
	can1_filter_init.FilterActivation = CAN_FILTERMODE_IDMASK;
	can1_filter_init.FilterBank = 0;
	can1_filter_init.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	can1_filter_init.FilterIdHigh = 0x0000;
	can1_filter_init.FilterIdLow = 0x0000;
	can1_filter_init.FilterMaskIdHigh = 0x01C0;
	can1_filter_init.FilterMaskIdLow = 0x0000;
	can1_filter_init.FilterMode = CAN_FILTERMODE_IDMASK;
	can1_filter_init.FilterScale = CAN_FILTERSCALE_32BIT;

	// configuring the filter
	if(HAL_CAN_ConfigFilter(&hcan1, &can1_filter_init) != HAL_OK)
	{
		Error_Handler();
	}
}

void send_response(uint32_t std_id)
{
	CAN_TxHeaderTypeDef TxHeader;
	uint32_t Txmailbox;
	uint8_t response[2] = {0xAB, 0xCD};

	TxHeader.DLC = 2;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.StdId = std_id;
	TxHeader.RTR = CAN_RTR_DATA;

	if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, response, &Txmailbox) != HAL_OK)
	{
		Error_Handler();
	}
}

void LED_output(uint8_t led_no)
{
	switch(led_no)
	{
	case 1:
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);

	case 2:
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);

	case 3:
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);

	case 4:
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
	}
}

// Callback functions to handle interrupt based events for CAN

// callback function to trigger interrupt if TxMailBox-0 is empty
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
	printf("Message transmitted from TxMailBox-0\n");
}

// callback function to trigger interrupt if TxMailBox-1 is empty
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
	printf("Message transmitted from TxMailBox-1\n");
}

// callback function to trigger interrupt if TxMailBox-2 is empty
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
	printf("Message transmitted from TxMailBox-2\n");
}

// callback function to get received message and process
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t rcvd_msg[8];

	// Checking if the received frame is valid
	if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, rcvd_msg) != HAL_OK)
	{
		Error_Handler();
	}

	// processing data frame sent from N1 to N2
	if(RxHeader.StdId == 0x65D && RxHeader.RTR == 0)
	{
		LED_output(rcvd_msg[0]);
		printf("Message received : %d", rcvd_msg[0]);
	}

	// processing remote frame sent from N1 to N2
	else if(RxHeader.StdId == 0x651 && RxHeader.RTR == 1)
	{
		send_response(RxHeader.StdId);
		return;
	}

	// processing data frame sent from N2 to N1
	else if(RxHeader.StdId == 0x651 && RxHeader.IDE == 0)
	{
		printf("Message received : %d", rcvd_msg[0] << 8 | rcvd_msg[1]);
	}

}

// Callback to generate interrupt when timer elapses
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	CAN_TxHeaderTypeDef TxHeader;
	uint32_t Txmailbox;
	uint8_t message;

	if(req_counter == 4)
	{
		// N1 sending remote frame to N2
		TxHeader.DLC = 2; 		//demanding 2 bytes of reply
		TxHeader.StdId = 0x651;
		TxHeader.IDE = CAN_ID_STD;
		TxHeader.RTR = CAN_RTR_REMOTE;

		if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, &message, &Txmailbox) != HAL_OK)
		{
			Error_Handler();
		}

		req_counter = 0;
	}

	else
	{
		CAN1_Tx();
		req_counter++;
	}
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
