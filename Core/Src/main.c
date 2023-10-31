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

#include "usbd_cdc.h"

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
CAN_HandleTypeDef hcan;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//#define BRIDGE

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;

uint8_t TxData[8];
uint8_t RxData[8];

uint32_t TxMailBox;

#if defined(BRIDGE)
uint8_t Data[14];
uint8_t DataRx[14];
#else
uint8_t Data[31];
uint8_t DataRx[31];

#endif

int flag = 0;

uint8_t ToChar(uint8_t c) {
	if  (0 <= c && c <= 9) {
		return c + '0';
	}else if ('a' <= c && c <= 'f') {
		return c + 'a' - 10;
	}else if (10 <= c && c <= 15) {
		return c + 'A' - 10;
	}
	//return c + '0';
}

uint8_t FromChar(uint8_t c) {

	if ('0' <= c && c <= '9') {
		return c - '0';
	}else if('a' <= c && c <= 'f'){
		return c - 'a' + 10;
	} else if('A' <= c && c <= 'F') {
		return c - 'A' + 10;
	}

}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

	HAL_GPIO_WritePin(GPIOB, RX_LED_Pin, GPIO_PIN_SET);
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
#if defined(BRIDGE)
	if(RxHeader.StdId == 0xA | RxHeader.StdId == 0x7
						 	 | RxHeader.StdId == 0x5
							 | RxHeader.StdId == 0x4
						     | RxHeader.StdId == 0x8
							 | RxHeader.StdId == 0x6 ) {
		return;
	}
#endif
	uint16_t ts = (uwTick % 0xEA60);

	if (RxHeader.StdId != 0) {
#if defined(BRIDGE)
		DataRx[0] = (uint8_t)(RxHeader.StdId >> 8);
		DataRx[1] = (uint8_t)RxHeader.StdId;
		DataRx[2] = (uint8_t)RxHeader.DLC;

		for (int i = 0; i < RxHeader.DLC; i++) {
			DataRx[i + 3] = RxData[i];
			if(DataRx[i + 3] == 0xFF) DataRx[i + 3] = 0xFE;
		}

		DataRx[RxHeader.DLC + 3] = (uint8_t)0xFF;
		CDC_Transmit_FS(DataRx, RxHeader.DLC + 4);
#else
		DataRx[0] = 't';
		DataRx[1] = ToChar((uint8_t)(RxHeader.StdId >> 8) & 0xF);
		DataRx[2] = ToChar((uint8_t)(RxHeader.StdId >> 4) & 0xF);
		DataRx[3] = ToChar((uint8_t)(RxHeader.StdId     ) & 0xF);
		DataRx[4] = ToChar((uint8_t)(RxHeader.DLC));

		int j = 0;
		for (int i = 0; i < RxHeader.DLC; i++) {
			DataRx[i + 5 + j++] = ToChar(RxData[i] >> 4);
			DataRx[i + 5 + j] = ToChar(RxData[i] & 0b00001111);
		}

		DataRx[RxHeader.DLC * 2 + 5] = ToChar((uint8_t) (ts >> 12));
		DataRx[RxHeader.DLC * 2 + 6] = ToChar((uint8_t) ((ts >> 8) & 0b00001111));
		DataRx[RxHeader.DLC * 2 + 7] = ToChar((uint8_t) ((ts >> 4) & 0b00001111));
		DataRx[RxHeader.DLC * 2 + 8] = ToChar((uint8_t) ((ts >> 0) & 0b00001111));

		DataRx[RxHeader.DLC * 2 + 5 + 4] = '\r'; //+4

		CDC_Transmit_FS(DataRx, RxHeader.DLC * 2 + 10); //+4
#endif
		RxHeader.StdId = 0;
	} else if (RxHeader.ExtId != 0){
#if defined(BRIDGE)
		DataRx[0] = (uint8_t)(RxHeader.ExtId >> 24);
		DataRx[1] = (uint8_t)(RxHeader.ExtId >> 16);
		DataRx[2] = (uint8_t)(RxHeader.ExtId >> 8);
		DataRx[3] = (uint8_t) RxHeader.ExtId;
		DataRx[4] = (uint8_t) RxHeader.DLC;

		for (int i = 0; i < RxHeader.DLC; i++) {

			DataRx[i + 5] = RxData[i];
			if(DataRx[i + 5] == 0xFF) DataRx[i + 5] = 0xFE;
		}

		DataRx[RxHeader.DLC + 5] = (uint8_t)0xFF;
		CDC_Transmit_FS(DataRx, RxHeader.DLC + 6);
#else
		DataRx[0] = 'T';
		DataRx[1] = ToChar((uint8_t)(RxHeader.ExtId >> 28) & 0xF);
		DataRx[2] = ToChar((uint8_t)(RxHeader.ExtId >> 24) & 0xF);
		DataRx[3] = ToChar((uint8_t)(RxHeader.ExtId >> 20) & 0xF);
		DataRx[4] = ToChar((uint8_t)(RxHeader.ExtId >> 16) & 0xF);
		DataRx[5] = ToChar((uint8_t)(RxHeader.ExtId >> 12) & 0xF);
		DataRx[6] = ToChar((uint8_t)(RxHeader.ExtId >> 8 ) & 0xF);
		DataRx[7] = ToChar((uint8_t)(RxHeader.ExtId >> 4 ) & 0xF);
		DataRx[8] = ToChar((uint8_t)(RxHeader.ExtId      ) & 0xF);

		DataRx[9] = ToChar((uint8_t)(RxHeader.DLC));
		int j = 0;
		for (int i = 0; i < RxHeader.DLC; i++) {
			DataRx[i + 10 + j++] = ToChar(RxData[i] >> 4);
			DataRx[i + 10 + j] = ToChar(RxData[i] & 0b00001111);
		}

		DataRx[RxHeader.DLC * 2 + 10] = ToChar((uint8_t) (ts >> 12));
		DataRx[RxHeader.DLC * 2 + 11] = ToChar((uint8_t) ((ts >> 8) & 0b00001111));
		DataRx[RxHeader.DLC * 2 + 12] = ToChar((uint8_t) ((ts >> 4) & 0b00001111));
		DataRx[RxHeader.DLC * 2 + 13] = ToChar((uint8_t) ((ts >> 0) & 0b00001111));

		DataRx[RxHeader.DLC * 2 + 10 + 4] = '\r'; //+4

		CDC_Transmit_FS(DataRx, RxHeader.DLC * 2 + 15); //+4
#endif
		RxHeader.ExtId = 0;
	}
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
  MX_USB_DEVICE_Init();
#if defined (BRIDGE)
  MX_CAN_Init_250();
#endif
//  MX_CAN_Init();
  /* USER CODE BEGIN 2 */


//  HAL_CAN_Start(&hcan);
//  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if (flag) {
#if defined(BRIDGE)
			TxHeader.DLC = Data[2];
			TxHeader.ExtId = 0;
			TxHeader.IDE = CAN_ID_STD;
			TxHeader.RTR = CAN_RTR_DATA;
			TxHeader.StdId = (uint32_t) (FromChar(Data[0]) << 8) | (uint32_t)FromChar(Data[1]);
			TxHeader.TransmitGlobalTime = DISABLE;
			for (int i = 0; i < Data[2]; i++) {
			  TxData[i] = Data[i+3];
			}
#else
			if (Data[0] == 't') {
				TxHeader.DLC = FromChar((uint8_t) (Data[4]));
				TxHeader.ExtId = 0;
				TxHeader.IDE = CAN_ID_STD;
				TxHeader.RTR = CAN_RTR_DATA;
				TxHeader.StdId = (uint32_t) (FromChar(Data[1]) << 8)
						| (uint32_t) (FromChar(Data[2]) << 4)
						| (uint32_t) (FromChar(Data[3]));
				TxHeader.TransmitGlobalTime = DISABLE;
				int j = 0;
				for (int i = 0; i < TxHeader.DLC; i++) {
					TxData[i] = (FromChar(Data[i + 5 + j++]) << 4)
							| FromChar(Data[i + 5 + j]);
				}

			} else if (Data[0] == 'T') {
				TxHeader.DLC = FromChar((uint8_t) (Data[9]));
				TxHeader.ExtId = (uint32_t) (FromChar(Data[1]) << 28)
						| (uint32_t) (FromChar(Data[2]) << 24)
						| (uint32_t) (FromChar(Data[3]) << 20)
						| (uint32_t) (FromChar(Data[4]) << 16)
						| (uint32_t) (FromChar(Data[5]) << 12)
						| (uint32_t) (FromChar(Data[6]) << 8)
						| (uint32_t) (FromChar(Data[7]) << 4)
						| (uint32_t) (FromChar(Data[8]));
				TxHeader.IDE = CAN_ID_EXT;
				TxHeader.RTR = CAN_RTR_DATA;
				TxHeader.StdId = 0;
				TxHeader.TransmitGlobalTime = DISABLE;
				int j = 0;
				for (int i = 0; i < TxHeader.DLC; i++) {
					TxData[i] = (FromChar(Data[i + 10 + j++]) << 4)
							| FromChar(Data[i + 10 + j]);
				}

			}
#endif
			HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailBox);

			flag = 0;
			HAL_GPIO_WritePin(GPIOB, TX_LED_Pin, GPIO_PIN_RESET);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

///**
//  * @brief CAN Initialization Function
//  * @param None
//  * @retval None
//  */


/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TX_LED_Pin|RX_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : TX_LED_Pin RX_LED_Pin */
  GPIO_InitStruct.Pin = TX_LED_Pin|RX_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

 void MX_CAN_Init_250(void)
{

  /* USER CODE BEGIN CAN_Init 0 */


  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 12;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  CAN_FilterTypeDef canfilterconfig;

	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
	canfilterconfig.FilterBank = 0;
	canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	canfilterconfig.FilterIdHigh = 0;
	canfilterconfig.FilterIdLow = 0;
	canfilterconfig.FilterMaskIdHigh = 0;
	canfilterconfig.FilterMaskIdLow = 0;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
	canfilterconfig.SlaveStartFilterBank = 14; // how many filters to assign to the CAN1 (master can)

	HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);

	HAL_CAN_Start(&hcan);
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

  /* USER CODE END CAN_Init 2 */

}

 void MX_CAN_Init_500(void)
{

  /* USER CODE BEGIN CAN_Init 0 */


  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 12;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_4TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  CAN_FilterTypeDef canfilterconfig;

	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
	canfilterconfig.FilterBank = 0;
	canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	canfilterconfig.FilterIdHigh = 0;
	canfilterconfig.FilterIdLow = 0;
	canfilterconfig.FilterMaskIdHigh = 0;
	canfilterconfig.FilterMaskIdLow = 0;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
	canfilterconfig.SlaveStartFilterBank = 14; // how many filters to assign to the CAN1 (master can)

	HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);

	HAL_CAN_Start(&hcan);
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

  /* USER CODE END CAN_Init 2 */

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

