/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAXSIZE_BufferMsg 50
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
uint8_t rxBufferMsgUsb[MAXSIZE_BufferMsg];
uint8_t *p_rxMsgUsb = &rxBufferMsgUsb[0];
uint8_t msgReturn[MAXSIZE_BufferMsg];
//Command
const char *REL_CLO = "REL_CLO";
const char *REL_OPE = "REL_OPE";
const char *REL_OUT = "REL_OUT";
const char *SEN_INP = "SEN_INP";
const char *BRD_DES = "BRD_DES";
//Messages
const char *STA_INP = "\rSTA_INP\tINP_OPT1:%d,INP_OPT2:%d,INP_OPT3:%d,INP_OPT4:%d\n";
const char *STA_OUT = "\rSTA_OUT\tOUT_REL1:%d,OUT_REL2:%d,OUT_REL3:%d,OUT_REL4:%d,OUT_REL5:%d,OUT_REL6:%d,OUT_REL7:%d,OUT_REL8:%d\n";
const char *MSG_FAI = "\rMSG_FAI\n";
//const char *MSG_PAS = "\rMSG_PAS\n";
const char *MSG_DES = "\rMSG_DES\tBOARD:RELAY_MATRIX,VERSION:1.1,SUPPLIER:MINERPUNK\n";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void Buffer_Receiver_USB(uint8_t* Buf);
void Command_Hub(uint8_t *msg_p);
void Refresh_Rel_Outputs(uint8_t *rel_outputs);
void Transmit_Rel_Outputs(void);
void Refresh_Opto_Inputs(uint8_t *opto_inputs);
void Transmit_Opto_Inputs(void);
void REL1_Set(uint8_t value);
void REL2_Set(uint8_t value);
void REL3_Set(uint8_t value);
void REL4_Set(uint8_t value);
void REL5_Set(uint8_t value);
void REL6_Set(uint8_t value);
void REL7_Set(uint8_t value);
void REL8_Set(uint8_t value);
void Led_Error_Set(uint8_t value);
void Led_Data_Set(uint8_t value);
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
  MX_GPIO_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7200-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, REL1_Pin|REL2_Pin|REL3_Pin|REL4_Pin
                          |REL8_Pin|REL7_Pin|REL6_Pin|REL5_Pin
                          |LED_DATA_Pin|LED_ERROR_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : REL1_Pin REL2_Pin REL3_Pin REL4_Pin
                           REL8_Pin REL7_Pin REL6_Pin REL5_Pin
                           LED_DATA_Pin LED_ERROR_Pin */
  GPIO_InitStruct.Pin = REL1_Pin|REL2_Pin|REL3_Pin|REL4_Pin
                          |REL8_Pin|REL7_Pin|REL6_Pin|REL5_Pin
                          |LED_DATA_Pin|LED_ERROR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : OPT1_Pin OPT2_Pin OPT3_Pin OPT4_Pin */
  GPIO_InitStruct.Pin = OPT1_Pin|OPT2_Pin|OPT3_Pin|OPT4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void Buffer_Receiver_USB(uint8_t* Buf){
	uint8_t rxFlatUsb=0;
	for(uint8_t i = 0; i<MAXSIZE_BufferMsg; i++){
		if(Buf[i]=='\r'){
			rxFlatUsb=1;
			Led_Data_Set(1);
		}
		else if(Buf[i]=='\n'){
			Command_Hub(p_rxMsgUsb);
			Led_Data_Set(0);
			break;
		}
		else{
			if(rxFlatUsb){
				p_rxMsgUsb[i-1]=Buf[i];
			}
		}
	}
}
void Command_Hub(uint8_t *msg_p)
{
	//Set Close and select relay number
	if (!strncmp(msg_p, REL_CLO, strlen(REL_CLO)))
	{
		switch(msg_p[7]){
			case '1':
				REL1_Set(1);
				break;
			case '2':
				REL2_Set(1);
				break;
			case '3':
				REL3_Set(1);
				break;
			case '4':
				REL4_Set(1);
				break;
			case '5':
				REL5_Set(1);
				break;
			case '6':
				REL6_Set(1);
				break;
			case '7':
				REL7_Set(1);
				break;
			case '8':
				REL8_Set(1);
				break;
			default:
				Led_Error_Set(1);
				break;
		}
		Transmit_Rel_Outputs();
		//CDC_Transmit_FS(MSG_PAS, strlen(MSG_PAS));
	}
	//Set Open and select relay number
	else if (!strncmp(msg_p, REL_OPE, strlen(REL_OPE)))
	{
		switch(msg_p[7]){
			case '1':
				REL1_Set(0);
				break;
			case '2':
				REL2_Set(0);
				break;
			case '3':
				REL3_Set(0);
				break;
			case '4':
				REL4_Set(0);
				break;
			case '5':
				REL5_Set(0);
				break;
			case '6':
				REL6_Set(0);
				break;
			case '7':
				REL7_Set(0);
				break;
			case '8':
				REL8_Set(0);
				break;
			default:
				Led_Error_Set(1);
				break;
		}
		Transmit_Rel_Outputs();
		//CDC_Transmit_FS(MSG_PAS, strlen(MSG_PAS));
	}
	else if (!strncmp(msg_p, REL_OUT, strlen(REL_OUT)))
	{
		Transmit_Rel_Outputs();
	}
	//Transmit Opto sensors values
	else if (!strncmp(msg_p, SEN_INP, strlen(SEN_INP)))
	{
		Transmit_Opto_Inputs();
	}
	//Transmit Board Name
	else if (!strncmp(msg_p, BRD_DES, strlen(BRD_DES))){
		CDC_Transmit_FS(MSG_DES, strlen(MSG_DES));
	}
	//Received message is unknown
	else{
		CDC_Transmit_FS(MSG_FAI, strlen(MSG_FAI));
		Led_Error_Set(1);
	}
}
void Refresh_Rel_Outputs(uint8_t *rel_outputs){
	rel_outputs[0] = (HAL_GPIO_ReadPin(REL1_GPIO_Port, REL1_Pin) == GPIO_PIN_RESET) ? 1 : 0;
	rel_outputs[1] = (HAL_GPIO_ReadPin(REL2_GPIO_Port, REL2_Pin) == GPIO_PIN_RESET) ? 1 : 0;
	rel_outputs[2] = (HAL_GPIO_ReadPin(REL3_GPIO_Port, REL3_Pin) == GPIO_PIN_RESET) ? 1 : 0;
	rel_outputs[3] = (HAL_GPIO_ReadPin(REL4_GPIO_Port, REL4_Pin) == GPIO_PIN_RESET) ? 1 : 0;
	rel_outputs[4] = (HAL_GPIO_ReadPin(REL5_GPIO_Port, REL5_Pin) == GPIO_PIN_RESET) ? 1 : 0;
	rel_outputs[5] = (HAL_GPIO_ReadPin(REL6_GPIO_Port, REL6_Pin) == GPIO_PIN_RESET) ? 1 : 0;
	rel_outputs[6] = (HAL_GPIO_ReadPin(REL7_GPIO_Port, REL7_Pin) == GPIO_PIN_RESET) ? 1 : 0;
	rel_outputs[7] = (HAL_GPIO_ReadPin(REL8_GPIO_Port, REL8_Pin) == GPIO_PIN_RESET) ? 1 : 0;
}
void Transmit_Rel_Outputs(void){
	uint8_t rels[8];
	Refresh_Rel_Outputs(&rels);
	sprintf(msgReturn, STA_OUT, *( &rels[0]),*( &rels[1]),*( &rels[2]),*( &rels[3]),*( &rels[4]),*( &rels[5]),*( &rels[6]),*( &rels[7]));
	CDC_Transmit_FS(msgReturn, strlen(msgReturn));
}
void Refresh_Opto_Inputs(uint8_t *opto_inputs){
	opto_inputs[0] = (HAL_GPIO_ReadPin(OPT1_GPIO_Port, OPT1_Pin) == GPIO_PIN_RESET) ? 1 : 0;
	opto_inputs[1] = (HAL_GPIO_ReadPin(OPT2_GPIO_Port, OPT2_Pin) == GPIO_PIN_RESET) ? 1 : 0;
	opto_inputs[2] = (HAL_GPIO_ReadPin(OPT3_GPIO_Port, OPT3_Pin) == GPIO_PIN_RESET) ? 1 : 0;
	opto_inputs[3] = (HAL_GPIO_ReadPin(OPT4_GPIO_Port, OPT4_Pin) == GPIO_PIN_RESET) ? 1 : 0;
}
void Transmit_Opto_Inputs(void){
	uint8_t sensors[4];
	Refresh_Opto_Inputs(&sensors);
	sprintf(msgReturn, STA_INP, *( &sensors[0]),*( &sensors[1]),*( &sensors[2]),*( &sensors[3]));
	CDC_Transmit_FS(msgReturn, strlen(msgReturn));
}
//
void Led_Error_Set(uint8_t value) {
	HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, value ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
void Led_Data_Set(uint8_t value) {
	HAL_GPIO_WritePin(LED_DATA_GPIO_Port, LED_DATA_Pin, value ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
//
void REL1_Set(uint8_t value) {
	HAL_GPIO_WritePin(REL1_GPIO_Port, REL1_Pin, value ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
void REL2_Set(uint8_t value) {
	HAL_GPIO_WritePin(REL2_GPIO_Port, REL2_Pin, value ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
void REL3_Set(uint8_t value) {
	HAL_GPIO_WritePin(REL3_GPIO_Port, REL3_Pin, value ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
void REL4_Set(uint8_t value) {
	HAL_GPIO_WritePin(REL4_GPIO_Port, REL4_Pin, value ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
void REL5_Set(uint8_t value) {
	HAL_GPIO_WritePin(REL5_GPIO_Port, REL5_Pin, value ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
void REL6_Set(uint8_t value) {
	HAL_GPIO_WritePin(REL6_GPIO_Port, REL6_Pin, value ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
void REL7_Set(uint8_t value) {
	HAL_GPIO_WritePin(REL7_GPIO_Port, REL7_Pin, value ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
void REL8_Set(uint8_t value) {
	HAL_GPIO_WritePin(REL8_GPIO_Port, REL8_Pin, value ? GPIO_PIN_RESET : GPIO_PIN_SET);
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
