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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
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
CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
CAN_HandleTypeDef HalCan1;
CAN_RxHeaderTypeDef pRxHeader; //This is used to give essential information about reception message
CAN_TxHeaderTypeDef pTxHeader; // This is used to give essential information about transmission message
//Data array used to store received data
uint32_t TxMailbox;
char uart_buf[50];//used for logging data to serial monitor
int uart_buf_len;
/*This multidimensional data array has all the requests payload so that it is easy to create the payload frame*/


uint8_t tim_flag = 0;
uint8_t recieved = 0;
uint8_t indx = 0 ;
uint8_t req_length = 5;//no.of requests
uint8_t brd_length = 4;//no.of broadcast responses
uint8_t response = 0;
uint8_t broadcast = 0;
uint8_t **trans_data;//transmission data array
uint8_t **rec_data;//reception data array
uint8_t **brd_data;//broadcast specifications data array
uint8_t **brd_rec;//broadcast recieve data array
uint32_t* brd_idnt;//broadcast identifier array
uitn8_t res_indx = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM10_Init(void);
static void MX_USART2_UART_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

/*Use this method to create your desired request
 *
 * pTxheader : CAN_TxHeaderTypeDef structure is used to specify important setting such as extended identifier , data length
 * identifier : EXTENDED IDENTIFIER OF REQUEST
 * length : LENGTH OF VALID BYTES
 * service_id : SERVICE ID OF YOUR REQUEST
 * parameter_id : PARAMETER ID OF YOUR REQUEST
 * request_no : Index of the request you want to create
 *
 * */
void create_request(CAN_TxHeaderTypeDef pTxHeader ,uint32_t identifier, uint8_t length , uint8_t service_id , uint8_t parameter_id,uint8_t request_no,uint8_t request_length);

/*
 *
 * identifier : Extended Identifier of the broadcasted response
 * indx: Index of the broadcasted response
 * length: Length of the brd parameter in frame
 * pass the position and length of parameter according to the length of brd parameters in frame
 * Note: Use this is in system intitalization part of code
 *
 * */

void brd_param(uint32_t identifier , uint8_t indx,uint8_t length,... );

/*
 * length: The number of broadcasted responses you are expecting
 * Note:- Use this before using brd_param
 *
 * */

void num_brd(uint8_t length);

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

  //Data array that needs to be transmitted
  trans_data = (uint8_t **)calloc(req_length , sizeof(uint8_t *));

  //Data array that will be recieved
  rec_data = (uint8_t **)calloc(req_length , sizeof(uint8_t *));

  //Broadcast parameter data array
  brd_data = (uint8_t **)calloc(brd_length , sizeof(uint8_t *));

  //Broadcast response array
  brd_rec = (uint8_t **)calloc(brd_length , sizeof(uint8_t *));

  //Broadcast message identifier array
  brd_idnt = (uint32_t*)calloc(brd_length , sizeof(uint32_t));

  /*Dynamic Memory allocation of the above declared arrays*/
  if (trans_data == NULL)
  {
          printf("Memory not allocated.\n");

  }
  if (rec_data == NULL)
  {
          printf("Memory not allocated.\n");

  }
  if (brd_data == NULL)
  {
          printf("Memory not allocated.\n");

  }
  if (brd_rec == NULL)
  {
          printf("Memory not allocated.\n");

  }
  for (uint8_t i=0; i<req_length; i++)
    {
	  trans_data[i] = (uint8_t *)calloc(8 , sizeof(uint8_t));

              if (trans_data[i] == NULL)
              {
            	  printf("Memory not allocated.\n");


              }


    }
  for (uint8_t i=0; i<req_length; i++)
    {
  	  rec_data[i] = (uint8_t *)calloc(8 , sizeof(uint8_t));

                if (rec_data[i] == NULL)
                {
              	  printf("Memory not allocated.\n");


                }


    }
  for (uint8_t i=0; i<brd_length; i++)
    {
  	  brd_data[i] = (uint8_t *)calloc(17 , sizeof(uint8_t));

                if (brd_data[i] == NULL)
                {
              	  printf("Memory not allocated.\n");


                }


    }
  for (uint8_t i=0; i<brd_length; i++)
    {
  	  brd_rec[i] = (uint8_t *)calloc(8 , sizeof(uint8_t));

                if (brd_rec[i] == NULL)
                {
              	  printf("Memory not allocated.\n");


                }


    }
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  create_request(pTxHeader,0x18DAFA00,0x21, 0x30, 0x1,1,5); // Creation of request 1

  create_request(pTxHeader,0x18DAFA00,0x21, 0x26, 0x2,2,5);  // Creation of request 2

  create_request(pTxHeader,0x18DAFA00,0x21, 0x24, 0x1,3,5);  // Creation of request 3

  create_request(pTxHeader,0x18DAFA00,0x21, 0x9a, 0x2,4,5);  // Creation of request 4

  create_request(pTxHeader,0x18DAFA00,0x21, 0x3d, 0x1,5,5);  // Creation of request 5

  num_brd(4); // Specifying the number of broadcast messages we would be expecting

  brd_param(0xF00400,1, 3, 2, 4, 5, 2, 1, 2); // Creation of Broadcast parameter frame 1

  brd_param(0xF00300,2, 2, 1, 3, 1, 2); // Creation of Broadcast parameter frame 2

  brd_param(0xFEFF00,3, 1, 4, 1); // Creation of Broadcast parameter frame 3

  brd_param(0xF0F400,4, 1, 3, 2); // Creation of Broadcast parameter frame 4
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USB_HOST_Init();
  MX_CAN1_Init();
  MX_TIM10_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_CAN_Start(&HalCan1);//Begin the CAN module

  HAL_CAN_ActivateNotification(&HalCan1,  CAN_IT_RX_FIFO0_MSG_PENDING );//This enables the interrupts of our CAN module

  HAL_TIM_Base_Start_IT(&htim10);//Begin timer in interrupt mode see the ioc file to see the time at which interrupt is generated



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      if(response == 1)//this means we have recieved a message which is ready to be logged

      {
    	  for(int i = 3;i<=rec_data[res_indx][0];i++)
    	  {

    		  if(i < rec_data[res_indx][0])
    		  {
    			  uart_buf_len = sprintf(uart_buf,"%x , ",rec_data[res_indx][i] );
    			  f_write(&data_log,uart_buf_len,sizeof(uart_buf_len) );//will print with comma if their are more valid data bytes

    		  }
    		  else
    		  {

    			  uart_buf_len = sprintf(uart_buf,"%x\n",rec_data[indx][i] );
    			  f_write(&data_log,uart_buf_len,sizeof(uart_buf_len) );// will not print comma as it is the last valid data byte

    		  }
    		  /*UART transmission*/



    	  }
    	  //response has been logged waiting for next response
    	  res_indx ++;//increase the index so that next response is logged

    	  /*PRINT THE RESPONSE FOR INDEX-1 request*/
      }
      if(brd_indx == brd_length)
      {
    	  broadcast = 0; //since all the messages have been recieved
          brd_indx = 0;//reseting the index to 0 so that it will put data in correct index during next cycle of broadcast messages
         /*START WRITING ALL THE BROADCAST DATA ON TO SD CARD*/

      }
	  if(res_indx == req_length)
		{
          response = 0; //all responses is logged onto sd card
		  tim_flag = 0; //reset timer flag
		  indx = 0 ; //reset index
		  res_indx = 0;//reset response log index
		}
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
  hcan1.Init.Prescaler = 21;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
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
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

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
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 168-1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 2500-1;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
                          |LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD10 PD11 
                           LD4_Pin LD3_Pin LD5_Pin LD6_Pin 
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
                          |LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */
/*THIS METHOD WILL BE CALLED EVERY TIME OUR COUNTER GENERATES A INPUT THAT IS AT THE END OF ITS FULL COUNT i.e 250 millisecond*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if(htim == &htim10)
	{
        /*Raising the timer flag so that code in while loop starts running since ISR should be generally fast to execute*/

		 tim_flag = 1 ;
		 broadcast = 1;//we can recieve broadcast messages now
         HAL_CAN_AddTxMessage(&HalCan1, &pTxHeader, trans_data[indx], &TxMailbox);//This will send the first transmission request
		 /*Visual verification of timer working*/
		 HAL_GPIO_TogglePin(GPIOA , GPIO_PIN_5);


	}

}

/*  DEFINTION OF REQUEST CREATION FUNCTION  */
void create_request(CAN_TxHeaderTypeDef pTxHeader,uint32_t identifier ,uint8_t length , uint8_t service_id , uint8_t parameter_id,uint8_t request_no,uint8_t request_length)
{
	 pTxHeader.IDE =CAN_ID_EXT;//using extended identifier
	 pTxHeader.ExtId = identifier;//setting the identifier
	 pTxHeader.DLC = 8;//length of payload in bytes
	 pTxHeader.RTR= CAN_RTR_DATA;

	 /*Creating according to request number*/
	 if(request_length > req_length)
	 {

	  req_length = request_length;//changing the length of request frames to be sent

	  /*Dynamic memory allocation of the data arrays according to the length*/
	  trans_data = realloc(trans_data,req_length*sizeof(uint8_t *));

	  rec_data = realloc(rec_data,req_length*sizeof(uint8_t *));
	  for (uint8_t i=0; i<req_length; i++)
	  {


		  trans_data[i] = (uint8_t *)calloc(8 , sizeof(uint8_t));

		  if (trans_data[i] == NULL)
		  {
			  printf("Memory not allocated.\n");


		  }


	    }



	  for (uint8_t i=0; i<req_length; i++)
	  {


		  rec_data[i] = (uint8_t *)calloc(8 , sizeof(uint8_t));

		  if (rec_data[i] == NULL)
		  {
			  printf("Memory not allocated.\n");


		  }


	    }



     }
     //Storing the required parameters that needs to be transmitted
	 trans_data[request_no - 1 ][0] = length;
	 trans_data[request_no - 1 ][1] = service_id;
	 trans_data[request_no - 1 ][2] = parameter_id;


}

/*  DEFINTION OF BROADCAST PARAMETER FRAME CREATION FUNCTION  */
void brd_param(uint32_t identifier , uint8_t indx,uint8_t length , ...)
{

  va_list parameters;

  va_start(parameters,length*2);

  brd_idnt[ indx - 1 ] = identifier;//storing the identifier in the broadcast identifier array

  brd_data[ indx - 1 ][0] = length;//storing the length of broadcast specification array

  for (uint8_t i = 0; i <= length*2; i++)
  {
	  brd_data[indx-1][i+1] = (uint8_t)va_arg(parameters, int);//storing the parameters of the broadcast parameter


  }


  va_end(parameters);


}

void num_brd(uint8_t length)
{
	 if(length > brd_length)
	 {

	  brd_length = length;//changing the length of broadcast parameter frame expected

	  /*Dynamic memory allocation for the arrays according to the length*/
	  brd_data = realloc(brd_data,length*sizeof(uint8_t *));

	  brd_rec = realloc(brd_rec,length*sizeof(uint8_t *));

	  brd_idnt = realloc(brd_idnt,length*sizeof(uint32_t *));

	  for (uint8_t i=0; i<length; i++)
	  {


		  brd_data[i] = (uint8_t *)calloc(17 , sizeof(uint8_t));

		  brd_rec[i] = (uint8_t *)calloc(8 , sizeof(uint8_t));

		  if (brd_data[i] == NULL)
		  {
			  printf("Memory not allocated.\n");


		  }
		  if (brd_rec[i] == NULL)
		  {
			  printf("Memory not allocated.\n");


		  }


	    }


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
