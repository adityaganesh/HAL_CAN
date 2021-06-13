/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include <stdio.h>
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint8_t rec[8];//to recieve the CAN response
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern HCD_HandleTypeDef hhcd_USB_OTG_FS;
extern CAN_HandleTypeDef hcan1;
extern TIM_HandleTypeDef htim10;
/* USER CODE BEGIN EV */

//To see what this variables do refer to main.c private variables
extern UART_HandleTypeDef huart2;
extern CAN_HandleTypeDef HalCan1;
extern CAN_TxHeaderTypeDef pTxHeader;
extern CAN_RxHeaderTypeDef pRxHeader;
extern uint32_t TxMailbox;
extern uint8_t recieved;
extern uint8_t indx;
extern uint8_t req_length ;
extern uint8_t brd_length;
extern uint8_t **trans_data;
extern uint8_t **rec_data;
extern uint8_t **brd_data;//broadcast specifications data array
extern uint8_t **brd_rec;//broadcast recieve data array
extern uint32_t* brd_idnt;//broadcast identifier array
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles CAN1 RX0 interrupts.
  */
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */

  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

  recieved = 0 ;//RESPONSE RECIEVED

  char uart_buf[50];

  int uart_buf_len;

  HAL_CAN_GetRxMessage(&HalCan1,  CAN_RX_FIFO0 , &pRxHeader, rec);//GET THE RESPONSE
  //TRANSFER THE RESPONSE TO TEMPORARILY STORE IN AN DATA ARRAY
  for(uint8_t k = 0; k<8;k++)
  {
	  rec_data[indx][k] = rec[k];
  }

  //THIS WILL HAPPEN ONLY IF WE GET THE RESPONSE FOR THE REQUEST WE SENT WE ALSO NEED TO TAKE CARE OF BROADCAST MESSAGES
  if(rec_data[indx][0] > 0 && rec_data[indx][1] == trans_data[indx][1]+0x40 && trans_data[indx][2] == rec_data[indx][2] )//checks if valid data bytes are more than 0 then checks service and parameter id
  {

	  for(int i = 3;i<=rec_data[indx][0];i++)
	  {
		  uart_buf_len = sprintf(uart_buf,"%x",rec_data[indx][i] );
		  if(i != rec_data[indx][0])
		  {
			  uart_buf_len = sprintf(uart_buf,"%x , ",rec_data[indx][i] );//will print with comma if their are more valid data bytes

		  }
		  else
		  {
			  uart_buf_len = sprintf(uart_buf,"%x\n",rec_data[indx][i] );// will not print comma as it is the last valid data byte

		  }
		  /*UART transmission*/
		  HAL_UART_Transmit(&huart2,(uint8_t *)uart_buf,uart_buf_len,100);


	  }
	  indx ++;
	  if(indx < req_length)
	  {
		  HAL_CAN_AddTxMessage(&HalCan1, &pTxHeader, trans_data[indx], &TxMailbox);//This will send the first transmission request
	      recieved = 1;
	  }
	  while(HAL_CAN_IsTxMessagePending(&HalCan1,TxMailbox));


		 /*UART transmission*/




  }
  //WHEN A BROADCAST MESSAGE IS SENT IT WILL COME HERE
  else
  {
	  uint8_t l = 0;//used so that the whole response is stored according to the parameters into another array

	  for (uint8_t k = 0 ; k  < brd_length ;k++)
	  {
		  if(pRxHeader.ExtId == brd_idnt[k])//checking if the broadcasted message is the one we are looking for
		  {
			     for (uint8_t i = 1 ; i <= brd_data[i][0]; i++)//increasing till the lenght of broadcast parameter length
			     {
			  	   for (uint8_t j  = 0 ;j < brd_data[k][brd_data[i][0]] ; j++)//increasing till the length of the particular parameter's length
			  	   {
			  		   brd_rec[k][l]=rec[i+j];//storing it in our broadcast data recorded array which we will later use to store in sd card
                       l++;
			  	   }

			     }
		  }
	  }

  }
  //INCREASE INDEX FOR NEXT REQUEST

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim10);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
  * @brief This function handles USB On The Go FS global interrupt.
  */
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_HCD_IRQHandler(&hhcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
