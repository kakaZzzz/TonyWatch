/**
  ******************************************************************************
  * @file    Templates/Src/main.c 
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    26-June-2014
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "misc.h"
#include "cmsis_os.h"
#include "debug_uart.h"
#include <string.h>

/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @addtogroup Templates
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define TX_MODE_SENDING		0
#define TX_MODE_NOT_SEND	1
#define TX_MODE_SEND_ONCE	2
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
__IO ITStatus UartTxReady = RESET;
__IO ITStatus UartRxReady = RESET;
unsigned char flagTxCtrl=TX_MODE_SENDING;
unsigned char flagPrintRxContent=RESET;

/* Buffer used for transmission */
uint8_t aTxBuffer[] = " ****UART_TwoBoards communication based on DMA**** \n How are you today?\n";
/* Buffer used for reception */
uint8_t aRxBuffer[RXBUFFERSIZE];

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);

/* Private functions ---------------------------------------------------------*/
/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
	/* Setup STM32 system (clock, PLL and Flash configuration) */
	SystemInit();

	/* Ensure all priority bits are assigned as preemption priority bits. */
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );

}

void vTask2a( void *pvParameters )
{
	volatile unsigned long ul;	
	for( ;; )
	{		
		BSP_LED_On(LED2);
		for( ul = 0; ul < 5000000; ul++ ) {
		}
		BSP_LED_Off(LED2);
		for( ul = 0; ul < 5000000; ul++ ) {
		}
	}
}

void vTask1a( void *pvParameters )
{
	volatile unsigned long ul;	
	for( ;; )
	{		
		BSP_LED_On(LED0);
		for( ul = 0; ul < 10000000; ul++ ) {
		}
		BSP_LED_Off(LED0);
		for( ul = 0; ul < 10000000; ul++ ) {
		}
	}
}


/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
	prvSetupHardware();
  /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, Flash preread and Buffer caches
       - Systick timer is configured by default as source of time base, but user 
             can eventually implement his proper time base source (a general purpose 
             timer for example or other time source), keeping in mind that Time base 
             duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
             handled in milliseconds basis.
       - Low Level Initialization
     */
  HAL_Init();

  /* Configure the System clock to have a frequency of 168 MHz */
  SystemClock_Config();

  /* Add your application code here
     

//  volatile unsigned long ulaa;
	BSP_LED_Init(LED3);//3.3V POWER
	BSP_LED_Init(LED0);
	BSP_LED_Init(LED1);
	BSP_LED_Init(LED2);
	
	xTaskCreate(vTask1a, "Task 1", 1000, NULL, 1, NULL);	
	xTaskCreate(vTask2a, "Task 2", 1000, NULL, 1, NULL);

	vTaskStartScheduler();*/
	BSP_LED_Init(LED3);//3.3V POWER
	BSP_LED_Init(LED0);
	BSP_LED_Init(LED1);
	BSP_LED_Init(LED2);
	/* Initialize all configured peripherals */
  	MX_GPIO_Init();
	MX_USART1_UART_Init();
	
debug_putchar(0x21);
debug_putchar(0x22);
debug_putchar(0x23);
debug_putchar(0x24);
debug_putchar(0x25);
debug_putchar(0x26);
debug_putchar(0x27);
  /*volatile unsigned long ul;	
 //Infinite loop 
  for(;;)
  {
//	BSP_LED_On(LED2);
//	osDelay(300);
//	BSP_LED_Off(LED2);
//	osDelay(300);
	if(HAL_UART_Transmit_DMA(&huart1, (uint8_t*)aTxBuffer, TXBUFFERSIZE)!= HAL_OK)
	  {
	    Error_Handler();
	  }
	while (UartTxReady != SET)
	  {
	  	osDelay(100);
	  }
	  
	  UartTxReady = RESET;
	  osDelay(2000);
  }*/

  
  /* Init code generated for FreeRTOS */
  /* Create Start thread */
  osThreadDef(USER_Thread, StartThread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
  osThreadDef(Uart_TxThread_Name, UartTxThread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
  osThreadDef(Uart_RxThread_Name, UartRxThread, osPriorityAboveNormal, 0, configMINIMAL_STACK_SIZE);
  osThreadCreate (osThread(USER_Thread), NULL);
  osThreadCreate (osThread(Uart_TxThread_Name), NULL);
  osThreadCreate (osThread(Uart_RxThread_Name), NULL);

  /* Start scheduler */
  osKernelStart(NULL, NULL);
  
  /* Infinite loop */
  while (1)
  {
		
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 168000000
  *            HCLK(Hz)                       = 168000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 336
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  
  /* Enable Power Control clock */
  __PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
  while(1)
  {
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif




/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  huart1.State = HAL_UART_STATE_RESET;//weizhong add 20141031
  huart1.Instance = USARTx;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
if(HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

/** Pinout Configuration
*/
void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __GPIOA_CLK_ENABLE();
}
static void StartThread(void const * argument)
{

  /* USER CODE BEGIN 5 */
 volatile unsigned long ul;	
  /* Infinite loop */
  for(;;)
  {
	BSP_LED_On(LED2);
//	for( ul = 0; ul < 5000000; ul++ ) {
//	}
	osDelay(500);
	BSP_LED_Off(LED2);
//	for( ul = 0; ul < 5000000; ul++ ) {
//	}
	osDelay(500);
  }
}

static void UartTxThread(void const * argument)
{

  /* USER CODE BEGIN 5 */
 volatile unsigned long ul;	
  /* Infinite loop */
  for(;;)
  {
	switch (flagTxCtrl)
	{
		case TX_MODE_SENDING:
			if(HAL_UART_Transmit_DMA(&huart1, (uint8_t*)aTxBuffer, TXBUFFERSIZE)!= HAL_OK)
			  {
			    Error_Handler();
			  }
			break;
		case TX_MODE_NOT_SEND:
			break;
		case TX_MODE_SEND_ONCE:
			if(HAL_UART_Transmit_DMA(&huart1, (uint8_t*)aRxBuffer, RXBUFFERSIZE)!= HAL_OK)
			  {
			    Error_Handler();
			  }
			flagTxCtrl=TX_MODE_NOT_SEND;
			break;
		default:
			break;
	}
	while (UartTxReady != SET)
	{
		osDelay(100);
	}
/*
if(TX_MODE_REPEART== flagTxCtrl)
{
	if(HAL_UART_Transmit_DMA(&huart1, (uint8_t*)aTxBuffer, TXBUFFERSIZE)!= HAL_OK)
	  {
	    Error_Handler();
	  }
		//##-3- Wait for the end of the transfer 
	  while (UartTxReady != SET)
	  {
	  	osDelay(100);
	  }
}
*/
	  /* Reset transmission flag */
	  UartTxReady = RESET;
//	  osDelay(100);
  }
  /* USER CODE END 5 */ 

}

static void UartRxThread(void const * argument)
{
 volatile unsigned long ul;
 uint8_t cmdTxCtrlString_Sending[]="Sending";
 uint8_t cmdTxCtrlString_NotSend[]="NotSend";
 uint8_t cmdTxCtrlString_SendOnce[]="SendOnce";
  for(;;)
  {
	//DO SOMETHING RX
//	BSP_LED_On(LED2);
//	osDelay(300);
//	BSP_LED_Off(LED2);
//	osDelay(300);	
	if(HAL_UART_Receive_DMA(&huart1, (uint8_t *)aRxBuffer, RXBUFFERSIZE) != HAL_OK)
	  {
	    Error_Handler();
	  }
	while(UartRxReady!=SET)
	{
		osDelay(100);
	}
	if(memcmp(aRxBuffer,cmdTxCtrlString_Sending,sizeof(cmdTxCtrlString_Sending)-1)==0)
	{
		flagTxCtrl=TX_MODE_SENDING;
		UartTxReady = SET;
	}
	if(memcmp(aRxBuffer,cmdTxCtrlString_NotSend,sizeof(cmdTxCtrlString_NotSend)-1)==0)
	{
  		flagTxCtrl=TX_MODE_NOT_SEND;
		UartTxReady = RESET;
	}
	if(memcmp(aRxBuffer,cmdTxCtrlString_SendOnce,sizeof(cmdTxCtrlString_SendOnce)-1)==0)
	{
  		flagTxCtrl=TX_MODE_SEND_ONCE;
		UartTxReady = SET;
	}
	UartRxReady = RESET;
//	memcpy(aTxBuffer,aRxBuffer,RXBUFFERSIZE);
//	UartTxReady = SET;
//	osDelay(100);
  }
}


/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle. 
  * @note   This example shows a simple way to report end of DMA Tx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: trasfer complete*/
  UartTxReady = SET;

  /* Turn LED6 on: Transfer in transmission process is correct */
//  BSP_LED_On(LED6); 
}

/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and 
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: trasfer complete*/
  UartRxReady = SET;
  /* Turn LED4 on: Transfer in reception process is correct */
  //BSP_LED_On(LED4);
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
