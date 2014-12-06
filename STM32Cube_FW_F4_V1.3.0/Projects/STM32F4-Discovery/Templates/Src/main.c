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
#include "AFE4490.h"
#include "stm32f4_tony.h"

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
uint8_t printf_Sending_error[] = " ****sending error****\n\r ";
uint8_t printf_Sending_sucess[] = " ****sending sucess****\n\r ";
uint8_t printf_Receiving_error[] = " ****receiving error****\n\r ";
uint8_t printf_IT_sucess[] = " ****IT sucess****\n\r ";
uint8_t printf_Queue_data_exist[] = " ****Queue data exist****\n\r ";
uint8_t printf_Queue_data_Nonexist[] = " ****Queue data Nonexist****\n\r ";
char aTxBuffer_initialdata[37];
static long * AFE44xxInitialData;
//static long * AFE44xxInitialData_ADDR;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
__IO ITStatus UartTxReady = RESET;
__IO ITStatus UartRxReady = RESET;
unsigned char flagTxCtrl=TX_MODE_SENDING;
unsigned char flagPrintRxContent=RESET;

/* Buffer used for transmission */
//uint8_t aTxBuffer[] = " ****UART_TwoBoards communication based on DMA**** \n How are you today?\n";

uint8_t aTxBuffer[] =// "bb01234567890123456789012345678901234567890123456789012345678901234567890123456789aa\n";
{0x01,0x23,0x45,0x67,0x89,0xab,0xcd,0xef,0xaa,0xbb,0x81,0x00,0x47,0xfb,0xa1,0x0a,0x4a,0x01,0x00,0x00,\
  0x01,0x47,0xfb,0xa1,0x0a,0x4a,0x01,0x00,0x00,0xf4,0x01,0x02,0x00,0x3c,0x00,0x02,0x47,0xfb,0xa1,0x0a,\
  0x4a,0x01,0x00,0x00,0x32,0x00,0x28,0x00,0xbf,0x55,0x03,0x00,0xf5,0x9e,0x0c,0x00,0x3d,0x6a,0x01,0x00,\
  0x84,0x35,0x02,0x00,0x98,0x8c,0x0c,0x00,0xb8,0x14,0x0f,0x00,0xdf,0xa1,0x00,0x00,0xfd,0xbd,0x01,0x00,\
  0x4f,0xfc,0x0a,0x00,0x2e,0x51,0x04,0x00,0x03,0x47,0xfb,0xa1,0x0a,0x4a,0x01,0x00,0x00,0x32,0x00,0x28,\
  0x00,0x30,0x75,0x00,0x00,0x30,0x75,0x00,0x00,0x30,0x75,0x00,0x00,0x30,0x75,0x00,0x00,0x30,0x75,0x00,\
  0x00,0x30,0x75,0x00,0x00,0x30,0x75,0x00,0x00,0x30,0x75,0x00,0x00,0x30,0x75,0x00,0x00,0x30,0x75,0x00,\
  0x00,0xbb,0xaa,0xfe,0xdc,0xba,0x98,0x76,0x54,0x32,0x10,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};/**/

/* Buffer used for reception */
uint8_t aRxBuffer[RXBUFFERSIZE];

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);
void SendDataThread(void const *argument);
/*extern function prototypes---------------------------------*/
extern void  AFE44xx_Init(void);
extern long * AFE44xx_RecData(void);

osSemaphoreId xBinarySemaphore;
osMessageQId xQueue;

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

/*********************************************************************/
void  DelayuS( uint32_t uS )
{
	   uint32_t i;
	   for( i=0; i< uS; i++ )
		 {
				 __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
     }
}
/*********************************************************************

*********************************************************************/
void  DelayMS( uint32_t MS )
{
	   uint32_t i,loop;
	   loop = 1000*MS;
	   for( i=0; i< loop; i++ )
		 {
				 __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
     }	
}
/********************************************************************/
static void AFE44xxInterrupt_Init(void)
{
  HAL_NVIC_SetPriority(ADRDY_EXTI_IRQn,configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);//那1車?RTOS?ao‘那y米??D??車??豕??辰aD?車迆米豕車迆configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY
}	
/********************************************************************/
static void AFE44xxInterrupt_Enable(void)
{
  HAL_NVIC_EnableIRQ(ADRDY_EXTI_IRQn);
}	
/********************************************************************/
//static void AFE4490Interrupt_Disable(void)
//{
//	 HAL_NVIC_DisableIRQ(ADRDY_EXTI_IRQn);
//}	
//*******************************************************/
// UART OUTPUT
/********************************************************/
void UART_printf(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
		if(HAL_UART_Transmit_DMA(huart, pData, Size)!= HAL_OK)
	  {
	    Error_Handler();
	  }
		/*##-3- Wait for the end of the transfer ###################################*/  
	  //while (UartReady != SET)
	  //{
	  //	osDelay(10);
	  //}
	  /* Reset transmission flag */
	  //UartReady = RESET; 
}
/***********************************************************************************/
/***********************************************************************************/
/**
  * @brief  This function handles External line 4 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI4_IRQHandler(void)
{
	static portBASE_TYPE xHigherPriorityTaskWoken;
  HAL_GPIO_EXTI_IRQHandler(AFE44XX_ADRDY_PIN);//?“辰??D??辰y??㏒?GPIO?D?㏒那?????∩ㄓ﹞⊿﹞?那?
	xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xBinarySemaphore, &xHigherPriorityTaskWoken);//﹞⊿3?芍???
	if(xHigherPriorityTaskWoken == pdTRUE)
	{
		vTaskSwitchContext();
	}	
}
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
/*********************************************************/
//*******************************************************//
///*************** AFE44XX IT TASK  CONTROL FOR freeRTOS
//******************************************************//
void vHandlerAFE44xxITTASK(void const *argument)
{
	//portTickType xLastWakeTime;
	//xLastWakeTime = xTaskGetTickCount();
	//const portTickType xTicksToWait = 10/portTICK_RATE_MS;
	for(;;)
	{
		xSemaphoreTake(xBinarySemaphore,portMAX_DELAY); //??豕?芍???
		//xSemaphoreTake(xBinarySemaphore,xTicksToWait);
		AFE44xxInitialData = AFE44xx_RecData();
		//xQueueSendToBack(xQueue, &AFE44xxInitialData,portMAX_DELAY);
		osMessagePut (xQueue, (uint32_t)AFE44xxInitialData, portMAX_DELAY);
		osDelay(10);
		//osDelayUntil(xLastWakeTime,10);
		// UART_printf(&huart1, (uint8_t*)printf_IT_sucess,COUNTOF(printf_IT_sucess)-1);
	}
}
void vReceiveAFE44xxDataTask(void const *argument)
{
	//    portBASE_TYPE xStatus;
	osEvent osEventAFE44XX;
	//portTickType xLastWakeTime;
	//xLastWakeTime = xTaskGetTickCount();
	//const portTickType xTicksToWait = 10/portTICK_RATE_MS;
	for(;;)
	{
		if(uxQueueMessagesWaiting(xQueue) !=0) //?D???車芍D?D那?﹞?車D那y?Y
		{
			//UART_printf(&huart1, (uint8_t*)printf_Queue_data_exist,COUNTOF(printf_Queue_data_exist)-1);
		}
		//xStatus = xQueueReceive(xQueue, &AFE44xxInitialData_ADDR,portMAX_DELAY);
		osEventAFE44XX = osMessageGet (xQueue, portMAX_DELAY);
		//if(xStatus == pdPASS)
		if(osEventAFE44XX.status == osEventMessage)
		{
			//sprintf(aTxBuffer_initialdata,"%lx\n%lx\n%lx\n",*AFE44xxInitialData_ADDR,*(AFE44xxInitialData_ADDR+1),*(AFE44xxInitialData_ADDR+2));
//in order to debug uart sending, weizhong comment the following two lines
//			sprintf(aTxBuffer_initialdata,"%lx\n%lx\n%lx\n",*(long *)osEventAFE44XX.value.v,*((long *)osEventAFE44XX.value.v+1),*((long *)osEventAFE44XX.value.v+2));
//			UART_printf(&huart1, (uint8_t*)aTxBuffer_initialdata,COUNTOF(aTxBuffer_initialdata)-1);  
		}
		else
		{
			//UART_printf(&huart1, (uint8_t*)printf_Receiving_error,COUNTOF(printf_Receiving_error)-1);
		}
		osDelay(10);
	}
}
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
/*
	prvSetupHardware();
  HAL_Init();

  SystemClock_Config();
	BSP_LED_Init(LED3);//3.3V POWER
	BSP_LED_Init(LED0);
	BSP_LED_Init(LED1);
	BSP_LED_Init(LED2);
  	MX_GPIO_Init();
	MX_USART1_UART_Init();
	
debug_putchar(0x21);
debug_putchar(0x22);
debug_putchar(0x23);
debug_putchar(0x24);
debug_putchar(0x25);
debug_putchar(0x26);
debug_putchar(0x27);
 
  osThreadDef(USER_Thread, StartThread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
  osThreadDef(Uart_TxThread_Name, UartTxThread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
  osThreadDef(Uart_RxThread_Name, UartRxThread, osPriorityAboveNormal, 0, configMINIMAL_STACK_SIZE);
  osThreadCreate (osThread(USER_Thread), NULL);
  osThreadCreate (osThread(Uart_TxThread_Name), NULL);
  osThreadCreate (osThread(Uart_RxThread_Name), NULL);
*/

	osMessageQDef(AFE44XXMsg, 2, sizeof(long));
	vSemaphoreCreateBinary(xBinarySemaphore);

	prvSetupHardware();
	HAL_Init();
	SystemClock_Config();  

	BSP_LED_Init(LED3);
	BSP_LED_Init(LED0);
	BSP_LED_Init(LED1);
	BSP_LED_Init(LED2);

	MX_GPIO_Init();  

	MX_USART1_UART_Init();

	AFE44xx_Init();       
	AFE44xxInterrupt_Init();
  
  
	xQueue = osMessageCreate (&os_messageQ_def_AFE44XXMsg, vHandlerAFE44xxITTASK);
	
	AFE44xxInterrupt_Enable()	;
	if(xBinarySemaphore != NULL)
	{

	/* Init code generated for FreeRTOS */
	/* Create Start thread */
	osThreadDef(USER_Thread, StartThread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
	//osThreadDef(Uart_Thread_Name, UartThread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
	osThreadCreate (osThread(USER_Thread), NULL);
	//osThreadCreate (osThread(Uart_Thread_Name), NULL);

	osThreadDef(AFE44xxIT_Thread_Name, vHandlerAFE44xxITTASK, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);	
	osThreadCreate (osThread(AFE44xxIT_Thread_Name), NULL);
		
	osThreadDef(vReceiveAFE44xxDataTask_Thread_Name, vReceiveAFE44xxDataTask, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);	
	osThreadCreate (osThread(vReceiveAFE44xxDataTask_Thread_Name), NULL);	

	osThreadDef(SendData_Thread, SendDataThread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);	
	osThreadCreate (osThread(SendData_Thread), NULL);	
	/* Start scheduler */
	osKernelStart(NULL, NULL);
  	}
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
  huart1.Init.BaudRate = 38400;//9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;//UART_WORDLENGTH_8B;
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
	BSP_LED_On(LED0);
	osDelay(300);
	BSP_LED_Off(LED0);
	osDelay(300);
  }
}

void SendDataThread(void const *argument)
{
//	osEvent bleSendDataEvent;
	uint8_t aTxBuffer20Bytes[20];
	uint16_t uByteCount;
	uint16_t uLoopCount;
	uint8_t uLastLoopByteCount;
	uint8_t *pTxBuffer;
	uint16_t aCnt;
	uint16_t uOffset;
	uByteCount=sizeof(aTxBuffer);
	uLoopCount=uByteCount/20;
	uLastLoopByteCount=(uint8_t)(uByteCount-uLoopCount*20);
	uOffset=0;
	for(;;)
	{
		for(aCnt=0;aCnt<uLoopCount;aCnt++)
		{
			if(HAL_UART_Transmit_DMA(&huart1, (uint8_t *)(aTxBuffer+uOffset), 20)!= HAL_OK)
			  {
			    Error_Handler();
			  }
			uOffset+=20;
			osDelay(100);
		}
		if(uLastLoopByteCount!=0)
		{
			if(HAL_UART_Transmit_DMA(&huart1, (uint8_t *)(aTxBuffer+uOffset), uLastLoopByteCount)!= HAL_OK)
			  {
			    Error_Handler();
			  }
			osDelay(100);
		}
		pTxBuffer=aTxBuffer;
		uOffset=0;
		//feed header
		
		//feed frame bytes number

		//feed body

		//feed hip

		//feed checksum

		//start sending
		osDelay(1000);
		
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
	osDelay(500);
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
