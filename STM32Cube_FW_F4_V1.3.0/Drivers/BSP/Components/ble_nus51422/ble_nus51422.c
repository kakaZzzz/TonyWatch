#include "ble_nus51422.h"
#include "debug_uart.h"
#include "cmsis_os.h"
#include <string.h>
#include "main.h"
#include "uart.h"

#define TX_MODE_SENDING		0
#define TX_MODE_NOT_SEND	1
#define TX_MODE_SEND_ONCE	2
uint8_t printf_Sending_error[] = " ****sending error****\n\r ";
uint8_t printf_Sending_sucess[] = " ****sending sucess****\n\r ";
uint8_t printf_Receiving_error[] = " ****receiving error****\n\r ";
uint8_t printf_IT_sucess[] = " ****IT sucess****\n\r ";
uint8_t printf_Queue_data_exist[] = " ****Queue data exist****\n\r ";
uint8_t printf_Queue_data_Nonexist[] = " ****Queue data Nonexist****\n\r ";
char aTxBuffer_initialdata[TX_BUF_DATA_LENGTH];

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
uint8_t *aTxPack;
//uint8_t 
UART_HandleTypeDef huart1;
uint8_t flagBleConStatus=BLE_STATUS_DISCONNECTED;

/*******************************************************/



static void UartRxThread(void const * argument)
{
	uint8_t aRxBuffer1[2];
	for(;;)
  	{
		if(HAL_UART_Receive_DMA(&huart1, (uint8_t *)aRxBuffer1, 2) != HAL_OK)
		  {
		    Error_Handler();
		  }
		if((aRxBuffer1[0]==0xFF)&&(aRxBuffer1[1]==0xFF))
		{
			osDelay(1000);
			flagBleConStatus=BLE_STATUS_CONNECTED;
		}
		if((aRxBuffer1[0]==0x55)&&(aRxBuffer1[1]==0x55))
		{
			flagBleConStatus=BLE_STATUS_DISCONNECTED;
		}
		while(UartRxReady!=SET)
		{
			osDelay(100);
		}
		UartRxReady=RESET;
	}
}

void startUartRxThread(void)
{
	osThreadDef(UartRx_ThreadName, UartRxThread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);	
	osThreadCreate (osThread(UartRx_ThreadName), NULL);	
}

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
/**********************************************************
UART Protocol Package
[in] aData: pointer to the data to be packaged
[out] length: pointer to the data finish package
***********************************************************/
void UartPackage(uint8_t *aData, uint16_t length)
{
	
}

void UartUnPackage(void)
{
	
}

void UartSendBySeg(uint8_t *aTxBufferTemp, uint16_t length)
{
//	osEvent bleSendDataEvent;
//	uint8_t aTxBuffer20Bytes[20];
	//uint16_t uByteCount;
	uint16_t uLoopCount;
	uint8_t uLastLoopByteCount;
	uint16_t aCnt;
	uint16_t uOffset;
	//uByteCount=sizeof(aTxBuffer);
	uLoopCount=length/20;
	uLastLoopByteCount=(uint8_t)(length-uLoopCount*20);
	uOffset=0;
	for(;;)
	{
		for(aCnt=0;aCnt<uLoopCount;aCnt++)
		{
			if(HAL_UART_Transmit_DMA(&huart1, (uint8_t *)(aTxBufferTemp+uOffset), 20)!= HAL_OK)
			  {
			    Error_Handler();
			  }
			uOffset+=20;
			osDelay(100);
		}
		if(uLastLoopByteCount!=0)
		{
			if(HAL_UART_Transmit_DMA(&huart1, (uint8_t *)(aTxBufferTemp+uOffset), uLastLoopByteCount)!= HAL_OK)
			  {
			    Error_Handler();
			  }
			osDelay(100);
		}
		uOffset=0;
		osDelay(1000);
	}
}

void SendDataThread(void const *argument)
{
	UartSendBySeg(aTxBuffer, sizeof(aTxBuffer));
}

void HeartRateSendThread(void const *argument)
{
	uint16_t uByteCount;
	uint16_t uLoopCount;
	uint8_t uLastLoopByteCount;
	uint16_t aCnt;
	uint16_t uOffset;
	portTickType hrTicks = 5000 / portTICK_RATE_MS;
	portBASE_TYPE hrSemaReturn;
	for(;;)
	{
		hrSemaReturn=xSemaphoreTake(xBinSemaHeartRateSend,hrTicks);
		if(hrSemaReturn==pdPASS)
		{
			aCnt=0;
		}
		else
		{
			aCnt=1;
		}
		uByteCount=sizeof(gHeartRateTxPackBuf);
		uLoopCount=uByteCount/20;
		uLastLoopByteCount=(uint8_t)(uByteCount-uLoopCount*20);
		uOffset=0;
		for(aCnt=0;aCnt<uLoopCount;aCnt++)
		{
			if(HAL_UART_Transmit_DMA(&huart1, (uint8_t *)(gHeartRateTxPackBuf+uOffset), 20)!= HAL_OK)
			  {
			    //Error_Handler();
			  }
			uOffset+=20;
			//osDelay(100);
		}
		if(uLastLoopByteCount!=0)
		{
			if(HAL_UART_Transmit_DMA(&huart1, (uint8_t *)(gHeartRateTxPackBuf+uOffset), uLastLoopByteCount)!= HAL_OK)
			  {
			    //Error_Handler();
			  }
			//osDelay(100);
		}
		uOffset=0;
		//osDelay(100);
//		xSemaphoreGive(xBinSemaHeartRateSend);
	}
}
/**/
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

	  UartTxReady = RESET;
//	  osDelay(100);
  }
  /* USER CODE END 5 */ 

}
/*
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
}*/




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

void TxSampleSend(uint8_t *pData, uint16_t length)
{
//	if(HAL_UART_Transmit_DMA(&huart1, (uint8_t*)pData, length)!= HAL_OK)
//	{
//		Error_Handler();
//	}
	while(HAL_UART_Transmit_DMA(&huart1, (uint8_t*)pData, length)!= HAL_OK)
	{
		osDelay(10);
	}
	while (UartTxReady != SET)
	{
		osDelay(10);
	}
}




