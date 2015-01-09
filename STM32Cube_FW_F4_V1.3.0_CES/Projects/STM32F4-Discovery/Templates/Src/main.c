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
#include <string.h>
#include "AFE4490.h"
#include "stm32f4_tony.h"
#include "ble_nus51422.h"
#include "stdio.h"

/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @addtogroup Templates
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/


typedef struct
{
	uint32_t sec;
	uint32_t msec;
}hr_time_t;

typedef struct
{
	uint8_t type;
	hr_time_t time;
	uint16_t period;
	uint16_t length;
}hr_data_header_t;

static long * AFE44xxInitialData;
//static long * AFE44xxInitialData_ADDR;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
void SendDataThread(void const *argument);
//void HeartRateSendThread(void const *argument);
void packHeartRateData(osEvent data);
/*extern function prototypes---------------------------------*/
extern void  AFE44xx_Init(void);
extern long * AFE44xx_RecData(void);

osSemaphoreId xBinarySemaphore;
osMutexId upQueueMutexSema;
osMutexDef_t upQueueMutex;
//osSemaphoreId xBinSemaHeartRateSend;
osMessageQId xQueue;

uint8_t aPackHeader[]={0x01,0x23,0x45,0x67,0x89,0xAB,0xCD,0xEF,0xAA,0xBB};
uint8_t aPackHip[]={0xBB,0xAA,0xFE,0xDC,0xBA,0x98,0x76,0x54,0x32,0x10};
uint32_t gSystemTimeInS=0;
uint32_t gSystemTimeInMS=0;
uint8_t gHeartRateTxPackBuf[HR_PACK_LENGTH];
uint8_t gHeartRateTxPack[HR_PACK_LENGTH];
uint8_t gHeartRateTxPackCnt=0;
uint8_t gPackStatus;
uint16_t gHeartRate=121;
uint8_t gDataClass=0;
uint8_t gPackCnt=1;
uint8_t gHRData=0;
uint8_t gPROData=0;


//	hr_data_header_t gHeartRateValue;
//	hr_data_header_t gHeartRateRaw;
//	hr_data_header_t gHeartRateProcess;
#define SIZEOF_DATA1 	2//sizeof(gHeartRate)	
#define SIZEOF_DATA2		4	
#define SIZEOF_DATA3		4
#define DATA_HEADER_LENGTH									13
#define HEART_RATE_VALUE_STRUCT_LENGTH				(DATA_HEADER_LENGTH+SIZEOF_DATA1)		//15
#define HEART_RATE_RAW_STRUCT_LENGTH					(DATA_HEADER_LENGTH+SIZEOF_DATA2*10)	//53
#define HEART_RATE_PORCESS_STRUCT_LENGTH			(DATA_HEADER_LENGTH+SIZEOF_DATA3*10)	//53
	uint8_t gHeartRateValueHeader[DATA_HEADER_LENGTH];
	uint8_t gHeartRateRawHeader[DATA_HEADER_LENGTH];
	uint8_t gHeartRateProcessHeader[DATA_HEADER_LENGTH];
#define PACK_STATUS_HEADER 1
#define PACK_STATUS_BODY 2
#define HR_BLOCK0_BASE		0
#define HR_BLOCK1_BASE		(HR_BLOCK0_BASE+sizeof(sysTime))//8
#define HR_BLOCK2_BASE		(HR_BLOCK1_BASE+HEART_RATE_VALUE_STRUCT_LENGTH)//23
#define HR_BLOCK3_BASE		(HR_BLOCK2_BASE+HEART_RATE_RAW_STRUCT_LENGTH)//76
	uint8_t ghr_block1_offset=0;
	uint8_t ghr_block2_offset=0;
	uint8_t ghr_block3_offset=0;

//	uint8_t gSizeofHRV;
//	uint8_t gSizeofHRR;
//	uint8_t gSizeofHRP;
	
	hr_time_t sysTime;
/* Private functions ---------------------------------------------------------*/
/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
	/* Setup STM32 system (clock, PLL and Flash configuration) */
	SystemInit();

	/* Ensure all priority bits are assigned as preemption priority bits. */
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );

}

void vTask2a(void const *argument)
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

void vTask3a(void const *argument)
{
	volatile unsigned long ul;	
	for( ;; )
	{	
		if(gHRData>=160)
		{
			gHRData=55;
		}
		else
		{
			gHRData++;			
		}
//		gPROData++;
		osDelay(2000);
	}
}

void vTask1a(void const *argument)
{
	volatile unsigned long ul;	
	for( ;; )
	{	/*	
		BSP_LED_On(LED0);
		for( ul = 0; ul < 10000000; ul++ ) {
		}
		BSP_LED_Off(LED0);
		for( ul = 0; ul < 10000000; ul++ ) {
		}*/
		gPROData++;
		osDelay(100);
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
		xSemaphoreGive(xBinarySemaphore);
		osDelay(10);
		//osDelayUntil(xLastWakeTime,10);
		// UART_printf(&huart1, (uint8_t*)printf_IT_sucess,COUNTOF(printf_IT_sucess)-1);
	}
}
void vReceiveAFE44xxDataTask(void const *argument)
{
	//    portBASE_TYPE xStatus;
	osEvent osEventAFE44XX;
//	uint8_t i=0;
//	uint32_t hrValue;
	//portTickType xLastWakeTime;
	//xLastWakeTime = xTaskGetTickCount();
	//const portTickType xTicksToWait = 10/portTICK_RATE_MS;
	//uint8_t TxSample[128];
	//uint8_t *pPara1;
	//for(uint8_t i=0;i<128;i++)
	//{
	//	TxSample[i]=i;
	//}
	for(;;)
	{
		if(uxQueueMessagesWaiting(xQueue) !=0) //?D???車芍D?D那?﹞?車D那y?Y
		{
			//UART_printf(&huart1, (uint8_t*)printf_Queue_data_exist,COUNTOF(printf_Queue_data_exist)-1);
		}
		//xStatus = xQueueReceive(xQueue, &AFE44xxInitialData_ADDR,portMAX_DELAY);
		xSemaphoreTake(xBinarySemaphore,portMAX_DELAY); 
		osEventAFE44XX = osMessageGet (xQueue, portMAX_DELAY);
		xSemaphoreGive(xBinarySemaphore);
		//if(xStatus == pdPASS)
		if(osEventAFE44XX.status == osEventMessage)
		{
		/*	if(i==0)//0
			{
				//take upQueue sema
				hrValue=*((long *)osEventAFE44XX.value.v+1);
				app_fifo_put(&upQueue,BREAK_UINT32(hrValue, 0));
				hrValue=*((long *)osEventAFE44XX.value.v);
				app_fifo_put(&upQueue,BREAK_UINT32(hrValue, 0));
				i++;
				//give upQueue sema
			}
			else if((i>0)&&(i<10))// 1-9
			{
				//take upQueue sema
				hrValue=*((long *)osEventAFE44XX.value.v);
				app_fifo_put(&upQueue,BREAK_UINT32(hrValue, 0));
				i++;
				//give upQueue sema
			}
			else
			{}
			if(i>=10)// 9
			{
				i=0;
			}			*/
			//sprintf(aTxBuffer_initialdata,"%lx\n%lx\n%lx\n",*AFE44xxInitialData_ADDR,*(AFE44xxInitialData_ADDR+1),*(AFE44xxInitialData_ADDR+2));
//in order to debug uart sending, weizhong comment the following two lines

//			sprintf(aTxBuffer_initialdata,"%lx\n%lx\n%lx\n",*(long *)osEventAFE44XX.value.v,*((long *)osEventAFE44XX.value.v+1),*((long *)osEventAFE44XX.value.v+2));
//			UART_printf(&huart1, (uint8_t *)aTxBuffer_initialdata,COUNTOF(aTxBuffer_initialdata)-1);  

//			pPara1=osEventAFE44XX.value.p;
//			if(flagBleConStatus==BLE_STATUS_CONNECTED)
//			{
//				TxSampleSend(aPackHeader,10);
//				TxSampleSend(pPara1,12);
//				TxSampleSend(aPackHip,10);
//			}
			if(flagBleConStatus==BLE_STATUS_CONNECTED)
			{
				packHeartRateData(osEventAFE44XX);
			}
			else
			{
				gPackCnt=0;
			}
		}
		else
		{
			//UART_printf(&huart1, (uint8_t*)printf_Receiving_error,COUNTOF(printf_Receiving_error)-1);
		}
		osDelay(50);
	}
}
/*
Block0: 	0~7 		//start time,length=8B		offset=0
Block1:		8~22 		//data1,length=15B			offset=8
Block2:		23~75 		//data2,length=53B			offset=23
Block3:		76~128	//data3,length=53B			offset=76

*/

void initHeartRateDataType(void)
{
	gHeartRateValueHeader[0]=0x01;
	gHeartRateValueHeader[1]=0x11;
	gHeartRateValueHeader[2]=0x22;
	gHeartRateValueHeader[3]=0x33;
	gHeartRateValueHeader[4]=0x44;
	gHeartRateValueHeader[5]=0x55;
	gHeartRateValueHeader[6]=0x66;
	gHeartRateValueHeader[7]=0x77;
	gHeartRateValueHeader[8]=0x88;
	gHeartRateValueHeader[9]=0xF4;
	gHeartRateValueHeader[10]=0x01;
	gHeartRateValueHeader[11]=0x02;
	gHeartRateValueHeader[12]=0x00;
	
	gHeartRateRawHeader[0]=0x02;
	gHeartRateRawHeader[1]=0x11;
	gHeartRateRawHeader[2]=0x22;
	gHeartRateRawHeader[3]=0x33;
	gHeartRateRawHeader[4]=0x44;
	gHeartRateRawHeader[5]=0x55;
	gHeartRateRawHeader[6]=0x66;
	gHeartRateRawHeader[7]=0x77;
	gHeartRateRawHeader[8]=0x88;
	gHeartRateRawHeader[9]=0x32;
	gHeartRateRawHeader[10]=0x00;
	gHeartRateRawHeader[11]=0x28;
	gHeartRateRawHeader[12]=0x00;
	
	gHeartRateProcessHeader[0]=0x03;
	gHeartRateProcessHeader[1]=0x11;
	gHeartRateProcessHeader[2]=0x22;
	gHeartRateProcessHeader[3]=0x33;
	gHeartRateProcessHeader[4]=0x44;
	gHeartRateProcessHeader[5]=0x55;
	gHeartRateProcessHeader[6]=0x66;
	gHeartRateProcessHeader[7]=0x77;
	gHeartRateProcessHeader[8]=0x88;
	gHeartRateProcessHeader[9]=0x32;
	gHeartRateProcessHeader[10]=0x00;
	gHeartRateProcessHeader[11]=0x28;
	gHeartRateProcessHeader[12]=0x00;

	sysTime.sec=0x11223344;
	sysTime.msec=0x55667788;
}
	
void packHeartRateData(osEvent data)
{
	uint32_t data0=*((long *)data.value.v+0);
	uint32_t data1=*((long *)data.value.v+1);
//	uint32_t data2=*((long *)data.value.v+2);
	
	if(gPackCnt==1)
	{
		memset(gHeartRateTxPack,0,HR_PACK_LENGTH);//clear package
		ghr_block1_offset=0;
		ghr_block2_offset=0;
		ghr_block3_offset=0;
		
		memcpy(gHeartRateTxPack+HR_BLOCK0_BASE,&sysTime,sizeof(sysTime));//start time
		
//		memcpy(gHeartRateTxPack+HR_BLOCK1_BASE,&gHeartRateValue,sizeof(gHeartRateValue));//header
//		ghr_block1_offset=HR_BLOCK1_BASE+sizeof(gHeartRateValue);
//		memcpy(gHeartRateTxPack+ghr_block1_offset,&gHeartRate,sizeof(gHeartRate));//data
		memcpy(gHeartRateTxPack+HR_BLOCK1_BASE,&gHeartRateValueHeader,DATA_HEADER_LENGTH);//data1 header
		ghr_block1_offset=HR_BLOCK1_BASE+DATA_HEADER_LENGTH;
		memcpy(gHeartRateTxPack+ghr_block1_offset,&gHeartRate,SIZEOF_DATA1);//piece1 of data1
		ghr_block1_offset+=SIZEOF_DATA1;
		
//		memcpy(gHeartRateTxPack+HR_BLOCK2_BASE,&gHeartRateRaw,sizeof(gHeartRateRaw));//header
//		ghr_block2_offset=HR_BLOCK2_BASE+sizeof(gHeartRateRaw);
//		memcpy(gHeartRateTxPack+ghr_block2_offset,&data0,sizeof(data0));//data
		memcpy(gHeartRateTxPack+HR_BLOCK2_BASE,&gHeartRateRawHeader,DATA_HEADER_LENGTH);//header
		ghr_block2_offset=HR_BLOCK2_BASE+DATA_HEADER_LENGTH;
		memcpy(gHeartRateTxPack+ghr_block2_offset,&data0,SIZEOF_DATA2);//data
		ghr_block2_offset+=SIZEOF_DATA2;
		
//		memcpy(gHeartRateTxPack+HR_BLOCK3_BASE,&gHeartRateProcess,sizeof(gHeartRateProcess));//header
//		ghr_block3_offset=HR_BLOCK3_BASE+sizeof(gHeartRateProcess);
//		memcpy(gHeartRateTxPack+ghr_block3_offset,&data1,sizeof(data1));//data
		memcpy(gHeartRateTxPack+HR_BLOCK3_BASE,&gHeartRateProcessHeader,DATA_HEADER_LENGTH);//header
		ghr_block3_offset=HR_BLOCK3_BASE+DATA_HEADER_LENGTH;
		memcpy(gHeartRateTxPack+ghr_block3_offset,&data1,SIZEOF_DATA3);//data
		ghr_block3_offset+=SIZEOF_DATA3;
	}
	else if((gPackCnt>=2)&&(gPackCnt<=10))
	{
//		memcpy(gHeartRateTxPack+ghr_block1_offset+sizeof(gHeartRate)*(gPackCnt-1),&gHeartRate,sizeof(gHeartRate));
//		memcpy(gHeartRateTxPack+ghr_block2_offset+sizeof(gHeartRateRaw)*(gPackCnt-1),&gHeartRateRaw,sizeof(gHeartRateRaw));
//		memcpy(gHeartRateTxPack+ghr_block3_offset+sizeof(gHeartRateProcess)*(gPackCnt-1),&gHeartRateProcess,sizeof(gHeartRateProcess));

		//HeartRateValue only has one data in 500ms, so here we comments it out
//		memcpy(gHeartRateTxPack+ghr_block1_offset+sizeof(gHeartRate)*(gPackCnt-1),&gHeartRate,sizeof(gHeartRate));
		memcpy(gHeartRateTxPack+ghr_block2_offset,&data0,SIZEOF_DATA2);
		ghr_block2_offset+=SIZEOF_DATA2;
		memcpy(gHeartRateTxPack+ghr_block3_offset,&data1,SIZEOF_DATA3);
		ghr_block3_offset+=SIZEOF_DATA3;
	}
	else
	{
		
	}

	if(gPackCnt==10)
	{
		memcpy(gHeartRateTxPackBuf,gHeartRateTxPack,HR_PACK_LENGTH);
		gPackCnt=0;
		flagHRDSend=true;
//		xSemaphoreGive(xBinSemaHeartRateSend);
	}
	gPackCnt++;
	/*
	switch(gPackStatus)
	{
		case PACK_STATUS_HEADER:
			//part1: start time
			gHeartRateTxPack[0+HR_BLOCK0_OFFSET]=BREAK_UINT32(gSystemTimeInS,0);
			gHeartRateTxPackCnt++;
			gHeartRateTxPack[1+HR_BLOCK0_OFFSET]=BREAK_UINT32(gSystemTimeInS,1);
			gHeartRateTxPackCnt++;
			gHeartRateTxPack[2+HR_BLOCK0_OFFSET]=BREAK_UINT32(gSystemTimeInS,2);
			gHeartRateTxPackCnt++;
			gHeartRateTxPack[3+HR_BLOCK0_OFFSET]=BREAK_UINT32(gSystemTimeInS,3);
			gHeartRateTxPackCnt++;
			gHeartRateTxPack[4+HR_BLOCK0_OFFSET]=BREAK_UINT32(gSystemTimeInMS,0);
			gHeartRateTxPackCnt++;
			gHeartRateTxPack[5+HR_BLOCK0_OFFSET]=BREAK_UINT32(gSystemTimeInMS,1);
			gHeartRateTxPackCnt++;
			gHeartRateTxPack[6+HR_BLOCK0_OFFSET]=BREAK_UINT32(gSystemTimeInMS,2);
			gHeartRateTxPackCnt++;
			gHeartRateTxPack[7+HR_BLOCK0_OFFSET]=BREAK_UINT32(gSystemTimeInMS,3);
			gHeartRateTxPackCnt++;
			//part2: data type
			gHeartRateTxPack[HR_BLOCK1_OFFSET+0]=0x01;
			gHeartRateTxPack[HR_BLOCK2_OFFSET+0]=0x02;
			gHeartRateTxPack[HR_BLOCK3_OFFSET+0]=0x03;
			//part3: data start time
			gHeartRateTxPack[HR_BLOCK1_OFFSET+1]=BREAK_UINT32(gSystemTimeInS,0);
			gHeartRateTxPack[HR_BLOCK1_OFFSET+2]=BREAK_UINT32(gSystemTimeInS,1);
			gHeartRateTxPack[HR_BLOCK1_OFFSET+3]=BREAK_UINT32(gSystemTimeInS,2);
			gHeartRateTxPack[HR_BLOCK1_OFFSET+4]=BREAK_UINT32(gSystemTimeInS,3);
			gHeartRateTxPack[HR_BLOCK1_OFFSET+5]=BREAK_UINT32(gSystemTimeInMS,4);
			gHeartRateTxPack[HR_BLOCK1_OFFSET+6]=BREAK_UINT32(gSystemTimeInMS,5);
			gHeartRateTxPack[HR_BLOCK1_OFFSET+7]=BREAK_UINT32(gSystemTimeInMS,6);
			gHeartRateTxPack[HR_BLOCK1_OFFSET+8]=BREAK_UINT32(gSystemTimeInMS,7);
			
			gHeartRateTxPack[HR_BLOCK2_OFFSET+1]=BREAK_UINT32(gSystemTimeInS,0);
			gHeartRateTxPack[HR_BLOCK2_OFFSET+2]=BREAK_UINT32(gSystemTimeInS,1);
			gHeartRateTxPack[HR_BLOCK2_OFFSET+3]=BREAK_UINT32(gSystemTimeInS,2);
			gHeartRateTxPack[HR_BLOCK2_OFFSET+4]=BREAK_UINT32(gSystemTimeInS,3);
			gHeartRateTxPack[HR_BLOCK2_OFFSET+5]=BREAK_UINT32(gSystemTimeInMS,4);
			gHeartRateTxPack[HR_BLOCK2_OFFSET+6]=BREAK_UINT32(gSystemTimeInMS,5);
			gHeartRateTxPack[HR_BLOCK2_OFFSET+7]=BREAK_UINT32(gSystemTimeInMS,6);
			gHeartRateTxPack[HR_BLOCK2_OFFSET+8]=BREAK_UINT32(gSystemTimeInMS,7);
			
			gHeartRateTxPack[HR_BLOCK3_OFFSET+1]=BREAK_UINT32(gSystemTimeInS,0);
			gHeartRateTxPack[HR_BLOCK3_OFFSET+2]=BREAK_UINT32(gSystemTimeInS,1);
			gHeartRateTxPack[HR_BLOCK3_OFFSET+3]=BREAK_UINT32(gSystemTimeInS,2);
			gHeartRateTxPack[HR_BLOCK3_OFFSET+4]=BREAK_UINT32(gSystemTimeInS,3);
			gHeartRateTxPack[HR_BLOCK3_OFFSET+5]=BREAK_UINT32(gSystemTimeInMS,4);
			gHeartRateTxPack[HR_BLOCK3_OFFSET+6]=BREAK_UINT32(gSystemTimeInMS,5);
			gHeartRateTxPack[HR_BLOCK3_OFFSET+7]=BREAK_UINT32(gSystemTimeInMS,6);
			gHeartRateTxPack[HR_BLOCK3_OFFSET+8]=BREAK_UINT32(gSystemTimeInMS,7);
			//part4: data period
			gHeartRateTxPack[HR_BLOCK1_OFFSET+9]=0xf4;//0x01f4=500
			gHeartRateTxPack[HR_BLOCK1_OFFSET+10]=0x01;
			gHeartRateTxPack[HR_BLOCK2_OFFSET+9]=0x32;//0x0032=50
			gHeartRateTxPack[HR_BLOCK2_OFFSET+10]=0x00;
			gHeartRateTxPack[HR_BLOCK3_OFFSET+9]=0x32;//0x0032=50
			gHeartRateTxPack[HR_BLOCK3_OFFSET+10]=0x00;
			//part5: data length
			gHeartRateTxPack[HR_BLOCK1_OFFSET+11]=0x02;//0x0002=2
			gHeartRateTxPack[HR_BLOCK1_OFFSET+12]=0x00;
			gHeartRateTxPack[HR_BLOCK2_OFFSET+11]=0x28;//0x0028=40
			gHeartRateTxPack[HR_BLOCK2_OFFSET+12]=0x00;
			gHeartRateTxPack[HR_BLOCK3_OFFSET+11]=0x28;//0x0028=40
			gHeartRateTxPack[HR_BLOCK3_OFFSET+12]=0x00;
			//part6: data body 
			gHeartRateTxPack[HR_BLOCK1_OFFSET+13]=LO_UINT16(gHeartRate);//data1
			gHeartRateTxPack[HR_BLOCK1_OFFSET+14]=HI_UINT16(gHeartRate);
			gHeartRateTxPack[HR_BLOCK2_OFFSET+13]=BREAK_UINT32(*((long *)data.value.v+0),0);//data2
			gHeartRateTxPack[HR_BLOCK2_OFFSET+14]=BREAK_UINT32(*((long *)data.value.v+0),1);
			gHeartRateTxPack[HR_BLOCK2_OFFSET+15]=BREAK_UINT32(*((long *)data.value.v+0),2);
			gHeartRateTxPack[HR_BLOCK2_OFFSET+16]=BREAK_UINT32(*((long *)data.value.v+0),3);
			gHeartRateTxPack[HR_BLOCK3_OFFSET+13]=BREAK_UINT32(*((long *)data.value.v+1),0);//data3
			gHeartRateTxPack[HR_BLOCK3_OFFSET+14]=BREAK_UINT32(*((long *)data.value.v+1),1);
			gHeartRateTxPack[HR_BLOCK3_OFFSET+15]=BREAK_UINT32(*((long *)data.value.v+1),2);
			gHeartRateTxPack[HR_BLOCK3_OFFSET+16]=BREAK_UINT32(*((long *)data.value.v+1),3);
			gDataClass=1;
			break;
		case PACK_STATUS_BODY:
			if(gDataClass==1)
			{
				gHeartRateTxPack[HR_BLOCK1_OFFSET+15]=LO_UINT16(gHeartRate);
				gHeartRateTxPack[HR_BLOCK1_OFFSET+16]=HI_UINT16(gHeartRate);
				gHeartRateTxPack[HR_BLOCK2_OFFSET+17]=BREAK_UINT32(*((long *)data.value.v+0),0);//data2
				gHeartRateTxPack[HR_BLOCK2_OFFSET+18]=BREAK_UINT32(*((long *)data.value.v+0),1);
				gHeartRateTxPack[HR_BLOCK2_OFFSET+19]=BREAK_UINT32(*((long *)data.value.v+0),2);
				gHeartRateTxPack[HR_BLOCK2_OFFSET+20]=BREAK_UINT32(*((long *)data.value.v+0),3);
				gHeartRateTxPack[HR_BLOCK3_OFFSET+17]=BREAK_UINT32(*((long *)data.value.v+1),0);//data3
				gHeartRateTxPack[HR_BLOCK3_OFFSET+18]=BREAK_UINT32(*((long *)data.value.v+1),1);
				gHeartRateTxPack[HR_BLOCK3_OFFSET+19]=BREAK_UINT32(*((long *)data.value.v+1),2);
				gHeartRateTxPack[HR_BLOCK3_OFFSET+20]=BREAK_UINT32(*((long *)data.value.v+1),3);
			}
			else//gDataClass!=1
			{
				gHeartRateTxPack[]
				
			}
			break;
		default:
			break;
	}*/

}




/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
	osMessageQDef(AFE44XXMsg, 2, sizeof(long));
	vSemaphoreCreateBinary(xBinarySemaphore);
//	xSemaphoreCreateMutex() ();
	upQueueMutexSema=osMutexCreate(&upQueueMutex);
//	upQueueMutexSema=xSemaphoreCreateMutex();
//	vSemaphoreCreateBinary(xBinSemaHeartRateSend);

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
	
 	initHeartRateDataType();
	
	xQueue = osMessageCreate (&os_messageQ_def_AFE44XXMsg, vHandlerAFE44xxITTASK);
	//Init up data queue from STM to NRF
	QueueInit();
	
	AFE44xxInterrupt_Enable()	;
	if(xBinarySemaphore != NULL)
	{

	/* Init code generated for FreeRTOS */
	/* Create Start thread */
	osThreadDef(USER_Thread, StartThread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
	//osThreadDef(Uart_Thread_Name, UartThread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
	osThreadCreate (osThread(USER_Thread), NULL);
	//osThreadCreate (osThread(Uart_Thread_Name), NULL);

//	osThreadDef(AFE44xxIT_Thread_Name, vHandlerAFE44xxITTASK, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);	
//	osThreadCreate (osThread(AFE44xxIT_Thread_Name), NULL);
		
//	osThreadDef(vReceiveAFE44xxDataTask_Thread_Name, vReceiveAFE44xxDataTask, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);	
//	osThreadCreate (osThread(vReceiveAFE44xxDataTask_Thread_Name), NULL);	
	osThreadDef(vTask1a_ThreadName, vTask1a, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);	
	osThreadCreate (osThread(vTask1a_ThreadName), NULL);
	osThreadDef(vTask3a_ThreadName, vTask3a, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);	
	osThreadCreate (osThread(vTask3a_ThreadName), NULL);
	osThreadDef(vTask2a_ThreadName, vTask2a, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);	
	osThreadCreate (osThread(vTask2a_ThreadName), NULL);
	osThreadDef(upQueueRead_Thread_Name, upQueueReadThread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);	
	osThreadCreate (osThread(upQueueRead_Thread_Name), NULL);
	osThreadDef(upQueueWrite_Thread_Name, upQueueWriteThread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);	
	osThreadCreate (osThread(upQueueWrite_Thread_Name), NULL);
//	osThreadDef(HRS_Thread_Name, HeartRateSendThread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);	
//	osThreadCreate (osThread(HRS_Thread_Name), NULL);
	
//	startUartRxThread();
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



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
