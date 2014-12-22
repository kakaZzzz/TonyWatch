/**
  ******************************************************************************
  * @file    stm32f4_tony.c
  * @author  ADDING Team
  * @version V1.0.0
  * @date    26-11-2014
  * @brief   This file provides the AFE4490 Codec driver.   
  ******************************************************************************/
#include "stm32f4_tony.h"
#include "FreeRTOS.h"
#include "task.h"
#include "misc.h"
#include "cmsis_os.h"
#include "debug_uart.h"
#include "AFE4490.h"

#define __STM32F4_DISCO_BSP_VERSION_MAIN   (0x02) /*!< [31:24] main version */
#define __STM32F4_DISCO_BSP_VERSION_SUB1   (0x00) /*!< [23:16] sub1 version */
#define __STM32F4_DISCO_BSP_VERSION_SUB2   (0x02) /*!< [15:8]  sub2 version */
#define __STM32F4_DISCO_BSP_VERSION_RC     (0x00) /*!< [7:0]  release candidate */ 
#define __STM32F4_DISCO_BSP_VERSION         ((__STM32F4_DISCO_BSP_VERSION_MAIN << 24)\
                                             |(__STM32F4_DISCO_BSP_VERSION_SUB1 << 16)\
                                             |(__STM32F4_DISCO_BSP_VERSION_SUB2 << 8 )\
                                             |(__STM32F4_DISCO_BSP_VERSION_RC)) 

GPIO_TypeDef* GPIO_PORT[LEDn] = {LED0_GPIO_PORT, 
                                 LED1_GPIO_PORT, 
                                 LED2_GPIO_PORT,
                                 LED3_GPIO_PORT};
const uint16_t GPIO_PIN[LEDn] = {LED0_PIN, 
                                 LED1_PIN, 
                                 LED2_PIN,
                                 LED3_PIN};

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
//__IO ITStatus UartReady = RESET;	

/* Buffer used for transmission */
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*-----------------------------------------------------------*/																 
//******************************************************************/
//
//****************************************************************/
/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
  while(1)
  {
  }
}																 
															 
																 
//GPIO_TypeDef* BUTTON_PORT[BUTTONn] = {KEY_BUTTON_GPIO_PORT}; 
//const uint16_t BUTTON_PIN[BUTTONn] = {KEY_BUTTON_PIN}; 
//const uint8_t BUTTON_IRQn[BUTTONn] = {KEY_BUTTON_EXTI_IRQn};

//uint32_t I2cxTimeout = I2Cx_TIMEOUT_MAX;    /*<! Value of Timeout when I2C communication fails */ 
//uint32_t SpixTimeout = SPIx_TIMEOUT_MAX;    /*<! Value of Timeout when SPI communication fails */

//static SPI_HandleTypeDef    SpiHandle;
//static I2C_HandleTypeDef    I2cHandle;
/**
  * @}
  */ 

/** @defgroup STM32F4_DISCOVERY_LOW_LEVEL_Private_FunctionPrototypes
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup STM32F4_DISCOVERY_LOW_LEVEL_Private_Functions
  * @{
  */ 
//static void     I2Cx_Init(void);
//static void     I2Cx_WriteData(uint8_t Addr, uint8_t Reg, uint8_t Value);
//static uint8_t  I2Cx_ReadData(uint8_t Addr, uint8_t Reg);
//static void     I2Cx_MspInit(void);
//static void     I2Cx_Error(uint8_t Addr);

//static void     SPIx_Init(void);
//static void     SPIx_MspInit(void);
//static uint8_t  SPIx_WriteRead(uint8_t Byte);
//static  void    SPIx_Error(void);

///* Link functions for Accelerometer peripheral */
//void            ACCELERO_IO_Init(void);
//void            ACCELERO_IO_ITConfig(void);
//void            ACCELERO_IO_Write(uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite);
//void            ACCELERO_IO_Read(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead);

///* Link functions for Audio peripheral */
//void            AUDIO_IO_Init(void);
//void            AUDIO_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value);
//uint8_t         AUDIO_IO_Read(uint8_t Addr, uint8_t Reg);



//****************************************************/
///*************** LED CONTROL FOR freeRTOS **********/
//****************************************************/
/** @defgroup STM32F4_DISCOVERY_LOW_LEVEL_LED_Functions
  * @{
  */ 

/**
  * @brief  This method returns the STM32F4 DISCO BSP Driver revision
  * @param  None
  * @retval version : 0xXYZR (8bits for each decimal, R for RC)
  */
uint32_t BSP_GetVersion(void)
{
  return __STM32F4_DISCO_BSP_VERSION;
}

/**
  * @brief  Configures LED GPIO.
  * @param  Led: Specifies the Led to be configured. 
  *   This parameter can be one of following parameters:
  *     @arg LED4
  *     @arg LED3
  *     @arg LED5
  *     @arg LED6
  * @retval None
  */
void BSP_LED_Init(Led_TypeDef Led)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  /* Enable the GPIO_LED Clock */
  LEDx_GPIO_CLK_ENABLE(Led);

  /* Configure the GPIO_LED pin */
  GPIO_InitStruct.Pin = GPIO_PIN[Led];
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  
  HAL_GPIO_Init(GPIO_PORT[Led], &GPIO_InitStruct);
  
//  HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET); 
//  HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_SET); 
}

/**
  * @brief  Turns selected LED On.
  * @param  Led: Specifies the Led to be set on. 
  *   This parameter can be one of following parameters:
  *     @arg LED0
  *     @arg LED1
  *     @arg LED2
  *     @arg LED3  
  * @retval None
  */
void BSP_LED_On(Led_TypeDef Led)
{
  HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_SET); 
}

/**
  * @brief  Turns selected LED Off.
  * @param  Led: Specifies the Led to be set off. 
  *   This parameter can be one of following parameters:
  *     @arg LED0
  *     @arg LED1
  *     @arg LED2
  *     @arg LED3 
  * @retval None
  */
void BSP_LED_Off(Led_TypeDef Led)
{
  HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET); 
}

/**
  * @brief  Toggles the selected LED.
  * @param  Led: Specifies the Led to be toggled. 
  *   This parameter can be one of following parameters:
  *     @arg LED0
  *     @arg LED1
  *     @arg LED2
  *     @arg LED3  
  * @retval None
  */
void BSP_LED_Toggle(Led_TypeDef Led)
{
  HAL_GPIO_TogglePin(GPIO_PORT[Led], GPIO_PIN[Led]);
}

/**
  * @brief  StartThread For LED control.
  * @param  Led: Specifies the Led to be toggled. 
  *   This parameter can be one of following parameters:
  *     @arg LED0
  *     @arg LED1
  *     @arg LED2
  *     @arg LED3  
  * @retval None
  */
  /*
void StartThread(void const * argument)
{

 volatile unsigned long ul;	
  for(;;)
  {
	BSP_LED_On(LED1);
//	for( ul = 0; ul < 5000000; ul++ ) {
//	}
	osDelay(500);
	BSP_LED_Off(LED1);
//	for( ul = 0; ul < 5000000; ul++ ) {
//	}
	osDelay(500);
  }
}*/


/***********************************************************************************/
//END
/***********************************************************************************/





