/**
  ******************************************************************************
  * @file    AFE4490.h
  * @author  ADDING Team
  * @version V1.0.0
  * @date    26-11-2014
  * @brief   This file provides the AFE4490 Codec driver.   
  ******************************************************************************/
#ifndef __AFE44XX_H__
#define __AFE44XX_H__

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"

#define AFE44XX_SPI              SPI2

#define AFE44XX_GPIO_AF          GPIO_AF5_SPI2
#define AFE44XX_SPI_CLK_ENABLE()   __SPI2_CLK_ENABLE()

#define AFE44XX_MISO_GPIO        GPIOC
#define AFE44XX_MISO_PIN         GPIO_PIN_3
#define AFE44XX_MISO_CLK_ENABLE()    __GPIOC_CLK_ENABLE()  
#define AFE44XX_MISO_CLK_DISABLE()   __GPIOC_CLK_DISABLE()  

#define AFE44XX_MOSI_GPIO        GPIOC
#define AFE44XX_MOSI_PIN         GPIO_PIN_2
#define AFE44XX_MOSI_CLK_ENABLE()    __GPIOC_CLK_ENABLE()  
#define AFE44XX_MOSI_CLK_DISABLE()   __GPIOC_CLK_DISABLE() 

#define AFE44XX_SCK_GPIO         GPIOB
#define AFE44XX_SCK_PIN          GPIO_PIN_13
#define AFE44XX_SCK_CLK_ENABLE()    __GPIOB_CLK_ENABLE()  
#define AFE44XX_SCK_CLK_DISABLE()   __GPIOB_CLK_DISABLE() 

#define AFE44XX_CS_GPIO          GPIOB
#define AFE44XX_CS_PIN           GPIO_PIN_12
#define AFE44XX_CS_CLK_ENABLE()    __GPIOB_CLK_ENABLE()  
#define AFE44XX_CS_CLK_DISABLE()   __GPIOB_CLK_DISABLE()

#define AFE44XX_PDN_GPIO          GPIOI
#define AFE44XX_PDN_PIN           GPIO_PIN_3
#define AFE44XX_PDN_CLK_ENABLE()    __GPIOI_CLK_ENABLE()  
#define AFE44XX_PDN_CLK_DISABLE()   __GPIOI_CLK_DISABLE()

#define AFE44XX_RESET_GPIO          GPIOD
#define AFE44XX_RESET_PIN           GPIO_PIN_5
#define AFE44XX_RESET_CLK_ENABLE()    __GPIOD_CLK_ENABLE()  
#define AFE44XX_RESET_CLK_DISABLE()   __GPIOD_CLK_DISABLE()

#define AFE44XX_ADRDY_GPIO          GPIOD
#define AFE44XX_ADRDY_PIN           GPIO_PIN_4
#define AFE44XX_ADRDY_CLK_ENABLE()    __GPIOD_CLK_ENABLE()  
#define AFE44XX_ADRDY_CLK_DISABLE()   __GPIOD_CLK_DISABLE()

#define ADRDY_EXTI_PORT_SOURCE      EXTI_Port_Source_GPIOD
#define ADRDY_EXTI_PIN_SOURCE       EXTI_PinSource4
#define ADRDY_EXTI_IRQn             EXTI4_IRQn 

//#define AFE44XX_DIAG_GPIO          GPIOD
//#define AFE44XX_DIAG_PIN           GPIO_Pin_0
//#define AFE44XX_DIAG_PINS          GPIO_PinSource0
//#define AFE44XX_DIAG_CLK           RCC_AHB1Periph_GPIOD

#define AFE44XX_EN_GPIO          GPIOG
#define AFE44XX_EN_PIN           GPIO_PIN_15
#define AFE44XX_EN_CLK_ENABLE()    __GPIOG_CLK_ENABLE()  
#define AFE44XX_EN_CLK_DISABLE()   __GPIOG_CLK_DISABLE()

extern void AFE44xx_init(void);
extern uint32_t AFE44xx_Diagnostic(void);
extern uint32_t AFE44xx_RDSample(void);
extern void Error_Handler(void);
#endif /* __AFE44XX_H__ */
