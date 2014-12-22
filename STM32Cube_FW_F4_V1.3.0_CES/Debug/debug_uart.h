/*
 ******************************************************************************
 *              COPYRIGHT 2013 乐科电子工作室
 *
 *文件：debug_uart.h
 *作者：乐科电子工作室
 *日期：2013.12.20
 *版本：V1.0 
 *描述：调试串口驱动头文件。 
 ******************************************************************************
 */
#ifndef __DEBUG_UART_H__
#define __DEBUG_UART_H__
#include "stm32f4xx.h"
#include "stdio.h"

#define DEBUG_UART1             1


#if DEBUG_UART1
#define DEBUG_UART              USART1
#define DEBUG_UART_RCCCLK       RCC_APB2Periph_USART1
#define DEBUG_UART_GPIO_AF      GPIO_AF_USART1

#define DEBUG_UART_TXPORT       GPIOA
#define DEBUG_UART_TXPINSOURCE  GPIO_PinSource9
#define DEBUG_UART_TXPIN        GPIO_Pin_9
#define DEBUG_UART_TXRCCCLK     RCC_AHB1Periph_GPIOA   
#define DEBUG_UART_RXPORT       GPIOA
#define DEBUG_UART_RXPINSOURCE  GPIO_PinSource10
#define DEBUG_UART_RXPIN        GPIO_Pin_10
#define DEBUG_UART_RXRCCCLK     RCC_AHB1Periph_GPIOA   
#elif DEBUG_UART6
#define DEBUG_UART              USART6
#define DEBUG_UART_RCCCLK       RCC_APB2Periph_USART6
#define DEBUG_UART_GPIO_AF      GPIO_AF_USART6

#define DEBUG_UART_TXPORT       GPIOG
#define DEBUG_UART_TXPINSOURCE  GPIO_PinSource14
#define DEBUG_UART_TXPIN        GPIO_Pin_14
#define DEBUG_UART_TXRCCCLK     RCC_AHB1Periph_GPIOG
#define DEBUG_UART_RXPORT       GPIOG
#define DEBUG_UART_RXPINSOURCE  GPIO_PinSource9
#define DEBUG_UART_RXPIN        GPIO_Pin_9
#define DEBUG_UART_RXRCCCLK     RCC_AHB1Periph_GPIOG

#endif
#define debug_printf            printf

void debug_uart_init(void);
void debug_putchar(const char ch);
uint8_t debug_getchar(uint8_t mode);
#endif /* __DEBUG_UART_H__ */
