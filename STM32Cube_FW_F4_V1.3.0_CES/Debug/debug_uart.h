/*
 ******************************************************************************
 *              COPYRIGHT 2013 �ֿƵ��ӹ�����
 *
 *�ļ���debug_uart.h
 *���ߣ��ֿƵ��ӹ�����
 *���ڣ�2013.12.20
 *�汾��V1.0 
 *���������Դ�������ͷ�ļ��� 
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
