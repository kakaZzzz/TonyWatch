/*
 ******************************************************************************
 *              COPYRIGHT 2013 乐科电子工作室
 *
 *文件：debug_uart.c
 *作者：乐科电子工作室
 *日期：2013.12.20
 *版本：V1.0 
 *描述：调试串口驱动
 ******************************************************************************
 */
#include "debug_uart.h"
#include "stdint.h"
#include <stdarg.h>
#include <stdio.h>
void USART_SendData(USART_TypeDef* USARTx, uint16_t Data);
FlagStatus USART_GetFlagStatus(USART_TypeDef* USARTx, uint16_t USART_FLAG);
#define USART_FLAG_CTS                       ((uint16_t)0x0200)

__inline void put_char(unsigned char ch)
{
  debug_putchar(ch);    
}

__inline unsigned char get_char(void)
{
 return debug_getchar(1);    
}

/*
 ******************************************************************************
 *函数：void debug_uart_init(void)
 *输入：void
 *输出：void
 *描述：调试串口初始化
 ******************************************************************************
 */ 
//void debug_uart_init(void)
//{
//  /* 定义GPIO、UART初始化结构体变量 */
//  GPIO_InitTypeDef GPIO_InitStructure;
//  USART_InitTypeDef USART_InitStructure;
//  /* 使能GPIO、USART时钟 */
//  RCC_AHB1PeriphClockCmd(DEBUG_UART_TXRCCCLK, ENABLE);
//  RCC_AHB1PeriphClockCmd(DEBUG_UART_RXRCCCLK, ENABLE);
//  RCC_APB2PeriphClockCmd(DEBUG_UART_RCCCLK, ENABLE);
//  /* 配置GPIO链接到USART */
//  GPIO_PinAFConfig(DEBUG_UART_TXPORT, DEBUG_UART_TXPINSOURCE, DEBUG_UART_GPIO_AF);
//  GPIO_PinAFConfig(DEBUG_UART_RXPORT, DEBUG_UART_RXPINSOURCE, DEBUG_UART_GPIO_AF);

//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//	
//  GPIO_InitStructure.GPIO_Pin = DEBUG_UART_TXPIN;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_Init(DEBUG_UART_TXPORT, &GPIO_InitStructure);

//  GPIO_InitStructure.GPIO_Pin = DEBUG_UART_RXPIN;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_Init(DEBUG_UART_RXPORT, &GPIO_InitStructure);
//  /**************************************************************************** 
//   *串口配置： 
//   *    波特率：115200
//   *    字长：  8bit
//   *    停止位：1bit
//   *    流控制：不使用
//   ****************************************************************************
//   */
//  USART_InitStructure.USART_BaudRate = 115200;
//  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
//  USART_InitStructure.USART_StopBits = USART_StopBits_1;
//  USART_InitStructure.USART_Parity = USART_Parity_No;
//  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

//  USART_Init(DEBUG_UART, &USART_InitStructure);
//  USART_Cmd(DEBUG_UART, ENABLE);
// 
//}
/*
 ******************************************************************************
 *函数：void debug_putchar(const void *str)
 *输入：void
 *输出：str : 要输出的字符
 *描述：调试串口输出字符
 ******************************************************************************
 */ 
void debug_putchar(const char ch)
{
  /* 将接收到的数据发送出去 */
  USART_SendData(DEBUG_UART, ch);
  /* 等待串口发送完成 */
  while (USART_GetFlagStatus(DEBUG_UART, USART_FLAG_TC) == RESET);   
}
/*
 ******************************************************************************
 *函数：void debug_putstr(const void *str)
 *输入：void
 *输出：str : 要输出的字符串
 *描述：调试串口输出字符串
 ******************************************************************************
 */ 
void debug_putstr(const void *str)
{
  uint8_t *s = (uint8_t*)str;
  while (USART_GetFlagStatus(DEBUG_UART, USART_FLAG_TC) == RESET);
  while(*s) {
    debug_putchar(*s++);    
  }
}

/*
 ******************************************************************************
 *函数：uint8_t debug_getchar(uint8_t mode)
 *输入：uint8_t 串口接收到的字符
 *输出：mode    :  0  如果接受到数据返回字符如果没有接收到数据返回0
 *                 1  循环等待直到接收到数据，并返回该字符。
 *描述：调试串口读取字符
 ******************************************************************************
 */ 
//uint8_t debug_getchar(uint8_t mode)
//{
//  uint8_t tmp = 0;
//  
//  if (mode) {
//    while (USART_GetFlagStatus(DEBUG_UART, USART_FLAG_RXNE) == RESET);
//    tmp = USART_ReceiveData(DEBUG_UART);
//  } else {
//    if (USART_GetFlagStatus(DEBUG_UART, USART_FLAG_RXNE)) {
//      tmp = USART_ReceiveData(DEBUG_UART);
//    }
//  }
//  return tmp;
//}

void USART_SendData(USART_TypeDef* USARTx, uint16_t Data)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_USART_DATA(Data)); 
    
  /* Transmit Data */
  USARTx->DR = (Data & (uint16_t)0x01FF);
}

FlagStatus USART_GetFlagStatus(USART_TypeDef* USARTx, uint16_t USART_FLAG)
{
  FlagStatus bitstatus = RESET;
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_USART_FLAG(USART_FLAG));

  /* The CTS flag is not available for UART4 and UART5 */
  if (USART_FLAG == USART_FLAG_CTS)
  {
    assert_param(IS_USART_1236_PERIPH(USARTx));
  } 
    
  if ((USARTx->SR & USART_FLAG) != (uint16_t)RESET)
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
}

//#ifdef __MICROLIB
//int fputc(int ch, FILE *f)
//{
//  debug_putchar(ch);
//  return ch;
//}

//int fgetc(FILE *f)
//{
//  return debug_getchar(1);
//}
//#else

//#endif

