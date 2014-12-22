/*
 ******************************************************************************
 *              COPYRIGHT 2013 �ֿƵ��ӹ�����
 *
 *�ļ���debug_uart.c
 *���ߣ��ֿƵ��ӹ�����
 *���ڣ�2013.12.20
 *�汾��V1.0 
 *���������Դ�������
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
 *������void debug_uart_init(void)
 *���룺void
 *�����void
 *���������Դ��ڳ�ʼ��
 ******************************************************************************
 */ 
//void debug_uart_init(void)
//{
//  /* ����GPIO��UART��ʼ���ṹ����� */
//  GPIO_InitTypeDef GPIO_InitStructure;
//  USART_InitTypeDef USART_InitStructure;
//  /* ʹ��GPIO��USARTʱ�� */
//  RCC_AHB1PeriphClockCmd(DEBUG_UART_TXRCCCLK, ENABLE);
//  RCC_AHB1PeriphClockCmd(DEBUG_UART_RXRCCCLK, ENABLE);
//  RCC_APB2PeriphClockCmd(DEBUG_UART_RCCCLK, ENABLE);
//  /* ����GPIO���ӵ�USART */
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
//   *�������ã� 
//   *    �����ʣ�115200
//   *    �ֳ���  8bit
//   *    ֹͣλ��1bit
//   *    �����ƣ���ʹ��
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
 *������void debug_putchar(const void *str)
 *���룺void
 *�����str : Ҫ������ַ�
 *���������Դ�������ַ�
 ******************************************************************************
 */ 
void debug_putchar(const char ch)
{
  /* �����յ������ݷ��ͳ�ȥ */
  USART_SendData(DEBUG_UART, ch);
  /* �ȴ����ڷ������ */
  while (USART_GetFlagStatus(DEBUG_UART, USART_FLAG_TC) == RESET);   
}
/*
 ******************************************************************************
 *������void debug_putstr(const void *str)
 *���룺void
 *�����str : Ҫ������ַ���
 *���������Դ�������ַ���
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
 *������uint8_t debug_getchar(uint8_t mode)
 *���룺uint8_t ���ڽ��յ����ַ�
 *�����mode    :  0  ������ܵ����ݷ����ַ����û�н��յ����ݷ���0
 *                 1  ѭ���ȴ�ֱ�����յ����ݣ������ظ��ַ���
 *���������Դ��ڶ�ȡ�ַ�
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

