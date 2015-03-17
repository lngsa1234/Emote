/*----------------------------------------------------------------------------
 * Name:    Serial.c
 * Purpose: Low Level Serial Routines
 * Note(s): possible defines select the used communication interface:
 *            __DBG_ITM   - ITM SWO interface
 *                        - USART1 interface  (default)
 *----------------------------------------------------------------------------
 * This file is part of the uVision/ARM development tools.
 * This software may only be used under the terms of a valid, current,
 * end user licence from KEIL for a compatible version of KEIL software
 * development tools. Nothing else gives you the right to use this software.
 *
 * This software is supplied "AS IS" without warranties of any kind.
 *
 * Copyright (c) 2012 Keil - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include <stm32f10x.h>                       /* STM32F10x definitions         */
#include "usart_conf.h"
#include "serial.h"
#ifdef __DBG_ITM
volatile int ITM_RxBuffer = ITM_RXBUFFER_EMPTY;  /*  CMSIS Debug Input        */
#endif

void assert_failed(uint8_t* file, uint32_t line)
{
	while(1)
	{
	}
}


/*----------------------------------------------------------------------------
  Initialize UART pins, Baudrate
 *----------------------------------------------------------------------------*/

void SER_Init (void) {
  USART1_Init();
}


/*----------------------------------------------------------------------------
  Write character to Serial Port
 *----------------------------------------------------------------------------*/
int SER_PutChar (int c) {

#ifdef __DBG_ITM
    ITM_SendChar(c);
#else  
  while (!(USART1->SR & USART_SR_TXE));
  USART1->DR = (c & 0x1FF);
#endif
  return (c);
}

void USART1_test()
{
	char* p="USART1 is printing Hello World!";
	while(*p){
		while (!(USART1->SR & USART_SR_TXE));
    USART1->DR = (*p++ & 0x1FF);
	}
}

void USART2_test()
{
	char* p="USART2 is printing Hello World!";
	while(*p){
		while (!(USART2->SR & USART_SR_TXE));
    USART2->DR = (*p++ & 0x1FF);
	}
}

void USART3_test()
{
	char* p="USART3 is printing Hello World!";
	while(*p){
		while (!(USART3->SR & USART_SR_TXE));
    USART3->DR = (*p++ & 0x1FF);
	}
}

void USART4_test()
{
	char* p="UART4 is printing Hello World!";
	while(*p){
		while (!(UART4->SR & USART_SR_TXE));
    UART4->DR = (*p++ & 0x1FF);
	}
}
/*----------------------------------------------------------------------------
  Read character from Serial Port   (blocking read)
 *----------------------------------------------------------------------------*/
int SER_GetChar (void) {
	
#ifdef __DBG_ITM
  while (ITM_CheckChar() != 1) __NOP();
  return (ITM_ReceiveChar());
#else
  while (!(USART1->SR & USART_SR_RXNE));
  return (USART1->DR & 0xFF);
#endif
}


/*----------------------------------------------------------------------------
  Check if a character is received
 *----------------------------------------------------------------------------*/
int SER_CheckChar (void) {

#ifdef __DBG_ITM
  return (ITM_CheckChar());
#else
  if (USART1->SR & USART_SR_RXNE)
    return (1);
  else
    return (0);
#endif
}
