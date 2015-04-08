/**
  ******************************************************************************
  * @file    USART/Printf/main.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include <stdio.h>
#include "stm32f10x_usart.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "misc.h"

/* Private function prototypes -----------------------------------------------*/

void USART3_NVIC_conf()
{
	NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable the USART2 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void USART3_GPIO_conf()
{
	
 GPIO_InitTypeDef GPIO_InitStructure;
 /* USART3 APB1; PB.10 PB.11; */
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO,ENABLE);
 RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	
	/* remap USART3*/
	/*GPIO_PinRemapConfig(GPIO_Remap_USART3,ENABLE);*/
	
 /*USART1 tx: PB.10 rx: PB.11*/
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //????-TX
 GPIO_Init(GPIOB,&GPIO_InitStructure);
	
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//????-RX
 GPIO_Init(GPIOB, &GPIO_InitStructure);

}

void USART3_init(void)
{
	USART_InitTypeDef USART_InitStructure;
	
	USART3_NVIC_conf();
	USART3_GPIO_conf();
	
		/* USARTx configured as follow:
        - BaudRate = 115200 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */
	
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	
	/*set USART1*/
	USART_Init(USART3, &USART_InitStructure);
	
	/* Enable the USART2 Receive interrupt*/
  /*USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);*/

	/*enable USART2*/
  USART_Cmd(USART3, ENABLE);
}
	
