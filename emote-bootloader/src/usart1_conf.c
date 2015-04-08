/**
  ******************************************************************************
  * @file    emote-bootloader/src/usart1_config.c
  * @author  Ling Wang
  * @version V1.0.0
  * @date    3/31/2015
  * @brief   Uart configuration
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

void USART1_NVIC_conf()
{
	NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable the USART1 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void USART1_GPIO_conf()
{
	
 GPIO_InitTypeDef GPIO_InitStructure;
	
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO,ENABLE);
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	
 /*USART1 tx: PA.9 rx: PA.10*/
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //????-TX
 GPIO_Init(GPIOA,&GPIO_InitStructure);
	
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//????-RX
 GPIO_Init(GPIOA, &GPIO_InitStructure);

}

void USART1_Init(void)
{
	USART_InitTypeDef USART_InitStructure;
	
	USART1_NVIC_conf();
	USART1_GPIO_conf();
	
		/* USARTx configured as follow:
        - BaudRate = 115200 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */
	
  USART_InitStructure.USART_BaudRate = 115200*8;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	
	/*set USART1*/
	USART_Init(USART1, &USART_InitStructure);
	
	/* Enable the USART1 Receive interrupt*/
  /*USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);*/

	/*enable USART1*/
  USART_Cmd(USART1, ENABLE);
}


	
