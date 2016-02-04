/**
  ******************************************************************************
  * @file    emote-bootloader/src/main.c 
  * @author  Ling Wang
  * @version V1.0.0
  * @date    3/31/2015
  * @brief   Main program body
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  */ 

#include "common.h"
#include "uart.h"


/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */

int main(void)
{
	uint8_t c;

	int32_t res=0;
  /* Flash unlock */
  FLASH_Unlock();
	
  /* Init serial port*/
  USART1_Init();
	
	/* Print version information*/
	SerialPutString("\r\n===================================================\r\n");
	SerialPutString("\r\nEmote bootloader v1.0.0, COPYRIGHT DNC 2016\r\n");
	SerialPutString("\r\nEnter 'p' to show menu; 'D' download; 'U' upload\r\n");
	SerialPutString("\r\n===================================================\r\n");
  
	while(1)
	{
		
		//res=Menu();
		
		/*wait 5s until receive a byte*/
		Serial_ReceiveByte(&c,400000);
		
		if(c == 0x70)  /*'p'*/
		{
			SerialPutChar(c);
			res=Menu();
	  }else if (c == 0x44)  /*'D'*/
		{
			SerialPutChar(ACK);
			res=SerialDownload();
		}else if (c == 0x55)  /*'U'*/
		{
			SerialPutChar(ACK);
			res=SerialUpload();
		}
		/* If download/upload success or there is no input, jump to implement app */
		if(res>=0)
			Jump_to_app();
  }
}



