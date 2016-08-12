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
#include "eeprom.h"
#include "uart.h"

uint16_t VirtAddVarTab[NumbOfVar]={0x5555};
/**
  * @brief  Display the Main Menu on to HyperTerminal
  * @param  None
  * @retval None
  */
int32_t Menu(void)
{
	int8_t res = -1;
  uint8_t key = 0;
	__IO uint32_t FlashProtection = 0;
	
  FlashProtection = FlashProtectionStatus();
	
	while(1)
	{
		
		SerialPutString("\r\n================== Main Menu ============================\r\n\n");
		SerialPutString("  Download Image To the STM32F10x Internal Flash ------- 1\r\n\n");
		SerialPutString("  Upload Image From the STM32F10x Internal Flash ------- 2\r\n\n");
		SerialPutString("  Execute The New Program ------------------------------ 3\r\n\n");
		
		if(FlashProtection != 0)
		{
			SerialPutString("  Disable the write protection ------------------------- 4\r\n\n");
		}
		
		SerialPutString("==========================================================\r\n\n");
		
		/* Send back received byte*/
		if(Serial_ReceiveByte(&key,-1))
			SerialPutChar(key);
	  
		if (key == 0x31)
		{
			return (SerialDownload()>0)?1:res;
		
		}
		else if (key == 0x32)
		{
			return (SerialUpload()== 0)?2:res;
		}
		else if (key == 0x33)
		{
			return 0;
		}
		else if ((key == 0x34) && (FlashProtection == 1))
		{
			/* Disable the write protection of desired pages */
			FLASH_DisableWriteProtectionPages();
		}
		else
		{
			if (FlashProtection == 0)
			{
				SerialPutString("Invalid Number ! ==> The number should be either 1, 2 or 3\r");
			}
			else
			{
				SerialPutString("Invalid Number ! ==> The number should be either 1, 2, 3 or 4\r");
			} 
		}
   }
 }

/* Execute application */
void Jump_to_app()
{
	pFunction Jump_To_Application;
  uint32_t JumpAddress;
	if (((*(__IO uint32_t*)ApplicationAddress) & 0x2FFE0000 ) == 0x20000000)
			{
				SerialPutChar(0x16);
			  JumpAddress = *(__IO uint32_t*) (ApplicationAddress + 4);

				/* Jump to user application */
				Jump_To_Application = (pFunction) JumpAddress;
				/* Initialize user application's Stack Pointer */
				__set_MSP(*(__IO uint32_t*) ApplicationAddress);
				Jump_To_Application();
			}
}

/*Write emote id to flash*/
int8_t WriteMoteId(void)
{
	uint8_t c;
	uint8_t cnt1=100;
	int32_t moteid;
	uint16_t tmp_id;
	uint8_t buf[8];

	while(cnt1--){
		Serial_ReceiveByte(&c,100000);
		if(c==0x4D) /*M*/
		{
			SerialPutChar(0x52); /*R*/
			GetInputString(buf);
			Str2Int(buf, &moteid);
			EE_WriteVariable(VirtAddVarTab[0], (uint16_t)moteid);
			EE_ReadVariable(VirtAddVarTab[0],&tmp_id);
			return  (moteid==tmp_id)?0:-1;
		}
}
	return 0;
}

int main(void)
{
	uint8_t c;

	int8_t res=0;
  /* Flash unlock */
  FLASH_Unlock();
	
	/* EEPROM Init */
  EE_Init();
	
  /* Init serial port*/
  USART1_Init();
	
	/* Print version information*/
	SerialPutString("\r\n===================================================\r\n");
	SerialPutString("\r\nEmote bootloader v1.0.0, COPYRIGHT DNC 2016\r\n");
	SerialPutString("\r\nEnter 'p' to show menu; 'D' to automatically download; 'U' to automatically upload\r\n");
	SerialPutString("\r\n===================================================\r\n");
  
	while(1)
	{
		
		/*wait 5s until receive a byte*/
		SerialPutString("\r\nEmote is waiting command from server\r\n");
		Serial_ReceiveByte(&c,400000);
		
		if(c == 0x70)  /*'p'*/
		{
			SerialPutChar(c);
			res = Menu();
	  }else if (c == 0x44)  /*'D'*/
		{
			SerialPutChar(ACK);
			res = (SerialDownload()>0)? 1: -1;

		}else if (c == 0x55)  /*'U'*/
		{
			SerialPutChar(ACK);
			res= (SerialUpload() == 0)? 2: -1;
		}
		/*Update mote id after downloading*/
		if(res == 1)
				res = WriteMoteId();
		
		/* If download/upload success or there is no input, jump to implement app */	
		if(res>=0){
			Jump_to_app();
		}
  }
}



