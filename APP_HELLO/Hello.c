/*----------------------------------------------------------------------------
 * Name:    Hello.c
 * Purpose: Hello World Example
 * Note(s):
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

#include <stdio.h>               /* prototype declarations for I/O functions  */
#include <stm32f10x.h>           /* STM32F10x definitions */
#include "stm32f10x_flash.h"
#include "eeprom.h"
#include "common.h"

extern void SER_Init(void);
uint16_t VirtAddVarTab[NumbOfVar]={0x5555};



/*----------------------------------------------------------------------------
  main program
 *----------------------------------------------------------------------------*/
int main (void)  {               /* execution starts here                     */
  
	uint16_t moteid;
	uint8_t buf[8]={0};
	uint16_t status;
	FLASH_Unlock();
	EE_Init();
	SER_Init ();                /* initialize the serial interface           */
	
	printf("Hello World!\n");
	printf("Hello World!\n");
	printf("Hello World!\n");
	status= EE_ReadVariable(VirtAddVarTab[0], &moteid);
	if( status == 0)
	{
		 Int2Str(buf, moteid);
		 printf("Read moteid successfully.\n");
		 printf("%s \n", buf);
	}else
	  printf("Read moteid failed");
	
	
  while (1) {                    /* An embedded program does not stop and     */
    //
		printf ("mote id is %s!\n", buf);    /* never returns. We use an endless loop.    */
  }                              /* Replace the dots (...) with your own code.*/

}
