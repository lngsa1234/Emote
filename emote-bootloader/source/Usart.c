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
#include <stm32f10x.h>           /* STM32F10x definitions                     */

extern void SER_Init(void);
extern void USART1_test();
extern void USART2_test();/* see Serial.c */
extern void USART3_test();/* see Serial.c */
extern void USART4_test();/* see Serial.c */
/*----------------------------------------------------------------------------
  main program
 *----------------------------------------------------------------------------*/
int main (void)  {               /* execution starts here                     */

  SER_Init();                   /* initialize the serial interface           */
  printf ("HELLO WORLD!\n");      /* the 'printf' function call                */
  
  while (1) {                    /* An embedded program does not stop and     */
     printf ("I am Lynn\n"); /* ... */                 /* never returns. We use an endless loop.    */
  }                              /* Replace the dots (...) with your own code.*/

}
