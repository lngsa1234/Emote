/**
  ******************************************************************************
  * @file    serial.h
  * @author  Ling Wang
  * @version V1.0.0
  * @date    3/31/2015
  * @brief   This file provides all the software function headers of the serial.c
  *          file.
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
  * <h2><center>&copy; COPYRIGHT 2010 </center></h2>
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _SERIAL_H_
#define _SERIAL_H_


/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
int SER_PutChar (int c);
int SER_GetChar (void);
int SER_CheckChar (int *c);

#endif  /* _SERIAL_H_ */

/*******************(C)COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
