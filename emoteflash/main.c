/*
  stm32flash - Open Source ST STM32 flash program for *nix
  Copyright (C) 2010 Geoffrey McRae <geoff@spacevs.com>

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/


  #include <sys/types.h>
  #include <sys/stat.h>
  #include <fcntl.h>
  #include <termios.h>
  #include <stdio.h>
  
  #include "ymodem.h"
  #include "serial_posix.h"
  #include "serial.h"
  
  
  #define FALSE 0
  #define TRUE 1
  FILE *in,*out; 
  
  
  uint8_t GetKey(void)
  {
	  uint8_t key = 0;

	  /* Waiting for user input */
	  while (1)
	  {
	    if (SerialGetChar((char *)&key)) break;
	  }
	  return key;
  }
  
  uint8_t Get_Byte (uint32_t timeout)
  {
    uint8_t c;
    while (timeout-- > 0)
    {
      if (SerialGetChar(&c) == 1)
      {
        return (uint8_t)c;
      }
    }
    return 0;
  }

  void signal_handler_IO (int status)
  {
  	
  }

 void SerialUpload(const char* filename)
 {
 	
  uint32_t status = 0;
  int r;
  
  if (GetKey() == CRC16)
  {
    /* Transmit the flash image through ymodem protocol */
    /*status = Ymodem_Transmit((uint8_t*)ApplicationAddress, (const uint8_t*)"UploadedFlashImage.bin", FLASH_IMAGE_SIZE);*/

	printf("\n\n\r Uploading the image to target...\n\r");
    status = Ymodem_Transmit((const uint8_t *)filename);
    if (status != 0) 
    {
      printf("\n\rError Occured while Transmitting File!please try again!\n\r");
    }
    else
    {
      printf("\n\rFile Trasmitted Successfully \n\r");
    }
  }

 }
     
  int main(int argc, char *argv[])
  {
    int status;
    char key,rec;
    char *device, *imageFile;
	int res;
	int cnt =10000;
      
    
    if( argc < 3){
    	printf("Please input image file name and serial device name: \n");
        printf(" FORMAT: emoteflash imageFile deviceName \n");
    	return 0;
    }else
    	{
    		imageFile = argv[1];
    		device = argv[2];
    }
   
    status = SerialInt(device);
    if(status <0)
    { 
      printf("serial port init failed...\n");
      return 0;
    }

	key= 0x16; //'SYN'
	
    {
   	
	 res = SerialPutChar(key);
	 if(res != 1)
	 {
	   printf("write key falied...%d\n",res);
	 }
	 //printf("%c\n",key);
	 //printf("write key %x successful...\n",key);

     rec = Get_Byte(100);
	 printf("%c",rec);
	 //printf("receive byte %x\n",rec);
	
	if(rec == 0x06){
	   SerialUpload(imageFile);
       SerialClose();
	   return 1;
	}	
	 
	 	
   	}
	
	return 1;
   #if 0 
   while (stop== FALSE) { 
    	
    /* loop for input */
    	
   	if(wait_flag == FALSE){
     
      res = serial_read(h,buf,255);
      buf[res]=0;               /* so we can printf... */
      printf("%s", buf);
    }
    
   /* process_key_input(); */

   }
   #endif 
   
  }

  
  
  #if 0
  
  
  void send_command(char key)
  {

  } 
   void process_key_input()
{
      int key;
  key = fgetc(in);
    switch(key){
  case 0x1b:
  stop = TRUE;
      break;
     case 0x31:
     case 0x32:
     case 0x33:
     fputc(key,out);
     send_command((char)key);
     break;
     default:
 break;
}
} 
#endif