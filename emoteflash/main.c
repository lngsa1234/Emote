/*
  emoteflash - Open Source ST STM32 flash program for *nix
  Copyright (C) 2010 Ling Wang <lngsa.wang@gmail.com>

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
  #include "debug.h"
  
  

  
  /* uart interrpt function*/
  void signal_handler_IO (int status)
  {
  	
  }

  static int Serial_ReceiveByte(char* c, int timeout)
  {
  	int ref = 0;

	if(timeout == -1)
		ref = 1;
	while(timeout--> 0 || ref){
		if(SerialGetChar(c)== 1)
			return 1;
		}
	return 0;
  }
  
  static void SerialUpload(const char* filename)
  {
	 
   uint32_t status = 0;
   char c;
   FILE* fp;
   uint32_t fileSize;
   char* buf;
   
   if(filename == NULL)
	return;
   fp = fopen(filename,"r");
   if(fp<0){
    printf("open the image file failed...");
    return;
   }
  /*get file size*/
  fseek(fp, 0, SEEK_END);
  fileSize = ftell(fp);
  fseek(fp,0,SEEK_SET); 

 /* read file into buffer*/
  buf = malloc(fileSize);
  if(buf == NULL){
   printf("malloc buffer failed...");
   return;
  }
   if (Serial_ReceiveByte(&c,-1) == 1 && c == CRC16){
   	printf("Uploading the image to target...\n");
	status = Ymodem_Transmit((int8_t *)buf, (const uint8_t *)filename, fileSize);
	if (status != 0){
		printf("\n\rError Occured while Transmitting File!please try again!\n\r");
		}else{
		printf("\n\rFile Trasmitted Successfully \n\r");
		}
		}
   }
 
  int main(int argc, char *argv[])
  {
    int8_t key;
	int8_t recv;
    char *device, *imageFile;
	int ret; 
	int cnt = 100;
	int timeout = 1;
	char buf[200];
	int i;
    
    if( argc < 3){
    	printf("Please input image file name and serial device name: \n");
        printf(" FORMAT: emoteflash imageFile deviceName \n");
    	return 0;
    }else{
    		imageFile = argv[1];
    		device = argv[2];
    }

	printf("Emoteflash stm32 flash progrmming version 1.0.0.....\n");
   
    ret= SerialInt(device);
    if(ret <0){
		printf("serial port init failed...\n");
		return 0;
    }

	key = 0x1C; /*send to application to restart*/

	key =0x1D;/*send to bootloader*/
	/* repeatly send key until receive response from bootloader or timeout*/
	while(cnt--){
	 debug_print("send sync information ...\n");	
	 ret= SerialPutChar(key);
	 if(ret != 1)
	 {
	   printf("write key falied...%d\n",ret);
	 }

	 while(Serial_ReceiveByte(&recv,timeout) == 1){
		if(recv== 0x1E){
			debug_print("Ack received...\n");
			SerialUpload(imageFile);
			SerialClose();
			return 1;
			}else
			printf("%c",recv);
			}
	 }
	   
	return 1;
  }

  
  
