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
   while(timeout-- > 0 || ref)
   {
     if(SerialGetChar(c)== 1)
	return 1;
   }
   return 0;
  }
  
  static void SerialUpload(const char* filename)
  {
   uint32_t status = 0;
   FILE* fp;
   int32_t fileSize;
   char* buf;
   
   debug_print("Enter function: SerialUpload...\n"); 
   if(filename == NULL)
   {
     printf("no image file.\n");
     return;
   } 

   fp = fopen(filename,"r");
   if(fp<0){
    printf("open the image file failed...\n");
    return;
   }
 
  /*get file size*/
  fseek(fp, 0, SEEK_END);
  fileSize = ftell(fp);
  fseek(fp,0,SEEK_SET); 

 /* read file into buffer*/
  buf = malloc(fileSize);
  if(buf == NULL){
   printf("malloc buffer failed...\n");
   return;
  }
 
 if(fread(buf,1,fileSize,fp) != fileSize){
  printf("read file failed...\n");
  }
 

 // if (Serial_ReceiveByte(&c,-1) == 1&& c==CRC16)
 {
        //debug_print("%d",c);	
        printf("Uploading image %s...\n",filename);
	status = Ymodem_Transmit((uint8_t *)buf, (const uint8_t *)filename, fileSize);
	if (status != 0){
		printf("Error Occured while Transmitting File!please try again!\n");
		}else{
		printf("Total %d bytes uploaded Successfully \n", fileSize);
		}
		}
  free(buf);
  buf =NULL;
  fclose(fp); 
  }
 
  int main(int argc, char *argv[])
  {
    int8_t key;
    int8_t recv;
    char *device, *imageFile;
    int cnt = 100;
    int rep=10; 
    /*Parameters*/    
    if( argc < 3){
    	printf("Please input image file name and serial device name: \n");
        printf("FORMAT: emoteflash imageFile deviceName \n");
    	return 0;
    }else{
    	imageFile = argv[1];
    	device = argv[2];
    }
   
    /*Print version information*/
    printf("Emoteflash stm32 flash progrmming version 1.0.0.....\n");
    /*Init serial port*/
    if(SerialInt(device)<0)
    {
	printf("serial port init failed...\n");
	return 0;
    }

    while(cnt-->0)
    {

	key = 0x4C; /*send to application to restart*/
        if(SerialPutChar(key)!=1)
        {
          printf("send byte %c faild.\n", key);
        }else
        {
          debug_print("sending restart command...\n "); 
        }
        /* Read and show any byte from serial port*/
	 
        if(Serial_ReceiveByte((char *)&recv,10) == 1)
 
           printf("%c",recv);

        while(Serial_ReceiveByte((char *)&recv,1) == 1)
        {
           printf("%c",recv);
	}
        key =0x44;/*send to bootloader*/
        if(SerialPutChar(key)!=1)
        {
          printf("send byte %c faild.\n", key);
        }
        /* Read and show any byte from serial port*/
       rep=10; 
       while(rep-->0)	
       {
        debug_print("send %d\n", 10-rep);        
        if(Serial_ReceiveByte((char *)&recv,1) == 1)
        {
	  debug_print("Ack received...\n");
          SerialUpload(imageFile);
          SerialClose();
	  return 1;
	 }
        else
        if(SerialPutChar(key)!=1)
        {
          printf("send byte %c faild.\n", key);
         }  
       }  
    }
   return 1;
  }

  
  
