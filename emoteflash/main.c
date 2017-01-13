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


#define TIMEOUT 100
#define REPEAT_TIME 2

#define COMMAND_RESTART      0x4c  
#define COMMAND_DOWNLOAD     0x44
#define COMMAND_SEND_MOTEID  0x4D  
#define COMMAND_MOTESYNC     0x16  
#define COMMAND_ACK          0x06

#define PROGRAM_NAME "EMOTEFLASH"

/*
void signal_handler_IO (int status)
{

}
*/

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

static uint32_t SerialUpload(const char* filename)
{
	uint32_t status = 0;
	FILE* fp;
	int32_t fileSize;
	char* buf;

	debug_print("[%s]DEBUG:enter function %s.\n", PROGRAM_NAME, __func__); 

	fp = fopen(filename,"r");
	if( fp == NULL){
		printf("[%s]ERROR:open image file failed...\n", PROGRAM_NAME);
		return -1;
	}

	/*get file size*/
	fseek(fp, 0, SEEK_END);
	fileSize = ftell(fp);
	fseek(fp,0,SEEK_SET); 

	/*read file into buffer*/
	buf = malloc(fileSize);
	if(buf == NULL){
		printf("malloc buffer failed...\n");
		return -1;
	}

	if(fread(buf,1,fileSize,fp) != fileSize){
		printf("read file failed...\n");
	}


	// if (Serial_ReceiveByte(&c,-1) == 1&& c==CRC16)
	{
		//debug_print("%d",c);	
		printf("[%s]INFO:uploading image %s...\n", PROGRAM_NAME, filename);
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
	return status;
}

int main(int argc, char *argv[])
{
	int8_t key;
	int8_t recv;
	char *device, *imageFile;
	int cnt = 0;
	int rep = 0; 
	uint32_t i = 0;
	uint16_t moteid = 0;
	char sbuf[8];
	FILE *fp;
	int readings = 0;

	/*Parameters*/    
	if (argc < 4){
		printf("Please input image file name and serial device name: \n");
		printf("FORMAT: emoteflash imageFile moteid deviceName \n");
		exit(-1);
	}else{
		imageFile = argv[1];
		moteid = atoi(argv[2]);
		device = argv[3];

	}

	/*Print version information*/
	printf("Emoteflash stm32 flash progrmming version 1.0.0, COPYRIGHT DNC 2016.\n");
       
        /*verify if image file exits or readable */	
	fp = fopen(imageFile,"r");
	if (fp == NULL) {
		printf("open the image file failed...\n");
		exit(-1);
	}
	fclose(fp);
        fp = NULL;

	/*Init serial port*/
	if (SerialInt(device)<0)
	{
		printf("serial port init failed...\n");
		exit(-1);
	}

	while (cnt++ < REPEAT_TIME)
	{

		key = 0x4C; /*send to application to restart*/
		if(SerialPutChar(key)!=1)
		{
			printf("[%s] ERROR:send byte %c faild.\n", PROGRAM_NAME, key);
			SerialClose();
			exit(-1);
		}else
		{
			debug_print("[%s]DEBUG:sending restart command..., %d time\n ", PROGRAM_NAME, cnt); 
		}

		/* Read and show the first byte from serial port*/
		if(Serial_ReceiveByte((char *)&recv, 100) == 1)
			printf("%c",recv);
		else{
			
			printf("[%s]ERROR:receive no response from emote and is exiting.\n", PROGRAM_NAME);
			SerialClose();
			exit(-1);
		}
                
                /*i++ < 1000; fix the bug when app doesn't restart and print constantly*/
		while(Serial_ReceiveByte((char *)&recv, 1) == 1 && (readings++ < 1000))
		{
			printf("%c",recv);
		}

		key =0x44;/*send to bootloader to request downloading*/
		if(SerialPutChar(key)!=1)
		{
			printf("[%s]ERROR:send byte %c faild.\n", PROGRAM_NAME, key);
			SerialClose();
			exit(-1);
		}
		printf("[%s]INFO:sending request to emote for dowloading...\n", PROGRAM_NAME);

		/* Read and show any byte from serial port*/
		while (rep++ < REPEAT_TIME)	
		{
			debug_print("[%s]DEBUG:wait to receive ack from emote, %d time\n", PROGRAM_NAME, rep);        
			if(Serial_ReceiveByte((char *)&recv, TIMEOUT) == 1 && recv == 0x06)/*receive repsonse*/
			{
				printf("[%s]INFO:receive ack response 0x%x from emote...\n", PROGRAM_NAME, recv);
				if(SerialUpload(imageFile)==0)
				{ 
					SerialPutChar(0x4D);
					if(Serial_ReceiveByte((char *)&recv, TIMEOUT) == 1 && recv == 0x52);
					{  
						sprintf(sbuf,"%d", moteid);
						debug_print("%s \n", sbuf);
						printf("[%s]INFO:sending mote id to emote....\n", PROGRAM_NAME);
						while(sbuf[i] != '\0')
						{
							SerialPutChar(sbuf[i++]);
						}

						while(Serial_ReceiveByte((char *)&recv, TIMEOUT) == 1 && recv !=0x16)
						{
							printf("%c", recv);
						}
						if( recv == 0x16)
							printf("\n[%s]INFO:emote image update has been done successfully.\n", PROGRAM_NAME);
						else {
							printf("\n[%s]INFO:emote image update failed.Please try again.\n", PROGRAM_NAME);
							SerialClose();
							exit(-1);
						}
					}

                                        
				}
					SerialClose();
					return 0;
                                
			}
			else if(SerialPutChar(key)!=1) /*send request again if not responded*/
			{
				printf("[%s]ERROR:send byte %c faild.\n", PROGRAM_NAME, key);
				SerialClose();
				exit(-1);
			}  
		}  
	}
	SerialClose();
	return 1;
}



