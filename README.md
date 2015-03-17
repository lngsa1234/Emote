# emote
STM32 embedded development. Include flash programming in Linux, bootloader on STM32 board, Tinyos v2.1 supporting STM32 and Keil Project.

This project is to support tinyos download from bootloader. 

Two main parts are included: emoteflash and emote bootloader.

Emoteflash runs in Linux enrionment. It functions like a flash programming.

Emote bootloader is a Keil Project. The bootloader program can be directly downloaded to board by keil via JTAG.

In order to support debugging, you have two options: using serial port or using keil.

TinyOS-STM-v0.9b is a linux based sourece code. Uart component is added into this code. 
You can use uart component to debug your code with serail port.

Tinyos-stm32-keil is a keil project. It supports debugging tinyos via keil. You can compile your tinyos code in Linux first. 
Then use the appmangle.pl script (included in TinyOS-STM-v0.9b)to convert app.c into the format supported by Keil. Then use keil
to debug your tinyos.  
