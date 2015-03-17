# emote
STM32 embedded development. Include flash programming in Linux, bootloader on STM32 board, Tinyos v2.1 supporting STM32 and Keil Project.

The uart component and its driver are added.

you can run the commands below and get the main.exe.
$ cd ~/\TinyOS-STM-v0.9b\apps\UsartSerialTest
$ make emote.

If you have your own tinyos, you can followe the steps below.

1. copy ~\TinyOS-STM-v0.9b\tos\chips\stm32\usb-serial to the corresponding dir of your tinyos.
2. make revision to HplSTM32InterruptM.nc(comment USART1_IRQHandler(). Tis funtion is redefined in uart component)
3. make revision to ~\TinyOS-STM-v0.9b\tos\platforms\emote\tos.x(add compiler path)
4. cope ~\TinyOS-STM-v0.9b\apps\UsartSerialTest to your corresponding dir.
$ cd ~/\TinyOS-STM-v0.9b\apps\UsartSerialTest
$ make emote.

Note: only the uart component is tested.  
