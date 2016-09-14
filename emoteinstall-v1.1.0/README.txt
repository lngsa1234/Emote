
1. Use any JTAG flash tool to burn emoteboot.bin to emote board.(tested with ST LINK V2 + STM-LINK Utility)

2. Run emoteflash on Linux system to install contiki.( Ubuntu 12.04 above recommended)
   Example:
   $./emoteflash app_hello_moteid.bin 23 /dev/ttyUSB1

   app_hello_moteid.bin is a test program. Once contiki system is ready，you can replace it here.


Note:
Before you run emoteflash, you have to make sure USB is working.How to make usb work?

1. Attach the usb device(serial port on board) to computer using serial port on board. If you have problem with it, you may need to install usb driver. You can download from the link http://www.silabs.com/products/mcu/Pages/USBtoUARTBridgeVCPDrivers.aspx. 

2. Look up if it is attached successfully using command: ls /dev/ttyUSB*. You should can see /dev/ttyUSB0 and /dev/ttyUSB1. We use /dev/ttyUSB1 here.

3. Make sure you have permit to access USB. Use the command: sudo chmod 777 /dev/ttyUSB1.
