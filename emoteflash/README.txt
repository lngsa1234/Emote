
Development System:
Ubuntu 12.04 above

Compiling:
$make

usage:
$emoteflash imagefile moteid /dev/ttyUSB1

Note:

Before you use it, you have to be sure that the USB is working.

How to make usb work?
1. Attach the usb device. If you have probelm with it, you may need to install usb driver. You can download from the link http://www.silabs.com/products/mcu/Pages/USBtoUARTBridgeVCPDrivers.aspx. 
2. Look up if it is attached successfully using command: ls /dev/ttyUSB*. You should can see /dev/ttyUSB0 and /dev/ttyUSB1. We use /dev/ttyUSB1 here.
3. Make sure you have permit to access USB. Use the command: sudo chmod 777 /dev/ttyUSB1.
