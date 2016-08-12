
This is a test program. The contiki system can refer it to learn how to use UART port and obtain mote id.

Add all driver files in the folder emlib to make UART and EEPROM work.

The serial port of the emote board uses uart 1 port. The configuration of UART 1 can be referred to uart1_conf.c.

See main.c to know how to read mote id.

The contiki system should be stored in the flash memory starting with address 0x 08060000.
Make sure to change your linker script to make the system work.