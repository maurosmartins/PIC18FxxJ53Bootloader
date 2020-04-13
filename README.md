# PIC18FxxJ53Bootloader
Serial Bootloader for the PIC18FxxJ53 family of Microcontrollers

The project consists of both FW for the Microcontroller and a Python command line application to download the new firmware.

##Bootloader
The booloader lives in the first 0x400 bytes of microcontroller memory. The firmware to be downloaded must have a corresponding 0x400 code offset (in XC8 this is easily achieved using the code offset option). 
Current version uses internal oscillator at 8MHz and a baud rate of 19200bps (8N1).

##Python Application
It is not yet possible to directly read a .hex file. As a workaround SRecord can be used to convert a .hex file to a .bin that the python application can use.

srec_cat usage 'srec_cat inputfile.hex -Intel -o outputfile.bin -Binary'

application usage 'python bootloader.py outputfile.bin'
previous command will default to a 19200bps (8N1), other baud rates can be selected using the '-b' option followed by the desired baudrate. 

##Bootloader Usage
To download a firmware to the microcontroller simply make a reset or power up the microcontroller and exececute bootloader.py within 5 seconds. 
