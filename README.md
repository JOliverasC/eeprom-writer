# eeprom-writer
Arduino firmware for EEPROM Writer project.

This code should be compiled by the Arduino IDE and run on an Arduino Mega 2560.

For project documentation, refer to http://danceswithferrets.org/geekblog/?page_id=903

Distributed under an acknowledgement licence, because I'm a shallow, attention-seeking tart. :)
=================================================================================================

Thanks to oddblk from http://www.danceswithferrets.org/ for the original idea & code 

Improvements: 
1).-Implement JEDEC Standard Software Data Protection command sequence (already implemented but now with full addresses)
2).-Implement JEDEC Chip-Erase operation void BlankEEPROM()
3).-Implement Page Program (Or Page Write) because I try to program a SST29EE010 memory and every time a program 16 bytes, the current page was erased. Now the program receives 127 bytes and then writes them all 

I also wrote a C# clone of eewriter (http://danceswithferrets.org/geekblog/?p=924) and I will release the source code. It's mandatory in order to use the Page Write functionality   
