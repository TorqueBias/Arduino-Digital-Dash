# Arduino-Digital-Dash
Credits:

"Obeisance"
for the original code that everything here was derived from. There isn't much left of his code besides some variable and subroutine names, but it got me started.
http://www.clubwrx.net/forums/tutorials-diy/134423369-clock-pod-mod-subarb-select-monitor-ecu-polling-arduino.html#/forumsite/20569/topics/134423369?page=2

"Xark" for his really fast graphics libraries (based on the Adafruit libraries). Without his amazing work, the graphical gauges
would be useless.
https://github.com/XarkLabs/PDQ_GFX_Libs

"Bodmer" for his round gauge graphics code which mine is very heavily derived from.
http://www.instructables.com/id/Arduino-analogue-ring-meter-on-colour-TFT-display/

Bill Porter for his EZTransfer library which I use to allow the two Arduino devices to communicate with each other.
http://www.clubwrx.net/forums/tutorials-diy/134423369-clock-pod-mod-subarb-select-monitor-ecu-polling-arduino.html#/forumsite/20569/topics/134423369?page=2

Info:
This project is broken up between two Arduinos. The Arduino Micro handles communication with the ECU and Wideband,
translates the information, and sends it to an Arduino Duemilanove which displays the information on an Adafruit touchscreen.

The communication code is modular and can be easily modified for just about any device that provides a serial data stream.
Currently it is configured to request the datastream from a 2006 Subaru STi and an Innovate Wideband. Most digital dash applications
will not need to request the packets, so you can discard the packet assembler and the code to send it. Then just update the section
that retrieves the data to identify the different packet headers, pull the data, and then calculate the CheckSum if desired.
