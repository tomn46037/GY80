GY80
====

Arduino/Processing sketch for GY80 9DOF AHRS




Data logging to SD card
=======================

I am wanting to log this data to an SD card so it can be reviewed
later after the flight.  This data will be logged to an SD card.

Updated - It now writs to a binary log format.  There is a corrisponding
python script to read that binary file as well.  This method is
very inefficient.  I can only get about 16 hz out of it.

It's taking about 60hz for each pass through the loop.

My next thought is to move up to a 1284p and buffer up a sector or
two before I try to write them to the file.



Required Libraries
==================

This is the library for the Gyro -
https://github.com/pololu/l3g4200d-arduino

This is the library for the Barometric pressure sensor -
https://github.com/adafruit/Adafruit-BMP085-Library

I was able to install the BMP085 library thorugh the new Arduino
1.6.4 library manager.


