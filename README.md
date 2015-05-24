GY80
====

Arduino/Processing sketch for GY80 9DOF AHRS




Data logging to UDP server
==========================

SD Cards only got me about 17hz.  I tried moving this over to UDP
packets with an Adafruit CC3000 wifi shield.  This got me up to
almost 24hz.  Still not quite where I want to be.  I suspect I need
to try a different micro.




Required Libraries - these are the libraries that I used.

This is the library for the Gyro -
https://github.com/pololu/l3g4200d-arduino

This is the library for the Barometric pressure sensor -
https://github.com/adafruit/Adafruit-BMP085-Library

I was able to install the BMP085 library thorugh the new Arduino
1.6.4 library manager.


