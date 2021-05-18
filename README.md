# An STM32 GPSDO
Designing, assembling, programming and testing an ultra-low-cost GPS-disciplined oven controlled crystal oscillator / lab clock based on an STM32 32-bit ARM microcontroller

Keywords: STM32, GPS, GPSDO, crystal oscillator, frequency standard, atomic clock

André Balsa, April 2021

The Project
===========

I decided to build my own version of a 10MHz GPSDO with the following features:
- Very low cost i.e. < 30 euros ($35), short and easily purchased BOM.
- Acceptable performance (+/- 1ppb) "out of the box", can be fine tuned over time.
- Uses an STM32F411CEU6 "Black Pill" microcontroller module, programmed in C/C++ using the Arduino IDE (STM32duino).
- Requires only a 5V @ 1A power supply, supplied through a USB C cable.
- Compact and portable.
- Optionally battery powered (allows taking it outdoors for optimal satellite signal reception).
- An optional suite of environmental sensors (temperature, humidity, air pressure, voltages, OCXO current/power consumption).
- An optional Bluetooth serial interface for wireless communication with a PC.
- Extensive logging of various operating parameters to allow further software tuning.
- An optional small OLED displaying room temperature, UTC time, uptime, operating status, measured frequency.

The first breadboard prototype:
<img src="https://github.com/AndrewBCN/STM32-GPSDO/raw/main/GPSDO_breadboarda.jpg">

In normal operation this is displayed on the small OLED:
<img src="https://github.com/AndrewBCN/STM32-GPSDO/raw/main/OLEDv002i_expl.jpg">
