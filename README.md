[![License: GPL v3](https://img.shields.io/github/license/AndrewBCN/STM32-GPSDO)](https://www.gnu.org/licenses/gpl-3.0) [![License: CC BY-SA 3.0](https://img.shields.io/badge/License-CC_BY--SA_3.0-lightgrey.svg)](https://creativecommons.org/licenses/by-sa/3.0/)

# An STM32 GPSDO

Designing, assembling, programming and testing an ultra-low-cost GPS-disciplined oven controlled crystal oscillator / lab clock based on an 100 MHz STM32 32-bit ARM microcontroller

**Keywords**: STM32, GPS, GPSDO, OCXO, 10MHz, crystal oscillator, frequency standard, atomic clock

## Introduction

I have been following the thread by the late Lars about his inspirational DIY Arduino GPSDO project. And I checked the dozen or so GPSDO projects published on the net.
Then I decided to join the party and design and build my own.

I didn't quite know what I was getting into, but after a couple of months waiting for the parts to come from China, and then another couple of months writing the firmware, I have a working, blinking, timekeeping and happily and precisely oscillating GPSDO.

Like with Lars' design, any OCXO, DOCXO or even rubidium source can be used, but the recommended oscillator is an inexpensive used square wave 10MHz 5V OCXO, which is what I am using in the breadboard prototype. These OCXOs are available from various sources on the net, recycled from decommissioned telecom equipment and sometimes still soldered onto a piece of PCB, for around 10€ or less (< US $12). The OCXO is the most expensive part in this GPSDO.

For Lars' design check following links:
- Original Lars' EEVBlog forum thread: https://www.eevblog.com/forum/projects/lars-diy-gpsdo-with-arduino-and-1ns-resolution-tic/
- GitHub project combaning all information in one place: https://github.com/AndrewBCN/Lars-DIY-GPSDO

The second most expensive part is the GPS receiver module. I strongly recommend a u-blox Neo-M8 GPS module with an SMA antenna connector. With a u-blox Neo-M8, I am still getting 5~9 satellites even indoors in my basement lab. The much cheaper Neo-M6 struggles to get a fix in the same conditions.

## Features

I have decided to build my own version of a **10MHz GPSDO** with the following features:
- **Very low cost i.e**. < 30 EUR ($35), short and easily purchased BOM.
- **Acceptable performance (+/- 1ppb)** "out of the box", can be fine tuned over time.
- Uses an **100 MHz STM32F411CEU6 "Black Pill"** microcontroller module, programmed in C/C++ using the Arduino IDE ([STM32duino](https://github.com/stm32duino/Arduino_Core_STM32)).
- Requires only a **5V @ 1A** power supply, supplied through a USB C cable.
- Compact and portable.
- Optionally battery powered (allows taking it outdoors for optimal satellite signal reception).
- **Optional modules**
  - **BMP280** for atmospheric pressure and temperature monitoring,
  - **AHT10**  for temperature and humidity monitoring,
  - **INA219** for OCXO power consumption monitoring,
  - **HC-06** for wireless communication with a PC/mobile using Bluetooth UART.
- Extensive logging of various operating parameters to allow further software tuning.
- An optional small OLED displaying room temperature, UTC time, uptime, operating status, measured frequency.
- **New!** Optional UTC-aligned 1PPS output using a **picDIV**.
- It uses a digital **FLL (Frequency Locked Loop)**,
  - **New!** It can also use a **PLL (Phase Locked Loop)**.
- **OXCO Voltage control** using a **16-bit DAC** provided by a **2kHz PWM** output pin on the STM32F411CEU6
  - [A quick discussion of noise, DAC resolution and OCXO frequency control](docs/DAC_resolution_discussion.md)
  - [Notes on using a software 16-bit PWM DAC instead of an external 12-bit I2C DAC](docs/DAC_STM32_16bit_PWM_notes.md)

## Schematics

Latest (0.6.2 revision) schematics are available here: [EDA/Schematics-STM32-GPSDO-0.6.2.pdf](EDA/Schematics-STM32-GPSDO-0.6.2.pdf)

## The first breadboard prototype

![GPSDO Breadboard Prototype](img/Prototype_GPSDO_breadboard.jpg)

![GPSDO Two Breadboard Prototypes](img/Prototype_TwoGPSDOs.jpg)

In normal operation this is displayed on the small OLED:
![OLED display](img/Prototype_OLEDv002i_expl.jpg)

![Oscilloscope 10MHz Square Wave](img/Prototype_10MHz_cleantrace1a.jpg)

You can see it in action on YouTube video (click image below):

https://www.youtube.com/watch?v=mgoK4KuVDhw

[![A $35 DIY GPSDO - with an STM32F411CEU6 MCU](https://img.youtube.com/vi/mgoK4KuVDhw/0.jpg)](https://www.youtube.com/watch?v=mgoK4KuVDhw)

## Links

- GPS
  - [Timing and Location Performance of Recent u-blox GNSS Receiver Modules by John Ackermann N8UR](https://hamsci.org/sites/default/files/publications/2020_TAPR_DCC/N8UR_GPS_Evaluation_August2020.pdf
)
    
## License

- Software: **GPL-3.0**
  - [![License: GPL v3](https://img.shields.io/github/license/AndrewBCN/STM32-GPSDO)](https://www.gnu.org/licenses/gpl-3.0)
- Hardware: **CC BY-SA 3.0**
  - [![License: CC BY-SA 3.0](https://img.shields.io/badge/License-CC_BY--SA_3.0-lightgrey.svg)](https://creativecommons.org/licenses/by-sa/3.0/)


## Authors

- André Balsa, April 2021
