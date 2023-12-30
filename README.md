# XNucleoGMControl

German Telescope Mount Motor Control

Implemented basic "LX200 protcol" http://www.meade.com/support/TelescopeProtocol_2010-10.pdf (via USB Serial)

Using ST Motor Driver Modules adapted for Arduino Due (See code for pin out, required a slight rerouting for Due)

https://www.st.com/en/ecosystems/x-nucleo-ihm03a1.html
X-NUCLEO-IHM03A1
High power stepper motor driver expansion board based on powerSTEP01 for STM32 Nucleo 


X-Nucleo-Driver 3 Stepper Motor Configuration (Hour, Declination, 3rd for focuser)
X-Nucleo Drivers via SPI on Adruino/Gunino Uno ONLY without modifcations
ATTENTION: for Due a few signal wire reroutings are required!

Essential Addons: I2C IO Expansion (MCP23017) for Hand Control Box on simple 2 wire+power i2C cable (using an Ethernetcable!).
Mini TFT Display: Adafruit_SSD1351 display

Optional: SD Card

Code is configured for Arduino Due Platform, but fairly poratble.

Uses: SPI.h, powerSTEP01ArduinoLibrary.h, Adafruit_GFX.h, Adafruit_SSD1351.h, HardwareSerial.h, DueTimer.h, AstroMini_Library.h, Wire.h


