# XNucleoGMControl

German Mount Telescope Motor Control

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



## Building (Arduino IDE)

The repo is laid out as a complete Arduino sketchbook:

```
XNucleoGMControl/                     <- sketchbook root
  X-Nucleo-Ardu_tvat-Due/
    X-Nucleo-Ardu_tvat-Due.ino        <- the sketch
  libraries/                          <- every library the sketch needs
    AstroMini_Library/                   custom
    powerSTEP01_Arduino_Library/         locally patched (signed 22-bit positions) - do NOT use the Library Manager version
    Adafruit_GFX_Library/                1.11.9
    Adafruit_BusIO/                      1.16.1
    Adafruit_SSD1351_library/            1.3.2
    DueTimer/                            1.4.7
```

1. `git clone https://github.com/pyzahl/XNucleoGMControl`
2. Make the IDE find the libraries -- either
   - **A (simplest):** File > Preferences > *Sketchbook location* = the cloned `XNucleoGMControl` folder
     (restart the IDE), or
   - **B (keep your sketchbook):** run `./link-libraries.sh` -- symlinks `libraries/*` into
     `~/Documents/Arduino/libraries` (macOS) or `~/Arduino/libraries` (Linux). Existing real folders
     with the same name are skipped; remove them first to use the repo versions.
3. Boards Manager: install **Arduino SAM Boards (32-bits ARM Cortex-M3)**, select **Arduino Due (Programming Port)**.
4. Open `X-Nucleo-Ardu_tvat-Due/X-Nucleo-Ardu_tvat-Due.ino`, Verify / Upload.
