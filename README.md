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

## Phone control (SkySafari / Stellarium Mobile Plus) via WiFi

Planetarium apps do the heavy lifting (catalogs, precession, alignment) and send the
mount standard LX200 commands. Neither app supports BLE (HM-10); they connect via
WiFi/TCP (Android + iOS) or classic Bluetooth SPP (Android only).

**Hardware:** an ESP32 dev board runs [`ESP32-WiFi-Bridge/`](ESP32-WiFi-Bridge/ESP32-WiFi-Bridge.ino)
(own access point, TCP port 4030) and is wired to the Due's Serial1:

| ESP32 | Due |
|---|---|
| GPIO17 (TX2) | pin 19 (RX1) |
| GPIO16 (RX2) | pin 18 (TX1) |
| GND | GND |
| VIN / 5V | 5V |

Both are 3.3 V logic: no level shifter. Serial1 runs 19200 baud in strict LX200 mode.

**App settings:** join WiFi `GM-K150` (password `k150mount`), mount *Meade LX200 Classic*
(SkySafari) / *LX200* (Stellarium Mobile Plus), WiFi/TCP `192.168.4.1` port `4030`.
Let the app set time and location on connect.

### Using GOTO
1. Power up, connect the app (it sends site, time, date: `:Sg :St :SG :SL :SC`).
2. Point at a bright star **with the tube on its normal side (counterweight down)**
   using the hand box, select that star in the app and press **Align / Sync** (`:CM#`).
3. Now select a target and **GoTo** (`:MS#`). The mount slews the short way, makes one
   short correction for the sidereal drift during the slew, then tracks.
4. Any hand-box key, ST-4 signal, or app *Stop* aborts a GOTO.

### Mount model and safety limits (`X-Nucleo-Ardu_tvat-Due.ino`, "GOTO / MOUNT MODEL")
One pier side only, no meridian flip: H position = hour angle (H+ = west = tracking direction),
D position = declination (D+ = north). Zero points come from the sync.

| Setting | Default | Meaning |
|---|---|---|
| `HA_LIMIT_EAST_H` / `HA_LIMIT_WEST_H` | -6.5 / +6.5 h | GOTO targets outside are refused (`:MS#` -> `2...#`) |
| `HA_TRACK_STOP_H` | 7.0 h | tracking stops when the H axis passes this hour angle west |
| `ALT_LIMIT_DEG` | 0 deg | GOTO targets below are refused (`:MS#` -> `1...#`) |
| `GOTO_SPEED` | 800 full steps/s | ~1.7 deg/s slew speed |

GOTO is refused until the first sync; limits are not applied before it either.

### Longitude convention
LX200: **west positive** (`:Sg+072*55#` = 72 deg 55' W). Old test notes in this repo used
`:Sg-072:55#`; that now means 72 deg 55' **east**.
