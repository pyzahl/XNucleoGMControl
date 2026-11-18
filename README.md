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

**Module:** Espressif **ESP32-DevKitC V4 with ESP32-WROOM-32E** (or a 30-pin "ESP32 DevKit V1"
clone with a WROOM-32 module). Use **ESP32-DevKitC-32UE** (u.FL antenna socket) if the electronics
sit in a metal box. Avoid WROVER modules (their PSRAM occupies GPIO16/17) and ESP32-C3/C6/S2
(different chip, sketch would need changes). Build it as *ESP32 Dev Module* (Boards Manager:
*esp32 by Espressif*).

**Power:** WiFi peaks ~250 mA. If the Due runs from 12 V on its barrel jack, do not load its
5 V regulator with the ESP32 -- power the ESP32 from its own USB (power bank / charger) or a small
12->5 V buck converter, and keep GND connected to the Due. On USB-powered setups the Due 5V pin is fine.

### Due pin map (as used by the firmware)

Check this against your own wiring -- the Uno-layout X-NUCLEO shields needed rewiring on the Due,
and only the firmware side is documented here. The shields themselves only reach the Uno-style
header (D0-D13, A0-A5); pins 14-21 are on the Due's extended header.

| Due pin | Use |
|---|---|
| 0 / 1 | `Serial` (programming port USB) -- console, main.py |
| native USB | `SerialUSB` -- console (`comm_master`) |
| 4 | SD card CS (SD disabled, pin held HIGH) |
| 5 | powerSTEP nBUSY (defined, not used -- status read via SPI) |
| 6 | OLED SSD1351 RST |
| 7 | OLED SSD1351 DC |
| 8 | powerSTEP nSTBY/nRESET (all 3 boards) |
| 9 | powerSTEP STCK (defined, not used) |
| 10 | powerSTEP nCS (daisy chain H, D, F) |
| 52 | OLED SSD1351 CS |
| SPI header (ICSP) MOSI/MISO/SCK | powerSTEP + OLED (+ SD) -- on the Due SPI is only on this header, not D11-D13 |
| 14 / 15 | `Serial3` TX3/RX3 -- HM-10 BLE (9600 baud), normal console protocol |
| 16 / 17 | `Serial2` TX2/RX2 -- opened at 19200, **free / unused** |
| **18 / 19** | **`Serial1` TX1/RX1 -- ESP32 WiFi bridge (strict LX200, 19200 baud)** |
| 20 / 21 | I2C SDA/SCL -- MCP23017 keypad + ST-4 (address 0x24) |

**If 18/19 are taken by your rewiring:** use the free `Serial2` (pins 16/17) instead -- in the
Due sketch change `comm_0 = new Communication ("COMM_WIFI_SERIAL1", &Serial1, NULL, true);` to
`&Serial2` (both run 19200). Or replace the HM-10 on `Serial3` (pins 14/15, already wired) with
the ESP32: use `&Serial3` there, remove the `comm_blue` line, and set `SERIAL3_BAUDRATE` to 19200
(the start-up `AT` probe on Serial3 can stay; the bridge drops data until a client connects).

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

### Guiding (ST-4 port or LX200 pulse guiding)
ST-4 inputs and `:Mg[nsew]NNNN#` pulses share one engine: RA guiding runs the tracking at
(1 +- guide rate) x sidereal (west = faster, east = slower, the RA motor never reverses);
Dec runs at +- guide rate. Default guide rate 0.5x sidereal, set with `:RgSS.S#` (arcsec/s),
read with `:Rg#`. PHD2 etc. calibrate the directions themselves.

### INDI / KStars / PyStarLight
Driver `indi_lx200generic` (device "Standard LX200"):
- **USB:** use the Due's **native USB port** (`/dev/serial/by-id/usb-Arduino_LLC_Arduino_Due-if00`):
  opening it does not reset the Due (the programming port does, and the start-up then takes ~10 s).
  Baud rate is irrelevant on the native port (19200 on the programming port).
- **WiFi:** connection mode TCP, `192.168.4.1:4030` (ESP32 bridge).
- The driver's `<ACK>` probe switches that port to strict LX200 automatically.
- Sync once (Align / sync on a star) before GOTO; park = pole, counterweight down.
- PyStarLight `[indi]`: `mount_driver = "indi_lx200generic"`, `mount_connection = "usb"` with
  `mount_port` = the native USB device above, or `"wifi"` with `mount_port = "192.168.4.1:4030"`.

### Longitude convention
LX200: **west positive** (`:Sg+072*55#` = 72 deg 55' W). Old test notes in this repo used
`:Sg-072:55#`; that now means 72 deg 55' **east**.
