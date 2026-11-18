// ESP32 WiFi <-> serial bridge for the GM K150 mount controller (Arduino Due).
//
// The ESP32 opens its own WiFi access point and passes everything between one
// TCP client (SkySafari, Stellarium Mobile Plus, KStars/INDI, ...) and the Due's
// Serial1, where the firmware speaks plain LX200.
//
// Phone app settings:
//   WiFi network : GM-K150   (password below)   -- no internet on this network
//   Mount type   : Meade LX200 Classic (SkySafari) / LX200 (Stellarium Mobile Plus)
//   Connection   : WiFi / TCP,  IP 192.168.4.1,  port 4030
//
// Wiring (both boards are 3.3 V logic -- connect directly, no level shifter):
//   ESP32 GPIO17 (TX2)  ->  Due pin 19 (RX1)
//   ESP32 GPIO16 (RX2)  <-  Due pin 18 (TX1)
//   ESP32 GND           --  Due GND
//   ESP32 VIN / 5V      <-  Due 5V   (or power the ESP32 from its own USB)
//
// Board: any ESP32 dev board ("ESP32 Dev Module"), Arduino-ESP32 core.

#include <WiFi.h>

const char*    AP_SSID  = "GM-K150";
const char*    AP_PASS  = "k150mount";  // at least 8 characters
const uint16_t TCP_PORT = 4030;         // SkySafari default (SkyFi)

#define DUE_RX_PIN  16
#define DUE_TX_PIN  17
#define DUE_BAUD    19200               // must match SERIAL1_BAUDRATE in the Due firmware

#define LED_PIN     2                   // on-board LED: on while a client is connected

WiFiServer server (TCP_PORT);
WiFiClient client;

void setup (){
  Serial.begin (115200);                                   // USB debug output
  Serial2.begin (DUE_BAUD, SERIAL_8N1, DUE_RX_PIN, DUE_TX_PIN);
  pinMode (LED_PIN, OUTPUT);
  digitalWrite (LED_PIN, LOW);

  WiFi.mode (WIFI_AP);
  WiFi.softAP (AP_SSID, AP_PASS);
  server.begin ();
  server.setNoDelay (true);

  Serial.print ("GM-K150 WiFi bridge: ");
  Serial.print (AP_SSID); Serial.print (" @ ");
  Serial.print (WiFi.softAPIP ()); Serial.print (":"); Serial.println (TCP_PORT);
}

void loop (){
  uint8_t buf[256];
  size_t n;

  // a new connection replaces the old one (apps reconnect after sleep / app switch)
  if (server.hasClient ()){
    if (client) client.stop ();
    client = server.available ();
    client.setNoDelay (true);
    while (Serial2.available ()) Serial2.read ();           // drop stale replies
    Serial.println ("client connected");
  }

  bool connected = client && client.connected ();
  digitalWrite (LED_PIN, connected ? HIGH : LOW);

  if (connected){
    while ((n = client.available ()) > 0){                  // app -> mount
      n = client.read (buf, n < sizeof (buf) ? n : sizeof (buf));
      Serial2.write (buf, n);
    }
    while ((n = Serial2.available ()) > 0){                 // mount -> app
      n = Serial2.readBytes (buf, n < sizeof (buf) ? n : sizeof (buf));
      client.write (buf, n);
    }
  } else {
    while (Serial2.available ()) Serial2.read ();           // nobody listening
  }
  delay (1);
}
