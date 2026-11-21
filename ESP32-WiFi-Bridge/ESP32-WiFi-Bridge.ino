// ESP32 WiFi <-> serial bridge for the GM K150 mount controller (Arduino Due).
//
// Passes everything between one TCP client (SkySafari, Stellarium Mobile Plus,
// KStars/INDI, ...) and the Due's Serial3 (the former HM-10 BLE socket), where the
// firmware speaks plain LX200 (ESP32_ON_SERIAL3 1 in the Due sketch).
//
// Network:
//   1. Joins the home WiFi from secrets.h (copy secrets.h.example -> secrets.h).
//      Reachable as the IP the router assigns (make a DHCP reservation, or set
//      STATIC_IP in secrets.h) and, on PCs, as gm-k150.local.
//   2. If that network is not found within WIFI_CONNECT_TIMEOUT_MS (or secrets.h
//      leaves HOME_SSID empty), it opens its own access point instead:
//        WiFi GM-K150 (password below), IP 192.168.4.1 -- no internet there.
//      In fallback mode it reboots every AP_RETRY_HOME_MS while nobody is
//      connected, so it finds the home network again after a router outage.
//   The address in use is printed on the USB serial port (115200) at boot.
//
// Phone app settings:
//   Mount type   : Meade LX200 Classic (SkySafari) / LX200 (Stellarium Mobile Plus)
//   Connection   : WiFi / TCP, IP as above, port 4030
//
// Wiring (both boards are 3.3 V logic -- connect directly, no level shifter):
//   ESP32 GPIO17 (TX2)  ->  Due pin 15 (RX3)   (the HM-10 TXD wire)
//   ESP32 GPIO16 (RX2)  <-  Due pin 14 (TX3)   (the HM-10 RXD wire)
//   ESP32 GND           --  Due GND
//   ESP32 VIN / 5V      <-  Due 5V   (or power the ESP32 from its own USB)
//   (ESP32_ON_SERIAL3 0 in the Due sketch: Due pins 19 RX1 / 18 TX1 instead)
//
// Board: any ESP32 dev board ("ESP32 Dev Module"), Arduino-ESP32 core.

#include <WiFi.h>
#include <ESPmDNS.h>

#if __has_include("secrets.h")
#include "secrets.h"
#endif
#ifndef HOME_SSID
#define HOME_SSID ""                    // no secrets.h: access point only
#define HOME_PASS ""
#endif

const char*    HOSTNAME = "gm-k150";    // DHCP name and mDNS gm-k150.local
const char*    AP_SSID  = "GM-K150";
const char*    AP_PASS  = "k150mount";  // at least 8 characters
const uint16_t TCP_PORT = 4030;         // SkySafari default (SkyFi)

#define WIFI_CONNECT_TIMEOUT_MS  15000UL
#define AP_RETRY_HOME_MS         300000UL   // fallback AP: retry home WiFi every 5 min when idle

#define DUE_RX_PIN  16
#define DUE_TX_PIN  17
#define DUE_BAUD    19200               // must match SERIAL1_BAUDRATE in the Due firmware

#define LED_PIN     2                   // on-board LED: on while a client is connected

WiFiServer server (TCP_PORT);
WiFiClient client;
bool       ap_mode = false;
uint32_t   idle_since = 0;

bool join_home_wifi (){
  if (strlen (HOME_SSID) == 0) return false;

  WiFi.mode (WIFI_STA);
  WiFi.setHostname (HOSTNAME);
#ifdef STATIC_IP
  WiFi.config (IPAddress (STATIC_IP), IPAddress (STATIC_GATEWAY),
               IPAddress (STATIC_SUBNET), IPAddress (STATIC_GATEWAY));
#endif
  WiFi.setAutoReconnect (true);         // rejoin by itself if the router drops us later
  WiFi.begin (HOME_SSID, HOME_PASS);

  Serial.print ("joining "); Serial.print (HOME_SSID);
  uint32_t t0 = millis ();
  while (WiFi.status () != WL_CONNECTED && millis () - t0 < WIFI_CONNECT_TIMEOUT_MS){
    digitalWrite (LED_PIN, !digitalRead (LED_PIN));      // blink while joining
    delay (250);
    Serial.print (".");
  }
  Serial.println ();
  digitalWrite (LED_PIN, LOW);
  return WiFi.status () == WL_CONNECTED;
}

void setup (){
  Serial.begin (115200);                                   // USB debug output
  Serial2.begin (DUE_BAUD, SERIAL_8N1, DUE_RX_PIN, DUE_TX_PIN);
  pinMode (LED_PIN, OUTPUT);
  digitalWrite (LED_PIN, LOW);

  IPAddress ip;
  if (join_home_wifi ()){
    WiFi.setSleep (false);              // no modem sleep: keeps LX200 round trips fast
    ip = WiFi.localIP ();
    if (MDNS.begin (HOSTNAME))
      MDNS.addService ("lx200", "tcp", TCP_PORT);
    Serial.print ("GM-K150 WiFi bridge on "); Serial.print (HOME_SSID);
    Serial.print (": "); Serial.print (ip);
    Serial.print (" ("); Serial.print (HOSTNAME); Serial.print (".local)");
  } else {
    ap_mode = true;
    WiFi.disconnect (true);
    WiFi.mode (WIFI_AP);
    WiFi.softAP (AP_SSID, AP_PASS);
    ip = WiFi.softAPIP ();
    Serial.print ("GM-K150 WiFi bridge, own access point "); Serial.print (AP_SSID);
    Serial.print (": "); Serial.print (ip);
  }
  Serial.print (":"); Serial.println (TCP_PORT);

  server.begin ();
  server.setNoDelay (true);
  idle_since = millis ();
}

void loop (){
  uint8_t buf[256];
  size_t n;

  // a new connection replaces the old one (apps reconnect after sleep / app switch)
  if (server.hasClient ()){
    if (client) client.stop ();
    client = server.accept ();
    client.setNoDelay (true);
    while (Serial2.available ()) Serial2.read ();           // drop stale replies
    Serial.print ("client connected: "); Serial.println (client.remoteIP ());
  }

  bool connected = client && client.connected ();
  digitalWrite (LED_PIN, connected ? HIGH : LOW);

  if (connected){
    idle_since = millis ();
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
    // fallback AP and nobody using it: reboot now and then to look for the home WiFi again
    if (ap_mode && strlen (HOME_SSID) > 0 && WiFi.softAPgetStationNum () == 0
        && millis () - idle_since > AP_RETRY_HOME_MS){
      Serial.println ("idle in fallback AP mode, restarting to retry home WiFi");
      delay (100);
      ESP.restart ();
    }
  }
  delay (1);
}
