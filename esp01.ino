#include <ESP8266WiFi.h>

#define SCAN_PERIOD 5000
#define BAUD_RATE 115200

long lastScanMillis;

void setup() {
  Serial.begin(BAUD_RATE);

  /*
   * Using WiFi.mode over wifi_set_opmode to delegate all logic to the library
   * https://github.com/esp8266/Arduino/blob/b0ece8cac42907b14794925ff6d256a0820ee4e9/libraries/ESP8266WiFi/src/ESP8266WiFiGeneric.cpp#L404
   */
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
}

void loop() {
  long currentMillis = millis();

  if (currentMillis - lastScanMillis > SCAN_PERIOD)
  {
    WiFi.scanNetworks(false, true);
    lastScanMillis = currentMillis;
  }

  int n = WiFi.scanComplete();
  if(n >= 0) {
    for (int i = 0; i < n; i++) {
      /*
       * There are other fields that could be of interest such as BSSID or isHidden
       */
      Serial.printf("%s, Ch:%d (%ddBm) %s\n", WiFi.SSID(i).c_str(), WiFi.channel(i), WiFi.RSSI(i), WiFi.encryptionType(i));
    }
    WiFi.scanDelete();
  }
}
