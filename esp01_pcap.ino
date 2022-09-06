#include <ESP8266WiFi.h>
#include <TimeLib.h>
#include <PCAP.h>

extern "C" {
  #include "user_interface.h"
}

#define INIT_CHANNEL 1
#define BAUD_RATE 115200
#define MAX_CHANNEL 11
#define HOP_TIME 250


PCAP pcap = PCAP();

int current_channel = INIT_CHANNEL;
unsigned long last_hop = 0;

// Process each packet
void sniffer(uint8_t *buf, uint16_t len) {
  /* 
   *  Send timestamp in seconds before packet data according to PCAP specification
   *  https://wiki.wireshark.org/Development/LibpcapFileFormat#record-packet-headerhtml
   */
  uint32_t timestamp = now();
  uint32_t microseconds = (unsigned int)(micros() - millis() * 1000);
  pcap.newPacketSerial(timestamp, microseconds, len, buf);
}


void setup() {
  
  Serial.begin(BAUD_RATE);
  // Wait for hardware initialization
  delay(10000);
  pcap.startSerial();
  
  /* setup wifi */
  wifi_set_opmode(STATION_MODE);
  wifi_promiscuous_enable(0);
  WiFi.disconnect();
  wifi_set_promiscuous_rx_cb(sniffer);
  wifi_set_channel(current_channel);
  wifi_promiscuous_enable(1);
}

void loop() {
  unsigned long current_time = millis();
  
  if(current_time - last_hop >= HOP_TIME){
    last_hop = current_time;
    current_channel++;
    if(current_channel > MAX_CHANNEL) {
      current_channel = 1;
    }
    wifi_set_channel(current_channel);
  }
}