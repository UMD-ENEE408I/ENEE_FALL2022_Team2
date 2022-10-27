//UDP_SERVER_LIB

#ifndef UDP_SERVER_LIB_H
#define UDP_SERVER_LIB_H
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <string.h>
#include <iostream>

//Packet Struct (to save)
  typedef struct dance_info
  {
    int8_t identity;
    int8_t heading;
  };

extern boolean connected = false;
extern WiFiUDP udp;
extern int udpPort;

void WiFiEvent(WiFiEvent_t event);
void packet_read(dance_info *info);
void connectToWiFi(const char * ssid, const char * pwd);

#endif