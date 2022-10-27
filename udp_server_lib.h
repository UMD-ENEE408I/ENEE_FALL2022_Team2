//UDP_SERVER_LIB

#ifndef UDP_SERVER_LIB
#define UDP_SERVER_LIB

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

//wifi event handler
void WiFiEvent(WiFiEvent_t event){
    switch(event) {
      case ARDUINO_EVENT_WIFI_STA_GOT_IP:
          //When connected set 
          Serial.print("WiFi connected! IP address: ");
          Serial.println(WiFi.localIP());  
          //initializes the UDP state
          //This initializes the transfer buffer
          udp.begin(WiFi.localIP(),udpPort);
          connected = true;
          break;
      case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
          Serial.println("WiFi lost connection");
          connected = false;
          break;
      default: break;
    }
}

//packet read function
void packet_read(dance_info *info)
{
  //check for packet
  int packetSize = udp.parsePacket();
  //Serial.print(packetSize);
  if(packetSize)
  {
    uint8_t* newinfo = (uint8_t*)info;
    udp.read(newinfo, sizeof(packetSize)); //length of the packet
  }
  //leave the function
  else
  {
    Serial.println("No Packet to Read");
  }
}

void connectToWiFi(const char * ssid, const char * pwd){
  Serial.println("Connecting to WiFi network: " + String(ssid));

  // delete old config
  WiFi.disconnect(true);
  //register event handler
  WiFi.onEvent(WiFiEvent);
  
  //Initiate connection
  WiFi.begin(ssid, pwd);

  Serial.println("Waiting for WIFI connection...");
}

#endif