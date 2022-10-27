#include <WiFi.h>
#include <WiFiUdp.h>
#include <string.h>
#include <iostream>

// WiFi network name and password:
const char * networkName = "408ITerps";
const char * networkPswd = "goterps2022";
char pbuff[255];

//IP address to send UDP data to:
// either use the ip address of the server or 
// a network broadcast address
const char * udpAddress = "192.168.2.139";
const int udpPort = 3333;

//Packet Struct (to save)
  typedef struct dance_info
  {
    int8_t identity;
    int8_t heading;
  };

//memory allocation for packet delivery
dance_info delivery;

//Are we currently connected?
boolean connected = false;

//The udp library class
WiFiUDP udp;

//*********************************************WIFI FUNCTIONS*****************************************************
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


void setup(){
  // Initilize hardware serial:
  Serial.begin(115200);
  
  // Stop the right motor by setting pin 14 low
  // this pin floats high or is pulled
  // high during the bootloader phase for some reason
  pinMode(14, OUTPUT);
  digitalWrite(14, LOW);
  delay(100);

  //Connect to the WiFi network
  connectToWiFi(networkName, networkPswd);
  delay(2000);

  //Send a packet
    udp.beginPacket(udpAddress,udpPort);
    udp.printf("Hi Jetson");
    udp.endPacket();
}

void loop()
{
  char buffer[255];
  int count;
  if(connected)
  {
    //Recieve a command
    packet_read(&delivery);
    sprintf(buffer, "ID = %d, Heading = %d", delivery.identity, delivery.heading);
    Serial.println(buffer);

    //Switch Statement
    switch(delivery.identity)
    {
      case 1 :
        Serial.println("ID #1 Selected"); 
        break;
        
      default:
        Serial.println("None");
        break;
    }

  //Wait for 1 second
  delay(1000);
  } 
}

