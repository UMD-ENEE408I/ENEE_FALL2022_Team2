#include <WiFi.h>
#include <WiFiUdp.h>
#include <string.h>
#include <iostream>
#include <time.h>
#include <algorithm>

// WiFi network name and password:
const char * networkName = "408ITerps";
const char * networkPswd = "goterps2022";
char pbuff[255];

//IP address to send UDP data to:
// either use the ip address of the server or 
// a network broadcast address
const char * udpAddress = "192.168.2.139";
const int udpPort = 3333;

//Are we currently connected?
boolean connected = false;

//The udp library class
WiFiUDP udp;

float t1 = 0;
int kp = 3;
float mlast = micros();
float error = 0;
float target;
int packetSize = 0;
float slow;
float dance_info[6] = {};
  
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

float p_controller(float target, float current, int kp) {
  // Calculate new time and drive it to target
  float e = target - current; // Error
  float u = kp * e; //
  return u;
}


void setup(){
  // Initilize hardware serial:
  Serial.begin(115200);
  pinMode(14, OUTPUT);
  digitalWrite(14, LOW);
  delay(100);
  //Connect to the wifi
  connectToWiFi(networkName, networkPswd);
  delay(6000);
  //Send a packet
  udp.beginPacket(udpAddress,udpPort);
  udp.printf("Ready to Start!");
  udp.endPacket();
  boolean check = false;
  while(!check){
    packetSize = udp.parsePacket();
    udp.read((char*)dance_info, sizeof(dance_info)); // Typecast the pointer to my_array to a array of 16 char's
    check = true;
    for(int i = 0; i < 6; i++)
    {
      if(dance_info[i] == 0)
      {
        check = false;
        break;
      }
    }
  }
  Serial.println(dance_info[0]); // interpret the array as a float again.
  Serial.println(dance_info[1]);
  Serial.println(dance_info[2]);
  Serial.println(dance_info[3]);
  Serial.println(dance_info[4]);
  Serial.println(dance_info[5]);
  delay(2000);
  udp.beginPacket(udpAddress,udpPort);
  udp.printf("Times Recieved!");
  udp.endPacket();
  delay(2000);
  Serial.println("Starting in 3");
  delay(1000);
  Serial.println("Starting in 2");
  delay(1000);
  Serial.println("Starting in 1");
  delay(1000);
}

void loop()
{
  if(connected)
  {
    float mlast2 = micros();
    udp.beginPacket(udpAddress,udpPort);
    udp.printf("ping");
    udp.endPacket();
    delay(100);

    //Checks for packet and obtains its contents
    packetSize = udp.parsePacket();
    if(packetSize >= sizeof(float))
    {
      mlast = micros();
      Serial.printf("packet size is %d\n", packetSize);
      float my_array[1]; 
      udp.read((char*)my_array, sizeof(my_array)); 
      udp.flush();
      Serial.printf("received value is %f\n", my_array[0]);
      target = my_array[0];
      Serial.printf("target is %f\n", target);
    }
    else{
      target = target + (micros()-mlast2)/1000000.0;
    }
    if(target >= t1 + 10 || target <= t1 - 5){
      target = t1;
      error = 0;
    }
    error = max(float(-1),p_controller(target,t1,kp));
    Serial.printf("err is %f\n", error);
    float dt = (micros()- mlast2)/1000000.0;
    t1 = t1 + (micros() - mlast2)/1000000.0 + dt*error;
    Serial.printf("t1 is %f target is %f \n",t1,target);
  }
}
