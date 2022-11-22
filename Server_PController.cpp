#include <WiFi.h>
#include <WiFiUdp.h>
#include <string.h>
#include <iostream>
#include <time.h>

// WiFi network name and password:
const char * networkName = "Samuel's Iphone";
const char * networkPswd = "testing123";
char pbuff[255];

//IP address to send UDP data to:
// either use the ip address of the server or 
// a network broadcast address
const char * udpAddress = "172.20.10.3";
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
  delay(5000);
  //Send a packet
  udp.beginPacket(udpAddress,udpPort);
  udp.printf("Hi Jetson");
  udp.endPacket();
  delay(4000);
  if(connected)
  {
    int packetSize = udp.parsePacket();
    if(packetSize)
    {
      float my_array[4]; // This declares an array of 4 floats which can typecast to an array of 16 char's
      udp.read((char*)my_array, sizeof(my_array)); // Typecast the pointer to my_array to a array of 16 char's
      Serial.println(my_array[0]); // interpret the array as a float again.
      Serial.println(my_array[1]);
      Serial.println(my_array[2]);
      Serial.println(my_array[3]);
    }
  } 
  delay(1000);
  udp.beginPacket(udpAddress,udpPort);
  udp.printf("Times Recieved");
  udp.endPacket();
  delay(3000);
}

void loop()
{
  if(connected)
  {
    udp.beginPacket(udpAddress,udpPort);
    udp.printf("ping");
    udp.endPacket();
    delay(100);

    int packetSize = udp.parsePacket();

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
      if(target > t1 + 60.00)
      {
        target = t1;
      }
    }
    float error = p_controller(target,t1,kp);
    if(error<=0)
    {
      error = 0;
    }
    float dt = (micros()- mlast)/1000000.0;
    Serial.printf("dt is %f err is %f\n", dt, error);
    t1 = t1 + (micros() - mlast)/1000000.0 + dt*error;
    Serial.printf("t1 is %f target is %f \n",t1,target);

  }
}
