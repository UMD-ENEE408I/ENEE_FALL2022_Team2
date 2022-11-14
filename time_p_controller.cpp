#include <Arduino.h>
#include <time.h>

float p_controller(float target, float current, int kp) {
  // Calculate new time and drive it to target
  float e = target - current; // Error
  float u = kp * e; //
  return u;
}

void setup(){
	Serial.begin(115200);
}

void loop(){
  float start_t2 = 10;
  float t1 = 2;
  int k = 3;
  int i;
  delay(1000);
  float mlast = micros();
	while(true){
    float dt = (micros()- mlast)/1000000.0;
    float t2 = ((float)micros()) / 1000000.0 + start_t2;
    float error = p_controller(t2,t1,k);
    t1 = t1 + (micros() - mlast)/1000000.0 + dt*error;
    mlast = micros();
    Serial.printf("%f %f \n",t1,t2);
    }
}
