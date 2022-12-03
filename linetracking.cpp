#include <Arduino.h>
#include <Adafruit_MCP3008.h>
#include <Adafruit_MPU6050.h>
#include <Encoder.h>
#include <tuple>
#include <stdlib.h> 
#include <udp_server_lib.h>
#include <algorithm>
#include <cstdlib>

// WiFi network name and password:
const char * networkName = "408ITerps";
const char * networkPswd = "goterps2022";
char pbuff[255];

//IP address to send UDP data to:
// either use the ip address of the server or 
// a network broadcast address
const char * udpAddress = "192.168.2.139";
//const int udpPort = 3333;

//memory allocation for packet delivery
dance_info delivery;

// IMU (rotation rate and acceleration)
Adafruit_MPU6050 mpu;

// Buzzer pin which we will use for indicating IMU initialization failure
const unsigned int BUZZ = 26;
const unsigned int BUZZ_CHANNEL = 0;

// Need these pins to turn off light bar ADC chips
const unsigned int ADC_1_CS = 2;
const unsigned int ADC_2_CS = 17;

// Battery voltage measurement constants
const unsigned int VCC_SENSE = 27;
const float ADC_COUNTS_TO_VOLTS = (2.4 + 1.0) / 1.0 * 3.3 / 4095.0;

// Motor encoder pins
const unsigned int M1_ENC_A = 39;
const unsigned int M1_ENC_B = 38;
const unsigned int M2_ENC_A = 37;
const unsigned int M2_ENC_B = 36;

// Motor power pins
const unsigned int M1_IN_1 = 13;
const unsigned int M1_IN_2 = 12;
const unsigned int M2_IN_1 = 25;
const unsigned int M2_IN_2 = 14;

// Motor PWM channels
const unsigned int M1_IN_1_CHANNEL = 8;
const unsigned int M1_IN_2_CHANNEL = 9;
const unsigned int M2_IN_1_CHANNEL = 10;
const unsigned int M2_IN_2_CHANNEL = 11;

const int M_PWM_FREQ = 5000;
const int M_PWM_BITS = 8;
const unsigned int MAX_PWM_VALUE = 255; // Max PWM given 8 bit resolution

float METERS_PER_TICK = (3.14159 * 0.031) / 360.0;
float TURNING_RADIUS_METERS = 4.3 / 100.0; // Wheels are about 4.3 cm from pivot point

int kp = 3;
const int numTimes = 6;
//float times[numTimes] = {};
// float* times;

void configure_imu() {
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      ledcWriteNote(BUZZ_CHANNEL, NOTE_C, 4);
      delay(500);
      ledcWriteNote(BUZZ_CHANNEL, NOTE_G, 4);
      delay(500);
    }
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_1000_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
}

void read_imu(float& w_z) {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  w_z = g.gyro.z;
}

void est_imu_bias(float& E_w_z, int N_samples) {
  float E_w_z_acc = 0.0;
  for (unsigned int i = 0; i < N_samples; i++) {
    float w_z;
    read_imu(w_z);
    E_w_z_acc += w_z;
    delay(5);
  }
  E_w_z = E_w_z_acc / N_samples;
}

void configure_motor_pins() {
  ledcSetup(M1_IN_1_CHANNEL, M_PWM_FREQ, M_PWM_BITS);
  ledcSetup(M1_IN_2_CHANNEL, M_PWM_FREQ, M_PWM_BITS);
  ledcSetup(M2_IN_1_CHANNEL, M_PWM_FREQ, M_PWM_BITS);
  ledcSetup(M2_IN_2_CHANNEL, M_PWM_FREQ, M_PWM_BITS);

  ledcAttachPin(M1_IN_1, M1_IN_1_CHANNEL);
  ledcAttachPin(M1_IN_2, M1_IN_2_CHANNEL);
  ledcAttachPin(M2_IN_1, M2_IN_1_CHANNEL);
  ledcAttachPin(M2_IN_2, M2_IN_2_CHANNEL);
}

// Positive means forward, negative means backwards
void set_motors_pwm(float left_pwm, float right_pwm) {
  if (isnan(left_pwm)) left_pwm = 0.0;
  if (left_pwm  >  255.0) left_pwm  =  255.0;
  if (left_pwm  < -255.0) left_pwm  = -255.0;
  if (isnan(right_pwm)) right_pwm = 0.0;
  if (right_pwm >  255.0) right_pwm =  255.0;
  if (right_pwm < -255.0) right_pwm = -255.0;

  if (left_pwm > 0) {
    ledcWrite(M1_IN_1_CHANNEL, 0);
    ledcWrite(M1_IN_2_CHANNEL, (uint32_t)(left_pwm));
  } else {
    ledcWrite(M1_IN_1_CHANNEL, (uint32_t)-left_pwm);
    ledcWrite(M1_IN_2_CHANNEL, 0);
  }

  if (right_pwm > 0) {
    ledcWrite(M2_IN_1_CHANNEL, 0);
    ledcWrite(M2_IN_2_CHANNEL, (uint32_t)(right_pwm));
  } else {
    ledcWrite(M2_IN_1_CHANNEL, (uint32_t)-right_pwm);
    ledcWrite(M2_IN_2_CHANNEL, 0);
  }
}

float update_pid(float dt, float kp, float ki, float kd,
                 float x_d, float x,
                 float& int_e, float abs_int_e_max, // last_x and int_e are updated by this function
                 float& last_x) {
  // Calculate or update intermediates
  float e = x_d - x; // Error

  // Integrate error with anti-windup
  int_e = int_e + e * dt;
  if (int_e >  abs_int_e_max) int_e =  abs_int_e_max;
  if (int_e < -abs_int_e_max) int_e = -abs_int_e_max;

  // Take the "Derivative of the process variable" to avoid derivative spikes if setpoint makes step change
  // with abuse of notation, call this de
  float de = -(x - last_x) / dt;
  last_x = x;

  float u = kp * e + ki * int_e + kd * de;
  return u;
}

float p_controller(float target, float current, int kp) {
  // Calculate new time and drive it to target
  float e = target - current; // Error
  float u = kp * e; //
  return u;
}

// a smooth and interesting trajectory
// https://en.wikipedia.org/wiki/Lemniscate_of_Bernoulli
void leminscate_of_bernoulli(float t, float a, float& x, float& y) {
  float sin_t = sin(t);
  float den = 1 + sin_t * sin_t;
  x = a * cos(t) / den;
  y = a * sin(t) * cos(t) / den;
}

std::tuple<float,float> leminscate_of_bernoulli2(float t, float a, float f) {
  float sin_t = sin(2*M_PI*f*t);
  float den = 1 + sin_t * sin_t;
  float x = a * cos(2*M_PI*f*t) / den;
  float y = a * sin(2*M_PI*f*t) * cos(2*M_PI*f*t) / den;
  return std::make_tuple(x,y);
}
std::tuple<float,float> circle(float t, float a, float f) {
  float sin_t = sin(2*M_PI*f*t);
  float den = 1 + sin_t * sin_t;
  float x = a * cos(2*M_PI*f*t);
  float y = a * sin(2*M_PI*f*t);

  return std::make_tuple(x,y);
}

std::tuple<float,float> rose(float t, float a, float f){
  float x = a*cos(2*M_PI*f*t/2)*cos(2*M_PI*f*t);
  float y = a*cos(2*M_PI*f*t/2)*sin(2*M_PI*f*t);
  return std::make_tuple(x,y);
}

std::tuple<float,float> quadr(float t, float a, float f){
  float x = 2*a*pow(sin(2*M_PI*f*t),2)*cos(2*M_PI*f*t);
  float y = 2*a*pow(cos(2*M_PI*f*t),2)*sin(2*M_PI*f*t);
  return std::make_tuple(x,y);
}

std::tuple<float,float> spiral_in(float t, float a, float f) {
  float sin_t = sin(2*M_PI*f*t);
  float den = 1 + sin_t * sin_t;
  float x = a * cos(2*M_PI*f*t)*exp(-0.2*2*M_PI*f*t);
  float y = a * sin(2*M_PI*f*t)*exp(-0.2*2*M_PI*f*t);
  return std::make_tuple(x,y);
}

typedef std::tuple<float,float> (*fptr)(float t, float a, float f);

fptr pick_traj(int n){
  if(n == 0){
    return &leminscate_of_bernoulli2;
  }
  else if(n==1){
    return &circle;
  }
  else if(n==2){
    return &rose;
  }
  else{
    return &quadr;
  }
}

void spiral_out(float t, float a, float& x, float& y) {
  float sin_t = sin(t);
  float den = 1 + sin_t * sin_t;
  x = a * cos(t)*exp(0.2*t);
  y = a * sin(t)*exp(0.2*t);
}

std::tuple<float, float> interp(float t, float a, float f, float alpha, std::tuple<float, float> (*func1)(float, float, float),std::tuple<float, float> (*func2)(float, float, float)){
  std::tuple<float, float> tup1 = func1(t,a,f);
  float x1 = std::get<0>(tup1);
  float y1 = std::get<1>(tup1);

  std::tuple<float, float> tup2 = func2(t,a,f);
  float x2 = std::get<0>(tup2);
  float y2 = std::get<1>(tup2);

  float x = alpha*x1 + (1-alpha)*x2;
  float y = alpha*y1 + (1-alpha)*y2;

  return std::make_tuple(x,y);

}

// Signed angle from (x0, y0) to (x1, y1)
// assumes norms of these quantities are precomputed
float signed_angle(float x0, float y0, float n0, float x1, float y1, float n1) {
  float normed_dot = (x1 * x0 + y1 * y0) / (n1 * n0);
  if (normed_dot > 1.0) normed_dot = 1.0; // Possible because of numerical error
  float angle = acosf(normed_dot);

  // use cross product to find direction of rotation
  // https://en.wikipedia.org/wiki/Cross_product#Coordinate_notation
  float s3 = x0 * y1 - x1 * y0;
  if (s3 < 0) angle = -angle;

  return angle;
}

void setup() {
  Serial.begin(115200);
  
  //Serial.printf("times is %d\n", times);

  // Disalbe the lightbar ADC chips so they don't hold the SPI bus used by the IMU
  pinMode(ADC_1_CS, OUTPUT);
  pinMode(ADC_2_CS, OUTPUT);
  digitalWrite(ADC_1_CS, HIGH);
  digitalWrite(ADC_2_CS, HIGH);

  ledcAttachPin(BUZZ, BUZZ_CHANNEL);

  pinMode(VCC_SENSE, INPUT);

  configure_motor_pins();
  configure_imu();

    //Connect to the WiFi network
  connectToWiFi(networkName, networkPswd);
  delay(6000);

  //Send a packet
  udp.beginPacket(udpAddress,udpPort);
  udp.printf("Hi Jetson");
  udp.endPacket();

  // if(connected)
  // {
    
  //   int packetSize = 0;
  //   // while(!packetSize){
  //   //   packetSize = udp.parsePacket();
      
  //   // }
  //   delay(1000);
  //   // int numTimes = 0;
  //   // while(!numTimes){
  //   //   udp.read((char*)numTimes, sizeof(numTimes));
  //   // }
  //   // udp.beginPacket(udpAddress,udpPort);
  //   // udp.printf("Num Times Recieved");
  //   // udp.endPacket();
  //   // delay(1000);
  //   float times2[numTimes] = {};
  //   boolean readTimes = false;
  //   while(!readTimes){
  //     packetSize = udp.parsePacket();
  //     udp.read((char*)times2, sizeof(times2));
  //     readTimes = true;
  //     for(int i = 0; i < numTimes; i++){
  //       if(times2[i] == 0.0){
  //         //Serial.println("HERE5");
  //         readTimes = false;
  //         break;
  //       }
  //     }

  //     //Serial.println("HERE3");
  //   }

  //   for(int i = 0; i < numTimes; i++){
  //     times[i] = times2[i];
  //     Serial.printf("times[i] is %f\n", times[i]);
  //   }
  //   Serial.println("HERE4");
  //   udp.beginPacket(udpAddress,udpPort);
  //   udp.printf("Times Recieved");
  //   udp.endPacket();
    
  // } 

  delay(1000);
  Serial.println("Starting!");
}

void loop() {
  float times[numTimes] = {};
  //times = (float*) malloc(6*sizeof(float));
  if(connected)
  {
    
    int packetSize = 0;
    // while(!packetSize){
    //   packetSize = udp.parsePacket();
      
    // }
    delay(1000);
    // int numTimes = 0;
    // while(!numTimes){
    //   udp.read((char*)numTimes, sizeof(numTimes));
    // }
    // udp.beginPacket(udpAddress,udpPort);
    // udp.printf("Num Times Recieved");
    // udp.endPacket();
    // delay(1000);
    float times2[numTimes] = {};
    boolean readTimes = false;
    while(!readTimes){
      packetSize = udp.parsePacket();
      udp.read((char*)times2, sizeof(times2));
      readTimes = true;
      for(int i = 0; i < numTimes; i++){
        if(times2[i] == 0.0){
          //Serial.println("HERE5");
          readTimes = false;
          break;
        }
      }

      //Serial.println("HERE3");
    }

    for(int i = 0; i < numTimes; i++){
      times[i] = times2[i];
      Serial.printf("times[i] is %f\n", times[i]);
    }
    Serial.println("HERE4");
    udp.beginPacket(udpAddress,udpPort);
    udp.printf("Times Recieved");
    udp.endPacket();
    
  } 
  // Create the encoder objects after the motor has
  // stopped, else some sort exception is triggered
  int count_traj = 0;

  Encoder enc1(M1_ENC_A, M1_ENC_B);
  Encoder enc2(M2_ENC_A, M2_ENC_B);

  // Loop period
  int target_period_ms = 2; // Loop takes about 3 ms so a delay of 2 gives 200 Hz or 5ms

  // States used to calculate target velocity and heading
  float leminscate_a = 0.5; // Radius
  float leminscate_t_scale = 2.0; // speedup factor
  float x0, y0;
  leminscate_of_bernoulli(0.0, leminscate_a, x0, y0);
  float last_x, last_y;
  leminscate_of_bernoulli(-leminscate_t_scale * target_period_ms / 1000.0, leminscate_a, last_x, last_y);
  float last_dx = (x0 - last_x) / ((float)target_period_ms / 1000.0);
  float last_dy = (y0 - last_y) / ((float)target_period_ms / 1000.0);
  float last_target_v = sqrtf(last_dx * last_dx + last_dy * last_dy);
  float target_theta = 0.0; // This is an integrated quantity

  // Motors are controlled by a position PID
  // with inputs interpreted in meters and outputs interpreted in volts
  // integral term has "anti-windup"
  // derivative term uses to derivative of process variable (wheel position)
  // instead of derivative of error in order to avoid "derivative kick"
  float kp_left = 200.0;
  float ki_left = 20.0;
  float kd_left = 20.0;
  float kf_left = 10.0;
  float target_pos_left  = 0.0;
  float last_pos_left = 0.0;
  float integral_error_pos_left = 0.0;
  float max_integral_error_pos_left = 1.0 * 8.0 / ki_left; // Max effect is the nominal battery voltage

  float kp_right = 200.0;
  float ki_right = 20.0;
  float kd_right = 20.0;
  float kf_right = 10.0;
  float last_pos_right = 0.0;
  float target_pos_right = 0.0;
  float integral_error_pos_right = 0.0;
  float max_integral_error_pos_right = 1.0 * 8.0 / ki_right; // Max effect is the nominal battery voltage

  // IMU Orientation variables
  float theta = 0.0;
  float bias_omega;
  // Gain applied to heading error when offseting target motor velocities
  // currently set to 360 deg/s compensation for 90 degrees of error
  float ktheta = (2 * 3.14159) / (90.0 * 3.14159 / 180.0);
  est_imu_bias(bias_omega, 500);// Could be expanded for more quantities

  // The real "loop()"
  // time starts from 0
  float start_t = (float)micros() / 1000000.0;
  float last_t = -target_period_ms / 1000.0; // Offset by expected looptime to avoid divide by zero
  float alpha = 1.0;
  int n = 0;
  int traj = 0;
  int traj_prev = 0;
  bool change_traj = false;
  fptr func_prev = pick_traj(0);
  bool interp_traj = false;
  fptr func = pick_traj(0);
  float t_prev = 0.0;
  while (true) {
    float t = ((float)micros()) / 1000000.0 - start_t;
    float mlast2;
    if(connected){
      mlast2 = micros();
      udp.beginPacket(udpAddress,udpPort);
      udp.printf("ping");
      udp.endPacket();
      delay(100);
    }

    int packetSize = udp.parsePacket();
    float target;
    float error;
    if(packetSize >= sizeof(float)){
      float mlast = micros();
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
    if(target >= t + 10){
      target = t;
      error = 0;
    }
    error = max(float(-1),p_controller(target,t,kp));
    //Serial.printf("err is %f\n", error);
    float dt2 = (micros()- mlast2)/1000000.0;
    t = t + (micros() - mlast2)/1000000.0 + dt2*error;
    Serial.printf("t1 is %f target is %f \n",t,target);

    // udp.beginPacket(udpAddress,udpPort);
    // udp.printf("Hi Jetson");
    // udp.endPacket();
    // char buffer[255];
    // int count;
    // packet_read(&delivery);
    // sprintf(buffer, "ID = %d, Heading = %d", delivery.identity, delivery.heading);
    // Serial.println(buffer);

    // Get the time elapsed
    
    float dt = ((float)(t - last_t)); // Calculate time since last update
    // Serial.print("t "); Serial.print(t);
    //Serial.print(" dt "); Serial.print(dt * 1000.0);
    last_t = t;

    // Get the distances the wheels have traveled in meters
    // positive is forward
    float pos_left  =  (float)enc1.read() * METERS_PER_TICK;
    float pos_right = -(float)enc2.read() * METERS_PER_TICK; // Take negative because right counts upwards when rotating backwards
  
    // TODO Battery voltage compensation, the voltage sense on my mouse is broken for some reason
    // int counts = analogRead(VCC_SENSE);
    // float battery_voltage = counts * ADC_COUNTS_TO_VOLTS;
    // if (battery_voltage <= 0) Serial.println("BATTERY INVALID");
  
    // Read IMU and update estimate of heading
    // positive is counter clockwise
    float omega;
    read_imu(omega); // Could be expanded to read more things
    omega -= bias_omega; // Remove the constant bias measured in the beginning
    theta = theta + omega * dt;
    // Serial.print(" omega "); Serial.print(omega);
    // Serial.print(" theta "); Serial.print(theta);

    // Serial.print(" last_x "); Serial.print(last_x);
    // Serial.print(" last_y "); Serial.print(last_y);
    // Serial.print(" last_dx "); Serial.print(last_dx);
    // Serial.print(" last_dy "); Serial.print(last_dy);
    // Serial.print(" last tv "); Serial.print(last_target_v);

    // Calculate target forward velocity and target heading to track the leminscate trajectory
    // of 0.5 meter radius
    float x, y;
  
    //leminscate_of_bernoulli(leminscate_t_scale * t, leminscate_a, x, y);
    //circle(t,leminscate_a,x,y);
    // if(t < 5){
    //   std::tuple<float, float> tup = spiral_in(t,leminscate_a);
    //   x = std::get<0>(tup);
    //   y = std::get<1>(tup);
    // }
    // else{
    //   if(alpha > 0){
    //     std::tuple<float, float> tup = interp(t,leminscate_a,alpha,&spiral_in,&circle);
    //     x = std::get<0>(tup);
    //     y = std::get<1>(tup);
    //     alpha = alpha - 0.001;
    //   }
    //   else{
    //     std::tuple<float, float> tup = circle(t,leminscate_a);
    //     x = std::get<0>(tup);
    //     y = std::get<1>(tup);
    //   }
    // }

    leminscate_a = 0.5;
    float freq = times[5];
    Serial.printf("lem a is %f\n",leminscate_a);
    if(true){
      //Serial.printf("current count is %f\n",times[count_traj]);
      //Serial.printf("alpha is %f", alpha);
      if (interp_traj && alpha > 0){
        std::tuple<float, float> tup = interp(t,leminscate_a,alpha,freq,func_prev,func);
        x = std::get<0>(tup);
        y = std::get<1>(tup);
        Serial.printf("x is %f, y is %f\n", x, y);
        alpha = alpha - 0.05;
        t_prev = t;
      }
      else{
        std::tuple<float, float> tup = func(t,leminscate_a,freq);
        x = std::get<0>(tup);
        y = std::get<1>(tup);
        Serial.printf("x is %f, y is %f\n", x, y);
        interp_traj = false;
        if (t > times[count_traj] & count_traj < numTimes - 1){
          change_traj = true;
          count_traj += 1;
          Serial.println("Changing trajectory!");
        }
        
      }
      if(change_traj){
        traj_prev = traj;
        traj = rand() % 4;
        traj = traj + 1;
        //traj = delivery.identity;

        if(traj != traj_prev){
          change_traj = false;
          func_prev = func;
          func = pick_traj(traj);
          interp_traj = true;
          alpha = 1.0;
        }
        t_prev = t;
      }


    }

    // Serial.print(" x "); Serial.print(x);
    // Serial.print(" y "); Serial.print(y);
    Serial.printf("x is %f, y is %f\n", x, y);
    float dx = (x - last_x) / dt;
    float dy = (y - last_y) / dt;
    float target_v = sqrtf(dx * dx + dy * dy); // forward velocity

    // Serial.print(" dx "); Serial.print(dx);
    // Serial.print(" dy "); Serial.print(dy);
    // Serial.print(" tv "); Serial.print(target_v);

    // Compute the change in heading using the normalized dot product between the current and last velocity vector
    // using this method instead of atan2 allows easy smooth handling of angles outsides of -pi / pi at the cost of
    // a slow drift defined by numerical precision
    float target_omega = signed_angle(last_dx, last_dy, last_target_v, dx, dy, target_v) / dt;
    target_theta = target_theta + target_omega * dt;

    // Serial.print(" target_omega "); Serial.print(target_omega);
    // Serial.print(" t theta "); Serial.print(target_theta);

    last_x = x;
    last_y = y;
    last_dx = dx;
    last_dy = dy;
    last_target_v = target_v;
  
    // Calculate target motor speeds from target forward speed and target heading
    // Could also include target path length traveled and target angular velocity
    float error_theta_z = target_theta - theta;
    float requested_v = target_v;
    float requested_w = ktheta * error_theta_z;

    float target_v_left  = requested_v - TURNING_RADIUS_METERS * requested_w;
    float target_v_right = requested_v + TURNING_RADIUS_METERS * requested_w;
    target_pos_left  = target_pos_left  + dt * target_v_left;
    target_pos_right = target_pos_right + dt * target_v_right;

    // Serial.print(" tpl "); Serial.print(target_pos_left);
    // Serial.print(" pl "); Serial.print(pos_left);
    // Serial.print(" tpr "); Serial.print(target_pos_right);
    // Serial.print(" pr "); Serial.print(pos_right);

    // Left motor position PID
    float left_voltage = update_pid(dt, kp_left, ki_left, kd_left,
                                    target_pos_left, pos_left,
                                    integral_error_pos_left, max_integral_error_pos_left,
                                    last_pos_left);
    left_voltage = left_voltage + kf_left * target_v_left;
    float left_pwm = (float)MAX_PWM_VALUE * (left_voltage / 8.0); // TODO use actual battery voltage

    // Right motor position PID
    float right_voltage = update_pid(dt, kp_right, ki_right, kd_right,
                                     target_pos_right, pos_right,
                                     integral_error_pos_right, max_integral_error_pos_right,
                                     last_pos_right);
    left_voltage = right_voltage + kf_right * target_v_right;
    float right_pwm = (float)MAX_PWM_VALUE * (right_voltage / 8.0); // TODO use actual battery voltage

    // Serial.print(" l voltage " ); Serial.print(left_voltage);
    // Serial.print(" r voltage " ); Serial.print(right_voltage);

    set_motors_pwm(left_pwm, right_pwm);

    // Serial.println();
    delay(target_period_ms);
    //delay(1000);
  }
}