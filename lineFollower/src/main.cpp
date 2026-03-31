#include <Arduino.h>
#include <motors.h>
#include <QTRSensors.h>

#ifndef SPECS
#define SPECS

#define FREQ 50000
#define RES 8
#define BASE_VEL 170

#endif

#define SENSOR_QNT 8
#define LEDON 19

// PWM config
#define IN1_R 32
#define IN1_L 33
#define CHN_R 0
#define CHN_L 1

// ======================= PROTOTYPES ======================= 
void calibrationRoutine();
int16_t pid(int16_t error);

// ========================================================== 

QTRSensors qtr;
Motors motors(IN1_R, IN1_L, CHN_R, CHN_L, FREQ, RES);

uint16_t sensorValues[SENSOR_QNT];
bool calibrationFlag = false;
int center = 2500;

bool followingLine = false;
bool slowMode = false; 

// PID
double Kp = 0.04;
double Ki = 0.0;
double Kd = 1.02;
double lastTime, dt;
double previousError = 0;
int16_t correction;

void setup() {
  motors.setSpeeds(0,0);
  pinMode(LEDON, OUTPUT);
  digitalWrite(LEDON, HIGH);

  // constant inicialization
  Kp = 0.04;
  Ki = 0.0;
  Kd = 1.02;

  // time variables inicialization
  dt = 0;
  lastTime = 0;

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){18, 5, 17, 16, 4, 0, 2, 15}, SENSOR_QNT);

  qtr.setEmitterPin(LEDON);

  Serial.begin(115200);
  delay(100);

  calibrationRoutine();

  if (calibrationFlag = false){
    return;
  }
}

void loop() {

  // time difference calculation
  double now = millis();
  dt = (now - lastTime)/1000.00;
  lastTime = now;

  if (followingLine = true){
    uint16_t pos = qtr.readLineWhite(sensorValues);

    int16_t error = center - pos;
    correction = pid(error);

    int16_t vel_R = BASE_VEL + correction;
    int16_t vel_L = BASE_VEL - correction;
    vel_R = constrain(vel_R, 0, 255);
    vel_L = constrain(vel_L, 0, 255);
    motors.setSpeeds(vel_R, vel_L);
  }

  if (calibrationFlag = false){
    return;
  }
}

void calibrationRoutine(){
  for (uint16_t i = 0; i < 100; i++) { 
    qtr.calibrate(); 
    delay(25); 
  }

  for (uint16_t i = 0; i < 4; i++){
    motors.setSpeeds(70,70);
    motors.setSpeeds(-70, -70);
  }

  calibrationFlag = true;
  followingLine = true;
}

int16_t pid(int16_t error){
  int16_t output;

  int16_t proportionalError = error;
  double integralError =+ error * dt;
  double derivativeError = (error - previousError) / dt;
  previousError = error;
  int16_t output = (int16_t) (Kp * proportionalError) + (Ki * integralError) + (Kd * derivativeError);

  return output;
}