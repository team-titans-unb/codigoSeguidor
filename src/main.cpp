#include <Arduino.h>
#include <motors.h>
#include <QTRSensors.h>

#ifndef SPECS
#define SPECS

#define FREQ 50000
#define RES 8
#define BASE_VEL 120

#endif

// qtr sensor specs
#define SENSOR_QNT 8  
#define LEDON 19

// encoder pins
#define ENCODER_CHN_A_L 36
#define ENCODER_CHN_B_L 39
#define ENCODER_CHN_A_R 35
#define ENCODER_CHN_B_R 34

// PWM config
#define IN1_R 32 
#define IN1_L 33
#define CHN_R 0
#define CHN_L 1

// ======================= PROTOTYPES ======================= 
void calibrationRoutine();
int16_t pid(int16_t error);
void IRAM_ATTR readEncoder();

// ========================================================== 

QTRSensors qtr;
Motors motors(IN1_R, IN1_L, CHN_R, CHN_L, FREQ, RES);

uint16_t sensorValues[SENSOR_QNT];
int center = 2500;

bool followingLine = true;
bool slowMode = false; 

volatile long encoderCount = 0;

// PID
double Kp = 0.5;
double Ki = 0.0;
double Kd = 0.0;
double lastTime, dt;
double previousError = 0;
int16_t correction;

void setup() {
  motors.setSpeeds(0,0);
  pinMode(LEDON, OUTPUT);
  digitalWrite(LEDON, HIGH);

  // time variables inicialization
  dt = 0;
  lastTime = 0;

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){18, 5, 17, 16, 4, 0, 2, 15}, SENSOR_QNT);

  qtr.setEmitterPin(LEDON);

  Serial.begin(115200);

  // waits for 500 ms, then call calibration
  delay(500);
  calibrationRoutine();

  delay(500);
}

void loop() {

  // time difference (dt) calculation
  double now = micros();
  dt = (now - lastTime)/1000.00;

  if (dt > 0) { // Only run if time has passed
    lastTime = now;

    if (followingLine){
      uint16_t pos = qtr.readLineWhite(sensorValues);
      int16_t error = center - pos;
      correction = pid(error);

      int16_t vel_R = BASE_VEL + correction;
      int16_t vel_L = BASE_VEL - correction;
      vel_R = constrain(vel_R, 0, 255);
      vel_L = constrain(vel_L, 0, 255);
      motors.setSpeeds(vel_R, vel_L);
    }
  }
}

void calibrationRoutine(){
  // Swing robot to the right
  motors.setSpeeds(100, 0);
  
  for (uint16_t i = 0; i < 100; i++) { 
    qtr.calibrate();
  }
  motors.setSpeeds(0,0);

}

int16_t pid(int16_t error){
  int16_t proportionalError = 0;
  double integralError = 0;
  double derivativeError = 0;

  proportionalError = error;
  integralError += error * dt;
  derivativeError = (error - previousError) / dt;

  previousError = error;
  int16_t output = (int16_t) (Kp * proportionalError) + (Ki * integralError) + (Kd * derivativeError);

  return output;
}


// ISR
void IRAM_ATTR readEncoder(int encoder ){
}
