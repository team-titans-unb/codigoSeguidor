#include <Arduino.h>
#include <motors.h>
#include <QTRSensors.h>

#ifndef SPECS
#define SPECS

#define FREQ 50000
#define RES 8
#define BASE_VEL 150

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
int center = 3500;

bool segueLinha = true;
bool slowMode = false; 

volatile long encoderCount = 0;

// PID
double Kp = 0.08;
double Ki = 0.0;
double Kd = 0.8;

int erroAnterior = 0;
long erroIntegral = 0;

int16_t correcao;

void setup() {
  motors.setSpeeds(0,0);
  pinMode(LEDON, OUTPUT);
  digitalWrite(LEDON, HIGH);

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){18, 5, 17, 16, 4, 0, 2, 15}, SENSOR_QNT);

  qtr.setEmitterPin(LEDON);

  Serial.begin(115200);

  // Blink LED while waiting, then stay ON when calibration starts
  for (int i = 0; i < 6; i++) {
    digitalWrite(LEDON, LOW);
    delay(250);
    digitalWrite(LEDON, HIGH);
    delay(250);
  }
  calibrationRoutine();

  for (int i = 0; i < SENSOR_QNT; i++) {
    Serial.print("S"); Serial.print(i);
    Serial.print(" min:"); Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(" max:"); Serial.println(qtr.calibrationOn.maximum[i]);
  }

  delay(500);
}

void loop() {
	int pos = qtr.readLineWhite(sensorValues);
	int erro = pos - center;

	int correcao = pid(erro);

	int Lvel = BASE_VEL - correcao;
	int Rvel = BASE_VEL + correcao;

	Lvel = constrain(Lvel, 0, 255);
	Rvel = constrain(Rvel, 0, 255);

	motors.setSpeeds(Lvel, Rvel);

  Serial.println(pos);
  Serial.print(" | erro: "); Serial.print(erro);
  Serial.print(" | cor: ");  Serial.println(correcao);
}

void calibrationRoutine(){
	for (uint16_t i = 0; i < 80; i++){
		qtr.calibrate();
    delay(5);
	}
}

int16_t pid(int16_t error){

  erroIntegral += error;
  erroIntegral = constrain(erroIntegral, -10000, 10000);
  
  int erroDerivada = error - erroAnterior;

  int output = (int) (Kp * error) + (Ki * erroIntegral) + (Kd * erroDerivada);
  erroAnterior = error;

  return output;
}


// ISR
void IRAM_ATTR readEncoder(int encoder ){
}
