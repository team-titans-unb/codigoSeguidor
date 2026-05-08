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

// Parte dos encoders
volatile long encoderCount = 0;

volatile long encoderLeft = 0;
volatile long encoderRight = 0;

// Leitura de velocidade 
unsigned long check_velocidade = 0;
const int intervalo = 50;
long lastEncoderLeft = 0;
long lastEncoderRight = 0;

float velocidadeL = 0;
float velocidadeR = 0; // unidade em m/s

const float diametro = 0.021;
const float PPR = 210.0;
const float pilido = 3.1415;

// Controle de malha fechada a partir do encoder
float ajusteL = 0;
float ajusteR = 0;
float kp_vel = 0.1;

void IRAM_ATTR readEncoderL() {
  if (digitalRead(ENCODER_CHN_B_L == LOW))
    encoderLeft++;
  else 
    encoderLeft--;
}
void IRAM_ATTR readEncoderR() {
  if (digitalRead(ENCODER_CHN_B_R == LOW)) // Se o canal CS1(A) sobe e o CS2(B) desce, vai para frente
    encoderRight++;
  else 
    encoderRight--;
}  // função de interrupção que conta os pulsos do encoder

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

  // Configuração dos pinos de encoders
  pinMode(ENCODER_CHN_A_L, INPUT);
  pinMode(ENCODER_CHN_A_R, INPUT);
  pinMode(ENCODER_CHN_B_L,INPUT);
  pinMode(ENCODER_CHN_B_R,INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_CHN_A_L), readEncoderL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_CHN_A_R), readEncoderR, RISING); // Ativa as interrupções


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

  unsigned currentTime = millis();

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

  // PID DE LINHA COM OS ENCODERS
  if(currentTime - check_velocidade >= intervalo){
    long delta_left = encoderLeft - lastEncoderLeft;
    long delta_right = encoderRight - lastEncoderRight;

    velocidadeL = (delta_left*1000.0)/intervalo;
    velocidadeR = (delta_right*1000.0)/intervalo;

    float velocidadeL_ms = velocidadeL * (pilido*diametro/PPR);
    float velocidadeR_ms = velocidadeR * (pilido*diametro/PPR); // Converte de pulsos/seg para m/seg

    float alvovel = 0.5;

    float epslon_velL = alvovel - velocidadeL_ms;
    float epslon_velR = alvovel - velocidadeR_ms;

    ajusteL += epslon_velL * kp_vel;
    ajusteR += epslon_velR * kp_vel;

    int16_t pwmFinalL = (BASE_VEL - correction) + ajusteL;
    int16_t pwmFinalR = (BASE_VEL + correction) + ajusteR;

    motors.setSpeeds(constrain(pwmFinalR, 0, 255), constrain(pwmFinalL, 0, 255)); // faz o ajuste de velocidade para o valor desejado

    lastEncoderLeft = encoderLeft;
    lastEncoderRight = encoderRight;
    check_velocidade = currentTime;

    Serial.print("roda esq: "); Serial.print(velocidadeL_ms);
    Serial.print("roda dir: "); Serial.print(velocidadeR_ms); // o valor pode ser visto na porta serial

    
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