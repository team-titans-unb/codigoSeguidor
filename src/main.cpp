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

// Protótipos corrigidos
void calibrationRoutine();
int16_t pid(int16_t error);
void IRAM_ATTR readEncoderL();
void IRAM_ATTR readEncoderR();

QTRSensors qtr;
Motors motors(IN1_R, IN1_L, CHN_R, CHN_L, FREQ, RES);

uint16_t sensorValues[SENSOR_QNT];
int center = 2500;
bool followingLine = true;

// Variáveis dos encoders (volatile para interrupções)
volatile long encoderLeft = 0;
volatile long encoderRight = 0;

// Variáveis de velocidade
unsigned long check_velocidade = 0;
const int intervalo = 50;
long lastEncoderLeft = 0;
long lastEncoderRight = 0;

const float diametro = 0.021;
const float PPR = 210.0;
const float pilido = 3.1415;

// Controle de malha fechada
float ajusteL = 0;
float ajusteR = 0;
float kp_vel = 0.05; // Valor sugerido para suavidade
float alvovel = 0.2;

// PID de Linha
double Kp = 0.5;
double Ki = 0.0;
double Kd = 0.0;
double lastTime, dt;
double previousError = 0;
int16_t correction = 0;

// Funções de Interrupção Corrigidas
void IRAM_ATTR readEncoderL() {
  if (digitalRead(ENCODER_CHN_B_L) == LOW) encoderLeft++;
  else encoderLeft--;
}

void IRAM_ATTR readEncoderR() {
  if (digitalRead(ENCODER_CHN_B_R) == LOW) encoderRight++;
  else encoderRight--;
}

void setup() {
  motors.setSpeeds(0, 0);
  pinMode(LEDON, OUTPUT);
  digitalWrite(LEDON, HIGH);

  pinMode(ENCODER_CHN_A_L, INPUT);
  pinMode(ENCODER_CHN_A_R, INPUT);
  pinMode(ENCODER_CHN_B_L, INPUT);
  pinMode(ENCODER_CHN_B_R, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_CHN_A_L), readEncoderL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_CHN_A_R), readEncoderR, RISING);

  dt = 0;
  lastTime = micros();

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){18, 5, 17, 16, 4, 0, 2, 15}, SENSOR_QNT);
  qtr.setEmitterPin(LEDON);

  Serial.begin(115200);
  delay(500);
  calibrationRoutine();
  delay(500);
}

void loop() {
  unsigned long currentTime = millis();

  // 1. PID de Linha (Malha Externa)
  double now = micros();
  dt = (now - lastTime) / 1000.00;

  if (dt > 0) {
    lastTime = now;
    if (followingLine) {
      uint16_t pos = qtr.readLineWhite(sensorValues);
      int16_t error = center - pos;
      correction = pid(error);
    }
  }

  // 2. PID de Velocidade (Malha Interna - Sincronização)
  if (currentTime - check_velocidade >= intervalo) {
    long delta_left = encoderLeft - lastEncoderLeft;
    long delta_right = encoderRight - lastEncoderRight;

    // Conversão para m/s
    float velocidadeL_ms = (delta_left * 1000.0 / intervalo) * (pilido * diametro / PPR);
    float velocidadeR_ms = (delta_right * 1000.0 / intervalo) * (pilido * diametro / PPR);

    // Cálculo do erro e acúmulo do ajuste
    ajusteL += (alvovel - velocidadeL_ms) * kp_vel;
    ajusteR += (alvovel - velocidadeR_ms) * kp_vel;

    // Limites de segurança para o ajuste não "explodir"
    ajusteL = constrain(ajusteL, -50, 50);
    ajusteR = constrain(ajusteR, -50, 50);

    lastEncoderLeft = encoderLeft;
    lastEncoderRight = encoderRight;
    check_velocidade = currentTime;

    // Debug Serial
    Serial.print(" L:"); Serial.print(velocidadeL_ms);
    Serial.print(" R:"); Serial.println(velocidadeR_ms);
  }

  // 3. Aplicação do PWM Final a partir do encoder
  int16_t pwmFinalL = (BASE_VEL - correction) + ajusteL;
  int16_t pwmFinalR = (BASE_VEL + correction) + ajusteR;

  motors.setSpeeds(constrain(pwmFinalR, 0, 255), constrain(pwmFinalL, 0, 255));
}

void calibrationRoutine() {
  motors.setSpeeds(100, -100); // Gira no próprio eixo para calibrar melhor
  for (uint16_t i = 0; i < 100; i++) {
    qtr.calibrate();
    delay(20);
  }
  motors.setSpeeds(0, 0);
}

int16_t pid(int16_t error) {
  static double integralError = 0;
  double proportionalError = error;
  integralError += error * dt;
  double derivativeError = (error - previousError) / dt;

  previousError = error;
  return (int16_t)(Kp * proportionalError + Ki * integralError + Kd * derivativeError);
}