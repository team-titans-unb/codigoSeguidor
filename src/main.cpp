#include <Arduino.h>
#include <motors.h>
#include <QTRSensors.h>
#include "BluetoothSerial.h"

#ifndef SPECS
#define SPECS
#define FREQ 50000
#define RES 8
// --- AJUSTES DE VELOCIDADE ALVO ---
#define VEL_BASE_MS 1.0     // Aumentado de 0.30 para 0.65 m/s (ajuste conforme a pista)
#define SENSIBILIDADE 0.0005  // Aumentado para curvas mais agressivas (era 0.0005)
#endif

BluetoothSerial SerialBT;

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

// Protótipos
void calibrationRoutine();
int16_t pidLinha(int16_t error);
void IRAM_ATTR readEncoderL();
void IRAM_ATTR readEncoderR();

QTRSensors qtr;
Motors motors(IN1_R, IN1_L, CHN_R, CHN_L, FREQ, RES);

uint16_t sensorValues[SENSOR_QNT];
int center = 2500;

// Variáveis dos encoders
volatile long encoderLeft = 0;
volatile long encoderRight = 0;

// Variáveis de velocidade
unsigned long check_velocidade = 0;
const int intervalo = 20; // Reduzido para 20ms para resposta mais rápida da malha
long lastEncoderLeft = 0;
long lastEncoderRight = 0;

const float diametro = 0.021;
const float PPR = 210.0;
const float pilido = 3.1415;

// Controle de malha fechada (PWM Acumulado)
float ajusteL = 0; 
float ajusteR = 0;
float kp_vel = 0.15; // Aumentado para o motor reagir mais rápido ao erro de velocidade

// PID de Linha (Ajustado para maior velocidade)
double Kp = 0.8;   // Aumentado para resposta de direção mais rápida
double Ki = 0.0;
double Kd = 0.05;  // Aumentado para suavizar a oscilação em alta velocidade
double lastTime, dt;
double previousError = 0;

// Funções de Interrupção
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
  SerialBT.begin("seguidorlinha_JoaoVictor"); 

  delay(500);
  calibrationRoutine();
  delay(500);
}

void loop() {
  unsigned long currentTime = millis();

  // --- 1. PID DE LINHA (Malha Externa) ---
  double now = micros();
  dt = (now - lastTime) / 1000.00;
  lastTime = now;

  uint16_t pos = qtr.readLineWhite(sensorValues);
  int16_t error = center - pos;
  int16_t diff_velocidade = pidLinha(error);

  // --- 2. DEFINIÇÃO DE ALVOS INDEPENDENTES (m/s) ---
  float alvoL = VEL_BASE_MS - (diff_velocidade * SENSIBILIDADE); 
  float alvoR = VEL_BASE_MS + (diff_velocidade * SENSIBILIDADE);

  // --- 3. PID DE VELOCIDADE (Malha Interna Independente) ---
  if (currentTime - check_velocidade >= intervalo) {
    long delta_left = encoderLeft - lastEncoderLeft;
    long delta_right = encoderRight - lastEncoderRight;

    float vL_ms = (delta_left * 1000.0 / intervalo) * (pilido * diametro / PPR);
    float vR_ms = (delta_right * 1000.0 / intervalo) * (pilido * diametro / PPR);

    // O ajuste agora é mais agressivo para atingir o alvo rápido
    ajusteL += (alvoL - vL_ms) * (kp_vel * 150); 
    ajusteR += (alvoR - vR_ms) * (kp_vel * 150);

    ajusteL = constrain(ajusteL, 0, 255);
    ajusteR = constrain(ajusteR, 0, 255);

    lastEncoderLeft = encoderLeft;
    lastEncoderRight = encoderRight;
    check_velocidade = currentTime;

    // Telemetria resumida para não engasgar o Bluetooth em alta velocidade
    if(currentTime % 100 == 0) {
       SerialBT.printf("V:%.2f|%.2f PWM:%d|%d\n", vL_ms, vR_ms, (int)ajusteL, (int)ajusteR);
    }
  }

  // --- 4. APLICAÇÃO NOS MOTORES ---
  motors.setSpeeds(ajusteR, ajusteL);
}

void calibrationRoutine() {
  motors.setSpeeds(100, -100); // Calibração um pouco mais rápida
  for (uint16_t i = 0; i < 100; i++) {
    qtr.calibrate();
    delay(20);
  }
  motors.setSpeeds(0, 0);
  ajusteL = 0;
  ajusteR = 0;
}

int16_t pidLinha(int16_t error) {
  static double integralError = 0;
  double proportionalError = error;
  integralError += error * dt;
  double derivativeError = (error - previousError) / dt;

  previousError = error;
  // Ki permanece 0 para evitar o "efeito mola" em alta velocidade
  return (int16_t)(Kp * proportionalError + Ki * integralError + Kd * derivativeError);
}