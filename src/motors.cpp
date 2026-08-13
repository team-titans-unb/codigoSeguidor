#include <Arduino.h>
#include "motors.h"

Motors::Motors(int _pinIN1_R, int _pinIN1_L, int _chn_R, int _chn_L, int _freq, int _res) {
    pinIN1_R = _pinIN1_R;
    pinIN1_L = _pinIN1_L;
    chn_R = _chn_R;
    chn_L = _chn_L;
    freq = _freq;
    res = _res;
    
    config();
}

void Motors::config() {
    pinMode(pinIN1_R, OUTPUT);
    pinMode(pinIN1_L, OUTPUT);

    // Classic ESP32 Arduino v2.x PWM configuration
    ledcSetup(chn_R, freq, res);
    ledcAttachPin(pinIN1_R, chn_R);

    ledcSetup(chn_L, freq, res);
    ledcAttachPin(pinIN1_L, chn_L);
}

void Motors::setSpeeds(int velR, int velL) {
    velR = constrain(velR, 0, 255);
    velL = constrain(velL, 0, 255);

    ledcWrite(chn_R, velR);
    ledcWrite(chn_L, velL);
}