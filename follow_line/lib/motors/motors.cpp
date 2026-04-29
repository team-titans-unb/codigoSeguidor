#include <Arduino.h>
#include "motors.h"


Motors::Motors( int _pinIN1_R, int _pinIN1_L,
                int _chn_R, int _chn_L,
                int _freq, int _res)
                : pinIN1_R(_pinIN1_R), pinIN1_L(_pinIN1_L), chn_R(_chn_R),
                  chn_L(_chn_L), freq(_freq), res(_res)
{
    config();
}

void Motors::config() {
    ledcAttachPin(pinIN1_R, 0); // Pino Direito no Canal 0
    ledcAttachPin(pinIN1_L, 1); // Pino Esquerdo no Canal 1
}

void Motors::setRightSpeed(int velocity){
    ledcWrite(0, velocity);
}

void Motors::setLeftSpeed(int velocity){
    ledcWrite(1, velocity);
}

void Motors::setSpeeds(int velocity_R, int velocity_L){
    setRightSpeed(velocity_R);
    setLeftSpeed(velocity_L);
}