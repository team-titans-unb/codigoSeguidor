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
    ledcAttachChannel(pinIN1_R, freq, res, chn_R);
    ledcAttachChannel(pinIN1_L, freq, res, chn_L);
}

void Motors::setRightSpeed(int velocity){
    ledcWrite(chn_R, velocity);
}

void Motors::setLeftSpeed(int velocity){
    ledcWrite(chn_L, velocity);
}

void Motors::setSpeeds(int velocity_R, int velocity_L){
    setRightSpeed(velocity_R);
    setLeftSpeed(velocity_L);
}