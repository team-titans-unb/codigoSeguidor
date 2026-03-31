#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>

class Motors{
private:
    int pinIN1_R, pinIN1_L;
    int chn_R, chn_L;
    int freq;
    int res;

    /**
     * @brief Configures the motors.
     *
     */
    void config();
public:
    Motors( int _pinIN1_R, int _pinIN1_L,
            int _chn_R, int _chn_L,
            int _freq, int _res);

    /**
     * @brief Sets the speed to the right motor.
     *
     * @param velocity PWM value.
     */        
    void setRightSpeed(int velocity);

    /**
     * @brief Sets the speed to the left motor.
     *
     * @param velocity PWM value.
     */        
    void setLeftSpeed(int velocity);

    /**
     * @brief Sets the speed to both motors.
     *
     * @param velocity_R PWM value (right motor).
     * @param velocity_L PWM value (left motor).
     */        
    void setSpeeds(int velocity_R, int velocity_L);
};

#endif