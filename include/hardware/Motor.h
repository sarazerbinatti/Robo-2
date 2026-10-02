#pragma once

#include <Arduino.h>


// =====================================================
// CLASSE MOTOR
// =====================================================

class Motor
{
private:

    int pinoPWM;
    int pinoIN1;
    int pinoIN2;

public:

    Motor(int pwm, int in1, int in2);

    void begin();

    void girar(int velocidade);

    void parar();
};