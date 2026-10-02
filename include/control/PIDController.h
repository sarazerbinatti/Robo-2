#pragma once

#include <Arduino.h>


// =====================================================
// CLASSE PID CONTROLLER
// =====================================================

class PIDController
{
private:

    float Kp;
    float Ki;
    float Kd;

    float erroAnterior;
    float integral;
    float derivada;

    float saida;

    unsigned long ultimoPID;

    float integralMax;
    float pidMax;

    float derivativeFilterAlpha;
    float derivativeMax;

    unsigned long intervalMs;
    float dtMin;
    float dtMax;

public:

    PIDController(
        float kp,
        float ki,
        float kd,
        unsigned long intervalMs,
        float dtMin,
        float dtMax,
        float integralMax,
        float pidMax,
        float derivativeFilterAlpha,
        float derivativeMax
    );

    void begin();

    float calcular(float erroAtual);

    void reset();

    void setGains(float kp, float ki, float kd);

    float getKp() const;
    float getKi() const;
    float getKd() const;

    float getIntegral() const;
    float getDerivada() const;
    float getSaida() const;

    bool setPID(
        float kp,
        float ki,
        float kd
    );
};