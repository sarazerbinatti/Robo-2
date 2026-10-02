#pragma once

#include "Motor.h"


// =====================================================
// CLASSE OMNI DRIVE
// =====================================================

class OmniDrive
{
private:

    Motor &motorA;
    Motor &motorB;
    Motor &motorC;
    Motor &motorD;

    int saidaA;
    int saidaB;
    int saidaC;
    int saidaD;

public:

    OmniDrive(
        Motor &a,
        Motor &b,
        Motor &c,
        Motor &d
    );

    void begin();

    void mover(
        int frente,
        int lateral,
        int giro
    );

    void parar();

    int getMotorA() const;
    int getMotorB() const;
    int getMotorC() const;
    int getMotorD() const;
};