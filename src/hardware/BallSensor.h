#pragma once

#include <Arduino.h>
#include <HTInfraredSeeker.h>


// =====================================================
// CLASSE BALL SENSOR
// =====================================================

class BallSensor
{
private:

    int direcao;
    int intensidade;

public:

    BallSensor();

    void begin();

    void update();

    int getDirection() const;

    int getStrength() const;
};