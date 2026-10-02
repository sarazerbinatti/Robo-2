#pragma once

#include <Arduino.h>


// =====================================================
// CLASSE LINE SENSOR
// =====================================================

class LineSensor
{
private:

    int pinos[4];

    int valores[4];
    int limiares[4];

public:

    LineSensor(
        int qre1,
        int qre2,
        int qre3,
        int qre4,
        int limiar1,
        int limiar2,
        int limiar3,
        int limiar4
    );

    void begin();

    void update();

    int getValue(int sensor) const;

    bool detectouLinha(int sensor) const;

    bool detectouAlgumaLinha() const;
};