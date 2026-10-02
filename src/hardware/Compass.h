#pragma once

#include <Arduino.h>
#include <Wire.h>


// =====================================================
// CLASSE COMPASS
// =====================================================

class Compass
{
private:

    uint8_t endereco;

    int angulo;
    int anguloZero;

    bool valida;

public:

    Compass(uint8_t endereco);

    void begin();

    bool ler();

    bool calibrar();

    int getAngle() const;

    int getRelativeAngle() const;

    bool isValid() const;

    int calcularErroAngular(
        float alvo,
        float atual
    ) const;
};