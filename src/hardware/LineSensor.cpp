#include "LineSensor.h"


// =====================================================
// CONSTRUTOR
// =====================================================

LineSensor::LineSensor(
    int qre1,
    int qre2,
    int qre3,
    int qre4,
    int limiar1,
    int limiar2,
    int limiar3,
    int limiar4
)
{
    pinos[0] = qre1;
    pinos[1] = qre2;
    pinos[2] = qre3;
    pinos[3] = qre4;

    limiares[0] = limiar1;
    limiares[1] = limiar2;
    limiares[2] = limiar3;
    limiares[3] = limiar4;

    for (int i = 0; i < 4; i++)
    {
        valores[i] = 0;
    }
}


// =====================================================
// INICIALIZAÇÃO
// =====================================================

void LineSensor::begin()
{
    for (int i = 0; i < 4; i++)
    {
        pinMode(
            pinos[i],
            INPUT
        );
    }
}


// =====================================================
// ATUALIZAR
// =====================================================

void LineSensor::update()
{
    for (int i = 0; i < 4; i++)
    {
        valores[i] =
            analogRead(pinos[i]);
    }
}


// =====================================================
// VALOR
// =====================================================

int LineSensor::getValue(int sensor) const
{
    if (sensor < 0 ||
        sensor > 3)
    {
        return 0;
    }

    return valores[sensor];
}


// =====================================================
// DETECÇÃO INDIVIDUAL
// =====================================================

bool LineSensor::detectouLinha(int sensor) const
{
    if (sensor < 0 ||
        sensor > 3)
    {
        return false;
    }

    // O QRE está invertido:
    // branco -> ADC menor
    // verde  -> ADC maior

    return valores[sensor] <
           limiares[sensor];
}


// =====================================================
// QUALQUER LINHA
// =====================================================

bool LineSensor::detectouAlgumaLinha() const
{
    for (int i = 0; i < 4; i++)
    {
        if (detectouLinha(i))
        {
            return true;
        }
    }

    return false;
}