#include "Compass.h"


// =====================================================
// CONSTRUTOR
// =====================================================

Compass::Compass(uint8_t endereco)
    : endereco(endereco),
      angulo(0),
      anguloZero(0),
      valida(false)
{
}


// =====================================================
// INICIALIZAÇÃO
// =====================================================

void Compass::begin()
{
    Wire.begin();

    angulo = 0;
    anguloZero = 0;
    valida = false;
}


// =====================================================
// LEITURA
// =====================================================

bool Compass::ler()
{
    Wire.beginTransmission(endereco);

    Wire.write(0x44);

    if (Wire.endTransmission() != 0)
    {
        valida = false;
        return false;
    }


    uint8_t quantidade =
        Wire.requestFrom(
            endereco,
            2
        );


    if (quantidade < 2)
    {
        valida = false;
        return false;
    }


    byte lowbyte = Wire.read();
    byte highbyte = Wire.read();

    int leitura = word(
        highbyte,
        lowbyte
    );


    // Proteção contra valores impossíveis.
    if (leitura < 0 ||
        leitura >= 360)
    {
        valida = false;
        return false;
    }


    angulo = leitura;
    valida = true;

    return true;
}


// =====================================================
// CALIBRAÇÃO
// =====================================================

bool Compass::calibrar()
{
    if (!ler())
    {
        return false;
    }

    anguloZero = angulo;

    return true;
}


// =====================================================
// ÂNGULO ABSOLUTO
// =====================================================

int Compass::getAngle() const
{
    return angulo;
}


// =====================================================
// ÂNGULO RELATIVO
// =====================================================

int Compass::getRelativeAngle() const
{
    int relativo =
        angulo - anguloZero;

    while (relativo < 0)
    {
        relativo += 360;
    }

    while (relativo >= 360)
    {
        relativo -= 360;
    }

    return relativo;
}


// =====================================================
// VALIDADE
// =====================================================

bool Compass::isValid() const
{
    return valida;
}


// =====================================================
// ERRO ANGULAR
// =====================================================

int Compass::calcularErroAngular(
    float alvo,
    float atual
) const
{
    float erro =
        alvo - atual;


    while (erro > 180.0f)
    {
        erro -= 360.0f;
    }


    while (erro < -180.0f)
    {
        erro += 360.0f;
    }


    return static_cast<int>(erro);
}