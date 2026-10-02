#include "Motor.h"


// =====================================================
// CONSTRUTOR
// =====================================================

Motor::Motor(int pwm, int in1, int in2)
    : pinoPWM(pwm),
      pinoIN1(in1),
      pinoIN2(in2)
{
}


// =====================================================
// INICIALIZAÇÃO
// =====================================================

void Motor::begin()
{
    pinMode(pinoPWM, OUTPUT);
    pinMode(pinoIN1, OUTPUT);
    pinMode(pinoIN2, OUTPUT);

    parar();
}


// =====================================================
// GIRAR MOTOR
// =====================================================

void Motor::girar(int velocidade)
{
    velocidade = constrain(velocidade, -255, 255);

    if (velocidade > 0)
    {
        digitalWrite(pinoIN1, HIGH);
        digitalWrite(pinoIN2, LOW);
    }
    else if (velocidade < 0)
    {
        digitalWrite(pinoIN1, LOW);
        digitalWrite(pinoIN2, HIGH);
    }
    else
    {
        parar();
        return;
    }

    analogWrite(pinoPWM, abs(velocidade));
}


// =====================================================
// PARAR
// =====================================================

void Motor::parar()
{
    digitalWrite(pinoIN1, LOW);
    digitalWrite(pinoIN2, LOW);

    analogWrite(pinoPWM, 0);
}   