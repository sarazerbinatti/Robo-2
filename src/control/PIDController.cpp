#include "PIDController.h"


// =====================================================
// CONSTRUTOR
// =====================================================

PIDController::PIDController(
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
)
    : Kp(kp),
      Ki(ki),
      Kd(kd),
      erroAnterior(0.0f),
      integral(0.0f),
      derivada(0.0f),
      saida(0.0f),
      ultimoPID(0),
      integralMax(integralMax),
      pidMax(pidMax),
      derivativeFilterAlpha(derivativeFilterAlpha),
      derivativeMax(derivativeMax),
      intervalMs(intervalMs),
      dtMin(dtMin),
      dtMax(dtMax)
{
}


// =====================================================
// INICIALIZAÇÃO
// =====================================================

void PIDController::begin()
{
    reset();
}


// =====================================================
// CÁLCULO PID
// =====================================================

float PIDController::calcular(float erroAtual)
{
    unsigned long agora = millis();

    unsigned long deltaMs = agora - ultimoPID;

    // Não calcula o PID em toda iteração do loop.
    // Isso evita dt muito pequeno e deixa a derivada previsível.
    if (deltaMs < intervalMs)
    {
        return saida;
    }

    float dt = deltaMs / 1000.0f;

    // Se houve uma pausa anormal, não deixe esse intervalo
    // contaminar o cálculo da derivada.
    if (dt > dtMax)
    {
        dt = dtMin;
    }

    dt = constrain(
        dt,
        dtMin,
        dtMax
    );

    ultimoPID = agora;


    // =================================================
    // TERMO INTEGRAL
    // =================================================

    integral += erroAtual * dt;

    integral = constrain(
        integral,
        -integralMax,
        integralMax
    );


    // =================================================
    // TERMO DERIVATIVO
    // =================================================

    float derivadaBruta =
        (erroAtual - erroAnterior) / dt;

    // Limita a derivada antes do filtro.
    derivadaBruta = constrain(
        derivadaBruta,
        -derivativeMax,
        derivativeMax
    );


    // Filtro passa-baixas:
    //
    // D_filtrado =
    //     alpha * D_novo +
    //     (1-alpha) * D_anterior

    derivada =
        derivativeFilterAlpha * derivadaBruta +
        (1.0f - derivativeFilterAlpha) * derivada;


    // =================================================
    // SAÍDA PID
    // =================================================

    saida =
        (Kp * erroAtual) +
        (Ki * integral) +
        (Kd * derivada);


    // =================================================
    // LIMITAÇÃO DA SAÍDA
    // =================================================

    saida = constrain(
        saida,
        -pidMax,
        pidMax
    );

    erroAnterior = erroAtual;

    return saida;
}


// =====================================================
// RESET
// =====================================================

void PIDController::reset()
{
    integral = 0.0f;
    derivada = 0.0f;
    saida = 0.0f;

    erroAnterior = 0.0f;

    ultimoPID = millis();
}


// =====================================================
// ALTERAR GANHOS
// =====================================================

void PIDController::setGains(
    float kp,
    float ki,
    float kd
)
{
    Kp = max(0.0f, kp);
    Ki = max(0.0f, ki);
    Kd = max(0.0f, kd);

    reset();
}


// =====================================================
// GETTERS
// =====================================================

float PIDController::getKp() const
{
    return Kp;
}


float PIDController::getKi() const
{
    return Ki;
}


float PIDController::getKd() const
{
    return Kd;
}


float PIDController::getIntegral() const
{
    return integral;
}


float PIDController::getDerivada() const
{
    return derivada;
}


float PIDController::getSaida() const
{
    return saida;
}


// =====================================================
// SET PID
// =====================================================

bool PIDController::setPID(
    float kp,
    float ki,
    float kd
)
{
    if (kp < 0.0f ||
        ki < 0.0f ||
        kd < 0.0f)
    {
        return false;
    }

    setGains(kp, ki, kd);

    return true;
}