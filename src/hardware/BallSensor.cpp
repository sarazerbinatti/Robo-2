#include "BallSensor.h"


// =====================================================
// CONSTRUTOR
// =====================================================

BallSensor::BallSensor()
    : direcao(0),
      intensidade(0)
{
}


// =====================================================
// INICIALIZAÇÃO
// =====================================================

void BallSensor::begin()
{
    InfraredSeeker::Initialize();

    direcao = 0;
    intensidade = 0;
}


// =====================================================
// ATUALIZAR LEITURA
// =====================================================

void BallSensor::update()
{
    InfraredResult resultado =
        InfraredSeeker::ReadAC();

    direcao = resultado.Direction;
    intensidade = resultado.Strength;
}


// =====================================================
// DIREÇÃO
// =====================================================

int BallSensor::getDirection() const
{
    return direcao;
}


// =====================================================
// INTENSIDADE
// =====================================================

int BallSensor::getStrength() const
{
    return intensidade;
}