#pragma once

// =====================================================
// CONFIGURAÇÃO GERAL DO ROBO
// =====================================================


// =====================================================
// PINOS PWM
// =====================================================

const int ENA_1 = 2;   // Motor A
const int ENB_1 = 3;   // Motor B
const int ENA_2 = 8;   // Motor C
const int ENB_2 = 9;   // Motor D


// =====================================================
// PINOS DE DIREÇÃO
// =====================================================

const int IN1_1 = 4;
const int IN2_1 = 5;

const int IN3_1 = 6;
const int IN4_1 = 7;

const int IN1_2 = 10;
const int IN2_2 = 11;

const int IN3_2 = 12;
const int IN4_2 = 13;


// =====================================================
// SENSORES QRE1113
// =====================================================

const int QRE_1 = A1;
const int QRE_2 = A2;
const int QRE_3 = A3;
const int QRE_4 = A4;


// =====================================================
// CALIBRAÇÃO QRE
// =====================================================

// Verde
const int VERDE_1 = 910;
const int VERDE_2 = 930;
const int VERDE_3 = 940;
const int VERDE_4 = 910;

// Branco
const int BRANCO_1 = 740;
const int BRANCO_2 = 735;
const int BRANCO_3 = 880;
const int BRANCO_4 = 865;

// Limiares
const int LIMIAR_1 = (VERDE_1 + BRANCO_1 + 1) / 2;
const int LIMIAR_2 = (VERDE_2 + BRANCO_2 + 1) / 2;
const int LIMIAR_3 = (VERDE_3 + BRANCO_3 + 1) / 2;
const int LIMIAR_4 = (VERDE_4 + BRANCO_4 + 1) / 2;


// =====================================================
// ULTRASSÔNICOS
// =====================================================

const int TRIG_1 = 22;
const int ECHO_1 = 23;

const int TRIG_2 = 24;
const int ECHO_2 = 25;

const int TRIG_3 = 26;
const int ECHO_3 = 27;


// =====================================================
// BÚSSOLA
// =====================================================

const int COMPASS_ADDRESS = 0x01;


// =====================================================
// PID
// =====================================================

const float PID_KP = 6.0f;
const float PID_KI = 0.0f;
const float PID_KD = 1.5f;

// O PID não precisa rodar a cada iteração do loop.
// Um período fixo deixa o cálculo da derivada muito mais previsível.
const unsigned long PID_INTERVAL_MS = 20;

// Proteção contra intervalos anormais.
const float PID_DT_MIN = 0.005f;
const float PID_DT_MAX = 0.100f;

// Limites
const float INTEGRAL_MAX = 100.0f;
const float PID_MAX = 255.0f;

// Filtro passa-baixas do termo D.
// Quanto menor, mais forte o filtro.
// 0.0 = D completamente congelado.
// 1.0 = sem filtragem.
const float DERIVATIVE_FILTER_ALPHA = 0.20f;

// Limite opcional para o termo derivativo antes de multiplicar por Kd.
// Evita que uma leitura extremamente ruim domine o PID.
const float DERIVATIVE_MAX = 300.0f;


// =====================================================
// MOVIMENTAÇÃO
// =====================================================

const int VELOCIDADE_LATERAL_7 = -120;
const int VELOCIDADE_LATERAL_6 = -90;
const int VELOCIDADE_LATERAL_5 = 0;
const int VELOCIDADE_LATERAL_4 = 90;
const int VELOCIDADE_LATERAL_3 = 120;


// =====================================================
// COMUNICAÇÃO SERIAL
// =====================================================

const long SERIAL_BAUDRATE = 115200;
const int SERIAL_TIMEOUT = 5;


// =====================================================
// DEBUG
// =====================================================

const unsigned long DEBUG_INTERVAL_MS = 100;


// =====================================================
// CONTROLE DA BÚSSOLA
// =====================================================

const float BUSSOLA_ZONA_MORTA = 5.0f;