#pragma once

#include <Arduino.h>

#include "hardware/Motor.h"
#include "hardware/OmniDrive.h"
#include "control/PIDController.h"
#include "hardware/Compass.h"
#include "hardware/BallSensor.h"
#include "hardware/LineSensor.h"


// =====================================================
// CLASSE ROBOT
// =====================================================

class Robot
{
private:

    // -------------------------------------------------
    // MOTORES
    // -------------------------------------------------

    Motor motorA;
    Motor motorB;
    Motor motorC;
    Motor motorD;


    // -------------------------------------------------
    // SISTEMA DE MOVIMENTO
    // -------------------------------------------------

    OmniDrive drive;


    // -------------------------------------------------
    // SENSORES
    // -------------------------------------------------

    Compass compass;
    BallSensor ballSensor;
    LineSensor lineSensor;


    // -------------------------------------------------
    // PID
    // -------------------------------------------------

    PIDController pid;


    // -------------------------------------------------
    // ESTADO DE MOVIMENTO
    // -------------------------------------------------

    int lateral;
    int giro;


    // -------------------------------------------------
    // CONTROLE PID
    // -------------------------------------------------

    bool pidAtivo;


    // -------------------------------------------------
    // SERIAL
    // -------------------------------------------------

    String serialBuffer;


    // -------------------------------------------------
    // DEBUG
    // -------------------------------------------------

    unsigned long ultimoDebug;


private:

    void atualizarSensores();

    void atualizarPID();

    void controlarBola();

    void mover();

    void debugSerial();

    void processarSerial();

    void processarComando(
        const String &comando
    );

    void enviarPID();


public:

    Robot();

    void begin();

    void update();
};