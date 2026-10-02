#include "OmniDrive.h"


// =====================================================
// CONSTRUTOR
// =====================================================

OmniDrive::OmniDrive(
    Motor &a,
    Motor &b,
    Motor &c,
    Motor &d
)
    : motorA(a),
      motorB(b),
      motorC(c),
      motorD(d),
      saidaA(0),
      saidaB(0),
      saidaC(0),
      saidaD(0)
{
}


// =====================================================
// INICIALIZAÇÃO
// =====================================================

void OmniDrive::begin()
{
    motorA.begin();
    motorB.begin();
    motorC.begin();
    motorD.begin();

    parar();
}


// =====================================================
// MOVIMENTO OMNI
// =====================================================

void OmniDrive::mover(
    int frente,
    int lateral,
    int giro
)
{
    int A =
        -frente +
        lateral +
        giro;


    int B =
        frente -
        lateral +
        giro;


    int C =
        frente +
        lateral +
        giro;


    int D =
        -frente -
        lateral +
        giro;


    // =================================================
    // NORMALIZAÇÃO
    // =================================================
    //
    // Em vez de cortar cada motor individualmente com
    // constrain(), preservamos a proporção entre eles.
    //
    // Exemplo:
    // A = 300, B = 100
    //
    // Antes:
    // A = 255, B = 100
    //
    // Agora:
    // A = 255, B = 85
    //
    // A direção do vetor é preservada.
    // =================================================

    int maior =
        max(
            max(abs(A), abs(B)),
            max(abs(C), abs(D))
        );


    if (maior > 255)
    {
        A = (A * 255L) / maior;
        B = (B * 255L) / maior;
        C = (C * 255L) / maior;
        D = (D * 255L) / maior;
    }


    // =================================================
    // TELEMETRIA
    // =================================================

    saidaA = A;
    saidaB = B;
    saidaC = C;
    saidaD = D;


    // =================================================
    // APLICAÇÃO
    // =================================================

    motorA.girar(A);
    motorB.girar(B);
    motorC.girar(C);
    motorD.girar(D);
}


// =====================================================
// PARAR
// =====================================================

void OmniDrive::parar()
{
    motorA.parar();
    motorB.parar();
    motorC.parar();
    motorD.parar();

    saidaA = 0;
    saidaB = 0;
    saidaC = 0;
    saidaD = 0;
}


// =====================================================
// TELEMETRIA DOS MOTORES
// =====================================================

int OmniDrive::getMotorA() const
{
    return saidaA;
}


int OmniDrive::getMotorB() const
{
    return saidaB;
}


int OmniDrive::getMotorC() const
{
    return saidaC;
}


int OmniDrive::getMotorD() const
{
    return saidaD;
}