#include "Robot.h"
#include "config/Config.h"


// =====================================================
// CONSTRUTOR
// =====================================================

Robot::Robot()

    // -------------------------------------------------
    // MOTORES
    // -------------------------------------------------

    : motorA(
        ENA_1,
        IN1_1,
        IN2_1
    ),

      motorB(
          ENB_1,
          IN3_1,
          IN4_1
      ),

      motorC(
          ENA_2,
          IN1_2,
          IN2_2
      ),

      motorD(
          ENB_2,
          IN3_2,
          IN4_2
      ),


      // -------------------------------------------------
      // DRIVE
      // -------------------------------------------------

      drive(
          motorA,
          motorB,
          motorC,
          motorD
      ),


      // -------------------------------------------------
      // BÚSSOLA
      // -------------------------------------------------

      compass(
          COMPASS_ADDRESS
      ),


      // -------------------------------------------------
      // SENSOR DE BOLA
      // -------------------------------------------------

      ballSensor(),


      // -------------------------------------------------
      // SENSOR DE LINHA
      // -------------------------------------------------

      lineSensor(
          QRE_1,
          QRE_2,
          QRE_3,
          QRE_4,

          LIMIAR_1,
          LIMIAR_2,
          LIMIAR_3,
          LIMIAR_4
      ),


      // -------------------------------------------------
      // PID
      // -------------------------------------------------

      pid(
          PID_KP,
          PID_KI,
          PID_KD,

          PID_INTERVAL_MS,

          PID_DT_MIN,
          PID_DT_MAX,

          INTEGRAL_MAX,
          PID_MAX,

          DERIVATIVE_FILTER_ALPHA,
          DERIVATIVE_MAX
      ),


      // -------------------------------------------------
      // ESTADO
      // -------------------------------------------------

      lateral(0),
      giro(0),

      pidAtivo(true),

      serialBuffer(""),

      ultimoDebug(0)
{
}


// =====================================================
// SETUP DO ROBÔ
// =====================================================

void Robot::begin()
{
    Serial.begin(
        SERIAL_BAUDRATE
    );

    Serial.setTimeout(
        SERIAL_TIMEOUT
    );


    // -------------------------------------------------
    // I2C
    // -------------------------------------------------

    Wire.begin();


    // -------------------------------------------------
    // MOTORES
    // -------------------------------------------------

    drive.begin();


    // -------------------------------------------------
    // SENSORES
    // -------------------------------------------------

    lineSensor.begin();

    ballSensor.begin();

    compass.begin();


    // -------------------------------------------------
    // TEMPO PARA I2C
    // -------------------------------------------------

    // Dá tempo para o sensor/I2C estabilizar antes da calibração.
    delay(100);


    // -------------------------------------------------
    // CALIBRAÇÃO DA BÚSSOLA
    // -------------------------------------------------

    if (!compass.calibrar())
    {
        Serial.println(
            "ERRO: NAO FOI POSSIVEL CALIBRAR BUSSOLA"
        );
    }


    // -------------------------------------------------
    // PID
    // -------------------------------------------------

    pid.begin();


    // -------------------------------------------------
    // SEGURANÇA
    // -------------------------------------------------

    drive.parar();


    // -------------------------------------------------
    // INICIALIZAÇÃO
    // -------------------------------------------------

    Serial.println(
        "ROBO INICIADO"
    );

    enviarPID();
}


// =====================================================
// LOOP DO ROBÔ
// =====================================================

void Robot::update()
{
    processarSerial();


    // -------------------------------------------------
    // SENSORES
    // -------------------------------------------------

    atualizarSensores();


    // -------------------------------------------------
    // PID
    // -------------------------------------------------

    atualizarPID();


    // -------------------------------------------------
    // CONTROLE DA BOLA
    // -------------------------------------------------

    controlarBola();


    // -------------------------------------------------
    // MOVIMENTO
    // -------------------------------------------------

    mover();


    // -------------------------------------------------
    // DEBUG
    // -------------------------------------------------

    debugSerial();
}


// =====================================================
// ATUALIZAR SENSORES
// =====================================================

void Robot::atualizarSensores()
{
    ballSensor.update();

    lineSensor.update();

    compass.ler();
}


// =====================================================
// ATUALIZAR PID
// =====================================================

void Robot::atualizarPID()
{
    if (pidAtivo &&
        compass.isValid())
    {
        float erro =
            compass.calcularErroAngular(
                0.0f,
                compass.getAngle()
            );


        // -------------------------------------------------
        // ZONA MORTA
        // -------------------------------------------------

        if (fabs(erro) <= BUSSOLA_ZONA_MORTA)
        {
            erro = 0.0f;
        }


        float saida =
            pid.calcular(erro);


        giro = constrain(
            static_cast<int>(saida),
            -255,
            255
        );
    }


    // -------------------------------------------------
    // BÚSSOLA INVÁLIDA
    // -------------------------------------------------

    else if (!compass.isValid())
    {
        // Não cria um comando de giro baseado em uma leitura inválida.

        giro = 0;
    }


    // -------------------------------------------------
    // PID DESLIGADO
    // -------------------------------------------------

    else
    {
        giro = 0;
    }
}


// =====================================================
// CONTROLE LATERAL PELO IR
// =====================================================

void Robot::controlarBola()
{
    int direcao =
        ballSensor.getDirection();


    switch (direcao)
    {
        case 7:

            lateral =
                VELOCIDADE_LATERAL_7;

            break;


        case 6:

            lateral =
                VELOCIDADE_LATERAL_6;

            break;


        case 5:

            lateral =
                VELOCIDADE_LATERAL_5;

            break;


        case 4:

            lateral =
                VELOCIDADE_LATERAL_4;

            break;


        case 3:

            lateral =
                VELOCIDADE_LATERAL_3;

            break;


        default:

            lateral = 0;

            break;
    }
}


// =====================================================
// MOVIMENTO
// =====================================================

void Robot::mover()
{
    drive.mover(
        0,
        lateral,
        giro
    );
}


// =====================================================
// DEBUG SERIAL
// =====================================================

void Robot::debugSerial()
{
    if (
        millis() - ultimoDebug <
        DEBUG_INTERVAL_MS
    )
    {
        return;
    }


    ultimoDebug = millis();


    Serial.print("[DEBUG] ");


    // -------------------------------------------------
    // IR
    // -------------------------------------------------

    Serial.print("IR: dir=");
    Serial.print(
        ballSensor.getDirection()
    );

    Serial.print(" str=");
    Serial.print(
        ballSensor.getStrength()
    );


    // -------------------------------------------------
    // BÚSSOLA
    // -------------------------------------------------

    Serial.print(
        " | COMPASS: angle="
    );

    Serial.print(
        compass.getAngle()
    );

    Serial.print(
        " target=0 err="
    );


    int erro =
        compass.calcularErroAngular(
            0.0f,
            compass.getAngle()
        );

    Serial.print(
        erro
    );


    // -------------------------------------------------
    // PID
    // -------------------------------------------------

    Serial.print(
        " | PID: P="
    );

    Serial.print(
        pid.getKp() * erro,
        2
    );


    Serial.print(
        " I="
    );

    Serial.print(
        pid.getKi() *
        pid.getIntegral(),
        2
    );


    Serial.print(
        " D="
    );

    Serial.print(
        pid.getKd() *
        pid.getDerivada(),
        2
    );


    Serial.print(
        " OUT="
    );

    Serial.print(
        pid.getSaida(),
        2
    );


    // -------------------------------------------------
    // MOVIMENTO
    // -------------------------------------------------

    Serial.print(
        " | MOVE: F=0 L="
    );

    Serial.print(lateral);


    Serial.print(
        " G="
    );

    Serial.print(giro);


    // -------------------------------------------------
    // MOTORES
    // -------------------------------------------------

    Serial.print(
        " | MOTOR: A="
    );

    Serial.print(
        drive.getMotorA()
    );


    Serial.print(
        " B="
    );

    Serial.print(
        drive.getMotorB()
    );


    Serial.print(
        " C="
    );

    Serial.print(
        drive.getMotorC()
    );


    Serial.print(
        " D="
    );

    Serial.println(
        drive.getMotorD()
    );
}


// =====================================================
// PROCESSAMENTO DA SERIAL
// =====================================================

void Robot::processarSerial()
{
    while (Serial.available())
    {
        char c =
            Serial.read();


        if (c == '\n')
        {
            serialBuffer.trim();


            if (serialBuffer.length() > 0)
            {
                processarComando(
                    serialBuffer
                );
            }


            serialBuffer = "";
        }


        else if (c != '\r')
        {
            // Proteção contra crescimento indefinido do String
            // caso chegue uma mensagem sem '\n'.

            if (serialBuffer.length() < 80)
            {
                serialBuffer += c;
            }


            else
            {
                serialBuffer = "";

                Serial.println(
                    "ERRO SERIAL: COMANDO MUITO LONGO"
                );
            }
        }
    }
}


// =====================================================
// PROCESSAMENTO DE COMANDOS
// =====================================================

void Robot::processarComando(
    const String &comando
)
{
    // =================================================
    // SET PID
    //
    // SET PID 1.80 0.15 0.35
    // =================================================

    if (comando.startsWith("SET PID"))
    {
        String dados =
            comando.substring(7);

        dados.trim();


        int espaco1 =
            dados.indexOf(' ');


        if (espaco1 == -1)
        {
            Serial.println(
                "ERRO PID: use SET PID KP KI KD"
            );

            return;
        }


        String textoKp =
            dados.substring(
                0,
                espaco1
            );


        dados =
            dados.substring(
                espaco1 + 1
            );

        dados.trim();


        int espaco2 =
            dados.indexOf(' ');


        if (espaco2 == -1)
        {
            Serial.println(
                "ERRO PID: use SET PID KP KI KD"
            );

            return;
        }


        String textoKi =
            dados.substring(
                0,
                espaco2
            );


        String textoKd =
            dados.substring(
                espaco2 + 1
            );


        textoKi.trim();
        textoKd.trim();


        // -----------------------------------------
        // CONVERSÃO
        // -----------------------------------------

        float novoKp =
            textoKp.toFloat();

        float novoKi =
            textoKi.toFloat();

        float novoKd =
            textoKd.toFloat();


        // -----------------------------------------
        // ATUALIZA
        // -----------------------------------------

        if (!pid.setPID(
            novoKp,
            novoKi,
            novoKd
        ))
        {
            Serial.println(
                "ERRO PID: valores invalidos"
            );

            return;
        }


        // -----------------------------------------
        // CONFIRMA
        // -----------------------------------------

        Serial.print(
            "OK PID KP="
        );

        Serial.print(
            pid.getKp(),
            6
        );


        Serial.print(
            " KI="
        );

        Serial.print(
            pid.getKi(),
            6
        );


        Serial.print(
            " KD="
        );

        Serial.println(
            pid.getKd(),
            6
        );


        return;
    }


    // =================================================
    // GET PID
    // =================================================

    if (comando == "GET PID")
    {
        enviarPID();

        return;
    }


    // =================================================
    // RESET PID
    // =================================================

    if (comando == "RESET PID")
    {
        pid.reset();


        Serial.println(
            "OK PID RESET"
        );


        return;
    }


    // =================================================
    // PID ON
    // =================================================

    if (comando == "PID ON")
    {
        pidAtivo = true;

        pid.reset();


        Serial.println(
            "OK PID ON"
        );


        return;
    }


    // =================================================
    // PID OFF
    // =================================================

    if (comando == "PID OFF")
    {
        pidAtivo = false;


        pid.reset();


        giro = 0;


        Serial.println(
            "OK PID OFF"
        );


        return;
    }


    // =================================================
    // COMANDO DESCONHECIDO
    // =================================================

    Serial.print(
        "ERRO COMANDO: "
    );

    Serial.println(
        comando
    );
}


// =====================================================
// ENVIAR GANHOS ATUAIS
// =====================================================

void Robot::enviarPID()
{
    Serial.print(
        "OK PID KP="
    );

    Serial.print(
        pid.getKp(),
        6
    );


    Serial.print(
        " KI="
    );

    Serial.print(
        pid.getKi(),
        6
    );


    Serial.print(
        " KD="
    );

    Serial.println(
        pid.getKd(),
        6
    );
}