#include <Arduino.h>
#include <Wire.h>
#include <HTInfraredSeeker.h>
#include <math.h>

// =====================================================
// CLASSE MOTOR
// =====================================================

class Motor
{
private:
    int pinoPWM;
    int pinoIN1;
    int pinoIN2;

public:

    Motor(int pwm, int in1, int in2)
    {
        pinoPWM = pwm;
        pinoIN1 = in1;
        pinoIN2 = in2;

        pinMode(pinoPWM, OUTPUT);
        pinMode(pinoIN1, OUTPUT);
        pinMode(pinoIN2, OUTPUT);
    }

    void girar(int velocidade)
    {
        velocidade = constrain(velocidade, -255, 255);

        if (velocidade > 0)
        {
            // HORÁRIO
            digitalWrite(pinoIN1, HIGH);
            digitalWrite(pinoIN2, LOW);
        }
        else if (velocidade < 0)
        {
            // ANTI-HORÁRIO
            digitalWrite(pinoIN1, LOW);
            digitalWrite(pinoIN2, HIGH);
        }
        else
        {
            // PARADO
            digitalWrite(pinoIN1, LOW);
            digitalWrite(pinoIN2, LOW);
        }

        analogWrite(pinoPWM, abs(velocidade));
    }
};


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

int Bussola = 0;
int BussolaZero = 0;


// =====================================================
// PID
// =====================================================

float Kp = 1.8;
float Ki = 0.15;
float Kd = 0.35;

// Variáveis do PID
float erro = 0.0;
float erroAnterior = 0.0;

float integral = 0.0;
float derivada = 0.0;

float saidaPID = 0.0;

// Controle de tempo
unsigned long ultimoPID = 0;

// Limites
const float INTEGRAL_MAX = 100.0;
const float PID_MAX = 255.0;

// PID habilitado
bool pidAtivo = true;


// =====================================================
// MOVIMENTAÇÃO
// =====================================================

int lateral = 0;
int giro = 0;

// Saída calculada para cada motor (telemetria)
int motorA_saida = 0;
int motorB_saida = 0;
int motorC_saida = 0;
int motorD_saida = 0;


// =====================================================
// SENSOR IR
// =====================================================

int ballDirecao = 0;
int ballIntens = 0;


// =====================================================
// OBJETOS
// =====================================================

Motor motorA(ENA_1, IN1_1, IN2_1);
Motor motorB(ENB_1, IN3_1, IN4_1);
Motor motorC(ENA_2, IN1_2, IN2_2);
Motor motorD(ENB_2, IN3_2, IN4_2);


// =====================================================
// PROTÓTIPOS
// =====================================================

void setPWM(
    int pwmA,
    int pwmB,
    int pwmC,
    int pwmD
);

void parar_motores();

void mov_frente();
void mov_tras();
void mov_esquerda();
void mov_direita();

void giro_esquerda();
void giro_direita();

void moverOmni(
    int frente,
    int lateral,
    int giro
);

int lerBussola();
int lerBussolaRelativa();

float calcularErroAngular(
    float alvo,
    float atual
);

void calcularPID(
    float erroAtual
);

// Comunicação Serial
void processarSerial();
void processarComando(String comando);

void enviarPID();
void resetarPID();

void debugSerial();


// =====================================================
// SETUP
// =====================================================

void setup()
{
    // =================================================
    // COMUNICAÇÃO SERIAL
    // =================================================

    Serial.begin(115200);

    // Timeout curto para não travar o loop
    Serial.setTimeout(5);


    // =================================================
    // I2C
    // =================================================

    Wire.begin();


    // =================================================
    // PINOS DE DIREÇÃO
    // =================================================

    pinMode(IN1_1, OUTPUT);
    pinMode(IN2_1, OUTPUT);

    pinMode(IN3_1, OUTPUT);
    pinMode(IN4_1, OUTPUT);

    pinMode(IN1_2, OUTPUT);
    pinMode(IN2_2, OUTPUT);

    pinMode(IN3_2, OUTPUT);
    pinMode(IN4_2, OUTPUT);


    // =================================================
    // PINOS PWM
    // =================================================

    pinMode(ENA_1, OUTPUT);
    pinMode(ENB_1, OUTPUT);

    pinMode(ENA_2, OUTPUT);
    pinMode(ENB_2, OUTPUT);


    // =================================================
    // PINOS QRE
    // =================================================

    pinMode(QRE_1, INPUT);
    pinMode(QRE_2, INPUT);
    pinMode(QRE_3, INPUT);
    pinMode(QRE_4, INPUT);


    // =================================================
    // SENSOR IR
    // =================================================

    InfraredSeeker::Initialize();


    // =================================================
    // CALIBRAÇÃO DA BÚSSOLA
    // =================================================

    BussolaZero = lerBussola();

    delay(500);


    // =================================================
    // INICIALIZAÇÃO DO PID
    // =================================================

    ultimoPID = millis();

    erroAnterior = 0.0;
    integral = 0.0;
    derivada = 0.0;
    saidaPID = 0.0;


    // =================================================
    // GARANTE MOTORES PARADOS
    // =================================================

    parar_motores();


    // =================================================
    // MENSAGEM DE INICIALIZAÇÃO
    // =================================================

    Serial.println("ROBO INICIADO");

    // Envia os ganhos atuais
    enviarPID();
}


// =====================================================
// LOOP PRINCIPAL
// =====================================================

void loop()
{
    // =================================================
    // PROCESSA COMANDOS DO PC
    // =================================================

    processarSerial();


    // =================================================
    // LEITURA DO SENSOR IR
    // =================================================

    InfraredResult InfraredBall =
        InfraredSeeker::ReadAC();

    ballDirecao = InfraredBall.Direction;
    ballIntens = InfraredBall.Strength;


    // =================================================
    // LEITURA DA BÚSSOLA
    // =================================================

    Bussola = lerBussolaRelativa();

    erro = calcularErroAngular(
        0,
        Bussola
    );

    const float ZONA_MORTA = 5.0;

    if (fabs(erro) <= ZONA_MORTA)
    {
        erro = 0;
    }


    // =================================================
    // PID
    // =================================================

    if (pidAtivo)
    {
        calcularPID(erro);

        giro = constrain(
            (int)saidaPID,
            -255,
            255
        );
    }
    else
    {
        giro = 0;
    }


    // =================================================
    // CONTROLE LATERAL PELO IR
    // =================================================

    switch (ballDirecao)
    {
        case 7:
            lateral = -120;
            break;

        case 6:
            lateral = -90;
            break;

        case 5:
            lateral = 0;
            break;

        case 4:
            lateral = 90;
            break;

        case 3:
            lateral = 120;
            break;

        default:
            lateral = 0;
            break;
    }


    // =================================================
    // MOVIMENTO OMNI
    // =================================================

    // Atualmente o controle usa apenas deslocamento lateral
    // e correção de rotação.
    // Frente permanece em 0 até ser definida pela estratégia.
    moverOmni(
        0,
        lateral,
        giro
    );


    // =================================================
    // DEBUG SERIAL
    // =================================================

    debugSerial();

}


// =====================================================
// CONTROLE PWM
// =====================================================

void setPWM(
    int pwmA,
    int pwmB,
    int pwmC,
    int pwmD
)
{
    pwmA = constrain(pwmA, 0, 255);
    pwmB = constrain(pwmB, 0, 255);
    pwmC = constrain(pwmC, 0, 255);
    pwmD = constrain(pwmD, 0, 255);

    analogWrite(ENA_1, pwmA);
    analogWrite(ENB_1, pwmB);

    analogWrite(ENA_2, pwmC);
    analogWrite(ENB_2, pwmD);
}


// =====================================================
// LEITURA DA BÚSSOLA
// =====================================================

int lerBussola()
{
    Wire.beginTransmission(COMPASS_ADDRESS);

    Wire.write(0x44);

    if (Wire.endTransmission() != 0)
    {
        Serial.println("ERRO I2C");

        return 0;
    }


    Wire.requestFrom(
        COMPASS_ADDRESS,
        2
    );


    unsigned long timeout = millis();

    while (Wire.available() < 2)
    {
        if (millis() - timeout > 100)
        {
            Serial.println("TIMEOUT BUSSOLA");

            return 0;
        }
    }


    byte lowbyte = Wire.read();
    byte highbyte = Wire.read();


    return word(
        highbyte,
        lowbyte
    );
}


// =====================================================
// BÚSSOLA RELATIVA
// =====================================================

int lerBussolaRelativa()
{
    int leitura = lerBussola();

    int angulo = leitura - BussolaZero;


    while (angulo < 0)
    {
        angulo += 360;
    }


    while (angulo >= 360)
    {
        angulo -= 360;
    }


    return angulo;
}


// =====================================================
// ERRO ANGULAR
// =====================================================

float calcularErroAngular(
    float alvo,
    float atual
)
{
    float erroAngular = alvo - atual;


    while (erroAngular > 180)
    {
        erroAngular -= 360;
    }


    while (erroAngular < -180)
    {
        erroAngular += 360;
    }


    return erroAngular;
}


// =====================================================
// PID
// =====================================================

void calcularPID(float erroAtual)
{
    unsigned long agora = millis();


    float dt =
        (agora - ultimoPID) / 1000.0;


    if (dt <= 0)
    {
        return;
    }


    // Proteção contra intervalo muito grande
    if (dt > 0.5)
    {
        dt = 0.01;
    }


    ultimoPID = agora;


    erro = erroAtual;


    // =================================================
    // TERMO INTEGRAL
    // =================================================

    integral += erro * dt;


    integral = constrain(
        integral,
        -INTEGRAL_MAX,
        INTEGRAL_MAX
    );


    // =================================================
    // TERMO DERIVATIVO
    // =================================================

    derivada =
        (erro - erroAnterior) / dt;


    // =================================================
    // SAÍDA PID
    // =================================================

    saidaPID =
        (Kp * erro) +
        (Ki * integral) +
        (Kd * derivada);


    // =================================================
    // LIMITAÇÃO DA SAÍDA
    // =================================================

    saidaPID = constrain(
        saidaPID,
        -PID_MAX,
        PID_MAX
    );


    erroAnterior = erro;
}


// =====================================================
// MOVIMENTO PARA FRENTE
// =====================================================

void mov_frente()
{
    digitalWrite(IN1_1, LOW);
    digitalWrite(IN2_1, HIGH);

    digitalWrite(IN3_1, HIGH);
    digitalWrite(IN4_1, LOW);

    digitalWrite(IN1_2, HIGH);
    digitalWrite(IN2_2, LOW);

    digitalWrite(IN3_2, LOW);
    digitalWrite(IN4_2, HIGH);
}


// =====================================================
// MOVIMENTO PARA TRÁS
// =====================================================

void mov_tras()
{
    digitalWrite(IN1_1, HIGH);
    digitalWrite(IN2_1, LOW);

    digitalWrite(IN3_1, LOW);
    digitalWrite(IN4_1, HIGH);

    digitalWrite(IN1_2, LOW);
    digitalWrite(IN2_2, HIGH);

    digitalWrite(IN3_2, HIGH);
    digitalWrite(IN4_2, LOW);
}


// =====================================================
// MOVIMENTO PARA ESQUERDA
// =====================================================

void mov_esquerda()
{
    digitalWrite(IN1_1, HIGH);
    digitalWrite(IN2_1, LOW);

    digitalWrite(IN3_1, LOW);
    digitalWrite(IN4_1, HIGH);

    digitalWrite(IN1_2, HIGH);
    digitalWrite(IN2_2, LOW);

    digitalWrite(IN3_2, LOW);
    digitalWrite(IN4_2, HIGH);
}


// =====================================================
// MOVIMENTO PARA DIREITA
// =====================================================

void mov_direita()
{
    digitalWrite(IN1_1, LOW);
    digitalWrite(IN2_1, HIGH);

    digitalWrite(IN3_1, HIGH);
    digitalWrite(IN4_1, LOW);

    digitalWrite(IN1_2, LOW);
    digitalWrite(IN2_2, HIGH);

    digitalWrite(IN3_2, HIGH);
    digitalWrite(IN4_2, LOW);
}


// =====================================================
// GIRO PARA ESQUERDA
// =====================================================

void giro_esquerda()
{
    digitalWrite(IN1_1, HIGH);
    digitalWrite(IN2_1, LOW);

    digitalWrite(IN3_1, HIGH);
    digitalWrite(IN4_1, LOW);

    digitalWrite(IN1_2, HIGH);
    digitalWrite(IN2_2, LOW);

    digitalWrite(IN3_2, HIGH);
    digitalWrite(IN4_2, LOW);
}


// =====================================================
// GIRO PARA DIREITA
// =====================================================

void giro_direita()
{
    digitalWrite(IN1_1, LOW);
    digitalWrite(IN2_1, HIGH);

    digitalWrite(IN3_1, LOW);
    digitalWrite(IN4_1, HIGH);

    digitalWrite(IN1_2, LOW);
    digitalWrite(IN2_2, HIGH);

    digitalWrite(IN3_2, LOW);
    digitalWrite(IN4_2, HIGH);
}


// =====================================================
// PARAR MOTORES
// =====================================================

void parar_motores()
{
    digitalWrite(IN1_1, LOW);
    digitalWrite(IN2_1, LOW);

    digitalWrite(IN3_1, LOW);
    digitalWrite(IN4_1, LOW);

    digitalWrite(IN1_2, LOW);
    digitalWrite(IN2_2, LOW);

    digitalWrite(IN3_2, LOW);
    digitalWrite(IN4_2, LOW);

    setPWM(
        0,
        0,
        0,
        0
    );
}


// =====================================================
// MOVIMENTAÇÃO OMNI
// =====================================================

void moverOmni(
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
    // LIMITAÇÃO
    // =================================================

    A = constrain(
        A,
        -255,
        255
    );

    B = constrain(
        B,
        -255,
        255
    );

    C = constrain(
        C,
        -255,
        255
    );

    D = constrain(
        D,
        -255,
        255
    );


    // =================================================
    // APLICAÇÃO
    // =================================================

    motorA_saida = A;
    motorB_saida = B;
    motorC_saida = C;
    motorD_saida = D;

    motorA.girar(A);
    motorB.girar(B);
    motorC.girar(C);
    motorD.girar(D);
}


// =====================================================
// DEBUG SERIAL
// =====================================================

void debugSerial()
{
    static unsigned long ultimoPrint = 0;

    if (millis() - ultimoPrint < 100)
        return;

    ultimoPrint = millis();

    // Linha compacta e legível para humano.
    // Também foi mantido o padrão "chave=valor" para o Python.
    Serial.print("[DEBUG] ");

    Serial.print("IR: dir=");
    Serial.print(ballDirecao);
    Serial.print(" str=");
    Serial.print(ballIntens);

    Serial.print(" | COMPASS: angle=");
    Serial.print(Bussola);
    Serial.print(" target=0 err=");
    Serial.print(erro, 2);

    Serial.print(" | PID: P=");
    Serial.print(Kp * erro, 2);
    Serial.print(" I=");
    Serial.print(Ki * integral, 2);
    Serial.print(" D=");
    Serial.print(Kd * derivada, 2);
    Serial.print(" OUT=");
    Serial.print(saidaPID, 2);

    Serial.print(" | MOVE: F=0 L=");
    Serial.print(lateral);
    Serial.print(" G=");
    Serial.print(giro);

    Serial.print(" | MOTOR: A=");
    Serial.print(motorA_saida);
    Serial.print(" B=");
    Serial.print(motorB_saida);
    Serial.print(" C=");
    Serial.print(motorC_saida);
    Serial.print(" D=");
    Serial.println(motorD_saida);
}


// =====================================================
// PROCESSAMENTO DA SERIAL
// =====================================================

void processarSerial()
{
    static String buffer = "";

    while (Serial.available())
    {
        char c = Serial.read();

        if (c == '\n')
        {
            buffer.trim();

            if (buffer.length() > 0)
            {
                processarComando(buffer);
            }

            buffer = "";
        }
        else
        {
            buffer += c;
        }
    }
}


// =====================================================
// PROCESSAMENTO DE COMANDOS
// =====================================================

void processarComando(String comando)
{
    comando.trim();

    // =================================================
    // SET PID
    //
    // SET PID 1.80 0.15 0.35
    // =================================================

    if (comando.startsWith("SET PID"))
    {
        String dados = comando.substring(7);
        dados.trim();

        int espaco1 = dados.indexOf(' ');

        if (espaco1 == -1)
        {
            Serial.println(
                "ERRO PID: use SET PID KP KI KD"
            );

            return;
        }

        String textoKp = dados.substring(0, espaco1);
        dados = dados.substring(espaco1 + 1);
        dados.trim();

        int espaco2 = dados.indexOf(' ');

        if (espaco2 == -1)
        {
            Serial.println(
                "ERRO PID: use SET PID KP KI KD"
            );

            return;
        }

        String textoKi = dados.substring(0, espaco2);
        String textoKd = dados.substring(espaco2 + 1);

        textoKi.trim();
        textoKd.trim();

        // -----------------------------------------
        // CONVERTE
        // -----------------------------------------

        float novoKp = textoKp.toFloat();
        float novoKi = textoKi.toFloat();
        float novoKd = textoKd.toFloat();

        // -----------------------------------------
        // ATUALIZA
        // -----------------------------------------

        Kp = max(0.0f, novoKp);
        Ki = max(0.0f, novoKi);
        Kd = max(0.0f, novoKd);

        // -----------------------------------------
        // RESET DO ESTADO
        // -----------------------------------------

        integral = 0.0;

        derivada = 0.0;

        erroAnterior = erro;

        saidaPID = 0.0;

        ultimoPID = millis();

        // -----------------------------------------
        // CONFIRMA
        // -----------------------------------------

        Serial.print("OK PID KP=");
        Serial.print(Kp, 6);

        Serial.print(" KI=");
        Serial.print(Ki, 6);

        Serial.print(" KD=");
        Serial.println(Kd, 6);

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
        resetarPID();

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

        ultimoPID = millis();

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

        saidaPID = 0.0;

        integral = 0.0;

        derivada = 0.0;

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

    Serial.println(comando);
}


// =====================================================
// ENVIAR GANHOS ATUAIS
// =====================================================

void enviarPID()
{
    Serial.print("OK PID KP=");
    Serial.print(Kp, 6);

    Serial.print(" KI=");
    Serial.print(Ki, 6);

    Serial.print(" KD=");
    Serial.println(Kd, 6);
}


// =====================================================
// RESETAR ESTADO DO PID
// =====================================================

void resetarPID()
{
    integral = 0.0;

    derivada = 0.0;

    erroAnterior = erro;

    saidaPID = 0.0;

    ultimoPID = millis();
}