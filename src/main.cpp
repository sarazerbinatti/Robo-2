#include <Arduino.h>
#include <Wire.h>
#include <HTInfraredSeeker.h>

// =====================================================
// PINOS PWM
// =====================================================

const int ENA_1 = 2; // Motor A | Cabo Laranja
const int ENB_1 = 3; // Motor B | Cabo Branco
const int ENA_2 = 8; // Motor C | Cabo Cinza
const int ENB_2 = 9; // Motor D | Cabo Branco

// =====================================================
// PINOS DIREÇÃO
// =====================================================

const int IN1_1 = 4;   // Cabo Verde
const int IN2_1 = 5;   // Cabo Azul

const int IN3_1 = 6;   // Cabo Roxo
const int IN4_1 = 7;   // Cabo Marrom

const int IN1_2 = 10;  // Cabo Laranja
const int IN2_2 = 11;  // Cabo Vermelho

const int IN3_2 = 12;  // Cabo Preto
const int IN4_2 = 13;  // Cabo Roxo

// =====================================================
// QRE
// > 900 = verde
// < 900 = branco
// =====================================================

const int QRE_1 = A1;
const int QRE_2 = A2;
const int QRE_3 = A3;
const int QRE_4 = A4;

// =====================================================
// LIMIAR QRE
// =====================================================

const int LIMIAR_1 = 800;
const int LIMIAR_2 = 855;
const int LIMIAR_3 = 850;
const int LIMIAR_4 = 875;

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

int compassAddress = 0x01;
int Bussola;
int BussolaZero;
bool BussolaInicializada = false;

// =====================================================
// PID
// =====================================================


float Kp = 1.7;
float Ki = 0;
float Kd = 0.5;

float erro = 0;
float erroAnterior = 0;

float integral = 0;
float derivada = 0;

float correcao = 0;

// =====================================================
// SENSOR IR
// =====================================================

int ballDirecao;
int ballIntens;

// =====================================================
// PROTÓTIPOS
// =====================================================


void mov_frente();
void mov_tras();
void mov_esquerda();
void mov_direita();
void giro_esquerda();
void giro_direita();
void parar_motores();

void setPWM(int pwmA, int pwmB, int pwmC, int pwmD);

int lerBussola();
int lerBussolaRelativa();
float calcularErroAngular(float alvo, float atual);

// =====================================================
// SETUP
// =====================================================

void setup()
{
    Serial.begin(115200);

    // ================= DIREÇÃO =================

    pinMode(IN1_1, OUTPUT);
    pinMode(IN2_1, OUTPUT);

    pinMode(IN3_1, OUTPUT);
    pinMode(IN4_1, OUTPUT);

    pinMode(IN1_2, OUTPUT);
    pinMode(IN2_2, OUTPUT);

    pinMode(IN3_2, OUTPUT);
    pinMode(IN4_2, OUTPUT);

    // ================= PWM =================

    pinMode(ENA_1, OUTPUT);
    pinMode(ENB_1, OUTPUT);

    pinMode(ENA_2, OUTPUT);
    pinMode(ENB_2, OUTPUT);

    // ================= QRE =================

    pinMode(QRE_1, INPUT);
    pinMode(QRE_2, INPUT);
    pinMode(QRE_3, INPUT);
    pinMode(QRE_4, INPUT);

    // ================= IR =================

    InfraredSeeker::Initialize();

    delay(500);

    BussolaZero = lerBussola();
    BussolaInicializada = true;

    Serial.print("ZERO = ");
    Serial.println(BussolaZero);

    parar_motores();

    Serial.println("ROBO INICIADO");
}

// =====================================================
// LOOP
// =====================================================

void loop()
{ 
    // ==========================
    // LEITURA DOS QRE
    // ==========================
    
    int q1 = analogRead(QRE_1);
    int q2 = analogRead(QRE_2);
    int q3 = analogRead(QRE_3);
    int q4 = analogRead(QRE_4);
    
    // ==========================
    // DEBUG SERIAL (100 ms)
    // ==========================
    
    static unsigned long ultimoPrint = 0;
    
    if (millis() - ultimoPrint >= 100)
    {
        ultimoPrint = millis();
    
        Serial.print("Q1: ");
        Serial.print(q1);
    
        Serial.print(" | Q2: ");
        Serial.print(q2);
    
        Serial.print(" | Q3: ");
        Serial.print(q3);
    
        Serial.print(" | Q4: ");
        Serial.println(q4);
    }
    
    // ==========================
    // TESTE DOS SENSORES
    // ==========================
    
    if (q1 < LIMIAR_1)
    {
        setPWM(150, 150, 150, 150);
        mov_tras();
        delay(150);
        return;
    }
    
    if (q2 < LIMIAR_2)
    {
        setPWM(150, 150, 150, 150);
        mov_esquerda();
        delay(150);
        return;
    }
    
    if (q3 < LIMIAR_3)
    {
        setPWM(150, 150, 150, 150);
        mov_frente();
        delay(150);
        return;
    }
    
    if (q4 < LIMIAR_4)
    {
        setPWM(150, 150, 150, 150);
        mov_direita();
        delay(150);
        return;
    }

    // ================================================
    // BOLA PERDIDA
    // ================================================

    InfraredResult InfraredBall = InfraredSeeker::ReadAC();

    ballDirecao = InfraredBall.Direction;
    ballIntens = InfraredBall.Strength;

    Serial.print("Dir: ");
    Serial.print(ballDirecao);
    Serial.print(" | Int: ");
    Serial.println(ballIntens);

    // =================================================
    // BOLA PERDIDA
    // =================================================

    if (ballDirecao == 0 || ballIntens < 15)
    {
        parar_motores();
        return;
    }

    // switch(ballDirecao)
    // {

    // case 7:
    //     setPWM(120,120,120,120);
    //     mov_direita();
    //     break;

    // case 6:
    //     setPWM(90,90,90,90);
    //     mov_direita();
    //     break;

    // case 5:
    //     parar_motores();
    //     break;

    // case 4:
    //     setPWM(90,90,90,90);
    //     mov_esquerda();
    //     break;

    // case 3:
    //     setPWM(120,120,120,120);
    //     mov_esquerda();
    //     break;
    // }

}

// =====================================================
// LEITURA DA BÚSSOLA
// =====================================================

int lerBussola()
{
    Wire.beginTransmission(compassAddress);
    Wire.write(0x44);

    if (Wire.endTransmission() != 0)
    {
        Serial.println("ERRO I2C");
        return 0;
    }

    Wire.requestFrom(compassAddress, 2);

    unsigned long timeout = millis();

    while (Wire.available() < 2)
    {
        if (millis() - timeout > 100)
        {
            Serial.println("TIMEOUT BUSSOLA");
            return 0;
        }
    }

    byte lowbyte  = Wire.read();
    byte highbyte = Wire.read();

    return word(highbyte, lowbyte);
}

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

float calcularErroAngular(float alvo, float atual)
{
    float erro = alvo - atual;

    while (erro > 180)
    {
        erro -= 360;
    }

    while (erro < -180)
    {
        erro += 360;
    }

    return erro;
}

// =====================================================
// PWM
// =====================================================

void setPWM(int pwmA, int pwmB, int pwmC, int pwmD)
{
    analogWrite(ENA_1, pwmA);
    analogWrite(ENB_1, pwmB);

    analogWrite(ENA_2, pwmC);
    analogWrite(ENB_2, pwmD);
}

// =====================================================
// MOVIMENTOS
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

    setPWM(0, 0, 0, 0);
}