#include <Arduino.h>
#include <Wire.h>
#include <HTInfraredSeeker.h>
#include <math.h>

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
// LEITURA QRE
// =====================================================

const int VERDE_1 = 910;
const int VERDE_2 = 930;
const int VERDE_3 = 940; 
const int VERDE_4 = 910;

const int BRANCO_1 = 740; 
const int BRANCO_2 = 735;
const int BRANCO_3 = 880;
const int BRANCO_4 = 865;

const int LIMIAR_1 = ceil((VERDE_1 + BRANCO_1)/2);
const int LIMIAR_2 = ceil((VERDE_2 + BRANCO_2)/2);
const int LIMIAR_3 = ceil((VERDE_3 + BRANCO_3)/2);
const int LIMIAR_4 = ceil((VERDE_4 + BRANCO_4)/2);

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