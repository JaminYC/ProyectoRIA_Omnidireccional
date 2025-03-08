#include "PinChangeInterrupt.h"

// Definición de pines para los encoders
const int ench1 = 5;  // Encoder motor 1
const int ench2 = 4;  // Encoder motor 2
const int ench3 = 8;  // Encoder motor 3
const int ench4 = 13; // Encoder motor 4

// Definición de pines para la dirección de motores
const int dirch1 = 7;
const int dirch2 = 2;
const int dirch3 = 9;
const int dirch4 = 12;

// Definición de pines para PWM
const int pwmch1 = 6;
const int pwmch2 = 3;
const int pwmch3 = 10;
const int pwmch4 = 11;

// Variables de conteo de encoders
volatile long encoder1Count = 0;
volatile long encoder2Count = 0;
volatile long encoder3Count = 0;
volatile long encoder4Count = 0;

// Variables de tiempo
volatile unsigned long muestreoAnterior = 0;
unsigned long muestreoActual = 0;
volatile double deltaMuestreo = 0;

// Variables de RPM
volatile float rpm1 = 0, rpm2 = 0, rpm3 = 0, rpm4 = 0;
const float pulsos_por_revolucion = 1390.0;  // Ajusta este valor según el encoder

void setup() {
    Serial.begin(115200);

    // Configurar pines de dirección y PWM como salida
    pinMode(dirch1, OUTPUT);
    pinMode(dirch2, OUTPUT);
    pinMode(dirch3, OUTPUT);
    pinMode(dirch4, OUTPUT);

    // Configurar pines de los encoders como entrada
    pinMode(ench1, INPUT);
    pinMode(ench2, INPUT);
    pinMode(ench3, INPUT);
    pinMode(ench4, INPUT);

    // Configurar interrupciones para cada encoder
    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(ench1), handleEncoder1, FALLING);
    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(ench2), handleEncoder2, FALLING);
    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(ench3), handleEncoder3, FALLING);
    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(ench4), handleEncoder4, FALLING);

    // Iniciar motores apagados
    analogWrite(pwmch1, 0);
    analogWrite(pwmch2, 0);
    analogWrite(pwmch3, 0);
    analogWrite(pwmch4, 0);

    delay(2000);
    Serial.println("Setup terminado");
    muestreoAnterior = millis();
}

void loop() {
    // Configurar dirección de motores (puedes cambiar LOW/HIGH según la dirección deseada)
    digitalWrite(dirch1, LOW);
    digitalWrite(dirch2, LOW);
    digitalWrite(dirch3, LOW);
    digitalWrite(dirch4, LOW);

    // Ajustar velocidad de los motores (de 0 a 255)
    analogWrite(pwmch1, 255);
    analogWrite(pwmch2, 255);
    analogWrite(pwmch3, 255);
    analogWrite(pwmch4, 255);

    muestreoActual = millis();
    deltaMuestreo = (double)(muestreoActual - muestreoAnterior);

    // Cada 100 ms, calcular RPM
    if (deltaMuestreo >= 100) {
        muestreoAnterior = muestreoActual;

        rpm1 = 10 * encoder1Count * (60.0 / pulsos_por_revolucion);
        rpm2 = 10 * encoder2Count * (60.0 / pulsos_por_revolucion);
        rpm3 = 10 * encoder3Count * (60.0 / pulsos_por_revolucion);
        rpm4 = 10 * encoder4Count * (60.0 / pulsos_por_revolucion);

        // Reset del contador
        encoder1Count = 0;
        encoder2Count = 0;
        encoder3Count = 0;
        encoder4Count = 0;
    }

    // Mostrar valores cada 100 ms
    if (millis() % 100 == 0) {
        Serial.print("RPM 1: "); Serial.print(rpm1);
        Serial.print(" | RPM 2: "); Serial.print(rpm2);
        Serial.print(" | RPM 3: "); Serial.print(rpm3);
        Serial.print(" | RPM 4: "); Serial.println(rpm4);
    }
}

// Funciones de interrupción para los encoders
void handleEncoder1() { encoder1Count++; }
void handleEncoder2() { encoder2Count++; }
void handleEncoder3() { encoder3Count++; }
void handleEncoder4() { encoder4Count++; }
