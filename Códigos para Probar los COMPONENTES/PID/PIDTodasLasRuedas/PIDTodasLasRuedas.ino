#include "PinChangeInterrupt.h"

// Definir clase PID
class SimplePID {
private:
    float kp, kd, ki, umax;
    float eprev, eintegral;

public:
    SimplePID() : kp(3.3), kd(0.4), ki(0.1), umax(255), eprev(0.0), eintegral(0.0) {}

    void setParams(float kpIn, float kdIn, float kiIn, float umaxIn) {
        kp = kpIn;
        kd = kdIn;
        ki = kiIn;
        umax = umaxIn;
    }

    void evalu(int value, int target, float deltaT, int &pwr) {
        int e = target - value;
        float dedt = (e - eprev) / deltaT;
        eintegral += e * deltaT;
        float u = kp * e + kd * dedt + ki * eintegral;
        pwr = (int) fabs(u);
        if (pwr > umax) pwr = umax;
        if (pwr < 0) pwr = 0;
        eprev = e;
    }
};

// Definición de pines para encoders y motores
const int ench1 = 5, dirch1 = 7, pwmch1 = 6;
const int ench2 = 4, dirch2 = 2, pwmch2 = 3;
const int ench3 = 8, dirch3 = 9, pwmch3 = 10;
const int ench4 = 13, dirch4 = 12, pwmch4 = 11;

// Variables de conteo de encoders
volatile long encoder1Count = 0, encoder2Count = 0, encoder3Count = 0, encoder4Count = 0;

// Variables de tiempo
volatile unsigned long muestreoAnterior = 0;
unsigned long muestreoActual = 0;
volatile double deltaMuestreo = 0;

// Variables de RPM
volatile float rpm1 = 0, rpm2 = 0, rpm3 = 0, rpm4 = 0;
const float pulsos_por_revolucion = 1390.0;

// Definir instancias del PID para cada motor
SimplePID pid1, pid2, pid3, pid4;

// Setpoints deseados (RPM)
int setpoint1 = 100, setpoint2 = 100, setpoint3 = 100, setpoint4 = 100;

void setup() {
    Serial.begin(115200);

    // Configurar pines de dirección y PWM como salida
    pinMode(dirch1, OUTPUT); pinMode(dirch2, OUTPUT);
    pinMode(dirch3, OUTPUT); pinMode(dirch4, OUTPUT);

    // Configurar pines de encoders como entrada
    pinMode(ench1, INPUT); pinMode(ench2, INPUT);
    pinMode(ench3, INPUT); pinMode(ench4, INPUT);

    // Configurar interrupciones de encoders
    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(ench1), handleEncoder1, FALLING);
    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(ench2), handleEncoder2, FALLING);
    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(ench3), handleEncoder3, FALLING);
    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(ench4), handleEncoder4, FALLING);

    // Configurar parámetros PID para cada motor
    pid1.setParams(2.0, 0.1, 0.5, 255);
    pid2.setParams(2.0, 0.1, 0.5, 255);
    pid3.setParams(2.0, 0.1, 0.5, 255);
    pid4.setParams(2.0, 0.1, 0.5, 255);

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

    muestreoActual = millis();
    deltaMuestreo = (double)(muestreoActual - muestreoAnterior) / 1000.0;

    // Cada 100 ms, calcular RPM y aplicar PID
    if (deltaMuestreo >= 0.1) {
        muestreoAnterior = muestreoActual;

        // Cálculo de RPM
        rpm1 = 10 * encoder1Count * (60.0 / pulsos_por_revolucion);
        rpm2 = 10 * encoder2Count * (60.0 / pulsos_por_revolucion);
        rpm3 = 10 * encoder3Count * (60.0 / pulsos_por_revolucion);
        rpm4 = 10 * encoder4Count * (60.0 / pulsos_por_revolucion);

        // Reset del contador
        encoder1Count = 0;
        encoder2Count = 0;
        encoder3Count = 0;
        encoder4Count = 0;

        // Variables de salida de PID (PWM)
        int pwm1, pwm2, pwm3, pwm4;
        
        // Evaluar PID para cada motor
        pid1.evalu(rpm1, setpoint1, deltaMuestreo, pwm1);
        pid2.evalu(rpm2, setpoint2, deltaMuestreo, pwm2);
        pid3.evalu(rpm3, setpoint3, deltaMuestreo, pwm3);
        pid4.evalu(rpm4, setpoint4, deltaMuestreo, pwm4);

        // Aplicar PWM a los motores
        analogWrite(pwmch1, pwm1);
        analogWrite(pwmch2, pwm2);
        analogWrite(pwmch3, pwm3);
        analogWrite(pwmch4, pwm4);

        // Imprimir valores
        Serial.print("RPM 1: "); Serial.print(rpm1);
        Serial.print(" | RPM 2: "); Serial.print(rpm2);
        Serial.print(" | RPM 3: "); Serial.print(rpm3);
        Serial.print(" | RPM 4: "); Serial.println(rpm4);
    }
}

// Funciones de interrupción
void handleEncoder1() { encoder1Count++; }
void handleEncoder2() { encoder2Count++; }
void handleEncoder3() { encoder3Count++; }
void handleEncoder4() { encoder4Count++; }
