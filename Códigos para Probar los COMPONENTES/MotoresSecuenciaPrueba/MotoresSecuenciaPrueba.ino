#include "PinChangeInterrupt.h"

// Definir clase PID
class SimplePID {
private:
    float kp, kd, ki, umax;
    float eprev, eintegral;

public:
    SimplePID() : kp(3.1), kd(0.75), ki(0.4), umax(255), eprev(0.0), eintegral(0.0) {}

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

// Definición de pines para los motores y encoders
const int ench1 = 5, dirch1 = 7, pwmch1 = 6;
const int ench2 = 4, dirch2 = 2, pwmch2 = 3;
const int ench3 = 8, dirch3 = 9, pwmch3 = 10;
const int ench4 = 13, dirch4 = 12, pwmch4 = 11;

// Variables de conteo de encoders
volatile long encoder1Count = 0, encoder2Count = 0, encoder3Count = 0, encoder4Count = 0;

// Variables de RPM
volatile float rpm1 = 0, rpm2 = 0, rpm3 = 0, rpm4 = 0;
const float pulsos_por_revolucion = 1390.0;

// Instancias de PID para cada motor
SimplePID pid1, pid2, pid3, pid4;

// Setpoint deseado (RPM)
int setpoint1 = 30, setpoint2 = 30, setpoint3 = 30, setpoint4 = 30;

// Variables de tiempo
unsigned long muestreoAnterior = 0, muestreoActual = 0;
double deltaMuestreo = 0;

void setup() {
    Serial.begin(115200);

    // Configurar pines de dirección y PWM como salida
    pinMode(dirch1, OUTPUT); pinMode(pwmch1, OUTPUT);
    pinMode(dirch2, OUTPUT); pinMode(pwmch2, OUTPUT);
    pinMode(dirch3, OUTPUT); pinMode(pwmch3, OUTPUT);
    pinMode(dirch4, OUTPUT); pinMode(pwmch4, OUTPUT);

    // Configurar pines de encoders como entrada
    pinMode(ench1, INPUT); pinMode(ench2, INPUT);
    pinMode(ench3, INPUT); pinMode(ench4, INPUT);

    // Configurar interrupciones para encoders
    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(ench1), handleEncoder1, FALLING);
    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(ench2), handleEncoder2, FALLING);
    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(ench3), handleEncoder3, FALLING);
    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(ench4), handleEncoder4, FALLING);

    // Inicializar motores apagados
    detenerCarro();
}

// Función para detener todos los motores
void detenerCarro() {
    analogWrite(pwmch1, 0);
    analogWrite(pwmch2, 0);
    analogWrite(pwmch3, 0);
    analogWrite(pwmch4, 0);
}

// Función para calcular RPM y aplicar control PID
void actualizarControlPID() {
    muestreoActual = millis();
    deltaMuestreo = (double)(muestreoActual - muestreoAnterior) / 1000.0;  // Convertir a segundos

    if (deltaMuestreo >= 0.1) {  // Cada 100ms
        muestreoAnterior = muestreoActual;

        // Calcular RPM de cada motor
        rpm1 = 10 * encoder1Count * (60.0 / pulsos_por_revolucion);
        rpm2 = 10 * encoder2Count * (60.0 / pulsos_por_revolucion);
        rpm3 = 10 * encoder3Count * (60.0 / pulsos_por_revolucion);
        rpm4 = 10 * encoder4Count * (60.0 / pulsos_por_revolucion);

        // Reset del contador
        encoder1Count = 0;
        encoder2Count = 0;
        encoder3Count = 0;
        encoder4Count = 0;

        // Aplicar control PID
        int pwm1, pwm2, pwm3, pwm4;
        pid1.evalu(rpm1, setpoint1, deltaMuestreo, pwm1);
        pid2.evalu(rpm2, setpoint2, deltaMuestreo, pwm2);
        pid3.evalu(rpm3, setpoint3, deltaMuestreo, pwm3);
        pid4.evalu(rpm4, setpoint4, deltaMuestreo, pwm4);

        // Aplicar PWM a los motores
        analogWrite(pwmch1, pwm1);
        analogWrite(pwmch2, pwm2);
        analogWrite(pwmch3, pwm3);
        analogWrite(pwmch4, pwm4);

        // Imprimir valores en Serial
        Serial.print("RPM1: "); Serial.print(rpm1);
        Serial.print(" | RPM2: "); Serial.print(rpm2);
        Serial.print(" | RPM3: "); Serial.print(rpm3);
        Serial.print(" | RPM4: "); Serial.println(rpm4);
    }
}

// Función para mover el carro usando PID
void moverCarro(int dir1, int dir2, int dir3, int dir4, int tiempo) {
    digitalWrite(dirch1, dir1);
    digitalWrite(dirch2, dir2);
    digitalWrite(dirch3, dir3);
    digitalWrite(dirch4, dir4);

    delay(tiempo);
    detenerCarro();
    delay(300);
}
// Función mejorada para mover el carro, permitiendo detener ruedas específicas
void moverCarroIndividual(int dir1, int vel1, int dir2, int vel2, int dir3, int vel3, int dir4, int vel4, int tiempo) {
    digitalWrite(dirch1, dir1);
    analogWrite(pwmch1, vel1);

    digitalWrite(dirch2, dir2);
    analogWrite(pwmch2, vel2);

    digitalWrite(dirch3, dir3);
    analogWrite(pwmch3, vel3);

    digitalWrite(dirch4, dir4);
    analogWrite(pwmch4, vel4);

    delay(tiempo);
    detenerCarro();  // Detiene todos los motores después del movimiento
    delay(300);      // Pequeño tiempo de espera antes del siguiente comando
}


void loop() {
    int tiempo = 1000;  // Duración de cada movimiento

    // Adelante
    Serial.println("Moviendo adelante");
    moverCarro(1, 0, 1, 0, tiempo);
    actualizarControlPID();

    // Atrás
    Serial.println("Moviendo atrás");
    moverCarro(0, 1, 0, 1, tiempo);
    actualizarControlPID();

    // Izquierda
    Serial.println("Moviendo izquierda");
    moverCarro(0, 0, 1, 1, tiempo);
    actualizarControlPID();

    // Derecha
    Serial.println("Moviendo derecha");
    moverCarro(1, 1, 0, 0, tiempo);
    actualizarControlPID();

    // Rotación en sentido horario
    Serial.println("Rotando horario");
    moverCarro(1, 1, 1, 1, tiempo);
    actualizarControlPID();

    // Rotación en sentido antihorario
    Serial.println("Rotando antihorario");
    moverCarro(0, 0, 0, 0, tiempo);
    actualizarControlPID();

    // Diagonal izquierda adelante (Solo motores 2 y 3 activos)
    Serial.println("Moviendo diagonal izquierda adelante");
    moverCarroIndividual(0, 0, 0, 255, 1, 255, 0, 0, 1000);
    actualizarControlPID();

    // Diagonal derecha adelante (Solo motores 1 y 4 activos)
    Serial.println("Moviendo diagonal derecha adelante");
    moverCarroIndividual(1, 255, 0, 0, 0, 0, 0, 255, 1000);
    actualizarControlPID();

    // Diagonal izquierda atrás (Solo motores 2 y 3 activos en reversa)
    Serial.println("Moviendo diagonal izquierda atrás");
    moverCarroIndividual(0, 0, 1, 255, 0, 255, 0, 0, 1000);
    actualizarControlPID();

    // Diagonal derecha atrás (Solo motores 1 y 4 activos en reversa)
    Serial.println("Moviendo diagonal derecha atrás");
    moverCarroIndividual(0, 255, 0, 0, 0, 0, 1, 255, 1000);
    actualizarControlPID();


}

// Funciones de interrupción de encoders
void handleEncoder1() { encoder1Count++; }
void handleEncoder2() { encoder2Count++; }
void handleEncoder3() { encoder3Count++; }
void handleEncoder4() { encoder4Count++; }
