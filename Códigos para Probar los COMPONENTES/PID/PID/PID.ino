#include "PinChangeInterrupt.h"

// Definir clase PID
class SimplePID {
private:
    float kp, kd, ki, umax;
    float eprev, eintegral;

public:
    SimplePID() : kp(3.5), kd(0), ki(0.35), umax(255), eprev(0.0), eintegral(0.0) {}

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

// Definición de pines para un solo motor
const int ench1 = 13;  // Encoder motor
const int dirch1 = 12;  // Dirección motor
const int pwmch1 = 11;  // PWM motor

// Definición de pines para encoders y motores cambiarlo antes
/*
const int ench1 = 5, dirch1 = 7, pwmch1 = 6; delantero izquierdo
  kp(3.1), kd(0.75), ki(0.4)
const int ench2 = 4, dirch2 = 2, pwmch2 = 3;delantero derecho
   kp(3.1), kd(0.75), ki(0.4)
    kp(5), kd(0.15), ki(0.4)
    kp(4.8), kd(0.1), ki(0.35)
const int ench3 = 8, dirch3 = 9, pwmch3 = 10; trasero izquierdo
    kp(4.8), kd(0), ki(0.35)
const int ench4 = 13, dirch4 = 12, pwmch4 = 11; trasero derecho
    kp(3.5), kd(0), ki(0.35)

*/

// Variables de conteo de encoder
volatile long encoder1Count = 0;

// Variables de tiempo
volatile unsigned long muestreoAnterior = 0;
unsigned long muestreoActual = 0;
volatile double deltaMuestreo = 0;

// Variables de RPM
volatile float rpm1 = 0;
const float pulsos_por_revolucion = 1390.0;  // Ajustar según el encoder

// Definir instancia PID para el motor
SimplePID pid1;

// Setpoint deseado (RPM)
int setpoint1 = 100;

void setup() {
    Serial.begin(115200);

    // Configurar pines de dirección y PWM como salida
    pinMode(dirch1, OUTPUT);
    pinMode(pwmch1, OUTPUT);

    // Configurar pin de encoder como entrada
    pinMode(ench1, INPUT);

    // Configurar interrupción para el encoder
    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(ench1), handleEncoder1, FALLING);

    // Configurar parámetros PID para el motor
    pid1.setParams(2.0, 0.1, 0.5, 255);

    delay(2000);
    Serial.println("Setup terminado");
    muestreoAnterior = millis();
}

void loop() {
    // Configurar dirección del motor
    digitalWrite(dirch1, LOW);

    muestreoActual = millis();
    deltaMuestreo = (double)(muestreoActual - muestreoAnterior) / 1000.0;

    // Cada 100 ms, calcular RPM y aplicar PID
    if (deltaMuestreo >= 0.1) {
        muestreoAnterior = muestreoActual;

        // Cálculo de RPM
        rpm1 = 10 * encoder1Count * (60.0 / pulsos_por_revolucion);

        // Reset del contador
        encoder1Count = 0;

        // Variable de salida PID (PWM)
        int pwm1;
        
        // Evaluar PID
        pid1.evalu(rpm1, setpoint1, deltaMuestreo, pwm1);

        // Aplicar PWM al motor
        analogWrite(pwmch1, pwm1);

        // Imprimir valores
        Serial.print("RPM: "); Serial.print(rpm1);
        Serial.print(" | PWM: "); Serial.println(pwm1);
    }
}

// Función de interrupción del encoder
void handleEncoder1() {
    encoder1Count++;
}
