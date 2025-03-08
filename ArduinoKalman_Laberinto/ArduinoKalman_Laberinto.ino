#include <util/atomic.h>
#include "PinChangeInterrupt.h"
#include <math.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "RobotControl.h"
/*********************************************************************/
/****************************   CLASSES   ****************************/
/*********************************************************************/

class KalmanFilter {
  private:
    double x_hat, P;
    double Q, R;
  public:
    KalmanFilter(double Q, double R) : Q(Q), R(R), x_hat(0), P(1) {}
    double update(double measurement) {
        P += Q;
        double K = P / (P + R);
        x_hat += K * (measurement - x_hat);
        P *= (1 - K);
        return x_hat;
    }
};
  // Instancias del filtro de Kalman
  KalmanFilter kalmanTheta(0.005, 0.02);
  KalmanFilter kalmanX(0.01, 0.05);
  KalmanFilter kalmanY(0.01, 0.05);
class SimplePID {
  private:
    float kp, kd, ki, umax, vmin;
    float eprev, eintegral;
  public:
    SimplePID() : kp(1), kd(0), ki(0), umax(255), eprev(0.0), eintegral(0.0), vmin(15.0) {}
    void setParams(float kpIn, float kdIn, float kiIn, float umaxIn, float vminIn) {
        kp = kpIn; kd = kdIn; ki = kiIn; umax = umaxIn; vmin = vminIn;
    }
    void evalu(int value, int target, float deltaT, int &pwr) {
        float e = (target - value) * ((float) fabs(target) > vmin);
        float dedt = (e - eprev) / (deltaT) * ((float) fabs(target) > vmin);
        eintegral = (eintegral + e * deltaT) * ((float) fabs(target) > vmin);
        if (umax / ki < eintegral) eintegral = umax / ki;
        if (-umax / ki > eintegral) eintegral = -umax / ki;
        float u = kp * e + kd * dedt + ki * eintegral;
        pwr = (int) fabs(u);
        if (pwr > umax) pwr = umax;
        if (pwr < 0) pwr = 0;
        eprev = e;
    }
};
/*********************************************************************/
/***************************   VARIABLES   **************************/
/*********************************************************************/

// Número de motores
#define NMOTORS 4
const int enc[] = {4, 5, 8, 13};
const int DIR[] = {2, 7, 9, 12};
const int pwm[] = {3, 6, 10, 11};

// Globales
int posPrev[] = {0, 0, 0, 0};
float vel[] = {0.0, 0.0, 0.0, 0.0};
float vt[] = {0.0, 0.0, 0.0, 0.0};
float vfil[] = {0.0, 0.0, 0.0, 0.0};
int dir[] = {0, 0, 0, 0};
long prevT = 0;

// PPR de cada motor
const double ppr[] = {1390, 1390, 1390, 1390};

// Dimensiones del robot
const double a_b = 0.2025, R = 0.04;
const double l_a_b = 1 / 0.2025;
const double  pprobot = 1390; //1390 - 680

RobotControl robot(a_b, R, pprobot);
// Ruta predefinida
#define MAX_PUNTOS 35
const double ruta[MAX_PUNTOS][3] = {
{0.000, 0.000, 0},
{1.600, 0.000, 0},
{1.600, 0.200, 0},
{1.600, 0.400, 0},
{1.400, 0.400, 0},
{1.200, 0.400, 0},
{1.200, 0.600, 0},
{1.200, 0.800, 0},
{1.000, 0.800, 0},
{0.800, 0.800, 0},
{0.800, 1.000, 0},
{0.800, 1.200, 0},
{0.600, 1.200, 0},
{0.400, 1.200, 0},
{0.400, 1.400, 0},
{0.400, 1.600, 0},
{0.200, 1.600, 0},
{0.000, 1.600, 0},
{0.000, 1.800, 0},
{0.000, 2.000, 0},
{-0.200, 2.000, 0},
{-0.400, 2.000, 0},
{-0.400, 2.200, 0},
{-0.400, 2.400, 0},
{0.400, 2.400, 0},
{0.400, 2.200, 0},
{0.400, 2.000, 0},
{0.600, 2.000, 0},
{0.800, 2.000, 0},
{0.800, 1.800, 0},
{0.800, 1.600, 0},
{1.000, 1.600, 0},
{1.200, 1.600, 0},
{1.200, 1.800, 0},
{1.200, 2.000, 0}
};




// Variables de posición y orientación
int punto_actual = 0;
double x = 0, y = 0, theta = 0;
double vx = 0, vy = 0, vw = 0;
double xPrev = 0, yPrev = 0, thetaPrev = 0; 
double gz_offset = 0;

float heading = 0;
// Sensor IMU
MPU6050 sensor;
int gx, gy, gz;
long tiempo_prev, dt;
float gz_bias = 0;
// Velocity limits
double vminLim = 15.0;
// Instancias de PID
SimplePID pid[NMOTORS];

// Variables de encoders y velocidades
int velEnc[NMOTORS] = {0, 0, 0, 0};
int velEncSlack[NMOTORS] = {0, 0, 0, 0};
float velAng[NMOTORS] = {0.0, 0.0, 0.0, 0.0};

// Variables de dirección
int sgn[NMOTORS] = {1, 1, 1, 1};
int sgnPrev[NMOTORS] = {1, 1, 1, 1};

// Variables de control de la secuencia
int secuencia = 1;  // Estado actual del movimiento
double sampleT = 0.1;  // Tiempo de muestreo en segundos
double errorTheta = PI;


/*********************************************************************/
/*****************************   SETUP   *****************************/
/*********************************************************************/

void setup() {
    Serial.begin(9600);
    Wire.begin();
    sensor.initialize();
    if (sensor.testConnection()) Serial.println("✅ Sensor IMU iniciado correctamente.");
    else Serial.println("❌ Error al iniciar el sensor.");
    for (int k = 0; k < NMOTORS; k++) {
        pinMode(enc[k], INPUT);
        pinMode(pwm[k], OUTPUT);
        pinMode(DIR[k], OUTPUT);
    }
    // PID gains for each motor
    pid[0].setParams(2.4, 0.05, 0.12, 255, vminLim);
    pid[1].setParams(2.4, 0.05, 0.12, 255, vminLim);
    pid[2].setParams(2.4, 0.05, 0.12, 255, vminLim);
    pid[3].setParams(2.4, 0.05, 0.12, 255, vminLim);
    // Activate interrupts
    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(enc[0]), readEncoder<0>, CHANGE);
    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(enc[1]), readEncoder<1>, CHANGE);
    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(enc[2]), readEncoder<2>, CHANGE);
    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(enc[3]), readEncoder<3>, CHANGE);
    delay(2000);
    calibrateIMU();
  
    tiempo_prev = millis();
    Serial.println("🚀 Iniciando secuencia de movimiento...");
}

void calibrateIMU() {
    float sum_gz = 0;
    int samples = 500;
    Serial.println("Calibrando IMU...");
    for (int i = 0; i < samples; i++) {
        sensor.getRotation(&gx, &gy, &gz);
        sum_gz += gz * (250.0 / 32768.0);
        delay(2);
    }
    gz_bias = sum_gz / samples;
    Serial.print("Bias del giroscopio (gz_bias): ");
    Serial.println(gz_bias);
}

void loop() {
    // Obtener la diferencia de tiempo
    long currT = micros();
    float deltaT = ((float)(currT - prevT)) / (1.0e6);
    prevT = currT;


    // Leer datos del giroscopio del IMU
      sensor.getRotation(&gx, &gy, &gz);
      float gz_rad_s = (gz * (250.0 / 32768.0)) - gz_bias;
      gz_rad_s *= (PI / 180.0);
      theta = robot.thetaValueIMU(theta, gz_rad_s, deltaT);
      theta = kalmanTheta.update(theta);

    // Calcular el ángulo de orientación del robot
    dt = millis() - tiempo_prev;
    tiempo_prev = millis();
 
    // Normalización del ángulo para evitar valores fuera de -PI a PI
    if (theta >= PI) theta -= 2 * PI;
    else if (theta <= -PI) theta += 2 * PI;

    // Ejecutar el control cada "sampleT" segundos
    if (sampleT <= deltaT) {
        // Leer encoders con interrupciones deshabilitadas
        noInterrupts();
        for (int k = 0; k < NMOTORS; k++) {
            velEncSlack[k] = velEnc[k];
            velEnc[k] = 0;
        }
        interrupts();
        robot.updateVAngValue(velEncSlack, deltaT, velAng, vel);
        // Verificar si hay más puntos en la secuencia
        if (punto_actual < MAX_PUNTOS) {
            // Obtener el punto objetivo actual

              x   = robot.xValueBilineal(deltaT, x, theta, velAng);
              y   = robot.yValueBilineal(deltaT, y, theta, velAng);
            Serial.print("📍 Posición actual -> X: "); Serial.print(x, 4);
            Serial.print(" | Y: "); Serial.print(y, 4);
            Serial.print(" | Theta: "); Serial.println(theta, 4);
            double xGoal = ruta[punto_actual][0];
            double yGoal = ruta[punto_actual][1];
              // Print estimated pose
              Serial.print("X = ");
              Serial.print(x*1000);
              Serial.print(" (mm), ");
              Serial.print("Y = ");
              Serial.print(y*1000);
              Serial.print(" (mm), ");
              Serial.print("theta = ");
              Serial.print(theta*180/PI);
              Serial.println(" (deg)");
               Serial.println("%%%%%%%%%%%%%%%%%%%%%%%%%%");
            // Calcular el error de posición y orientación
            CalculatePositionError(xGoal, yGoal, vx, vy, vw);

            // Comprobar si se ha alcanzado el punto objetivo
            if (fabs(xGoal - x) < 0.05 && fabs(yGoal - y) < 0.05) {
                Serial.print("✅ Punto alcanzado (");
                Serial.print(punto_actual);
                Serial.print("): ");
                Serial.print(xGoal, 2);
                Serial.print(", ");
                Serial.println(yGoal, 2);
                
                punto_actual++;  // Avanzar en la ruta
                if (punto_actual >= MAX_PUNTOS) {
                    Serial.println("🏁 Secuencia finalizada. Deteniendo motores.");
                    StopMotors();
                }
            }

        } else {
            Serial.println("🏁 Secuencia finalizada. Deteniendo motores.");
            StopMotors();
        }

        // Calcular velocidades para los motores
        robot.CalculateVelAngGoal(vx, vy, vw, dir, vt);
        for (int k = 0; k < NMOTORS; k++) {
            int pwr;
            pid[k].evalu(vel[k], vt[k], deltaT, pwr);
            setMotor(dir[k], pwr, pwm[k], DIR[k]);
        }
        Serial.print("🔄 Contador de encoders: ");
          for (int i = 0; i < NMOTORS; i++) {
              Serial.print("M");
              Serial.print(i);
              Serial.print(": ");
              Serial.print(velEnc[i]);
              Serial.print(" ");
          }
          Serial.println();

    }
}

/******************************************************************************************************************************/
/************************************************   FUNCTIONS   ***************************************************************/
/******************************************************************************************************************************/

void CalculatePositionError(double xGoal_l, double yGoal_l, double &vx, double &vy, double &vw){
    // Error en posición
    double errorX = xGoal_l - x;
    double errorY = yGoal_l - y;
    
    // Probar valores de K y usar el que dé un buen resultado
    double k=2;

    // Calcular la velocidad global en función del error y la constante k en cada eje
    double vx_global = errorX*k;
    double vy_global = errorY*k;

    double ct = cos(theta); 
    double st = sin(theta);

    // No hay rotación
    vw = 0;

    // Calcular la velocidad relativa para el robot: vx y vy
    vx = ct*vx_global+st*vy_global;
    vy = -st*vx_global+ct*vy_global;

}

void CalculateOrientationError(double thetaGoal_l, double &vx, double &vy, double &vw, double &errorTheta){
    // Error de orientación
    errorTheta = thetaGoal_l - theta;

    //No modificar el condicional
    if (errorTheta >= PI){
      errorTheta -= 2*PI;
    }
    else if (errorTheta <= -PI){
      errorTheta += 2*PI;
    }

    //Colocar el valor a vx y vy, de manera que solo haya rotación
    /* COMPLETAR  */
    vx = 0;
    vy = 0;

    // Calcular la velocidad angular en función del error y la constante k2
    double k2 = 1.6;
    vw = k2*errorTheta;

    // Lógica en caso vw sea mayor a PI/5
    if (fabs(vw) > PI/5){
        vw = (vw/fabs(vw))*PI/5;
    }
    // Lógica en caso errorTheta sea menor a 0.4, que genera velocidades muy pequeñas, asignar un valor alto de velocidad angular con el símbolo correspondiente
    if (fabs(errorTheta)<0.4){
        vw = (errorTheta/fabs(errorTheta))*PI/5.3;
    }

}


/******************************************************************************************************************************/
/************************************************  NO MODIFICAR ***************************************************************/
/******************************************************************************************************************************/

void EulerEstimation(double deltaT){
  /* 
  Function that integrates numericaly online robot position.
  Inputs:
    - deltaT: Time step.
  */
  // Pre-compute cosine and sine
  double ct, st;
  
  ct = cos(theta); 
  st = sin(theta);

  // Euler numerical estimation (use pose and processed variables)

  x = xPrev + deltaT*(R/4 * ((ct+st)*velAng[0] + (ct-st)*velAng[1] + (ct-st)*velAng[2] + (ct+st)*velAng[3]));
  y = yPrev + deltaT*(R/4 * ((st-ct)*velAng[0] + (st+ct)*velAng[1] + (st+ct)*velAng[2] + (st-ct)*velAng[3]));
  

  // Update previous values
  xPrev = x;
  yPrev = y;
  thetaPrev = theta;

  // Print estimated pose
  Serial.print("X = ");
  Serial.print(x*1000);
  Serial.print(" (mm), ");
  Serial.print("Y = ");
  Serial.print(y*1000);
  Serial.print(" (mm), ");
  Serial.print("theta = ");
  Serial.print(theta*180/PI);
  Serial.println(" (deg)");
  Serial.println("%%%%%%%%%%%%%%%%%%%%%%%%%%");

}


void CalculateVelAng(double vx, double vy, double vw) { 
  /* 
  Function that computes the velocity in rpm and the direction 
  of each wheel from the absolute velocity.

  Inputs:
    - vx: Linear velocity in X axis, in m/s.
    - vy: Linear velocity in Y axis, in m/s.
    - vw: Angular velocity in Z axis, in rad/s.
  */
  double w[] = {0, 0, 0, 0};
  // Angular velocity of each motor in rad/s
  w[0] = (vx - vy - vw * a_b) / R;
  w[1] = (vx + vy + vw * a_b) / R;
  w[2] = (vx + vy - vw * a_b) / R;
  w[3] = (vx - vy + vw * a_b) / R;
  for (int i = 0; i < NMOTORS; i++) {
    sgnPrev[i] = sgn[i];
    sgn[i] = w[i] / fabs(w[i]); 
    // Update motor direction
    dir[i] = (1 + sgn[i]) / 2;
    // Calculate desired angular velocity in rpm
    vt[i] = fabs(w[i]*30/PI);
  }
}

void setMotor(int dir, int pwmVal, int pwmch, int dirch) {
  /* 
  Function to setup pins to control motors.

  Inputs:
    - dir: Motor direction (1 or 0).
    - pwmVal: PWM control to pin.
    - pwmch: PWM pin channel.
    - dirch: Direction pin channel.
  */
  analogWrite(pwmch, pwmVal);
  if(dirch==12 || dirch==7){
    if (dir == 1) {
      digitalWrite(dirch, LOW);
    } else if (dir == 0) {
      digitalWrite(dirch, HIGH);
    } else {
      digitalWrite(dirch, LOW);
    }
  }
  else{
    if (dir == 1) {
      digitalWrite(dirch, HIGH);
    } else if (dir == 0) {
      digitalWrite(dirch, LOW);
    } else {
      digitalWrite(dirch, HIGH);
    }
  }
}


template <int j>
void readEncoder() {
  /* 
  Function that counts each rising edge of a encoder
  */
  velEnc[j]++;
}


void StopMotors(){
  /* 
  Function that stops each DC motor. 
  */
  CalculateVelAng(0,0,0);
  for(int k = 0; k < NMOTORS; k++){
    setMotor(dir[k], 0.0, pwm[k], DIR[k]);
  }
  CalculateVelAng(0,0,0);
  delay(6000);
}




