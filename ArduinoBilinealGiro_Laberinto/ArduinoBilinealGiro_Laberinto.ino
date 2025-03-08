/*********************************************************************/
/***************************   LIBRARIES   ***************************/
/*********************************************************************/

#include <util/atomic.h>
#include "PinChangeInterrupt.h"
#include <math.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include <EEPROM.h>
#include "RobotControl.h"

/*********************************************************************/
/****************************   CLASSES   ****************************/
/*********************************************************************/

class SimplePID{
  /* 
  This class computes the control signal for velocity control of a 
  DC motor with a PID controller base on controller gains, a setpoint and 
  the actual velocity value.
  Params:
    - kp: Proportional gain.
    - kd: Derivative gain.
    - ki: Integral gain.
    - umax: Maximun control value.
    - eprec: Previous error in the control loop.
    - umax: Integral cumulative error.
    - vmin: Minimun velocity in rpm.
  */
  private:
    float kp, kd, ki, umax, vmin; // Parameters
    float eprev, eintegral; // Cumulative variables

  public:
  // Constructor
  SimplePID() : kp(1), kd(0), ki(0), umax(255), eprev(0.0), eintegral(0.0), vmin(15.0){}
  // A function to set the parameters
  void setParams(float kpIn, float kdIn, float kiIn, float umaxIn, float vminIn){
    kp = kpIn; kd = kdIn; ki = kiIn; umax = umaxIn; vmin = vminIn;
  }
  // A function to compute the control signal
  void evalu(int value, int target, float deltaT, int &pwr){
    // Error
    float e = (target - value)*((float) fabs(target) > vmin);
    // Derivative
    float dedt = (e - eprev)/(deltaT)*((float) fabs(target) > vmin);
    // Integral
    eintegral = (eintegral + e * deltaT)*((float) fabs(target) > vmin);
    if (umax/ki<eintegral){
      eintegral = umax/ki;
    }
    if (-umax/ki>eintegral){
      eintegral = -umax/ki;
    }
    // Control signal
    float u = kp * e + kd * dedt + ki * eintegral;
    pwr = (int) fabs(u);
    // Truncate signal
    if (pwr > umax) {
      pwr = umax;
    }
    if (pwr < 0) {
      pwr = 0;
    }
    eprev = e;
  }
};

/*********************************************************************/
/***************************   VARIABLES   ***************************/
/*********************************************************************/

// Number of motors
#define NMOTORS 4
// Pins
const int enc[] = {4, 5, 8, 13};
const int DIR[] = {2, 7, 9, 12};
const int pwm[] = {3, 6, 10, 11};

// Globals
int posPrev[] = {0, 0, 0, 0};
float vel[]   = {0.0, 0.0, 0.0, 0.0};
float vt[]    = {0.0, 0.0, 0.0, 0.0};
float vfil[]  = {0.0, 0.0, 0.0, 0.0};
int dir[]     = {0, 0, 0, 0};
long prevT    = 0;
// PPR of each motor
const double ppr[] = {1390, 1390, 1390, 1390};
const double ppr1 = 1390;
// const double ppr[] = {780, 780, 780, 780}; 
// Robot dimentions
const double a_b = 0.2025, R = 0.04;
const double l_a_b = 1/0.2025;



// Pose variables
double x = 0, y = 0, theta = 0;
double xPrev = 0, yPrev = 0, thetaPrev = 0; 


// PID class instances
SimplePID pid[NMOTORS];

// Dynamic variables
float velAng[]     = {0, 0, 0, 0};
int velEnc[]       = {0, 0, 0, 0};
int velEncSlack[]  = {0, 0, 0, 0};
float sampleT      = 0.1;
// Target variables for control
double vx = 0, vy = 0, vw = 0;



// Case variable
int secuencia = 1;
// Time periods of sequences
double T1 = 1.0, T2 = 1.0, T3 = 1.0;
// Initial and elapsed time of a sequence
double t0 = 0.0, t1 = 0.0;
// Velocity limits
double vminLim = 15.0;
// Velocities signs
int sgn[]     = {1, 1, 1, 1};
int sgnPrev[] = {1, 1, 1, 1};

//IMU
MPU6050 sensor;
int gx, gy, gz;
long tiempo_prev, dt;

int seq = 0;
float xGoal = 1;
float yGoal = -0.5;
float thetaGoal = -PI/2;
double errorTheta = PI;

float directionGoal = 2*PI;
float gz_bias = 0;

// Ruta predefinida
int punto_actual = 0;
#define MAX_PUNTOS 35
const double ruta[MAX_PUNTOS][3] = {
{1.600, 0.000, 0.000}, {0.000, 0.000, PI/2}, {0.400, 0.000, 0.000}, {0.000, 0.000, PI/2}, {0.400, 0.000, 0.000}, {0.000, 0.000, -PI/2}, {0.400, 0.000, 0.000}, {0.000, 0.000, PI/2}, {0.400, 0.000, 0.000}, {0.000, 0.000, -PI/2}, {0.400, 0.000, 0.000}, {0.000, 0.000, PI/2}, {0.400, 0.000, 0.000}, {0.000, 0.000, -PI/2}, {0.400, 0.000, 0.000}, {0.000, 0.000, PI/2}, {0.400, 0.000, 0.000}, {0.000, 0.000, -PI/2}, {0.400, 0.000, 0.000}, {0.000, 0.000, PI/2}, {0.400, 0.000, 0.000}, {0.000, 0.000, -PI/2}, {0.400, 0.000, 0.000}, {0.000, 0.000, -PI/2}, {0.800, 0.000, 0.000}, {0.000, 0.000, -PI/2}, {0.400, 0.000, 0.000}, {0.000, 0.000, PI/2}, {0.400, 0.000, 0.000}, {0.000, 0.000, -PI/2}, {0.400, 0.000, 0.000}, {0.000, 0.000, PI/2}, {0.400, 0.000, 0.000}, {0.000, 0.000, PI/2}, {0.400, 0.000, 0.000}
};
RobotControl robot(a_b, R, ppr1);

float velRPM[]      = {0, 0, 0, 0};

/*********************************************************************/
/*****************************   SETUP   *****************************/
/*********************************************************************/
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
    Serial.print("Bias del giroscopio: ");
    Serial.println(gz_bias);
}

void setup() {
  // Begin communication
  Serial.begin(9600);
  Wire.begin();           //Iniciando I2C  
  sensor.initialize();    //Iniciando el sensor

  if (sensor.testConnection()) Serial.println("Sensor iniciado correctamente");
  else Serial.println("Error al iniciar el sensor");
  // Setup I/O pins
  for(int k = 0; k < NMOTORS; k++){
    pinMode(enc[k], INPUT);
    pinMode(pwm[k], OUTPUT);
    pinMode(DIR[k], OUTPUT);
  }
  // PID gains for each motor
    for (int k = 0; k < NMOTORS; k++) {
        pid[k].setParams(3.4, 0.03, 0.01, 255, vminLim);
    }
  // Activate interrupts
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(enc[0]), readEncoder<0>, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(enc[1]), readEncoder<1>, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(enc[2]), readEncoder<2>, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(enc[3]), readEncoder<3>, CHANGE);
  delay(2000);

  /*IMPORTANTE*/
  /*Es útil en esta posición solo para la pregunta de movimiento 2*/
  calibrateIMU();
  /*Eliminar cuando se realice la pregunta de secuencias y asignarlo según su lógica*/
  //CalculateDirectionGoal(xGoal, yGoal);

  prevT = micros();
  tiempo_prev = millis();

}

/*********************************************************************/
/*****************************   LOOP   ******************************/
/*********************************************************************/
void loop() {
    long currT = micros();
    float deltaT = ((float)(currT - prevT)) / (1.0e6);
    //prevT = currT;
    t1 = ((float) currT) / 1.0e6;

    // Obtener datos del giroscopio
    sensor.getRotation(&gx, &gy, &gz);
    //float gz_rad_s = (gz * (250.0 / 32768.0)) - gz_bias;
    //gz_rad_s *= (PI / 180.0);
    dt = millis() - tiempo_prev;
    tiempo_prev = millis();
    float gyroZ = gz / 131.0;  // en °/s
    theta += (gyroZ * (PI / 180.0)) * (dt / 1000.0);

    if (theta >= PI) {
      theta -= 2*PI;
    }
    else if (theta <= -PI) {
      theta += 2*PI;
    }

    // Obtener delta de tiempo y actualizar ángulo
    //dt = millis() - tiempo_prev;
    //tiempo_prev = millis();
    //theta += gz_rad_s * (dt / 1000.0);
    //theta = fmod(theta + PI, 2 * PI) - PI;

    // 📌 **Imprimir estado de variables claves**
    /*
    Serial.print("Secuencia: "); Serial.println(seq);
    Serial.print("Theta: "); Serial.println(theta * 180 / PI);
    Serial.print("Error Theta: "); Serial.println(errorTheta * 180 / PI);
    Serial.print("X: "); Serial.print(x); Serial.print(" Y: "); Serial.println(y);
    Serial.print("vx: "); Serial.print(vx); Serial.print(" vy: "); Serial.print(vy);
    Serial.print(" vw: "); Serial.println(vw);
    */
    //EulerEstimation(dt);
    // Compute program each sample time
    if (sampleT <= deltaT) {
      prevT = currT;
      // Disable interrupts temporarily while reading encoder counts
      noInterrupts(); 
      for (int k = 0; k < NMOTORS; k++){
        // Reset counter
        velEncSlack[k] = velEnc[k];
        velEnc[k] = 0;
      }
      interrupts();
      robot.updateVAngValue(velEncSlack, deltaT, velAng, vel);

      /*
      for (int k = 0; k < NMOTORS; k++){
        // Calculate velocity in rpm
        vel[k] = velEncSlack[k] / deltaT / ppr[k] * 60.0;
        // Calculate velocity in rad/s (keeping the sign)
        velAng[k] = sgn[k] * vel[k] * PI / 30;
      }
      */
      // Estimate Pose via Euler integration
      //EulerEstimation(deltaT);
    

      // Determinar la secuencia de movimiento
      /*
      if (seq == 0) {
        CalculatePositionError(xGoal, 0.0, vx, vy, vw);
      } else if (seq == 1) {
        Serial.println("🚗 Fase 1: Movimiento hacia el objetivo");
        //CalculatePositionError(0.0, 0.0, vx, vy, vw);
        CalculateOrientationError(thetaGoal, vx, vy, vw, errorTheta);
      } else if (seq == 2) {
        Serial.println("🔄 Fase 2: Giro final");
        //CalculateOrientationError(thetaGoal, vx, vy, vw, errorTheta);
        CalculatePositionError(xGoal, yGoal, vx, vy, vw);
      } else if (seq == 3) {
        Serial.println("🛑 Fase 3: Detención");
        CalculateVelAng(0, 0, 0);
        StopMotors();
      }
      */

      //CalculateOrientationError(directionGoal, vx, vy, vw, errorTheta);
      if (punto_actual < MAX_PUNTOS) {
        x         = xValueBilineal2(deltaT, x, theta, velAng);
        //y         = robot.yValueBilineal(deltaT, y, theta, velAng);
        xGoal = ruta[punto_actual][0];
        yGoal = ruta[punto_actual][1];
        thetaGoal = ruta[punto_actual][2];
        if (thetaGoal != 0 ){
          CalculateOrientationError(thetaGoal, vx, vy, vw, errorTheta);
        }
        else if (thetaGoal == 0){
          CalculatePositionError(xGoal, yGoal, vx, vy, vw);
        }
      
      // Aplicar velocidades a los motores
      CalculateVelAng(vx, vy, vw);
      for (int k = 0; k < NMOTORS; k++) {
          int pwr;
          pid[k].evalu(vel[k], vt[k], deltaT, pwr);
          setMotor(dir[k], pwr, pwm[k], DIR[k]);

          // 📌 **Imprimir potencias**
          Serial.print("Motor "); Serial.print(k);
          Serial.print(": Dir "); Serial.print(dir[k]);
          Serial.print(" | PWM "); Serial.println(pwr);
      }
    }
    

    // 📌 **Transiciones entre secuencias**
    if (thetaGoal != 0 && fabs(thetaGoal - theta) < 0.05) {
        //Serial.println("✅ Transición 0 → 1 (Iniciar Movimiento)");
        //StopMotors();
        punto_actual++;
        x=0;
        //y=0;
        theta=0;
        //StopMotors();
        robot.StopMotors(pwm);
    } 
    else if (thetaGoal == 0 && fabs(xGoal - x) < 0.1 ) {
        //Serial.println("✅ Transición 1 → 2 (Iniciar Giro Final)");
        //StopMotors();
        punto_actual++;
        x=0;
        //y=0;
        theta=0;
        //StopMotors();
        robot.StopMotors(pwm);
    }
    }
}


/*********************************************************************/
/****************************   FUNCIONES   **************************/
/*********************************************************************/

double xValueBilineal2(double deltaT, double &x, double theta, float* meanVelAng){

/* 
  Function to calculate the x position using Bilinear Estimation based on the previous x value.

  Inputs:
    - deltaT     : Time interval since the last position update.
    - x          : Previous x position of the robot in Cartesian coordinates.
    - theta      : Current orientation angle of the robot in radians.
    - meanVelAng : Current angular velocity of the robot around the Z axis.

  Output:
    - x : Updated x position of the robot in Cartesian coordinates.
*/

  double ct, st;
  ct = cos(theta); st = sin(theta);
  // Bilineal numerical estimation
  x += + deltaT*(R/4 * ((ct+st)*meanVelAng[0] + (ct-st)*meanVelAng[1] + (ct-st)*meanVelAng[2] + (ct+st)*meanVelAng[3]));
  //xPrev = x;

  return x;
}

void CalculateDirectionGoal(double xGoal_l, double yGoal_l) {
    double delta_x = xGoal_l - x;
    double delta_y = yGoal_l - y;
    directionGoal = atan2(delta_y, delta_x);
    directionGoal = fmod(directionGoal + PI, 2 * PI) - PI;
}

void CalculatePositionError(double xGoal_l, double yGoal_l, double &vx, double &vy, double &vw) {
  /*
    double errorX = xGoal_l - x;
    double errorY = yGoal_l - y;
    double k = 1.2;
    double vx_global = k * errorX;
    double vy_global = k * errorY;
    double ct = cos(theta);
    double st = sin(theta);
    vw = 0;
    vx = ct * vx_global + st * vy_global;
    vy = -st * vx_global + ct * vy_global;
    */
    double errorX = xGoal_l - x;
    vx        = 1.2*errorX/abs(errorX);
    if (abs(errorX)<0.05)
          vx        *= 0.1;
}

void CalculateOrientationError(double thetaGoal_l, double &vx, double &vy, double &vw, double &errorTheta) {
    errorTheta = thetaGoal_l - theta;
    if (errorTheta >= PI) {
        errorTheta -= 2 * PI;
    } else if (errorTheta <= -PI) {
        errorTheta += 2 * PI;
    }
    vx = 0;
    vy = 0;
    double k2 = 2.0;
    vw = k2 * errorTheta;
    if (fabs(vw) > PI / 5) {
        vw = (vw / fabs(vw)) * PI / 2;
    }
    if (fabs(errorTheta) < 0.5) {
        vw = (errorTheta / fabs(errorTheta)) * PI / 9;
    }
}

void CalculateVelAng(double vx, double vy, double vw) {
  double w[NMOTORS];
  w[0] = (vx - vy - vw * a_b) / R;
  w[1] = (vx + vy + vw * a_b) / R;
  w[2] = (vx + vy - vw * a_b) / R;
  w[3] = (vx - vy + vw * a_b) / R;
  for (int i = 0; i < NMOTORS; i++) {
    //dir[i] = (w[i] > 0) ? 1 : 0;
    //vt[i] = fabs(w[i] * 30 / PI);
    sgnPrev[i] = sgn[i];
    sgn[i] = w[i] / fabs(w[i]); 
    // Update motor direction
    dir[i] = (1 + sgn[i]) / 2;
    // Calculate desired angular velocity in rpm
    vt[i] = fabs(w[i] * 30 / PI);
  }
}

void EulerEstimation(double deltaT) {
    double ct = cos(theta);
    double st = sin(theta);
    x += deltaT * (R / 4) * ((ct + st) * velAng[0] + (ct - st) * velAng[1] + (ct - st) * velAng[2] + (ct + st) * velAng[3]);
    y += deltaT * (R / 4) * ((st - ct) * velAng[0] + (st + ct) * velAng[1] + (st + ct) * velAng[2] + (st - ct) * velAng[3]);
    // Update previous values
    xPrev = x;
    yPrev = y;
    thetaPrev = theta;
    Serial.print("X = ");
    Serial.print(x * 1000);
    Serial.print(" mm, Y = ");
    Serial.print(y * 1000);
    Serial.print(" mm, Theta = ");
    Serial.print(theta * 180 / PI);
    Serial.println(" deg");
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
    velEnc[j]++;
}

void StopMotors() {
    CalculateVelAng(0, 0, 0);
    for (int k = 0; k < NMOTORS; k++) {
        setMotor(dir[k], 0, pwm[k], DIR[k]);
    }
    delay(6000);
}


