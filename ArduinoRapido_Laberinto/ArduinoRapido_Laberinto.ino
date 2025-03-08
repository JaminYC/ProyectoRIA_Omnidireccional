/*********************************************************************/
/***************************   LIBRARIES   ***************************/
/*********************************************************************/

#include <util/atomic.h>
#include "PinChangeInterrupt.h"
#include <math.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "RobotControl.h"
#include "SimplePID.h"

/*********************************************************************/
/***************************   VARIABLES   ***************************/
/*********************************************************************/

// Número de motores
#define NMOTORS 4

// Pines
const int enc[] = {4, 5, 8, 13};      // Pines de encoders
const int DIR[] = {2, 7, 9, 12};      // Pines de dirección
const int pwm[] = {3, 6, 10, 11};     // Pines PWM

// Dimensiones del robot
const double a_b = 0.2025, R = 0.04, ppr = 1390; //1390 - 680
const double l_a_b = 1/0.2025;


// Cinemática
double x  = 0.0,      y  = 0.0,       theta = 0.0;
double xPrev = 0.0,   yPrev = 0.0,    thetaPrev = 0.0;
double vx = 0.0,      vy = 0.0,       vw    = 0.0;
float  xError = 10.0, yError = 0.0,  thetaError = 10.0;
double xGoal = 0.0,   yGoal = 0.0,    thetaGoal = 0.0;


// Variables globales de control de velocidad
float velAngGoal[]  = {0.0, 0.0, 0.0, 0.0};
float velAng[]      = {0.0, 0.0, 0.0, 0.0};
float velRPM[]      = {0, 0, 0, 0};
int encCountAct[]   = {0, 0, 0, 0}; 
int encCount[]      = {0, 0, 0, 0};  
int dir[]           = {1, 1, 1, 1};

// Variables de tiempo y muestreo
float sampleT = 0.05;       // Tiempo de muestreo
//long tiempo_prev = 0;      // Tiempo anterior del IMU
long prevT = 0; 

// Sensor IMU
MPU6050 sensor;
int gx = 0, gy = 0, gz = 0;  // Valores del giroscopio (ejes X, Y y Z)
double gz_offset = 0;
long tiempo_prev, dt;
float heading = 0;
int trigger=0;

//Kalman filter
double state = 0, P = 0.1, K = 0;   // State, Proccess error variance, Kalman Gain
double Q_vAng  = 0.0025, R_angle = 0.00087; // Ruido de la medición
double thetaIMU = 0, vAngIMU = 0, vwRobot = 0, kalmanAngle = 0;

// Posiciones
struct Pose {
  double type, x, y; //1: x movement  | 2: y movement 
};


// Puntos dados por el plano
struct PathPlaning{
  double x, y, theta;
};

int n = 0, cantMov=18, casoDiagonal = 0;
Pose mov[18] = {{1, -1.6, 0.0}, {2, -1.6, -0.4}, {1, -1.2, -0.4}, {2, -1.2, -0.8}, {1, -0.8, -0.8}, {2, -0.8, -1.2}, {1, -0.4, -1.2}, {2, -0.4, -1.6}, {1, 0.0, -1.6}, {2, 0.0, -2.0}, {1, 0.4, -2.0}, {2, 0.4, -2.4}, {1, -0.4, -2.4}, {2, -0.4, -2.0}, {1, -0.8, -2.0}, {2, -0.8, -1.6}, {1, -1.2, -1.6}, {2, -1.2, -2.0}};


SimplePID pid[NMOTORS];
int pwr;
RobotControl robot(a_b, R, ppr);

/*********************************************************************/
/*****************************   SETUP   *****************************/
/*********************************************************************/

void setup() {
  Serial.begin(9600);
  Wire.begin();       
  //Iniciando el sensor
  sensor.initialize();    
  if (sensor.testConnection()) Serial.println("Sensor iniciado correctamente");
  else Serial.println("Error al iniciar el sensor");
  gz_offset = robot.calibrarGZ();

  // Setup I/O pins
  for(int k = 0; k < NMOTORS; k++){
    pinMode(enc[k], INPUT);
    pinMode(pwm[k], OUTPUT);
    pinMode(DIR[k], OUTPUT);
  }

  float KP[] = {0.5, 0.5, 0.5, 0.5};  
  float KD[] = {0.02, 0.02, 0.02, 0.02};
  float KI[] = {6.0, 6.0, 6.0, 6.0};    
  

  // Set parameters kit antigup 
  pid[0].setParams(KP[0], KD[0], KI[0], 255, 10.0);
  pid[1].setParams(KP[1], KD[1], KI[1], 255, 10.0);
  pid[2].setParams(KP[2], KD[2], KI[2], 255, 10.0);
  pid[3].setParams(KP[3], KD[3], KI[3], 255, 10.0);

  // Activate interrupts
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(enc[0]), readEncoder<0>, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(enc[1]), readEncoder<1>, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(enc[2]), readEncoder<2>, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(enc[3]), readEncoder<3>, CHANGE);
  delay(500);

  prevT = micros();
}

/*********************************************************************/
/*****************************   LOOP   ******************************/
/*********************************************************************/

void loop() {
 
  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / (1.0e6);
  sensor.getRotation(&gx, &gy, &gz);
  gz = gz-gz_offset;
  heading = (float)(gz/131)*dt/1000*PI/180 + heading;

  if (sampleT <= deltaT) {
    prevT = currT;
    
    noInterrupts();
    for (int k = 0; k < NMOTORS; k++) {
      encCountAct[k] = encCount[k];
      encCount[k] = 0;
    }
    interrupts();
    robot.updateVAngValue(encCountAct, deltaT, velAng, velRPM);

    if(n<cantMov){
      x         = robot.xValueBilineal(deltaT, x, theta, velAng);
      y         = robot.yValueBilineal(deltaT, y, theta, velAng);
      theta     = robot.thetaValueBilineal(deltaT, theta, velAng);
      
      switch(int(mov[n].type)){
      
      case 0:
        vw         = PI*thetaError/(abs(thetaError)*6);
        vy        = 0;
        vx        = 0;
        if(fabs(thetaError)<1){n++;robot.StopMotors(pwm); delay(1000);} 
      break;

      case 1:
        xError    = robot.xError(mov[n].x, x);
        vx        = 1*xError/abs(xError);
        if (abs(xError)<0.05)
          vx        *= 0.1;
        vy        = 0;
        vw        = 0;
        if(fabs(xError)<0.005){n++; robot.StopMotors(pwm); delay(500);} 
      break;

      case 2:
        
        yError    = robot.yError(mov[n].y, y);
        vy        = 1*yError/abs(yError);
        if (abs(yError)<0.05)
          vy        *= 0.1;
        vx        = 0;
        vw        = 0;
        if(fabs(yError)<0.005){n++;robot.StopMotors(pwm); delay(500);}
      break;
      case 3:
        // Pasos
        switch(casoDiagonal){
          case 0: 
          thetaGoal = robot.CalculateDirectionGoal(mov[n].x, mov[n].y, x, y);
          delay(500);
          casoDiagonal=1;
          break;
          case 1:
          thetaError = robot.thetaError(thetaGoal, theta);
          vw         = PI*thetaError/(abs(thetaError)*6);
          vy         = 0;
          vx         = 0;
          if(fabs(thetaError)<0.03){robot.StopMotors(pwm); casoDiagonal=2; delay(500);}
          break;
          case 2:
          xError    = robot.xError(mov[n].x, x);
          vx        = 0.2;
          vy        = 0.0;
          vw        = 0;
          if(fabs(xError)<0.005){robot.StopMotors(pwm); casoDiagonal=3; delay(500);} 
          break;
          case  3: 
          thetaError = robot.thetaError(0, theta);
          vw         = PI*thetaError/(abs(thetaError)*6);
          vy         = 0;
          vx         = 0;
          if(fabs(thetaError)<0.03){n++;robot.StopMotors(pwm); casoDiagonal=0; delay(500);}
          break;
        }
      }
      robot.CalculateVelAngGoal(vx, vy, vw, dir, velAngGoal);
      for (int k = 0; k < NMOTORS; k++) {
        pid[k].evalu(velRPM[k], velAngGoal[k], deltaT, pwr);
        robot.setMotor(dir[k], pwr, pwm[k], DIR[k]);
      }
    }else{robot.StopMotors(pwm);Serial.print("FIN");}
  
    Serial.print("   |x:");
    Serial.print(x);
    Serial.print("   |y:");
    Serial.print(y);
    Serial.print("   |theta:");
    Serial.print(theta);
    Serial.print("   |caso diagonal:");
    Serial.print(casoDiagonal);
    Serial.print("   |Movimiento:");
    Serial.println(n+1);
  }
}

/**************************************************************************/
/*****************************   FUNCTIONS   ******************************/
/**************************************************************************/

template <int j>
void readEncoder() {encCount[j]++;}

double kalmanFilter(float dt, float u_n, float z_n){
  state = state + u_n*dt;
  P     = P + (Q_vAng*dt)*(Q_vAng*dt);

  double gainKalman     = z_n - state; // H = 1

  K     = P/(P + (R_angle*dt)*(R_angle*gainKalman));

  state = state + K*y;

  P     = (1-K)*P;

  return state;
}



