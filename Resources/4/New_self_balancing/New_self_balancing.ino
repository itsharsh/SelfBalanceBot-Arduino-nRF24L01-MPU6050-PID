#include "PID_v1.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "PID_AutoTune_v0.h"

#define MOTOR1 10
#define MOTOR2 11
#define FORW 7
#define BACK 8

MPU6050 mpu;
int16_t ax, ay, az;
int16_t gx, gy, gz;

double valax;
double valgx;
double valay;
double valgy;
double valgz;

double Setpoint, Input, Output;
double Kp = 50.00;//proportional action
double Ki = 1.00;//integral action
double Kd = 2.00;//derivative action

int STD_LOOP_TIME = 9;
int lastLoopTime = STD_LOOP_TIME;
int lastLoopUsefulTime = STD_LOOP_TIME;
unsigned long loopStartTime = 0;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() 
{
    Wire.begin();
    
    
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(60, 255);
    myPID.SetSampleTime(100);
 
    mpu.initialize();
    
    pinMode(7,OUTPUT);
    pinMode(8,OUTPUT);
    pinMode(10,OUTPUT);
    pinMode(11,OUTPUT);
}

void loop(){
  
mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

int valx = ax;
int valgy = -gy;


int input = (0.98*(input + valgy*0.01) + (0.02)*(valx));


  if(input<0){
    Input = input;
    Setpoint = 0;
    myPID.Compute();
    analogWrite(MOTOR1, Output);
    analogWrite(MOTOR2, Output);
    digitalWrite(FORW, 0);
    digitalWrite(BACK, 1);
  }else{
    Input = -input;
    Setpoint = 0;
    myPID.Compute();
    analogWrite(MOTOR1, Output);
    analogWrite(MOTOR2, Output);
    digitalWrite(BACK, 0);
    digitalWrite(FORW, 1);
  }
    
    lastLoopUsefulTime = millis()-loopStartTime;
  if(lastLoopUsefulTime<STD_LOOP_TIME)         delay(STD_LOOP_TIME-lastLoopUsefulTime);
  lastLoopTime = millis() - loopStartTime;
  loopStartTime = millis();
}
