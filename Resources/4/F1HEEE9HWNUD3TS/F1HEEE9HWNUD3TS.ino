#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL

bool dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount; // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q; // [w, x, y, z] quaternion container
VectorInt16 aa; // [x, y, z] accel sensor measurements
VectorInt16 aaReal; // [x, y, z] gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z] world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z] gravity vector
float euler[3]; // [psi, theta, phi] Euler angle container
float ypr[3]; // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector

static int pot1Pin = A0;
static int pot2Pin = A1;
static int pot3Pin = A2;
float Kp = 0;
float Ki = 0;
float Kd = 0;
float targetAngle = 8.5;
float currAngle = 0;
float pTerm = 0;
float iTerm = 0;
float dTerm = 0;
float integrated_error = 0;
float last_error = 0;
float K = 1;
// ================================================================
// === INTERRUPT DETECTION ROUTINE ===
// ================================================================

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// === INITIAL SETUP ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
#endif

    Serial.begin(115200);
    while (!Serial); 
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
    if (devStatus == 0) {
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    pinMode(3, OUTPUT);
    pinMode(4, OUTPUT);
    pinMode(pot3Pin, INPUT);
    pinMode(pot1Pin, INPUT);
    pinMode(pot2Pin, INPUT);
    pinMode(A3, OUTPUT);
    pinMode(12,OUTPUT);
    pinMode(13,OUTPUT);
  }

// ================================================================
// === MAIN PROGRAM LOOP ===
// ================================================================

void loop() {
    if (!dmpReady) return;
    while (!mpuInterrupt && fifoCount < packetSize) {
    }
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_YAWPITCHROLL
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
#endif
        Kp = map(analogRead(pot1Pin), 0, 1024, 0, 255);
        Ki = map(analogRead(pot2Pin), 0, 1024, 0, 20);
        Kd = map(analogRead(pot3Pin), 0, 1024, 0, 20);
        Serial.print("Kp: ");
        Serial.print(Kp);
        Serial.print("        Ki: ");
        Serial.print(Ki);
        Serial.print("        Kd: ");
        Serial.println(Kd);
        currAngle = (ypr[1] * 180/M_PI);
        Drive_Motor(updateSpeed());
        Serial.print("speed: ");
        Serial.println(updateSpeed());
    }
}

float updateSpeed() {
  float error = targetAngle - currAngle;
  pTerm = Kp * error;
  integrated_error += error;
  iTerm = Ki * constrain(integrated_error, -50, 50);
  dTerm = Kd * (error - last_error);
  last_error = error;
  return -constrain(K*(pTerm + iTerm + dTerm), -255, 255);
}

float Drive_Motor(float torque)  {
  Serial.print("torque: ");
  Serial.println(torque);
  if (torque >= 0)  {                                        // drive motors forward
    digitalWrite(3, LOW);
    digitalWrite(4, HIGH);
    digitalWrite(12, LOW);
    digitalWrite(13, HIGH);
  }  else {                                                  // drive motors backward
    digitalWrite(3, HIGH);
    digitalWrite(4, LOW);
    digitalWrite(12, HIGH);
    digitalWrite(13, LOW);
    torque = abs(torque);
  }
  analogWrite(A3,torque);  
}

