#include<SPI.h>
#include<RF24.h>
#include<nRF24L01.h>
#include "printf.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <PID_v1.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
  mpuInterrupt = true;
}
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3], yaw, pitch, roll;        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
#define INTERRUPT_PIN 2 // use pin 2 on Arduino Uno & most boards
MPU6050 mpu;

int lm1 = 5, lm2 = 6, rm1 = 9, rm2 = 10;
int throttle = 150;

int CE_PIN = 7, CSN_PIN = 8;
RF24 radio(CE_PIN, CSN_PIN);
const uint64_t pipeIn = 0xFFBA1B01E1LL;
int radio_led = 3;

int data;

double setpoint, input, output;
double kp = 56, ki = 0.5, kd = 3.4 , tune_kp, tune_ki, tune_kd;//52 0.7 3.5
PID pid(&input, &output, &setpoint, kp, ki, kd, DIRECT);

double pitch_err;
void setup()
{
  Serial.begin(115200);
  pinMode(radio_led, OUTPUT);
  pinMode(lm1, OUTPUT);
  pinMode(lm2, OUTPUT);
  pinMode(rm1, OUTPUT);
  pinMode(rm2, OUTPUT);

  digitalWrite(radio_led, LOW);
  start_mpu();
//  start_radio();
//  go_stop();
//  set_pid();
//  tune_kp = kp;
//  tune_ki = ki;
//  tune_kd = kd;
  /*
    delay(1000);
    digitalWrite(radio_led, HIGH);
    get_mpu_data();
    //  pitch_err = pitch;

    delay(1000);
    digitalWrite(radio_led, LOW);
  */
//  delay(2000);
}

void loop()
{
//  get_radio_data();
  get_mpu_data();

  //  pitch -= pitch_err;
/*
  input = pitch;
  pid.Compute();
  if (output < 0)
    output = -output;
  throttle = output;
/*
  //  cal_manual_pid();
  Serial.print("\t");
  Serial.print(throttle);
  Serial.print("\t");
  Serial.print(tune_kp);
*/
//  balance();
//  Serial.println();
}

void balance()
{
  if (pitch < -1)
    go_reverse();
  else if (pitch > 1)
    go_forward();
  else
    go_stop();
}
/*
  void cal_manual_pid()
  {
  //  if (data == 5)
  //  {
  if (pitch < 1 && pitch > -1)
    throttle = 65;
  else if (pitch < 2 && pitch > -2)
    throttle = 80;
  else if (pitch < 3 && pitch > -3)
    throttle = 95;
  else if (pitch < 3.5 && pitch > -3.5)
    throttle = 125;
  else if (pitch < 4 && pitch > -4)
    throttle = 140;
  else if (pitch < 5 && pitch > -5)
    throttle = 155;
  else if (pitch < 6 && pitch > -6)
    throttle = 170;
  else if (pitch < 7 && pitch > -7)
    throttle = 200;
  else if (pitch < 10 && pitch > -10)
    throttle = 210;
  else if (pitch < 12 && pitch > -12)
    throttle = 200;
  else if (pitch < 14 && pitch > -14)
    throttle = 215;
  else if (pitch < 15 && pitch > -15)
    throttle = 230;
  else if (pitch < 16 && pitch > -16)
    throttle = 240;
  else if (pitch < 17 && pitch > -17)
    throttle = 250;
  else
    throttle = 255;
  //  }
  //  else
  //    go();
  }
*/
void start_radio()
{
  radio.begin();
  printf_begin();
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_250KBPS);
  radio.openReadingPipe(0, pipeIn);
  radio.startListening();
  radio.printDetails();
}

void get_radio_data()
{
  if (radio.available())
  {
    radio.read(&data, sizeof(&data));
    digitalWrite(radio_led, HIGH);
    tune_kp = data ;
    pid.SetTunings(tune_kp, tune_ki, tune_kd);
  }
  else
  {
    digitalWrite(radio_led, LOW);
  }
}

void start_mpu()
{
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, input);

  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void get_mpu_data()
{
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize)
  { /*
      pid.Compute();
      motorController.move(output, MIN_ABS_SPEED);

      unsigned long currentMillis = millis();

      if (currentMillis - time1Hz >= 1000)
      {
       loopAt1Hz();
       time1Hz = currentMillis;
      }

      if (currentMillis - time5Hz >= 5000)
      {
       loopAt5Hz();
       time5Hz = currentMillis;
      }*/
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    yaw = (ypr[0] * 180) / M_PI;
    pitch = (ypr[1] * 180) / M_PI;
    roll = (ypr[2] * 180) / M_PI;

    Serial.print("ypr\t");
    Serial.print(yaw);
    Serial.print("\t");
    Serial.print(pitch);
    Serial.print("\t");
    Serial.println(roll);
  }
}

void set_pid()
{
  get_mpu_data();
  input = pitch;
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-230, 230);
  pid.SetSampleTime(10);
  setpoint = 0;
}
/*
  void go()
  {
  switch (data)
  {
    case  2:
      go_forward();
      break;
    case 4:
      go_left();
      break;
    case 6:
      go_right();
      break;
    case 8:
      go_reverse();
      break;
    default:
      go_stop();
  }
  }
*/
void motor_val(int a, int b, int c, int d)
{
  analogWrite(lm1, a);
  analogWrite(lm2, b);
  analogWrite(rm1, c);
  analogWrite(rm2, d);
}

void go_forward ()
{
  //  Serial.println("Forward");
  motor_val(0, throttle, 0, throttle);
}

void go_reverse()
{
  //  Serial.println("Reverse");
  motor_val(throttle, 0, throttle, 0);
}

void go_left()
{
  //  Serial.println("Left");
  motor_val(throttle, 0, 0, throttle);
}

void go_right()
{
  //  Serial.println("Right");
  motor_val(0, throttle, throttle, 0);
}
void go_stop()
{
  //  Serial.println("Stop");
  motor_val(0, 0, 0, 0);
}
