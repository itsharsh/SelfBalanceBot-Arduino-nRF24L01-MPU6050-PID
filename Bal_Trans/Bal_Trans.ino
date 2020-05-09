#include<SPI.h>
#include<RF24.h>
#include<nRF24L01.h>
#include "printf.h"

int CE_PIN = 9, CSN_PIN = 10;
RF24 radio(CE_PIN, CSN_PIN);
const uint64_t pipeOut = 0xFFBA1B01E1LL;

int data = 10;

void setup()
{
  Serial.begin(115200);
  start_nrf();
  delay(2000);
}

void loop()
{
  if (Serial.available())
  {
    data = Serial.parseInt();
    Serial.print("\t");
    Serial.println(data);
  }
  Serial.println(data);
  radio.write(&data, sizeof(data));
}

void start_nrf()
{
  radio.begin();
  printf_begin();
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(pipeOut);
  radio.printDetails();
}

