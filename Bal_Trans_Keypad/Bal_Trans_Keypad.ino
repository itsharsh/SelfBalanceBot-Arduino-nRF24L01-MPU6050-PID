#include<Keypad.h>
#include<SPI.h>
#include<RF24.h>
#include<nRF24L01.h>
#include "printf.h"

const byte ROWS = 4; // Four rows
const byte COLS = 4; // four columns
char keys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};
byte rowPins[ROWS] = {  8, 7, 6 , 5};
byte colPins[COLS] = {  4, 3, 2 , A5};
Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

RF24 radio(9, 10);
const uint64_t pipeOut = 0xABCDEFD0E1LL;

int data = 5;

void setup()
{
  Serial.begin(115200);
  keypad.addEventListener(keypadEvent);
  start_nrf();
  delay(2000);
}

void loop()
{
  keypad.getKey();
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

void keypadEvent(KeypadEvent eKey)
{

  switch (keypad.getState())
  {
    case PRESSED:
      delay(50);
      switch (eKey)
      {
        case '2': data = 2;
          break;
        case '8': data = 8;
          break;
        case '4': data = 4;
          break;
        case '6': data = 6;
          break;
        default: data = 5;
      }
      break;
  }
  Serial.println(data);
}

