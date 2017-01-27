#include <Arduino.h>
#include "BasicStepperDriver.h"

#define STEP 4
#define DIR 5
#define MS1 8
#define MS2 7
#define MS3 6
#define enablePin 9
//#define pot A1

int direction;    // rotation: (CW-CCW) of the motor
//int steps = 140; // this is equivalent to 252 degrees of potentiometer that stepper is connected to.

//MSGEQ7
int analogPin = 1; // read from multiplexer using analog input 0
int strobePin = 2; // strobe is attached to digital pin 2
int resetPin = 3; // reset is attached to digital pin 3
int spectrumValue[7]; // to hold a2d values

int band1 = 0; //7 banttan ilki (0'dan başlıyo)
int maxValue = 0; //sensörden gelen data (yumuşatılmış olan)
int currentValue = 0; //motoru hareket ettircek olan değerler (0-180 arası olcak servo için, bizim belirleyeceğimiz range'de)

//Smoothing
#define smoothingFactor 1
int sensorValue;
int sensorValues[smoothingFactor];
int smoothN16Values[smoothingFactor];
long interimN16Values[smoothingFactor];
//int smoothStep;

#define MOTOR_STEPS 200
#define MICROSTEPS 1

BasicStepperDriver stepper(MOTOR_STEPS, DIR, STEP);
long previous = MOTOR_STEPS / 2;

int Val1;

int donus;

void setup() {

  //Serial.begin(9600);


  //MSGEQ7
  pinMode(analogPin, INPUT);
  pinMode(strobePin, OUTPUT);
  pinMode(resetPin, OUTPUT);
  analogReference(DEFAULT);

  digitalWrite(resetPin, LOW);
  digitalWrite(strobePin, HIGH);

  //A9844
  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(STEP, OUTPUT);
  pinMode(enablePin, OUTPUT);


  stepper.setRPM(200);


  digitalWrite(MS1, LOW);
  digitalWrite(MS2, LOW);
  digitalWrite(MS3, LOW);

}

void loop() {

  //Serial.println(pot);

  //MSGEQ7
  digitalWrite(resetPin, HIGH);
  digitalWrite(resetPin, LOW);
  for (int i = 0; i < 7; i++)
  {
    digitalWrite(strobePin, LOW);
    delayMicroseconds(30); // to allow the output to settle
    spectrumValue[i] = analogRead(analogPin);

    if (i == 0) { //bu ilk bantsa (0.'ı) onu bas
      band1 = spectrumValue[i];
    }


    digitalWrite(strobePin, HIGH);
  }



  stepper.setMicrostep(MICROSTEPS);


  //Smoothing

  for (int i = 0; i < smoothingFactor; i++)
  {
    //sensorValues[i] = analogRead(1);
    interimN16Values[i] = (long(band1) << 1) + ((interimN16Values[i] * 15L) >> 4);
    smoothN16Values[i] = int((interimN16Values[i] + 16L) >> 5);

    // smoothStep = sensorValues[i];


    maxValue = smoothN16Values[i];
    //Serial.print(maxValue);
    //Serial.print(", ");


    delay(10);

  }





  //Stepper positioning
  /*
    if (maxValue < 60) {
      digitalWrite(enablePin, HIGH);
    }
    else {
      digitalWrite(enablePin, LOW);
    }*/
  //donus = 300;

  //Read analog 0 (pot) and map it to the range of the motor
  Val1 = constrain(maxValue, 300, 800);

  //Serial.print(Val1);
  //Serial.print(", ");

  long val = (long)map(Val1, 800, 300, 0, 140);
  //Move motor based off of last recorded position

  //int moving = val - previous;
  //Serial.println(val);
  // Serial.print(", ");
  //Serial.println(moving);


  stepper.move(val - previous);
  //Store current position
  previous = val;

  delay(20);
}



