#include <Arduino.h>
#include <AccelStepper.h>

// put function declarations here:
int myFunction(int, int);
AccelStepper Step1(AccelStepper::FULL4WIRE,2,4,5,18);


void setup() {
  Step1.setMaxSpeed(200.0);
  Step1.setAcceleration(100.0);
  Step1.moveTo(24);
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}