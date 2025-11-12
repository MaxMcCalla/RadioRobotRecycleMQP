#include <Arduino.h>
#include <AccelStepper.h>

// put function declarations here:
AccelStepper Step1(AccelStepper::FULL2WIRE,12,14);
AccelStepper Step2(AccelStepper::FULL2WIRE, 27,26);
AccelStepper Step3(AccelStepper::FULL2WIRE, 25,33);
AccelStepper Step4(AccelStepper::FULL2WIRE, 32,35);


void setup() {
  Step1.setMaxSpeed(200.0);
  Step1.setAcceleration(100.0);
  Step2.setMaxSpeed(200.0);
  Step2.setAcceleration(100.0);
  Step3.setMaxSpeed(200.0);
  Step3.setAcceleration(100.0);
  Step4.setMaxSpeed(200.0);
  Step4.setAcceleration(100.0);
  Step1.move(1000);
  Step1.run();

  delay(1000);
  // put your setup code here, to run once:
  
}

void loop() {
  Step2.moveTo(1000);
  Step2.run();
  Step1.moveTo(-100);
  Step1.run();
  Step3.moveTo(300);
  Step3.run();
  Step4.moveTo(500);
  Step4.run();
  //delay(100);
  // put your main code here, to run repeatedly:

}