#include <Arduino.h>
#include <AccelStepper.h>

const int STATE_RESET_ENCODERS = 0;
const int STATE_MOVE_ARM = 1;

int buttonPins[4] = {1,2,3,4};

int ticksToDegrees[4] = {1,1,1,1};

// put function declarations here:
AccelStepper Step1(AccelStepper::FULL2WIRE,12,14);
AccelStepper Step2(AccelStepper::FULL2WIRE, 27,26);
AccelStepper Step3(AccelStepper::FULL2WIRE, 25,33);
AccelStepper Step4(AccelStepper::FULL2WIRE, 32,35);

AccelStepper Motors[4] = {Step1,Step2,Step3,Step4};

int state;

bool getLimitSwitch(int switchID){
  return digitalRead(buttonPins[switchID]);
}

void resetMotors(){
  for(int i = 0; i < sizeof(Motors); i++){
    Motors[i].setCurrentPosition(0);
  }
}

void moveMotor(int motorID, double position){
  Motors[motorID].moveTo(position);
  Motors[motorID].run();
}

void moveAllMotors(double motorPositions[]){
  for(int i = 0; i < sizeof(motorPositions); i++){
    moveMotor(i,motorPositions[i]);
  }
}


void setup() {
  state = STATE_RESET_ENCODERS;
  Step1.setMaxSpeed(200.0);
  Step1.setAcceleration(100.0);
  Step2.setMaxSpeed(200.0);
  Step2.setAcceleration(100.0);
  Step3.setMaxSpeed(200.0);
  Step3.setAcceleration(100.0);
  Step4.setMaxSpeed(200.0);
  Step4.setAcceleration(100.0);
//  delay(1000);
  // put your setup code here, to run once:
  
}

void loop() {
  if(state == STATE_RESET_ENCODERS){

    Step1.setCurrentPosition(0);
    Step2.setCurrentPosition(0);
    Step3.setCurrentPosition(0);
    Step4.setCurrentPosition(0);

  } 
  if(state == STATE_MOVE_ARM){

  }
  //delay(100);
  // put your main code here, to run repeatedly:

}