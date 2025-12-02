#include <Arduino.h>
#include <AccelStepper.h>

const int STATE_RESET_ENCODERS = 0;
const int STATE_MOVE_ARM = 1;

int state;

int buttonPins[4] = {2,4,5,18};

int ticksToDegrees[4] = {1,1,1,1};

// put function declarations here:
AccelStepper Step1(AccelStepper::FULL2WIRE,12,14);
AccelStepper Step2(AccelStepper::FULL2WIRE, 27,26);
AccelStepper Step3(AccelStepper::FULL2WIRE, 25,33);
AccelStepper Step4(AccelStepper::FULL2WIRE, 32,35);

AccelStepper Motors[4] = {Step1,Step2,Step3,Step4};

bool getLimitSwitch(int switchID){
  //return digitalRead(buttonPins[switchID]);
  return true;
}

void resetMotors(){
  for(int i = 0; i < sizeof(Motors); i++){
    Motors[i].setCurrentPosition(0);
  }
}



void moveMotor(int motorID, double position){
  if(!getLimitSwitch(motorID)){
    Motors[motorID].moveTo(position);
    Motors[motorID].run();
  } else{
    Motors[motorID].stop();    
  }
}

void moveMotorUnprotected(int motorID, double position){
    Motors[motorID].moveTo(position);
    Motors[motorID].run();
}

void moveMotorToSwitch(int motorID, bool FWD){
  int mult = 0;
  if(FWD){
    mult = 1;
  } else{
    mult = -1;
  }
  if(!getLimitSwitch(motorID)){
    Motors[motorID].move(100*mult);
  } else{
    Motors[motorID].stop();
  }
}

void moveAllMotors(double motorPositions[]){
  for(int i = 0; i < 4; i++){
    moveMotor(i,motorPositions[i]);
  }
}


void stopAllMotors(){
  for(int i = 0; i < sizeof(Motors); i++){
    Motors[i].stop();
  }
}

int readSerial(){
  if (Serial.available() > 0) {

    // read the incoming byte:

    int incomingByte = Serial.read();


    // say what you got:

    return incomingByte;
  }
  return 0;
}


void setup() {
  Serial.begin(115200);
  state = STATE_RESET_ENCODERS;
  Motors[0].setMaxSpeed(400.0);
  Motors[0].setAcceleration(200.0);
  Motors[1].setMaxSpeed(400.0);
  Motors[1].setAcceleration(200.0);
  Motors[2].setMaxSpeed(400.0);
  Motors[2].setAcceleration(200.0);
  Motors[3].setMaxSpeed(400.0);
  Motors[3].setAcceleration(200.0);
//  delay(1000);
  // put your setup code here, to run once:
  
}

void loop() {
  /*if(state == STATE_RESET_ENCODERS){
    double motorSettings[4] = {0,0,0,0};
    moveMotorToSwitch(0,true);
    moveMotorToSwitch(1,true);
    moveMotorToSwitch(2,true);
    moveMotorToSwitch(3,true);
    
    if(getLimitSwitch(0) && getLimitSwitch(1) && getLimitSwitch(2) && getLimitSwitch(3)){
      state = STATE_MOVE_ARM;
      stopAllMotors();
    }

  }  
  if(state == STATE_MOVE_ARM){*/
    //1 - Base
    //0 - Shoulder
    //2 - Elbow
    //3 - Wrist

    //moveMotorUnprotected(1,-1000);
    
    moveMotorUnprotected(0,1000);
    moveMotorUnprotected(1,-1000);
    moveMotorUnprotected(2,2000);
    //moveMotorUnprotected(3,-200);
  //}
  //delay(100);
  //int in = readSerial();
  //if(in != 0){
  //Serial.println(in);
 // }
  // put your main code here, to run repeatedly:

}