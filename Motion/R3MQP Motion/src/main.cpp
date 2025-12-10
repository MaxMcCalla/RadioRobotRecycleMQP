#include <Arduino.h>
#include <AccelStepper.h>

const int STATE_RESET_ENCODERS = 0;
const int STATE_MOVE_ARM = 1;

int state;

int serialTimer = 0;

int buttonPins[4] = {2,4,5,18};

int ticksToDegrees[4] = {1,1,1,1};

char receivedChars[32] = {};
bool settingsReady = false;

// put function declarations here:
AccelStepper Step1(AccelStepper::FULL2WIRE,12,14);
AccelStepper Step2(AccelStepper::FULL2WIRE, 27,26);
AccelStepper Step3(AccelStepper::FULL2WIRE, 25,33);
AccelStepper Step4(AccelStepper::FULL2WIRE, 13,32);

AccelStepper Motors[4] = {Step1,Step2,Step3,Step4};

double motorSetpoints[4] = {0,0,0,0};

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

void moveAllMotorsUnprotected(double motorPositions[]){
  for(int i = 0; i < 4; i++){
    moveMotorUnprotected(i,motorPositions[i]);
  }
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

void changeMotorSetpoint(int motorID, int setpoint){
  motorSetpoints[motorID] = motorSetpoints[motorID] + setpoint;
}

void setMotorSetpoint(int motorID, int setpoint){
  motorSetpoints[motorID] = setpoint;
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
  serialTimer = millis();
}

void recvWithEndMarker() {
    int numChars = 32;
    static byte ndx = 0;
    char endMarker = '>';
    char rc;
    
    while (Serial.available() > 0 && settingsReady == false) {
        rc = Serial.read();

        if (rc != endMarker) {
            receivedChars[ndx] = rc;
            ndx++;
            if (ndx >= numChars) {
                ndx = numChars - 1;
            }
        }
        else {
            receivedChars[ndx] = '\0'; // terminate the string
            ndx = 0;
            settingsReady = true;
        }
    }
}

void useSettings(){
  if(settingsReady){
    char j[1] = {};
    j[0] = receivedChars[0];
    int jointID = atoi(j);
    char a[31] = {};
    for(int i = 1; i < 32; i++){
      a[i-1] = receivedChars[i];
    }

    float set = atof(a);

    Serial.println(jointID);
    Serial.println(set);

    setMotorSetpoint(jointID,set);

    settingsReady = false;

  }
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
    
  //}
  //delay(100);



  if(millis() - serialTimer > 100){
    //q 81/113, w 87/119, a 65/97, s 83/115, z 90/122, x 88/120, 1 49 2 50  

    recvWithEndMarker();
    useSettings();
    
    /*int in = readSerial();
    if(in != 0){
      Serial.println(in);
      switch(in){
        case 113:
          changeMotorSetpoint(0,100);
          break;
        case 119:
          changeMotorSetpoint(0,-100);
          break;
        case 97:
          changeMotorSetpoint(2,100);
          break;
        case 115:
          changeMotorSetpoint(2,-100);
          break;
        case 122:
          changeMotorSetpoint(3,100);
          break;
        case 120:
          changeMotorSetpoint(3,-100);
          break;
        case 49:
          changeMotorSetpoint(1,100);
          break;
        case 50:
          changeMotorSetpoint(1,-100);
          break;
        default:
          break;
      }
    }
    serialTimer = millis();*/



  }

  moveAllMotorsUnprotected(motorSetpoints);

  //1 Base Left
  //Q Shoulder Out
  //A Elbow Up
  //Z Wrist Clockwise
  // put your main code here, to run repeatedly:

}