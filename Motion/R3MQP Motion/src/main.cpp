#include <Arduino.h>
#include <AccelStepper.h>

int STATE_RESET_ENCODERS = 0;
int STATE_MOVE_ARM = 1;

//From the settings on the Drivers
const float ticksPerRotation = 1000;

//For each motor, including the gearbox on the motor
float GearRatio[4] = {19,25,20,1};

//This controls whether the robot is in calibration state or motion state
int state = 0;

//This stores the time when we should check for new serial messages from the kinematics code
int serialTimer = 0;

//Limit Switch Tracker
int switchHit = 0;

//Pins for the limit switches
int buttonPins[4] = {23,18,21,22};

//For recieving Serial data from kinematics
char receivedChars[32] = {};
bool settingsReady = false;

//This tells which motors have already been calibrated
int motorsReset = -1;


//Creates the four stepper motors
AccelStepper Step1(AccelStepper::FULL2WIRE, 26,27);
AccelStepper Step2(AccelStepper::FULL2WIRE,12,14);
AccelStepper Step3(AccelStepper::FULL2WIRE, 25,33);
AccelStepper Step4(AccelStepper::FULL2WIRE, 13,32);

AccelStepper Motors[4] = {Step1,Step2,Step3,Step4};

//Positive and Negative soft limits for each motor
int MotorsSoftLimitPositive[4] = {90,90,60,90};
int MotorsSoftLimitNegative[4] = {-90,-45,-90,-90};

//For storing the current motor target setpoints
double motorSetpoints[4] = {0,0,0,0};

//Input: SwitchID (0-4)
//Output: The status of the requested limit switch (True = Pressed)
bool getLimitSwitch(int switchID){
  return digitalRead(buttonPins[switchID]);
}

//Input: motorID (0-4), position (in motor ticks)
//Moves the desired motor to the desired position without checking software limits
void moveMotorUnprotected(int motorID, double position){
  if(motorID == 1 || motorID == 0){
    Motors[motorID].moveTo(-position);
    Motors[motorID].run();
  } else{
    Motors[motorID].moveTo(position);
    Motors[motorID].run();
  }
}

//Input: motorPositions (4 value array, in motor ticks)
//Moves all motors to the desired positions without checking software limits 
void moveAllMotorsUnprotected(double motorPositions[]){
  for(int i = 0; i < 4; i++){
    moveMotorUnprotected(i,motorPositions[i]);
  }
}

//Input: motorPositions (4 value array, in degrees)
//Moves all motors to the desired positions without check software limits
void moveAllMotorsUnprotectedDegrees(double motorPositions[]){
  for(int i = 0; i < 4; i++){
    moveMotorUnprotected(i,motorPositions[i]*ticksPerRotation*GearRatio[i]/90);
  }
}


//Input: motorID (0-4), position (in motor ticks)
//Moves the desired motor to the desired position while checking software limits for each motor
void moveMotorProtected(int motorID, double position){
  if(position > MotorsSoftLimitPositive[motorID]){
    moveMotorUnprotected(motorID,MotorsSoftLimitPositive[motorID]);

  } else if(position < MotorsSoftLimitNegative[motorID]){
    moveMotorUnprotected(motorID,MotorsSoftLimitNegative[motorID]);
  } else{
    moveMotorUnprotected(motorID,position);
  }
}

//Input: motorPositions (4 value array, in degrees)
//Moves all motors to the desired positions while checking software limits for each motor
void moveAllMotorsProtectedDegrees(double motorPositions[]){
  for(int i = 0; i < 4; i++){
    moveMotorProtected(i,motorPositions[i]*ticksPerRotation*GearRatio[i]/90);
  }
}

//Input: motorID(0-3), FWD (should the motor move forward or backward to hit the switch?)
//The selected motor will move until it hits the switch, where it will stop and set the motorsReset variable to indicate that it has been reset
void moveMotorToSwitch(int motorID, bool FWD){
  //int switchPositions[4] = {-170, -45, -135, -225}; 14
    int switchPositions[4] = {90, -46, 111, -225};
  int mult = 0;
  if(FWD){
    mult = 1;
  } else{
    mult = -1;
  }

  if(getLimitSwitch(motorID)){
    switchHit ++;
    //Serial.print (switchHit);
  } else{
    switchHit = 0;
  }

  if(switchHit > 3 || motorID==3){
    Motors[motorID].setCurrentPosition(switchPositions[motorID]*ticksPerRotation*GearRatio[motorID]/90);
    //Add the precise positions to set. Base = 180, Shoulder = ?, Elbow = 135, Wrist = 225
    motorsReset = motorID;
    Motors[motorID].moveTo(0);
    Motors[motorID].run();
    //Serial.print(motorsReset);
    switchHit = 0;
  } else{
    Motors[motorID].move(1000*mult);
    Motors[motorID].run();
    for(int i = motorID-1; i >= 0; i--){
      Motors[i].moveTo(0);
      Motors[i].run();
    }
  }
}


//Input: motorID(0-4), setpoint(in degrees)
//Sets the desired motor setpoint to the current angle + the desired change
void changeMotorSetpoint(int motorID, int setpoint){
  motorSetpoints[motorID] = motorSetpoints[motorID] + setpoint;
}

//Input: motorID(0-4), setpoint(in degrees)
//Sets the desired motor setpoint to the desired angle
void setMotorSetpoint(int motorID, int setpoint){
  motorSetpoints[motorID] = setpoint;
}


//Sets all motors to stop at their current positions
void stopAllMotors(){
  for(int i = 0; i < 4; i++){
    Motors[i].stop();
  }
}

//Fetches data from the Serial buffer
int readSerial(){
  if (Serial.available() > 0) {

    // read the incoming byte:

    int incomingByte = Serial.read();


    // say what you got:

    return incomingByte;
  }
  return 0;
}

//Processes any serial data ending with ">" and stores it in "recievedChars"
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


//Takes the serial data from recievedChars and sets the motor setpoints to the recieved data
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

    setMotorSetpoint(jointID-1,set);

    settingsReady = false;

  }
}

void setup() {
  pinMode(23,INPUT);
  pinMode(22,INPUT);
  pinMode(21,INPUT);
  pinMode(18,INPUT);
  Serial.begin(115200);
  state = STATE_RESET_ENCODERS;
  Motors[0].setMaxSpeed(2000.0);
  Motors[0].setAcceleration(500.0);
  Motors[1].setMaxSpeed(2000.0);
  Motors[1].setAcceleration(500.0);
  Motors[2].setMaxSpeed(2000.0);
  Motors[2].setAcceleration(500.0);
  Motors[3].setMaxSpeed(2000.0);
  Motors[3].setAcceleration(500.0);
//  delay(1000);
  // put your setup code here, to run once:
  serialTimer = millis();
}


void loop() {
//  Serial.print(digitalRead(getLimitSwitch(0)));
  if(state == STATE_RESET_ENCODERS){
    double motorSettings[4] = {0,0,0,0};
    bool motorFWD[4] = {true,false,true,false};
    if(motorsReset < 3){
      /*Serial.print(" ");
      Serial.print(getLimitSwitch(motorsReset + 1));
      Serial.print(" ");
      Serial.print(Motors[0].currentPosition());
      Serial.print(" ");
      Serial.print(getLimitSwitch(0));
      Serial.print(" ");
      Serial.print(getLimitSwitch(1));
      Serial.print(" ");
      Serial.print(getLimitSwitch(2));
      Serial.print(" ");
      Serial.println(getLimitSwitch(3));*/
      moveMotorToSwitch(motorsReset+1,motorFWD[motorsReset+1]);
    }
    
    if(motorsReset >= 3){
      if(Motors[0].currentPosition() == 0 && Motors[1].currentPosition() == 0 && Motors[2].currentPosition() == 0 && Motors[3].currentPosition() == 0){
        state = STATE_MOVE_ARM;
        stopAllMotors();
      }else{
        double targets[] = {0,0,0,0};
        moveAllMotorsProtectedDegrees(targets);
      }
    }

    
  }  
  if(state == STATE_MOVE_ARM){
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

  moveAllMotorsUnprotectedDegrees(motorSetpoints);

  //1 Base Left
  //Q Shoulder Out
  //A Elbow Up
  //Z Wrist Clockwise
  // put your main code here, to run repeatedly:
  }
}