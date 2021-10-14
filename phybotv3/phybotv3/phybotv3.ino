#include "AccelStepper.h"
#include <Wire.h>
#include <EEPROM.h>
//#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#define rangohombro 7000
#define rangocodo 5850
#define rangomuneca 1000
#define rangobase 100

//#define I2C_ADDR    0x27
//LiquidCrystal_I2C             lcd(I2C_ADDR,2, 1, 0, 4, 5, 6, 7);
const int TOTAL_MOTORS  = 4;
const int marcador1=0;
const int finalsw= A3;
const int Pasomin[TOTAL_MOTORS]   = {0,0,0,0};
const int Pasomax[TOTAL_MOTORS]   = {10903,       10301,      8702,       4900};
const int PasoTope[TOTAL_MOTORS]  = {Pasomax[0],  Pasomin[1], Pasomin[2], Pasomax[3]};
const int PosIdle[TOTAL_MOTORS]   = {5400, 2000, 1000, 2500};
const int Velocid[TOTAL_MOTORS]   = {1600, 1600, 1600 ,1600};
const float motorMaxSpeed[TOTAL_MOTORS] = {3500.0,    3500.0,   3500.0,   3500.0};
const float motorMinSpeed[TOTAL_MOTORS] = {150.0,     600.0,    800.0,    800.0};
const float motorAccel[TOTAL_MOTORS]    = {2000.0,    2000.0,   2000.0,   2000.0};
//const float stepsPerDegree[TOTAL_MOTORS]= {34.0750,   35.5555,  33.5570,  26.6666};

#define SHOULDER_MOTOR   0
#define ELBOW_MOTOR      1
#define WRIST_MOTOR      2
#define BASE_MOTOR       3
// Computed from calibration
static float stepsPerDeg[TOTAL_MOTORS]  =  {0,      0,        0,        0}; 
static long stepPosAt90 [TOTAL_MOTORS]  =  {5763,   5551,   4500,        2400};
static long stepPosAt0 [TOTAL_MOTORS]   =  {1752,   2551,   8166,        0};
static int  homing_order[TOTAL_MOTORS]  =  {WRIST_MOTOR, ELBOW_MOTOR,SHOULDER_MOTOR , BASE_MOTOR};

/*static float stepsPerDeg[TOTAL_MOTORS]  =  {0,      0,        0,        0}; 
static long stepPosAt90 [TOTAL_MOTORS]  =  {0,      0,        0,        0};
static long stepPosAt0 [TOTAL_MOTORS]   =  {0,      0,        0,        0};
static int  homing_order[TOTAL_MOTORS]  =  {WRIST_MOTOR, ELBOW_MOTOR,SHOULDER_MOTOR , BASE_MOTOR};
*/



const unsigned char motorCmdIdentifier[TOTAL_MOTORS]  = {'s', 'e', 'w', 'b'};
const int EEPROM_CALIBRATION_DATA_ADDRESS             = 0;
const int EEPROM_DEFAULTPOSITION_ADDRESS              = EEPROM_CALIBRATION_DATA_ADDRESS + 8*TOTAL_MOTORS;

// Used by IK
const float ik_base_val     = 350;           
const float ik_shoulder_val = 320;
const float ik_elbow_val    = 211;
const float ik_wrist_val    = 50;
const float ik_clamp_val    = 150;
const float pi              = atan(1.0)*4;    
const float rad             = pi/180;
const float grad            = 180/pi;  

// Motors
Servo        handServo;
Servo        gripperServo;
AccelStepper stepper1(AccelStepper::DRIVER, 8, 7);
AccelStepper stepper2(AccelStepper::DRIVER, 6, 16);
AccelStepper stepper3(AccelStepper::DRIVER, 4, 3);
AccelStepper stepper4(AccelStepper::DRIVER, 13, 12);

AccelStepper *motors[] = {&stepper1, &stepper2, &stepper3, &stepper4};

long readLongFromEEPROM (unsigned int address){
  unsigned long value = 0;
  value |= EEPROM.read(address);
  value |= EEPROM.read(address+1) << 8;
  value |= EEPROM.read(address+2) << 16;
  value |= EEPROM.read(address+3) << 24;
  return value;
}



void writeLongToEEPROM (unsigned int address, long val){
  unsigned long uvalue = (unsigned long)val;
  EEPROM.write(address,   (byte)(uvalue & 0xff));
  EEPROM.write(address+1, (byte)((uvalue>>8) & 0xff));
  EEPROM.write(address+2, (byte)((uvalue>>16) & 0xff));
  EEPROM.write(address+3, (byte)((uvalue>>24) & 0xff));
}

void computeAngleConversion(){
  for (int i = 0; i < TOTAL_MOTORS; i++){
    stepsPerDeg[i] = (float)(stepPosAt90[i] - stepPosAt0[i]) / (float)90.0;
  }
}

void stopmotors(){

  for (int i = 0; i < TOTAL_MOTORS; i++){
    motors[i]->stop();
    
  }

 }


bool inverseKinematic(float x, float y, float z, float pitch, float spd, bool syncRoll){
  long posArray[TOTAL_MOTORS];
  float baseAngle = (atan2(y,x))*grad;
  
  float modulo = sqrt(abs(x*x) + abs(y*y));
  float xprima = modulo - 37 ;
  float yprima = z;
  float Afx = cos(rad*pitch)*ik_wrist_val;
  float ladob=xprima -Afx;
  float Afy = sin(rad*pitch)*ik_wrist_val;
  float ladoa=yprima -Afy - ik_base_val;
  float hipotenusa = sqrt((ladoa*ladoa)+(ladob*ladob));
  float Alfa = atan2(ladoa,ladob);
  float Beta= acos(((ik_shoulder_val*ik_shoulder_val)-(ik_elbow_val*ik_elbow_val)+(hipotenusa*hipotenusa))/(2*ik_shoulder_val*hipotenusa));
  
  
  float shoulderAngle = (Alfa + Beta)*grad;
  float Gamma = acos(((ik_shoulder_val*ik_shoulder_val)+(ik_elbow_val*ik_elbow_val)-(hipotenusa*hipotenusa))/(2*ik_shoulder_val*ik_elbow_val));
  float elbowAngle = (-((180*rad)-Gamma))*grad;
  float wristAngle = pitch - shoulderAngle - elbowAngle;

  baseAngle     += 180;
  elbowAngle    += 90;
  wristAngle    += 90;

  posArray[SHOULDER_MOTOR]  = angleToSteps(SHOULDER_MOTOR, shoulderAngle);
  posArray[ELBOW_MOTOR]     = angleToSteps(ELBOW_MOTOR, elbowAngle);
  posArray[WRIST_MOTOR]     = angleToSteps(WRIST_MOTOR, wristAngle);
  posArray[BASE_MOTOR]      = angleToSteps(BASE_MOTOR, baseAngle);

  if (syncRoll) moveHandServo (baseAngle);
  return moveMotorSimultaneously (posArray, spd);
}

void readCalibration(){
  unsigned int address = EEPROM_CALIBRATION_DATA_ADDRESS;
  for (int i = 0; i < TOTAL_MOTORS; i++){
    stepPosAt0[i]  = readLongFromEEPROM (address);
    stepPosAt90[i] = readLongFromEEPROM (address+4);
    address += 8;
  }
  computeAngleConversion();
}

void writeCalibration(){
  unsigned int address = EEPROM_CALIBRATION_DATA_ADDRESS;
  for (int i = 0; i < TOTAL_MOTORS; i++){
    writeLongToEEPROM (address    , stepPosAt0[i]);
    writeLongToEEPROM (address+4  , stepPosAt90[i]);
    address += 8;
  }
}

void homing(){
  int m, i;

  for (i=0; i < TOTAL_MOTORS; i++){
    m = homing_order[i];
    //motors[m]->setMaxSpeed(motorMaxSpeed[m]/2);
    motors[m]->setMaxSpeed(1000);
    motors[m]->moveTo((PasoTope[m] == Pasomin[m] ? -50000 : 50000));
    
    while(analogRead(finalsw)  < 100 ){
      motors[m]->run();
      }
    
    motors[m]->stop();
    motors[m]->setCurrentPosition(PasoTope[m]);
    delay(10);
    motors[m]->runToNewPosition(PosIdle[m]);
  }
}

int motorIdToNumber (unsigned char id){
  for (int i=0; i < TOTAL_MOTORS; i++){
    if (motorCmdIdentifier[i] == id) return i;
  }
  return -1;
}

bool moveMotorSteps(int motor, long targetStep, float movementSpeed){
  if (motor >= TOTAL_MOTORS || motor < 0) return false;
  if (targetStep > Pasomax[motor] || targetStep < Pasomin[motor]) return false;

  motors[motor]->setMaxSpeed(constrain(movementSpeed, motorMinSpeed[motor], motorMaxSpeed[motor]));
  motors[motor]->runToNewPosition(targetStep);
  return true;
}

bool moveMotorSimultaneously(long targetPos[], float movementSpeed){
  bool motorsDone = false;
  float motorTime, longestTime = 0;
  float spd;
  byte i;

  // Verify limits
  for (i = 0; i < TOTAL_MOTORS; i++){
    if (targetPos[i] > Pasomax[i] || targetPos[i] < Pasomin[i]) return false;
  }

  // Find the time it would take to the each motor to complete the motion and store the longest time
  for (i = 0; i < TOTAL_MOTORS; i++) {
    motorTime = (float)(targetPos[i] - getMotorPosition(i)) / constrain(movementSpeed, motorMinSpeed[i], motorMaxSpeed[i]);
    //motorTime = (float)(targetPos[i] - getMotorPosition(i)) / movementSpeed;
    if (motorTime > longestTime) longestTime = motorTime;
  }
 
  // Set speeds and assign target positions
  for (i = 0; i < TOTAL_MOTORS; i++) {
    spd = (float)(targetPos[i] - getMotorPosition(i))/longestTime;
    //spd = constrain((targetPos[i] - getMotorPosition(i))/longestTime, motorMinSpeed[i], motorMaxSpeed[i]);
    motors[i]->setMaxSpeed(spd);
    motors[i]->setSpeed(spd);
    
    motors[i]->moveTo(targetPos[i]);
  }

  // Move motors
  while (!motorsDone){
    motorsDone = true;
    for (i = 0; i < TOTAL_MOTORS; i++) {
      motors[i]->run();
      if (motors[i]->distanceToGo() != 0) motorsDone = false;
    }
  }
  return true;
}


void moveHandServo (float angle){
  handServo.write(angle);
}

void moveGripperServo (float angle){
  gripperServo.write(angle);
}

long getMotorPosition(int motor){
  if (motor >= TOTAL_MOTORS || motor < 0) return 0;
  return motors[motor]->currentPosition();
}

// Returns the current position in 1/100th of degree
long getMotorPositionAngle(int motor){
  if (motor >= TOTAL_MOTORS || motor < 0) return 0;
  return (motors[motor]->currentPosition() - stepPosAt0[motor])*100 / stepsPerDeg[motor];
}

long angleToSteps(int motor, float targetAngle){
  if (motor >= TOTAL_MOTORS || motor < 0) return 0.0;

  return stepPosAt0[motor] + targetAngle*stepsPerDeg[motor];
}

// CONFIGURACION 
void setup() {   
  //lcd.begin (16,1);
 // lcd.setBacklightPin(3,POSITIVE);
 // lcd.setBacklight(HIGH); 
 // lcd.home ();   
//  lcd.print("Phyto Robotic ");
//  lcd.begin (16,2);
//  lcd.print("arm 5 axis ");
  
    
  
  for (int i = 0; i < TOTAL_MOTORS; i++) {
    motors[i]->setAcceleration (motorAccel[i]);
  }

  handServo.attach(9);
  gripperServo.attach(11);
  pinMode(finalsw, INPUT); 
  Serial.begin(115200);

  //homing();
  //readCalibration();
  computeAngleConversion();
  moveHandServo (0);
  moveGripperServo (0);
}

int blockingSerialRead(){
  while (!Serial.available()) {
    // Loop until a character is available
  }
  return Serial.read();
}

// the loop routine runs over and over again forever:
void loop(){
  //displayss();

  int i, cmd, targetMotor, cmd2, cmd3;
  bool success = false;
  long targetPos;
  float x, y, z, pitch, spd;
  bool readInDegrees;
  long posArray[TOTAL_MOTORS];
  

  cmd = blockingSerialRead();
  switch (cmd){
    case 'H':
      // Homing
      homing();
      success = true;
      break;

    case 'F':
      if (blockingSerialRead() != ' ') break;
      x = (float)Serial.parseInt() / 100.0;
      moveHandServo(x);
      success = true;
      break;

    case 'G':
      if (blockingSerialRead() != ' ') break;
      x = (float)Serial.parseInt() / 100.0;
      moveGripperServo(x);
      success = true;
      break;

    case 'I':
        if (blockingSerialRead() != 'K') break;
        if (blockingSerialRead() != ' ') break;
        x = Serial.parseFloat();
        if (blockingSerialRead() != ' ') break;
        y = Serial.parseFloat();
        if (blockingSerialRead() != ' ') break;
        z = Serial.parseFloat();       
        if (blockingSerialRead() != ' ') break;
        pitch = Serial.parseFloat(); 
        if (blockingSerialRead() != ' ') break;
        spd = Serial.parseFloat();
        if (blockingSerialRead() != ' ') break;
        cmd2 = blockingSerialRead();   //Options: S= Sync hand servo with base rotation. Anything else: Don't sync

        success = inverseKinematic (x, y, z, pitch, spd, (cmd2 == 'S'));
      break;

    case 'M':
      cmd2 = blockingSerialRead();
      if (cmd2 == 's'){
        readInDegrees = false;
      }else if (cmd2 == 'a'){
        readInDegrees = true;
      }else {
        break;
      }

      for (i = 0; i < TOTAL_MOTORS; i++){
        if (readInDegrees){
          posArray[i] = angleToSteps(i, (float)Serial.parseInt() / 100.0);
        }else {
          posArray[i] = Serial.parseInt();
        }
        if (blockingSerialRead() != ' ') break;
      }  
      x = (float)Serial.parseInt() / 100.0;     // Roll
      if (blockingSerialRead() != ' ') break;
      y = (float)Serial.parseInt() / 100.0;     // Grip
      if (blockingSerialRead() != ' ') break;
      spd = Serial.parseFloat();                // Speed

      moveHandServo(x);
      moveGripperServo(y);
      success = moveMotorSimultaneously (posArray, spd);
      break;
    
        
    // Calibration command
    case 'C':
      targetMotor = motorIdToNumber (blockingSerialRead());
      if (targetMotor == -1) break;
 
      success = true;
      cmd2 = blockingSerialRead();
      if (cmd2 == '0'){
        stepPosAt0[targetMotor] = getMotorPosition(targetMotor);
        writeCalibration();
        computeAngleConversion();
      }else if (cmd2 == '9' && blockingSerialRead() == '0'){
        stepPosAt90[targetMotor] = getMotorPosition(targetMotor);
        writeCalibration();
        computeAngleConversion();
      }else{
        success = false;
      }
      break;
    
    case 'R': // Read current position of each motor
      for (i = 0; i < TOTAL_MOTORS; i++){

        Serial.print(getMotorPosition(i), DEC);
        Serial.print(" ");
        Serial.print(getMotorPositionAngle(i), DEC);
        Serial.print(" ");
      }
      Serial.print((long)(handServo.read()*100));
      Serial.print(" ");
      Serial.print((long)(gripperServo.read()*100));
      Serial.println("");
      
      /*
      //para obtener variables
      for (int hola ; hola < TOTAL_MOTORS; hola++){
      
      Serial.print(stepPosAt90[hola], DEC);
      Serial.print(" ");
      Serial.print(stepPosAt0[hola], DEC);
      Serial.print(" ");}
      */
      
      success = true;
      break;
      

           
    case 'S': // Move by step
    case 'A': // Move by angle
      targetMotor = motorIdToNumber (blockingSerialRead());
      if (targetMotor == -1) break;
      if (blockingSerialRead() != ' ') break;
      // If the command is "angle" we expect a float value to follow (and we will convert that to a target step)
      if (cmd == 'A'){
        targetPos = angleToSteps(targetMotor, (float)Serial.parseInt() / 100.0);
      // Otherwise we expect to receive an integer
      }else {
        targetPos = Serial.parseInt();
      }
      if (blockingSerialRead() != ' ') break;
      x = Serial.parseFloat();
      success = moveMotorSteps (targetMotor,targetPos , x);
      break;

    case 'W':
      if (blockingSerialRead() != ' ') break;
      i = Serial.parseInt();
      delay(i);
      success = true;
      
      
    case 'T':
      
      int targetPos2[4];
      
      while(1)
      {
        bool motorsDone2 = false;
        cmd3 = blockingSerialRead();
       
      if (cmd3 == 'R'){
      for (i = 0; i < TOTAL_MOTORS; i++){

      Serial.print(getMotorPosition(i), DEC);
      Serial.print(" ");
      Serial.print(getMotorPositionAngle(i), DEC);
      Serial.print(" ");
      }
      Serial.print((long)(handServo.read()*100));
      Serial.print(" ");
      Serial.print((long)(gripperServo.read()*100));
      Serial.println("");
      }
      
      if (blockingSerialRead() == 'A'){  
      
      targetPos2[0] = Serial.parseInt();
      if (blockingSerialRead() != ' ') break;
      targetPos2[1] = Serial.parseInt();
      if (blockingSerialRead() != ' ') break;
      targetPos2[2] = Serial.parseInt();
      if (blockingSerialRead() != ' ') break;
      targetPos2[3] = Serial.parseInt();
      if (blockingSerialRead() != ' ') break;
      
      for (i = 0; i < TOTAL_MOTORS; i++) {
      
      motors[i]->setSpeed(Velocid[i]);
      motors[i]->moveTo(targetPos2[i]); 
      }
      
    while (!motorsDone2){
      
    motorsDone2 = true;
    for (i = 0; i < TOTAL_MOTORS; i++) {
    motors[i]->run();
    if (motors[i]->distanceToGo() != 0) motorsDone2 = false;
    }
    }

    
    }

      
      } //fin while
    
    
      
  }
  
  //fin switch
  // Wait for a newline character
  while ((cmd != '\r') && (cmd != -1)) cmd = Serial.read();

  // Return the result of the operation
  if (success) {
    Serial.println("OK");
  }else {
    Serial.println("ERROR");
  }
}

