#include "turnMotors.h"
#include "constants.h"
#include <PID_v1.h>
#include <Adafruit_MotorShield.h>
#include <Wire.h>

extern TwoWire Wire2; // Borrowing mpu6050 i2c bus because the pins for the default bus conflict with the FL simpleFOC pins

float errorFL = 0;
float errorFR = 0;
float errorBL = 0;
float errorBR = 0;

// Azimuth motors & encoders
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *FRMotor = AFMS.getMotor(4);
const int FRSensor = PC1;

Adafruit_DCMotor *FLMotor = AFMS.getMotor(1);
const int FLSensor = PA4;

Adafruit_DCMotor *BRMotor = AFMS.getMotor(3);
const int BRSensor = PC2;

Adafruit_DCMotor *BLMotor = AFMS.getMotor(2);
const int BLSensor = PC0;



//PID stuff-------------------
int PIDUpperLimit = 255;
int PIDLowerLimit = -255;

double SetpointFR, InputFR, OutputFR;
double FRKp=0.85, FRKi=0, FRKd=0;
PID FRPID(&InputFR, &OutputFR, &SetpointFR, FRKp, FRKi, FRKd, REVERSE);

double SetpointFL, InputFL, OutputFL;
float FLKp=0.85, FLKi=0, FLKd=0;
PID FLPID(&InputFL, &OutputFL, &SetpointFL, FLKp, FLKi, FLKd, REVERSE);

double SetpointBR, InputBR, OutputBR;
double BRKp=0.85, BRKi=0, BRKd=0;
PID BRPID(&InputBR, &OutputBR, &SetpointBR, BRKp, BRKi, BRKd, DIRECT);

double SetpointBL, InputBL, OutputBL;
double BLKp=0.85, BLKi=0, BLKd=0;
PID BLPID(&InputBL, &OutputBL, &SetpointBL, BLKp, BLKi, BLKd, DIRECT);
// ---------------------------



void setupTurnMotors() {
  

  // if (!AFMS.begin(1600, &Wire2)) {         // create with the default frequency 1.6KHz
  //  //if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
  //   Serial.println("Could not find Motor Shield. Check wiring.");
  //   while (1);
  // }
  AFMS.begin(1600, &Wire2); //same i2c as the mpu6050
  //Serial.println("Motor Shield found.");

  // // PID Setup
  FRPID.SetMode(AUTOMATIC);
  FRPID.SetOutputLimits(PIDLowerLimit, PIDUpperLimit);

  FLPID.SetMode(AUTOMATIC);
  FLPID.SetOutputLimits(PIDLowerLimit, PIDUpperLimit);

  BRPID.SetMode(AUTOMATIC);
  BRPID.SetOutputLimits(PIDLowerLimit, PIDUpperLimit);

  BLPID.SetMode(AUTOMATIC);
  BLPID.SetOutputLimits(PIDLowerLimit, PIDUpperLimit);
  
  // Sensor Setups
  pinMode(FRSensor, INPUT);
  pinMode(FLSensor, INPUT);
  pinMode(BRSensor, INPUT);
  pinMode(BLSensor, INPUT);
}

void turnModuleFR(double angle){
  SetpointFR = angle;

  // float mappedAngleFR = map(analogRead(FRSensor), 0, 1023, 0, 360);
  InputFR = getAngleFR();

  errorFR = InputFR - SetpointFR;

  if (errorFR >= 180) {
    SetpointFR += 360;
  } else if (errorFR <= -180) {
    SetpointFR -= 360;
  }
  
  FRPID.Compute();

  // For debugging
  // Serial.print(SetpointFR);
  // Serial.print("  ");
  // Serial.print(InputFR);
  // Serial.print("  ");
  // Serial.print(OutputFR);

  if (OutputFR > 0){
    FRMotor->run(FORWARD);
    FRMotor->setSpeed(abs(OutputFR));
  }else if (OutputFR < 0){
    FRMotor->run(BACKWARD);
    FRMotor->setSpeed(abs(OutputFR));
  }else{
    FRMotor->setSpeed(0);
    FRMotor->run(RELEASE);
  }
}

void turnModuleFL(double angle){
  FLPID.SetTunings(FLKp, FLKi, FLKd);

  SetpointFL = angle;

  //float mappedAngleFL = map(analogRead(FLSensor), 0, 1023, -180, 180);
  InputFL = getAngleFL();
  
  errorFL = InputFL - SetpointFL;

  if (errorFL >= 180) {
    SetpointFL += 360;
  } else if (errorFL <= -180) {
    SetpointFL -= 360;
  }

  FLPID.Compute();

  // For debugging
  // Serial.print(SetpointFL);
  // Serial.print("  ");
  // Serial.print(InputFL);
  // Serial.print("  ");
  // Serial.print(OutputFL);

  if (OutputFL > 0){
    FLMotor->run(FORWARD);
    FLMotor->setSpeed(abs(OutputFL));
  }else if (OutputFL < 0){
    FLMotor->run(BACKWARD);
    FLMotor->setSpeed(abs(OutputFL));
  }else{
    FLMotor->setSpeed(0);
    FLMotor->run(RELEASE);
  }
}

void turnModuleBR(double angle){
  SetpointBR = angle;

  // float mappedAngleBR = map(analogRead(BRSensor), 0, 1023, 0, 360);
  InputBR = getAngleBR();

  errorBR = InputBR - SetpointBR;

  if (errorBR > 180) {
    SetpointBR += 360;
  } else if (errorBR < -180) {
    SetpointBR -= 360;
  }

  BRPID.Compute();

  // For debugging
  // Serial.print(SetpointBR);
  // Serial.print("  ");
  // Serial.print(InputBR);
  // Serial.print("  ");
  // Serial.print(OutputBR);
  
  if (OutputBR > 10){
    BRMotor->run(FORWARD);
    BRMotor->setSpeed(abs(OutputBR));
  }else if (OutputBR < 10){
    BRMotor->run(BACKWARD);
    BRMotor->setSpeed(abs(OutputBR));
  }else{
    BRMotor->setSpeed(0);
    BRMotor->run(RELEASE);
  }
}

void turnModuleBL(double angle){
  SetpointBL = angle;

  // float mappedAngleBL = map(analogRead(BLSensor), 0, 1023, 0, 360);
  InputBL = getAngleBL();

  errorBL = InputBL - SetpointBL;

  if (errorBL >= 180) {
    SetpointBL += 360;
  } else if (errorBL <= -180) {
    SetpointBL -= 360;
  }

  BLPID.Compute();

  // For debugging
  // Serial.print(SetpointBL);
  // Serial.print("  ");
  // Serial.print(InputBL);
  // Serial.print("  ");
  // Serial.print(OutputBL);

  if (OutputBL > 0){
    BLMotor->run(FORWARD);
    BLMotor->setSpeed(abs(OutputBL));
  }else if (OutputBL < 0){
    BLMotor->run(BACKWARD);
    BLMotor->setSpeed(abs(OutputBL));
  }else{
    BLMotor->setSpeed(0);
    BLMotor->run(RELEASE);
  }
}

float getAngleFR() {
  // float mappedAngle = analogRead(FRSensor);
  float mappedAngleFR = map(analogRead(FRSensor), 40, 1000, 0, 360);
  mappedAngleFR *= -1;

  mappedAngleFR -= Constants::FRangleOffset;

  if (mappedAngleFR < -180) {
    mappedAngleFR += 360;
  }
  return mappedAngleFR;
}

float getAngleFL() {
  //float mappedAngle = analogRead(FLSensor);
  float mappedAngleFL = map(analogRead(FLSensor), 40, 1000, 0, 360);
  mappedAngleFL *= -1;

  mappedAngleFL -= Constants::FLangleOffset;

  if (mappedAngleFL < -180) {
    mappedAngleFL += 360;
  }
  return mappedAngleFL;
}

float getAngleBR() {
  //float mappedAngle = analogRead(BRSensor);
  float mappedAngleBR = map(analogRead(BRSensor), 40, 1000, 0, 360);
  mappedAngleBR *= -1;

  mappedAngleBR -= Constants::BRangleOffset;

  if (mappedAngleBR < -180) {
    mappedAngleBR += 360;
  }
  return mappedAngleBR;
}

float getAngleBL() {
  float mappedAngleBL = map(analogRead(BLSensor), 40, 1000, 0, 360);
  mappedAngleBL *= -1;

  mappedAngleBL -= Constants::BLangleOffset;

  if (mappedAngleBL < -180) {
    mappedAngleBL += 360;
  }
  return mappedAngleBL;
}

void estopTurnMotors(){
  FRMotor->run(RELEASE);
  FLMotor->run(RELEASE);
  BRMotor->run(RELEASE);
  BLMotor->run(RELEASE);
}