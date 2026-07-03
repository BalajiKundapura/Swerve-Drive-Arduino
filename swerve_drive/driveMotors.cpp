#include <SimpleFOC.h>
#include "constants.h"
// BLDC motor & driver instance
// BLDCMotor motor = BLDCMotor(pole pair number);
BLDCDriver3PWM driverFR = BLDCDriver3PWM(PA11, PA9, PA10, PA12); //PA11, PA9, PA10, PA12
BLDCMotor motorFR = BLDCMotor(7);

BLDCDriver3PWM driverFL = BLDCDriver3PWM(PB6, PB8, PB9, PB5); //PB6, PB8, PB9, PB5
BLDCMotor motorFL = BLDCMotor(7);

BLDCDriver3PWM driverBR = BLDCDriver3PWM(PC6, PC7, PC8, PC5); 
BLDCMotor motorBR = BLDCMotor(7);

BLDCDriver3PWM driverBL = BLDCDriver3PWM(PA0, PA1, PA15, PC4); //PA0, PA3, PA1, PC4
BLDCMotor motorBL = BLDCMotor(7);

int maxSpeed = Constants::maxSpeed / Constants::wheelRadius;
int minSpeed = -Constants::maxSpeed / Constants::wheelRadius;



void setupDriveMotors() {

  // driver config
  // power supply voltage [V]
  driverFR.voltage_power_supply = 12;
  driverFL.voltage_power_supply = 12;
  driverBR.voltage_power_supply = 12;
  driverBL.voltage_power_supply = 12;
  // limit the maximal dc voltage the driver can set
  // as a protection measure for the low-resistance motors
  // this value is fixed on startup
  
  driverFR.voltage_limit = Constants::maxDriverVoltage;
  driverFL.voltage_limit = Constants::maxDriverVoltage;
  driverBR.voltage_limit = Constants::maxDriverVoltage;
  driverBL.voltage_limit = Constants::maxDriverVoltage;

  if(!driverFR.init()){
    Serial.println("Driver init FR failed!");
    return;
  }
  if(!driverFL.init()){
    Serial.println("Driver init FL failed!");
    //return;
  }
  if(!driverBR.init()){
    Serial.println("Driver init BR failed!");
    //return;
  }
  if(!driverBL.init()){
    Serial.println("Driver init BL failed!");
    return;
  }
  // link the motor and the driver
  motorFR.linkDriver(&driverFR);
  motorFL.linkDriver(&driverFL);
  motorBR.linkDriver(&driverBR);
  motorBL.linkDriver(&driverBL);

  // limiting motor movements
  // limit the voltage to be set to the motor
  // start very low for high resistance motors
  // current = voltage / resistance, so try to be well under 1Amp
  motorFR.voltage_limit = Constants::maxMotorVoltage;   // [V]
  motorFL.voltage_limit = Constants::maxMotorVoltage;   // [V]
  motorBR.voltage_limit = Constants::maxMotorVoltage;   // [V]
  motorBL.voltage_limit = Constants::maxMotorVoltage;   // [V]
 
  // open loop control config
  motorFR.controller = MotionControlType::velocity_openloop;
  motorFL.controller = MotionControlType::velocity_openloop;
  motorBR.controller = MotionControlType::velocity_openloop;
  motorBL.controller = MotionControlType::velocity_openloop;

  

  //init motor hardware
  if(!motorFR.init()){
    Serial.println("Motor FR init failed!");
    return;
  }
  if(!motorFL.init()){
    Serial.println("Motor FL init failed!");
    //return;
  }
  if(!motorBR.init()){
    Serial.println("Motor BR init failed!");
    //return;
  }
  if(!motorBL.init()){
    Serial.println("Motor BL init failed!");
    return;
  }

  motorFR.disable();
  motorFL.disable();
  motorBR.disable();
  motorBL.disable();

  Serial.println("Motors ready!");
  _delay(1000);

}

void driveFR(float speed) {
  if (speed == 0){
    motorFR.disable();
  }  else {
    motorFR.enable();
    motorFR.move(speed);
  }
}

void driveFL(float speed) {
  if (speed == 0){
    motorFL.disable();
  }  else {
    motorFL.enable();
    motorFL.move(speed);
  }
}

void driveBR(float speed) {
  if (speed == 0){
    motorBR.disable();
  }  else {
    motorBR.enable();
    motorBR.move(speed);
  }
}

void driveBL(float speed) {
  if (speed == 0){
    motorBL.disable();
  }  else {
    motorBL.enable();
    motorBL.move(speed);
  }
}

void estopDriveMotors(){
  motorFR.disable();
  motorFL.disable();
  motorBR.disable();
  motorBL.disable();
}
