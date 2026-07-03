// Programmed by Joshua Rambiya

// Swerve Kinematics derived from: https://www.chiefdelphi.com/t/paper-4-wheel-independent-drive-independent-steering-swerve/107383
//                                 https://www.chiefdelphi.com/uploads/default/original/3X/8/c/8c0451987d09519712780ce18ce6755c21a0acc0.pdf
// The best swerve drives are built by FIRST Robotics Competition students, no other swerve compares ^^^^^^^^

// Credit to DroneBot Workshop for flysky reciever code: https://dronebotworkshop.com/radio-control-arduino-car/

// Controls: 
// SWD (Channel 6) -> Up - enabled, down - disabled
// SWA (Channel 5) -> Down - Field relative swerve, Up - robot relative swerve
// Left Stick X axis (Channel 1) -> Rotate
// Right Stick X axis (Channel 4) -> Translate left/right
// Right Stick Y axis (Channel 2) -> Translate up/down

// Type "E1" into serial monitor for E-stop



unsigned long lastPIDTime = 0;
const unsigned long PIDSampleTime = 100;

struct moduleState {
  float speed;
  float angle;
};
  
struct swerveState{
  moduleState FR;
  moduleState FL;
  moduleState BR;
  moduleState BL;
};

swerveState calculateSwerve(float inputVX, float inputVY, float inputRotation, bool fieldOriented);
#include "driveMotors.h"
#include "turnMotors.h"
#include "fsRCcontrol.h"
#include "gyro.h"
#include "swerveKinematics.h"
#include "constants.h"
#include <SimpleFOC.h>

float estop = 0;

bool fieldOriented = false;

Commander command = Commander(Serial);
void activateEstop(char* cmd) { command.scalar(&estop, cmd); }

void setup() {
  Serial.begin(115200);

  setupFSRCcontrol();
  setupGyro(); //init gyro first to enable wire2 since adafruit motor shield uses the same i2c bus
  setupDriveMotors(); 
  setupTurnMotors(); 
  

  // E-stop through serial
  command.add('E', activateEstop, "estop");

  

}

void loop() {
  while (estop == 0 && getFSRCData(6) < 0 ){     // only run main loop when enabled
    
    if (getFSRCData(5) <= 0) {
      fieldOriented = false;
    } else {
      fieldOriented = true;
    }

    swerveState states = calculateSwerve(getFSRCData(4), getFSRCData(2), getFSRCData(1)/Constants::wheelRadius, fieldOriented);

    driveFL(states.FL.speed);
    driveFR(states.FR.speed);
    driveBR(states.BR.speed);
    driveBL(states.BL.speed);
    
    // turn turn motors only when needed, otherwise wheels would default back to 0 heading with no input
    unsigned long currentTime = millis();
    if (getFSRCData(4) == 0 && getFSRCData(2) == 0 && getFSRCData(1) < 10 && getFSRCData(1) > -10){
        estopTurnMotors();
    } else {
      if (currentTime - lastPIDTime >= PIDSampleTime) {
      
        turnModuleFL(states.FL.angle);
        turnModuleFR(states.FR.angle);
        turnModuleBR(states.BR.angle);
        turnModuleBL(states.BL.angle);
      }
    }

    // // stuff for debugging
    // Serial.print(getAngleFR());
    // Serial.print(" | ");
    // Serial.print(fieldOriented);
    // Serial.print(" ");
    // Serial.print(getYaw());
    // Serial.print(" | ");
    // Serial.print(getFSRCData(4));
    // Serial.print(" ");
    // Serial.print(getFSRCData(2));
    // Serial.print(" ");
    // Serial.print(getFSRCData(1));
    // Serial.print(" | ");
    // Serial.print(states.FL.angle);
    // Serial.print(" ");
    // Serial.print(states.FR.angle);
    // Serial.print(" ");
    // Serial.print(states.BL.angle);
    // Serial.print(" ");
    // Serial.print(states.BR.angle);
    // Serial.print(" ");
    // Serial.print(" | ");
    // Serial.print(states.FL.speed);
    // Serial.print(" ");
    // Serial.print(states.FR.speed);
    // Serial.print(" ");
    // Serial.print(states.BL.speed);
    // Serial.print(" ");
    // Serial.print(states.BR.speed);
    // Serial.print(" ");


    Serial.println("");
    command.run();
  }
  Serial.println("E-stop is active. Reset to continue");
  estopTurnMotors();
  estopDriveMotors();
  
  
}


