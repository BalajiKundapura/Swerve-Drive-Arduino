#include "swerveKinematics.h"
#include <Arduino.h>
#include "constants.h"
#include "turnMotors.h"
#include "gyro.h"
#include "fsRCcontrol.h"
// module naming convention goes like a coordinate plane:
// module1 = FR, module2 = FL, module3 = BL, module4 = BR

// kinematics derived from: https://www.chiefdelphi.com/t/paper-4-wheel-independent-drive-independent-steering-swerve/107383
//                          https://www.chiefdelphi.com/uploads/default/original/3X/8/c/8c0451987d09519712780ce18ce6755c21a0acc0.pdf

float VX = 0;
float VY = 0;

swerveState calculateSwerve(float inputVX, float inputVY, float inputRotation, bool fieldOriented){
  swerveState state;
  Serial.print(VX);
  Serial.print(" ");
  Serial.print(VY);
  Serial.print(" | ");

  if (fieldOriented == true) {
    VX =  inputVX * cos(-getYaw() * (M_PI/180)) + inputVY * sin(-getYaw() * (M_PI/180));
    VY =  -inputVX * sin(-getYaw() * (M_PI/180)) + inputVY * cos(-getYaw() * (M_PI/180));
  } else {
    VX = inputVX;
    VY = inputVY;
  }
  float v1x = VX + inputRotation*(Constants::length/2);
  float v1y = VY - inputRotation*(Constants::width/2);
  float speed1 = sqrt(v1x * v1x + v1y * v1y);
  float angle1 = atan2(v1x, v1y)*(180/M_PI);

  

  float v2x = VX + inputRotation*(Constants::length/2);
  float v2y = VY + inputRotation*(Constants::width/2);
  float speed2 = sqrt(v2x * v2x + v2y * v2y);
  float angle2 = atan2(v2x, v2y)*(180/M_PI);

  
  float v3x = VX - inputRotation*(Constants::length/2);
  float v3y = VY + inputRotation*(Constants::width/2);
  float speed3 = sqrt(v3x * v3x + v3y * v3y);
  float angle3 = atan2(v3x, v3y)*(180/M_PI);

  

  float v4x = VX - inputRotation*(Constants::length/2);
  float v4y = VY - inputRotation*(Constants::width/2);
  float speed4 = sqrt(v4x * v4x + v4y * v4y);
  float angle4 = atan2(v4x, v4y)*(180/M_PI);

  

  float speeds[4] = {speed1, speed2, speed3, speed4};
  size_t size = sizeof(speeds) / sizeof(speeds[0]); 

  // correct kinematics for speeds greater than the max of the motor
  if (speed1 > Constants::maxSpeed || speed2 > Constants::maxSpeed || speed3 > Constants::maxSpeed || speed4 > Constants::maxSpeed) {
      float max = *std::max_element(speeds, speeds + size);
      float fixerProportion = max/Constants::maxSpeed;

      speeds[0] = speed1 / fixerProportion;
      speeds[1] = speed2 / fixerProportion;
      speeds[2] = speed3 / fixerProportion;
      speeds[3] = speed4 / fixerProportion;
      
  }

  state.FR.speed = speeds[0] / Constants::wheelRadius;
  state.FR.angle = angle1;
  
  state.FL.speed = speeds[1] / Constants::wheelRadius;
  state.FL.angle = angle2;

  state.BL.speed = speeds[2] / Constants::wheelRadius;
  state.BL.angle = angle3;

  state.BR.speed = speeds[3] / Constants::wheelRadius;
  state.BR.angle = angle4;


  // optimizing angle math so that the swerve never has to rotate more than 90 degrees at once
  if ((getAngleFR() - angle1) > 90 || (getAngleFR() - angle1) < -90) {
    state.FR.speed *= -1;
    state.FR.angle -= 180;
  }

  if ((getAngleFL() - angle2) > 90 || (getAngleFL() - angle2) < -90) {
    state.FL.speed *= -1;
    state.FL.angle -= 180;
  }
  
  if ((getAngleBR() - angle4) > 90 || (getAngleBR() - angle4) < -90) {
    
    
    state.BR.speed *= -1;
    state.BR.angle -= 180;
  }

  if ((getAngleBL() - angle3) > 90 || (getAngleBL() - angle3) < -90) { 
  
    state.BL.speed *= -1;
    state.BL.angle -= 180;
  }

  return state;
}

