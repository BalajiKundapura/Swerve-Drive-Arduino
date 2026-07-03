#ifndef FSRCCONTROL_H
#define FSRCCONTROL_H

// Declare your functions here
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

#endif