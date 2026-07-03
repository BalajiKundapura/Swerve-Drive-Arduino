#ifndef DRIVEMOTORS_H
#define DRIVEMOTORS_H

// Declare your functions here
void setupDriveMotors();

void driveFR(float speed);
void driveFL(float speed);
void driveBR(float speed);
void driveBL(float speed);

void estopDriveMotors();
#endif