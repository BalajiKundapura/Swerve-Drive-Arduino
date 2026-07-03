#ifndef TURNMOTORS_H
#define TURNMOTORS_H

// Declare your functions here
void setupTurnMotors();

void turnModuleFR(double angle);
void turnModuleFL(double angle);
void turnModuleBR(double angle);
void turnModuleBL(double angle);

float getAngleFR();
float getAngleFL();
float getAngleBR();
float getAngleBL();

void estopTurnMotors();

extern float errorFL;
extern float errorFR;
extern float errorBL;
extern float errorBR;

#endif