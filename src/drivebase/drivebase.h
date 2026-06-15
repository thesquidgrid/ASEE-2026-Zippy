#ifndef DRIVEBASE_H   // "if this file hasn't been loaded yet..."
#define DRIVEBASE_H   // "...mark it as loaded" (prevents loading twice)

#include <Arduino.h>
#include "../DGMotor/DGMotor.h" 

//EXAMPLE_H can be anyname

//place functions from .cpp file here

void driveForward(int speed);
void driveBackward(int speed);
void stopMotors();
void initMotors(unsigned long baud);
void rightWheels(int speed);
void leftWheels(int speed);

#endif