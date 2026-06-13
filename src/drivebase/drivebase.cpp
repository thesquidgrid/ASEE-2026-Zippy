#include "drivebase.h"
#include "../DGMotor/DGMotor.h" 
#include <Arduino.h>

// --- Motor definitions ---
DGMotor leftRear(Serial6, 1);  // back-left  wheel
DGMotor rightRear(Serial7, 1);  // back-right wheel
DGMotor leftFront(Serial1, 1);  // front-left  wheel
DGMotor rightFront(Serial2, 1);  // front-right wheel

void initMotors(unsigned long baud) {
  leftRear.begin(baud);
  rightRear.begin(baud);
  leftFront.begin(baud);
  rightFront.begin(baud);
}

void driveForward(int speed) {
  speed = abs(speed);

  leftRear.setMotorSpeed(speed);
  leftFront.setMotorSpeed(speed);

  rightRear.setMotorSpeed(-speed);
  rightFront.setMotorSpeed(-speed);
}

void driveBackward(int speed) {
  speed = abs(speed);

  leftRear.setMotorSpeed(-speed);
  leftFront.setMotorSpeed(-speed);

  rightRear.setMotorSpeed(speed);
  rightFront.setMotorSpeed(speed);
}

void stopMotors() {
  leftRear.setMotorSpeed(0);
  rightRear.setMotorSpeed(0);
  leftFront.setMotorSpeed(0);
  rightFront.setMotorSpeed(0);
}