#include "example.h"
#include <Arduino.h> 

//These are just were you put the functions for this library.
//It works the same as when functions are in the main program.

//example functions:
void setMotorSpeed(int left, int right) {
  analogWrite(2, left);
  analogWrite(3, right);
}

void stopMotors() {
  analogWrite(2, 0);
  analogWrite(3, 0);
}