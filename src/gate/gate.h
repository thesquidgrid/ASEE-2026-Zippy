#ifndef GATE_H   // "if this file hasn't been loaded yet..."
#define GATE_H   // "...mark it as loaded" (prevents loading twice)

#include <Arduino.h>
#include <Servo.h>

void gateInit();
void openGate();
void closeGate();


#endif