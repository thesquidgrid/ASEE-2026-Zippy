#ifndef GATE_H   // "if this file hasn't been loaded yet..."
#define GATE_H   // "...mark it as loaded" (prevents loading twice)

#include <Arduino.h>
#include <Servo.h>

// --- Pin definitions ---
Servo servo;
servo.attach(12);

void openGate();
void closeGate();
void intake_off();


#endif