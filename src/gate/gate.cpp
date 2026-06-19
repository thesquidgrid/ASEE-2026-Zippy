#include "gate.h"
#include <Arduino.h>
#include <Servo.h>

// --- Definitions ---
Servo servo;
const int GATE_MOVE_TIME_MS = 600;
const int SERVO_OPEN_POS = 50;
const int SERVO_CLOSED_POS = 150;
const int SERVO_PIN = 12;

void gateInit(){
    servo.attach(SERVO_PIN);
}

void openGate(){
    servo.write(SERVO_OPEN_POS);
}

void closeGate(){
    servo.write(SERVO_CLOSED_POS); //180 originally
}

void gateRoutine(){
    openGate();
    delay(GATE_MOVE_TIME_MS);
    closeGate();
}

