#include "gate.h"
#include <Arduino.h>
#include <Servo.h>

// --- Definitions ---
Servo servo;

void gateInit(){
    servo.attach(12);
}

void openGate(){
    servo.write(100);
}

void closeGate(){
    servo.write(180);
}

// int senseColor(){
//     return 1;
// }

// void move_to_blue(){

// }

// void move_to_red(){

// }

// void move_to_white(){

// }

