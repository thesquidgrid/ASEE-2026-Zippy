#include "intake.h"
#include <Arduino.h>

// --- Pin definitions ---
const int enable = 36;


void intake_init(){
  pinMode(enable, OUTPUT);
  digitalWrite(enable, LOW);
}

void intake_on(){
  digitalWrite(enable, HIGH);
}

void intake_off(){
  digitalWrite(enable, LOW);
}

