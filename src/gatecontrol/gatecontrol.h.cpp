#include "intake.h"
#include <Arduino.h>

void intake_init(){
  pinMode(enable, OUTPUT);
  pinMode(input1, OUTPUT);
  pinMode(input2, OUTPUT);

  digitalWrite(enable, HIGH);
  digitalWrite(input1, LOW);
  digitalWrite(input2, LOW);
}

void intake_on(){
  digitalWrite(input1, HIGH);
}


void intake_off(){
  digitalWrite(input1, LOW);
}

