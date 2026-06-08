#include "collisionSense.h"
#include <Arduino.h>

bool collision_detect() {
  if ((digitalRead(sw1) == LOW) || (digitalRead(sw2) == LOW)) {
    Serial.println("Hit!");
    delay(100); // debounce
    return true;
  }
  return false;
}

void collision_init() {
  pinMode(sw1, INPUT_PULLUP);
  pinMode(sw2, INPUT_PULLUP);
}
