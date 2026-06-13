#ifndef INTAKE_H   // "if this file hasn't been loaded yet..."
#define INTAKE_H   // "...mark it as loaded" (prevents loading twice)

#include <Arduino.h>

// --- Pin definitions ---
const int input1 = 14;
const int input2 = 15;
const int enable = 36;

void intake_init();
void intake_on();
void intake_off();


#endif