#ifndef DISPENSE_H   // "if this file hasn't been loaded yet..."
#define DISPENSE_H   // "...mark it as loaded" (prevents loading twice)

#include <Arduino.h>
#include "../drivebase/drivebase.h" 
#include "../colorSensor/colorSensor.h"
#include "Adafruit_TCS34725.h"

enum BinPosition {
  AT_BLUE,
  AT_WHITE,
  AT_RED
};

//place functions from .cpp file here
const char* binToString(BinPosition bin);
void blueToWhite();
void blueToRed();
void whiteToRed();
void whiteToBlue();
void redToWhite();
void redToBlue();
void moveToBin(BinPosition targetBin);
BinPosition colorToBin(BallColor color);
bool dispenseBall(int ballNumber);
void dispenseAll();

#endif