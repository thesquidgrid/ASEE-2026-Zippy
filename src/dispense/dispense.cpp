#include "dispense.h"
#include <Arduino.h>
#include "../drivebase/drivebase.h" 
#include "../colorSensor/colorSensor.h" 
#include "../gate/gate.h" 
#include "Adafruit_TCS34725.h"


BinPosition currentBin = AT_BLUE;

// ---------------- TUNING ----------------

const int BIN_MOVE_SPEED = 200; //60, 100
const int ONE_BIN_TIME_MS = 300; //1750, 900

// ---------------- DEBUG HELPERS ----------------

const char* binToString(BinPosition bin) {
  if (bin == AT_BLUE) return "BLUE";
  if (bin == AT_WHITE) return "WHITE";
  if (bin == AT_RED) return "RED";
  return "UNKNOWN_BIN";
}



// ---------------- BIN MOVEMENT ----------------

void blueToWhite() {
  Serial.println("Movement: BLUE to WHITE");
  driveForward(BIN_MOVE_SPEED);
  delay(ONE_BIN_TIME_MS);
  stopMotors();
  Serial.println("Arrived at WHITE.");
}

void blueToRed() {
  Serial.println("Movement: BLUE to WHITE");
  driveForward(BIN_MOVE_SPEED);
  delay(ONE_BIN_TIME_MS*2);
  stopMotors();
  Serial.println("Arrived at WHITE.");
}

void whiteToRed() {
  Serial.println("Movement: WHITE to RED");
  driveForward(BIN_MOVE_SPEED);
  delay(ONE_BIN_TIME_MS);
  stopMotors();
  Serial.println("Arrived at RED.");
}

void whiteToBlue() {
  Serial.println("Movement: WHITE to BLUE");
  driveBackward(BIN_MOVE_SPEED);
  delay(ONE_BIN_TIME_MS);
  stopMotors();
  Serial.println("Arrived at BLUE.");
}

void redToWhite() {
  Serial.println("Movement: RED to WHITE");
  driveBackward(BIN_MOVE_SPEED);
  delay(ONE_BIN_TIME_MS);
  stopMotors();
  Serial.println("Arrived at WHITE.");
}

void redToBlue() {
  Serial.println("Movement: RED to WHITE");
  driveBackward(BIN_MOVE_SPEED);
  delay(ONE_BIN_TIME_MS*2);
  stopMotors();
  Serial.println("Arrived at WHITE.");
}

void moveToBin(BinPosition targetBin) {

  if (currentBin == targetBin) {
    Serial.println("Already at target bin. No movement.");
    return;
  }

  if (currentBin == AT_BLUE && targetBin == AT_WHITE) {
    blueToWhite();
  }
  else if (currentBin == AT_BLUE && targetBin == AT_RED) {
    blueToRed();
  }
  else if (currentBin == AT_WHITE && targetBin == AT_BLUE) {
    whiteToBlue();
  }
  else if (currentBin == AT_WHITE && targetBin == AT_RED) {
    whiteToRed();
  }
  else if (currentBin == AT_RED && targetBin == AT_WHITE) {
    redToWhite();
  }
  else if (currentBin == AT_RED && targetBin == AT_BLUE) {
    redToBlue();
  }

  // software position update
  currentBin = targetBin;

  Serial.print("Updated current bin to: ");
  Serial.println(binToString(currentBin));
}

// ---------------- COLOR -> BIN ----------------

BinPosition colorToBin(BallColor color) {
  if (color == BLUE) return AT_BLUE;
  if (color == WHITE) return AT_WHITE;
  if (color == RED) return AT_RED;

  // if color is UNKNOWN, stay at current bin instead of moving
  return currentBin;
}

// ---------------- DISPENSE LOGIC ----------------

bool dispenseBall(int ballNumber) {

  BallColor firstRead = readBallColor();

  while(firstRead == UNKNOWN){
     firstRead = readBallColor();
     Serial.println("inloop");
     delay(300);
  }


  Serial.print("Reading Ball: "); Serial.println(ballColorToString(firstRead));
  BinPosition targetBin = colorToBin(firstRead);
  
  moveToBin(targetBin);
  gateRoutine();
  return true;
}

// ---------------- DISPENSE ALL ----------------

void dispenseAll() {
  int dispensedCount = 0;
  int attemptCount = 0;
  const int MAX_ATTEMPTS = 250; //25

  // 12 originally, 250 is max limit of reads so robot doesnt loop foreer
  while (dispensedCount < 12 && attemptCount < MAX_ATTEMPTS) {
    attemptCount++;


    bool actuallyDispensed = dispenseBall(dispensedCount + 1);

    // actually increment dispense count if full sequence succeeded.
    if (actuallyDispensed) {
      dispensedCount++;
    }

    delay(50);
  }

  stopMotors();
}

