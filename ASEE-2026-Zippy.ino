#include "src/DGMotor/DGMotor.h"  

#include "src/collisionSense/collisionSense.h"  

#include "src/drivebase/drivebase.h"  

#include "src/intake/intake.h"  

#include "HardwareSerial.h"

#include "Arduino.h"

#include <Wire.h>

#define RAD2DEG (180.0 / PI)

#include <Servo.h>


int led = 13;

void setup() {
  //-------------Wire-------------//
  Wire.begin();
  Wire.setClock(400000);

  //------------Serial------------//
  Serial.begin(115200);

  //---------Drive Motors---------//
  initMotors(115200);
  
  //------------Intake------------//
  intake_init();


}

void loop() {
  // drive for one second, then stop permanently
  driveForward(1000);
  delay(1000);
  stopMotors();
}
