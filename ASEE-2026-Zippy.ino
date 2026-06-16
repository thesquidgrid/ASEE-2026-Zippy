#include "src/DGMotor/DGMotor.h"

#include "src/collisionSense/collisionSense.h"

#include "src/drivebase/drivebase.h"

#include "src/ColorSensor/ColorSensor.h"

#include "src/mpu9250/mpu9250.h"

#include "src/intake/intake.h"

#include "src/gate/gate.h"

#include "HardwareSerial.h"

#include "Arduino.h"

#include <Wire.h>

#include <Servo.h>

#include "Adafruit_TCS34725.h"

// straight forward is 25.5 kp and p_limit = 20

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

  mpu_init();

  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);

}

void loop() {

  int prev_baseline = 180;
  int baseline = 0;
  int period = 2000;

  for (int i = 0; i < 6; i++) {
    int time = millis();

    while ((millis() - time) < period) {
      delay(10);
      steerForward(baseline, 20, 25.5, 255); //25.5 20 60
    }

    period = 1850;

    while (abs(updateYaw() - prev_baseline) > .05) {
      Serial.println(updateYaw());
      steerForward(prev_baseline, 22.5, 100, 60);
    }

    prev_baseline = baseline;
    baseline = updateYaw();

  }

  //back up

  int time = millis();

  while ((millis() - time) < 1500) {
    delay(10);
    steerBackward(0, 2.25, 10);
  }

  //ram into the corner 
  int angle = -90;
  int leftSpd = 50;
  int rightSpd = -255;
  rightWheels(rightSpd);
  leftWheels(leftSpd);
  while (abs(updateYaw() - angle) > 2) {
    Serial.println(updateYaw());

  }

  stopMotors();
  while (1);
}

//speed bounds = speed +/- p_limit (caps how much the speed can change)
//speed: baseline speed the wheels will move at.
void steerForward(int setpoint, float kp, float p_limit, int speed) {
  float p = next_control_output(setpoint, kp, p_limit);

  int leftSpd = speed - p;
  int rightSpd = speed + p;

  rightWheels(rightSpd);
  leftWheels(leftSpd);
}

void steerBackward(int setpoint, float kp, float p_limit) {
  float p = next_control_output(setpoint, kp, p_limit);

  int leftSpd = 60 - p;
  int rightSpd = 60 + p;

  rightWheels(-rightSpd);
  leftWheels(-leftSpd);
}