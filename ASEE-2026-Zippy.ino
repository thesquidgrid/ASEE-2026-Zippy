
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
  delay(10);
  turn(180);
  
}

void steer(int setpoint){
  float p = next_control_output(setpoint);
  Serial.println(p);

  int leftSpd = 60 - p;
  int rightSpd = 60 + p;

  Serial.print("YAW: ");
  Serial.println(updateYaw());
  
  if(leftSpd < 0){
    leftSpd = 0;
  }
  if(rightSpd < 0){
    rightSpd = 0;
  }

  rightWheels(rightSpd);
  leftWheels(leftSpd);

  Serial.print("R: "); Serial.println(rightSpd);
  Serial.print("L: "); Serial.println(leftSpd);
  Serial.println("-------------");
}

void turn(int angle){
  float baseline = updateYaw();
  if(angle < baseline){
    leftWheels(60);  
    while(updateYaw() > angle){  
    }
  } else{
    rightWheels(60);
    while(updateYaw() < angle){
    }
  }
  stopMotors();
}


