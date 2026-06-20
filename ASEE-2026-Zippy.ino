#include "src/DGMotor/DGMotor.h"

#include "src/movement/movement.h"

#include "src/drivebase/drivebase.h"

#include "src/colorSensor/colorSensor.h"

#include "src/dispense/dispense.h"

#include "src/intake/intake.h"

#include "src/gate/gate.h"

#include "HardwareSerial.h"

#include "Arduino.h"

#include <Servo.h>

#include "Adafruit_TCS34725.h"

int led = 13;

void setup() {
  //------------Serial------------//
  Serial.begin(115200);

  //---------Drive Motors---------//
  initMotors(115200);

  //------------Intake------------//
  intake_init();
  
  //------------Gate-------------//
  gateInit();
  closeGate(); //close gate

  //------------LED--------------//
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);
  
  //----------Intake-------------//
  intake_on();

  //------------Gyro-------------//
  gyro_init();
  calibrateGyroZ();

  //---------Color Sensor--------//
  initColorSensor();
}


void loop() {

  int prev_setpoint = 180;
  int setpoint = 0;
  int period = 0;

  for (int i = 0; i < 6; i++) {
  
    int time = millis();
    intake_on();

    if((i % 2) == 0){ 
      period = 1800; //move forward for period
    } else{
      period = 1600; //move backward for period
    }

    if((i == 5) || (i == 0)){
      period = 2000; 
    } //traverse entire turf

    while ((millis() - time) < period) { 
      delay(10); //delay to prevent overwhelming the cpu
      steerForward(25.5, 20, 12.75, 10, setpoint, 200); // kp p_limit kd d_limit setpoint base_speed
    } //move forward

    //turn motors off to mitigate mechanical noise while turning
    stopMotors();
    delay(500);
    intake_off(); 

    if(i != 5) {
      while (abs(updateYaw() - prev_setpoint) > .5) { 
        delay(10);
        steerTurn(12.5, 100, 0, 0, prev_setpoint, 40); // kp p_limit kd d_limit setpoint base_speed
      } //turn around
    }

    prev_setpoint = setpoint; 
    setpoint = 180 - prev_setpoint;
  
  }

  park(); //get in corner

  dispenseAll(); 
  Serial.println("dispenseAll() fin");
  stopMotors();

  while(1);
}




