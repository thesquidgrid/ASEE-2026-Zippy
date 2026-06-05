#include "src/DGMotor/DGMotor.h"  

#include "src/example/example.h"  

#include "HardwareSerial.h"

#include "Arduino.h"

#include <Wire.h>

#define RAD2DEG (180.0 / PI)

#include <Servo.h>

Servo ms1;
Servo ms2;
Servo ms3;


// z calib offset
float GzError = 0;
// yaw
float yaw = 0;
float turnYaw = 0;
float angularDisplacement_prev = 0;
int TpLeft = 300; // Target base power for left motor
int TpRight = 165; // Target base power for right motor

float Kp = 50;

// timing var for dt
unsigned long lastTime = 0;

DGMotor leftMotor(Serial6, 1);
DGMotor rightMotor(Serial7, 1);


int input1 = 14;
int input2 = 15;
int enable = 36;

int speedLeft = 0;
int speedRight = 0;


void setup() {
  //Wire
  Wire.begin();
  Wire.setClock(400000);

  //Serial
  Serial.begin(115200);
  while (!Serial) {}

  //---------Drive Motors---------//
  leftMotor.begin(115200);
  rightMotor.begin(115200);

  leftMotor.setVelocityMode();
  rightMotor.setVelocityMode();

  //---------Gyro---------//
  // if (!mpu.begin()) {
  //   //Serial.println("Failed to find MPU6050 chip");
  //   while (1) {
  //     delay(10);
  //   }
  // }
  // //Serial.println("MPU6050 Found!");

  // mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  // mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // // Serial.println("Stay Still");
  // delay(1000);

  // //grab 100 samples of Gz to determine the mean gyro offset
  // for (int i = 0; i < 100; i++) {
  //   mpu.getEvent( & a, & g, & temp);
  //   GzError += g.gyro.z;
  //   delay(10);
  // }
  // GzError /= 100.0;
  // //end

  // // Serial.println("Calibration Completed!");
  // lastTime = millis(); //start calculating time passed

  //---------Intake---------//
  pinMode(enable, OUTPUT);
  pinMode(input1, OUTPUT);
  pinMode(input2, OUTPUT);
  digitalWrite(enable, HIGH);
  digitalWrite(input2, LOW);
 

}

void loop() {

}
