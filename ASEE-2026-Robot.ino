// note - want to integrate threading to see if that helps with teensy control of wheels


#include "DGMotor.h"

#include "HardwareSerial.h"

#include "Arduino.h"

#include <Adafruit_MPU6050.h>

#include <Adafruit_Sensor.h>

#include <Wire.h>

#define RAD2DEG (180.0 / PI)

#include <Servo.h>

Servo ms1;
Servo ms2;
Servo ms3;

sensors_event_t a, g, temp;

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
Adafruit_MPU6050 mpu;

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

  //Drive motors
  leftMotor.begin(115200);
  rightMotor.begin(115200);

  leftMotor.setVelocityMode();
  rightMotor.setVelocityMode();

  //Gyro
  if (!mpu.begin()) {
    //Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  //Serial.println("MPU6050 Found!");

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Serial.println("Stay Still");
  delay(1000);

  //grab 100 samples of Gz to determine the mean gyro offset
  for (int i = 0; i < 100; i++) {
    mpu.getEvent( & a, & g, & temp);
    GzError += g.gyro.z;
    delay(10);
  }
  GzError /= 100.0;
  //end

  // Serial.println("Calibration Completed!");
  lastTime = millis(); //start calculating time passed

  //Intake
  pinMode(enable, OUTPUT);
  pinMode(input1, OUTPUT);
  pinMode(input2, OUTPUT);

  digitalWrite(enable, HIGH);
  digitalWrite(input2, LOW);

  menu();
 

}

void backup() {
  leftMotor.setMotorSpeed(100);
  rightMotor.setMotorSpeed(-100);
}

void forward() {
  leftMotor.setMotorSpeed(-100);
  rightMotor.setMotorSpeed(100);
}



void loop() {

  if (Serial.available() > 1) {
    int choice = Serial.parseInt();
    switch (choice) {
    case 0:
      analogWrite(input1, 0);
      driveBreak();
      delay(100);
      driveBreak();
      break;
    case 1: {
      Serial.println("Left: ");
      Serial.read(); //clear buffer
      while (Serial.available() < 1); //wait until input
      speedLeft = Serial.parseInt();
      Serial.read(); //clear buffer
      delay(100);
      Serial.println("Right: ");
      while (Serial.available() < 1); //wait until input
      int speedRight = Serial.parseInt();
      leftMotor.setMotorSpeed(-1 * speedRight);
      rightMotor.setMotorSpeed(speedLeft);
    }
    break;
    case 2: {
      unsigned long startDriveTime = millis();
      lastTime = millis(); // update lastTime to reflect new start time of function

      while (millis() - startDriveTime <= 2800) {
        PI_drive(TpLeft, TpRight);
      }
      driveBreak(); // stop moving
    }

    break;
    case 3:

    {
      Serial.read(); //clear buffer
      while (Serial.available() < 1); //wait until input
      Kp = Serial.parseInt();
      Serial.print("New Kp is: ");
      Serial.println(Kp);
    }

    break;
    case 4: {
      Serial.read(); //clear buffer
      while (Serial.available() < 1); //wait until input
      TpLeft = Serial.parseInt();
    }
    break;
    case 5: {
      Serial.read(); //clear buffer
      while (Serial.available() < 1); //wait until input
      TpRight = Serial.parseInt();
    }
    break;
    case 6: {
      String buffer = "Kp: " + (String) Kp + " TpLeft: " + (String) TpLeft + " TpRight: " + (String) TpRight;
      Serial.println(buffer);
    }
    break;
    case 7:
      analogWrite(input1, 220);
      break;
    case 8:
       analogWrite(input1, 0);
      break;
    case 9:
      digitalWrite(input1, HIGH);
      break;
    case 10:
      digitalWrite(input1, LOW);
      break;
    case 11:
      leftMotor.setMotorSpeed(100);
      leftMotor.setMotorSpeed(-100);
      break;
    }
    menu();
  }
}

void menu() {
  Serial.println("============================");
  Serial.println("0.)                    Break");
  Serial.println("1.)             Stupid Drive");
  Serial.println("2.)              Smart Drive");
  Serial.println("3.)                Change Kp");
  Serial.println("4.)                    Drive");
  Serial.println("5.)            Change TpLeft");
  Serial.println("6.)           Change TpRight");
  Serial.println("7.)           Turn on Intake");
  Serial.println("8.)          Turn off Intake");
  Serial.println("============================");
}


float Ki = 0;
float Kp = 0;
float Kd = 0;
float integral_term = 0;

void Steer(int left, int right) {
  // calculate dt
  long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0; //ms to s
  lastTime = currentTime; //update

  mpu.getEvent( & a, & g, & temp);

  float Gz_deg = (g.gyro.z - GzError) * RAD2DEG; //calculate Gz by accounting for offset and converting to deg/s
  yaw += Gz_deg * dt;

  float error = 0 - yaw; 

  float proportion_unbounded = error*Kp;
  //float proportion = constrain(proportion_unbounded, 0, proportion_unbounded);
  
  integral_term = integral_term + (error*Ki);
  integral_term = constrain(integral_term, 0, integral_term);
  
  // neg yaw: ccw
  // pos yaw: cw
  double correction = Kp * yaw;

  double powerLeft = left - correction;
  double powerRight = right + correction;

  powerLeft = constrain(powerLeft, 0, 300);
  powerRight = constrain(powerRight, 0, 300);

  leftMotor.setMotorSpeed(-1 * powerLeft);
  rightMotor.setMotorSpeed(powerRight);
}

void turn(int deg) {
  {

    int displacement_goal = deg + abs(angularDisplacement_prev);

    unsigned long startDriveTime = millis();

    double angularDisplacement = 0;

    leftMotor.setMotorSpeed(-70);
    rightMotor.setMotorSpeed(-100);

    while (abs(angularDisplacement) < displacement_goal) {
      angularDisplacement = driveTurn();
    }

    if (abs(angularDisplacement) > displacement_goal) {
      leftMotor.setMotorSpeed(5);
      rightMotor.setMotorSpeed(10);
      while (abs(angularDisplacement) > displacement_goal) {
        angularDisplacement = driveTurn();
      }
    }

    angularDisplacement_prev = angularDisplacement;

    leftMotor.setMotorSpeed(0);
    rightMotor.setMotorSpeed(0);
    Serial.println("done!");
  }
}

double driveTurn() {
  long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0; //ms to s
  lastTime = currentTime; //update

  mpu.getEvent( & a, & g, & temp);

  float Gz_deg = (g.gyro.z - GzError) * RAD2DEG; //calculate Gz by accounting for offset and converting to deg/s

  return turnYaw += Gz_deg * dt; //integration to get position
}

void driveRoutine(int delay, int left, int right) {
  {
    unsigned long startDriveTime = millis();
    lastTime = millis(); // update lastTime to reflect new start time of function

    while (millis() - startDriveTime <= delay) {
      PI_drive(left, right);
    }
    driveBreak(); // stop moving
  }
}



void driveBreak() {
  leftMotor.setMotorSpeed(0);
  rightMotor.setMotorSpeed(0);
}