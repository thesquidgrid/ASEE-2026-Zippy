#include "src/DGMotor/DGMotor.h"

#include "src/drivebase/drivebase.h"

#include "src/ColorSensor/ColorSensor.h"

#include "src/intake/intake.h"

#include "src/gate/gate.h"

#include "HardwareSerial.h"

#include "Arduino.h"

#include <Wire.h>

#include <Servo.h>

#include "Adafruit_TCS34725.h"

int led = 13;
const uint8_t MPU_ADDR = 0x68;

float yawAngle = 0.0;      // degrees
float gyroBiasRawZ = 0.0;  // raw LSB units
unsigned long lastTime = 0;

// Derivative state
static float prevMeasurement   = 0.0f;
static bool  hasPrevMeasurement = false;

enum BinPosition {
  AT_BLUE,
  AT_WHITE,
  AT_RED
};

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

  int prev_baseline = 180;
  int baseline = 0;
  int period = 0;

  for (int i = 0; i < 6; i++) {
  
    int time = millis();
    intake_on();

    if((i % 2) == 0){ 
      period = 1650; //move forward for period
    } else{
      period = 1600; //move backward for period
    }

    if((i == 5) || (i == 0)){
      period = 2000; 
    } //traverse entire turf

    while ((millis() - time) < period) { 
      delay(10); //delay to prevent overwhelming the cpu
      steerForward(baseline, 25.5, 20, 220); //setpoint kp adjustment_max base_speed
    } //move forward

      
    stopMotors();
    delay(500);
    intake_off(); 

    if(i != 5) {
      while (abs(updateYaw() - prev_baseline) > .5) { 
        delay(10);
        steerTurn(prev_baseline, 12.5, 100, 40); //setpoint kp adjustment_max base_speed
      } //turn around
    }

    prev_baseline = baseline; 
    baseline = 180 - prev_baseline;
  
  }

  park(); //get in corner

  dispenseAll(); 
  Serial.println("dispenseAll() fin");
  stopMotors();

  while(1);
}

//------------- Steering Commands -----------//

//speed bounds = speed +/- p_limit (caps how much the speed can change)
//speed: baseline speed the wheels will move at.
//setpoint: point at the gyro we want to steer to and at
void steerForward(int setpoint, float kp, float p_limit, int speed) {
  float p = next_control_output(setpoint, kp, kp, p_limit);

  int leftSpd = speed - p;
  int rightSpd = speed + p;

  rightWheels(rightSpd);
  leftWheels(leftSpd);
}

void steerTurn(int setpoint, float kp, float p_limit, int speed) {
  float p = next_control_output(setpoint, kp, 0, p_limit); //remove kd for steering

  int leftSpd = speed - p;
  int rightSpd = speed + p;

  rightWheels(rightSpd);
  leftWheels(leftSpd);
}


void steerBackward(int setpoint, float kp, float p_limit) {
  float p = next_control_output(setpoint, kp, kp, p_limit);

  int leftSpd = 60 - p;
  int rightSpd = 60 + p;

  rightWheels(-rightSpd);
  leftWheels(-leftSpd);
}

// ---------- Low-level: read raw gyro Z ----------
int16_t readGyroZRaw() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47);  // GYRO_ZOUT_H
  Wire.endTransmission(false);

  Wire.requestFrom((int)MPU_ADDR, 2);
  while (Wire.available() < 2) {
    // wait
  }

  int16_t gz = (Wire.read() << 8) | Wire.read();
  return gz;
}

// ---------- Calibrate gyro Z bias (raw units) ----------
void calibrateGyroZ() {
  long sum = 0;
  const int samples = 500;

  for (int i = 0; i < samples; i++) {
    sum += readGyroZRaw();
    delay(3);
  }

  gyroBiasRawZ = (float)sum / samples;  // store in raw units
}

// ---------- Update yaw angle (degrees) ----------
float updateYaw() {
  unsigned long now = micros();
  float dt = (now - lastTime) / 1e6;  // seconds
  lastTime = now;

  // raw → bias-corrected raw
  float gzRaw = (float)readGyroZRaw() - gyroBiasRawZ;

  // raw → deg/sec (±250 dps → 131 LSB/deg/s)
  float gzDegPerSec = gzRaw / 131.0;

  // integrate
  yawAngle += gzDegPerSec * dt;

  return yawAngle;
}

//-----------init gyro ---------------//
void gyro_init(){
  Wire.begin();
  Serial.begin(115200);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission();
}


// ---------- Clamp helper ---------- //
float apply_limit(float limit, float value) {
  if (value >  limit) return limit;
  else if (value < -limit) return -limit;
  return value;
}


// ---------- PD controller ---------- //
float next_control_output(float setpoint, float kp, float kd, float p_limit) {
  float measurement = updateYaw();   
  
  float error = setpoint - measurement;
  float p_unbounded = error * kp;
  float p = apply_limit(p_limit, p_unbounded);

  float d_unbounded = 0.0f;
  if (hasPrevMeasurement) {
    d_unbounded = -(measurement - prevMeasurement) * (kd / 2);
  }
  prevMeasurement = measurement;
  hasPrevMeasurement = true;
  float d = apply_limit(p_limit, d_unbounded);
  float output = d + p;
  return output;
}

//-----------park car in corner -----------//
void park(){

  int angle = 270;
  int leftSpd = -50;
  int rightSpd = 255;
  rightWheels(rightSpd);
  leftWheels(leftSpd);

  while (abs(updateYaw() - angle) > 1) { 
    //move until the angle is around 1 degree away from the goal angle
  } 

  driveBackward(-90); //really get in that corner
  delay(400);
  stopMotors(); 
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
    else {

    }

    delay(50);
  }


  stopMotors();
}
