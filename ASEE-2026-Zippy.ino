#include "src/DGMotor/DGMotor.h"

#include "src/collisionSense/collisionSense.h"

#include "src/drivebase/drivebase.h"

#include "src/ColorSensor/ColorSensor.h"

#include "src/intake/intake.h"

#include "src/gate/gate.h"

#include "HardwareSerial.h"

#include "Arduino.h"

#include <Wire.h>

#include <Servo.h>

#include "Adafruit_TCS34725.h"

// straight forward is 25.5 kp and p_limit = 20

int led = 13;
const uint8_t MPU_ADDR = 0x68;
float yawAngle = 0.0;      // degrees
float gyroBiasRawZ = 0.0;  // raw LSB units
unsigned long lastTime = 0;
// Derivative state
static float prevMeasurement   = 0.0f;
static bool  hasPrevMeasurement = false;


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
  


  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);
  
  Wire.begin();
  Serial.begin(115200);

  // Wake MPU (PWR_MGMT_1 = 0)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission();

  delay(100);
  intake_on();
  calibrateGyroZ();
  lastTime = micros();

  
}

void flushSerial() {
  delay(10);
  while (Serial.available()) Serial.read();
}

void loop() {

  int prev_baseline = 180;
  int baseline = 0;
  int period = 2400;

  for (int i = 0; i < 6; i++) {

    int time = millis();
    intake_on();

    while ((millis() - time) < period) {
      delay(10);
      steerForward(baseline,20, 22.5, 150); //25.5 20 60
    }
    intake_off();
    period = 2700;
    stopMotors();
    delay(500);
    
    
    while (abs(updateYaw() - prev_baseline) > 1) {
      delay(10);
      Serial.println(updateYaw());
      steerForward(prev_baseline, 12.75, 100, 60);
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
  float p = next_control_output(setpoint, kp, kp, p_limit);

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

// ---------- Clamp helper ----------
float apply_limit(float limit, float value) {
  if (value >  limit) return limit;
  else if (value < -limit) return -limit;
  return value;
}


// ---------- PD controller ----------
float next_control_output(float setpoint, float kp, float kd, float p_limit) {
  float measurement = updateYaw();   
  Serial.println(measurement);
  float error = setpoint - measurement;
  float p_unbounded = error * kp;
  float p = apply_limit(p_limit, p_unbounded);

  float d_unbounded = 0.0f;
  // if (hasPrevMeasurement) {
  //   d_unbounded = -(measurement - prevMeasurement) * kd;
  // }
  // prevMeasurement    = measurement;
  // hasPrevMeasurement = true;
  // float d = apply_limit(p_limit, d_unbounded);

  return p;
}


