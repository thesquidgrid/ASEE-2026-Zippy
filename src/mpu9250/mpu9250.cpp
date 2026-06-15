#include "mpu9250.h"
#include <Arduino.h>
#include <Wire.h>

const uint8_t MPU_ADDR = 0x68;
float yawAngle = 0.0;      // degrees
float gyroBiasRawZ = 0.0;  // raw LSB units
unsigned long lastTime = 0;
float kp = 25.5;
float p_limit = 20;


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

void mpu_init() {
  // Wake MPU (PWR_MGMT_1 = 0)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission();

  delay(100);
  calibrateGyroZ();
  lastTime = micros();
}

float apply_limit(float limit, float value) {
    if (value > limit) {
        return limit;
    } else if (value < -limit) {
        return -limit;
    }
    return value;
}

float next_control_output(float setpoint) {
    float error = setpoint - updateYaw();

    float p_unbounded = error * kp;
    float p = apply_limit(p_limit, p_unbounded);
    
    return p;
}