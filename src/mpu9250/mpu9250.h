#ifndef MPU9250_H   // "if this file hasn't been loaded yet..."
#define MPU9250_H   // "...mark it as loaded" (prevents loading twice)

#include <Arduino.h>

int16_t readGyroZRaw();
void calibrateGyroZ();
float updateYaw();
void mpu_init();
float apply_limit(float limit, float value);
float next_control_output(float setpoint);
#endif