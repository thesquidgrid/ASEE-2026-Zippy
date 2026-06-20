#ifndef MOVEMENT_H   // "if this file hasn't been loaded yet..."
#define MOVEMENT_H   // "...mark it as loaded" (prevents loading twice)

#include <Arduino.h>
#include "../drivebase/drivebase.h" 
#include <Wire.h>

//place functions from .cpp file here
void steerForward(float kp, float p_limit, float kd, float d_limit, int setpoint, int speed);
void steerTurn(float kp, float p_limit, float kd, float d_limit, int setpoint, int speed);
//void steerBackward(int setpoint, float kp, float p_limit);
int16_t readGyroZRaw();
void calibrateGyroZ();
float updateYaw();
void gyro_init();
float apply_limit(float limit, float value);
float next_control_output(float setpoint, float kp, float p_limit, float kd, float d_limit);
void park();
#endif