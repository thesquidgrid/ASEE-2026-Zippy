#ifndef COLOR_SENSOR_H
#define COLOR_SENSOR_H

#include <Arduino.h>

enum BallColor {
  RED,
  WHITE,
  BLUE,
  UNKNOWN
};

BallColor detectBallColor(uint16_t r, uint16_t g, uint16_t b, uint16_t c);

const char* ballColorToString(BallColor color);

#endif