#include "ColorSensor.h"

BallColor detectBallColor(uint16_t r, uint16_t g, uint16_t b, uint16_t c)
{ 
  //60k saturated light
  if (c == 0 || c > 60000) {
    return UNKNOWN;
  }

  float rn = (float)r / (float)c;
  float gn = (float)g / (float)c;
  float bn = (float)b / (float)c;

  // RED
  if (rn >= 0.55 && rn <= 0.75 &&
      gn >= 0.16 && gn <= 0.26 &&
      bn >= 0.13 && bn <= 0.22)
  {
    return RED;
  }

  // BLUE
  if (rn >= 0.18 && rn <= 0.31 &&
      gn >= 0.30 && gn <= 0.37 &&
      bn >= 0.37 && bn <= 0.46)
  {
    return BLUE;
  }

  // WHITE
  if (rn >= 0.29 && rn <= 0.36 &&
      gn >= 0.34 && gn <= 0.39 &&
      bn >= 0.27 && bn <= 0.33)
  {
    return WHITE;
  }

  return UNKNOWN;
}

const char* ballColorToString(BallColor color)
{
  switch (color) {
    case RED: return "RED";
    case WHITE: return "WHITE";
    case BLUE: return "BLUE";
    default: return "UNKNOWN";
  }
}