#include "ColorSensor.h"
#include <Wire.h>
#include "Adafruit_TCS34725.h"

static Adafruit_TCS34725 tcs = Adafruit_TCS34725(
  TCS34725_INTEGRATIONTIME_600MS,
  TCS34725_GAIN_1X
);

bool initColorSensor()
{
  //sda pin 18
  //scl pin 19
  Wire1.begin();
  Wire1.setClock(400000);

  if (!tcs.begin(TCS34725_ADDRESS, &Wire1)) {
    return false;
  }

  return true;
}

BallColor readBallColor()
{
  uint16_t r, g, b, c;

  tcs.getRawData(&r, &g, &b, &c);

  Serial.print(r);
  Serial.print(",");
  Serial.print(g);
  Serial.print(",");
  Serial.print(b);
  Serial.print(",");
  Serial.println(c);

  return detectBallColor(r, g, b, c);
}

BallColor detectBallColor(uint16_t r, uint16_t g, uint16_t b, uint16_t c)
{

if (c > 750)
{
    return WHITE;
}

if ((b < r) && (r > 200))
{
    return RED;
}


// Blue top or blue bottom
if ((b > r) && (c < 500) && (c > 100))
{
    return BLUE;
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