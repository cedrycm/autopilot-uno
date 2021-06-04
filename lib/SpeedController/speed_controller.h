

#ifndef speed_controller_h
#define speed_controller_h

#define PI 3.14159265

#define degreesToRadians(angleDegrees) (angleDegrees * PI / 180.0)
#define radiansToDegrees(angleRadians) (angleRadians * 180.0 / PI)

#ifdef NATIVE
// include common file for the project in native env for gcc compiler
#include <cstdint> 
#include <cstdio>
#include <cmath>
#endif

#ifdef ARDUINO
// include common file for the project in native env for arduino compiler
#include <Arduino.h>
#endif

#include "controller_structs.h"

class SpeedController
{
private:
public:
  SpeedController()
  {
  }

  void update_airspeed(Airspeed *zip_speed, float v_x_wind, float v_y_wind, int16_t d_x, int8_t d_y);

  void update_airspeed(Airspeed *zip_speed, float v_x_wind, float v_y_wind, float val1, float val2, bool avoid = false);

  void update_airspeed(Airspeed *zip_speed, float v_x_wind, float v_y_wind, float theta);

  void update_airspeed(Airspeed *zip_speed, float v_x_wind, float v_y_wind);
};

#endif