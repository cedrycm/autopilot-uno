

#ifndef package_controller_h
#define package_controller_h

#define DELIVERY_SITE_RADIUS 5.0
#define PACKAGE_DROP_TIME 0.5
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

class PackageController
{
private:
  int16_t drop_timestamp = 0;

public:
  PackageController()
  {
  }

  int check_for_target(int v_x_sum, int v_y_sum, float theta, float distance, int16_t current_time);

  bool contains(float target_x, float target_y, float position_x, float position_y);
};

#endif
