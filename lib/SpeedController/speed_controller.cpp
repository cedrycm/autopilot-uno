#include "speed_controller.h"

void SpeedController::update_airspeed(Airspeed *zip_speed, float v_x_wind, float v_y_wind, float theta) //Assign lateral airspeed to zip_speed.v_y given object angle from vehicle
{
  zip_speed->v_y = ((zip_speed->v_x + v_x_wind) * atan(degreesToRadians(theta))) - v_y_wind;
}

void SpeedController::update_airspeed(Airspeed *zip_speed, float v_x_wind, float v_y_wind, float val1, float val2, bool avoid) //Assign lateral airspeed to zip_speed.v_y given target distance vectors. Optional bool input theta range to avoid.
{
  float theta;
  if (avoid)
  {
    if (abs(val1) < abs(val2))
    {
      float theta1 = val1;
      theta = theta1 > 0 ? theta1++ : theta1--;
    }
    else if (abs(val2) < abs(val1))
    {
      float theta2 = val2;
      theta = theta2 > 0 ? theta2++ : theta2--;
    }
  }
  else
  {
    theta = atan(val1 / val2);

    zip_speed->v_y = ((zip_speed->v_x + v_x_wind) * atan(theta)) - v_y_wind;
  }
}

void SpeedController::update_airspeed(Airspeed *zip_speed, float v_x_wind, float v_y_wind, int16_t d_x, int8_t d_y)
{
  float theta = d_y != 0 ? atan(d_x / d_y) : 0;

  zip_speed->v_y = ((zip_speed->v_x + v_x_wind) * atan(theta)) - v_y_wind;
}

void SpeedController::update_airspeed(Airspeed *zip_speed, float v_x_wind, float v_y_wind) //return lateral airspeed to go counter wind
{
  zip_speed->v_y = (-1) * v_y_wind;
}