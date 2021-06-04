#include "package_controller.h"

int PackageController::check_for_target(int v_x_sum, int v_y_sum, float theta, float distance, int16_t current_time)
{
  int16_t time_elapsed = current_time - drop_timestamp;
  float target_x = distance * sin(degreesToRadians(theta));
  float target_y = distance * cos(degreesToRadians(theta));
  bool in_range = PackageController::contains(target_x, target_y, v_x_sum * PACKAGE_DROP_TIME, v_y_sum * PACKAGE_DROP_TIME);

  if ((0 < time_elapsed <= 500) || (!in_range))
  {
    return 0;
  }
  else if (in_range)
  {
    drop_timestamp = current_time;
    return 1;
  }
  else
    return 0;
}

bool PackageController::contains(float target_x, float target_y, float position_x, float position_y)
{
  float delta_x = abs(target_x - position_x);
  float delta_y = abs(target_y - position_y);

  return (pow(delta_x, 2) + pow(delta_y, 2)) < pow((DELIVERY_SITE_RADIUS - 1.0), 2);
}