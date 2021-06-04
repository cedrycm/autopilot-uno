#include "detector.h"

void DetectorController::interpret_lidar(uint8_t (&lidar_samples)[31], Measurements *closest_object)
{
  // LidarSamples temp = {lidar_samples};

  //   temp.sample_set = ;

  DetectorController::lidar_buffer.push(lidar_samples);
  DetectorController::flatten();

  Measurements object = {0};

  DetectorController::get_closest_object(closest_object);

  DetectorController::clear_groups();
}

void DetectorController::flatten() //flatten buffer of samples to one avg of previous entries (max of 5)
{

  uint8_t avg_distance, last, group_idx;

  for (decltype(lidar_buffer)::index_t i = 0; i < DetectorController::lidar_buffer.size(); i++) //iterate through buffer arrays
  {
    for (int j = 0; j < 31; j++) //iterate through list elements
    {
      DetectorController::avg_samples[j] += DetectorController::lidar_buffer[i][j]; //add lidar value to appropiate index slot

      if (i == (DetectorController::lidar_buffer.size() - 1)) //if outside loop is on the last list - average total and group into objects
      {
        size_t group_idx = num_groups - 1;

        DetectorController::avg_samples[j] = (uint8_t)(avg_samples[j] / DetectorController::lidar_buffer.size());

        if (j == 0) //if first entry in list - create first group entry
        {
          DetectorController::num_groups++;

          DetectorController::groups[num_groups - 1].start_index = j;
          DetectorController::groups[num_groups - 1].end_index = j;

          last = j;
        }
        else if ((DetectorController::avg_samples[j] != 0 && DetectorController::avg_samples[last] != 0) && (abs(DetectorController::avg_samples[j] - DetectorController::avg_samples[last]) <= TREE_RADIUS)) //extend object to adjacent index
        {
          DetectorController::groups[num_groups - 1].end_index = j;
          last = j;
        }
        else if (DetectorController::avg_samples[j] == 0 && DetectorController::avg_samples[last] == 0) //if adjacent zeros - group together
        {
          DetectorController::groups[num_groups - 1].end_index = j;
          last = j;
        }
        else //create next entry
        {
          DetectorController::num_groups++;

          DetectorController::groups[num_groups - 1].start_index = j;
          DetectorController::groups[num_groups - 1].end_index = j;

          last = j;
        }
      }
    }
  }
}

void DetectorController::get_closest_object(Measurements *closest_object) //Fills euclidian measurements for objects in the lidar samples
{

  Measurements current = {0};

  float obstacle_distance, delivery_distance;

  obstacle_distance = MAX_LIDAR_DISTANCE;
  delivery_distance = MAX_LIDAR_DISTANCE;

  for (int i = 0; i < DetectorController::num_groups; i++)
  {
    int idx_first = DetectorController::groups[i].start_index;
    int idx_last = DetectorController::groups[i].end_index;

    if (DetectorController::avg_samples[idx_first] != 0)
    {
      if (idx_first != idx_last)
      {
        euclid_values(&current, idx_first, idx_last);

        if (current.obstacle_diameter <= DELIVERY_SITE_LIDAR_DIAMETER && current.obstacle_distance < delivery_distance)
        {
          closest_object->delivery_distance = current.obstacle_distance;
          closest_object->delivery_theta = current.theta_mid;
        }
        else if (current.obstacle_diameter >= DELIVERY_SITE_LIDAR_DIAMETER && current.obstacle_distance < obstacle_distance)
        {
          closest_object->obstacle_distance = current.obstacle_distance;
          closest_object->obstacle_diameter = current.obstacle_diameter;
          closest_object->theta_edge1 = current.theta_edge1;
          closest_object->theta_edge2 = current.theta_edge2;
          closest_object->theta_mid = current.theta_mid;
        }
      }
      else if (idx_first == idx_last)
      {
        if (DetectorController::avg_samples[idx_first] <= DELIVERY_SITE_LIDAR_DIAMETER && current.obstacle_distance < delivery_distance)
        {
          closest_object->delivery_distance = DetectorController::avg_samples[idx_first];
          closest_object->delivery_theta = (-1) * (idx_first) + 15;
        }
      }
    }
  }
}

void DetectorController::euclid_values(Measurements *object, int idx_first, int idx_last) //Compute euclidian values for objects
{
  float d_1, d_2, x_1, x_2, y_1, y_2, m_x, m_y;
  d_1 = DetectorController::avg_samples[idx_first];
  d_2 = DetectorController::avg_samples[idx_last];

  object->theta_edge1 = (-1) * idx_first + 15;
  object->theta_edge2 = (-1) * idx_last + 15;
  x_1 = d_1 * cos(degreesToRadians(object->theta_edge1));
  y_1 = d_1 * sin(degreesToRadians(object->theta_edge1));

  x_2 = d_2 * cos(degreesToRadians(object->theta_edge2));
  y_2 = d_2 * sin(degreesToRadians(object->theta_edge2));

  object->obstacle_diameter = sqrt(pow((x_1 - x_2), 2) + pow((y_1 - y_2), 2));

  m_x = (x_1 + x_2) / 2;
  m_y = (y_1 + y_2) / 2;

  object->obstacle_distance = sqrt(pow(m_x, 2) + pow(m_y, 2));
  object->theta_mid = radiansToDegrees(atan(m_y / m_x));
}

void DetectorController::clear_groups() //Reset groups for next iteration
{
  DetectorController::num_groups = 0;

  for (int i = 0; i < 31; i++)
  {
    DetectorController::avg_samples[i] = 0;

    DetectorController::groups[i].start_index = 0;
    DetectorController::groups[i].end_index = 0;
  }
}

void DetectorController::clr() //Reset groups for next iteration
{
  DetectorController::clear_groups();
}
