
#ifdef NATIVE

#include <unity.h>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <cstdlib>
#include "header.h"

Packet tx_packet; // store packet to be sent
//Telemetry rx_data; // store received data

CircularBuffer<Telemetry *, 5> telem_buffer;

Telemetry telemetry_array[5];
uint8_t avg_samples[31];

Group groups[31];
size_t num_groups = 0;

Measurements target_object;

Airspeed zip_speed;

CtrlFlags statusFlag;

int16_t drop_timestamp = 0;

const size_t BUFFERSIZE = 44;
char buff1[BUFFERSIZE];
unsigned char buffer[BUFFERSIZE];
unsigned char cmd_buff[8];
const char *fname = "test\\native_test\\telem_file.bin";
std::ifstream fl(fname, std::ios::binary | std::ios::in);

int run_test();

void test_autopilot_send_receive(void)
{
  TEST_ASSERT_EQUAL_INT(0, run_test());
}

int run_test()
{
  int count = 0;
  fl.seekg(0, fl.beg);
  while (!fl.eof())
  {
    Telemetry *rx_data = &telemetry_array[count];
    char temp[44];
    fl.read((char *)rx_data, BUFFERSIZE);

    if (fl)
    {
      //Perform endian conversion
      rx_data->timestamp = __builtin_bswap16(rx_data->timestamp);
      rx_data->recovery_x_error = __builtin_bswap16(rx_data->recovery_x_error);
      rx_data->wind_vector_x = ReverseFloat(rx_data->wind_vector_x);
      rx_data->wind_vector_y = ReverseFloat(rx_data->wind_vector_y);

      telem_buffer.push(rx_data); //add to bvuffer

      //Interpret Telemetry data
      InterpretLidar();

      GetClosestObject();

      InterpretData();

      std::cout << "Lateral Airpspeed: " << tx_packet.tx_data.lateral_airspeed << std::endl;
      std::cout << "Drop: " << (bool)tx_packet.tx_data.drop_package << std::endl;
      //SendPacket();

      count--;
      count = (count + 5) % 5;
    }
  }
  return 0;
}

int main()
{
  statusFlag = CtrlFlags::APPROACH_TARGET;
  // init tx packet
  tx_packet.start_seq = 0x0210;
  tx_packet.end_seq = 0x0310;

  //init rx packet
  tx_packet.len = COMMAND_SIZE;
  UNITY_BEGIN();

  RUN_TEST(test_autopilot_send_receive);

  return 0;
}

float ReverseFloat(const float inFloat)
{
  float retVal;
  char *floatToConvert = (char *)&inFloat;
  char *returnFloat = (char *)&retVal;

  // swap the bytes into a temporary buffer
  returnFloat[0] = floatToConvert[3];
  returnFloat[1] = floatToConvert[2];
  returnFloat[2] = floatToConvert[1];
  returnFloat[3] = floatToConvert[0];

  return retVal;
}

void ReverseArr(uint8_t arr[], int start, int end)
{
  uint8_t temp;
  while (start < end)
  {
    temp = arr[start];
    arr[start] = arr[end];
    arr[end] = temp;
    start++;
    end--;
  }
}

uint8_t CalcChecksum(void *data, uint8_t len)
{
  uint8_t checksum = 0;
  uint8_t *addr;
  for (addr = (uint8_t *)data; addr < (data + len); addr++)
  {
    // xor all the bytes
    checksum ^= *addr; // checksum = checksum xor value stored in addr
  }
  return checksum;
}

bool ReadPacket(Telemetry rx_data)
{
  char *temp[44];
  fl.read((char *)&temp, BUFFERSIZE);
  std::streamsize s = fl.gcount();

  if (s != TELEMETRY_SIZE)
  {
    // cannot receive required length within timeout

    return false;
  }

  // Yeah! a valid packet is received

  return true;
}

void SendPacket()
{
  tx_packet.checksum = CalcChecksum(&tx_packet.tx_data, tx_packet.len); //produce new checksum

  std::cout << ((char *)&tx_packet, sizeof(tx_packet)); // send the packet
}

void InterpretLidar() //parse array of lidar points and groups objects into an array of structs that hold the index of the values two edges of an individual object 
{
  int last = 0;

  for (decltype(telem_buffer)::index_t i = 0; i < telem_buffer.size(); i++) //iterate through buffer arrays
  {
    for (int j = 0; j < 31; j++) //iterate through list elements
    {
      avg_samples[j] += telem_buffer[i]->lidar_samples[j];

      if (i == (telem_buffer.size() - 1)) //if outside loop is on the last list - average total and group into objects
      {
        avg_samples[j] = (avg_samples[j] / telem_buffer.size());

        if (j == 0) //if first entry in list - create first group entry
        {
          num_groups++;

          groups[num_groups - 1].start_index = j;
          groups[num_groups - 1].end_index = j;

          last = j;
        }
        else if ((avg_samples[j] != 0 && avg_samples[last] != 0) && (abs(avg_samples[j] - avg_samples[last]) <= TREE_RADIUS)) //extend object to adjacent index
        {
          groups[num_groups - 1].end_index = j;
          last = j;
        }
        else if (avg_samples[j] == 0 && avg_samples[last] == 0) //if adjacent zeros - group together
        {
          groups[num_groups - 1].end_index = j;
          last = j;
        }
        else //create next entry
        {
          num_groups++;

          groups[num_groups - 1].start_index = j;
          groups[num_groups - 1].end_index = j;

          last = j;
        }
      }
    }
  }
}

void GetClosestObject() //Fills euclidian measurements for objects in the lidar samples
{
  Measurements current = {0};

  float obstacle_distance, delivery_distance;

  obstacle_distance = MAX_LIDAR_DISTANCE;
  delivery_distance = MAX_LIDAR_DISTANCE;

  for (int i = 0; (unsigned)i < num_groups; i++)
  {
    int idx_first = groups[i].start_index;
    int idx_last = groups[i].end_index;

    if (avg_samples[idx_first] != 0)
    {
      if (idx_first != idx_last)
      {
        EuclidValues(&current, idx_first, idx_last);

        if (current.obstacle_diameter <= DELIVERY_SITE_LIDAR_DIAMETER && current.obstacle_distance < delivery_distance)
        {
          target_object.delivery_distance = current.obstacle_distance;
          target_object.delivery_theta = current.theta_mid;
        }
        else if (current.obstacle_diameter >= DELIVERY_SITE_LIDAR_DIAMETER && current.obstacle_distance < obstacle_distance)
        {
          target_object.obstacle_distance = current.obstacle_distance;
          target_object.obstacle_diameter = current.obstacle_diameter;
          target_object.theta_edge1 = current.theta_edge1;
          target_object.theta_edge2 = current.theta_edge2;
          target_object.theta_mid = current.theta_mid;
        }
      }
      else if (idx_first == idx_last)
      {
        if (avg_samples[idx_first] <= DELIVERY_SITE_LIDAR_DIAMETER && current.obstacle_distance < delivery_distance)
        {
          target_object.delivery_distance = avg_samples[idx_first];
          target_object.delivery_theta = (-1) * (idx_first) + 15;
        }
      }
    }
  }
  ClearGroups();
}

void EuclidValues(Measurements *object, int idx_first, int idx_last) //Compute euclidian values for objects
{
  float d_1, d_2, x_1, x_2, y_1, y_2, m_x, m_y;
  d_1 = avg_samples[idx_first];
  d_2 = avg_samples[idx_last];

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

void ClearGroups() //Needed to clear groups for next iteration
{
  num_groups = 0;

  for (int i = 0; i < 31; i++)
  {
    avg_samples[i] = 0;

    groups[i].start_index = 0;
    groups[i].end_index = 0;
  }
}

void InterpretData() //Updates airspeed and command telemetry according to control status flag
{
  Telemetry *last_telem = telem_buffer.last(); //retrieve last data packet stored to buffer

  if ((abs(telem_buffer.last()->recovery_x_error) < DELIVERY_ZONE) || (statusFlag == CtrlFlags::RECOVER))
  {
    //update lateral velocity to head into recovery site
    UpdateAirspeed(&zip_speed, last_telem->wind_vector_x, last_telem->wind_vector_y,
                   last_telem->recovery_x_error, last_telem->recovery_y_error);
  }
  else if (target_object.obstacle_distance < VEHICLE_AVOID_THRESHOLD && (abs(target_object.theta_edge1) < ABS_AVOIDANCE_ANGLE || abs(target_object.theta_edge2) < ABS_AVOIDANCE_ANGLE))
  {
    statusFlag = CtrlFlags::AVOID_COLLISION;

    UpdateAirspeed(&zip_speed, last_telem->wind_vector_x, last_telem->wind_vector_y,
                   target_object.theta_edge1, target_object.theta_edge2);
  }
  else if (target_object.delivery_distance != 0)
  {
    statusFlag = target_object.delivery_distance <= VEHICLE_DROP_THRESHOLD ? CtrlFlags::DELIVER_PACKAGE : CtrlFlags::APPROACH_TARGET; //Approach Zip Delivery Site

    UpdateAirspeed(&zip_speed, last_telem->wind_vector_x, last_telem->wind_vector_y,
                   target_object.delivery_theta);
  }
  else
  {
    UpdateAirspeed(&zip_speed, last_telem->wind_vector_x, last_telem->wind_vector_y);
  }

  tx_packet.tx_data.lateral_airspeed = zip_speed.v_y;

  if (statusFlag == CtrlFlags::DELIVER_PACKAGE)
  {
    float v_x_sum = zip_speed.v_x + last_telem->wind_vector_x;
    float v_y_sum = zip_speed.v_y + last_telem->wind_vector_y;

    tx_packet.tx_data.drop_package = CheckForTarget(v_x_sum, v_y_sum, &target_object.delivery_theta, &target_object.delivery_distance, last_telem->timestamp);
  }
  else
  {
    tx_packet.tx_data.drop_package = 0;
  }
}

void UpdateAirspeed(Airspeed *zip_speed, float v_x_wind, float v_y_wind, float theta) //Assign lateral airspeed to zip_speed.v_y given object angle from vehicle
{
  zip_speed->v_y = ((zip_speed->v_x + v_x_wind) * atan(degreesToRadians(theta))) - v_y_wind;
}

void UpdateAirspeed(Airspeed *zip_speed, float v_x_wind, float v_y_wind, float val1, float val2, bool avoid) //Assign lateral airspeed to zip_speed.v_y given target distance vectors. Optional bool input theta range to avoid.
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

void UpdateAirspeed(Airspeed *zip_speed, float v_x_wind, float v_y_wind, int16_t d_x, int8_t d_y)
{
  float theta = d_y != 0 ? atan(d_x / d_y) : 0;

  zip_speed->v_y = ((zip_speed->v_x + v_x_wind) * atan(theta)) - v_y_wind;
}

void UpdateAirspeed(Airspeed *zip_speed, float v_x_wind, float v_y_wind) //return lateral airspeed to go counter wind
{
  zip_speed->v_y = (-1) * v_y_wind;
}

int CheckForTarget(float v_x_sum, float v_y_sum, float *theta, float *distance, int16_t current_time) //Returns 1 if calculated package fall lands object in delivery zone - 0 if it doesn't
{
  int16_t time_elapsed = current_time - drop_timestamp;
  float target_x = *distance * sin(degreesToRadians(*theta));
  float target_y = *distance * cos(degreesToRadians(*theta));
  bool in_range = Contains(&target_x, &target_y, (v_x_sum * PACKAGE_DROP_TIME), (v_y_sum * PACKAGE_DROP_TIME));

  if (((0 < time_elapsed) && (time_elapsed <= DROP_TIMEOUT)) || (!in_range))
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

bool Contains(float *target_x, float *target_y, float position_x, float position_y) //if within drop distance -> check if my package will land according to the sum of my vehicles + wind velocity vectors
{
  float delta_x = abs(*target_x - position_x);
  float delta_y = abs(*target_y - position_y);

  return (pow(delta_x, 2) + pow(delta_y, 2)) < pow((DELIVERY_SITE_RADIUS - 1.0), 2);
}

#endif