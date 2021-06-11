
#include <unity.h>
#include <vector>
#include <string>
#include <fstream>
#include <cstdint>
#include <iostream>
#include <stdio.h>
#include "autopilot_controller.h"

AutoPilot pilot;
Command cmd;
Telemetry tele;

const size_t BUFFERSIZE = 44;
char buff1[BUFFERSIZE];
unsigned char buffer[BUFFERSIZE];
unsigned char cmd_buff[8];
const char *fname = "test\\native_test\\telem_file.bin";

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

size_t cmd_size = sizeof(Command);

void test_autopilot_send_receive(void)
{
  size_t telem_size = sizeof(Telemetry);

  TEST_ASSERT_EQUAL_INT(0, pilot.send_receive_data(&tele, &cmd));
}

int main()
{

  UNITY_BEGIN();
  unsigned int i = 1;
  char *c = (char *)&i;
  if (*c)
    std::cout << "Little endian" << std::endl;
  else
    std::cout << "Big endian" << std::endl;

  std::ifstream fl(fname, std::ios::binary | std::ios::in);
  fl.seekg(0, fl.beg);
  while (!fl.eof())
  {
    fl.read((char *)&tele, BUFFERSIZE);
    std::streamsize s = fl.gcount();

    tele.timestamp = __builtin_bswap16(tele.timestamp);
    tele.recovery_x_error = __builtin_bswap16(tele.recovery_x_error);
    tele.wind_vector_x = ReverseFloat(tele.wind_vector_x);
    tele.wind_vector_y = ReverseFloat(tele.wind_vector_y);

    ReverseArr(tele.lidar_samples, 0, sizeof(tele.lidar_samples) / sizeof(uint8_t) - 1);

    //std::cout << "Wind Speed: " << tele.lidar_samples << std::endl;
    for (int i = 31 - 1; i >= 0; i--)
      std::cout << "Items: " << tele.lidar_samples[i];

    RUN_TEST(test_autopilot_send_receive);

    memcpy(&cmd, cmd_buff, cmd_size);
    std::cout << "Speed: " << cmd.lateral_airspeed << std::endl;
  }
  return 0;
}

template <class T>
T my_ntoh(unsigned char *buf)
{
  const auto s = sizeof(T);
  T value = 0;
  for (unsigned i = 0; i < s; i++)
    value |= buf[i] << CHAR_BIT * (s - 1 - i);
  return value;
}