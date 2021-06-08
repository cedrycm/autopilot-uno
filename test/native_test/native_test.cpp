
#include <unity.h>
#include <vector>
#include <string>
#include <fstream>
#include <cstdint>
#include <iostream>
#include <stdio.h>
#include "autopilot_controller.h"

template <class T>
T my_ntoh(unsigned char *buf);

AutoPilot pilot;
Command cmd;

const size_t BUFFERSIZE = 44;
char buff1[BUFFERSIZE];
unsigned char buffer[BUFFERSIZE];
unsigned char cmd_buff[8];
const char *fname = "test\\native_test\\telem_file.bin";

size_t cmd_size = sizeof(cmd_buff);
void test_autopilot_send_receive(void)
{
  size_t telem_size = sizeof(buff1);
  for (int i = 0; i < telem_size; i++)
  {
    buffer[i] ^= buff1[i];
  }
  TEST_ASSERT_EQUAL_INT(0, pilot.send_receive_data(buffer, telem_size, cmd_buff, cmd_size));
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
    fl.read(buff1, BUFFERSIZE);
    std::streamsize s = fl.gcount();

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