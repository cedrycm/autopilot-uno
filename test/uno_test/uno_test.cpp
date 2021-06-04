


#ifdef NATIVE_PLATFORM
#   include <stdint.h>
#else
#   include <Arduino.h>
#endif 

#include "autopilot_controller.h"

AutoPilot pilot;

char buffer[44];
char cmd_buff[8];

//uint8_t numray[31] = {0, 1, 2, 3, 4, 10, 0, 2, 3, 4, 3, 10, 12, 12, 0, 0, 3, 1, 3, 4, 5, 0, 0, 0, 0, 0, 1, 3, 4, 5, 5};

int count = 0;

Measurements closest_object = {0};

Telemetry telem = {
    100,
    700,
    20.0,
    -10.0,
    0,
    {0, 1, 2, 3, 4, 10, 0, 2, 3, 4, 3, 10, 12, 12, 0, 0, 3, 1, 3, 4, 5, 0, 0, 0, 0, 0, 1, 3, 4, 5, 5}};

size_t telem_size = sizeof(telem);
size_t cmd_size = sizeof(cmd_buff);

void setup()
{
  // put your setup code here, to run once:
}

void loop()
{
  if (count == 0)
  {
    memcpy(buffer, &telem, telem_size);

    while (count != 5)
    {
      pilot.send_receive_data(buffer, telem_size, cmd_buff, cmd_size);

      count++;
    }
  }
}
