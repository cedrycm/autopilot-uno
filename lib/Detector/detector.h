

#ifndef detector_h
#define detector_h

#ifdef NATIVE
// include common file for the project in native env for gcc compiler
#include <cstdint>
#include <cstdio>
#include <cmath>
#define PI 3.14159265
#endif

#ifdef ARDUINO
// include common file for the project in native env for arduino compiler
#include <Arduino.h>
#endif

#include <CircularBuffer.h>
#include <LinkedList.h>

#include "controller_structs.h"

#define TREE_RADIUS 3.0
#define DELIVERY_SITE_LIDAR_DIAMETER 1.0
#define MAX_LIDAR_DISTANCE 255
#define MAX_LIDAR_ANGLE 15.0

#define degreesToRadians(angleDegrees) (angleDegrees * PI / 180.0)
#define radiansToDegrees(angleRadians) (angleRadians * 180.0 / PI)

typedef struct
{
  uint8_t *sample_set;
} LidarSamples;

class DetectorController
{
private:
  CircularBuffer<uint8_t *, 5> lidar_buffer;

  LinkedList<uint8_t> avg_samples;

  void flatten();

  void clear_groups();

  void get_closest_object(Measurements *closest_object);

  void euclid_values(Measurements *object, int idx_first, int idx_last);

public:
  DetectorController() : groups()
  {
    for (int i = 0; i < 32; i++)
    {
      avg_samples.add(0);
    }
  }

  Group groups[31];

  size_t num_groups = 0;

  void clr(); //delete after testing

  void interpret_lidar(uint8_t (&lidar_samples)[31], Measurements *closest_object);

  void avoid_collision();
};

#endif