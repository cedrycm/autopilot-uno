//**********************telem_structs.h*************************************
//Header included by the code for the definition of
//Telemetry and Command structs for interprocess communication
#ifndef telem_struct_h
#define telem_struct_h

#include <Arduino.h>

typedef struct
{
  uint16_t timestamp;
  int16_t recovery_x_error;
  float wind_vector_x;
  float wind_vector_y;
  int8_t recovery_y_error;
  uint8_t lidar_samples[31];
} Telemetry;

typedef struct
{
  float lateral_airspeed;
  uint8_t drop_package;
  uint8_t padding[3];
} Command;

typedef struct
{
  uint8_t start_index;
  uint8_t end_index;
} Group;

typedef struct
{
  float v_x;
  float v_y;
} Airspeed;

typedef struct
{
  float theta_mid;
  float theta_edge1;
  float theta_edge2;
  float obstacle_distance;
  float obstacle_diameter;
  float delivery_distance;
  float delivery_theta;
} Measurements;

#endif