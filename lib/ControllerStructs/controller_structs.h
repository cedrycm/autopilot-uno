//**********************telem_structs.h*************************************
//Header included by the code for the definition of
//Telemetry and Command structs for interprocess communication
#ifndef telem_struct_h
#define telem_struct_h

#ifdef NATIVE
// include common file for the project in native env for gcc compiler
#include <cstdint>
#endif

#ifdef ARDUINO
// include common file for the project in native env for arduino compiler
#include <Arduino.h>
#endif

typedef struct
{
  uint16_t timestamp;        //time stamp from current sim
  int16_t recovery_x_error;  //proximal distance to recovery
  float wind_vector_x;       //wind velocity x
  float wind_vector_y;       //wind velocity y
  int8_t recovery_y_error;   //lateral distance to recovery
  uint8_t lidar_samples[31]; //lidar sample array
} Telemetry;

typedef struct
{
  float lateral_airspeed;
  uint8_t drop_package;
  uint8_t padding[3];
} Command;

typedef struct __attribute__((packed))
{
  // send first
  uint16_t start_seq; // 0x0210, 0x10 will be sent first
  uint8_t len;        // length of payload
  Command tx_data;
  uint8_t checksum;
  uint16_t end_seq; // 0x0310, 0x10 will be sent first
  // send last
} Packet;

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