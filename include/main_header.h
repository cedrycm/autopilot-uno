#ifndef MAIN_HEADER_H_
#define MAIN_HEADER_H_

#include <CircularBuffer.h>
#include <LinkedList.h>

#include "controller_structs.h"
#include "controller_enums.h"

//packet size definitions
#define TELEMETRY_SIZE 44
#define COMMAND_SIZE 8
#define PACKET_SIZE 50
#define LIDAR_SAMPLE_SIZE 31

//world measurements
#define TREE_RADIUS 3.0
#define DELIVERY_SITE_RADIUS 5.0
#define DELIVERY_SITE_LIDAR_DIAMETER 1.0
#define LIDAR_DELIVERY_DIAMETER 1.5
#define MAX_LIDAR_DISTANCE 255
#define MAX_LIDAR_ANGLE 15.0
#define VEHICLE_AIRSPEED_X 30.0

#define DELIVERY_ZONE 100.0          //recovery zone
#define VEHICLE_AVOID_THRESHOLD 50.0 //max distance to check for collision
#define VEHICLE_DROP_THRESHOLD 30.0  //min distance to check for drop
#define ABS_AVOIDANCE_ANGLE 3.0      //max angle range +/- 3deg to check for collision
#define PACKAGE_DROP_TIME 0.5        //time for package to reach the ground
#define DROP_TIMEOUT 500             //500ms before dropping

#define degreesToRadians(angleDegrees) (angleDegrees * PI / 180.0)
#define radiansToDegrees(angleRadians) (angleRadians * 180.0 / PI)

inline float ReverseFloat(const float inFloat) __attribute__((always_inline)); //function for endian conversion of floats

inline void ReverseArr(uint8_t arr[], int start, int end) __attribute__((always_inline)); //function for endian conversion of uint8_t array

inline uint8_t CalcChecksum(void *data, uint8_t len) __attribute__((always_inline)); //check sum function for robust data serialization/deserialization

inline void UpdateAirspeed(Airspeed *zip_speed, float v_x_wind, float v_y_wind, float theta) __attribute__((always_inline)); //Assign lateral airspeed to zip_speed.v_y given object angle from vehicle

inline void UpdateAirspeed(Airspeed *zip_speed, float v_x_wind, float v_y_wind, float val1, float val2, bool avoid) __attribute__((always_inline)); //Assign lateral airspeed to zip_speed.v_y given target distance vectors. Optional bool input theta range to avoid.

inline void UpdateAirspeed(Airspeed *zip_speed, float v_x_wind, float v_y_wind) __attribute__((always_inline)); //return lateral airspeed to go counter wind

inline void UpdateAirspeed(Airspeed *zip_speed, float v_x_wind, float v_y_wind, int16_t d_x, int8_t d_y) __attribute__((always_inline));

inline int CheckForTarget(float v_x_sum, float v_y_sum, float *theta, float *distance, int16_t current_time) __attribute__((always_inline));

inline void ClearGroups() __attribute__((always_inline));

inline void SendPacket() __attribute__((always_inline)); //serialize tx packet

inline bool Contains(float *target_x, float *target_y, float position_x, float position_y) __attribute__((always_inline));

bool ReadPacket(Telemetry *rx_data); //deserialize rx packet

void InterpretLidar(); //parse lidar samples to create objects of Group

void InterpretData(); //Updates airspeed and command telemetry according to control status flag

void GetClosestObject(); //Fills euclidian measurements for objects in the lidar samples

void EuclidValues(Measurements *object, int idx_first, int idx_last);

#endif // MAIN_HEADER_H_
