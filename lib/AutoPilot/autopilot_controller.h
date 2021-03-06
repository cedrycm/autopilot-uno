//**********************autopilot_controller.h**********************
//Contains definition of the AutoPilot class and its methods
//
#ifndef autopilot_controller_h
#define autopilot_controller_h

#ifdef NATIVE
// include common file for the project in native env for gcc compiler
#include <cstdint>
#include <cstdio>
#include <cmath>
#include <cstring>
#endif

#ifdef ARDUINO
// include common file for the project in native env for arduino compiler
#include <Arduino.h>
#endif

#include "detector.h"
#include "speed_controller.h"
#include "package_controller.h"
#include "controller_structs.h"
#include "controller_enums.h"

#define VEHICLE_AIRSPEED_X 30.0
#define LIDAR_DELIVERY_DIAMETER 1.5
#define DELIVERY_ZONE 100.0
#define VEHICLE_AVOID_THRESHOLD 50.0
#define VEHICLE_DROP_THRESHOLD 30.0
#define ABS_AVOIDANCE_ANGLE 3.0

class AutoPilot
{
private:
    Telemetry telemetry = {0};
    Command command = {0};
    Measurements target_obj = {0};
    Airspeed zip_speed = {0};

    DetectorController detector_ctrl;
    SpeedController speed_ctrl;
    PackageController package_ctrl;

    CtrlFlags statusFlag;

    size_t PackedCmdSize() const;
    size_t PackedTelemSize() const;

    void interpret_data();

public:
    AutoPilot() : detector_ctrl(), speed_ctrl(), package_ctrl()
    {
        statusFlag = CtrlFlags::APPROACH_TARGET;
        zip_speed.v_x = VEHICLE_AIRSPEED_X;
    }
    int send_receive_data(unsigned char *buffer, size_t telem_size, unsigned char *cmd_buff, size_t cmd_size);

    int send_receive_data(Telemetry *rx_telemetry, Command *tx_command);
};

#endif