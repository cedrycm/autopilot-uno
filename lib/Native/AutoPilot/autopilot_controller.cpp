#include "autopilot_controller.h"

int AutoPilot::send_receive_data(char *buffer, size_t telem_size, char *cmd_buff, size_t cmd_size)
{

    //check if buffer size is the same size as Telemetry struct
    if (telem_size == AutoPilot::PackedTelemSize())
    {
        memcpy(&telemetry, buffer, telem_size);
    }

    AutoPilot::interpret_data();

    //check if cmd_buff size is the same size as Command struct
    if (cmd_size == PackedCmdSize())
    {
        memcpy(cmd_buff, &command, cmd_size);
    }
    return 0;
}

void AutoPilot::interpret_data()
{
    int drop_status;
    float v_y;
    float d_1_2, d_x, d_y, theta;

    detector_ctrl.interpret_lidar(telemetry.lidar_samples, &target_obj);

    if ((abs(AutoPilot::telemetry.recovery_x_error) < DELIVERY_ZONE) || (statusFlag == CtrlFlags::RECOVER))
    {
        //update lateral velocity to head into recovery site
        speed_ctrl.update_airspeed(&zip_speed, telemetry.wind_vector_x, telemetry.wind_vector_y,
                                   telemetry.recovery_x_error, telemetry.recovery_y_error);
    }
    else if (target_obj.obstacle_distance < VEHICLE_AVOID_THRESHOLD && (abs(target_obj.theta_edge1) < ABS_AVOIDANCE_ANGLE || abs(target_obj.theta_edge2) < ABS_AVOIDANCE_ANGLE))
    {
        statusFlag == CtrlFlags::AVOID_COLLISION;

        speed_ctrl.update_airspeed(&zip_speed, telemetry.wind_vector_x, telemetry.wind_vector_y,
                                   target_obj.theta_edge1, target_obj.theta_edge2);
    }
    else if (target_obj.delivery_distance != 0)
    {
        statusFlag = target_obj.delivery_distance == VEHICLE_DROP_THRESHOLD ? CtrlFlags::DELIVER_PACKAGE : CtrlFlags::APPROACH_TARGET; //Approach Zip Delivery Site

        speed_ctrl.update_airspeed(&zip_speed, telemetry.wind_vector_x, telemetry.wind_vector_y,
                                   target_obj.delivery_theta);
    }
    else
    {
        speed_ctrl.update_airspeed(&zip_speed, telemetry.wind_vector_x, telemetry.wind_vector_y);
    }

    command.lateral_airspeed = zip_speed.v_y;

    if (statusFlag == CtrlFlags::DELIVER_PACKAGE)
    {
        float v_x_sum = zip_speed.v_x + telemetry.wind_vector_x;
        float v_y_sum = zip_speed.v_y + telemetry.wind_vector_y;

        command.drop_package = package_ctrl.check_for_target(v_x_sum, v_y_sum, target_obj.delivery_theta, target_obj.delivery_distance, telemetry.timestamp);
    }
    else
    {
        command.drop_package = 0;
    }
}

size_t AutoPilot::PackedTelemSize() const { return sizeof(Telemetry); }

size_t AutoPilot::PackedCmdSize() const { return sizeof(Command); }

