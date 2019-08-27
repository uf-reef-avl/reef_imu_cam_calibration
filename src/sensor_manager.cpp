//
// Created by prashant on 7/16/19.
//
#include "../include/camera_imu_calib/sensor_manager.h"

#include <sensor_msgs/Imu.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "camera_imu_calibration");


    sensor_msgs::Imu test_msg;
    test_msg.angular_velocity.x = 0.0082;
    test_msg.angular_velocity.y = 0.1087;
    test_msg.angular_velocity.z = 0.0269;

    test_msg.linear_acceleration.x = 0.0043;
    test_msg.linear_acceleration.y = 0.0142;
    test_msg.linear_acceleration.z = -0.0013;

    calibration::SensorManager obj;
    obj.calib_obj.sensorUpdate(test_msg);

    ros::spin();
    return 0;
}
