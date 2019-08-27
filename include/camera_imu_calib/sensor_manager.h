//
// Created by prashant on 7/16/19.
//

#ifndef CALIBRATION_WS_SENSOR_MANAGER_H
#define CALIBRATION_WS_SENSOR_MANAGER_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose.h>

#include "camera_imu_ekf.h"

namespace calibration{
    class SensorManager{
    private:

        ros::NodeHandle private_nh_;
        ros::NodeHandle nh_;

        ros::Subscriber imu_subscriber_;
        ros::Subscriber mocap_board_subscriber_;
        ros::Subscriber mocap_imu_subscriber_;
        ros::Subscriber mocap_twist_subscriber_;
        ros::Subscriber aruco_subscriber_;

        std::string imu_topic_name;
        std::string mocap_board_name;
        std::string mocap_imu_name;
        std::string mocap_twist_name;
        std::string aruco_topic_name;

    public:

        sensor_msgs::Imu imu_msg;
        geometry_msgs::Pose mocap_board;
        geometry_msgs::Pose mocap_imu;
        geometry_msgs::Pose aruco_msg;

        CameraIMUEKF calib_obj;

        void imuCallback(const sensor_msgs::Imu);





    };
}

#endif //CALIBRATION_WS_SENSOR_MANAGER_H
