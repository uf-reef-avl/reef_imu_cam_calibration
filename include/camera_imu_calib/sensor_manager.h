//
// Created by prashant on 7/16/19.
//

#ifndef CALIBRATION_WS_SENSOR_MANAGER_H
#define CALIBRATION_WS_SENSOR_MANAGER_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Pose.h>
#include <charuco_ros/CharucoCornerMsg.h>
#include <opencv2/core.hpp>
#include <tf2_eigen/tf2_eigen.h>

#include <eigen3/Eigen/Eigen>

#include "camera_imu_ekf.h"

#define TOTAL_POSE 10;

namespace calibration{
    class SensorManager{
    private:

        ros::NodeHandle private_nh_;
        ros::NodeHandle nh_;

        ros::Subscriber imu_subscriber_;
        ros::Subscriber corner_subscriber;
        ros::Subscriber initial_pose_subscriber;
        ros::Subscriber camera_info_subscriber;
        ros::Subscriber mocap_board_subscriber;

        void imuCallback(const sensor_msgs::ImuConstPtr &msg);
        void cornerCallback(const charuco_ros::CharucoCornerMsg &msg);
        void initialPoseCallback(const camera_imu_calib::IMUCalibrationConstPtr  &msg);
        void cameraInfoCallback(const sensor_msgs::CameraInfo &msg);
        void boardPoseCallback(const geometry_msgs::TransformStampedConstPtr  &msg);
        CameraIMUEKF calib_obj;
        int num_pose;

    public:

        SensorManager();
        ~SensorManager(){}

        sensor_msgs::Imu imu_msg;
        geometry_msgs::Pose mocap_board;
        geometry_msgs::Pose mocap_imu;
        geometry_msgs::Pose aruco_msg;
        charuco_ros::CharucoCornerMsg corner_msg;

    };
}

#endif //CALIBRATION_WS_SENSOR_MANAGER_H
