//
// Created by prashant on 7/16/19.
//

#ifndef CALIBRATION_WS_CAMERA_IMU_EKF_H
#define CALIBRATION_WS_CAMERA_IMU_EKF_H

#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <tf2_eigen/tf2_eigen.h>
#include <reef_msgs/dynamics.h>
#include <reef_msgs/matrix_operation.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <reef_msgs/XYZDebugEstimate.h>

#include "camera_imu_calib/estimator.h"

namespace calibration{
    class CameraIMUEKF : public reef_estimator::Estimator{

    private:
        ros::NodeHandle nh_private;
        ros::NodeHandle nh_;
        ros::Publisher state_publisher_;

        double last_time_stamp;

        bool initialize;

        Eigen::Quaterniond world_to_imu;
        Eigen::Vector3d position_to_imu;

        


    public:
        CameraIMUEKF();
        ~CameraIMUEKF();

        void nonLinearPropogation(Eigen::Vector3d omega, Eigen::Vector3d acceleration);
        void nonLinearUpdate();

        void sensorUpdate(sensor_msgs::Imu imu);



    };
}

#endif //CALIBRATION_WS_CAMERA_IMU_EKF_H
