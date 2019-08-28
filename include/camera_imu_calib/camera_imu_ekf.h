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
#include <ar_sys/ArucoCornerMsg.h>
#include <ar_sys/SingleCorner.h>


#include "camera_imu_calib/estimator.h"

#define TOP_LEFT 0
#define TOP_RIGHT 2
#define BOTTOM_LEFT 4
#define BOTTOM_RIGHT 6


namespace calibration{
    class CameraIMUEKF : public reef_estimator::Estimator{

    private:
        ros::NodeHandle nh_private;
        ros::NodeHandle nh_;
        ros::Publisher state_publisher_;

        double last_time_stamp;
        bool initialize;

        void nonLinearPropagation(Eigen::Vector3d omega, Eigen::Vector3d acceleration);
        void nonLinearUpdate(Eigen::MatrixXd y_exp);
        void aruco_helper(ar_sys::SingleCorner metric_corner, ar_sys::SingleCorner pixel_corner, Eigen::MatrixXd& y_exp, unsigned int index, unsigned int position);

        


    public:
        CameraIMUEKF();
        ~CameraIMUEKF();

        int fx;
        int fy;
        int cx;
        int cy;


        void sensorUpdate(sensor_msgs::Imu imu);
        void sensorUpdate(ar_sys::ArucoCornerMsg aruco_corners);



    };
}

#endif //CALIBRATION_WS_CAMERA_IMU_EKF_H
