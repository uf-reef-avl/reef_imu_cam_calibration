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
#include <sensor_msgs/CameraInfo.h>
#include <reef_msgs/XYZDebugEstimate.h>
#include <ar_sys/ArucoCornerMsg.h>
#include <ar_sys/SingleCorner.h>
#include <camera_imu_calib/ExpectedMeasurement.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>


#include "camera_imu_calib/estimator.h"

#include <camera_imu_calib/IMUCalibration.h>

#define TOP_LEFT 0
#define TOP_RIGHT 2
#define BOTTOM_LEFT 4
#define BOTTOM_RIGHT 6

#define ACC_SAMPLE_SIZE 250
#define CORNER_SAMPLE_SIZE 20


namespace calibration{
    class CameraIMUEKF : public reef_estimator::Estimator{

    private:
        ros::NodeHandle nh_private;
        ros::NodeHandle nh_;
        ros::Publisher state_publisher_;
        ros::Publisher expect_pixel_publisher_;

        bool initialize;
        bool got_measurement;
        bool accel_calibrated;
        bool initialized_pnp;
        bool publish_full_quaternion;

        int accInitSampleCount;
        int cornerSampleCount;

        double last_time_stamp;

        geometry_msgs::Vector3 accSampleAverage;

        Eigen::MatrixXd expected_measurement;

        Eigen::Vector3d pnp_average_translation;
        Eigen::Vector3d pnp_average_euler;

        void nonLinearPropagation(Eigen::Vector3d omega, Eigen::Vector3d acceleration);
        void nonLinearUpdate();
        void aruco_helper(ar_sys::SingleCorner metric_corner, ar_sys::SingleCorner pixel_corner, unsigned int index, unsigned int position);
        void initializeAcc(geometry_msgs::Vector3 acc);
        void initializePNP(ar_sys::ArucoCornerMsg aruco_corners);
        void getCamParams(const sensor_msgs::CameraInfo& cam_info);

        void publish_state();

        cv::Mat cameraMatrix, distortionCoeffs;
        camera_imu_calib::IMUCalibration state_msg;

        int number_of_features;
        bool publish_expected_meas_;
        bool enable_partial_update_;

    public:
        CameraIMUEKF();
        ~CameraIMUEKF();

        int fx;
        int fy;
        int cx;
        int cy;
        Eigen::Vector3d q_attitude_error;
        Eigen::Vector3d q_offset_error;

        bool got_camera_parameters;

        void sensorUpdate(sensor_msgs::Imu imu);
        void sensorUpdate(ar_sys::ArucoCornerMsg aruco_corners);
        void getCameraInfo(const sensor_msgs::CameraInfo &msg);

    };

    double getVectorMagnitude(double x, double y, double z);
}

#endif //CALIBRATION_WS_CAMERA_IMU_EKF_H
