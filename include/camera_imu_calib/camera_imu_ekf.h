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
#include <charuco_ros/CharucoCornerMsg.h>
#include <charuco_ros/SingleCorner.h>
#include <camera_imu_calib/ExpectedMeasurement.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>


#include "camera_imu_calib/estimator.h"

#include <camera_imu_calib/IMUCalibration.h>

#define ACC_SAMPLE_SIZE 200
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
        bool initialized_current_pixels;
        bool publish_full_quaternion;
        bool enable_dynamic_update;
        double pixel_difference_threshold;
        bool use_mocap_to_initialize;
        bool FIRST_IMU_MEASUREMENT;
        double mahalanobis_param;

        int accInitSampleCount;
        int cornerSampleCount;

        double last_time_stamp;

        geometry_msgs::Vector3 accSampleAverage;
        geometry_msgs::Vector3 gyroSampleAverage;

        Eigen::Vector3d gravity;
        Eigen::Vector3d gravity_in_board;

        Eigen::MatrixXd expected_measurement;
        Eigen::Vector3d pnp_average_translation;
        Eigen::Vector3d pnp_average_euler;
        Eigen::Vector3d rpy_measurement;
        Eigen::Vector3d w_k0;
        Eigen::Vector3d s_k0;

        void nonLinearPropagation(Eigen::Vector3d omega, Eigen::Vector3d acceleration);
        void nonLinearUpdate();
        void aruco_helper(charuco_ros::SingleCorner metric_corner, charuco_ros::SingleCorner pixel_corner, unsigned int index);
        void initializeAcc(geometry_msgs::Vector3 acc, geometry_msgs::Vector3 gyro);
        void initializePNP(charuco_ros::CharucoCornerMsg charuco_corners);
        Eigen::Vector3d computePNP(charuco_ros::CharucoCornerMsg charuco_corners);
        void getCamParams(const sensor_msgs::CameraInfo& cam_info);
        Eigen::MatrixXd  integration_function(Eigen::MatrixXd x, double t,Eigen::Vector3d w, Eigen::Vector3d s);
        Eigen::MatrixXd  integration_function_P(Eigen::MatrixXd x, double t,Eigen::Vector3d w, Eigen::Vector3d s);
        Eigen::Vector3d simpson_integrate( Eigen::Quaterniond q_Ik_to_I, Eigen::Vector3d acceleration);
        void RK45integrate(Eigen::Vector3d w, Eigen::Vector3d s,double step);
        bool chi2AcceptPixels();
        bool acceptCharucoMeasurement();
        void nonLinearUpdateSequentially(charuco_ros::CharucoCornerMsg charuco_measurement);
        void publish_state();

        cv::Mat cameraMatrix, distortionCoeffs;
        camera_imu_calib::IMUCalibration state_msg;
        charuco_ros::CharucoCornerMsg charuco_measurement;
        charuco_ros::CharucoCornerMsg past_charuco_measurent;

        int number_of_features;
        bool publish_expected_meas_;
        bool enable_partial_update_;
        bool initialized_timer;
        ros::Time initial_time;


        enum StateIndicies
        {   QX,  QY,  QZ, QW,  // Orientation (body wrt world frame)
            PX,  PY,  PZ,      // Position (body wrt world frame)
            U,   V,   W,      // Velocity (body frame)
            BWX, BWY, BWZ,      // Gyro Biases
            BAX, BAY, BAZ,      // Accel Biases
            P_IX, P_IY, P_IZ, //Calibration Position
            Q_IX, Q_IY, Q_IZ, Q_IW // Calibration Orientation
        };

    public:
        CameraIMUEKF();
        ~CameraIMUEKF();

        double fx;
        double fy;
        double cx;
        double cy;
        Eigen::Vector3d q_attitude_error;
        Eigen::Vector3d q_offset_error;

        Eigen::Quaterniond  initial_imu_q;
        Eigen::Vector3d initial_imu_position;

        Eigen::Quaterniond  initial_board_q;
        Eigen::Quaterniond  charuco_pose;
        Eigen::Vector3d initial_board_position;
        double charuco_roll;
        Eigen::Vector3d previous_charuco_roll;


        bool got_camera_parameters;

        void sensorUpdate(sensor_msgs::Imu imu);
        void sensorUpdate(charuco_ros::CharucoCornerMsg aruco_corners);
        void getCameraInfo(const sensor_msgs::CameraInfo &msg);
        void getInitialPose(camera_imu_calib::IMUCalibration msg);
        void getBoardPose(geometry_msgs::PoseStamped msg);
        void getCharucoPose(geometry_msgs::PoseStamped msg);

    };

    double getVectorMagnitude(double x, double y, double z);
}

#endif //CALIBRATION_WS_CAMERA_IMU_EKF_H
