//
// Created by prashant on 7/16/19.
//

#include "camera_imu_calib/camera_imu_ekf.h"

namespace calibration{

    CameraIMUEKF::CameraIMUEKF() :
    nh_private("~"),
    nh_(""),
    initialize(false)
    {
        nh_private.param<double>("estimator_dt", dt, 0.002);

        F = Eigen::MatrixXd(21,21);
        F.setZero();

        B = Eigen::MatrixXd(21,1);
        B.setZero();

        G = Eigen::MatrixXd(21, 12);
        G.setZero();

        P0 = Eigen::MatrixXd(21,21);
        P0.setZero();

        P = Eigen::MatrixXd(21,21);
        P.setZero();

        K = Eigen::MatrixXd(21,32);
        K.setZero();

        z = Eigen::MatrixXd(32,1);

        H = Eigen::MatrixXd(32,21);

        xHat0 = Eigen::MatrixXd(23,1);
        xHat = Eigen::MatrixXd(23,1);
        Q = Eigen::MatrixXd(12,12);

        betaVector = Eigen::MatrixXd(21,1);

        R = Eigen::MatrixXd(32,32);

        I = Eigen::MatrixXd(3,3);
        I.setIdentity();

        reef_msgs::importMatrixFromParamServer(nh_private, xHat0, "xHat0");
        reef_msgs::importMatrixFromParamServer(nh_private, P0, "P0");
        reef_msgs::importMatrixFromParamServer(nh_private, Q, "Q");
//        reef_msgs::importMatrixFromParamServer(nh_private, R, "R");
//        reef_msgs::importMatrixFromParamServer(nh_private, betaVector, "beta");

        ROS_WARN_STREAM("XHat0 is \n" << xHat0);
        ROS_WARN_STREAM("P0 is \n" << P0);
        ROS_WARN_STREAM("Q is \n" << Q);

        dt = 0.01; //initial dt
        Q = Q * dt;

        Estimator::initialize();

        //TODO Make new message or change the message type.
        state_publisher_ = nh_.advertise<reef_msgs::XYZDebugEstimate>("xyz_estimate", 1, true);

    }

    CameraIMUEKF::~CameraIMUEKF() {}

    void CameraIMUEKF::sensorUpdate(sensor_msgs::Imu imu) {

        initialize = true;

        if(!initialize){
            // TODO: Initialize the xhat using PNP
            last_time_stamp = imu.header.stamp.toSec();
            initialize = true;
            return;
        }

        dt = imu.header.stamp.toSec() - last_time_stamp;

        Eigen::Vector3d omega_imu;
        Eigen::Vector3d accelxyz_in_body_frame;
        omega_imu << imu.angular_velocity.x , imu.angular_velocity.y, imu.angular_velocity.z;
        accelxyz_in_body_frame << imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z; //This is a column vector.

        // TODO: Calibrate the IMU and subtract gravity vector from it.
        nonLinearPropagation(omega_imu, accelxyz_in_body_frame);

    }

    void CameraIMUEKF::nonLinearPropagation(Eigen::Vector3d omega, Eigen::Vector3d acceleration) {

        // Break state into variables to make it easier to read (and write) code
        Eigen::Quaterniond world_to_imu_quat;
        world_to_imu_quat.vec() << xHat(0), xHat(1), xHat(2);
        world_to_imu_quat.w() = xHat(3);

        Eigen::Vector3d world_to_imu_position(xHat(4), xHat(5), xHat(6));
        Eigen::Vector3d velocity_W(xHat(7), xHat(8), xHat(9));
        Eigen::Vector3d accel_bias(xHat(10), xHat(11) , xHat(12));
        Eigen::Vector3d gyro_bias(xHat(13), xHat(14) , xHat(15));
        Eigen::Vector3d camera_to_imu_position(xHat(16), xHat(17), xHat(18));

        Eigen::Quaterniond camera_to_imu_quad;
        camera_to_imu_quad.vec() << xHat(19), xHat(20), xHat(21);
        camera_to_imu_quad.w() = xHat(22);

        omega = omega - gyro_bias;
        acceleration = acceleration - accel_bias;

        F.topLeftCorner<3,3>() = -reef_msgs::skew(omega);
        F.block<3,3>(0,9) = -I;
        F.block<3,3>(3,6) = I;
        F.block<3,3>(6,0) = -1 * world_to_imu_quat.toRotationMatrix() * reef_msgs::skew(acceleration);
        F.block<3,3>(6,12)  = -1 * world_to_imu_quat.toRotationMatrix();

        G.topLeftCorner<3,3>() = -I;
        G.block<3,3>(6,3) = -1 * world_to_imu_quat.toRotationMatrix();
        G.block<3,3>(9,6) = I;
        G.block<3,3>(12,9) = I;

        Eigen::MatrixXd true_dynamics(19,1);
        true_dynamics.setZero();
        true_dynamics.block<3,1>(0,0) = velocity_W;
        true_dynamics.block<3,1>(3,0) = world_to_imu_quat.toRotationMatrix() * acceleration;

        Eigen::Matrix4d Omega_matrix;
        Omega_matrix.setZero();
        Omega_matrix.topLeftCorner<3,3>() = -reef_msgs::skew(omega);
        Omega_matrix.block<3,1>(0,3) = omega;
        Omega_matrix.block<1,3>(3,0) = -omega.transpose();

        dt = 0.01;
        world_to_imu_quat = (Eigen::Matrix4d::Identity() + 0.5 * dt * Omega_matrix) * world_to_imu_quat.coeffs() ;

        xHat.block<19,1>(4,0) = xHat.block<19,1>(4,0) + dt * true_dynamics;
        xHat.block<4,1>(0,0) =  world_to_imu_quat.coeffs();

        P = P + (F * P + P * F.transpose() + G * Q * G.transpose()) * dt;









//        Eigen::Quaterniond q_Ik_I;
//        Eigen::Quaterniond current_frame;
//        Eigen::MatrixXd quaternion_integration(4,4);
//
//        current_frame.w() = 1;
//        current_frame.vec() << 0, 0, 0;
//
//        quaternion_integration.topLeftCorner<3,3>() = -1 * reef_msgs::skew(omega);
//        quaternion_integration.block<3,1>(0,3) = omega;
//        quaternion_integration.block<1,3>(3,0) = -1 * omega;
//
//        q_Ik_I = 0.5 * dt * quaternion_integration * current_frame;
//        q_Ik_I.normalize();
//
//        world_to_imu = q_Ik_I * world_to_imu;
//        world_to_imu.normalize();
//
//
//        xHat(0) = q_Ik_I.vec()(0);
//        xHat(1) = q_Ik_I.vec()(1);
//        xHat(2) = q_Ik_I.vec()(2);
//
//        xHat.block<3,1>(6,0) = xHat.block<3,1>(6,0) + dt * world_to_imu.toRotationMatrix() * acceleration;
//
//        xHat.block<3,1>(3,0) = xHat.block<3,1>(3,0) + xHat.block<3,1>(6,0) * dt;
    }

}
