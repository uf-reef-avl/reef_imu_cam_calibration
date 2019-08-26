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

        F = Eigen::MatrixXd(15,15);
        F.setZero();

        B = Eigen::MatrixXd(15,1);
        B.setZero();

        G = Eigen::MatrixXd(15, 12);
        G.setZero();

        P0 = Eigen::MatrixXd(15,15);
        P0.setZero();

        P = Eigen::MatrixXd(15,15);
        P.setZero();

        K = Eigen::MatrixXd(15,32);
        K.setZero();

        z = Eigen::MatrixXd(32,1);

        H = Eigen::MatrixXd(32,15);

        xHat0 = Eigen::MatrixXd(15,1);
        xHat = Eigen::MatrixXd(15,1);
        Q = Eigen::MatrixXd(15,15);

        betaVector = Eigen::MatrixXd(15,15);

        R = Eigen::MatrixXd(32,32);

        I.setIdentity();

        reef_msgs::importMatrixFromParamServer(nh_private, xHat0, "xHat0");
        reef_msgs::importMatrixFromParamServer(nh_private, P0, "P0");
        reef_msgs::importMatrixFromParamServer(nh_private, Q, "Q");
        reef_msgs::importMatrixFromParamServer(nh_private, R, "R");
        reef_msgs::importMatrixFromParamServer(nh_private, betaVector, "beta");

        Estimator::initialize();

        world_to_imu.w() = 1;
        world_to_imu.vec() << 0, 0, 0;

        position_to_imu << 0, 0, 0;

        //TODO Make new message or change the message type.
        state_publisher_ = nh_.advertise<reef_msgs::XYZDebugEstimate>("xyz_estimate", 1, true);

    }

    CameraIMUEKF::~CameraIMUEKF() {}

    void CameraIMUEKF::sensorUpdate(sensor_msgs::Imu imu) {

        if(!initialize){
            last_time_stamp = imu.header.stamp.toSec();
            initialize = true;
            return;
        }

        dt = imu.header.stamp.toSec() - last_time_stamp;

        Eigen::Vector3d omega_imu;
        Eigen::Vector3d accelxyz_in_body_frame;
        omega_imu << imu.angular_velocity.x , imu.angular_velocity.y, imu.angular_velocity.z;
        accelxyz_in_body_frame << imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z; //This is a column vector.

        nonLinearPropogation(omega_imu, accelxyz_in_body_frame);

    }

    void CameraIMUEKF::nonLinearPropogation(Eigen::Vector3d omega, Eigen::Vector3d acceleration) {

        Eigen::Vector3d accel_bias;
        Eigen::Vector3d gyro_bias;

        gyro_bias << xHat(9), xHat(10) , xHat(11);
        accel_bias << xHat(12), xHat(12) , xHat(14);

        omega = omega - gyro_bias;
        acceleration = acceleration - accel_bias;

        Eigen::Quaterniond q_Ik_I;
        Eigen::Quaterniond current_frame;
        Eigen::MatrixXd quaternion_integration(4,4);

        current_frame.w() = 1;
        current_frame.vec() << 0, 0, 0;

        quaternion_integration.topLeftCorner<3,3>() = -1 * reef_msgs::skew(omega);
        quaternion_integration.block<3,1>(0,3) = omega;
        quaternion_integration.block<1,3>(3,0) = -1 * omega;

        q_Ik_I = 0.5 * dt * quaternion_integration * current_frame;
        q_Ik_I.normalize();

        world_to_imu = q_Ik_I * world_to_imu;
        world_to_imu.normalize();


        xHat(0) = q_Ik_I.vec()(0);
        xHat(1) = q_Ik_I.vec()(1);
        xHat(2) = q_Ik_I.vec()(2);

        xHat.block<3,1>(6,0) = xHat.block<3,1>(6,0) + dt * world_to_imu.toRotationMatrix() * acceleration;

        xHat.block<3,1>(3,0) = xHat.block<3,1>(3,0) + xHat.block<3,1>(6,0) * dt;


        F.topLeftCorner<3,3>() = reef_msgs::skew(omega);
        F.block<3,3>(0,9) = -I;
        F.block<3,3>(3,12) = -I;

        F.block<3,3>(6,0) = -1 * world_to_imu.toRotationMatrix() * reef_msgs::skew(acceleration);
        F.block<3,3>(6,12)  = -1 * world_to_imu.toRotationMatrix();











    }




}
