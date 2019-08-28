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
        nh_private.param<int>("number_of_features", number_of_features, 16);

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

        K = Eigen::MatrixXd(21,2*number_of_features);
        K.setZero();

        z = Eigen::MatrixXd(2*number_of_features,1);

        H = Eigen::MatrixXd(2*number_of_features,21);

        xHat0 = Eigen::MatrixXd(23,1);
        xHat = Eigen::MatrixXd(23,1);
        Q = Eigen::MatrixXd(12,12);

        betaVector = Eigen::MatrixXd(23,1);

        R = Eigen::MatrixXd(number_of_features,number_of_features);

        I = Eigen::MatrixXd(3,3);
        I.setIdentity();

        reef_msgs::importMatrixFromParamServer(nh_private, xHat0, "xHat0");
        reef_msgs::importMatrixFromParamServer(nh_private, P0, "P0");
        reef_msgs::importMatrixFromParamServer(nh_private, Q, "Q");
        reef_msgs::importMatrixFromParamServer(nh_private, R, "R");
        reef_msgs::importMatrixFromParamServer(nh_private, betaVector, "beta");

        ROS_WARN_STREAM("XHat0 is \n" << xHat0);
        ROS_WARN_STREAM("P0 is \n" << P0);
        ROS_WARN_STREAM("Q is \n" << Q);
        ROS_WARN_STREAM("R is \n" << R);
        ROS_WARN_STREAM("Beta is \n" << betaVector);

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

    void CameraIMUEKF::sensorUpdate(ar_sys::ArucoCornerMsg aruco_corners) {

        if(aruco_corners.pixel_corners.size() != number_of_features/4)
            return;

        Eigen::MatrixXd expected_measurement(2 * number_of_features, 1);
        expected_measurement.setZero();
        z.setZero();
        H.setZero();

        fx = fy = 600;
        cx = 320;
        cy = 240;

        for(unsigned int i =0; i < aruco_corners.metric_corners.size(); i++){
            aruco_helper(aruco_corners.metric_corners[i].top_left, aruco_corners.pixel_corners[i].top_left, expected_measurement, i, TOP_LEFT);
            aruco_helper(aruco_corners.metric_corners[i].top_right, aruco_corners.pixel_corners[i].top_right, expected_measurement, i, TOP_RIGHT);
            aruco_helper(aruco_corners.metric_corners[i].bottom_right, aruco_corners.pixel_corners[i].bottom_right, expected_measurement, i, BOTTOM_RIGHT);
            aruco_helper(aruco_corners.metric_corners[i].bottom_left, aruco_corners.pixel_corners[i].bottom_left, expected_measurement, i, BOTTOM_LEFT);
        }
        nonLinearUpdate(expected_measurement);

    }

    void CameraIMUEKF::aruco_helper(ar_sys::SingleCorner metric_corner, ar_sys::SingleCorner pixel_corner,
                                    Eigen::MatrixXd &y_exp, unsigned int index, unsigned int position) {

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

        Eigen::Matrix3d q_block;
        Eigen::Matrix3d pos_block;
        Eigen::Matrix3d zero_block;
        Eigen::Matrix3d p_c_i_block;
        Eigen::Matrix3d alpha_block;

        pos_block = -1 * camera_to_imu_quad.toRotationMatrix().transpose() * world_to_imu_quat.toRotationMatrix().transpose();
        zero_block.setZero();
        p_c_i_block = -1 * camera_to_imu_quad.toRotationMatrix().transpose();

        Eigen::MatrixXd partial_y_measure_p_fc(2,3);
        Eigen::MatrixXd partial_x_measure(3,21);

        Eigen::Vector3d measured_metric(metric_corner.x, metric_corner.y, metric_corner.z);
        Eigen::Vector3d feature_metric_position_camera_frame;
        feature_metric_position_camera_frame = camera_to_imu_quad.toRotationMatrix().transpose() * ( world_to_imu_quat.toRotationMatrix().transpose() * (measured_metric -  world_to_imu_position) - camera_to_imu_position);

        Eigen::Vector2d feature_pixel_position_camera_frame;

        feature_pixel_position_camera_frame << fx * (feature_metric_position_camera_frame(0)/feature_metric_position_camera_frame(2)) + cx,
                                                fy * (feature_metric_position_camera_frame(1)/feature_metric_position_camera_frame(2)) + cy;
        y_exp.block<2,1>(8*index + position, 0) = feature_pixel_position_camera_frame;
        z.block<2,1>(8*index + position, 0) << pixel_corner.x, pixel_corner.y;

        q_block = camera_to_imu_quad.toRotationMatrix().transpose() * reef_msgs::skew( world_to_imu_quat.toRotationMatrix().transpose() * (measured_metric - world_to_imu_position) );
        alpha_block = reef_msgs::skew( camera_to_imu_quad.toRotationMatrix().transpose() * ( world_to_imu_quat.toRotationMatrix().transpose() * (measured_metric - world_to_imu_position) - camera_to_imu_position) );

        partial_y_measure_p_fc << fx, 0, -fx * feature_metric_position_camera_frame(0)/feature_metric_position_camera_frame(2),
                0,fy, -fy * feature_metric_position_camera_frame(1)/feature_metric_position_camera_frame(2);
        partial_y_measure_p_fc = (1/feature_metric_position_camera_frame(2)) * partial_y_measure_p_fc;
        partial_x_measure << q_block, pos_block, zero_block, zero_block, zero_block, p_c_i_block, alpha_block;
        H.block<2,21>(8*index + position,0)  = partial_y_measure_p_fc * partial_x_measure;



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
    }

    void CameraIMUEKF::nonLinearUpdate(Eigen::MatrixXd y_exp) {

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

        K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
        Eigen::MatrixXd correction(21,1);
        correction = K * (z- y_exp);

        Eigen::Quaterniond quat_error;
        quat_error.vec() = 0.5 * correction.block<1,3>(0,0);
        quat_error.w() = 1;
        world_to_imu_quat = quat_error * world_to_imu_quat;
        world_to_imu_quat.normalize();
        xHat.block<4,1>(0,0) =  world_to_imu_quat.coeffs();

        quat_error.vec() = 0.5 * correction.block<1,3>(18,0);
        quat_error.w() = 1;
        camera_to_imu_quad = quat_error * camera_to_imu_quad;
        camera_to_imu_quad.normalize();
        xHat.block<4,1>(19,0) =  camera_to_imu_quad.coeffs();

        xHat.block<15,1>(4,0) = xHat.block<15,1>(4,0) + correction.block<15,1>(3,0);

        P = ( Eigen::MatrixXd::Identity(21,21) - K * H ) * P;

    }

}
