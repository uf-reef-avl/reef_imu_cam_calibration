//
// Created by prashant on 7/16/19.
//

#include "camera_imu_calib/camera_imu_ekf.h"

namespace calibration{

    CameraIMUEKF::CameraIMUEKF() :
    nh_private("~"),
    nh_(""),
    initialize(false),
    got_measurement(false),
    got_camera_parameters(false),
    initialized_pnp(false),
    accel_calibrated(false),
    accInitSampleCount(0),
    cornerSampleCount(false),
    fx(0),fy(0),cx(0),cy(0)
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
        z.setZero();

        expected_measurement = Eigen::MatrixXd(2 * number_of_features, 1);
        expected_measurement.setZero();

        H = Eigen::MatrixXd(2*number_of_features,21);

        xHat0 = Eigen::MatrixXd(23,1);
        xHat = Eigen::MatrixXd(23,1);
        Q = Eigen::MatrixXd(12,12);

        betaVector = Eigen::MatrixXd(23,1);

        R = Eigen::MatrixXd(2 * number_of_features, 2 * number_of_features);

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
        state_publisher_ = nh_.advertise<camera_imu_calib::IMUCalibration>("imu_calib_result", 1, true);

        pnp_average_translation.setZero();
        pnp_average_euler.setZero();
//        pnp_quaternion_stack = Eigen::MatrixXd(4,CORNER_SAMPLE_SIZE);
//        pnp_quaternion_stack.setZero();

    }

    CameraIMUEKF::~CameraIMUEKF() {}

    void CameraIMUEKF::initializePNP(ar_sys::ArucoCornerMsg aruco_corners) {


        int num_of_markers = aruco_corners.pixel_corners.size();

        std::vector< cv::Point3f > objPnts;
        objPnts.reserve(4 * num_of_markers);
        std::vector< cv::Point2f > imgPnts;
        imgPnts.reserve(4 * num_of_markers);

        for(int i = 0; i <num_of_markers; i++)
        {
            int index = i;
            objPnts.push_back(cv::Point3f(aruco_corners.metric_corners[index].top_left.x, aruco_corners.metric_corners[index].top_left.y, aruco_corners.metric_corners[index].top_left.z ));
            imgPnts.push_back(cv::Point2f(aruco_corners.pixel_corners[index].top_left.x, aruco_corners.pixel_corners[index].top_left.y));

            objPnts.push_back(cv::Point3f(aruco_corners.metric_corners[index].top_right.x, aruco_corners.metric_corners[index].top_right.y, aruco_corners.metric_corners[index].top_right.z ));
            imgPnts.push_back(cv::Point2f(aruco_corners.pixel_corners[index].top_right.x, aruco_corners.pixel_corners[index].top_right.y));

            objPnts.push_back(cv::Point3f(aruco_corners.metric_corners[index].bottom_right.x, aruco_corners.metric_corners[index].bottom_right.y, aruco_corners.metric_corners[index].bottom_right.z ));
            imgPnts.push_back(cv::Point2f(aruco_corners.pixel_corners[index].bottom_right.x, aruco_corners.pixel_corners[index].bottom_right.y));

            objPnts.push_back(cv::Point3f(aruco_corners.metric_corners[index].bottom_left.x, aruco_corners.metric_corners[index].bottom_left.y, aruco_corners.metric_corners[index].bottom_left.z ));
            imgPnts.push_back(cv::Point2f(aruco_corners.pixel_corners[index].bottom_left.x, aruco_corners.pixel_corners[index].bottom_left.y));
        }

        cv::Mat objPoints, imgPoints;
        cv::Mat(objPnts).copyTo(objPoints);
        cv::Mat(imgPnts).copyTo(imgPoints);

        if (objPoints.total() == 0)
            return;

        cv::Vec3d tvec(0, 0, 1);
        cv::Vec3d rvec(0, 0, 0);
        cv::Mat guessRotMat = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
        cv::Rodrigues(guessRotMat, rvec);

        cv::solvePnP(objPoints, imgPoints, cameraMatrix, distortionCoeffs, rvec, tvec, true);

        cv::Mat rotMat;
        cv::Rodrigues(rvec, rotMat);

        if (tvec[2] < 0) {
            std::cout << "cv::solvePnP converged to invalid transform translation z = " << tvec[2] <<
                      " when, in reality we must assert, z > 0." << std::endl;
            return;
        }

        Eigen::Matrix3d rot_mat;
        rot_mat << rotMat.at<double>(0, 0), rotMat.at<double>(0, 1), rotMat.at<double>(0, 2),
                rotMat.at<double>(1, 0), rotMat.at<double>(1, 1), rotMat.at<double>(1, 2),
                rotMat.at<double>(2, 0), rotMat.at<double>(2, 1), rotMat.at<double>(2, 2);

        Eigen::Vector3d pnp_trans(tvec[0], tvec[1], tvec[2]);
        ROS_WARN_STREAM("PNP DCM is \n " << rot_mat.transpose());
        ROS_WARN_STREAM("PNP Translation is \n " << -rot_mat * pnp_trans);

        Eigen::Vector3d rpy;
        reef_msgs::roll_pitch_yaw_from_rotation321(rot_mat.transpose(), rpy); //This is a C Matrix now!

        pnp_average_euler +=rpy;
        pnp_average_translation.x() += tvec[0];
        pnp_average_translation.y() += tvec[1];
        pnp_average_translation.z() += tvec[2];
        cornerSampleCount++;
        if(cornerSampleCount == CORNER_SAMPLE_SIZE){

            pnp_average_euler.x() = pnp_average_euler.x()/CORNER_SAMPLE_SIZE;
            pnp_average_euler.y() = pnp_average_euler.y()/CORNER_SAMPLE_SIZE;
            pnp_average_euler.z() = pnp_average_euler.z()/CORNER_SAMPLE_SIZE;

            pnp_average_translation.x() = pnp_average_translation.x()/CORNER_SAMPLE_SIZE;
            pnp_average_translation.y() = pnp_average_translation.y()/CORNER_SAMPLE_SIZE;
            pnp_average_translation.z() = pnp_average_translation.z()/CORNER_SAMPLE_SIZE;

            ROS_WARN_STREAM("Average Rotation is  \n" <<pnp_average_euler);
            ROS_WARN_STREAM("Average Translation is  \n" <<pnp_average_translation);


            Eigen::Quaterniond imu_to_camera_quat;
            imu_to_camera_quat.vec() << xHat(19), xHat(20), xHat(21);
            imu_to_camera_quat.w() = xHat(22);

            Eigen::Vector3d imu_to_camera_position(xHat(16), xHat(17), xHat(18));

            Eigen::Matrix3d C_average_world_to_camera;
            C_average_world_to_camera = reef_msgs::DCM_from_Euler321(pnp_average_euler);
            Eigen::Matrix3d C_imu_to_camera = imu_to_camera_quat.toRotationMatrix().transpose();

            Eigen::Matrix3d C_world_to_imu = (C_imu_to_camera.transpose() * C_average_world_to_camera);
            Eigen::Vector3d world_to_imu_pose = -C_world_to_imu.transpose() * (imu_to_camera_position + C_imu_to_camera.transpose() * pnp_average_translation);

            Eigen::Quaterniond world_to_imu_quat(C_world_to_imu);
            initialized_pnp = true;

            xHat.block<3,1>(0,0) << world_to_imu_quat.vec();
            xHat(3,0) = world_to_imu_quat.w();
            xHat.block<3,1>(4,0) << world_to_imu_pose;
            ROS_WARN_STREAM("XHat post initialization is  \n" <<xHat);
        }
    }

    void CameraIMUEKF::getCameraInfo(const sensor_msgs::CameraInfo &msg){

        fx = msg.K[0];
        fy = msg.K[4];
        cx = msg.K[2];
        cy = msg.K[5];
        getCamParams(msg);
        got_camera_parameters = true;
    }

    void CameraIMUEKF::getCamParams(const sensor_msgs::CameraInfo &cam_info) {
        cameraMatrix = cv::Mat::zeros(3, 3, CV_32FC1);
        distortionCoeffs = cv::Mat::zeros(5, 1, CV_32FC1);

        for (int i = 0; i < 9; ++i) {
            cameraMatrix.at<float>(i / 3, i % 3) = cam_info.K[i];
        }
        for (int i = 0; i < 5; ++i) {
            distortionCoeffs.at<float>(i, 0) = cam_info.D[i];
        }
    }

    void CameraIMUEKF::initializeAcc(geometry_msgs::Vector3 acc) {

        accSampleAverage.x += acc.x;
        accSampleAverage.y += acc.y;
        accSampleAverage.z += acc.z;
        accInitSampleCount++;

        if (accInitSampleCount == ACC_SAMPLE_SIZE){

            accSampleAverage.x /= ACC_SAMPLE_SIZE;
            accSampleAverage.y /= ACC_SAMPLE_SIZE;
            accSampleAverage.z /= ACC_SAMPLE_SIZE;
            accel_calibrated = true;
        }

    }

    void CameraIMUEKF::sensorUpdate(sensor_msgs::Imu imu) {

        if (isnan(getVectorMagnitude(imu.linear_acceleration.x, imu.linear_acceleration.y,imu.linear_acceleration.z))){
            ROS_ERROR_STREAM("IMU is giving NaNs");
            return;
        }


        if(!accel_calibrated){
            // TODO: Initialize the xhat using PNP
            initializeAcc(imu.linear_acceleration);
            last_time_stamp = imu.header.stamp.toSec();
            return;
        }

        dt = imu.header.stamp.toSec() - last_time_stamp;
        last_time_stamp = imu.header.stamp.toSec();

        if(!initialized_pnp)
            return;

        state_msg.header = imu.header;

        ROS_WARN_STREAM("Average Accelerations is \n" <<accSampleAverage);

        ROS_WARN_STREAM("Measured Accelerations is \n" <<imu.linear_acceleration);

        imu.linear_acceleration.x = imu.linear_acceleration.x - accSampleAverage.x;
        imu.linear_acceleration.y = imu.linear_acceleration.y - accSampleAverage.y;
        imu.linear_acceleration.z = imu.linear_acceleration.z - accSampleAverage.z;

        Eigen::Vector3d omega_imu;
        Eigen::Vector3d accelxyz_in_body_frame;
        omega_imu << imu.angular_velocity.x , imu.angular_velocity.y, imu.angular_velocity.z;
        accelxyz_in_body_frame << imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z;

        nonLinearPropagation(omega_imu, accelxyz_in_body_frame);
        ROS_WARN_STREAM("Xhat in propogartion is \n " << xHat);

        if(got_measurement){
            nonLinearUpdate();
            ROS_WARN_STREAM("Xhat after Update is \n " << xHat);
            got_measurement = false;
        }
        publish_state();

    }

    void CameraIMUEKF::sensorUpdate(ar_sys::ArucoCornerMsg aruco_corners) {

        ROS_WARN_STREAM("Number of markers detected \t" << aruco_corners.pixel_corners.size());

        if(aruco_corners.pixel_corners.size() != number_of_features/4)
            return;

        if(!initialized_pnp){
            initializePNP(aruco_corners);
            return;
        }
        z.setZero();
        H.setZero();
        expected_measurement.setZero();
        for(unsigned int i =0; i < aruco_corners.metric_corners.size(); i++){
            aruco_helper(aruco_corners.metric_corners[i].top_left, aruco_corners.pixel_corners[i].top_left, i, TOP_LEFT);
            aruco_helper(aruco_corners.metric_corners[i].top_right, aruco_corners.pixel_corners[i].top_right, i, TOP_RIGHT);
            aruco_helper(aruco_corners.metric_corners[i].bottom_right, aruco_corners.pixel_corners[i].bottom_right, i, BOTTOM_RIGHT);
            aruco_helper(aruco_corners.metric_corners[i].bottom_left, aruco_corners.pixel_corners[i].bottom_left, i, BOTTOM_LEFT);
        }
//        ROS_WARN_STREAM("Expected Measurement is \n" << expected_measurement);
        ROS_WARN_STREAM("Expected Measurement is \n" << expected_measurement);
        ROS_WARN_STREAM("Measurement is \n" << z);
        got_measurement = true;
    }

    void CameraIMUEKF::aruco_helper(ar_sys::SingleCorner metric_corner, ar_sys::SingleCorner pixel_corner,
                                     unsigned int index, unsigned int position) {

        ROS_WARN_STREAM("Xhat is \n" << xHat);

        Eigen::Quaterniond world_to_imu_quat;
        world_to_imu_quat.vec() << xHat(0), xHat(1), xHat(2);
        world_to_imu_quat.w() = xHat(3);

        Eigen::Vector3d world_to_imu_position(xHat(4), xHat(5), xHat(6));
        Eigen::Vector3d velocity_W(xHat(7), xHat(8), xHat(9));
        Eigen::Vector3d gyro_bias(xHat(10), xHat(11) , xHat(12));
        Eigen::Vector3d accel_bias(xHat(13), xHat(14) , xHat(15));
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

        ROS_WARN_STREAM("Pos block is \n" << pos_block);
        ROS_WARN_STREAM("zero_block is \n" << zero_block);
        ROS_WARN_STREAM("p_c_i_block is \n" << p_c_i_block);

        Eigen::MatrixXd partial_y_measure_p_fc(2,3);
        Eigen::MatrixXd partial_x_measure(3,21);

        Eigen::Vector3d measured_metric(metric_corner.x, metric_corner.y, metric_corner.z);
        Eigen::Vector3d feature_metric_position_camera_frame;
        feature_metric_position_camera_frame = camera_to_imu_quad.toRotationMatrix().transpose() * ( world_to_imu_quat.toRotationMatrix().transpose() * (measured_metric -  world_to_imu_position) - camera_to_imu_position);

        ROS_WARN_STREAM("Metric feature is \n" << measured_metric);
        ROS_WARN_STREAM("DCM is \n" << camera_to_imu_quad.toRotationMatrix().transpose());
        ROS_WARN_STREAM("Metric feature in camera frame is \n" << feature_metric_position_camera_frame);
        Eigen::Vector2d feature_pixel_position_camera_frame;

        feature_pixel_position_camera_frame << fx * (feature_metric_position_camera_frame(0)/feature_metric_position_camera_frame(2)) + cx,
                                                fy * (feature_metric_position_camera_frame(1)/feature_metric_position_camera_frame(2)) + cy;

        ROS_WARN_STREAM("Pixel feature in camera frame is \n" << feature_pixel_position_camera_frame);

        expected_measurement.block<2,1>(8*index + position, 0) = feature_pixel_position_camera_frame;
        z.block<2,1>(8*index + position, 0) << pixel_corner.x, pixel_corner.y;

        q_block = camera_to_imu_quad.toRotationMatrix().transpose() * reef_msgs::skew( world_to_imu_quat.toRotationMatrix().transpose() * (measured_metric - world_to_imu_position) );
        alpha_block = reef_msgs::skew( camera_to_imu_quad.toRotationMatrix().transpose() * ( world_to_imu_quat.toRotationMatrix().transpose() * (measured_metric - world_to_imu_position) - camera_to_imu_position) );

        ROS_WARN_STREAM("q block is \n" << q_block);
        ROS_WARN_STREAM("alpha block is \n" << alpha_block);

        partial_y_measure_p_fc << fx, 0, -fx * feature_metric_position_camera_frame(0)/feature_metric_position_camera_frame(2),
                0,fy, -fy * feature_metric_position_camera_frame(1)/feature_metric_position_camera_frame(2);
        partial_y_measure_p_fc = (1/feature_metric_position_camera_frame(2)) * partial_y_measure_p_fc;
        partial_x_measure << q_block, pos_block, zero_block, zero_block, zero_block, p_c_i_block, alpha_block;

        ROS_WARN_STREAM("partial_y_measure_p_fc is \n" << partial_y_measure_p_fc);
        ROS_WARN_STREAM("partial_x_measure is \n" << alpha_block);
        ROS_WARN_STREAM("H is \n" << partial_y_measure_p_fc * partial_x_measure);

        H.block<2,21>(8*index + position,0)  = partial_y_measure_p_fc * partial_x_measure;
    }

    void CameraIMUEKF::nonLinearPropagation(Eigen::Vector3d omega, Eigen::Vector3d acceleration) {

        // Break state into variables to make it easier to read (and write) code
        Eigen::Quaterniond world_to_imu_quat;
        world_to_imu_quat.vec() << xHat(0), xHat(1), xHat(2);
        world_to_imu_quat.w() = xHat(3);

        Eigen::Vector3d world_to_imu_position(xHat(4), xHat(5), xHat(6));
        Eigen::Vector3d velocity_W(xHat(7), xHat(8), xHat(9));
        Eigen::Vector3d gyro_bias(xHat(10), xHat(11) , xHat(12));
        Eigen::Vector3d accel_bias(xHat(13), xHat(14) , xHat(15));
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


        world_to_imu_quat = (Eigen::Matrix4d::Identity() + 0.5 * dt * Omega_matrix) * world_to_imu_quat.coeffs() ;
        world_to_imu_quat.normalize();

        xHat.block<19,1>(4,0) = xHat.block<19,1>(4,0) + dt * true_dynamics;
        xHat.block<4,1>(0,0) =  world_to_imu_quat.coeffs();

        P = P + (F * P + P * F.transpose() + G * Q * G.transpose()) * dt;
    }

    void CameraIMUEKF::nonLinearUpdate() {

        // Break state into variables to make it easier to read (and write) code
        Eigen::Quaterniond world_to_imu_quat;
        world_to_imu_quat.vec() << xHat(0), xHat(1), xHat(2);
        world_to_imu_quat.w() = xHat(3);

        Eigen::Vector3d world_to_imu_position(xHat(4), xHat(5), xHat(6));
        Eigen::Vector3d velocity_W(xHat(7), xHat(8), xHat(9));
        Eigen::Vector3d gyro_bias(xHat(10), xHat(11) , xHat(12));
        Eigen::Vector3d accel_bias(xHat(13), xHat(14) , xHat(15));
        Eigen::Vector3d camera_to_imu_position(xHat(16), xHat(17), xHat(18));

        Eigen::Quaterniond camera_to_imu_quad;
        camera_to_imu_quad.vec() << xHat(19), xHat(20), xHat(21);
        camera_to_imu_quad.w() = xHat(22);

        K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
        Eigen::MatrixXd correction(21,1);
        correction = K * (z- expected_measurement);

        Eigen::Quaterniond quat_error;
        quat_error.vec() = 0.5 * correction.block<3,1>(0,0);
        quat_error.w() = 1;
        world_to_imu_quat = world_to_imu_quat * quat_error;
        world_to_imu_quat.normalize();
        xHat.block<4,1>(0,0) =  world_to_imu_quat.coeffs();

        quat_error.vec() = 0.5 * correction.block<3,1>(18,0);
        quat_error.w() = 1;
        camera_to_imu_quad = camera_to_imu_quad * quat_error;
        camera_to_imu_quad.normalize();
        xHat.block<4,1>(19,0) =  camera_to_imu_quad.coeffs();

        xHat.block<15,1>(4,0) = xHat.block<15,1>(4,0) + correction.block<15,1>(3,0);

        P = ( Eigen::MatrixXd::Identity(21,21) - K * H ) * P;


    }

    void CameraIMUEKF::publish_state() {
        state_msg.world_to_imu.orientation.x = xHat(0);
        state_msg.world_to_imu.orientation.y = xHat(1);
        state_msg.world_to_imu.orientation.z = xHat(2);
        state_msg.world_to_imu.orientation.w = xHat(3);

        state_msg.world_to_imu.position.x = xHat(4);
        state_msg.world_to_imu.position.y = xHat(5);
        state_msg.world_to_imu.position.z = xHat(6);

        state_msg.velocity.x = xHat(7);
        state_msg.velocity.y = xHat(8);
        state_msg.velocity.z = xHat(9);

        state_msg.gyro_bias.x = xHat(10);
        state_msg.gyro_bias.y = xHat(11);
        state_msg.gyro_bias.z = xHat(12);

        state_msg.accel_bias.x = xHat(13);
        state_msg.accel_bias.y = xHat(14);
        state_msg.accel_bias.z = xHat(15);

        state_msg.imu_to_camera.position.x = xHat(16);
        state_msg.imu_to_camera.position.y = xHat(17);
        state_msg.imu_to_camera.position.z = xHat(18);

        state_msg.imu_to_camera.orientation.x = xHat(19);
        state_msg.imu_to_camera.orientation.y = xHat(20);
        state_msg.imu_to_camera.orientation.z = xHat(21);
        state_msg.imu_to_camera.orientation.w = xHat(22);

        for(unsigned int i =0; i<20; i++){
            state_msg.P[i] = P(i,i);
            state_msg.sigma_minus[i] = xHat(i) - 3 * sqrt(P(i,i));
            state_msg.sigma_plus[i] = xHat(i) + 3 * sqrt(P(i,i));
        }

        state_publisher_.publish(state_msg);

    }

    double getVectorMagnitude(double x, double y, double z)
    {
        return sqrt(x * x + y * y + z * z);
    }
}
