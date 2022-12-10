//
// Created by prashant on 7/16/19.
//

#include "camera_imu_calib/camera_imu_ekf.h"

class IMUCalibration;
namespace calibration{

    CameraIMUEKF::CameraIMUEKF() :
            nh_private("~"),
            nh_(""),
            initialize(false),
            got_measurement(false),
            got_camera_parameters(false),
            initialized_pnp(false),
            initialized_current_pixels(false),
            accel_calibrated(false),
            accInitSampleCount(0),
            cornerSampleCount(false),
            fx(0),fy(0),cx(0),cy(0),
            number_of_features(0),
            publish_full_quaternion(false),
            publish_expected_meas_(false),
            enable_partial_update_(true),
            initialized_timer(false)

    {
        nh_private.param<double>("estimator_dt", dt, 0.005);
        nh_private.param<int>("number_of_features", number_of_features, 15);
        nh_private.param<bool>("publish_full_quaternion", publish_full_quaternion, true);
        nh_private.param<bool>("publish_expected_meas", publish_expected_meas_, true);
        nh_private.param<bool>("enable_partial_update", enable_partial_update_, true);
        nh_private.param<bool>("enable_dynamic_update", enable_dynamic_update, true);
        nh_private.param<double>("mahalanobis_param", mahalanobis_param, 100);
        nh_private.param<double>("pixel_difference_threshold", pixel_difference_threshold, 10);
        nh_private.param<bool>("use_mocap_to_initialize", use_mocap_to_initialize, false);

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

        K = Eigen::MatrixXd(21,1);
        K.setZero();

        z = Eigen::MatrixXd(1,1);
        z.setZero();

        expected_measurement = Eigen::MatrixXd(1, 1);
        expected_measurement.setZero();

        H = Eigen::MatrixXd(1,21);

        xHat0 = Eigen::MatrixXd(23,1);
        xHat = Eigen::MatrixXd(23,1);
        Q = Eigen::MatrixXd(12,12);

        betaVector = Eigen::MatrixXd(21,1);

        R = Eigen::MatrixXd(1, 1);

        I = Eigen::MatrixXd(3,3);
        I.setIdentity();

        R_std = Eigen::MatrixXd(1,1);

        reef_msgs::importMatrixFromParamServer(nh_private, xHat0, "xHat0");
        reef_msgs::importMatrixFromParamServer(nh_private, P0, "P0");
        reef_msgs::importMatrixFromParamServer(nh_private, Q, "Q");
        reef_msgs::importMatrixFromParamServer(nh_private, R_std, "R_std");
        reef_msgs::importMatrixFromParamServer(nh_private, betaVector, "beta");

//        Q = Q*0.01;

        double R_std_float; // Set to 1 as default.
        R_std_float = R_std(0,0);

        R = R_std_float*R.setIdentity();

        R = R*R;

        Estimator::initialize();
        ROS_WARN_STREAM("XHat0 is \n" << xHat);
        ROS_WARN_STREAM("P0 is \n" << P);
        ROS_WARN_STREAM("Q is \n" << Q);
        ROS_WARN_STREAM("R is \n" << R);
        ROS_WARN_STREAM("Beta is \n" << betaVector);

        //TODO Make new message or change the message type.
        state_publisher_ = nh_.advertise<camera_imu_calib::IMUCalibration>("imu_calib_result", 1, true);
        expect_pixel_publisher_ = nh_.advertise<camera_imu_calib::ExpectedMeasurement>("expected_measurements", 1, true);

        pnp_average_translation.setZero();
        pnp_average_euler.setZero();
        previous_charuco_roll = Eigen::Vector3d(0,0,0);
        FIRST_IMU_MEASUREMENT = true;
    }

    CameraIMUEKF::~CameraIMUEKF() {}

    void CameraIMUEKF::initializePNP(charuco_ros::CharucoCornerMsg aruco_corners) {

        int num_of_markers = aruco_corners.pixel_corners.size();

        std::vector< cv::Point3f > objPnts;
        objPnts.reserve(num_of_markers);
        std::vector< cv::Point2f > imgPnts;
        imgPnts.reserve(num_of_markers);

        for(int i = 0; i <num_of_markers; i++)
        {
            objPnts.push_back(cv::Point3f(aruco_corners.metric_corners[i].corner.x, aruco_corners.metric_corners[i].corner.y, aruco_corners.metric_corners[i].corner.z ));
            imgPnts.push_back(cv::Point2f(aruco_corners.pixel_corners[i].corner.x, aruco_corners.pixel_corners[i].corner.y));
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

        Eigen::Matrix3d C_rot_mat;
        C_rot_mat << rotMat.at<double>(0, 0), rotMat.at<double>(0, 1), rotMat.at<double>(0, 2),
                rotMat.at<double>(1, 0), rotMat.at<double>(1, 1), rotMat.at<double>(1, 2),
                rotMat.at<double>(2, 0), rotMat.at<double>(2, 1), rotMat.at<double>(2, 2);
        // ^^ This can be interpreted as the rotation matrix from the Camera to the board or the DCM of the board to the camera frame

//        Eigen::Quaterniond our_orientation_quat = reef::msgs

        Eigen::Vector3d rpy;
        reef_msgs::roll_pitch_yaw_from_rotation321(C_rot_mat, rpy); //This is a C Matrix now!

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
            imu_to_camera_quat.vec() << xHat(Q_IX), xHat(Q_IY), xHat(Q_IZ);
            imu_to_camera_quat.w() = xHat(Q_IW);

            Eigen::Vector3d imu_to_camera_position(xHat(P_IX), xHat(P_IY), xHat(P_IZ));

            Eigen::Matrix3d C_average_world_to_camera;
            C_average_world_to_camera = reef_msgs::DCM_from_Euler321(pnp_average_euler);
            Eigen::Matrix3d C_imu_to_camera = imu_to_camera_quat.toRotationMatrix().transpose();

            Eigen::Matrix3d C_world_to_imu = (C_imu_to_camera.transpose() * C_average_world_to_camera);
            Eigen::Vector3d world_to_imu_pose = -C_world_to_imu.transpose() * (imu_to_camera_position + C_imu_to_camera.transpose() * pnp_average_translation);

            Eigen::Quaterniond world_to_imu_quat(C_world_to_imu.transpose());
            initialized_pnp = true;

            if(use_mocap_to_initialize){
                world_to_imu_quat = initial_imu_q;
                world_to_imu_pose = initial_imu_position;
            }
            xHat.block<3,1>(QX,0) = world_to_imu_quat.vec();
            xHat(QW,0) = world_to_imu_quat.w();
            xHat.block<3,1>(PX,0) = world_to_imu_pose;
            ROS_WARN_STREAM("PnP XHat post initialization is  \n" <<xHat);
        }

    }

    Eigen::Vector3d CameraIMUEKF::computePNP(charuco_ros::CharucoCornerMsg aruco_corners) {
        Eigen::Vector3d rpy;
        int num_of_markers = aruco_corners.pixel_corners.size();

        std::vector<cv::Point3f> objPnts;
        objPnts.reserve(num_of_markers);
        std::vector<cv::Point2f> imgPnts;
        imgPnts.reserve(num_of_markers);

        for (int i = 0; i < num_of_markers; i++) {
            objPnts.push_back(
                    cv::Point3f(aruco_corners.metric_corners[i].corner.x, aruco_corners.metric_corners[i].corner.y,
                                aruco_corners.metric_corners[i].corner.z));
            imgPnts.push_back(
                    cv::Point2f(aruco_corners.pixel_corners[i].corner.x, aruco_corners.pixel_corners[i].corner.y));
        }

        cv::Mat objPoints, imgPoints;
        cv::Mat(objPnts).copyTo(objPoints);
        cv::Mat(imgPnts).copyTo(imgPoints);

        if (objPoints.total() == 0){
             rpy<<0,0,0;
            return rpy;
        }


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
            rpy<<0,0,0;
            return rpy;
        }

        Eigen::Matrix3d C_rot_mat;
        C_rot_mat << rotMat.at<double>(0, 0), rotMat.at<double>(0, 1), rotMat.at<double>(0, 2),
                rotMat.at<double>(1, 0), rotMat.at<double>(1, 1), rotMat.at<double>(1, 2),
                rotMat.at<double>(2, 0), rotMat.at<double>(2, 1), rotMat.at<double>(2, 2);
        // ^^ This can be interpreted as the rotation matrix from the Camera to the board or the DCM of the board to the camera frame

//        Eigen::Quaterniond our_orientation_quat = reef::msgs


        reef_msgs::roll_pitch_yaw_from_rotation321(C_rot_mat, rpy);
        return rpy;
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

    void CameraIMUEKF::initializeAcc(geometry_msgs::Vector3 acc, geometry_msgs::Vector3 gyro) {

        accSampleAverage.x += acc.x;
        accSampleAverage.y += acc.y;
        accSampleAverage.z += acc.z;

        gyroSampleAverage.x += gyro.x;
        gyroSampleAverage.y += gyro.y;
        gyroSampleAverage.z += gyro.z;

        accInitSampleCount++;

        if (accInitSampleCount == ACC_SAMPLE_SIZE){

            gravity<<0,0,9.80665;

            accSampleAverage.x /= ACC_SAMPLE_SIZE;
            accSampleAverage.y /= ACC_SAMPLE_SIZE;
            accSampleAverage.z /= ACC_SAMPLE_SIZE;

            xHat(BAX, 0) = accSampleAverage.x;
            xHat(BAY, 0) = accSampleAverage.y;
            xHat(BAZ, 0) = accSampleAverage.z + gravity(2);

            gyroSampleAverage.x /= ACC_SAMPLE_SIZE;
            gyroSampleAverage.y /= ACC_SAMPLE_SIZE;
            gyroSampleAverage.z /= ACC_SAMPLE_SIZE;

            accel_calibrated = true;


            xHat(BWX, 0) = gyroSampleAverage.x;
            xHat(BWY, 0) = gyroSampleAverage.y;
            xHat(BWZ, 0) = gyroSampleAverage.z;


            ROS_WARN_STREAM("XHat post IMU initialization is  \n" <<xHat);
            ROS_WARN_STREAM("Gravity is  \n" <<gravity);
            ROS_WARN_STREAM("Accel components are  \n" <<accSampleAverage);
        }

    }

    void CameraIMUEKF::sensorUpdate(sensor_msgs::Imu imu) {

        if (std::isnan(getVectorMagnitude(imu.linear_acceleration.x, imu.linear_acceleration.y,imu.linear_acceleration.z))){
            ROS_ERROR_STREAM("IMU is giving NaNs");
            return;
        }


        if(!accel_calibrated){
            initializeAcc(imu.linear_acceleration, imu.angular_velocity);
            last_time_stamp = imu.header.stamp.toSec();
            return;
        }

        if(!initialized_pnp)
            return;

        if(!initialized_timer)
        {
            initial_time =  ros::Time::now();
            initialized_timer = true;
        }

        dt = imu.header.stamp.toSec() - last_time_stamp;
        last_time_stamp = imu.header.stamp.toSec();
        state_msg.header = imu.header;


        Eigen::Vector3d omega_imu;
        Eigen::Vector3d accelxyz_in_body_frame;
        omega_imu << imu.angular_velocity.x , imu.angular_velocity.y, imu.angular_velocity.z;
        accelxyz_in_body_frame << imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z;

        nonLinearPropagation(omega_imu, accelxyz_in_body_frame);

        if(got_measurement){
            if(enable_dynamic_update){
                //Vary beta as a function of time.
                //Compute time elapsed
                ros::Time currtime=ros::Time::now();
                ros::Duration diff=currtime-initial_time;

                if(diff.toSec() >= 80.0 && diff.toSec() < 100.0)
                {
                    betaVector.block<6,1>(P_IX-1,0) << 0.01,0.01,0.01,0.3,0.3,0.3;
//                    ROS_WARN_STREAM("Beta updated \n" << betaVector);
                }
                if(diff.toSec() >= 100.0 && diff.toSec() < 130.0)
                {
                    betaVector.block<6,1>(P_IX-1,0) << 0.025,0.025,0.025,0.6,0.6,0.6;
//                    ROS_WARN_STREAM("Beta updated \n" << betaVector);
                }

                if(diff.toSec() >= 130.0 )
                {
                    betaVector.block<6,1>(P_IX-1,0) << 0.05,0.05,0.05,1.0,1.0,1.0;
//                    ROS_WARN_STREAM("Beta updated \n" << betaVector);
                }

            }
            //Don't update if the measurement is an outlier
            if(chi2AcceptPixels()){
                nonLinearUpdateSequentially(charuco_measurement);
            }
            //Reset got measurement flag
            got_measurement = false;
        }
        publish_state();

    }

    void CameraIMUEKF::sensorUpdate(charuco_ros::CharucoCornerMsg aruco_corners) {

        if(!initialized_current_pixels){
            if(aruco_corners.pixel_corners.size() != number_of_features){
                return;}
            past_charuco_measurent = aruco_corners;
            initialized_current_pixels = true;
            return;
        }

        if(!initialized_pnp){
            if(aruco_corners.pixel_corners.size() != number_of_features){
                return;}
            initializePNP(aruco_corners);
            return;
        }
        charuco_measurement = aruco_corners;
        if(acceptCharucoMeasurement()) {
            got_measurement = true;
        }

    }

    void CameraIMUEKF::aruco_helper(charuco_ros::SingleCorner metric_corner, charuco_ros::SingleCorner pixel_corner,
                                    unsigned int index) {

       Eigen::Quaterniond q_W_to_I;
        q_W_to_I.vec() << xHat(QX), xHat(QY), xHat(QZ);
        q_W_to_I.w() = xHat(QW);

        Eigen::Vector3d p_I_in_W(xHat(PX), xHat(PY), xHat(PZ));
        Eigen::Vector3d velocity_W(xHat(U), xHat(V), xHat(W));
        Eigen::Vector3d gyro_bias(xHat(BWX), xHat(BWY) , xHat(BWZ));
        Eigen::Vector3d accel_bias(xHat(BAX), xHat(BAY) , xHat(BAZ));
        Eigen::Vector3d position_camera_in_imu_frame(xHat(P_IX), xHat(P_IY), xHat(P_IZ));

        Eigen::Quaterniond q_I_to_C;
        q_I_to_C.vec() << xHat(Q_IX), xHat(Q_IY), xHat(Q_IZ);
        q_I_to_C.w() = xHat(Q_IW);

        Eigen::Matrix3d q_block;
        Eigen::Matrix3d pos_block;
        Eigen::Matrix3d zero_block;
        Eigen::Matrix3d p_c_i_block;
        Eigen::Matrix3d alpha_block;



        pos_block = -1 * q_I_to_C.toRotationMatrix().transpose() * q_W_to_I.toRotationMatrix().transpose();
        zero_block.setZero();
        p_c_i_block = -1 * q_I_to_C.toRotationMatrix().transpose();

        Eigen::MatrixXd partial_y_measure_p_fc(2,3);
        Eigen::MatrixXd partial_x_measure(3,21);

        Eigen::Vector3d measured_metric(metric_corner.x, metric_corner.y, metric_corner.z);
        Eigen::Vector3d h_hat;
        h_hat = q_I_to_C.toRotationMatrix().transpose() * ( q_W_to_I.toRotationMatrix().transpose() * (measured_metric -  p_I_in_W) - position_camera_in_imu_frame);
        Eigen::Vector2d expected_feature_pixel_position_camera_frame;
//Camera parameters in Jacobian
        expected_feature_pixel_position_camera_frame << fx * (h_hat(0)/h_hat(2)) + cx,
                                                fy * (h_hat(1)/h_hat(2)) + cy;


        q_block = q_I_to_C.toRotationMatrix().transpose() * reef_msgs::skew( q_W_to_I.toRotationMatrix().transpose() * (measured_metric - p_I_in_W) );
        alpha_block =  reef_msgs::skew(q_I_to_C.toRotationMatrix().transpose() *q_W_to_I.toRotationMatrix().transpose()*measured_metric) -1* reef_msgs::skew ( q_I_to_C.toRotationMatrix().transpose()*q_W_to_I.toRotationMatrix().transpose() * p_I_in_W) -1*reef_msgs::skew(q_I_to_C.toRotationMatrix().transpose()*position_camera_in_imu_frame) ;
        partial_y_measure_p_fc << fx, 0,    -fx * h_hat(0)/h_hat(2),
                                  0,  fy,   -fy * h_hat(1)/h_hat(2);
        partial_y_measure_p_fc = (1./h_hat(2)) * partial_y_measure_p_fc;
        partial_x_measure << q_block, pos_block, zero_block, zero_block, zero_block, p_c_i_block, alpha_block;
        Eigen::MatrixXd tempH(2,21);
        tempH = partial_y_measure_p_fc * partial_x_measure;
        if ( index == 0 )
        {
            expected_measurement.block<1,1>(0, 0) << expected_feature_pixel_position_camera_frame(index);
            z.block<1,1>(0, 0) << pixel_corner.x;
            H.block<1,21>(0,0)  = tempH.block<1,21>(index,0);
        }
        else if(index == 1){
            expected_measurement.block<1,1>(0, 0) << expected_feature_pixel_position_camera_frame(index);
            z.block<1,1>(0, 0) << pixel_corner.y;
            H.block<1,21>(0,0)  = tempH.block<1,21>(index,0);
        }
    }

    void CameraIMUEKF::nonLinearPropagation(Eigen::Vector3d omega, Eigen::Vector3d acceleration) {

        gravity_in_board = reef_msgs::quaternion_to_rotation(initial_board_q)*gravity;
        //Based on the derivation, gravity must be interpreted as the value needed to cancel the accel's gravity out
        //expressed in the inertial frame. Since the accel reports ~-9.8 , here gravity will be ~+9.8 in the corresponding axis.
//
        // Break state into variables to make it easier to read (and write) code
        Eigen::Vector3d w_hat;
        Eigen::Vector3d s_hat;
        Eigen::Quaterniond q_W_to_I;
        q_W_to_I.vec() << xHat(QX), xHat(QY), xHat(QZ);
        q_W_to_I.w() = xHat(QW);

        Eigen::Vector3d world_to_imu_position(xHat(PX), xHat(PY), xHat(PZ));
        Eigen::Vector3d velocity_W(xHat(U), xHat(V), xHat(W));
        Eigen::Vector3d gyro_bias(xHat(BWX), xHat(BWY) , xHat(BWZ));
        Eigen::Vector3d accel_bias(xHat(BAX), xHat(BAY) , xHat(BAZ));
        Eigen::Vector3d position_camera_in_imu_frame(xHat(P_IX), xHat(P_IY), xHat(P_IZ));

        Eigen::Quaterniond imu_to_camera_quat;
        imu_to_camera_quat.vec() << xHat(Q_IX), xHat(Q_IY), xHat(Q_IZ);
        imu_to_camera_quat.w() = xHat(Q_IW);

        w_hat = omega - gyro_bias;
        s_hat = acceleration  - accel_bias;

        F.setZero();
        F.topLeftCorner<3,3>() = -reef_msgs::skew(w_hat);
        F.block<3,3>(0,9) = -I;
        F.block<3,3>(3,6) = I;
        F.block<3,3>(6,0) = -1.0 * (q_W_to_I.toRotationMatrix() * reef_msgs::skew(s_hat));
        F.block<3,3>(6,12)  = -1.0 * q_W_to_I.toRotationMatrix();

        G.setZero();
        G.topLeftCorner<3,3>() = -I;
        G.block<3,3>(6,3) = -1 * (q_W_to_I.toRotationMatrix());
        G.block<3,3>(9,6) = I;
        G.block<3,3>(12,9) = I;


//=========================Euler integration
        //Propagate velocity of IMU wrt world
//        xHat.block<3,1>(U,0) = xHat.block<3,1>(U,0) + (q_W_to_I.toRotationMatrix() * s_hat + gravity_in_board)*dt;
//        //Propagate position of IMU wrt world
//        xHat.block<3,1>(PX,0) =  xHat.block<3,1>(PX,0) + velocity_W * dt;
//        //Propagate the rest of the states
//        xHat.block<11,1>(BWX,0) = xHat.block<11,1>(BWX,0);
//
//        //Propagate attitude original block
//        Eigen::Matrix4d Omega_matrix;
//        Omega_matrix.setZero();
//        Omega_matrix.topLeftCorner<3,3>() = -1*(reef_msgs::skew(w_hat));
//        Omega_matrix.block<3,1>(0,3) = w_hat;
//        Omega_matrix.block<1,3>(3,0) = -w_hat.transpose();
//        q_W_to_I = (Eigen::Matrix4d::Identity() + 0.5 * dt * Omega_matrix) * q_W_to_I.coeffs() ;
//        xHat.block<4,1>(0,0) =  q_W_to_I.coeffs();
//======================================================================================================================

        if(FIRST_IMU_MEASUREMENT){
            //This block initializes the previous values for the gyro and accel measurements.
            FIRST_IMU_MEASUREMENT = false;
            w_k0 = omega;
            s_k0 = acceleration;
            return;
        }


        int steps = 20;
        RK45integrate(omega,acceleration,steps);
        //Change this propagation. Do UDU now.
        P = P + (F * P + P * F.transpose() + G * Q * G.transpose()) * dt;
        w_k0 = omega;
        s_k0 = acceleration;
    }

    void CameraIMUEKF::RK45integrate(Eigen::Vector3d w_hat, Eigen::Vector3d s_hat, double steps){
        // Following equations from http://maths.cnam.fr/IMG/pdf/RungeKuttaFehlbergProof.pdf
        //step size is assumed to be dt
        double h = dt/steps;
        Eigen::MatrixXd ic = Eigen::MatrixXd::Zero(23,1);
        ic.block<4,1>(QX,0) << 0,0,0,1.;
        Eigen::Quaterniond q_Ik_I;
        Eigen::Quaterniond q_W_to_I;
        Eigen::MatrixXd k1,k2,k3,k4;
        for (int i = 1; i <=steps ; i++) {
            //Integrate dynamics
            k1 = integration_function(ic, 0.0, w_hat, s_hat);
            k2 = integration_function(ic + 0.5*k1*h, h/2, w_hat, s_hat);
            k3 = integration_function(ic + 0.5*k2*h, h/2, w_hat, s_hat);
            k4 = integration_function(ic + k3*h, h, w_hat, s_hat);
            //Integration
            ic = ic + (k1 + 2*k2 + 2*k3 + k4)*(1./6)*h;
            //Normalize quaternion part
            q_Ik_I = Eigen::Quaterniond (ic.block<4,1>(QX,0));
            q_Ik_I.normalize();
            ic.block<4,1>(QX,0) = q_Ik_I.coeffs();
            //Integrate Covariance matrix

//            k1 = integration_function_P(P, 0.0, w_hat, s_hat);
//            k2 = integration_function_P(P + 0.5*k1*h, h/2, w_hat, s_hat);
//            k3 = integration_function_P(P + 0.5*k2*h, h/2, w_hat, s_hat);
//            k4 = integration_function_P(P + k3*h, h, w_hat, s_hat);
//            //Integration
//            P = P + (k1 + 2*k2 + 2*k3 + k4)*(1./6)*h;

        }
        //ALTERNATIVE INTEGRATION FOR position and velocity.
        Eigen::Vector3d ss;
        Eigen::Vector3d yy;

        ss = simpson_integrate(q_Ik_I, s_hat);
        yy = (dt/6.0)*( (4/2.0)*ss + ss); //With ss at t_initial = 0


        //Re-Construct original states
        gravity_in_board = reef_msgs::quaternion_to_rotation(initial_board_q)*gravity;
        Eigen::Vector3d world_to_imu_position(xHat(PX), xHat(PY), xHat(PZ));
        Eigen::Vector3d velocity_W(xHat(U), xHat(V), xHat(W));
        Eigen::Vector3d gyro_bias(xHat(BWX), xHat(BWY) , xHat(BWZ));
        Eigen::Vector3d accel_bias(xHat(BAX), xHat(BAY) , xHat(BAZ));
        Eigen::Vector3d position_camera_in_imu_frame(xHat(P_IX), xHat(P_IY), xHat(P_IZ));
//        //Attitude
        q_W_to_I = xHat.block<4,1>(QX,0);
        xHat.block<4,1>(QX,0) = reef_msgs::quatMult(q_Ik_I,q_W_to_I).coeffs(); //This recovers the full current attitude after propagation.
        //Velocity
//        xHat.block<3,1>(U,0) = velocity_W  + q_W_to_I.toRotationMatrix()*ic.block<3,1>(U,0) + gravity_in_board*dt;
        xHat.block<3,1>(U,0) = velocity_W  + q_W_to_I.toRotationMatrix()*ss + gravity_in_board*dt;
        //Position
//        xHat.block<3,1>(PX,0) = world_to_imu_position + velocity_W*dt + 0.5*gravity_in_board*dt*dt + q_W_to_I.toRotationMatrix()*ic.block<3,1>(PX,0);
        xHat.block<3,1>(PX,0) = world_to_imu_position + velocity_W*dt + q_W_to_I.toRotationMatrix()*yy + 0.5*gravity_in_board*dt*dt;
    }

    Eigen::Vector3d CameraIMUEKF::simpson_integrate( Eigen::Quaterniond q_t1, Eigen::Vector3d acceleration)
    {
        //Interpolate w and s here
        Eigen::Vector3d s0;
        Eigen::Vector3d s1;
        Eigen::Vector3d s2;
        s0 = s_k0 + (acceleration - s_k0)*(0) - xHat.block<3,1>(BAX,0);
        s1 = s_k0 + (acceleration - s_k0)*(0.5) - xHat.block<3,1>(BAX,0);
        s2 = s_k0 + (acceleration - s_k0)*(1.0) - xHat.block<3,1>(BAX,0);
        Eigen::Quaterniond q_t0_halfdt;

        q_t0_halfdt.coeffs() << 0.5*q_t1.x(), 0.5*q_t1.y(), 0.5*q_t1.z(), 1.0;
        Eigen::Vector3d integral;
        integral = (dt/6.0)*( ( s0 )  + 4* q_t0_halfdt.toRotationMatrix().transpose() * (s1) + q_t1.toRotationMatrix().transpose()*(s2));
        return integral;

    }


    Eigen::MatrixXd CameraIMUEKF::integration_function( Eigen::MatrixXd x, double t, Eigen::Vector3d omega,Eigen::Vector3d acceleration)
    {
        //Interpolate w and s here
        Eigen::Vector3d w;
        Eigen::Vector3d s;
        w = w_k0 + (omega - w_k0)*(t/dt) - xHat.block<3,1>(BWX,0);
        s = s_k0 + (acceleration - s_k0)*(t/dt) - xHat.block<3,1>(BAX,0);

        gravity_in_board = reef_msgs::quaternion_to_rotation(initial_board_q)*gravity;
        Eigen::Quaterniond q_Ik_to_I;
        q_Ik_to_I.vec() << x(QX), x(QY), x(QZ);
        q_Ik_to_I.w() = x(QW);
        q_Ik_to_I.normalize();


        Eigen::Vector3d world_to_imu_position(x(PX), x(PY), x(PZ));
        Eigen::Vector3d velocity_W(x(U), x(V), x(W));
        Eigen::Vector3d gyro_bias(x(BWX), x(BWY) , x(BWZ));
        Eigen::Vector3d accel_bias(x(BAX), x(BAY) , x(BAZ));
        Eigen::Vector3d position_camera_in_imu_frame(x(P_IX), x(P_IY), x(P_IZ));

        Eigen::Quaterniond imu_to_camera_quat;
        imu_to_camera_quat.vec() << x(Q_IX), x(Q_IY), x(Q_IZ);
        imu_to_camera_quat.w() = x(Q_IW);
        Eigen::Matrix4d Omega_matrix;
        Omega_matrix.setZero();
        Omega_matrix.topLeftCorner<3,3>() = -1.0*(reef_msgs::skew(w));
        Omega_matrix.block<3,1>(0,3) = w;
        Omega_matrix.block<1,3>(3,0) = -w.transpose();

        Eigen::MatrixXd dxdt = Eigen::MatrixXd::Zero(23,1);
        Eigen::Quaterniond q_Ik_to_I_dot;
        q_Ik_to_I_dot.coeffs() = 0.5 * Omega_matrix * q_Ik_to_I.coeffs();
//        High-precision, consistent EKF-based visual-inertial odometry.The International Journal of Robotics Research
        dxdt.block<4,1>(QX,0) = q_Ik_to_I_dot.coeffs();//Attitude_dot
//        dxdt.block<3,1>(U,0) =  s;//ve_dot
        dxdt.block<3,1>(U,0) = q_Ik_to_I.toRotationMatrix() * s;//ve_dot
        dxdt.block<3,1>(PX,0) =  x.block<3,1>(U,0);//Rho_dot
        return dxdt;
    }


    Eigen::MatrixXd CameraIMUEKF::integration_function_P( Eigen::MatrixXd Pic, double t, Eigen::Vector3d omega,Eigen::Vector3d acceleration)
    {
        //Interpolate w and s here
        Eigen::Vector3d w;
        Eigen::Vector3d s;
        w = w_k0 + (omega - w_k0)*(t/dt) - xHat.block<3,1>(BWX,0);
        s = s_k0 + (acceleration - s_k0)*(t/dt) - xHat.block<3,1>(BAX,0);

        Eigen::Quaterniond q_W_to_I;
        q_W_to_I.vec() << xHat(QX), xHat(QY), xHat(QZ);
        q_W_to_I.w() = xHat(QW);
        q_W_to_I.normalize();

        F.setZero();
        F.topLeftCorner<3,3>() = -reef_msgs::skew(w);
        F.block<3,3>(0,9) = -I;
        F.block<3,3>(3,6) = I;
        F.block<3,3>(6,0) = -1.0 * (q_W_to_I.toRotationMatrix() * reef_msgs::skew(s));
        F.block<3,3>(6,12)  = -1.0 * q_W_to_I.toRotationMatrix();

        G.setZero();
        G.topLeftCorner<3,3>() = -I;
        G.block<3,3>(6,3) = -1 * (q_W_to_I.toRotationMatrix());
        G.block<3,3>(9,6) = I;
        G.block<3,3>(12,9) = I;

        Eigen::MatrixXd Pdot;
        Pdot = F * Pic + Pic * F.transpose() + G * Q * G.transpose();

        return Pdot;
    }



    void CameraIMUEKF::nonLinearUpdate() {

        // Break state into variables to make it easier to read (and write) code
        Eigen::Quaterniond world_to_imu_quat;
        world_to_imu_quat.vec() << xHat(QX), xHat(QY), xHat(QZ);
        world_to_imu_quat.w() = xHat(QW);

        Eigen::Vector3d world_to_imu_position(xHat(PX), xHat(PY), xHat(PZ));
        Eigen::Vector3d velocity_W(xHat(U), xHat(V), xHat(W));
        Eigen::Vector3d gyro_bias(xHat(BWX), xHat(BWY) , xHat(BWZ));
        Eigen::Vector3d accel_bias(xHat(BAX), xHat(BAY) , xHat(BAZ));
        Eigen::Vector3d position_camera_in_imu_frame(xHat(P_IX), xHat(P_IY), xHat(P_IZ));

        Eigen::Quaterniond imu_to_camera_quat;
        imu_to_camera_quat.vec() << xHat(Q_IX), xHat(Q_IY), xHat(Q_IZ);
        imu_to_camera_quat.w() = xHat(Q_IW);

        K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
        Eigen::MatrixXd correction(21,1);
        alphaVector = Eigen::MatrixXd::Ones(21, 1) - betaVector;
        gammas.setZero();
        gammas.diagonal() = alphaVector;

        if(enable_partial_update_){
            correction = (Eigen::MatrixXd::Identity(21, 21) - gammas) * K * (z - expected_measurement);
        }
        else{
            correction = K * (z- expected_measurement);
        }

        Eigen::Quaterniond quat_error;
        //Let's save the correction for the quaternion attitude.
        q_attitude_error = correction.block<3,1>(0,0);
        quat_error.vec() = 0.5 * correction.block<3,1>(0,0);
        quat_error.w() = 1.0;
        world_to_imu_quat = reef_msgs::quatMult(quat_error , world_to_imu_quat) ;
        world_to_imu_quat.normalize();
        xHat.block<4,1>(QX,0) =  world_to_imu_quat.coeffs();

        //Here we also save the quaterinion error. Now for the offset orientation.
        q_offset_error = correction.block<3,1>(18,0);
        quat_error.vec() = 0.5 * correction.block<3,1>(18,0);
        quat_error.w() = 1.0;
        imu_to_camera_quat =  reef_msgs::quatMult(quat_error , imu_to_camera_quat );
        imu_to_camera_quat.normalize();
        xHat.block<4,1>(Q_IX,0) =  imu_to_camera_quat.coeffs();

        xHat.block<15,1>(PX,0) = xHat.block<15,1>(PX,0) + correction.block<15,1>(3,0);

        if(enable_partial_update_)
        {
            //Change this update. Do UDU^T.
            Eigen::MatrixXd P_prior;
            P_prior = P; //store the prior for partial update
            P = ( Eigen::MatrixXd::Identity(21,21) - K * H ) * P;
            P = (gammas * (P_prior - P) * gammas) + P;
            if(P.diagonal().array().sqrt().hasNaN()){
                ROS_ERROR("Nan detected");
                            }


        } else
            P = ( Eigen::MatrixXd::Identity(21,21) - K * H ) * P;

    }

    void CameraIMUEKF::nonLinearUpdateSequentially(charuco_ros::CharucoCornerMsg charuco_measure) {
        Eigen::MatrixXd S;
        camera_imu_calib::ExpectedMeasurement expected_msg;
        if(publish_expected_meas_&& charuco_measure.metric_corners.size() == 15) {

            expected_msg.header = charuco_measure.header;
            expected_msg.expected_measurement.reserve(2 * charuco_measure.metric_corners.size());
            expected_msg.pixel_measurement.reserve(2 * charuco_measure.metric_corners.size());
        }

        for (unsigned int i = 0; i < charuco_measure.metric_corners.size(); i++) {
            //Update with x
            aruco_helper(charuco_measure.metric_corners[i].corner, charuco_measure.pixel_corners[i].corner, 0);
            nonLinearUpdate();
            if(publish_expected_meas_&& charuco_measure.metric_corners.size() == 15) {
                S = H * P * H.transpose() + R;
                expected_msg.expected_measurement.push_back(expected_measurement(0));
                expected_msg.pixel_measurement.push_back(z(0));
                expected_msg.covariance.push_back(S(0,0));
            }

            //Update with y
            aruco_helper(charuco_measure.metric_corners[i].corner, charuco_measure.pixel_corners[i].corner, 1);
            nonLinearUpdate();
            if(publish_expected_meas_&& charuco_measure.metric_corners.size() == 15) {
                S = H * P * H.transpose() + R;
                expected_msg.expected_measurement.push_back(expected_measurement(0));
                expected_msg.pixel_measurement.push_back(z(0));
                expected_msg.covariance.push_back(S(0,0));
            }

        }

        if(publish_expected_meas_ && charuco_measure.metric_corners.size() == 15) {
            expect_pixel_publisher_.publish(expected_msg);
        }

        }



    void CameraIMUEKF::publish_state() {
        if(publish_full_quaternion){
            state_msg.world_to_imu.orientation.x = xHat(0);
            state_msg.world_to_imu.orientation.y = xHat(1);
            state_msg.world_to_imu.orientation.z = xHat(2);
            state_msg.world_to_imu.orientation.w = xHat(3);
        }
        else{
            state_msg.world_to_imu.orientation.x = q_attitude_error(0);
            state_msg.world_to_imu.orientation.y = q_attitude_error(1);
            state_msg.world_to_imu.orientation.z = q_attitude_error(2);
            state_msg.world_to_imu.orientation.w = 1.0;
        }

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

        if(publish_full_quaternion){
            state_msg.imu_to_camera.orientation.x = xHat(19);
            state_msg.imu_to_camera.orientation.y = xHat(20);
            state_msg.imu_to_camera.orientation.z = xHat(21);
            state_msg.imu_to_camera.orientation.w = xHat(22);
        }
        else{
            state_msg.imu_to_camera.orientation.x = q_offset_error(0);
            state_msg.imu_to_camera.orientation.y = q_offset_error(1);
            state_msg.imu_to_camera.orientation.z = q_offset_error(2);
//            state_msg.imu_to_camera.orientation.x = rpy_measurement(0);
//            state_msg.imu_to_camera.orientation.y = rpy_measurement(1);
//            state_msg.imu_to_camera.orientation.z = rpy_measurement(2);
            state_msg.imu_to_camera.orientation.w = 1.0;
        }

        for(unsigned int i = 0; i<=20; i++){
            //In this section we use the index i to acces the covariance matrix.

            state_msg.P[i] = P(i,i);
            if(i<3)
            {   //This covers i = 0,1,2
                state_msg.sigma_minus[i] = q_attitude_error(i) - 3 * sqrt(P(i,i));
                state_msg.sigma_plus[i] = q_attitude_error(i) + 3 * sqrt(P(i,i));

            }
            if (i >= 3 && i<=17)
            {   //This covers i = 3,4,5,..,17
                state_msg.sigma_minus[i] = xHat(i+1) - 3 * sqrt(P(i,i));
                state_msg.sigma_plus[i] = xHat(i+1) + 3 * sqrt(P(i,i));
            }

            if(i >=18){
                //This covers i = 18,19,20
                state_msg.sigma_minus[i] = q_offset_error(i-18) - 3 * sqrt(P(i,i));
                state_msg.sigma_plus[i] = q_offset_error(i-18) + 3 * sqrt(P(i,i));

            }

        }

        state_publisher_.publish(state_msg);

    }

    void CameraIMUEKF::getInitialPose(camera_imu_calib::IMUCalibration msg){
        initial_imu_q.vec() << msg.world_to_imu.orientation.x,msg.world_to_imu.orientation.y,msg.world_to_imu.orientation.z;
        initial_imu_q.w() = msg.world_to_imu.orientation.w;
        initial_imu_position << msg.world_to_imu.position.x,msg.world_to_imu.position.y,msg.world_to_imu.position.z;
    }

    void CameraIMUEKF::getBoardPose(geometry_msgs::PoseStamped msg){
        initial_board_q.vec() << msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z;
        initial_board_q.w() = msg.pose.orientation.w;
        initial_board_position << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
    }


    void CameraIMUEKF::getCharucoPose(geometry_msgs::PoseStamped msg){
        charuco_pose.vec() << msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z;
        charuco_pose.w() = msg.pose.orientation.w;
        Eigen::Vector3d rpy;
        reef_msgs::roll_pitch_yaw_from_rotation321(charuco_pose.toRotationMatrix().transpose(), rpy);
        charuco_roll = 57.3*rpy(0);
    }

    bool CameraIMUEKF::acceptCharucoMeasurement(){
//        Eigen::MatrixXd differential_pixel_vector(15,1);
//        for (int i = 0; i < charuco_measurement.pixel_corners.size() ; ++i) {
//            differential_pixel_vector(i,0) = sqrt(pow((charuco_measurement.pixel_corners[i].corner.x - past_charuco_measurent.pixel_corners[i].corner.x),2) + pow((charuco_measurement.pixel_corners[i].corner.y - past_charuco_measurent.pixel_corners[i].corner.y),2));
//        }
////        ROS_INFO("Vector pixel %f",differential_pixel_vector.maxCoeff());
//        past_charuco_measurent = charuco_measurement; //This will reject the current outlier and the next measurement.
//        double roll_difference;
//        roll_difference = charuco_roll - previous_charuco_roll;
//        previous_charuco_roll = charuco_roll;
//        if (differential_pixel_vector.maxCoeff() > pixel_difference_threshold || abs(roll_difference)>=30 ){
//            ROS_INFO("Vector Pixel rejection with %f",differential_pixel_vector.maxCoeff());
//            ROS_INFO("Charuco Attitude jump %f",abs(roll_difference));
//            return false;
//        }
//        else{
//            return true;}
        rpy_measurement = 57.3*computePNP(charuco_measurement);
        Eigen::Vector3d angle_difference;
        angle_difference = rpy_measurement - previous_charuco_roll;
        previous_charuco_roll =  rpy_measurement;
        if (abs(angle_difference(0))>=pixel_difference_threshold || angle_difference(1)>pixel_difference_threshold || angle_difference(2)>pixel_difference_threshold || rpy_measurement.norm()==0){
//            ROS_INFO("Charuco Attitude jump     ");
//            std::cout<<angle_difference;
            return false;
        }
        else{
            return true;}
    }


    bool CameraIMUEKF::chi2AcceptPixels()
    {

        //Compute Mahalanobis distance.
        Eigen::MatrixXd S(1, 1);
        S = H * P * H.transpose() + R;
        Eigen::MatrixXd Mahalanonbis_d(1,1);

        Mahalanonbis_d = (z - expected_measurement).transpose() * S.inverse() * (z - expected_measurement);
        //Value for 99% we need 6.63.
        //Value for 95% we 3.84
        if (Mahalanonbis_d(0) >= mahalanobis_param)
        {
            ROS_INFO_STREAM("Pixels rejected");
            return false;
        }
        else
        {
            return true;
        }
    }




    double getVectorMagnitude(double x, double y, double z)
    {
        return sqrt(x * x + y * y + z * z);
    }
}