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
            accel_calibrated(false),
            accInitSampleCount(0),
            cornerSampleCount(false),
            fx(0),fy(0),cx(0),cy(0),
            number_of_features(16),
            publish_full_quaternion(false),
            publish_expected_meas_(false),
            enable_partial_update_(true),
            initialized_timer(false)
    {
        nh_private.param<double>("estimator_dt", dt, 0.002);
        nh_private.param<int>("number_of_features", number_of_features, 16);
        nh_private.param<bool>("publish_full_quaternion", publish_full_quaternion, true);
        nh_private.param<bool>("publish_expected_meas", publish_expected_meas_, true);
        nh_private.param<bool>("enable_partial_update", enable_partial_update_, true);
        nh_private.param<bool>("enable_dynamic_update", enable_dynamic_update, true);
        nh_private.param<double>("mahalanobis_param", mahalanobis_param, 100);

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

        betaVector = Eigen::MatrixXd(21,1);

        R = Eigen::MatrixXd(2 * number_of_features, 2 * number_of_features);

        I = Eigen::MatrixXd(3,3);
        I.setIdentity();

        R_std = Eigen::MatrixXd(1,1);

        reef_msgs::importMatrixFromParamServer(nh_private, xHat0, "xHat0");
        reef_msgs::importMatrixFromParamServer(nh_private, P0, "P0");
        reef_msgs::importMatrixFromParamServer(nh_private, Q, "Q");
        reef_msgs::importMatrixFromParamServer(nh_private, R_std, "R_std");
        reef_msgs::importMatrixFromParamServer(nh_private, betaVector, "beta");

        double R_std_float; // Set to 1 as default.
        R_std_float = R_std(0,0);
        ROS_INFO_STREAM("hI");
        ROS_INFO_STREAM("R_std_float");
        ROS_INFO_STREAM(R_std_float);

        R = R_std_float*R.setIdentity();

        R = R*R;
//        P0 = P0*P0;
//        Q = Q * dt;

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

        Eigen::Matrix3d C_rot_mat;
        C_rot_mat << rotMat.at<double>(0, 0), rotMat.at<double>(0, 1), rotMat.at<double>(0, 2),
                rotMat.at<double>(1, 0), rotMat.at<double>(1, 1), rotMat.at<double>(1, 2),
                rotMat.at<double>(2, 0), rotMat.at<double>(2, 1), rotMat.at<double>(2, 2);
        // ^^ This can be interpreted as the rotation matrix from the Camera to the board or the DCM of the board to the camera frame

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

            //Block to overwrite PNP
//            20191008_001.bag
//            world_to_imu_quat.vec() << -0.523072936955,-0.483652921944,-0.472375409361;
//            world_to_imu_quat.w() = 0.518975901453;
//            world_to_imu_quat.normalize();
//            world_to_imu_pose << -0.157741842982, 0.0736774667826, -2.63509427242;

              world_to_imu_quat = initial_imu_q;
              world_to_imu_pose = initial_imu_position;



//                closer_002.bag
//                world_to_imu_quat.vec() << -0.523072936955,-0.483652921944,-0.472375409361;
//                world_to_imu_quat.w() = 0.518975901453;
//                world_to_imu_quat.normalize();
//                world_to_imu_pose << -0.157741842982, 0.0736774667826, -2.63509427242;




            xHat.block<3,1>(QX,0) = world_to_imu_quat.vec();
            xHat(QW,0) = world_to_imu_quat.w();
            xHat.block<3,1>(PX,0) = world_to_imu_pose;
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

    void CameraIMUEKF::initializeAcc(geometry_msgs::Vector3 acc, geometry_msgs::Vector3 gyro) {

        accSampleAverage.x += acc.x;
        accSampleAverage.y += acc.y;
        accSampleAverage.z += acc.z;

        gyroSampleAverage.x += gyro.x;
        gyroSampleAverage.y += gyro.y;
        gyroSampleAverage.z += gyro.z;

        accInitSampleCount++;

        if (accInitSampleCount == ACC_SAMPLE_SIZE){

            accSampleAverage.x /= ACC_SAMPLE_SIZE;
            accSampleAverage.y /= ACC_SAMPLE_SIZE;
            accSampleAverage.z /= ACC_SAMPLE_SIZE;

            gyroSampleAverage.x /= ACC_SAMPLE_SIZE;
            gyroSampleAverage.y /= ACC_SAMPLE_SIZE;
            gyroSampleAverage.z /= ACC_SAMPLE_SIZE;

            accel_calibrated = true;

            xHat(BAX,0) = accSampleAverage.x;
            xHat(BAY,0) = accSampleAverage.y;
            xHat(BWX, 0) = gyroSampleAverage.x;
            xHat(BWY, 0) = gyroSampleAverage.y;
            xHat(BWZ, 0) = gyroSampleAverage.z;
            ROS_WARN_STREAM("XHat post IMU initialization is  \n" <<xHat);
            ROS_WARN_STREAM("Gravity is  \n" <<getVectorMagnitude(accSampleAverage.x,accSampleAverage.y,accSampleAverage.z));
            ROS_WARN_STREAM("Accel components are  \n" <<accSampleAverage);

        }

    }

    void CameraIMUEKF::sensorUpdate(sensor_msgs::Imu imu) {

        if (isnan(getVectorMagnitude(imu.linear_acceleration.x, imu.linear_acceleration.y,imu.linear_acceleration.z))){
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
                if(diff.toSec() >= 60.0 && diff.toSec() < 80.0)
                {
                    betaVector.block<3,1>(P_IX-1,0) << 0.3,0.3,0.3;
                    ROS_WARN_STREAM("Beta updated \n" << betaVector);
                }
                if(diff.toSec() >= 80.0 && diff.toSec() < 100.0)
                {
                    betaVector.block<3,1>(P_IX-1,0) << 0.6,0.6,0.6;
                    ROS_WARN_STREAM("Beta updated \n" << betaVector);

                }

                if(diff.toSec() >= 100.0 )
                {
                    betaVector.block<3,1>(P_IX-1,0) << 0.9,0.9,0.9;
                    ROS_WARN_STREAM("Beta updated \n" << betaVector);

                }

            }
            //Don't update if the measurement is an outlier
//            if(chi2AcceptPixels()){
                nonLinearUpdate();
//            }
            //Reset got measurement flag
            got_measurement = false;
        }
        publish_state();

    }

    void CameraIMUEKF::sensorUpdate(ar_sys::ArucoCornerMsg aruco_corners) {

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
//            ROS_WARN_STREAM(i);
        }

        if(publish_expected_meas_){
            camera_imu_calib::ExpectedMeasurement expected_msg;
            expected_msg.header = aruco_corners.header;
            expected_msg.expected_measurement.reserve(4 * aruco_corners.metric_corners.size());
            expected_msg.pixel_measurement.reserve(4 * aruco_corners.metric_corners.size());

            for(unsigned int i =0; i < 8 * aruco_corners.metric_corners.size(); i++){
                expected_msg.expected_measurement.push_back(expected_measurement(i));
                expected_msg.pixel_measurement.push_back(z(i));
            }
            expect_pixel_publisher_.publish(expected_msg);

        }
        got_measurement = true;
    }

    void CameraIMUEKF::aruco_helper(ar_sys::SingleCorner metric_corner, ar_sys::SingleCorner pixel_corner,
                                    unsigned int index, unsigned int position) {

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

        Eigen::Vector2d feature_pixel_position_camera_frame;

        feature_pixel_position_camera_frame << fx * (h_hat(0)/h_hat(2)) + cx,
                                                fy * (h_hat(1)/h_hat(2)) + cy;

        expected_measurement.block<2,1>(8*index + position, 0) = feature_pixel_position_camera_frame;
        z.block<2,1>(8*index + position, 0) << pixel_corner.x, pixel_corner.y;

        q_block = q_I_to_C.toRotationMatrix().transpose() * reef_msgs::skew( q_W_to_I.toRotationMatrix().transpose() * (measured_metric - p_I_in_W) );
        alpha_block =  reef_msgs::skew(q_I_to_C.toRotationMatrix().transpose() *q_W_to_I.toRotationMatrix().transpose()*measured_metric) -1* reef_msgs::skew ( q_I_to_C.toRotationMatrix().transpose()*q_W_to_I.toRotationMatrix().transpose() * p_I_in_W) -1*reef_msgs::skew(q_I_to_C.toRotationMatrix().transpose()*position_camera_in_imu_frame) ;
//        alpha_block =  reef_msgs::skew(q_I_to_C.toRotationMatrix().transpose() * ( q_W_to_I.toRotationMatrix().transpose() * (measured_metric - p_I_in_W) - position_camera_in_imu_frame) );

        partial_y_measure_p_fc << fx, 0,    -fx * h_hat(0)/h_hat(2),
                                  0,  fy,   -fy * h_hat(1)/h_hat(2);
        partial_y_measure_p_fc = (1./h_hat(2)) * partial_y_measure_p_fc;
        partial_x_measure << q_block, pos_block, zero_block, zero_block, zero_block, p_c_i_block, alpha_block;

        H.block<2,21>(8*index + position,0)  = partial_y_measure_p_fc * partial_x_measure;
    }

    void CameraIMUEKF::nonLinearPropagation(Eigen::Vector3d omega, Eigen::Vector3d acceleration) {
        Eigen::Vector3d gravity(0,-accSampleAverage.z,0);
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
        F.block<3,3>(6,0) = -1 * (q_W_to_I.toRotationMatrix() * reef_msgs::skew(s_hat));
        F.block<3,3>(6,12)  = -1 * q_W_to_I.toRotationMatrix();

        G.setZero();
        G.topLeftCorner<3,3>() = -I;
        G.block<3,3>(6,3) = -1 * (q_W_to_I.toRotationMatrix());
        G.block<3,3>(9,6) = I;
        G.block<3,3>(12,9) = I;


        //Block to correct gravity by takin into account the rotation of the board;
        Eigen::Vector3d corrected_gravity;
        Eigen::Matrix3d C_from_optitrack_to_board;
        Eigen::Quaterniond q_optitrack_to_board;

//Trying a different integration here for the attitude
//        Eigen::Quaterniond q_I_to_Ik;
//        q_I_to_Ik.vec() = (w_hat/w_hat.norm())*sin(w_hat.norm()*0.5*dt);
//        q_I_to_Ik.w() = cos(w_hat.norm()*0.5*dt);
//        q_W_to_I = reef_msgs::quatMult(q_I_to_Ik,q_W_to_I);


        //Propagate velocity of IMU wrt world
        xHat.block<3,1>(U,0) = xHat.block<3,1>(U,0) + (q_W_to_I.toRotationMatrix() * s_hat + gravity)*dt;
        //Propagate position of IMU wrt world
        xHat.block<3,1>(PX,0) =  xHat.block<3,1>(PX,0) + velocity_W * dt;

        //Propagate the rest of the states
        xHat.block<11,1>(BWX,0) = xHat.block<11,1>(BWX,0);



        //Propagate attitude original block
        Eigen::Matrix4d Omega_matrix;
        Omega_matrix.setZero();
        Omega_matrix.topLeftCorner<3,3>() = -1*(reef_msgs::skew(w_hat));
        Omega_matrix.block<3,1>(0,3) = w_hat;
        Omega_matrix.block<1,3>(3,0) = -w_hat.transpose();
        q_W_to_I = (Eigen::Matrix4d::Identity() + 0.5 * dt * Omega_matrix) * q_W_to_I.coeffs() ;


//        xHat.block<19,1>(4,0) = xHat.block<19,1>(4,0) + dt * true_dynamics;
        xHat.block<4,1>(0,0) =  q_W_to_I.coeffs();

        P = P + (F * P + P * F.transpose() + G * Q * G.transpose()) * dt;
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

        if(enable_partial_update_)
        {correction = (Eigen::MatrixXd::Identity(21, 21) - gammas) * K * (z - expected_measurement);}
        else
        {correction = K * (z- expected_measurement);}

        Eigen::Quaterniond quat_error;
        //Let's save the correction for the quaternion attitude.
        q_attitude_error = correction.block<3,1>(0,0);
        quat_error.vec() = 0.5 * correction.block<3,1>(0,0);
        quat_error.w() = 1;
//        world_to_imu_quat = world_to_imu_quat * quat_error;
        world_to_imu_quat = reef_msgs::quatMult(quat_error , world_to_imu_quat) ;
        world_to_imu_quat.normalize();
        xHat.block<4,1>(0,0) =  world_to_imu_quat.coeffs();

        //Here we also save the quaterinion error. Now for the offset orientation.
        q_offset_error = correction.block<3,1>(18,0);
        quat_error.vec() = 0.5 * correction.block<3,1>(18,0);
        quat_error.w() = 1;
//        imu_to_camera_quat = imu_to_camera_quat * quat_error;
        imu_to_camera_quat =  reef_msgs::quatMult(quat_error , imu_to_camera_quat );
        imu_to_camera_quat.normalize();
        xHat.block<4,1>(19,0) =  imu_to_camera_quat.coeffs();

        xHat.block<15,1>(4,0) = xHat.block<15,1>(4,0) + correction.block<15,1>(3,0);

        if(enable_partial_update_)
        {
            Eigen::MatrixXd P_prior;
            P_prior = P; //store the prior for partial update
            P = ( Eigen::MatrixXd::Identity(21,21) - K * H ) * P;
            P = (gammas * (P_prior - P) * gammas) + P;
        } else
            P = ( Eigen::MatrixXd::Identity(21,21) - K * H ) * P;

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
            state_msg.imu_to_camera.orientation.w = 1.0;
        }

        for(unsigned int i =0; i<20; i++){
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