//initialized_pnp
// Created by prashant on 7/16/19.
//
#include "../include/camera_imu_calib/sensor_manager.h"

namespace calibration{

    SensorManager::SensorManager() : private_nh_("~"), nh_(""), num_pose(0){

        corner_subscriber = nh_.subscribe("/charuco_node/corner", 1, &SensorManager::cornerCallback, this);
        pose_charuco_subscriber = nh_.subscribe("/charuco_node/pose", 1, &SensorManager::charucoPoseCallback, this);
        initial_pose_subscriber = nh_.subscribe("/calib_truth", 1, &SensorManager::initialPoseCallback, this);
        imu_subscriber_ = nh_.subscribe("imu/imu", 1, &SensorManager::imuCallback, this);
        camera_info_subscriber = nh_.subscribe("camera/rgb/camera_info", 1, &SensorManager::cameraInfoCallback,this);
        mocap_board_subscriber = nh_.subscribe("/tf_board/ned/pose_stamped", 1, &SensorManager::boardPoseCallback,this);
    }

    void SensorManager::cornerCallback(const charuco_ros::CharucoCornerMsg &msg) {
        if(calib_obj.got_camera_parameters)
            calib_obj.sensorUpdate(msg);
    }

    void SensorManager::charucoPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg) {
        if(calib_obj.got_camera_parameters)
            calib_obj.getCharucoPose(*msg);
    }

    void SensorManager::imuCallback(const sensor_msgs::ImuConstPtr &msg) {
        calib_obj.sensorUpdate(*msg);
    }

    void SensorManager::cameraInfoCallback(const sensor_msgs::CameraInfo &msg) {

        if (msg.K[0] == 0) {
            std::cout << msg << std::endl;
            ROS_ERROR("Camera Info message is zero --> Cannot use an uncalibrated camera!");
            return;
        }
        calib_obj.getCameraInfo(msg);
        camera_info_subscriber.shutdown();
    }

    void SensorManager::initialPoseCallback(const camera_imu_calib::IMUCalibrationConstPtr  &msg){
        calib_obj.getInitialPose(*msg);
//        initial_pose_subscriber.shutdown();

    }

    void SensorManager::boardPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg){
        calib_obj.getBoardPose(*msg);
        mocap_board_subscriber.shutdown();

    }

}

int main(int argc, char **argv) {

    ros::init(argc, argv, "camera_imu_calibration");
    calibration::SensorManager obj;
    ros::spin();
    return 0;
}
