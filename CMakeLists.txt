cmake_minimum_required(VERSION 2.8.3)
project(camera_imu_calib)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  reef_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
        tf2_eigen
        message_filters
        message_generation
        charuco_ros
)

find_package(Eigen3 REQUIRED)

add_message_files(
        FILES
        IMUCalibration.msg
        ExpectedMeasurement.msg
)

generate_messages(
        DEPENDENCIES
        geometry_msgs
        std_msgs

)

catkin_package(
        INCLUDE_DIRS
        LIBRARIES
        CATKIN_DEPENDS roscpp rospy charuco_ros sensor_msgs reef_msgs geometry_msgs sensor_msgs tf2_eigen
)

include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(${PROJECT_NAME}_node
        src/sensor_manager.cpp
        src/estimator.cpp
        include/camera_imu_calib/sensor_manager.h
        include/camera_imu_calib/estimator.h
        src/camera_imu_ekf.cpp include/camera_imu_calib/camera_imu_ekf.h)

add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})


add_executable(show_res
        src/view_expected_measurement.cpp)
add_dependencies(show_res ${catkin_EXPORTED_TARGETS})
target_link_libraries(show_res ${catkin_LIBRARIES})