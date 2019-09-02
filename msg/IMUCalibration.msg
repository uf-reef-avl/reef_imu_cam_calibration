Header header
uint32 node_id

geometry_msgs/Pose world_to_imu
geometry_msgs/Pose imu_to_camera

geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 accel_bias
geometry_msgs/Vector3 gyro_bias

float64[21] P
float64[21] sigma_plus
float64[21] sigma_minus

