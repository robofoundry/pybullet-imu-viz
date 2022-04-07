ros2 launch imu_listener imu_pybullet.launch.py

ros2 daemon stop; ros2 daemon start;

ros2 bag record /imu/data -o pybullet_imu_data_rosbag

ros2 bag info pybullet_imu_data_rosbag/

ros2 bag play pybullet_imu_data_rosbag/