# Run following commands to build and launch simulation after git clone

```
source /opt/ros/foxy/setup.bash

cd pybullet-imu-viz

colcon build

source install/setup.bash

ros2 launch imu_listener imu_pybullet.launch.py

```

see the blog article here - https://robofoundry.medium.com/simple-pybullet-ros2-node-to-visualize-sensor-data-c88d97f18d7d

## in order to use the ros bag for testing without having real sensor publishing to /imu/data

### use following command to display details of ros bag

```
ros2 bag info pybullet_imu_data_rosbag/
```

### use following command to start publishing data to /imu/data topic from ros bag [make sure you have launched the simulation using launch file above first to see the husky robot moving in simulation]

```
ros2 bag play pybullet_imu_data_rosbag/
```