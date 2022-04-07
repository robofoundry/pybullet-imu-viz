# Run following commands to build and launch simulation after git clone

```
source /opt/ros/foxy/setup.bash

cd pybullet-imu-viz

colcon build

source install/setup.bash

ros2 launch imu_listener imu_pybullet.launch.py

```

see the blog article here - 