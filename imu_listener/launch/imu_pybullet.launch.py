# Author: Robofoundry
# Date: Apr 2, 2022
# Description: Launch file to start nodes for RPi


import os
from setuptools import setup
from glob import glob
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

  # Set the path to different files and folders.
  use_sim_time = LaunchConfiguration('use_sim_time', default='false')
  pkg_share = get_package_share_directory('imu_listener')
  #imu_params_config = os.path.join(get_package_share_directory('imu_listener'), 'config', 'imu_params.yaml')


  
  # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
  imu_pybullet = Node(
    package='imu_listener',
    executable='imu_sub_pybullet',
    )
    

  ld = LaunchDescription()
  ld.add_action(imu_pybullet)


  return ld