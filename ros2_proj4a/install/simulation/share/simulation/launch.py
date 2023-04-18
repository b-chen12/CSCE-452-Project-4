from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import *
from launch.substitutions import *
from launch_ros.actions import Node
from simulation.disc_robot import load_disc_robot
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import yaml


def generate_launch_description():
    # Argument Object That Takes in Launch Arguments
    bag_in = LaunchConfiguration('bag_in')
    # Sets up location of where the bag will be played from
    bag_in_arg = DeclareLaunchArgument(
                            'bag_in',
                            default_value='bags/example1')
    urdf_file_name = 'normal.robot'
    urdf = os.path.join(
        get_package_share_directory('simulation'),
        urdf_file_name)
    
    robot = load_disc_robot(urdf)
    play_bag = ExecuteProcess(cmd = ['ros2', 'bag', 'play', bag_in])
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot['urdf']}],),
            
        Node(
            package='simulation',
            executable='proj4a',
            name='proj4a',
            output='screen'),
        bag_in_arg,
        play_bag
    ])