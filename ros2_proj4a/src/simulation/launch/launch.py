from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import *
from launch.substitutions import *
from launch_ros.actions import Node
from simulation.disc_robot import load_disc_robot
import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import yaml


def generate_launch_description():
    # Argument Object That Takes in Launch Arguments
    # bag_in = LaunchConfiguration('bag_in')
    #robot_name = 'normal.robot'
    map_name = LaunchConfiguration('map_name')
    # Sets up location of where the bag will be played from
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='normal.robot',
        description='asldkf',
    )
    map_name_arge = DeclareLaunchArgument(
        'map_name',
        default_value='brick.world',
    )
    # bag_in_arg = DeclareLaunchArgument(
    #                         'bag_in',
    #                         default_value='bags/input1')
    file_name_robot_str = 'ideal.robot'
    u = os.path.join(
        get_package_share_directory('simulation'),
        file_name_robot_str)
    
    robot = load_disc_robot(u)
    # play_bag = ExecuteProcess(cmd = ['ros2', 'bag', 'play', bag_in])
    return LaunchDescription([
        robot_name_arg,
        map_name_arge,
        # bag_in_arg,
        # play_bag,
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
            parameters=[{'robot_name': LaunchConfiguration('robot_name')}],
            output='screen'),
        Node(
            package='simulation',
            executable='nav_controller',
            name='nav_controller',
            parameters=[{'robot_name': LaunchConfiguration('robot_name')}],
            output='screen'),
        Node(
            package='simulation',
            executable='vel_translator',
            name='vel_translator',
            parameters=[{'robot_name': LaunchConfiguration('robot_name')}],
            output='screen'),
        
        
    ])