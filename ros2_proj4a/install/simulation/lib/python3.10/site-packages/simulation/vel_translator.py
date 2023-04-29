import rclpy
import numpy as np
import math
from simulation.disc_robot import load_disc_robot
from simulation.disc_robot import disc_robot_urdf
from simulation.disc_robot import load_map
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os
from sensor_msgs.msg import LaserScan
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from math import sin, cos
import rclpy
from geometry_msgs.msg import Quaternion
from tf2_ros import TransformBroadcaster, TransformStamped, Buffer, TransformListener
from std_msgs.msg import Float64
from time import time
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import Twist


class VelController(Node):
    # This node publishes onto the three separate topics
    def __init__(self):
        super().__init__('VelController')
        
        # File name to get robot info and then store info in self.robot_info
        file_name = 'normal.robot' # self.get_parameter('robot_name').value
        u = os.path.join(get_package_share_directory('simulation'),file_name)
        self.robot_info = load_disc_robot(u)
        self.wheel_distance = self.robot_info['wheels']['distance']

        self.vel_sub = self.create_subscription(Twist, '/cmd_vel', self.listener_callback, 10)

        self.vl_pub = self.create_publisher(Float64, '/vl', 10)
        self.vr_pub = self.create_publisher(Float64, '/vr', 10)

    def listener_callback(self, msg):
        vl_msg = Float64()
        vr_msg = Float64()

        R = msg.linear.x / msg.angular.z

        vl = msg.angular.z * (R - self.wheel_distance / 2)
        vr = msg.angular.z * (R + self.wheel_distance / 2)
        
        vl_msg.data = vl
        vr_msg.data = vr

        self.vl_pub.publish(vl_msg)
        self.vr_pub.publish(vr_msg)

def main(args=None):

    rclpy.init(args=args)

    node = VelController()

    rclpy.spin(node)

    # Destroy node

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
