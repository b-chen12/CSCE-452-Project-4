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


class NavController(Node):
    # This node publishes onto the three separate topics
    def __init__(self):
        super().__init__('NavController')
        
        # File name to get robot info and then store info in self.robot_info
        file_name = 'normal.robot' # self.get_parameter('robot_name').value
        u = os.path.join(get_package_share_directory('simulation'),file_name)
        self.robot_info = load_disc_robot(u)
        self.wheel_distance = self.robot_info['wheels']['distance']
        self.v = .1

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.listener_callback, 10)

        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def listener_callback(self, msg):
        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment
        cur_angle = angle_min
        vs = []
        for i in range(len(ranges)):
            if(math.isnan(ranges[i]) is False):
               
                cur_vector = [cos(cur_angle)*ranges[i], sin(cur_angle)*ranges[i]]
                vs.append(cur_vector)
            cur_angle += angle_increment
        average_vector = np.mean(vs, axis=0)
        d = np.arctan2(average_vector[1], average_vector[0])

        R = self.wheel_distance / d / 2
        angular_v = self.v / R
        
        t_msg = Twist()
        t_msg.linear.x = self.v
        t_msg.angular.z = angular_v
        self.vel_pub.publish(t_msg)

def main(args=None):

    rclpy.init(args=args)

    node = NavController()

    rclpy.spin(node)

    # Destroy node

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
