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
        file_name = 'normal.robot' 
        u = os.path.join(get_package_share_directory('simulation'),file_name)
        self.robot_info = load_disc_robot(u)
        self.wheel_distance = self.robot_info['wheels']['distance']
        self.v = 0.115 # default linear velocity, very slow so we do not run into walls
        self.tol = 0.065 # default tolerance for wall detection (end of hallway)
        # Subscribes to /scan of type LaserScan in order to make decisions
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.listener_callback, 10)

        # Publishes to /cmd_vel of type Twist to give robot it's new velocity
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def listener_callback(self, msg):
        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment
        cur_angle = angle_min # Current angle for iteration purposes
        laser_vectors = [] # Array that will store the vectors of each laser scan in the range
        # Loops through each range in the laser scan and obtains the x and y components of the vector; need to check for nan because of our error
        for i in range(len(ranges)):
            if(math.isnan(ranges[i]) is False):
                # the range gives us the magnitude of the vector (the radius in terms of trigonometric functions), so x = cos(theta) * r, y = sin(theta) * r
                cur_vector = (cos(cur_angle)*ranges[i], sin(cur_angle)*ranges[i])
                laser_vectors.append(cur_vector)
            cur_angle += angle_increment
        average_vector = np.mean(laser_vectors, axis=0) # Get the average x and y components of the average vector; axis = 0 makes it so the average is computed for each index, so all index 0's (aka x coordinates) will be averaged and all index 1's will be averaged
        magnitude = average_vector[1]**2 + average_vector[0]**2 # How far the average vector extends

        # To get the angle of the vector: theta = arctan(y / x); also known as the direction we want to move
        d_angle = np.arctan2(average_vector[1], average_vector[0])

        # To get the radius of the circle the robot will turn; constant allows for tighter turns
        R = self.wheel_distance / d_angle / 1.5

        # Angular velocity = linear velocity / radius of circle (w = v/r)
        angular_v = self.v / R
        
        t_msg = Twist()
        t_msg.linear.x = self.v
        t_msg.angular.z = angular_v

        # Checks if the robot is reaching an area where it must turn arround because it is a dead end
        if(magnitude <= self.tol):
            t_msg.linear.x = 0.0 # Does not move forward while it is turning around
            t_msg.angular.z = 3.0 # High angular velocity lets it turn in place so that it will face the other direction
        
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
