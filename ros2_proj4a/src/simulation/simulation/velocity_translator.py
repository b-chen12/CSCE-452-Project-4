import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from launch_ros.actions import Node
from rclpy.node import Node

# Subscribe to /cmd_vel
# DataType: geometry_msgs/msg/Twist

# Publish to /vr and /vl
# DataType: std_msgs/msg/Float64

# Goal: Translate linear.x and angular.z into
# appropriate wheel velocities

class VelocityTranslator(Node):
    def __init__(self):
        super.__init__('VelocityTranslator')

