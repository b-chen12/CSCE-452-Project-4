import rclpy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from launch_ros.actions import Node
from rclpy.node import Node

# Subscribe to: /scan
# DataType: sensor_msgs/msg/LaserScan

# Publishing to: /cmd_vel
# DataType: geometry_msgs/msg/Twist

class NavigationController(Node):
    def __init__(self):
        super.__init__(self)

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.listener_callback, 10)

        self.vel_pub = self.create_publisher(Twist, '/cmd_vel')
    
    def listener_callback(self, msg):
        self.get_logger().info('Laser Scan: "%s"' % msg.ranges)

        # Do stuff here based on the laser scan ranges array

def main(args = None):
    rclpy.init(args = args)

    node = NavigationController()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == main():
    main()