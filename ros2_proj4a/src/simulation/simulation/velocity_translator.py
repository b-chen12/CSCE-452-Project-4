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

        self.vel_sub = self.create_subscription(Twist, '/cmd_vel', self.listener_callback, 10)

        self.vl_pub = self.create_publisher(Float64, '/vl')
        self.vr_pub = self.create_publisher(Float64, '/vr')

        self.wheel_dist = None # distance between wheels, need to figure out how to get this

    def listener_callback(self, msg):
        self.get_logger().info('Linear X: "%s"' % msg.linear.x)
        self.get_logger().info('Angular Y: "%s"' % msg.angular.z)
        
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        vl = linear_x - (self.wheel_dist / 2) * angular_z # calculates vl
        vr = linear_x + (self.wheel_dist / 2) * angular_z # calculates vr

        # publishes out the calculated data
        self.vl_pub.publish(Float64(vl))
        self.vl_pub.publish(Float64(vr))

def main(args = None):
    rclpy.init(args = args)

    node = VelocityTranslator()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == main():
    main()