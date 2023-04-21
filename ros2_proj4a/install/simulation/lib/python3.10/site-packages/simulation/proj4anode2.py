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
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from simulation.disc_robot import load_disc_robot
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from tf2_ros import TransformBroadcaster, TransformStamped

from sensor_msgs.msg import LaserScan
from example_interfaces.msg import Int64
from rclpy.node import Node
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import Float32MultiArray
from rclpy.qos import QoSProfile
from geometry_msgs.msg import PointStamped
from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped, Buffer, TransformListener
import tf2_ros
import numpy as np
from std_msgs.msg import Float64
from time import time
from tf2_geometry_msgs import do_transform_point


class RobotSimulator2(Node):
    # This node publishes onto the three separate topics
    def __init__(self):
        self.d = 3

    def createScan(self):
        rate = 1
        count = 25
        angle_min = -1.8
        angle_max = 1.8
        range_min = 0.01
        range_max = 40.0

        if angle_min < angle_max:
            total_angle = angle_max - angle_min
        else:
            total_angle = 2 * math.pi - angle_min + angle_max

        # Calculate the angle increment needed to get num_angles evenly spaced angles
        angle_increment = total_angle / count
        self.get_logger().info('Message from /vl: "%s"' % angle_increment )

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser'
        msg.angle_min = angle_min
        msg.angle_max = angle_max
        msg.angle_increment = angle_increment
        msg.time_increment = 0.0
        msg.scan_time = 1.0
        msg.range_min = range_min
        msg.range_max = range_max
        tangentLine = 0.1
        current_ang = angle_min
        msg.ranges=[]
        t = self.tf_buffer.lookup_transform('world', msg.header.frame_id,rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=7.0))
        for i in range(count):
            tangentLine = 0.01
            while(tangentLine < range_max):
                point = PointStamped()
                point.header.frame_id = msg.header.frame_id

                temp_x = tangentLine * (cos(current_ang))
                temp_y = tangentLine * (sin(current_ang))
                point.point.x=temp_x
                point.point.y=temp_y
                point.point.z = 0.0
                point_in_world_frame = do_transform_point(point, t)
                # Transform the PointStamped message into the world frame
               
                
                index_x = int((point_in_world_frame.point.x ) / 0.15)
                index_y = int((point_in_world_frame.point.y ) / 0.15)
                if(self.temp2[index_y][index_x] == '#'):
                    msg.ranges.append(tangentLine)
                    # self.get_logger().info('X: "%s"' % temp_x)
                    # self.get_logger().info('Y: "%s"' % temp_y)
                    break
                tangentLine+=0.01
            current_ang+=angle_increment

       # msg.ranges = [1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0]  # Example range data
        self.laser_pub.publish(msg)
        #self.get_logger().info("%s" % msg)

        
    def createMap(self):
        urdf_file_name = 'pillars.world'
        urdf = os.path.join(
            get_package_share_directory('simulation'),
            urdf_file_name)
        mapinfo = load_map(urdf)
        occupancygrid_msg = OccupancyGrid()
        occupancygrid_msg.header = Header()
        occupancygrid_msg.header.frame_id = 'world'
        occupancygrid_msg.info.origin.position.x = 0.0
        occupancygrid_msg.info.origin.position.y = 0.0
        occupancygrid_msg.info.origin.position.z = 0.0
        occupancygrid_msg.info.origin.orientation.x = 0.0
        occupancygrid_msg.info.origin.orientation.y = 0.0
        occupancygrid_msg.info.origin.orientation.z = 0.0
        occupancygrid_msg.info.origin.orientation.w = 1.0
        occupancygrid_msg.info.resolution = mapinfo['resolution']
        
        x = 0
        occupancygrid_msg.data = []
        self.temp = mapinfo['map'].split('\n')
      #  self.get_logger().info("%s" % self.temp)
        occupancygrid_msg.info.width = len(self.temp[0])
        occupancygrid_msg.info.height = len(self.temp)
        self.temp2 = []

        for i in range(len(self.temp)):
            row=[]
            for j in range(len(self.temp[0])):
                if(self.temp[len(self.temp)-1-i][j] == '#'):
                    occupancygrid_msg.data.append(1)
                    row.append('#')
                elif(self.temp[len(self.temp)-1-i][j] == '.'):
                    occupancygrid_msg.data.append(0)
                    row.append('.')
            self.temp2.append(row)
       # self.get_logger().info("%s" % self.temp2)
        # for i in range(len(mapinfo['map'])):
        #     if(mapinfo['map'][i] == '\n'):
        #         occupancygrid_msg.info.height+=1
                
        #         x = 0
        #     else:
        #         if(mapinfo['map'][i] == '#'):
        #             #temp.append(1)
        #             occupancygrid_msg.data.insert(0,1)
        #         elif(mapinfo['map'][i] == '.'):
        #             #temp.append(0)
        #             occupancygrid_msg.data.insert(0,0)
        #         x+=1
        # occupancygrid_msg.info.width = x
        return occupancygrid_msg
        

def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main(args=None):

    rclpy.init(args=args)

    node2 = RobotSimulator2()

    rclpy.spin(node2)

    # Destroy node

    node2.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
