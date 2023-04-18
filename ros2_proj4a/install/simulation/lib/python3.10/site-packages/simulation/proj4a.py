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

from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
import numpy as np
from std_msgs.msg import Float64
from time import time


class RobotSimulator(Node):
    # This node publishes onto the three separate topics
    def __init__(self):
        super().__init__('RobotSimulator')
        self.vl = 0.0
        self.vr = 0.0
        self.x =  1.0
        self.y = 2.2
        self.theta = 1.45
        self.curTime = time()
        self.prevTime = None
        self.delta_t = 0.1
        self.map_pub = self.createMap()

        self.occupancy_grid = self.create_publisher(OccupancyGrid, '/map', 10)
        self.vlsub = self.create_subscription(Float64, '/vl', self.listener_callback_vl, 10)
        self.vrsub = self.create_subscription(Float64, '/vr', self.listener_callback_vr, 10)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_timer_callback)
        self.oneSecondTimer = self.create_timer(1.0, self.set_vels)

    def listener_callback_vl(self, msg):
        self.vl = msg.data
        self.get_logger().info('Message from /vl: "%s"' % self.vl )
    
    def listener_callback_vr(self, msg):
        self.vr = msg.data
        self.get_logger().info('Message from /vr: "%s"' % self.vr )
        
        self.oneSecondTimer.reset()

    def set_vels(self):
        self.vl = 0.0
        self.vr = 0.0

    def broadcast_timer_callback(self):
        self.curTime = time()
        if self.prevTime is not None:
            self.delta_t = self.curTime - self.prevTime
            #self.get_logger().info('Message from /scan: "%s"' % self.delta_t )
        self.prevTime = self.curTime
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'
        
        l = 0.3
        if(self.vr==self.vl):
            v = self.vr
            delta_x = v*self.delta_t
            x_change = delta_x * cos(self.theta)
            y_change = delta_x * sin(self.theta)
            t.transform.translation.x = x_change + self.x
            t.transform.translation.y = y_change + self.y
            t.transform.translation.z = 0.0

            t.transform.rotation = euler_to_quaternion(0,0,self.theta)

        else:
            R = ((l/2)*(self.vr+self.vl)/(self.vr-self.vl))
            # self.get_logger().info('Message from /R: "%s"' % R)
            c = [self.x-R*sin(self.theta), self.y + R*cos(self.theta)]
            w = (self.vr-self.vl)/l
            # self.get_logger().info('Message from /c: "%s"' % c)
            # self.get_logger().info('Message from /w: "%s"' % w)
            one = [[cos(w*self.delta_t), -1*sin(w*self.delta_t), 0],[sin(w*self.delta_t),cos(w*self.delta_t),0],[0,0,1]]
            two = [[self.x-c[0]],[self.y-c[1]],[self.theta]]
            three = [[c[0]],[c[1]],[w*self.delta_t]]
            newState2 = np.array(one) @ np.array(two)
            
            newState = np.array(newState2) + np.array(three)
            # self.get_logger().info('Message from /scan: "%s"' % newState )    
            t.transform.translation.x = newState[0][0]
            t.transform.translation.y = newState[1][0]
            t.transform.translation.z = 0.0

            t.transform.rotation = euler_to_quaternion(0,0,newState[2][0])
            #self.get_logger().info('Message from /VL: "%s"' % newState[2][0])
            self.theta=newState[2][0]
            # self.get_logger().info('Message from /VR: "%s"' % self.vr)
        self.x=t.transform.translation.x
        self.y=t.transform.translation.y
        

        
        self.occupancy_grid.publish(self.map_pub)
        self.tf_broadcaster.sendTransform(t)

        
    def createMap(self):
        urdf_file_name = 'brick.world'
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
        temp = mapinfo['map'].split('\n')
        self.get_logger().info("%s" % temp)
        occupancygrid_msg.info.width = len(temp[0])
        occupancygrid_msg.info.height = len(temp)
        for i in range(len(temp)):
            for j in range(len(temp[0])):
                if(temp[len(temp)-1-i][j] == '#'):
                    occupancygrid_msg.data.append(1)
                elif(temp[len(temp)-1-i][j] == '.'):
                    occupancygrid_msg.data.append(0)
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

    node = RobotSimulator()

    rclpy.spin(node)

    # Destroy node

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
