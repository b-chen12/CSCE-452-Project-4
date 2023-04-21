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


class RobotSimulator(Node):
    # This node publishes onto the three separate topics
    def __init__(self):
        super().__init__('RobotSimulator')
        urdf_file_name = 'bad.robot' # self.get_parameter('robot_name').value
        urdf = os.path.join(
            get_package_share_directory('simulation'),
            urdf_file_name)
        self.robot_info = load_disc_robot(urdf)

        urdf_file_name2 = 'ell.world' # self.get_parameter('map_name').value
        urdf2 = os.path.join(
            get_package_share_directory('simulation'),
            urdf_file_name2)
        self.mapinfo = load_map(urdf2)


        self.rate = self.robot_info['laser']['rate']
        self.count = self.robot_info['laser']['count']
        self.angle_min = self.robot_info['laser']['angle_min']
        self.angle_max = self.robot_info['laser']['angle_max']
        self.range_min = self.robot_info['laser']['range_min']
        self.range_max = self.robot_info['laser']['range_max']

        self.errorl = np.random.normal(loc=1.0, scale=np.sqrt(self.robot_info['wheels']['error_variance_left']))
        self.errorr = np.random.normal(loc=1.0, scale=np.sqrt(self.robot_info['wheels']['error_variance_right']))

        self.vl = 0.0
        self.temp=[]
        self.vr = 0.0
        self.x =  self.mapinfo['initial_pose'][0]
        self.y = self.mapinfo['initial_pose'][1]
        self.theta = self.mapinfo['initial_pose'][2]
        self.radius = self.robot_info['body']['radius']
        
        self.curTime = time()
        self.prevTime = None
        self.delta_t = 0.1
        self.map_pub = self.createMap()

        self.occupancy_grid = self.create_publisher(OccupancyGrid, '/map', 10)
        self.laser_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.vlsub = self.create_subscription(Float64, '/vl', self.listener_callback_vl, 10)
        self.vrsub = self.create_subscription(Float64, '/vr', self.listener_callback_vr, 10)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.001, self.broadcast_timer_callback)
        self.oneSecondTimer = self.create_timer(1.0, self.set_vels)
        self.scanTimer = self.create_timer(self.rate, self.createScan)
        self.errorTimer = self.create_timer(self.robot_info['wheels']['error_update_rate'], self.updateError)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def listener_callback_vl(self, msg):
        self.vl = msg.data * self.errorl
        self.get_logger().info('Message from /vl: "%s"' % self.vl )
    
    def listener_callback_vr(self, msg):
        self.vr = msg.data * self.errorr
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
        
        l = self.robot_info['wheels']['distance']
        if(self.vr==self.vl):
            v = self.vr
            delta_x = v*self.delta_t
            x_change = delta_x * cos(self.theta)
            y_change = delta_x * sin(self.theta)
            t.transform.translation.x = x_change + self.x
            t.transform.translation.y = y_change + self.y
            t.transform.translation.z = 0.0

            t.transform.rotation = self.euler_to_quaternion(0,0,self.theta)

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

            t.transform.rotation = self.euler_to_quaternion(0,0,newState[2][0])
            #self.get_logger().info('Message from /VL: "%s"' % newState[2][0])
            self.theta=newState[2][0]
            # self.get_logger().info('Message from /VR: "%s"' % self.vr)

        testWallX = self.radius * cos(self.theta) + t.transform.translation.x
        testWallY = self.radius * sin(self.theta) + t.transform.translation.y
        index_x = int((testWallX) / self.mapinfo['resolution'])
        index_y = int((testWallY) / self.mapinfo['resolution'])
        if(self.temp2[index_y][index_x] == '#'):
            self.vl = 0.0
            self.vr = 0.0
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
        else:
            self.x=t.transform.translation.x
            self.y=t.transform.translation.y
            
        self.occupancy_grid.publish(self.map_pub)
        self.tf_broadcaster.sendTransform(t)

    def createScan(self):
        if self.angle_min < self.angle_max:
            total_angle = self.angle_max - self.angle_min
        else:
            total_angle = 2 * math.pi - self.angle_min + self.angle_max

        # Calculate the angle increment needed to get num_angles evenly spaced angles
        angle_increment = total_angle / self.count
        # self.get_logger().info('Message from /vl: "%s"' % angle_increment )

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser'
        msg.angle_min = self.angle_min
        msg.angle_max = self.angle_max
        msg.angle_increment = angle_increment
        msg.time_increment = 0.0
        msg.scan_time = 1.0
        msg.range_min = self.range_min
        msg.range_max = self.range_max
        tangentLine = self.range_min
        current_ang = self.angle_min
        msg.ranges=[]
        t = self.tf_buffer.lookup_transform('world', msg.header.frame_id,rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=7.0))
        
        for i in range(self.count):
            tangentLine = self.range_min
            while(tangentLine < self.range_max):
                point = PointStamped()
                point.header.frame_id = msg.header.frame_id

                temp_x = tangentLine * (cos(current_ang))
                temp_y = tangentLine * (sin(current_ang))
                point.point.x=temp_x
                point.point.y=temp_y
                point.point.z = 0.0
                point_in_world_frame = do_transform_point(point, t)
                # Transform the PointStamped message into the world frame
               
                index_x = int((point_in_world_frame.point.x ) / self.mapinfo['resolution'])
                index_y = int((point_in_world_frame.point.y ) / self.mapinfo['resolution'])
                if(self.temp2[index_y][index_x] == '#'):
                    error = np.random.normal(loc=0.0, scale=np.sqrt(self.robot_info['laser']['error_variance']), size=1)
                    fail = np.random.rand()
                    if(fail < self.robot_info['laser']['fail_probability']):
                        msg.ranges.append(np.nan)
                    else:
                        msg.ranges.append(tangentLine+error)
                    break
                tangentLine+=0.05
            current_ang+=angle_increment


        self.laser_pub.publish(msg)
        #self.get_logger().info("%s" % msg)

        
    def createMap(self):
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
        occupancygrid_msg.info.resolution = self.mapinfo['resolution']
        
        occupancygrid_msg.data = []
        self.temp = self.mapinfo['map'].split('\n')
      #  self.get_logger().info("%s" % self.temp)
        occupancygrid_msg.info.width = len(self.temp[0])
        occupancygrid_msg.info.height = len(self.temp)-1
        self.temp2 = []

        for i in range(1, len(self.temp)):
            row=[]
            for j in range(len(self.temp[0])):
                if(self.temp[len(self.temp)-1-i][j] == '#'):
                    occupancygrid_msg.data.append(1)
                    row.append('#')
                elif(self.temp[len(self.temp)-1-i][j] == '.'):
                    occupancygrid_msg.data.append(0)
                    row.append('.')
            self.temp2.append(row)

        return occupancygrid_msg
        
    def updateError(self):
        self.errorl = np.random.normal(loc=1.0, scale=np.sqrt(self.robot_info['wheels']['error_variance_left']))
        self.errorr = np.random.normal(loc=1.0, scale=np.sqrt(self.robot_info['wheels']['error_variance_right']))
    
    def euler_to_quaternion(self, roll, pitch, yaw):
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
