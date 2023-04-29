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
        
        # File name to get robot info and then store info in self.robot_info
        file_name = 'normal.robot' # self.get_parameter('robot_name').value
        u = os.path.join(get_package_share_directory('simulation'),file_name)
        self.robot_info = load_disc_robot(u)

        # File name to get world info and then store info in self.map_info
        file_name2 = 'example.world' # self.get_parameter('map_name').value
        u2 = os.path.join(get_package_share_directory('simulation'),file_name2)
        self.map_info = load_map(u2)


        # Store all necessary information for lidar scan
        self.rate = self.robot_info['laser']['rate']
        self.count = self.robot_info['laser']['count']
        self.angle_min = self.robot_info['laser']['angle_min']
        self.angle_max = self.robot_info['laser']['angle_max']
        self.range_min = self.robot_info['laser']['range_min']
        self.range_max = self.robot_info['laser']['range_max']

        # Get error for left and right wheel velocity
        self.errorl = np.random.normal(loc=1.0, scale=np.sqrt(self.robot_info['wheels']['error_variance_left']))
        self.errorr = np.random.normal(loc=1.0, scale=np.sqrt(self.robot_info['wheels']['error_variance_right']))

        # Store various robot starting information
        self.vl = 0.0
        self.vr = 0.0
        self.x =  self.map_info['initial_pose'][0]
        self.y = self.map_info['initial_pose'][1]
        self.theta = self.map_info['initial_pose'][2]
        self.radius = self.robot_info['body']['radius']
        self.temp=[]
        
        self.curTime = time()
        self.prevTime = None
        self.delta_t = 0.1 # Time between movements
        self.map_pub = self.createMap() # store the information to publish map so we don't have to keep iterating through the information

        # Publish occupancy grid of the map and laser scan; Subscribe to left and right wheel velocities
        self.occupancy_grid = self.create_publisher(OccupancyGrid, '/map', 10)
        self.laser_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.vlsub = self.create_subscription(Float64, '/vl', self.listener_callback_vl, 10)
        self.vrsub = self.create_subscription(Float64, '/vr', self.listener_callback_vr, 10)

        # Set up tf2 broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.001, self.broadcast_timer_callback) # Broadcast every 0.001 seconds
        self.oneSecondTimer = self.create_timer(1.0, self.set_vels) # Timer that sets velocity to 0 if it gets called; is reset everytime new velocity is brough received
        self.scanTimer = self.create_timer(self.rate, self.createScan) # Timer for how often we scan using the rate
        self.errorTimer = self.create_timer(self.robot_info['wheels']['error_update_rate'], self.updateError) # Timer for how often we update the error value for the wheels

        # Set up tf2 buffer and transform listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    # Get new left wheel velocity, apply appropriate error
    def listener_callback_vl(self, msg):
        self.vl = msg.data * self.errorl
        # self.get_logger().info('Message from /vl: "%s"' % self.vl )
    
    # Get new right wheel velocity, apply appropriate error
    def listener_callback_vr(self, msg):
        self.vr = msg.data * self.errorr
        # self.get_logger().info('Message from /vr: "%s"' % self.vr )
        
        self.oneSecondTimer.reset() # New velocity obtained, reset timer to stop robot

    # Stop robot if going one second without new velocity
    def set_vels(self):
        self.vl = 0.0
        self.vr = 0.0

    # Broadcast to convert between world and base_link frames
    def broadcast_timer_callback(self):
        self.curTime = time()
        if self.prevTime is not None:
            self.delta_t = self.curTime - self.prevTime # Get time between movements
        self.prevTime = self.curTime
        
        # Set frame information
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'
        
        l = self.robot_info['wheels']['distance'] # distance between wheels
        # Checks if velocities are same, goes in a straight line, using V = Change in distance / Change in time then getting the x and y coordinates for the new position
        if(self.vr==self.vl):
            v = self.vr
            delta_x = v*self.delta_t
            x_change = delta_x * cos(self.theta)
            y_change = delta_x * sin(self.theta)
            t.transform.translation.x = x_change + self.x
            t.transform.translation.y = y_change + self.y
            t.transform.translation.z = 0.0

            t.transform.rotation = self.getQuaternion(0,0,self.theta)

        # Not in a straight line, need Differential drive state transitions
        else:
            R = ((l/2)*(self.vr+self.vl)/(self.vr-self.vl)) # Distance from robot center to ICC
            c = [self.x-R*sin(self.theta), self.y + R*cos(self.theta)] # ICC location
            w = (self.vr-self.vl)/l # angular vel
            one = [[cos(w*self.delta_t), -1*sin(w*self.delta_t), 0],[sin(w*self.delta_t),cos(w*self.delta_t),0],[0,0,1]] # First part of updating state (Using New State equation in Lecture 3 notes)
            two = [[self.x-c[0]],[self.y-c[1]],[self.theta]] # Second part
            three = [[c[0]],[c[1]],[w*self.delta_t]] # Third part
            newState2 = np.array(one) @ np.array(two)
            newState = np.array(newState2) + np.array(three)  
            t.transform.translation.x = newState[0][0]
            t.transform.translation.y = newState[1][0]
            t.transform.translation.z = 0.0

            t.transform.rotation = self.getQuaternion(0,0,newState[2][0])
            self.theta=newState[2][0]

        angles = [0,0.785398,1.5708,2.35619,3.14159,3.92699,4.71239,5.49779]
        found = False
        for a in angles:
            test_x = self.radius * cos(a) + t.transform.translation.x
            test_y = self.radius * sin(a) + t.transform.translation.y
            index_x = int((test_x) / self.map_info['resolution'])
            index_y = int((test_y) / self.map_info['resolution'])
            if(self.temp2[index_y][index_x] == '#'):
                self.vl = 0.0
                self.vr = 0.0
                t.transform.translation.x = self.x
                t.transform.translation.y = self.y
                found = True
        
        if found == False:
            # Check if the robot is about to run into a wall
            testWallX = self.radius * cos(self.theta) + t.transform.translation.x
            testWallY = self.radius * sin(self.theta) + t.transform.translation.y
            index_x = int((testWallX) / self.map_info['resolution'])
            index_y = int((testWallY) / self.map_info['resolution'])
            # Stop if it is about to run into a wall, otherwise, move forward using new state
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

    # Function to get the laser scan data of the robot
    def createScan(self):
        # Get angle_increment so that they are evenly spaced
        if self.angle_min < self.angle_max:
            total_angle = self.angle_max - self.angle_min
        angle_increment = total_angle / self.count

        # Set up laser scan info that was read from file earlier
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
        magnitude = self.range_min
        current_ang = self.angle_min
        msg.ranges=[]
        # Get last transform so that we can convert into world frame
        laser_t = self.tf_buffer.lookup_transform('world', msg.header.frame_id,rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=7.0))
        
        # For count amount of times, find the magnitude of the range; find range by slowly incrementing distance until reaches a wall
        for i in range(self.count):
            magnitude = self.range_min
            while(magnitude < self.range_max):
                point = PointStamped()
                point.header.frame_id = msg.header.frame_id
                temp_x = magnitude * (cos(current_ang))
                temp_y = magnitude * (sin(current_ang))
                point.point.x=temp_x
                point.point.y=temp_y
                point.point.z = 0.0
                point_in_world_frame = do_transform_point(point, laser_t)
                # Transform the PointStamped message into the world frame
               
                # Convert from map to index of positions that we got when reading in the map
                index_x = int((point_in_world_frame.point.x ) / self.map_info['resolution'])
                index_y = int((point_in_world_frame.point.y ) / self.map_info['resolution'])
                if(self.temp2[index_y][index_x] == '#'):
                    # Add appropriate error
                    error = np.random.normal(loc=0.0, scale=np.sqrt(self.robot_info['laser']['error_variance']), size=1)
                    fail = np.random.rand()

                    # Check if reading failed based on error
                    if(fail < self.robot_info['laser']['fail_probability']):
                        msg.ranges.append(np.nan)
                    else:
                        msg.ranges.append(magnitude+error)
                    break
                magnitude+=0.1
            current_ang+=angle_increment


        self.laser_pub.publish(msg)
        
    def createMap(self):
        # Set various occupancy grid attributes
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
        occupancygrid_msg.info.resolution = self.map_info['resolution']
        
        occupancygrid_msg.data = []
        self.temp = self.map_info['map'].split('\n') # Split based on newline
        occupancygrid_msg.info.width = len(self.temp[0])
        occupancygrid_msg.info.height = len(self.temp)-1
        self.temp2 = []

        # Go through and add values from the string into the list we made so we can have a representaiton of the map
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
        
    # Update error when timer goes off for the wheels
    def updateError(self):
        self.errorl = np.random.normal(loc=1.0, scale=np.sqrt(self.robot_info['wheels']['error_variance_left']))
        self.errorr = np.random.normal(loc=1.0, scale=np.sqrt(self.robot_info['wheels']['error_variance_right']))
    
    # Need to convert from euler coordinates to quaternion so that we can transform correctly
    def getQuaternion(self, x, y, z):
        qz = cos(x/2) * cos(y/2) * sin(z/2) - sin(x/2) * sin(y/2) * cos(z/2)
        qw = cos(x/2) * cos(y/2) * cos(z/2) + sin(x/2) * sin(y/2) * sin(z/2)
        return Quaternion(x=0.0, y=0.0, z=qz, w=qw)



def main(args=None):

    rclpy.init(args=args)

    node = RobotSimulator()

    rclpy.spin(node)

    # Destroy node

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
