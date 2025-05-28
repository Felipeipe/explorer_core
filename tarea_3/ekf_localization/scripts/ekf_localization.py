#!/usr/bin/env python
import rclpy
import numpy as np
from rclpy.node import Node
import math

import rosidl_parser
import threading
import std_msgs.msg
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from scipy.signal import convolve

import numpy as np

def conv_circ( signal, ker ):
    '''
        signal: real 1D array
        ker: real 1D array
        signal and ker must have same shape
    '''
    return np.real(np.fft.ifft( np.fft.fft(signal)*np.fft.fft(ker) ))

class EkfLocalization(Node):

    def __init__(self):
        super().__init__('ekf_localization')
        self.laser_subscriber_ = self.create_subscription(LaserScan, 'scan', self.laser_callback_, 10)
        self.odom_subscriber_ = self.create_subscription(Odometry, 'odom', self.odom_callback_, 10)
        self.marker_publisher_ = self.create_publisher(Marker, 'marker', 10)
        
        
    def odom_callback_(self, msg: Odometry):
        # use odometry to do the prediction step of the ekf
        # YOUR CODE HERE
        pass

    def laser_callback_(self, msg: LaserScan):

        marker_points = Marker()
        marker_points.header = msg.header
        marker_points.ns = 'detected_points'
        marker_points.id = 0
        marker_points.type = marker_points.SPHERE_LIST
        marker_points.action = marker_points.ADD
        marker_points.pose.position.x = 0.0
        marker_points.pose.position.y = 0.0
        marker_points.pose.position.z = 0.0
        marker_points.pose.orientation.x = 0.0
        marker_points.pose.orientation.y = 0.0
        marker_points.pose.orientation.z = 0.0
        marker_points.pose.orientation.w = 1.0
        marker_points.scale.x = 0.1
        marker_points.scale.y = 0.1
        marker_points.scale.z = 0.1
        marker_points.color.a = 1.0
        marker_points.color.r = 1.0
        marker_points.color.g = 0.0
        marker_points.color.b = 0.0

        
        # detect corners with infinity 
        for i in range(len(msg.ranges)):
            if math.isinf( msg.ranges[i] ) and not math.isinf( msg.ranges[(i+1)%len(msg.ranges)] )  and  msg.ranges[(i+1)%len(msg.ranges)] > 0.0 and msg.ranges[(i+1)%len(msg.ranges)]< msg.range_max*0.9 :
                point = Point()
                point.x = msg.ranges[(i+1)%len(msg.ranges)]*np.cos(msg.angle_min + msg.angle_increment*(i+1)%len(msg.ranges))
                point.y = msg.ranges[(i+1)%len(msg.ranges)]*np.sin(msg.angle_min + msg.angle_increment*(i+1)%len(msg.ranges))
                point.z = 0.0
                marker_points.points.append(point)
            elif not math.isinf( msg.ranges[i] ) and msg.ranges[i] > 0.0  and msg.ranges[i] <msg.range_max*0.9 and math.isinf( msg.ranges[(i+1)%len(msg.ranges)] ):
                point = Point()
                point.x = msg.ranges[i]*np.cos(msg.angle_min + msg.angle_increment*i)
                point.y = msg.ranges[i]*np.sin(msg.angle_min + msg.angle_increment*i)
                point.z = 0.0
                marker_points.points.append(point)

        # detect corners
        # kernel = np.pad(np.array([1,1,1,1,1,-10,1,1,1,1,1]), (0, len(msg.ranges)-11), 'constant', constant_values=(0.0,))
        # diffrange  = conv_circ(msg.ranges, kernel)
        kernel = np.array([1.0,1.0,1.0,-6.0,1.0,1.0,1.0])
        diffrange = convolve(msg.ranges, kernel, mode='valid')
        curvature = diffrange * diffrange / (np.array(msg.ranges[3:-3]) **2 )
        print(curvature)

        # sort by curvature
        sorted_indices = np.argsort(curvature)[::-1]

        print(curvature[sorted_indices])
        # get the first 10 points

        curvature_num_points = 0

        indices = []
        for index in sorted_indices:
            if curvature_num_points > 3:
                break
            if  math.isfinite(curvature[index] ) and math.isfinite(msg.ranges[(index+3)%len(msg.ranges)] ):
                is_close = False
                for idx in indices:
                    if abs(index - idx) < 6:
                        is_close = True
                        break
                if not is_close:
                    indices.append(index)
                    point = Point()
                    point.x = msg.ranges[(index+3)%len(msg.ranges)]*np.cos(msg.angle_min + msg.angle_increment*((index+3)%len(msg.ranges)))
                    point.y = msg.ranges[(index+3)%len(msg.ranges)]*np.sin(msg.angle_min + msg.angle_increment*((index+3)%len(msg.ranges)))
                    point.z = 0.
                    marker_points.points.append(point)
                    curvature_num_points += 1

        self.marker_publisher_.publish(marker_points)

        # Aqui use los puntos detectados ( en marker_points.points ) para realizar la correccion usando EKF

        # YOUR CODE HERE










def main():
    rclpy.init()
    node = EkfLocalization()
    # node.run()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()

