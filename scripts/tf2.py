#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster 
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
import numpy as np
import json
import os


class DynamicFrameBroadcaster(Node):

    def __init__(self,r,l):
        super().__init__('dynamic_frame_tf2_broadcaster')

        self._subscription = self.create_subscription(JointState, '/joint_states', self.vel_listener,10)
        self._period = 0.1
        self._tf_broadcaster = TransformBroadcaster(self)
        self._timer = self.create_timer(self._period, self.broadcast_timer_callback)
        self._pub = self.create_publisher(Odometry,'/odom',10)
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._phi_r = 0.0
        self._phi_l = 0.0
        self._r = r
        self._l = l
        self.get_logger().info('TF2 initialized correctly!')


    def vel_listener(self, msg):
        self._phi_l, self._phi_r = msg.velocity 
                               
    def broadcast_timer_callback(self):
        t = TransformStamped()
        odom = Odometry()

        v = self._r*(self._phi_r + self._phi_l)/2
        omega = self._r*(self._phi_r - self._phi_l)/self._l

        self._x += v*np.cos(self._theta)*self._period
        self._y += v*np.sin(self._theta)*self._period
        self._theta += omega*self._period

        qx,qy,qz,qw = quaternion_from_euler(0,0,self._theta) 

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self._x
        t.transform.translation.y = self._y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(t)

        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega

        self._pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    script_dir = os.path.dirname(os.path.realpath(__file__))

    json_path = os.path.join(script_dir, 'variables.json')

    # Leer el archivo
    with open(json_path, 'r') as f:
        data = json.load(f)
    r = data['_r']
    l = data['_l']
    minimal_subscriber = DynamicFrameBroadcaster(r,l)

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()