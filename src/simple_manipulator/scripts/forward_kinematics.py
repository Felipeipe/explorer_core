#!/usr/bin/env python3

import rclpy
from tf_transformations import quaternion_from_matrix, translation_from_matrix
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Point, Quaternion
from sensor_msgs.msg import JointState

#import tf_transformations

import numpy as np


class FKPublisher(Node):

    def __init__(self):
        super().__init__('fk_publisher')

        # Publisher for the eef_pose according to forward kinematics
        self._fk_publisher = self.create_publisher(PoseStamped, 'eef_pose', 10)

        # Subscriber for joint states
        self._joint_state_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_states_callback,
            10)
        
        # Timer to publish the eef_pose according to forward kinematics at a fixed frequency
        timer_period = 0.02  # seconds
        self._fk_timer = self.create_timer(timer_period, self._timer_callback)

        # Auxiliary variables
        self._joint_states = None


    def _joint_states_callback(self, msg):
        # TODO: get the joint states position accessing the 'msg'
        self._joint_states = msg.position
        pass

    def _timer_callback(self):

        if self._joint_states is None:
            return 
        
        # Get the eef position (x,y,z) and orientation (rx, ry, rz, rw)

        eef_position, eef_quaternion = self._compute_fk(self._joint_states)
        
        # TODO: construct a message for a pose stamped and publish it
        eef_pose = PoseStamped()
        eef_pose.header.stamp = self.get_clock().now().to_msg()  # Tiempo actual
        eef_pose.header.frame_id = "base_link"
        eef_pose.pose.position = eef_position
        eef_pose.pose.orientation = eef_quaternion

        # TODO: your code goes here

        self._fk_publisher.publish(eef_pose)


    def _compute_fk(self, joint_states):
        cos = np.cos(joint_states)
        sin = np.sin(joint_states)
        # TODO: program a method that receives a vector of six variables (joint_states)
        # an by computing the forward kinematics of the manipulator, returns the end effector
        # position (x,y,z) and orientation as a quaternion (rx, ry, rz, rw)
        T01 = np.array([[ cos[0],-sin[0],       0,    0],
                        [ sin[0], cos[0],       0,    0],
                        [      0,      0,       1,  0.1],
                        [      0,      0,       0,    1]])
        
        T12 = np.array([[ cos[1],      0,  sin[1],    0],
                        [      0,      1,       0, 0.15],
                        [-sin[1],      0,  cos[1],  0.2],
                        [      0,      0,       0,    1]])
        
        T23 = np.array([[ cos[2],      0,  sin[2],    0],
                        [      0,      1,       0, -0.1],
                        [-sin[2],      0,  cos[2],  0.8],
                        [      0,      0,       0,    1]])
        
        T34 = np.array([[ cos[3],      0,  sin[3],    0],
                        [      0,      1,       0,  0.1],
                        [-sin[3],      0,  cos[3],  0.8],
                        [      0,      0,       0,    1]])
        
        T45 = np.array([[      1,      0,       0, -0.1],
                        [      0, cos[4], -sin[4],    0],
                        [      0, sin[4],  cos[4],  0.3],
                        [      0,      0,       0,    1]])
        
        T56 = np.array([[ cos[5],-sin[5],       0,    0],
                        [ sin[5], cos[5],       0,    0],
                        [      0,      0,       1,  0.2],
                        [      0,      0,       0,    1]])
        
        T6f = np.array([[      1,      0,       0,    0],
                        [      0,      1,       0,    0],
                        [      0,      0,       1,  0.1],
                        [      0,      0,       0,    1]])
        
        T0f = np.linalg.multi_dot([T01, T12, T23, T34, T45, T56, T6f])
        eef_position = translation_from_matrix(T0f)
        eef_quaternion = quaternion_from_matrix(T0f)
    
        position = Point()
        orientation = Quaternion()
        position.x, position.y, position.z = eef_position
        orientation.x, orientation.y, orientation.z, orientation.w = eef_quaternion

        return position, orientation



def main(args=None):
    rclpy.init(args=args)

    fk_publisher = FKPublisher()
    fk_publisher.get_logger().info("Started Forward Kinematics node successfully")
    rclpy.spin(fk_publisher)
    

    fk_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
