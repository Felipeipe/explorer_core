#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

#import tf_transformations

import numpy as np
from fk_solution import fk_solution


class IKPublisher(Node):

    def __init__(self):
        super().__init__('fk_publisher')

        # TODO: get the callable methods for the jacobian and forward kinematics for the position of 
        # the end effector

        self._position_jacobian, self._fk = fk_solution()

        # Publisher for joint states 
        self._ik_publisher = self.create_publisher(JointState, '/joint_states', 10)

        # Subscribers for joint states and external commands given by teleoperation
        self._joint_state_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_states_callback,
            10)

        self._cmd_vel_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self._cmd_vel_callback,
            10)
        
        # Timer to publish joint states according to inverse kinematics at a fixed frequency
        timer_period = 0.02  # seconds
        self._ik_timer = self.create_timer(timer_period, self._timer_callback)

        # Auxiliary variables that may be useful
        self._joint_states = np.zeros(6)
        self._delta_signs = np.zeros(3)
        self._delta_pos_gain = 0.009
        self._nb_iterations = 1


    def _cmd_vel_callback(self, msg):
        # TODO: get an array for the signs of the commands in the linear velocity part of the twist.
        # For instance: if twist.linear.x is -0.5, assign a -1 to the first element of self._delta_signs.
        # Make the resulting row vector a numpy array
        V = np.array([msg.linear.x, msg.linear.y, msg.linear.z])   
        self._delta_signs = np.sign(V)


    def _joint_states_callback(self, msg):
        # TODO: get the position of the joint state message assigned to self._joint_states
        self._joint_states = np.array(msg.position)


    def _timer_callback(self):

        if self._joint_states is None:
            return 
        
        # TODO: compute the forwatd kinematics for the eef position using the method you got via
        # lambdify using sympy
        current_eef_position = self._fk(*self._joint_states).squeeze()
        
        delta_pos = np.array([self._delta_pos_gain * self._delta_signs[0],
                               self._delta_pos_gain * self._delta_signs[1], 
                               self._delta_pos_gain * self._delta_signs[2]]).T
        
        # Get the target eef_position
        target_eef_position = current_eef_position + delta_pos
        # Get the ik result so as to compute the required joint states for that target position

        ik_result = self._compute_ik(target_eef_position)
        
        # TODO: construct a joint state message and publish it.
        # NOTE: the names for the joint state should be:
        # "shoulder_yaw_joint", "shoulder_pitch_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", and "wrist_3_joint"

        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ["shoulder_yaw_joint", "shoulder_pitch_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        joint_state.position = ik_result
        self._ik_publisher.publish(joint_state)


    def _compute_ik(self, eef_target_position):
        
        # TODO: program the newton raphson method using the target eef_position, and an initial guess
        # equal to the current joint state positions
        # NOTE: you may want to look up for the documentation on:
        #   - np.linal.pinv()
        # NOTE: here you should use the method obtained via lambdify to compute the jacobian for the eef_position
        # NOTE: be careful when multiplying matrices
        Theta_k = self._joint_states
        for _ in range(self._nb_iterations):
            eef_pos = self._fk(*Theta_k).squeeze()
            Jpinv = np.linalg.pinv(self._position_jacobian(*Theta_k))
            Theta_k = Theta_k + Jpinv@(eef_target_position - eef_pos)
        return Theta_k
         

def main(args=None):
    rclpy.init(args=args)
    
    ik_publisher = IKPublisher()
    ik_publisher.get_logger().info("Inverse kinematics node started successfully")
    rclpy.spin(ik_publisher)

    ik_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
